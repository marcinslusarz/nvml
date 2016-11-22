/*
 * Copyright 2016, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Gosh, this path resolving was pain to write.
 * TODO: clean up this whole file, and add some more explanations.
 */

#include <assert.h>
#include <stdbool.h>
#include <fcntl.h>
#include <stdlib.h>
#include <inttypes.h>
#include <syscall.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>

#include "libcintercept_hook_point.h"
#include "libpmemfile-core.h"

#include "preload.h"

static void path_setup_prefix(struct path_component *dst, const char *path,
			struct pool_description **in_pool);
static void path_initial_copy(struct path_component *dst, const char *src);
static size_t resolve_root_parent(struct path_component *result,
				struct pool_description **in_pool);

void
resolve_path(struct pool_description *in_pool, const char *path,
			struct path_component *result,
			enum resolve_last_or_not follow_last)
{
	size_t resolved; // how many chars are resolved already?
	result->error_code = 0;
	result->pool = NULL;

	if (path == NULL || path[0] == '\0') {
		result->error_code = -ENOTDIR;
		return;
	}

	if (path[0] == '\0') {
		result->error_code = -EIO;
		return;
	}

	if (strcmp(path, "/dev/shm/dummy_mount_point/a") == 0) {
		result->error_code = 0;
	}

	/* If it is a relative path, prefix it with the CWD */
	path_setup_prefix(result, path, &in_pool);
	if (result->error_code != 0)
		return;

	resolved = result->path_len;

	/* Make a copy of the whole path. */
	path_initial_copy(result, path);
	if (result->error_code != 0)
		return;

	/* Loop over the path component strings */
	while (resolved < result->path_len) {
		assert(resolved > 0);

		struct stat stat_buf;
		char *end;
		char end_backup;
		long error_code;
		char *r = result->path + resolved;

		if (r[0] == '.' && r[1] == '.' && r[2] == '\0') {
			end = r + 2;
			if (resolved == 1) {
				/* "/../a/b/c/d" -> "/a/b/c/d" */
				resolved =
				    resolve_root_parent(result, &in_pool);
				if (result->error_code != 0)
					return;

			} else {
				assert(resolved > 1);

				/* "/a/b/../c/d" -> "/a/c/d" */
				char *c = r - 2;
				size_t copy_len = (size_t)(
				    result->path + result->path_len - end);

				while (*c != '/' && c > result->path)
					--c;

				memmove(c, end, copy_len);
				result->path_len -= (size_t)(end - c);
				resolved -= (size_t)(end - c);
				end = c;
			}
			continue;
		}

		/* Look for the end of current component */
		for (end = result->path + resolved;
		    *end != '/' && *end != '\0';
		    ++end)
			;
		/*
		 * At this point r points to the start of the current
		 * component, end points to the '/' or '\0' following
		 * the current component.
		 */

		bool is_last_component = (*end == '\0' || end[1] == '\0');

		if (is_last_component && follow_last == no_resolve_last_slink) {
			if (in_pool == NULL) {
				in_pool = lookup_pd_by_path(result->path);
				if (in_pool != NULL) {
					result->path[0] = '/';
					result->path[1] = '\0';
					result->path_len = 1;
				}
			}
			result->pool = in_pool;
			return;
		}

		end_backup = *end;
		*end = '\0';

		if (in_pool != NULL) {
			errno = 0;
			// todo:
			// pmemfile_lstat(in_pool->pool,
			// result->path, &stat_buf);
			stat_buf.st_mode = 0;
			error_code = -errno;
		} else {
			error_code = syscall_no_intercept(SYS_lstat,
						result->path, &stat_buf);
		}

		*end = end_backup;

		assert(error_code <= 0);
		if (error_code < 0) {
			result->error_code = error_code;
			return;
		}

		if (S_ISLNK(stat_buf.st_mode)) {
			char link_buf[0x400];
			long l;

			*end = '\0';
			if (in_pool != NULL) {
				l = pmemfile_readlink(in_pool->pool,
						result->path,
						link_buf, sizeof(link_buf));
			} else {
				l = syscall_no_intercept(SYS_readlink,
						result->path,
						link_buf, sizeof(link_buf));
			}
			*end = end_backup;

			assert(l != 0);

			if (l < 0) {
				result->error_code = l;
				return;
			}

			if (link_buf[0] == '/') {
				/*
				 * If the symlink points to an absolute path
				 * then replace the beginning of the resolved
				 * string with the symlink's value.
				 *
				 * E.g.
				 * if "/usr/share" is a symlink to "/mnt/a/b"
				 *
				 * "/usr/share/some" -> "/mnt/a/b/some"
				 */
				size_t prefix_len_before =
				    (size_t)(end - result->path);
				size_t prefix_len_after = (size_t)l;
				size_t postfix_len = (size_t)(
				    result->path_len - prefix_len_before);

				if (prefix_len_after + postfix_len >=
				    sizeof(result->path) - 2) {
					result->error_code = -EIO;
					return;
				}
				memmove(result->path + prefix_len_after, end,
				    postfix_len);
				memcpy(result->path, link_buf,
				    prefix_len_after);

				result->path_len +=
				    prefix_len_after - prefix_len_before;
				result->path[result->path_len] = '\0';
				r += prefix_len_after - prefix_len_before;
			} else {
				/*
				 * If the symlink points to a relative path
				 * then replace the current component in the
				 * resolved string with the symlink's value.
				 *
				 * E.g. if "/usr/share" is a symlink to "a/b"
				 *
				 * "/usr/share/some" -> "/usr/a/b/some"
				 */
				size_t link_len = (size_t)l;
				size_t postfix_len = (size_t)(result->path_len -
				    (size_t)(end - result->path));
				size_t prefix_len = resolved;
				if (prefix_len + link_len + postfix_len >=
				    sizeof(result->path)) {
					result->error_code = -EIO;
					return;
				}
				memmove(result->path + prefix_len + link_len,
				    end, postfix_len + 1);
				memcpy(r, link_buf, link_len);
				result->path_len =
				    prefix_len + link_len + postfix_len;
			}
		} else if (!S_ISDIR(stat_buf.st_mode)) {
			if (!is_last_component)
				result->error_code = -ENOTDIR;
			else
				result->pool = in_pool;
			return;
		} else {
			if (in_pool == NULL) {
				struct pool_description *pool_mounted =
				    lookup_pd_by_inode(stat_buf.st_ino);

				if (pool_mounted != NULL) {
					in_pool = pool_mounted;

					size_t new_len = (size_t)((
					    result->path + result->path_len)
					    - end);

					if (new_len == 0) {
						result->path[0] = '/';
						result->path[1] = '\0';
						result->pool = in_pool;
						return;
					}

					memmove(result->path, end, new_len);
					result->path_len = new_len;
					resolved = 1;
					result->path[result->path_len] = '\0';
					assert(result->path[0] == '/');
					continue;
				}
			}
			resolved = (size_t)(end - result->path);
			if (end_backup == '/')
				++resolved;
		}
	}
	result->pool = in_pool;
}

static void
path_setup_prefix(struct path_component *dst, const char *path,
			struct pool_description **in_pool)
{

	if (path[0] != '/') {
		if (*in_pool != NULL) {
			char *r = pmemfile_getcwd((*in_pool)->pool,
					dst->path, sizeof(dst->path));

			if (r == NULL) {
				dst->error_code = -errno;
				return;
			}
		} else {
			long r = syscall_no_intercept(SYS_getcwd,
					dst->path, sizeof(dst->path));
			if (r < 0) {
				dst->error_code = r;
				return;
			}
		}
		dst->path_len = strlen(dst->path);
	} else {
		dst->path_len = 0;
		*in_pool = NULL;
	}

	dst->path[dst->path_len++] = '/';
	dst->path[dst->path_len] = '\0';
}

static void
path_initial_copy(struct path_component *dst, const char *src)
{
	assert(dst->path_len >= 1);
	assert(dst->path[0] == '/');
	assert(dst->path[dst->path_len - 1] == '/');
	assert(src[0] != '\0');

	const char *p = src;

	while (*p == '/')
		++p;

	while (*p != '\0') {
		/* Collapse slash clusters + skip self references in path */
		if (p[0] == '/' && (p[1] == '/' || p[1] == '\0')) {
			++p;
		} else if (p[0] == '.' && p[1] == '/') {
			p += 2;
		} else if (p[0] == '.' && p[1] == '\0') {
			break;
		} else if (p[0] == '/') {
			dst->path[dst->path_len++] = '/';
			++p;
		} else {
			assert(*p != '/');
			do {
				dst->path[dst->path_len++] = *p++;
				if (dst->path_len >= sizeof(dst->path) - 2) {
					dst->error_code = -EIO;
					return;
				}
			} while (*p != '/' && *p != '\0');
		}
	}
	if (dst->path_len > 0) {
		if (dst->path[dst->path_len - 1] == '/')
			--dst->path_len;
	}

	dst->path[dst->path_len] = '\0';
}

static size_t
resolve_root_parent(struct path_component *result,
			struct pool_description **in_pool)
{
	if (*in_pool == NULL) {
		/*
		 * Parent directory of the root directory
		 *
		 * "/../a/b/c" -> "/a/b/c"
		 */
		result->path_len -= 3;
		memmove(result->path, result->path + 3, result->path_len + 1);
		return 1;
	} else {
		/*
		 * Parent directory of the pmemfile mount point.
		 * E.g.: if the mount point is at "/mnt/point_parent/point"
		 * then the beginning of the string "/.." must be replaced
		 * with "/mnt/point_parent/point/..", thus:
		 *
		 * "/../a/b/c" -> "/mnt/point_parent/a/b/c"
		 */
		size_t mnt_len = (*in_pool)->len_mount_point_parent;
		size_t new_len = result->path_len + mnt_len - 3;
		if (new_len > sizeof(result->path)) {
			result->error_code = -EIO;
			return 0;
		}
		memmove(result->path + mnt_len,
		    result->path + 3, result->path_len - 3 + 1);
		memmove(result->path, (*in_pool)->mount_point_parent, mnt_len);
		result->path_len = new_len;
		*in_pool = NULL;
		return mnt_len;
	}
}
