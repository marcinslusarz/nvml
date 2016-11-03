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
 * file.c -- basic file operations
 */

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "callbacks.h"
#include "data.h"
#include "dir.h"
#include "file.h"
#include "inode.h"
#include "inode_array.h"
#include "internal.h"
#include "locks.h"
#include "out.h"
#include "pool.h"
#include "sys_util.h"
#include "util.h"

/*
 * file_check_flags -- (internal) open(2) flags tester
 */
static int
file_check_flags(int flags)
{
	if (flags & O_APPEND) {
		LOG(LSUP, "O_APPEND");
		flags &= ~O_APPEND;
	}

	if (flags & O_ASYNC) {
		LOG(LSUP, "O_ASYNC is not supported");
		errno = ENOTSUP;
		return -1;
	}

	if (flags & O_CREAT) {
		LOG(LTRC, "O_CREAT");
		flags &= ~O_CREAT;
	}

	// XXX: move to interposing layer
	if (flags & O_CLOEXEC) {
		LOG(LINF, "O_CLOEXEC is always enabled");
		flags &= ~O_CLOEXEC;
	}

	if (flags & O_DIRECT) {
		LOG(LINF, "O_DIRECT is always enabled");
		flags &= ~O_DIRECT;
	}

	/* O_TMPFILE contains O_DIRECTORY */
	if ((flags & O_TMPFILE) == O_TMPFILE) {
		LOG(LTRC, "O_TMPFILE");
		flags &= ~O_TMPFILE;
	}

	if (flags & O_DIRECTORY) {
		LOG(LSUP, "O_DIRECTORY");
		flags &= ~O_DIRECTORY;
	}

	if (flags & O_DSYNC) {
		LOG(LINF, "O_DSYNC is always enabled");
		flags &= ~O_DSYNC;
	}

	if (flags & O_EXCL) {
		LOG(LTRC, "O_EXCL");
		flags &= ~O_EXCL;
	}

	if (flags & O_NOCTTY) {
		LOG(LINF, "O_NOCTTY is always enabled");
		flags &= ~O_NOCTTY;
	}

	if (flags & O_NOATIME) {
		LOG(LTRC, "O_NOATIME");
		flags &= ~O_NOATIME;
	}

	if (flags & O_NOFOLLOW) {
		LOG(LSUP, "O_NOFOLLOW");
		// XXX we don't support symlinks yet, so we can just ignore it
		flags &= ~O_NOFOLLOW;
	}

	if (flags & O_NONBLOCK) {
		LOG(LINF, "O_NONBLOCK is ignored");
		flags &= ~O_NONBLOCK;
	}

	if (flags & O_PATH) {
		LOG(LSUP, "O_PATH is not supported (yet)");
		errno = ENOTSUP;
		return -1;
	}

	if (flags & O_SYNC) {
		LOG(LINF, "O_SYNC is always enabled");
		flags &= ~O_SYNC;
	}

	if (flags & O_TRUNC) {
		LOG(LTRC, "O_TRUNC");
		flags &= ~O_TRUNC;
	}

	if ((flags & O_ACCMODE) == O_RDONLY) {
		LOG(LTRC, "O_RDONLY");
		flags -= O_RDONLY;
	}

	if ((flags & O_ACCMODE) == O_WRONLY) {
		LOG(LTRC, "O_WRONLY");
		flags -= O_WRONLY;
	}

	if ((flags & O_ACCMODE) == O_RDWR) {
		LOG(LTRC, "O_RDWR");
		flags -= O_RDWR;
	}

	if (flags) {
		ERR("unknown flag 0x%x\n", flags);
		errno = ENOTSUP;
		return -1;
	}

	return 0;
}

/*
 * file_check_pathname -- (internal) validates pathname
 */
const char *
file_check_pathname(const char *pathname)
{
	const char *orig_pathname = pathname;
	if (pathname[0] != '/') {
		LOG(LUSR, "pathname %s does not start with /", orig_pathname);
		errno = EINVAL;
		return NULL;
	}

	while (*pathname == '/')
		pathname++;

	if (strchr(pathname, '/')) {
		LOG(LSUP, "opening files in subdirectories is not supported yet"
			" (%s)", orig_pathname);
		errno = EISDIR;
		return NULL;
	}

	return pathname;
}

/*
 * pmemfile_open -- open file
 */
PMEMfile *
pmemfile_open(PMEMfilepool *pfp, const char *pathname, int flags, ...)
{
	if (!pathname) {
		LOG(LUSR, "NULL pathname");
		errno = EFAULT;
		return NULL;
	}

	LOG(LDBG, "pathname %s flags 0x%x", pathname, flags);

	const char *orig_pathname = pathname;

	if (file_check_flags(flags))
		return NULL;

	va_list ap;
	va_start(ap, flags);
	mode_t mode;

	/* NOTE: O_TMPFILE contains O_DIRECTORY */
	if ((flags & O_CREAT) || (flags & O_TMPFILE) == O_TMPFILE) {
		mode = va_arg(ap, mode_t);
		LOG(LDBG, "mode %o", mode);
		if (mode & ~(mode_t)(S_IRWXU | S_IRWXG | S_IRWXO)) {
			LOG(LUSR, "invalid mode 0%o", mode);
			errno = EINVAL;
			return NULL;
		}

		if (mode & (S_IXUSR | S_IXGRP | S_IXOTH)) {
			LOG(LSUP, "execute bits are not supported");
			mode = mode & ~(mode_t)(S_IXUSR | S_IXGRP | S_IXOTH);
		}
	}
	va_end(ap);

	pathname = file_check_pathname(pathname);
	if (!pathname)
		return NULL;

	int error = 0;
	PMEMfile *file = NULL;

	struct pmemfile_vinode *parent_vinode = pfp->root;
	struct pmemfile_vinode *volatile old_vinode;
	struct pmemfile_vinode *vinode;

	file_inode_ref(pfp, parent_vinode);
	old_vinode = vinode =
			file_lookup_dentry(pfp, parent_vinode, pathname);

	if ((flags & O_TMPFILE) == O_TMPFILE) {
		int oerrno = errno;
		file_vinode_unref_tx(pfp, parent_vinode);
		parent_vinode = vinode;
		old_vinode = vinode = NULL;
		errno = oerrno;
	}

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		if (vinode == NULL) {
			if ((flags & O_TMPFILE) == O_TMPFILE) {
				if (parent_vinode == NULL)
					pmemobj_tx_abort(ENOENT);
				if (!file_is_dir(parent_vinode))
					pmemobj_tx_abort(ENOTDIR);
				if ((flags & O_ACCMODE) == O_RDONLY)
					pmemobj_tx_abort(EINVAL);
			} else {
				if (errno != ENOENT) {
					ERR("!pmemfile_lookup_dentry failed in "
							"unexpected way");
					pmemobj_tx_abort(errno);
				}

				if (!(flags & O_CREAT)) {
					LOG(LUSR, "file %s does not exist",
							orig_pathname);
					pmemobj_tx_abort(ENOENT);
				}

				if (flags & O_DIRECTORY)
					LOG(LDBG, "O_DIRECTORY is ignored for "
							"O_CREAT");
			}

			// create file
			struct pmemfile_time t;

			rwlock_tx_wlock(&parent_vinode->rwlock);

			vinode = file_inode_alloc(pfp, S_IFREG | mode, &t);

			if ((flags & O_TMPFILE) == O_TMPFILE) {
				file_register_orphaned_inode(pfp, vinode);
			} else {
				file_add_dentry(pfp, parent_vinode, pathname,
						vinode, &t);
			}

			rwlock_tx_unlock_on_commit(&parent_vinode->rwlock);
		} else {
			if ((flags & (O_CREAT | O_EXCL)) ==
					(O_CREAT | O_EXCL)) {
				LOG(LUSR, "file %s already exists",
						orig_pathname);
				pmemobj_tx_abort(EEXIST);
			}

			if ((flags & O_DIRECTORY) && !file_is_dir(vinode))
				pmemobj_tx_abort(ENOTDIR);

			if (flags & O_TRUNC) {
				if (!file_is_regular_file(vinode)) {
					LOG(LUSR, "truncating non regular "
							"file");
					pmemobj_tx_abort(EINVAL);
				}

				if ((flags & O_ACCMODE) == O_RDONLY) {
					LOG(LUSR, "O_TRUNC without write "
							"permissions");
					pmemobj_tx_abort(EACCES);
				}

				rwlock_tx_wlock(&vinode->rwlock);

				file_truncate(vinode);

				rwlock_tx_unlock_on_commit(&vinode->rwlock);
			}
		}

		file = Zalloc(sizeof(*file));
		if (!file)
			pmemobj_tx_abort(errno);

		file->vinode = vinode;

		if ((flags & O_ACCMODE) == O_RDONLY)
			file->flags = PFILE_READ;
		else if ((flags & O_ACCMODE) == O_WRONLY)
			file->flags = PFILE_WRITE;
		else if ((flags & O_ACCMODE) == O_RDWR)
			file->flags = PFILE_READ | PFILE_WRITE;

		if (flags & O_NOATIME)
			file->flags |= PFILE_NOATIME;
		if (flags & O_APPEND)
			file->flags |= PFILE_APPEND;
	} TX_ONABORT {
		error = 1;
	} TX_END

	file_vinode_unref_tx(pfp, parent_vinode);

	if (error) {
		int oerrno = errno;

		if (old_vinode != NULL)
			file_vinode_unref_tx(pfp, old_vinode);

		errno = oerrno;
		LOG(LDBG, "!");

		return NULL;
	}

	if (old_vinode == NULL)
		file_set_path_debug(pfp, parent_vinode, vinode, pathname);

	util_mutex_init(&file->mutex, NULL);

	LOG(LDBG, "pathname %s opened inode 0x%lx", orig_pathname,
			file->vinode->inode.oid.off);
	return file;
}

/*
 * pmemfile_close -- close file
 */
void
pmemfile_close(PMEMfilepool *pfp, PMEMfile *file)
{
	LOG(LDBG, "inode 0x%lx path %s", file->vinode->inode.oid.off,
			pmfi_path(file->vinode));

	file_vinode_unref_tx(pfp, file->vinode);

	util_mutex_destroy(&file->mutex);

	Free(file);
}

/*
 * pmemfile_link -- make a new name for a file
 */
int
pmemfile_link(PMEMfilepool *pfp, const char *oldpath, const char *newpath)
{
	if (!oldpath || !newpath) {
		LOG(LUSR, "NULL pathname");
		errno = EFAULT;
		return -1;
	}

	LOG(LDBG, "oldpath %s newpath %s", oldpath, newpath);

	oldpath = file_check_pathname(oldpath);
	if (!oldpath)
		return -1;

	newpath = file_check_pathname(newpath);
	if (!newpath)
		return -1;

	struct pmemfile_vinode *parent_vinode = pfp->root;
	struct pmemfile_vinode *src_vinode;
	struct pmemfile_vinode *dst_vinode = NULL;

	int oerrno = 0;
	file_inode_ref(pfp, parent_vinode);

	src_vinode = file_lookup_dentry(pfp, parent_vinode, oldpath);
	if (src_vinode == NULL) {
		oerrno = errno;
		goto end;
	}

	dst_vinode = file_lookup_dentry(pfp, parent_vinode, newpath);
	if (dst_vinode != NULL) {
		oerrno = EEXIST;
		goto end;
	}

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		rwlock_tx_wlock(&parent_vinode->rwlock);

		struct pmemfile_time t;
		file_get_time(&t);
		file_add_dentry(pfp, parent_vinode, newpath, src_vinode, &t);

		rwlock_tx_unlock_on_commit(&parent_vinode->rwlock);
	} TX_ONABORT {
		oerrno = errno;
	} TX_END

	if (oerrno == 0)
		file_set_path_debug(pfp, parent_vinode, src_vinode, newpath);

end:
	if (dst_vinode != NULL)
		file_vinode_unref_tx(pfp, dst_vinode);
	if (src_vinode != NULL)
		file_vinode_unref_tx(pfp, src_vinode);
	file_vinode_unref_tx(pfp, parent_vinode);

	if (oerrno) {
		errno = oerrno;
		return -1;
	}

	return 0;
}

/*
 * pmemfile_unlink -- delete a name and possibly the file it refers to
 */
int
pmemfile_unlink(PMEMfilepool *pfp, const char *pathname)
{
	if (!pathname) {
		LOG(LUSR, "NULL pathname");
		errno = EFAULT;
		return -1;
	}

	LOG(LDBG, "pathname %s", pathname);

	pathname = file_check_pathname(pathname);
	if (!pathname)
		return -1;

	struct pmemfile_vinode *parent_vinode = pfp->root;

	int oerrno, ret = 0;

	file_inode_ref(pfp, parent_vinode);
	struct pmemfile_vinode *volatile vinode = NULL;

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		rwlock_tx_wlock(&parent_vinode->rwlock);
		file_unlink_dentry(pfp, parent_vinode, pathname, &vinode);
		rwlock_tx_unlock_on_commit(&parent_vinode->rwlock);
	} TX_ONABORT {
		oerrno = errno;
		ret = -1;
	} TX_END

	if (vinode)
		file_vinode_unref_tx(pfp, vinode);

	file_vinode_unref_tx(pfp, parent_vinode);

	if (ret)
		errno = oerrno;

	return ret;
}

/*
 * _pmemfile_list_root -- dumps root directory listing to log file
 *
 * XXX: remove once directory traversal API is implemented
 */
void
_pmemfile_list_root(PMEMfilepool *pfp, const char *msg)
{
	LOG(LINF, "START %s", msg);
	struct pmemfile_vinode *parent_vinode = pfp->root;
	file_inode_ref(pfp, parent_vinode);

	util_rwlock_rdlock(&parent_vinode->rwlock);

	_pmemfile_list(pfp, parent_vinode);

	util_rwlock_unlock(&parent_vinode->rwlock);

	file_vinode_unref_tx(pfp, parent_vinode);

	LOG(LINF, "STOP  %s", msg);
}

/*
 * _pmemfile_stats -- dumps pool statistics to log file
 *
 * XXX: figure out how to export this information and remove this function
 */
void
_pmemfile_stats(PMEMfilepool *pfp)
{
	PMEMoid oid;
	unsigned inodes = 0, dirs = 0, block_arrays = 0, inode_arrays = 0,
			blocks = 0;

	POBJ_FOREACH(pfp->pop, oid) {
		unsigned t = (unsigned)pmemobj_type_num(oid);

		if (t == TOID_TYPE_NUM(struct pmemfile_inode))
			inodes++;
		else if (t == TOID_TYPE_NUM(struct pmemfile_dir))
			dirs++;
		else if (t == TOID_TYPE_NUM(struct pmemfile_block_array))
			block_arrays++;
		else if (t == TOID_TYPE_NUM(struct pmemfile_inode_array))
			inode_arrays++;
		else if (t == TOID_TYPE_NUM(char))
			blocks++;
		else
			FATAL("unknown type %u", t);
	}

	LOG(LINF, "inodes %u dirs %u block_arrays %u inode_arrays %u blocks %u",
			inodes, dirs, block_arrays, inode_arrays, blocks);
}

/*
 * _pmemfile_file_size -- returns size of the file
 *
 * XXX: remove once pmemfile_stat is implemented
 */
size_t
_pmemfile_file_size(PMEMfilepool *pfp, PMEMfile *file)
{
	return D_RW(file->vinode->inode)->size;
}
