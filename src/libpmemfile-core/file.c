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
 * check_flags -- (internal) open(2) flags tester
 */
static int
check_flags(int flags)
{
	if (flags & O_APPEND) {
		LOG(LSUP, "O_APPEND");
		flags &= ~O_APPEND;
	}

	if (flags & O_ASYNC) {
		LOG(LSUP, "O_ASYNC is not supported");
		errno = EINVAL;
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
		errno = EINVAL;
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
		errno = EINVAL;
		return -1;
	}

	return 0;
}

static struct pmemfile_vinode *
create_file(PMEMfilepool *pfp, const char *filename, const char *full_path,
		struct pmemfile_vinode *parent_vinode, int flags, mode_t mode)
{
	struct pmemfile_time t;

	rwlock_tx_wlock(&parent_vinode->rwlock);

	struct pmemfile_vinode *vinode =
			inode_alloc(pfp, S_IFREG | mode, &t, NULL, NULL);

	if ((flags & O_TMPFILE) == O_TMPFILE)
		vinode_orphan(pfp, vinode);
	else
		vinode_add_dirent(pfp, parent_vinode, filename, vinode, &t);

	rwlock_tx_unlock_on_commit(&parent_vinode->rwlock);

	return vinode;
}

static void
open_file(const char *orig_pathname, struct pmemfile_vinode *vinode, int flags)
{
	if ((flags & O_DIRECTORY) && !vinode_is_dir(vinode))
		pmemobj_tx_abort(ENOTDIR);

	if (flags & O_TRUNC) {
		if (!vinode_is_regular_file(vinode)) {
			LOG(LUSR, "truncating non regular file");
			pmemobj_tx_abort(EINVAL);
		}

		if ((flags & O_ACCMODE) == O_RDONLY) {
			LOG(LUSR, "O_TRUNC without write permissions");
			pmemobj_tx_abort(EACCES);
		}

		rwlock_tx_wlock(&vinode->rwlock);

		vinode_truncate(vinode);

		rwlock_tx_unlock_on_commit(&vinode->rwlock);
	}
}

/*
 * _pmemfile_openat -- open file
 */
static PMEMfile *
_pmemfile_openat(PMEMfilepool *pfp, struct pmemfile_vinode *dir,
		const char *pathname, int flags, ...)
{
	if (!pathname) {
		LOG(LUSR, "NULL pathname");
		errno = EFAULT;
		return NULL;
	}

	LOG(LDBG, "pathname %s flags 0x%x", pathname, flags);

	const char *orig_pathname = pathname;

	if (check_flags(flags))
		return NULL;

	va_list ap;
	va_start(ap, flags);
	mode_t mode = 0;

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

	int error = 0;
	int txerrno = 0;
	PMEMfile *file = NULL;

	struct pmemfile_path_info info;

	struct pmemfile_vinode *volatile vparent = NULL;
	struct pmemfile_vinode *volatile vinode;
	if (pathname[0] == '/')
		traverse_path(pfp, pathname, false, &info);
	else
		traverse_pathat(pfp, dir, pathname, false, &info);
	vinode = info.vinode;

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		// TODO: remove once relative paths work
		if (vinode == NULL)
			pmemobj_tx_abort(EINVAL);

		if (strchr(info.remaining, '/'))
			pmemobj_tx_abort(ENOENT);

		if ((flags & O_TMPFILE) == O_TMPFILE) {
			if (!vinode_is_dir(vinode))
				pmemobj_tx_abort(ENOTDIR);
			if (info.remaining[0])
				pmemobj_tx_abort(ENOENT);
			if ((flags & O_ACCMODE) == O_RDONLY)
				pmemobj_tx_abort(EINVAL);

			vparent = vinode;
			vinode = NULL;
		} else if ((flags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL)) {
			if (info.remaining[0] == 0) {
				LOG(LUSR, "file %s already exists", pathname);
				pmemobj_tx_abort(EEXIST);
			}
			vparent = vinode;
			vinode = NULL;
		} else if (flags & O_CREAT) {
			if (info.remaining[0] != 0) {
				vparent = vinode;
				vinode = NULL;
			}
		} else if (info.remaining[0] != 0)
			pmemobj_tx_abort(ENOENT);

		if (vinode == NULL) {
			vinode = create_file(pfp, info.remaining,
					orig_pathname, vparent, flags, mode);
		} else {
			open_file(orig_pathname, vinode, flags);
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
		txerrno = errno;
	} TX_END

	if (!error)
		vinode_set_debug_path(pfp, vparent, vinode, pathname);

	if (vparent)
		vinode_unref_tx(pfp, vparent);

	if (error) {
		if (vinode != NULL)
			vinode_unref_tx(pfp, vinode);

		errno = txerrno;
		LOG(LDBG, "!");

		return NULL;
	}

	util_mutex_init(&file->mutex, NULL);

	LOG(LDBG, "pathname %s opened inode 0x%lx", orig_pathname,
			file->vinode->inode.oid.off);
	return file;
}

/*
 * pmemfile_openat -- open file
 */
PMEMfile *
pmemfile_openat(PMEMfilepool *pfp, PMEMfile *dir, const char *pathname,
		int flags, ...)
{
	va_list ap;
	va_start(ap, flags);
	mode_t mode = 0;
	if ((flags & O_CREAT) || (flags & O_TMPFILE) == O_TMPFILE)
		mode = va_arg(ap, mode_t);
	va_end(ap);

	struct pmemfile_vinode *at;
	int at_unref = 0;

	if (dir == PMEMFILE_AT_CWD) {
		if (pathname && pathname[0] != '/') {
			at = pool_get_cwd(pfp);
			at_unref = 1;
		} else
			at = NULL;
	} else
		at = dir->vinode;

	PMEMfile *ret = _pmemfile_openat(pfp, at, pathname, flags, mode);

	if (at_unref)
		vinode_unref_tx(pfp, at);

	return ret;
}

/*
 * pmemfile_open -- open file
 */
PMEMfile *
pmemfile_open(PMEMfilepool *pfp, const char *pathname, int flags, ...)
{
	va_list ap;
	va_start(ap, flags);
	mode_t mode = 0;
	if ((flags & O_CREAT) || (flags & O_TMPFILE) == O_TMPFILE)
		mode = va_arg(ap, mode_t);
	va_end(ap);

	struct pmemfile_vinode *at;
	if (pathname && pathname[0] == '/')
		at = NULL;
	else
		at = pool_get_cwd(pfp);

	PMEMfile *f = _pmemfile_openat(pfp, at, pathname, flags, mode);

	if (at)
		vinode_unref_tx(pfp, at);

	return f;
}

/*
 * pmemfile_close -- close file
 */
void
pmemfile_close(PMEMfilepool *pfp, PMEMfile *file)
{
	LOG(LDBG, "inode 0x%lx path %s", file->vinode->inode.oid.off,
			pmfi_path(file->vinode));

	vinode_unref_tx(pfp, file->vinode);

	util_mutex_destroy(&file->mutex);

	Free(file);
}

static int
_pmemfile_linkat(PMEMfilepool *pfp,
		struct pmemfile_vinode *olddir, const char *oldpath,
		struct pmemfile_vinode *newdir, const char *newpath,
		int flags)
{
	if (!oldpath || !newpath) {
		LOG(LUSR, "NULL pathname");
		errno = EFAULT;
		return -1;
	}

	LOG(LDBG, "oldpath %s newpath %s", oldpath, newpath);

	flags &= ~AT_SYMLINK_FOLLOW; /* No symlinks for now XXX */

	if (oldpath[0] == 0 && (flags & AT_EMPTY_PATH)) {
		errno = EINVAL;
		return -1;
	}

	flags &= ~AT_EMPTY_PATH;

	if (flags != 0) {
		errno = EINVAL;
		return -1;
	}

	struct pmemfile_path_info src, dst;
	if (oldpath[0] == '/')
		traverse_path(pfp, oldpath, false, &src);
	else
		traverse_pathat(pfp, olddir, oldpath, false, &src);

	if (newpath[0] == '/')
		traverse_path(pfp, newpath, false, &dst);
	else
		traverse_pathat(pfp, newdir, newpath, false, &dst);

	int oerrno = 0;
	if (dst.vinode == NULL || src.vinode == NULL || src.remaining[0] != 0 ||
			strchr(dst.remaining, '/')) {
		oerrno = ENOENT;
		goto end;
	}
	if (dst.remaining[0] == 0) {
		oerrno = EEXIST;
		goto end;
	}

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		rwlock_tx_wlock(&dst.vinode->rwlock);

		struct pmemfile_time t;
		file_get_time(&t);
		vinode_add_dirent(pfp, dst.vinode, dst.remaining, src.vinode,
				&t);

		rwlock_tx_unlock_on_commit(&dst.vinode->rwlock);
	} TX_ONABORT {
		oerrno = errno;
	} TX_END

	if (oerrno == 0)
		vinode_set_debug_path(pfp, dst.vinode, src.vinode, newpath);

end:
	if (dst.vinode != NULL)
		vinode_unref_tx(pfp, dst.vinode);
	if (src.vinode != NULL)
		vinode_unref_tx(pfp, src.vinode);

	if (oerrno) {
		errno = oerrno;
		return -1;
	}

	return 0;
}

int
pmemfile_linkat(PMEMfilepool *pfp, PMEMfile *olddir, const char *oldpath,
		PMEMfile *newdir, const char *newpath, int flags)
{
	struct pmemfile_vinode *olddir_at, *newdir_at;
	int olddir_at_unref = 0, newdir_at_unref = 0;

	if (olddir == PMEMFILE_AT_CWD) {
		if (oldpath && oldpath[0] != '/') {
			olddir_at = pool_get_cwd(pfp);
			olddir_at_unref = 1;
		} else
			olddir_at = NULL;
	} else
		olddir_at = olddir->vinode;

	if (newdir == PMEMFILE_AT_CWD) {
		if (newpath && newpath[0] != '/') {
			if (olddir_at_unref) {
				newdir_at = olddir_at;
			} else {
				newdir_at = pool_get_cwd(pfp);
				newdir_at_unref = 1;
			}
		} else
			newdir_at = NULL;
	} else
		newdir_at = newdir->vinode;

	int ret = _pmemfile_linkat(pfp, olddir_at, oldpath, newdir_at, newpath,
			flags);

	if (olddir_at_unref)
		vinode_unref_tx(pfp, olddir_at);

	if (newdir_at_unref)
		vinode_unref_tx(pfp, newdir_at);

	return ret;
}

/*
 * pmemfile_link -- make a new name for a file
 */
int
pmemfile_link(PMEMfilepool *pfp, const char *oldpath, const char *newpath)
{
	struct pmemfile_vinode *at;
	if (oldpath && oldpath[0] == '/' && newpath && newpath[0] == '/')
		at = NULL;
	else
		at = pool_get_cwd(pfp);

	int ret = _pmemfile_linkat(pfp, at, oldpath, at, newpath, 0);

	if (at)
		vinode_unref_tx(pfp, at);

	return ret;
}

static int
_pmemfile_unlinkat(PMEMfilepool *pfp, struct pmemfile_vinode *dir,
		const char *pathname, int flags)
{
	if (!pathname) {
		LOG(LUSR, "NULL pathname");
		errno = EFAULT;
		return -1;
	}

	LOG(LDBG, "pathname %s", pathname);

	if (flags & AT_REMOVEDIR) {
		LOG(LSUP, "AT_REMOVEDIR is not yet supported");
		errno = EINVAL;
		return -1;
	}

	if (flags != 0) {
		errno = EINVAL;
		return -1;
	}

	int oerrno, ret = 0;

	struct pmemfile_path_info info;
	if (pathname[0] == '/')
		traverse_path(pfp, pathname, true, &info);
	else
		traverse_pathat(pfp, dir, pathname, true, &info);
	struct pmemfile_vinode *vparent = info.parent;
	struct pmemfile_vinode *volatile vinode2 = NULL;
	volatile bool parent_refed = false;

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		if (info.vinode == NULL)
			pmemobj_tx_abort(EINVAL);

		if (info.remaining[0])
			pmemobj_tx_abort(ENOENT);

		if (vinode_is_dir(info.vinode))
			pmemobj_tx_abort(EISDIR);

		rwlock_tx_wlock(&vparent->rwlock);
		vinode_unlink_dirent(pfp, vparent, info.name, &vinode2,
				&parent_refed);
		rwlock_tx_unlock_on_commit(&vparent->rwlock);
	} TX_ONABORT {
		oerrno = errno;
		ret = -1;
	} TX_END

	if (info.vinode)
		vinode_unref_tx(pfp, info.vinode);
	if (vinode2)
		vinode_unref_tx(pfp, vinode2);
	if (vparent)
		vinode_unref_tx(pfp, vparent);

	if (ret) {
		if (parent_refed)
			vinode_unref_tx(pfp, vparent);
		errno = oerrno;
	}

	return ret;
}

int
pmemfile_unlinkat(PMEMfilepool *pfp, PMEMfile *dir, const char *pathname,
		int flags)
{
	struct pmemfile_vinode *at;
	int at_unref = 0;

	if (dir == PMEMFILE_AT_CWD) {
		if (pathname && pathname[0] != '/') {
			at = pool_get_cwd(pfp);
			at_unref = 1;
		} else
			at = NULL;
	} else
		at = dir->vinode;

	int ret = _pmemfile_unlinkat(pfp, at, pathname, flags);

	if (at_unref)
		vinode_unref_tx(pfp, at);

	return ret;
}

/*
 * pmemfile_unlink -- delete a name and possibly the file it refers to
 */
int
pmemfile_unlink(PMEMfilepool *pfp, const char *pathname)
{
	struct pmemfile_vinode *at;
	if (pathname && pathname[0] == '/')
		at = NULL;
	else
		at = pool_get_cwd(pfp);

	int ret = _pmemfile_unlinkat(pfp, at, pathname, 0);

	if (at)
		vinode_unref_tx(pfp, at);

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
	vinode_ref(pfp, parent_vinode);

	util_rwlock_rdlock(&parent_vinode->rwlock);

	_pmemfile_list(pfp, parent_vinode);

	util_rwlock_unlock(&parent_vinode->rwlock);

	vinode_unref_tx(pfp, parent_vinode);

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
