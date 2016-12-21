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

static bool
is_tmpfile(int flags)
{
#ifdef O_TMPFILE
	return (flags & O_TMPFILE) == O_TMPFILE;
#else
	return false;
#endif
}

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

#ifdef O_TMPFILE
	/* O_TMPFILE contains O_DIRECTORY */
	if ((flags & O_TMPFILE) == O_TMPFILE) {
		LOG(LTRC, "O_TMPFILE");
		flags &= ~O_TMPFILE;
	}
#endif

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

	struct pmemfile_vinode *vinode = inode_alloc(pfp, S_IFREG | mode, &t,
			parent_vinode, NULL, filename);

	if (is_tmpfile(flags))
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
	LOG(LDBG, "pathname %s flags 0x%x", pathname, flags);

	const char *orig_pathname = pathname;

	if (check_flags(flags))
		return NULL;

	va_list ap;
	va_start(ap, flags);
	mode_t mode = 0;

	/* NOTE: O_TMPFILE contains O_DIRECTORY */
	if ((flags & O_CREAT) || is_tmpfile(flags)) {
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
	traverse_path(pfp, dir, pathname, false, &info);
	vinode = info.vinode;

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		if (strchr(info.remaining, '/'))
			pmemobj_tx_abort(ENOENT);

		if (is_tmpfile(flags)) {
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
	if (!pathname) {
		LOG(LUSR, "NULL pathname");
		errno = ENOENT;
		return NULL;
	}

	va_list ap;
	va_start(ap, flags);
	mode_t mode = 0;
	if ((flags & O_CREAT) || is_tmpfile(flags))
		mode = va_arg(ap, mode_t);
	va_end(ap);

	struct pmemfile_vinode *at;
	bool at_unref;

	at = pool_get_dir_for_path(pfp, dir, pathname, &at_unref);

	PMEMfile *ret = _pmemfile_openat(pfp, at, pathname, flags, mode);
	int oerrno;
	if (!ret)
		oerrno = errno;

	if (at_unref)
		vinode_unref_tx(pfp, at);

	if (!ret)
		errno = oerrno;

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
	if ((flags & O_CREAT) || is_tmpfile(flags))
		mode = va_arg(ap, mode_t);
	va_end(ap);

	return pmemfile_openat(pfp, PMEMFILE_AT_CWD, pathname, flags, mode);
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
	LOG(LDBG, "oldpath %s newpath %s", oldpath, newpath);

	flags &= ~AT_SYMLINK_FOLLOW; /* No symlinks for now XXX */

	if (oldpath[0] == 0 && (flags & AT_EMPTY_PATH)) {
		LOG(LSUP, "AT_EMPTY_PATH not supported yet");
		errno = EINVAL;
		return -1;
	}

	flags &= ~AT_EMPTY_PATH;

	if (flags != 0) {
		errno = EINVAL;
		return -1;
	}

	struct pmemfile_path_info src, dst;
	traverse_path(pfp, olddir, oldpath, false, &src);
	traverse_path(pfp, newdir, newpath, false, &dst);

	int oerrno = 0;

	if (src.remaining[0] != 0 && !vinode_is_dir(src.vinode)) {
		oerrno = ENOTDIR;
		goto end;
	}

	if (dst.remaining[0] != 0 && !vinode_is_dir(dst.vinode)) {
		oerrno = ENOTDIR;
		goto end;
	}

	if (src.remaining[0] != 0 || strchr(dst.remaining, '/')) {
		oerrno = ENOENT;
		goto end;
	}

	if (dst.remaining[0] == 0) {
		oerrno = EEXIST;
		goto end;
	}

	if (vinode_is_dir(src.vinode)) {
		oerrno = EPERM;
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

	if (oerrno == 0) {
		vinode_clear_debug_path(pfp, src.vinode);
		vinode_set_debug_path(pfp, dst.vinode, src.vinode, newpath);
	}

end:
	vinode_unref_tx(pfp, dst.vinode);
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
	bool olddir_at_unref, newdir_at_unref;

	if (!oldpath || !newpath) {
		LOG(LUSR, "NULL pathname");
		errno = ENOENT;
		return -1;
	}

	olddir_at = pool_get_dir_for_path(pfp, olddir, oldpath,
			&olddir_at_unref);
	newdir_at = pool_get_dir_for_path(pfp, newdir, newpath,
			&newdir_at_unref);

	int ret = _pmemfile_linkat(pfp, olddir_at, oldpath, newdir_at, newpath,
			flags);
	int oerrno;
	if (ret)
		oerrno = errno;

	if (olddir_at_unref)
		vinode_unref_tx(pfp, olddir_at);

	if (newdir_at_unref)
		vinode_unref_tx(pfp, newdir_at);

	if (ret)
		errno = oerrno;

	return ret;
}

/*
 * pmemfile_link -- make a new name for a file
 */
int
pmemfile_link(PMEMfilepool *pfp, const char *oldpath, const char *newpath)
{
	struct pmemfile_vinode *at;

	if (!oldpath || !newpath) {
		LOG(LUSR, "NULL pathname");
		errno = ENOENT;
		return -1;
	}

	if (oldpath[0] == '/' && newpath[0] == '/')
		at = NULL;
	else
		at = pool_get_cwd(pfp);

	int ret = _pmemfile_linkat(pfp, at, oldpath, at, newpath, 0);

	int oerrno;
	if (ret)
		oerrno = errno;

	if (at)
		vinode_unref_tx(pfp, at);

	if (ret)
		errno = oerrno;

	return ret;
}

static int
_pmemfile_unlinkat(PMEMfilepool *pfp, struct pmemfile_vinode *dir,
		const char *pathname)
{
	LOG(LDBG, "pathname %s", pathname);

	int oerrno, ret = 0;

	struct pmemfile_path_info info;
	traverse_path(pfp, dir, pathname, true, &info);
	struct pmemfile_vinode *vparent = info.parent;
	struct pmemfile_vinode *volatile vinode2 = NULL;
	volatile bool parent_refed = false;

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		if (info.remaining[0]) {
			if (!vinode_is_dir(info.vinode))
				pmemobj_tx_abort(ENOTDIR);
			else
				pmemobj_tx_abort(ENOENT);
		}

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
	bool at_unref;

	if (!pathname) {
		errno = ENOENT;
		return -1;
	}

	at = pool_get_dir_for_path(pfp, dir, pathname, &at_unref);

	int ret;

	if (flags & AT_REMOVEDIR)
		ret = _pmemfile_rmdirat(pfp, at, pathname);
	else {
		if (flags != 0) {
			errno = EINVAL;
			ret = -1;
		} else {
			ret = _pmemfile_unlinkat(pfp, at, pathname);
		}
	}

	int oerrno;
	if (ret)
		oerrno = errno;
	if (at_unref)
		vinode_unref_tx(pfp, at);
	if (ret)
		errno = oerrno;

	return ret;
}

/*
 * pmemfile_unlink -- delete a name and possibly the file it refers to
 */
int
pmemfile_unlink(PMEMfilepool *pfp, const char *pathname)
{
	return pmemfile_unlinkat(pfp, PMEMFILE_AT_CWD, pathname, 0);
}

int
pmemfile_fcntl(PMEMfilepool *pfp, PMEMfile *file, int cmd, ...)
{
	int ret = 0;

	(void) pfp;
	(void) file;

	switch (cmd) {
		case F_SETLK:
		case F_UNLCK:
			// XXX
			return 0;
		case F_GETFL:
			ret |= O_LARGEFILE;
			if (file->flags & PFILE_APPEND)
				ret |= O_APPEND;
			if (file->flags & PFILE_NOATIME)
				ret |= O_NOATIME;
			if ((file->flags & PFILE_READ) == PFILE_READ)
				ret |= O_RDONLY;
			if ((file->flags & PFILE_WRITE) == PFILE_WRITE)
				ret |= O_WRONLY;
			if ((file->flags & (PFILE_READ | PFILE_WRITE)) ==
					(PFILE_READ | PFILE_WRITE))
				ret |= O_RDWR;
			return ret;
	}

	errno = ENOTSUP;
	return -1;
}

/*
 * pmemfile_stats -- get pool statistics
 */
void
pmemfile_stats(PMEMfilepool *pfp, struct pmemfile_stats *stats)
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
	stats->inodes = inodes;
	stats->dirs = dirs;
	stats->block_arrays = block_arrays;
	stats->inode_arrays = inode_arrays;
	stats->blocks = blocks;
}
