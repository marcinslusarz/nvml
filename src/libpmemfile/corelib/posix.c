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
 * posix.c -- the POSIX inspired public interface
 */

#include "libpmemfile-core.h"

#include <pthread.h>

#include "dir.h"
#include "file.h"
#include "inode.h"
#include "pool.h"
#include "internal.h"
#include "sys_util.h"

/*
 * choose_parent_vinode -- Choose the right inode used
 * for resolving a path present in a syscall argument.
 * If the path starts with a '/' character, it is an absolute path.
 * Otherwise path resolving should start either at the current working
 * directory, and at a used supplied inode ( in the *at syscalls ).
 *
 * This must be done while holding the pool's rwlock.
 * TODO: That rwlock is not relevant when dir->vinode ( supplied by the user )
 * is returned, so might want eliminate the rwlock_rdlock and rwlock_unlock
 * calls.
 */
static struct pmemfile_vinode *
choose_parent_vinode(PMEMfilepool *pfp, PMEMfile *dir, const char *pathname)
{
	return pfp->root;
/*
	if (pathname[0] == '/')
		return pfp->root;
	else if (dir == PMEMFILE_AT_CWD)
		return pfp->cwd;
	else
		return dir->vinode;
*/
}

/*
 * acquire_parent_vinode_at, acquire_parent_vinode_at2 -- These are just
 * wrappers around the choose_parent_vinode function.
 * Holding the rwlock, and increasing the ref count of the choosed vinode.
 * The caller is responsible for decreasing the ref acount.
 */
static void
acquire_parent_vinode_at(PMEMfilepool *pfp, PMEMfile *dir,
		const char *pathname,
		struct pmemfile_vinode **parent)
{
	pthread_rwlock_rdlock(&pfp->rwlock);

	*parent = choose_parent_vinode(pfp, dir, pathname);
	file_inode_ref(pfp, *parent);

	pthread_rwlock_unlock(&pfp->rwlock);
}

static void
acquire_parent_vinode_at2(PMEMfilepool *pfp,
		PMEMfile *dir1,
		const char *pathname1,
		struct pmemfile_vinode **parent1,
		PMEMfile *dir2,
		const char *pathname2,
		struct pmemfile_vinode **parent2)
{
	pthread_rwlock_rdlock(&pfp->rwlock);

	*parent1 = choose_parent_vinode(pfp, dir1, pathname1);
	file_inode_ref(pfp, *parent1);
	*parent2 = choose_parent_vinode(pfp, dir2, pathname2);
	file_inode_ref(pfp, *parent2);

	pthread_rwlock_unlock(&pfp->rwlock);
}

/*
 * relativize -- a silly sounding word, anyways, this makes
 * sure all paths passed to pmemfile are relative paths.
 * ( relative to the choosen parent vinode )
 *
 * TODO: is this needed?
 */
static const char *
relativize(const char *pathname)
{
	while (pathname[0] == '/')
		++pathname;

	return pathname;
}

static int
check_path(const char *path)
{
	if (path == NULL) {
		LOG(LUSR, "NULL pathname");
		errno = EFAULT;
		return -1;
	}

	if (path[0] != '/') {
		LOG(LUSR, "pathname %s does not start with /", path);
		errno = EINVAL;
		return -1;
	}

	if (strchr(path + strspn(path, "/"), '/') != NULL) {
		LOG(LSUP, "opening files in subdirectories is not supported yet"
			" (%s)", path);
		errno = EISDIR;
		return -1;
	}

	return 0;
}

/*
 * pmemfile_open -- open dir
 */
PMEMfile *
pmemfile_open(PMEMfilepool *pfp, const char *pathname, int flags, ...)
{
	PMEMfile *result;

	va_list ap;
	va_start(ap, flags);
	result = pmemfile_openat(pfp, PMEMFILE_AT_CWD, pathname,
					flags, va_arg(ap, mode_t));
	va_end(ap);

	return result;
}

/*
 * pmemfile_openat -- open dir
 */
PMEMfile *
pmemfile_openat(PMEMfilepool *pfp, PMEMfile *dir, const char *path,
		int flags, ...)
{
	PMEMfile *result;
	struct pmemfile_vinode *parent;

	if (check_path(path) != 0)
		return NULL;

	acquire_parent_vinode_at(pfp, dir, path, &parent);

	va_list ap;
	va_start(ap, flags);
	result = file_open_at_vinode(pfp, parent, relativize(path),
					flags, va_arg(ap, mode_t));
	va_end(ap);

	file_vinode_unref_tx(pfp, parent);

	return result;
}

/*
 * pmemfile_close -- close dir
 */
void
pmemfile_close(PMEMfilepool *pfp, PMEMfile *dir)
{
	file_close(pfp, dir);
}

/*
 * pmemfile_link -- make a new name for a dir
 */
int
pmemfile_link(PMEMfilepool *pfp, const char *oldpath, const char *newpath)
{
	return pmemfile_linkat(pfp,
	    PMEMFILE_AT_CWD, oldpath,
	    PMEMFILE_AT_CWD, newpath);
}

/*
 * pmemfile_link_at -- make a new name for a dir
 */
int
pmemfile_linkat(PMEMfilepool *pfp, PMEMfile *dir1, const char *oldpath,
		PMEMfile *dir2, const char *newpath)
{
	int result;
	struct pmemfile_vinode *parent1;
	struct pmemfile_vinode *parent2;

	if (check_path(oldpath) != 0)
		return -1;

	if (check_path(newpath) != 0)
		return -1;

	acquire_parent_vinode_at2(pfp, dir1, oldpath, &parent1,
	    dir2, newpath, &parent2);

	result = file_link_at_vinodes(pfp,
	    parent1, relativize(oldpath),
	    parent2, relativize(newpath));

	file_vinode_unref_tx(pfp, parent1);
	file_vinode_unref_tx(pfp, parent2);

	return result;
}

/*
 * pmemfile_unlink -- delete a name and possibly the dir it refers to
 */
int
pmemfile_unlink(PMEMfilepool *pfp, const char *pathname)
{
	return pmemfile_unlinkat(pfp, PMEMFILE_AT_CWD, pathname);
}

/*
 * pmemfile_unlinkat -- delete a name and possibly the dir it refers to
 */
int
pmemfile_unlinkat(PMEMfilepool *pfp, PMEMfile *dir, const char *path)
{
	int result;
	struct pmemfile_vinode *parent;

	if (check_path(path) != 0)
		return -1;

	acquire_parent_vinode_at(pfp, dir, path, &parent);

	result = file_unlink_at_vinode(pfp, parent, relativize(path));

	file_vinode_unref_tx(pfp, parent);

	return result;
}

/*
 * pmemfile_fstat
 */
int
pmemfile_fstat(PMEMfilepool *pfp, PMEMfile *dir, struct stat *buf)
{
	if (dir == NULL || buf == NULL) {
		errno = EFAULT;
		return -1;
	}

	return file_fill_stat(dir->vinode, buf);
}

/*
 * pmemfile_stat
 */
int
pmemfile_stat(PMEMfilepool *pfp, const char *path, struct stat *buf)
{
	return pmemfile_statat(pfp, PMEMFILE_AT_CWD, path, buf);
}

/*
 * pmemfile_statat
 */
int
pmemfile_statat(PMEMfilepool *pfp, PMEMfile *dir, const char *path,
			struct stat *buf)
{
	int result;
	struct pmemfile_vinode *parent;

	if (check_path(path) != 0)
		return -1;

	if (buf == NULL) {
		errno = EFAULT;
		return -1;
	}

	acquire_parent_vinode_at(pfp, dir, path, &parent);

	result = file_stat_at_vinode(pfp, parent, relativize(path), buf);

	file_vinode_unref_tx(pfp, parent);

	return result;
}

/*
 * pmemfile_lstat
 */
int
pmemfile_lstat(PMEMfilepool *pfp, const char *path, struct stat *buf)
{
	return pmemfile_lstatat(pfp, PMEMFILE_AT_CWD, path, buf);
}

/*
 * pmemfile_lstatat
 */
int
pmemfile_lstatat(PMEMfilepool *pfp, PMEMfile *dir, const char *path,
		struct stat *buf)
{
	// XXX because symlinks are not yet implemented
	return pmemfile_statat(pfp, dir, path, buf);
}
