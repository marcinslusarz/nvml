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
 * dir.c -- directory operations
 */

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>

#include "callbacks.h"
#include "dir.h"
#include "file.h"
#include "inode.h"
#include "inode_array.h"
#include "internal.h"
#include "locks.h"
#include "out.h"
#include "sys_util.h"
#include "util.h"

/*
 * file_set_path_debug_locked -- (internal) sets full path in runtime structures
 * of child_inode based on parent inode and name.
 *
 * Works only in DEBUG mode.
 * Assumes child inode is already locked.
 */
static void
file_set_path_debug_locked(PMEMfilepool *pfp,
		struct pmemfile_vinode *parent_vinode,
		struct pmemfile_vinode *child_vinode,
		const char *name)
{
#ifdef DEBUG
	if (child_vinode->path)
		return;

	if (parent_vinode == NULL) {
		child_vinode->path = Strdup(name);
		return;
	}

	if (strcmp(parent_vinode->path, "/") == 0) {
		child_vinode->path = Malloc(strlen(name) + 2);
		sprintf(child_vinode->path, "/%s", name);
		return;
	}

	char *p = Malloc(strlen(parent_vinode->path) + 1 + strlen(name) + 1);
	sprintf(p, "%s/%s", parent_vinode->path, name);
	child_vinode->path = p;
#endif
}

/*
 * file_set_path_debug -- sets full path in runtime structures
 * of child_inode based on parent inode and name.
 */
void
file_set_path_debug(PMEMfilepool *pfp,
		struct pmemfile_vinode *parent_vinode,
		struct pmemfile_vinode *child_vinode,
		const char *name)
{
	util_rwlock_wrlock(&child_vinode->rwlock);

	file_set_path_debug_locked(pfp, parent_vinode, child_vinode, name);

	util_rwlock_unlock(&child_vinode->rwlock);
}

/*
 * file_add_dentry -- adds child inode to parent directory
 *
 * Must be called in transaction. Caller must have exclusive access to parent
 * inode, by locking parent in WRITE mode.
 */
void
file_add_dentry(PMEMfilepool *pfp,
		struct pmemfile_vinode *parent_vinode,
		const char *name,
		struct pmemfile_vinode *child_vinode,
		const struct pmemfile_time *tm)
{
	LOG(LDBG, "parent 0x%lx ppath %s name %s child_inode 0x%lx",
		parent_vinode->inode.oid.off, pmfi_path(parent_vinode),
		name, child_vinode->inode.oid.off);

	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	if (strlen(name) > PMEMFILE_MAX_FILE_NAME) {
		LOG(LUSR, "file name too long");
		pmemobj_tx_abort(EINVAL);
	}

	struct pmemfile_inode *parent = D_RW(parent_vinode->inode);

	struct pmemfile_dir *dir = &parent->file_data.dir;

	struct pmemfile_dirent *dentry = NULL;
	bool found = false;

	do {
		for (uint32_t i = 0; i < dir->num_elements; ++i) {
			if (strcmp(dir->dentries[i].name, name) == 0)
				pmemobj_tx_abort(EEXIST);

			if (!found && dir->dentries[i].name[0] == 0) {
				dentry = &dir->dentries[i];
				found = true;
			}
		}

		if (!found && TOID_IS_NULL(dir->next)) {
			TX_SET_DIRECT(dir, next,
					TX_ZALLOC(struct pmemfile_dir, 4096));

			size_t sz = pmemobj_alloc_usable_size(dir->next.oid);

			TX_ADD_DIRECT(&parent->size);
			parent->size += sz;

			D_RW(dir->next)->num_elements =
				(uint32_t)(sz - sizeof(struct pmemfile_dir)) /
					sizeof(struct pmemfile_dirent);
		}

		dir = D_RW(dir->next);
	} while (dir);

	TX_ADD_DIRECT(dentry);

	dentry->inode = child_vinode->inode;

	strncpy(dentry->name, name, PMEMFILE_MAX_FILE_NAME);
	dentry->name[PMEMFILE_MAX_FILE_NAME] = '\0';

	TX_ADD_FIELD(child_vinode->inode, nlink);
	D_RW(child_vinode->inode)->nlink++;

	/*
	 * From "stat" man page:
	 * "The field st_ctime is changed by writing or by setting inode
	 * information (i.e., owner, group, link count, mode, etc.)."
	 */
	TX_SET(child_vinode->inode, ctime, *tm);

	/*
	 * From "stat" man page:
	 * "st_mtime of a directory is changed by the creation
	 * or deletion of files in that directory."
	 */
	TX_SET(parent_vinode->inode, mtime, *tm);
}

/*
 * file_new_dir -- creates new directory relative to parent
 *
 * Note: caller must hold WRITE lock on parent.
 */
struct pmemfile_vinode *
file_new_dir(PMEMfilepool *pfp, struct pmemfile_vinode *parent,
		const char *name, mode_t mode, bool add_to_parent)
{
	LOG(LDBG, "parent 0x%lx ppath %s new_name %s",
			parent ? parent->inode.oid.off : 0,
			pmfi_path(parent), name);

	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	if (mode & ~(mode_t)0777) {
		/* TODO: what kernel does? */
		ERR("invalid mode flags 0%o", mode);
		pmemobj_tx_abort(EINVAL);
	}

	struct pmemfile_time t;
	struct pmemfile_vinode *child =
			file_inode_alloc(pfp, S_IFDIR | mode, &t);
	file_set_path_debug_locked(pfp, parent, child, name);

	/* add . and .. to new directory */
	file_add_dentry(pfp, child, ".", child, &t);

	if (parent == NULL) /* special case - root directory */
		file_add_dentry(pfp, child, "..", child, &t);
	else
		file_add_dentry(pfp, child, "..", parent, &t);

	if (add_to_parent)
		file_add_dentry(pfp, parent, name, child, &t);

	return child;
}

/*
 * file_lookup_dentry_locked -- looks up file name in passed directory
 *
 * Caller must hold lock on parent.
 */
static struct pmemfile_dirent *
file_lookup_dentry_locked(PMEMfilepool *pfp, struct pmemfile_vinode *parent,
		const char *name, struct pmemfile_dir **outdir)
{
	LOG(LDBG, "parent 0x%lx ppath %s name %s", parent->inode.oid.off,
			pmfi_path(parent), name);

	struct pmemfile_inode *par = D_RW(parent->inode);
	if (!_file_is_dir(par)) {
		errno = ENOTDIR;
		return NULL;
	}

	struct pmemfile_dir *dir = &par->file_data.dir;

	while (dir != NULL) {
		for (uint32_t i = 0; i < dir->num_elements; ++i) {
			struct pmemfile_dirent *d = &dir->dentries[i];

			if (strcmp(d->name, name) == 0) {
				if (outdir)
					*outdir = dir;
				return d;
			}
		}

		dir = D_RW(dir->next);
	}

	errno = ENOENT;
	return NULL;
}

/*
 * file_lookup_dentry -- looks up file name in passed directory
 *
 * Takes reference on found inode. Caller must hold reference to parent inode.
 * Does not need transaction.
 */
struct pmemfile_vinode *
file_lookup_dentry(PMEMfilepool *pfp, struct pmemfile_vinode *parent,
		const char *name)
{
	LOG(LDBG, "parent 0x%lx ppath %s name %s", parent->inode.oid.off,
			pmfi_path(parent), name);

	struct pmemfile_vinode *vinode = NULL;

	util_rwlock_rdlock(&parent->rwlock);

	/* XXX hack, deal with this properly */
	if (name[0] == 0 || (name[0] == '.' && (name[1] == 0 ||
			(name[1] == '.' && name[2] == 0)))) {
		vinode = file_vinode_ref(pfp, parent->inode);
	} else {
		struct pmemfile_dirent *dentry =
			file_lookup_dentry_locked(pfp, parent, name, NULL);
		if (dentry) {
			vinode = file_vinode_ref(pfp, dentry->inode);
			if (vinode)
				file_set_path_debug(pfp, parent, vinode, name);
		}
	}

	util_rwlock_unlock(&parent->rwlock);

	return vinode;
}

/*
 * file_unlink_dentry -- removes dentry from directory
 *
 * Must be called in transaction. Caller must have exclusive access to parent
 * inode, eg by locking parent in WRITE mode.
 */
void
file_unlink_dentry(PMEMfilepool *pfp, struct pmemfile_vinode *parent,
		const char *name, struct pmemfile_vinode *volatile *vinode)
{
	LOG(LDBG, "parent 0x%lx ppath %s name %s", parent->inode.oid.off,
			pmfi_path(parent), name);

	struct pmemfile_dir *dir;
	struct pmemfile_dirent *dentry =
			file_lookup_dentry_locked(pfp, parent, name, &dir);

	if (!dentry)
		pmemobj_tx_abort(errno);

	TOID(struct pmemfile_inode) tinode = dentry->inode;
	struct pmemfile_inode *inode = D_RW(tinode);

	if (_file_is_dir(inode))
		pmemobj_tx_abort(EISDIR);

	*vinode = file_vinode_ref(pfp, tinode);
	rwlock_tx_wlock(&(*vinode)->rwlock);

	ASSERT(inode->nlink > 0);

	TX_ADD_FIELD(tinode, nlink);
	TX_ADD_DIRECT(dentry);

	struct pmemfile_time tm;
	file_get_time(&tm);

	if (--inode->nlink == 0)
		file_register_orphaned_inode(pfp, *vinode);
	else {
		/*
		 * From "stat" man page:
		 * "The field st_ctime is changed by writing or by setting inode
		 * information (i.e., owner, group, link count, mode, etc.)."
		 */
		TX_SET((*vinode)->inode, ctime, tm);
	}
	/*
	 * From "stat" man page:
	 * "st_mtime of a directory is changed by the creation
	 * or deletion of files in that directory."
	 */
	TX_SET(parent->inode, mtime, tm);

	rwlock_tx_unlock_on_commit(&(*vinode)->rwlock);

	dentry->name[0] = '\0';
	dentry->inode = TOID_NULL(struct pmemfile_inode);
}

/*
 * _pmemfile_list -- dumps directory listing to log file
 *
 * XXX: remove once directory traversal API is implemented
 */
void
_pmemfile_list(PMEMfilepool *pfp, struct pmemfile_vinode *parent)
{
	LOG(LINF, "parent 0x%lx ppath %s", parent->inode.oid.off,
			pmfi_path(parent));

	struct pmemfile_inode *par = D_RW(parent->inode);

	struct pmemfile_dir *dir = &par->file_data.dir;

	LOG(LINF, "- ref    inode nlink   size   flags name");

	while (dir != NULL) {
		for (uint32_t i = 0; i < dir->num_elements; ++i) {
			const struct pmemfile_dirent *d = &dir->dentries[i];
			if (d->name[0] == 0)
				continue;

			const struct pmemfile_inode *inode = D_RO(d->inode);
			struct pmemfile_vinode *vinode;

			if (TOID_EQUALS(parent->inode, d->inode))
				vinode = file_vinode_get(pfp, d->inode, false);
			else {
				vinode = file_vinode_get(pfp, d->inode, true);
				if (vinode)
					file_set_path_debug(pfp, parent, vinode,
							d->name);
			}

			if (vinode == NULL)
				LOG(LINF, "0x%lx %d", d->inode.oid.off, errno);
			else
				LOG(LINF, "* %3d 0x%6lx %5lu %6lu 0%06lo %s",
					vinode->ref, d->inode.oid.off,
					inode->nlink, inode->size, inode->flags,
					d->name);

			if (!TOID_EQUALS(parent->inode, d->inode))
				file_vinode_unref_tx(pfp, vinode);
		}

		dir = D_RW(dir->next);
	}
}

#define DENTRY_ID_MASK 0xffffffffULL

#define DIR_ID(offset) ((offset) >> 32)
#define DENTRY_ID(offset) ((offset) & DENTRY_ID_MASK)

/*
 * file_seek_dir - translates between file->offset and dir/dentry
 *
 * returns 0 on EOF
 * returns !0 on successful translation
 */
static int
file_seek_dir(PMEMfile *file, struct pmemfile_inode *inode,
		struct pmemfile_dir **dir, unsigned *dentry)
{
	if (file->offset == 0) {
		*dir = &inode->file_data.dir;
	} else if (DIR_ID(file->offset) == file->dir_pos.dir_id) {
		*dir = file->dir_pos.dir;
		if (*dir == NULL)
			return 0;
	} else {
		*dir = &inode->file_data.dir;

		unsigned dir_id = 0;
		while (DIR_ID(file->offset) != dir_id) {
			if (TOID_IS_NULL((*dir)->next))
				return 0;
			*dir = D_RW((*dir)->next);
			++dir_id;
		}

		file->dir_pos.dir = *dir;
		file->dir_pos.dir_id = dir_id;
	}
	*dentry = DENTRY_ID(file->offset);

	while (*dentry >= (*dir)->num_elements) {
		if (TOID_IS_NULL((*dir)->next))
			return 0;

		*dentry -= (*dir)->num_elements;
		*dir = D_RW((*dir)->next);

		file->dir_pos.dir = *dir;
		file->dir_pos.dir_id++;
	}

	file->offset = ((size_t)file->dir_pos.dir_id) << 32 | *dentry;

	return 1;
}

static int
file_getdents(PMEMfilepool *pfp, PMEMfile *file, struct pmemfile_inode *inode,
		struct linux_dirent *dirp, unsigned count)
{
	struct pmemfile_dir *dir;
	unsigned dentry;

	if (file_seek_dir(file, inode, &dir, &dentry) == 0)
		return 0;

	int read1 = 0;
	char *data = (void *)dirp;

	while (true) {
		if (dentry >= dir->num_elements) {
			if (TOID_IS_NULL(dir->next))
				break;

			dir = D_RW(dir->next);
			file->dir_pos.dir = dir;
			file->dir_pos.dir_id++;
			dentry = 0;
			file->offset = ((size_t)file->dir_pos.dir_id) << 32 | 0;
		}

		struct pmemfile_dirent *dirent = &dir->dentries[dentry];
		if (TOID_IS_NULL(dirent->inode)) {
			++dentry;
			++file->offset;
			continue;
		}

		size_t namelen = strlen(dirent->name);
		unsigned short slen = (unsigned short)
				(8 + 8 + 2 + namelen + 1 + 1);
		uint64_t next_off = file->offset + 1;
		if (dentry + 1 >= dir->num_elements)
			next_off = ((next_off >> 32) + 1) << 32;

		if (count < slen)
			break;

		memcpy(data, &dirent->inode.oid.off, 8);
		data += 8;

		memcpy(data, &next_off, 8);
		data += 8;

		memcpy(data, &slen, 2);
		data += 2;

		memcpy(data, dirent->name, namelen + 1);
		data += namelen + 1;

		if (_file_is_regular_file(D_RO(dirent->inode)))
			*data = DT_REG;
		else
			*data = DT_DIR;
		data++;

		read1 += slen;

		++dentry;
		++file->offset;
	}

	return read1;
}

int
pmemfile_getdents(PMEMfilepool *pfp, PMEMfile *file,
			struct linux_dirent *dirp, unsigned count)
{
	struct pmemfile_vinode *vinode = file->vinode;

	if (!file_is_dir(vinode)) {
		errno = ENOTDIR;
		return -1;
	}

	if (!(file->flags & PFILE_READ)) {
		errno = EBADF;
		return -1;
	}

	if ((int)count < 0)
		count = INT_MAX;

	int bytes_read = 0;

	struct pmemfile_inode *inode = D_RW(vinode->inode);

	util_mutex_lock(&file->mutex);
	util_rwlock_rdlock(&vinode->rwlock);

	bytes_read = file_getdents(pfp, file, inode, dirp, count);
	ASSERT(bytes_read >= 0);

	util_rwlock_unlock(&vinode->rwlock);
	util_mutex_unlock(&file->mutex);

	ASSERT((unsigned)bytes_read <= count);
	return bytes_read;
}

static int
file_getdents64(PMEMfilepool *pfp, PMEMfile *file, struct pmemfile_inode *inode,
		struct linux_dirent64 *dirp, unsigned count)
{
	struct pmemfile_dir *dir;
	unsigned dentry;

	if (file_seek_dir(file, inode, &dir, &dentry) == 0)
		return 0;

	int read1 = 0;
	char *data = (void *)dirp;

	while (true) {
		if (dentry >= dir->num_elements) {
			if (TOID_IS_NULL(dir->next))
				break;

			dir = D_RW(dir->next);
			file->dir_pos.dir = dir;
			file->dir_pos.dir_id++;
			dentry = 0;
			file->offset = ((size_t)file->dir_pos.dir_id) << 32 | 0;
		}

		struct pmemfile_dirent *dirent = &dir->dentries[dentry];
		if (TOID_IS_NULL(dirent->inode)) {
			++dentry;
			++file->offset;
			continue;
		}

		size_t namelen = strlen(dirent->name);
		unsigned short slen = (unsigned short)
				(8 + 8 + 2 + 1 + namelen + 1);
		uint64_t next_off = file->offset + 1;
		if (dentry + 1 >= dir->num_elements)
			next_off = ((next_off >> 32) + 1) << 32;

		if (count < slen)
			break;

		memcpy(data, &dirent->inode.oid.off, 8);
		data += 8;

		memcpy(data, &next_off, 8);
		data += 8;

		memcpy(data, &slen, 2);
		data += 2;

		if (_file_is_regular_file(D_RO(dirent->inode)))
			*data = DT_REG;
		else
			*data = DT_DIR;
		data++;

		memcpy(data, dirent->name, namelen + 1);
		data += namelen + 1;

		read1 += slen;

		++dentry;
		++file->offset;
	}

	return read1;
}

int
pmemfile_getdents64(PMEMfilepool *pfp, PMEMfile *file,
			struct linux_dirent64 *dirp, unsigned count)
{
	struct pmemfile_vinode *vinode = file->vinode;

	if (!file_is_dir(vinode)) {
		errno = ENOTDIR;
		return -1;
	}

	if (!(file->flags & PFILE_READ)) {
		errno = EBADF;
		return -1;
	}

	if ((int)count < 0)
		count = INT_MAX;

	int bytes_read = 0;

	struct pmemfile_inode *inode = D_RW(vinode->inode);

	util_mutex_lock(&file->mutex);
	util_rwlock_rdlock(&vinode->rwlock);

	bytes_read = file_getdents64(pfp, file, inode, dirp, count);
	ASSERT(bytes_read >= 0);

	util_rwlock_unlock(&vinode->rwlock);
	util_mutex_unlock(&file->mutex);

	ASSERT((unsigned)bytes_read <= count);
	return bytes_read;
}

/*
 * traverse_pathat - traverses directory structure
 *
 * Traverses directory structure starting from parent using pathname
 * components from path.
 * Returns the deepest inode reachable and sets *name to the remaining path
 * that was unreachable.
 *
 * Takes reference on returned inode.
 */
static struct pmemfile_vinode *
traverse_pathat(PMEMfilepool *pfp, struct pmemfile_vinode *parent,
		const char *path, const char **name)
{
	char tmp[PATH_MAX];
	file_inode_ref(pfp, parent);

	while (1) {
		struct pmemfile_vinode *child;
		const char *slash = strchr(path, '/');

		if (slash == NULL) {
			child = file_lookup_dentry(pfp, parent, path);
			if (child) {
				file_vinode_unref_tx(pfp, parent);
				while (path[0])
					path++;

				*name = path;
				return child;
			} else {
				*name = path;
				return parent;
			}
		} else {
			strncpy(tmp, path, (uintptr_t)slash - (uintptr_t)path);
			tmp[slash - path] = 0;

			if (tmp[0] == 0) // workaround for file_lookup_dentry
				child = NULL;
			else
				child = file_lookup_dentry(pfp, parent, tmp);
			if (child) {
				file_vinode_unref_tx(pfp, parent);
				parent = child;
				path = slash + 1;
				while (path[0] == '/')
					path++;
			} else {
				*name = path;
				return parent;
			}
		}
	}
}

static struct pmemfile_vinode *
traverse_path(PMEMfilepool *pfp, const char *path, const char **name)
{
	if (path[0] != '/')
		return NULL;

	while (path[0] == '/')
		path++;

	return traverse_pathat(pfp, pfp->root, path, name);
}

int
pmemfile_mkdir(PMEMfilepool *pfp, const char *path, mode_t mode)
{
	const char *name;
	struct pmemfile_vinode *parent = traverse_path(pfp, path, &name);

	if (!parent) {
		errno = ENOENT;
		return -1;
	}

	if (name[0] == 0) {
		file_vinode_unref_tx(pfp, parent);
		errno = EEXIST;
		return -1;
	}

	if (!file_is_dir(parent)) {
		file_vinode_unref_tx(pfp, parent);
		errno = ENOTDIR;
		return -1;
	}

	if (strchr(name, '/')) {
		file_vinode_unref_tx(pfp, parent);
		errno = ENOENT;
		return -1;
	}

	int error = 0;
	int txerrno = 0;
	struct pmemfile_vinode *child = NULL;

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		rwlock_tx_wlock(&parent->rwlock);

		child = file_new_dir(pfp, parent, name, mode, true);

		rwlock_tx_unlock_on_commit(&parent->rwlock);
	} TX_ONABORT {
		error = 1;
		txerrno = errno;
	} TX_END

	if (!error)
		file_vinode_unref_tx(pfp, child);

	file_vinode_unref_tx(pfp, parent);

	if (error) {
		errno = txerrno;
		return -1;
	}

	return 0;
}
