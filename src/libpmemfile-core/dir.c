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
 * vinode_set_debug_path_locked -- (internal) sets full path in runtime
 * structures of child_inode based on parent inode and name.
 *
 * Works only in DEBUG mode.
 * Assumes child inode is already locked.
 */
void
vinode_set_debug_path_locked(PMEMfilepool *pfp,
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
 * vinode_set_debug_path -- sets full path in runtime structures
 * of child_inode based on parent inode and name.
 */
void
vinode_set_debug_path(PMEMfilepool *pfp,
		struct pmemfile_vinode *parent_vinode,
		struct pmemfile_vinode *child_vinode,
		const char *name)
{
	util_rwlock_wrlock(&child_vinode->rwlock);

	vinode_set_debug_path_locked(pfp, parent_vinode, child_vinode, name);

	util_rwlock_unlock(&child_vinode->rwlock);
}

/*
 * vinode_clear_debug_path -- clears full path in runtime structures
 */
void
vinode_clear_debug_path(PMEMfilepool *pfp, struct pmemfile_vinode *vinode)
{
	util_rwlock_wrlock(&vinode->rwlock);
#ifdef DEBUG
	Free(vinode->path);
	vinode->path = NULL;
#endif
	util_rwlock_unlock(&vinode->rwlock);
}

/*
 * vinode_add_dirent -- adds child inode to parent directory
 *
 * Must be called in transaction. Caller must have exclusive access to parent
 * inode, by locking parent in WRITE mode.
 */
void
vinode_add_dirent(PMEMfilepool *pfp,
		struct pmemfile_vinode *parent_vinode,
		const char *name,
		struct pmemfile_vinode *child_vinode,
		const struct pmemfile_time *tm)
{
	LOG(LDBG, "parent 0x%lx ppath %s name %s child_inode 0x%lx",
		parent_vinode->inode.oid.off, pmfi_path(parent_vinode),
		name, child_vinode->inode.oid.off);

	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	size_t namelen = strlen(name);
	if (namelen > PMEMFILE_MAX_FILE_NAME) {
		LOG(LUSR, "file name too long");
		pmemobj_tx_abort(ENAMETOOLONG);
	}

	if (strchr(name, '/') != NULL)
		FATAL("trying to add dirent with slash: %s", name);

	struct pmemfile_inode *parent = D_RW(parent_vinode->inode);

	struct pmemfile_dir *dir = &parent->file_data.dir;

	struct pmemfile_dirent *dirent = NULL;
	bool found = false;

	do {
		for (uint32_t i = 0; i < dir->num_elements; ++i) {
			if (strcmp(dir->dirents[i].name, name) == 0)
				pmemobj_tx_abort(EEXIST);

			if (!found && dir->dirents[i].name[0] == 0) {
				dirent = &dir->dirents[i];
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

	pmemobj_tx_add_range_direct(dirent,
			sizeof(dirent->inode) + namelen + 1);

	dirent->inode = child_vinode->inode;

	strncpy(dirent->name, name, namelen);
	dirent->name[namelen] = '\0';

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
 * vinode_new_dir -- creates new directory relative to parent
 *
 * Note: caller must hold WRITE lock on parent.
 */
struct pmemfile_vinode *
vinode_new_dir(PMEMfilepool *pfp, struct pmemfile_vinode *parent,
		const char *name, mode_t mode, bool add_to_parent,
		volatile bool *parent_refed)
{
	LOG(LDBG, "parent 0x%lx ppath %s new_name %s",
			parent ? parent->inode.oid.off : 0,
			pmfi_path(parent), name);

	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	if (mode & ~(mode_t)0777) {
		/* XXX: what the kernel does in this case? */
		ERR("invalid mode flags 0%o", mode);
		pmemobj_tx_abort(EINVAL);
	}

	struct pmemfile_time t;
	struct pmemfile_vinode *child = inode_alloc(pfp, S_IFDIR | mode, &t,
			parent, parent_refed, name);

	/* add . and .. to new directory */
	vinode_add_dirent(pfp, child, ".", child, &t);

	if (parent == NULL) /* special case - root directory */
		vinode_add_dirent(pfp, child, "..", child, &t);
	else
		vinode_add_dirent(pfp, child, "..", parent, &t);

	if (add_to_parent)
		vinode_add_dirent(pfp, parent, name, child, &t);

	return child;
}

/*
 * vinode_lookup_dirent_by_name_locked -- looks up file name in passed directory
 *
 * Caller must hold lock on parent.
 */
static struct pmemfile_dirent *
vinode_lookup_dirent_by_name_locked(PMEMfilepool *pfp,
		struct pmemfile_vinode *parent, const char *name)
{
	LOG(LDBG, "parent 0x%lx ppath %s name %s", parent->inode.oid.off,
			pmfi_path(parent), name);

	struct pmemfile_inode *iparent = D_RW(parent->inode);
	if (!inode_is_dir(iparent)) {
		errno = ENOTDIR;
		return NULL;
	}

	struct pmemfile_dir *dir = &iparent->file_data.dir;

	while (dir != NULL) {
		for (uint32_t i = 0; i < dir->num_elements; ++i) {
			struct pmemfile_dirent *d = &dir->dirents[i];

			if (strcmp(d->name, name) == 0)
				return d;
		}

		dir = D_RW(dir->next);
	}

	errno = ENOENT;
	return NULL;
}

/*
 * vinode_lookup_dirent_by_vinode_locked -- looks up file name in passed
 * directory
 *
 * Caller must hold lock on parent.
 */
static struct pmemfile_dirent *
vinode_lookup_dirent_by_vinode_locked(PMEMfilepool *pfp,
		struct pmemfile_vinode *parent,	struct pmemfile_vinode *child)
{
	LOG(LDBG, "parent 0x%lx ppath %s", parent->inode.oid.off,
			pmfi_path(parent));

	struct pmemfile_inode *iparent = D_RW(parent->inode);
	if (!inode_is_dir(iparent)) {
		errno = ENOTDIR;
		return NULL;
	}

	struct pmemfile_dir *dir = &iparent->file_data.dir;

	while (dir != NULL) {
		for (uint32_t i = 0; i < dir->num_elements; ++i) {
			struct pmemfile_dirent *d = &dir->dirents[i];

			if (TOID_EQUALS(d->inode, child->inode))
				return d;
		}

		dir = D_RW(dir->next);
	}

	errno = ENOENT;
	return NULL;
}

/*
 * vinode_lookup_dirent -- looks up file name in passed directory
 *
 * Takes reference on found inode. Caller must hold reference to parent inode.
 * Does not need transaction.
 */
struct pmemfile_vinode *
vinode_lookup_dirent(PMEMfilepool *pfp, struct pmemfile_vinode *parent,
		const char *name)
{
	LOG(LDBG, "parent 0x%lx ppath %s name %s", parent->inode.oid.off,
			pmfi_path(parent), name);

	if (name[0] == 0)
		return NULL;

	struct pmemfile_vinode *vinode = NULL;

	util_rwlock_rdlock(&parent->rwlock);

	if (strcmp(name, "..") == 0) {
		vinode = parent->parent;
		if (!vinode)
			vinode = parent;
		vinode_ref(pfp, vinode);
		goto end;
	}

	struct pmemfile_dirent *dirent =
		vinode_lookup_dirent_by_name_locked(pfp, parent, name);
	if (dirent) {
		bool parent_refed = false;
		vinode = inode_ref(pfp, dirent->inode, parent, &parent_refed,
				name);

		if (!vinode && parent_refed)
			vinode_unref_tx(pfp, parent);
	}

end:
	util_rwlock_unlock(&parent->rwlock);

	return vinode;
}

/*
 * vinode_unlink_dirent -- removes dirent from directory
 *
 * Must be called in transaction. Caller must have exclusive access to parent
 * inode, eg by locking parent in WRITE mode.
 */
void
vinode_unlink_dirent(PMEMfilepool *pfp, struct pmemfile_vinode *parent,
		const char *name, struct pmemfile_vinode *volatile *vinode,
		volatile bool *parent_refed, bool abort_on_ENOENT)
{
	LOG(LDBG, "parent 0x%lx ppath %s name %s", parent->inode.oid.off,
			pmfi_path(parent), name);

	struct pmemfile_dirent *dirent =
			vinode_lookup_dirent_by_name_locked(pfp, parent, name);
	if (!dirent) {
		if (errno == ENOENT && !abort_on_ENOENT)
			return;
		pmemobj_tx_abort(errno);
	}

	TOID(struct pmemfile_inode) tinode = dirent->inode;
	struct pmemfile_inode *inode = D_RW(tinode);

	if (inode_is_dir(inode))
		pmemobj_tx_abort(EISDIR);

	*vinode = inode_ref(pfp, tinode, parent, parent_refed, NULL);
	rwlock_tx_wlock(&(*vinode)->rwlock);

	ASSERT(inode->nlink > 0);

	TX_ADD_FIELD(tinode, nlink);
	pmemobj_tx_add_range_direct(dirent, sizeof(dirent->inode) + 1);

	struct pmemfile_time tm;
	file_get_time(&tm);

	if (--inode->nlink == 0)
		vinode_orphan(pfp, *vinode);
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

	dirent->name[0] = '\0';
	dirent->inode = TOID_NULL(struct pmemfile_inode);
}

#define DIRENT_ID_MASK 0xffffffffULL

#define DIR_ID(offset) ((offset) >> 32)
#define DIRENT_ID(offset) ((offset) & DIRENT_ID_MASK)

/*
 * file_seek_dir - translates between file->offset and dir/dirent
 *
 * returns 0 on EOF
 * returns !0 on successful translation
 */
static int
file_seek_dir(PMEMfile *file, struct pmemfile_inode *inode,
		struct pmemfile_dir **dir, unsigned *dirent)
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
	*dirent = DIRENT_ID(file->offset);

	while (*dirent >= (*dir)->num_elements) {
		if (TOID_IS_NULL((*dir)->next))
			return 0;

		*dirent -= (*dir)->num_elements;
		*dir = D_RW((*dir)->next);

		file->dir_pos.dir = *dir;
		file->dir_pos.dir_id++;
	}

	file->offset = ((size_t)file->dir_pos.dir_id) << 32 | *dirent;

	return 1;
}

static int
file_getdents(PMEMfilepool *pfp, PMEMfile *file, struct pmemfile_inode *inode,
		struct linux_dirent *dirp, unsigned count)
{
	struct pmemfile_dir *dir;
	unsigned dirent_id;

	if (file_seek_dir(file, inode, &dir, &dirent_id) == 0)
		return 0;

	int read1 = 0;
	char *data = (void *)dirp;

	while (true) {
		if (dirent_id >= dir->num_elements) {
			if (TOID_IS_NULL(dir->next))
				break;

			dir = D_RW(dir->next);
			file->dir_pos.dir = dir;
			file->dir_pos.dir_id++;
			dirent_id = 0;
			file->offset = ((size_t)file->dir_pos.dir_id) << 32 | 0;
		}

		struct pmemfile_dirent *dirent = &dir->dirents[dirent_id];
		if (TOID_IS_NULL(dirent->inode)) {
			++dirent_id;
			++file->offset;
			continue;
		}

		size_t namelen = strlen(dirent->name);
		unsigned short slen = (unsigned short)
				(8 + 8 + 2 + namelen + 1 + 1);
		uint64_t next_off = file->offset + 1;
		if (dirent_id + 1 >= dir->num_elements)
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

		if (inode_is_regular_file(D_RO(dirent->inode)))
			*data = DT_REG;
		else
			*data = DT_DIR;
		data++;

		read1 += slen;

		++dirent_id;
		++file->offset;
	}

	return read1;
}

int
pmemfile_getdents(PMEMfilepool *pfp, PMEMfile *file,
			struct linux_dirent *dirp, unsigned count)
{
	struct pmemfile_vinode *vinode = file->vinode;

	if (!vinode_is_dir(vinode)) {
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
	unsigned dirent_id;

	if (file_seek_dir(file, inode, &dir, &dirent_id) == 0)
		return 0;

	int read1 = 0;
	char *data = (void *)dirp;

	while (true) {
		if (dirent_id >= dir->num_elements) {
			if (TOID_IS_NULL(dir->next))
				break;

			dir = D_RW(dir->next);
			file->dir_pos.dir = dir;
			file->dir_pos.dir_id++;
			dirent_id = 0;
			file->offset = ((size_t)file->dir_pos.dir_id) << 32 | 0;
		}

		struct pmemfile_dirent *dirent = &dir->dirents[dirent_id];
		if (TOID_IS_NULL(dirent->inode)) {
			++dirent_id;
			++file->offset;
			continue;
		}

		size_t namelen = strlen(dirent->name);
		unsigned short slen = (unsigned short)
				(8 + 8 + 2 + 1 + namelen + 1);
		uint64_t next_off = file->offset + 1;
		if (dirent_id + 1 >= dir->num_elements)
			next_off = ((next_off >> 32) + 1) << 32;

		if (count < slen)
			break;

		memcpy(data, &dirent->inode.oid.off, 8);
		data += 8;

		memcpy(data, &next_off, 8);
		data += 8;

		memcpy(data, &slen, 2);
		data += 2;

		if (inode_is_regular_file(D_RO(dirent->inode)))
			*data = DT_REG;
		else
			*data = DT_DIR;
		data++;

		memcpy(data, dirent->name, namelen + 1);
		data += namelen + 1;

		read1 += slen;

		++dirent_id;
		++file->offset;
	}

	return read1;
}

int
pmemfile_getdents64(PMEMfilepool *pfp, PMEMfile *file,
			struct linux_dirent64 *dirp, unsigned count)
{
	struct pmemfile_vinode *vinode = file->vinode;

	if (!vinode_is_dir(vinode)) {
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
static void
_traverse_pathat(PMEMfilepool *pfp, struct pmemfile_vinode *parent,
		const char *path, bool get_parent,
		struct pmemfile_path_info *path_info)
{
	char tmp[PATH_MAX];
	vinode_ref(pfp, parent);
	struct pmemfile_vinode *prev_parent = NULL;

	memset(path_info, 0, sizeof(*path_info));

	while (1) {
		struct pmemfile_vinode *child;
		const char *lookup_path = path;
		const char *slash = strchr(lookup_path, '/');

		if (slash) {
			/* handle paths ending with / */
			const char *it = slash;
			while (*it == '/')
				it++;

			if (*it == 0) {
				strncpy(tmp, path,
					(uintptr_t)slash - (uintptr_t)path);
				tmp[slash - path] = 0;
				lookup_path = tmp;
				slash = NULL;
			}
		}

		if (slash == NULL) {
			child = vinode_lookup_dirent(pfp, parent, lookup_path);
			if (child) {
				if (get_parent) {
					path_info->parent = parent;
					path_info->name = path;
				} else
					vinode_unref_tx(pfp, parent);

				while (path[0])
					path++;

				if (prev_parent)
					vinode_unref_tx(pfp, prev_parent);

				path_info->vinode = child;
			} else {
				if (get_parent)
					path_info->parent = prev_parent;
				else if (prev_parent)
					vinode_unref_tx(pfp, prev_parent);

				path_info->vinode = parent;
			}

			path_info->remaining = path;
			return;
		}

		strncpy(tmp, path, (uintptr_t)slash - (uintptr_t)path);
		tmp[slash - path] = 0;

		child = vinode_lookup_dirent(pfp, parent, tmp);
		if (!child) {
			if (get_parent)
				path_info->parent = prev_parent;
			else if (prev_parent)
				vinode_unref_tx(pfp, prev_parent);

			path_info->remaining = path;
			path_info->vinode = parent;
			return;
		}

		if (prev_parent)
			vinode_unref_tx(pfp, prev_parent);
		prev_parent = parent;

		parent = child;
		path = slash + 1;

		while (path[0] == '/')
			path++;
	}
}

void
traverse_path(PMEMfilepool *pfp, struct pmemfile_vinode *parent,
		const char *path, bool get_parent,
		struct pmemfile_path_info *path_info)
{
	if (path[0] == '/') {
		while (path[0] == '/')
			path++;
		parent = pfp->root;
	}

	_traverse_pathat(pfp, parent, path, get_parent, path_info);
}

static int
_pmemfile_mkdirat(PMEMfilepool *pfp, struct pmemfile_vinode *dir,
		const char *path, mode_t mode)
{
	struct pmemfile_path_info info;
	traverse_path(pfp, dir, path, false, &info);

	if (info.remaining[0] == 0) {
		vinode_unref_tx(pfp, info.vinode);
		errno = EEXIST;
		return -1;
	}

	struct pmemfile_vinode *parent = info.vinode;

	if (!vinode_is_dir(parent)) {
		vinode_unref_tx(pfp, parent);
		errno = ENOTDIR;
		return -1;
	}

	char *sanitized = NULL;
	const char *slash = strchr(info.remaining, '/');
	if (slash) {
		const char *after_slash = slash + 1;

		while (*after_slash == '/')
			after_slash++;

		if (*after_slash != 0) {
			vinode_unref_tx(pfp, parent);
			errno = ENOENT;
			return -1;
		}

		sanitized = strndup(info.remaining,
			(uintptr_t)slash - (uintptr_t)info.remaining);
	}

	int error = 0;
	int txerrno = 0;
	struct pmemfile_vinode *child = NULL;
	volatile bool parent_refed = false;

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		rwlock_tx_wlock(&parent->rwlock);

		child = vinode_new_dir(pfp, parent, sanitized ? sanitized :
				info.remaining, mode, true, &parent_refed);

		rwlock_tx_unlock_on_commit(&parent->rwlock);
	} TX_ONABORT {
		error = 1;
		txerrno = errno;
	} TX_END

	if (sanitized)
		free(sanitized);

	if (!error)
		vinode_unref_tx(pfp, child);

	vinode_unref_tx(pfp, parent);

	if (error) {
		if (parent_refed)
			vinode_unref_tx(pfp, parent);

		errno = txerrno;
		return -1;
	}

	return 0;
}

int
pmemfile_mkdirat(PMEMfilepool *pfp, PMEMfile *dir, const char *path,
		mode_t mode)
{
	struct pmemfile_vinode *at;
	bool at_unref;

	if (!path) {
		errno = ENOENT;
		return -1;
	}

	at = pool_get_dir_for_path(pfp, dir, path, &at_unref);

	int ret = _pmemfile_mkdirat(pfp, at, path, mode);

	int oerrno;
	if (ret)
		oerrno = errno;

	if (at_unref)
		vinode_unref_tx(pfp, at);

	if (ret)
		errno = oerrno;

	return ret;
}

int
pmemfile_mkdir(PMEMfilepool *pfp, const char *path, mode_t mode)
{
	return pmemfile_mkdirat(pfp, PMEMFILE_AT_CWD, path, mode);
}

int
_pmemfile_rmdirat(PMEMfilepool *pfp, struct pmemfile_vinode *dir,
		const char *path)
{
	struct pmemfile_path_info info;
	traverse_path(pfp, dir, path, true, &info);

	if (info.remaining[0] != 0) {
		vinode_unref_tx(pfp, info.vinode);
		if (info.parent)
			vinode_unref_tx(pfp, info.parent);
		errno = ENOENT;
		return -1;
	}

	struct pmemfile_vinode *vdir = info.vinode;
	if (!vinode_is_dir(vdir)) {
		vinode_unref_tx(pfp, vdir);
		if (info.parent)
			vinode_unref_tx(pfp, info.parent);
		errno = ENOTDIR;
		return -1;
	}

	int error = 0;
	int txerrno = 0;
	struct pmemfile_vinode *vparent = info.parent;
	struct pmemfile_inode *iparent = D_RW(vparent->inode);

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		if (vparent == vdir)
			pmemobj_tx_abort(EINVAL);

		rwlock_tx_wlock(&vparent->rwlock);
		rwlock_tx_wlock(&vdir->rwlock);

		struct pmemfile_dirent *dirent =
			vinode_lookup_dirent_by_vinode_locked(pfp, vparent,
					vdir);

		if (!dirent) {
			LOG(LUSR, "rmdir race");
			pmemobj_tx_abort(EBUSY);
		}

		struct pmemfile_inode *idir = D_RW(vdir->inode);
		struct pmemfile_dir *ddir = &idir->file_data.dir;
		if (!TOID_IS_NULL(ddir->next)) {
			LOG(LUSR, "directory %s not empty", path);
			pmemobj_tx_abort(ENOTEMPTY);
		}


		struct pmemfile_dirent *dirdot = &ddir->dirents[0];
		struct pmemfile_dirent *dirdotdot = &ddir->dirents[1];

		ASSERTeq(strcmp(dirdot->name, "."), 0);
		ASSERT(TOID_EQUALS(dirdot->inode, vdir->inode));

		ASSERTeq(strcmp(dirdotdot->name, ".."), 0);
		ASSERT(TOID_EQUALS(dirdotdot->inode, vparent->inode));


		for (uint32_t i = 2; i < ddir->num_elements; ++i) {
			struct pmemfile_dirent *d = &ddir->dirents[i];

			if (!TOID_IS_NULL(d->inode)) {
				LOG(LUSR, "directory %s not empty", path);
				pmemobj_tx_abort(ENOTEMPTY);
			}
		}

		pmemobj_tx_add_range_direct(dirdot, sizeof(dirdot->inode) + 1);
		dirdot->name[0] = '\0';
		dirdot->inode = TOID_NULL(struct pmemfile_inode);

		pmemobj_tx_add_range_direct(dirdotdot,
				sizeof(dirdotdot->inode) + 1);
		dirdotdot->name[0] = '\0';
		dirdotdot->inode = TOID_NULL(struct pmemfile_inode);

		ASSERTeq(idir->nlink, 2);
		TX_ADD_DIRECT(&idir->nlink);
		idir->nlink = 0;

		pmemobj_tx_add_range_direct(dirent, sizeof(dirent->inode) + 1);
		dirent->name[0] = '\0';
		dirent->inode = TOID_NULL(struct pmemfile_inode);

		TX_ADD_DIRECT(&iparent->nlink);
		iparent->nlink--;

		vinode_orphan(pfp, vdir);

		struct pmemfile_time tm;
		file_get_time(&tm);

		/*
		 * From "stat" man page:
		 * "The field st_ctime is changed by writing or by setting inode
		 * information (i.e., owner, group, link count, mode, etc.)."
		 */
		TX_SET_DIRECT(iparent, ctime, tm);

		/*
		 * From "stat" man page:
		 * "st_mtime of a directory is changed by the creation
		 * or deletion of files in that directory."
		 */
		TX_SET_DIRECT(iparent, mtime, tm);

		rwlock_tx_unlock_on_commit(&vdir->rwlock);
		rwlock_tx_unlock_on_commit(&vparent->rwlock);
	} TX_ONABORT {
		error = 1;
		txerrno = errno;
	} TX_END

	vinode_unref_tx(pfp, vparent);

	vinode_unref_tx(pfp, vdir);

	if (error) {
		errno = txerrno;
		return -1;
	}

	return 0;
}

int
pmemfile_rmdir(PMEMfilepool *pfp, const char *path)
{
	struct pmemfile_vinode *at;
	bool at_unref;

	if (!path) {
		errno = ENOENT;
		return -1;
	}

	at = pool_get_dir_for_path(pfp, PMEMFILE_AT_CWD, path, &at_unref);

	int ret = _pmemfile_rmdirat(pfp, at, path);

	int oerrno;
	if (ret)
		oerrno = errno;

	if (at_unref)
		vinode_unref_tx(pfp, at);

	if (ret)
		errno = oerrno;

	return ret;
}

static int
_pmemfile_chdir(PMEMfilepool *pfp, struct pmemfile_vinode *dir)
{
	if (!vinode_is_dir(dir)) {
		vinode_unref_tx(pfp, dir);
		errno = ENOTDIR;
		return -1;
	}

	util_rwlock_wrlock(&pfp->cwd_rwlock);
	struct pmemfile_vinode *old_cwd = pfp->cwd;
	pfp->cwd = dir;
	util_rwlock_unlock(&pfp->cwd_rwlock);
	vinode_unref_tx(pfp, old_cwd);

	return 0;
}

int
pmemfile_chdir(PMEMfilepool *pfp, const char *path)
{
	struct pmemfile_path_info info;
	struct pmemfile_vinode *at;
	int ret = 0;
	int err = 0;
	bool at_unref;

	if (!path) {
		errno = ENOENT;
		return -1;
	}

	at = pool_get_dir_for_path(pfp, PMEMFILE_AT_CWD, path, &at_unref);

	traverse_path(pfp, at, path, false, &info);

	if (info.remaining[0] != 0) {
		vinode_unref_tx(pfp, info.vinode);
		err = ENOENT;
		ret = -1;
		goto end;
	}

	ret = _pmemfile_chdir(pfp, info.vinode);
	if (ret)
		err = errno;

end:
	if (at_unref)
		vinode_unref_tx(pfp, at);
	if (err)
		errno = err;

	return ret;
}

int
pmemfile_fchdir(PMEMfilepool *pfp, PMEMfile *dir)
{
	vinode_ref(pfp, dir->vinode);

	return _pmemfile_chdir(pfp, dir->vinode);
}

/*
 * pool_get_cwd -- returns current working directory
 *
 * Takes a reference on returned vinode.
 */
struct pmemfile_vinode *
pool_get_cwd(PMEMfilepool *pfp)
{
	struct pmemfile_vinode *cwd;

	util_rwlock_rdlock(&pfp->cwd_rwlock);
	cwd = pfp->cwd;
	vinode_ref(pfp, cwd);
	util_rwlock_unlock(&pfp->cwd_rwlock);

	return cwd;
}

struct pmemfile_vinode *
pool_get_dir_for_path(PMEMfilepool *pfp, PMEMfile *dir, const char *path,
		bool *unref)
{
	*unref = false;
	if (path[0] == '/')
		return NULL;

	if (dir == PMEMFILE_AT_CWD) {
		*unref = true;
		return pool_get_cwd(pfp);
	}

	return dir->vinode;
}

static char *
_pmemfile_get_dir_path(PMEMfilepool *pfp, struct pmemfile_vinode *vinode,
		char *buf, size_t size)
{
	struct pmemfile_vinode *parent = NULL, *child = vinode;

	if (buf && size == 0) {
		vinode_unref_tx(pfp, child);

		errno = EINVAL;
		return NULL;
	}

	util_rwlock_rdlock(&child->rwlock);
	if (child->orphaned.arr) {
		util_rwlock_unlock(&child->rwlock);
		vinode_unref_tx(pfp, child);

		errno = ENOENT;
		return NULL;
	}
	parent = child->parent;
	if (parent)
		vinode_ref(pfp, parent);
	util_rwlock_unlock(&child->rwlock);

	if (size == 0)
		size = PATH_MAX;

	if (!buf) {
		buf = malloc(size);
		if (!buf) {
			int oerrno = errno;
			vinode_unref_tx(pfp, child);
			errno = oerrno;
			return NULL;
		}
	}

	char *curpos = buf + size;
	*(--curpos) = 0;

	if (!parent) {
		if (curpos - 1 < buf)
			goto range_err;
		*(--curpos) = '/';
	}

	while (parent) {
		util_rwlock_rdlock(&parent->rwlock);
		struct pmemfile_dirent *dirent =
				vinode_lookup_dirent_by_vinode_locked(pfp,
						parent, child);
		size_t len = strlen(dirent->name);
		if (curpos - len - 1 < buf) {
			util_rwlock_unlock(&parent->rwlock);
			vinode_unref_tx(pfp, parent);
			goto range_err;
		}
		curpos -= len;
		memcpy(curpos, dirent->name, len);

		*(--curpos) = '/';

		struct pmemfile_vinode *grandparent = parent->parent;
		if (grandparent)
			vinode_ref(pfp, grandparent);
		util_rwlock_unlock(&parent->rwlock);

		vinode_unref_tx(pfp, child);

		child = parent;
		parent = grandparent;
	}

	vinode_unref_tx(pfp, child);

	memmove(buf, curpos, (uintptr_t)(buf + size - curpos));

	return buf;

range_err:
	vinode_unref_tx(pfp, child);
	errno = ERANGE;
	return NULL;
}

char *
pmemfile_get_dir_path(PMEMfilepool *pfp, PMEMfile *dir, char *buf, size_t size)
{
	struct pmemfile_vinode *vdir;

	if (dir == PMEMFILE_AT_CWD) {
		vdir = pool_get_cwd(pfp);
	} else {
		vdir = dir->vinode;
		vinode_ref(pfp, vdir);
	}

	return _pmemfile_get_dir_path(pfp, vdir, buf, size);
}

char *
pmemfile_getcwd(PMEMfilepool *pfp, char *buf, size_t size)
{
	return _pmemfile_get_dir_path(pfp, pool_get_cwd(pfp), buf, size);
}
