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
 * inode.c -- inode operations
 */

#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>

#include "callbacks.h"
#include "data.h"
#include "dir.h"
#include "inode.h"
#include "inode_array.h"
#include "internal.h"
#include "locks.h"
#include "out.h"
#include "sys_util.h"

/*
 * pmfi_path -- returns one of the full paths inode can be reached on
 *
 * Only for debugging.
 */
const char *
pmfi_path(struct pmemfile_vinode *vinode)
{
#ifdef DEBUG
	if (!vinode)
		return NULL;
	if (!vinode->path)
		LOG(LTRC, "0x%lx: no vinode->path", vinode->inode.oid.off);
	return vinode->path;
#else
	return NULL;
#endif
}

/*
 * file_inode_ref -- increases inode runtime reference counter
 *
 * Does not need transaction.
 */
void
file_inode_ref(PMEMfilepool *pfp, struct pmemfile_vinode *vinode)
{
	__sync_fetch_and_add(&vinode->ref, 1);
}

#define BUCKET_SIZE 2

struct inode_map_bucket {
	struct {
		TOID(struct pmemfile_inode) pinode;
		struct pmemfile_vinode *vinode;
	} arr[BUCKET_SIZE];
};

/* First impl */
struct pmemfile_inode_map {
	pthread_rwlock_t rwlock;
	uint32_t hash_fun_a; /* fun */
	uint32_t hash_fun_b; /* even more fun */
	uint64_t hash_fun_p; /* party! */

	size_t sz;
	struct inode_map_bucket *buckets;
	size_t inodes;
};

/*
 * inode_map_rand_params -- randomizes coefficients of the hashmap function
 */
static void
inode_map_rand_params(struct pmemfile_inode_map *c)
{
	// XXX use independent random pool
	do {
		c->hash_fun_a = (uint32_t)rand();
	} while (c->hash_fun_a == 0);
	c->hash_fun_b = (uint32_t)rand();
}

/*
 * file_inode_map_alloc -- allocates inode hashmap
 */
struct pmemfile_inode_map *
file_inode_map_alloc()
{
	struct pmemfile_inode_map *c = Zalloc(sizeof(*c));

	c->sz = 2;
	c->buckets = Zalloc(c->sz * sizeof(c->buckets[0]));

	inode_map_rand_params(c);
	c->hash_fun_p = 32212254719ULL;

	util_rwlock_init(&c->rwlock);

	return c;
}

/*
 * file_inode_map_free -- destroys inode hashmap
 */
void
file_inode_map_free(struct pmemfile_inode_map *c)
{
	for (unsigned i = 0; i < c->sz; ++i) {
		struct inode_map_bucket *bucket = &c->buckets[i];

		for (unsigned j = 0; j < BUCKET_SIZE; ++j)
			if (bucket->arr[j].vinode)
				FATAL("memory leak");
	}

	util_rwlock_destroy(&c->rwlock);
	Free(c->buckets);
	Free(c);
}

/*
 * file_hash_inode -- returns hash value of the inode
 */
static inline size_t
file_hash_inode(struct pmemfile_inode_map *c, TOID(struct pmemfile_inode) inode)
{
	return (c->hash_fun_a * inode.oid.off + c->hash_fun_b) % c->hash_fun_p;
}

/*
 * file_inode_map_rebuild -- rebuilds the whole inode hashmap
 */
static bool
file_inode_map_rebuild(struct pmemfile_inode_map *c, size_t new_sz)
{
	struct inode_map_bucket *new_buckets =
			Zalloc(new_sz * sizeof(new_buckets[0]));
	size_t idx;

	for (size_t i = 0; i < c->sz; ++i) {
		struct inode_map_bucket *b = &c->buckets[i];

		for (unsigned j = 0; j < BUCKET_SIZE; ++j) {
			if (b->arr[j].pinode.oid.off == 0)
				continue;

			idx = file_hash_inode(c, b->arr[j].pinode) % new_sz;
			struct inode_map_bucket *newbucket = &new_buckets[idx];
			unsigned k;
			for (k = 0; k < BUCKET_SIZE; ++k) {
				if (newbucket->arr[k].pinode.oid.off == 0) {
					newbucket->arr[k] = b->arr[j];
					break;
				}
			}

			if (k == BUCKET_SIZE) {
				Free(new_buckets);
				return false;
			}
		}
	}

	Free(c->buckets);
	c->sz = new_sz;
	c->buckets = new_buckets;

	return true;
}

/*
 * file_vinode_unregister_locked -- removes vinode from  inode map
 */
static void
file_vinode_unregister_locked(PMEMfilepool *pfp,
		struct pmemfile_vinode *vinode)
{
	struct pmemfile_inode_map *c = pfp->inode_map;

	size_t idx = file_hash_inode(c, vinode->inode) % c->sz;
	struct inode_map_bucket *b = &c->buckets[idx];
	unsigned j;
	for (j = 0; j < BUCKET_SIZE; ++j) {
		if (b->arr[j].vinode == vinode) {
			memset(&b->arr[j], 0, sizeof(b->arr[j]));
			break;
		}
	}

	if (j == BUCKET_SIZE)
		FATAL("vinode not found");

	c->inodes--;

	file_destroy_data_state(vinode);

#ifdef DEBUG
	/* "path" field is defined only in DEBUG builds */
	Free(vinode->path);
#endif
	util_rwlock_destroy(&vinode->rwlock);
	Free(vinode);
}

/*
 * _file_vinode_get -- (internal) deals with vinode life time related to inode
 */
static struct pmemfile_vinode *
_file_vinode_get(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) inode, bool ref, bool is_new)
{
	struct pmemfile_inode_map *c = pfp->inode_map;
	int tx = 0;

	util_rwlock_rdlock(&c->rwlock);
	size_t idx = file_hash_inode(c, inode) % c->sz;

	struct inode_map_bucket *b = &c->buckets[idx];
	struct pmemfile_vinode *vinode;
	for (unsigned j = 0; j < BUCKET_SIZE; ++j) {
		if (TOID_EQUALS(b->arr[j].pinode, inode)) {
			vinode = b->arr[j].vinode;
			goto end;
		}
	}
	util_rwlock_unlock(&c->rwlock);

	if (is_new) {
		rwlock_tx_wlock(&c->rwlock);
		tx = 1;
	} else
		util_rwlock_wrlock(&c->rwlock);

	/* recalculate slot, someone could rebuild the hashmap */
	idx = file_hash_inode(c, inode) % c->sz;

	/* check again */
	b = &c->buckets[idx];
	unsigned empty_slot = UINT32_MAX;
	for (unsigned j = 0; j < BUCKET_SIZE; ++j) {
		if (TOID_EQUALS(b->arr[j].pinode, inode)) {
			vinode = b->arr[j].vinode;
			goto end;
		}
		if (empty_slot == UINT32_MAX && b->arr[j].pinode.oid.off == 0)
			empty_slot = j;
	}

	int tries = 0;
	while (empty_slot == UINT32_MAX) {
		size_t new_sz = c->sz;

		do {
			if (c->inodes > 2 * new_sz || tries == 2) {
				new_sz *= 2;
				tries = 0;
			} else {
				inode_map_rand_params(c);
				tries++;
			}
		} while (!file_inode_map_rebuild(c, new_sz));

		idx = file_hash_inode(c, inode) % c->sz;
		b = &c->buckets[idx];

		for (unsigned j = 0; j < BUCKET_SIZE; ++j) {
			if (b->arr[j].pinode.oid.off == 0) {
				empty_slot = j;
				break;
			}
		}
	}

	vinode = Zalloc(sizeof(*vinode));
	if (!vinode)
		goto end;

	util_rwlock_init(&vinode->rwlock);
	vinode->inode = inode;

	b->arr[empty_slot].pinode = inode;
	b->arr[empty_slot].vinode = vinode;
	c->inodes++;

	if (is_new)
		cb_push_front(TX_STAGE_ONABORT,
			(cb_basic)file_vinode_unregister_locked,
			vinode);

end:
	if (ref)
		__sync_fetch_and_add(&vinode->ref, 1);
	if (is_new && tx)
		rwlock_tx_unlock_on_commit(&c->rwlock);
	else
		util_rwlock_unlock(&c->rwlock);

	return vinode;
}

/*
 * file_vinode_get -- returns volatile inode
 *
 * May be called outside of transaction.
 */
struct pmemfile_vinode *
file_vinode_get(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) inode, bool ref)
{
	return _file_vinode_get(pfp, inode, ref, false);
}

/*
 * file_vinode_ref_new -- increases inode reference counter
 *
 * Assumes inode was allocated in the same transaction.
 * Return volatile inode.
 */
struct pmemfile_vinode *
file_vinode_ref_new(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) inode)
{
	return _file_vinode_get(pfp, inode, true, true);
}

/*
 * file_vinode_ref -- increases inode reference counter
 *
 * Assumes inode was not allocated in the same transaction.
 * Return volatile inode.
 */
struct pmemfile_vinode *
file_vinode_ref(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) inode)
{
	return _file_vinode_get(pfp, inode, true, false);
}

/*
 * file_vinode_unref -- decreases inode reference counter
 *
 * Must be called in transaction.
 */
void
file_vinode_unref(PMEMfilepool *pfp, struct pmemfile_vinode *vinode)
{
	struct pmemfile_inode_map *c = pfp->inode_map;

	rwlock_tx_wlock(&c->rwlock);
	if (__sync_sub_and_fetch(&vinode->ref, 1) > 0) {
		rwlock_tx_unlock_on_commit(&c->rwlock);
		return;
	}

	if (D_RO(vinode->inode)->nlink == 0) {
		file_inode_array_unregister(pfp, vinode->orphaned.arr,
				vinode->orphaned.idx);

		file_inode_free(pfp, vinode->inode);
	}

	cb_push_back(TX_STAGE_ONCOMMIT,
		(cb_basic)file_vinode_unregister_locked,
		vinode);

	rwlock_tx_unlock_on_commit(&c->rwlock);
}

/*
 * file_vinode_unref_tx -- decreases inode reference counter
 */
void
file_vinode_unref_tx(PMEMfilepool *pfp, struct pmemfile_vinode *vinode)
{
	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_NONE);

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		file_vinode_unref(pfp, vinode);
	} TX_ONABORT {
		FATAL("!");
	} TX_END
}

/*
 * file_get_time -- sets *t to current time
 */
void
file_get_time(struct pmemfile_time *t)
{
	struct timespec tm;
	if (clock_gettime(CLOCK_REALTIME, &tm)) {
		ERR("!clock_gettime");
		pmemobj_tx_abort(errno);
	}
	t->sec = tm.tv_sec;
	t->nsec = tm.tv_nsec;
}

/*
 * file_inode_alloc -- allocates inode
 *
 * Must be called in transaction.
 */
struct pmemfile_vinode *
file_inode_alloc(PMEMfilepool *pfp, uint64_t flags, struct pmemfile_time *t)
{
	LOG(LDBG, "flags 0x%lx", flags);

	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	TOID(struct pmemfile_inode) tinode = TX_ZNEW(struct pmemfile_inode);
	struct pmemfile_inode *inode = D_RW(tinode);

	file_get_time(t);

	inode->flags = flags;
	inode->ctime = *t;
	inode->mtime = *t;
	inode->atime = *t;
	inode->nlink = 0;
	inode->uid = geteuid();
	inode->gid = getegid();

	if (_file_is_regular_file(inode))
		inode->file_data.blocks.length =
				(sizeof(inode->file_data) -
				sizeof(inode->file_data.blocks)) /
				sizeof(struct pmemfile_block);
	else if (_file_is_dir(inode)) {
		inode->file_data.dir.num_elements =
				(sizeof(inode->file_data) -
				sizeof(inode->file_data.dir)) /
				sizeof(struct pmemfile_dirent);
		inode->size = sizeof(inode->file_data);
	}

	return file_vinode_ref_new(pfp, tinode);
}

/*
 * file_register_orphaned_inode -- (internal) register specified inode in
 * orphaned_inodes array
 */
void
file_register_orphaned_inode(PMEMfilepool *pfp, struct pmemfile_vinode *vinode)
{
	LOG(LDBG, "inode 0x%lx path %s", vinode->inode.oid.off,
			pmfi_path(vinode));

	ASSERTeq(vinode->orphaned.arr, NULL);

	rwlock_tx_wlock(&pfp->rwlock);

	TOID(struct pmemfile_inode_array) orphaned =
			D_RW(pfp->super)->orphaned_inodes;
	if (TOID_IS_NULL(orphaned)) {
		orphaned = TX_ZNEW(struct pmemfile_inode_array);
		TX_SET(pfp->super, orphaned_inodes, orphaned);
	}

	file_inode_array_add(pfp, orphaned, vinode,
			&vinode->orphaned.arr, &vinode->orphaned.idx);

	rwlock_tx_unlock_on_commit(&pfp->rwlock);
}

/*
 * file_assert_no_dentries -- checks that directory has no entries
 */
static void
file_assert_no_dentries(struct pmemfile_dir *dir)
{
	for (uint32_t i = 0; i < dir->num_elements; ++i)
		if (dir->dentries[i].inode.oid.off)
			FATAL("Trying to free non-empty directory");
}

/*
 * file_inode_free -- frees inode
 *
 * Must be called in transaction.
 */
void
file_inode_free(PMEMfilepool *pfp, TOID(struct pmemfile_inode) tinode)
{
	LOG(LDBG, "inode 0x%lx", tinode.oid.off);

	struct pmemfile_inode *inode = D_RW(tinode);
	if (_file_is_dir(inode)) {
		struct pmemfile_dir *dir = &inode->file_data.dir;
		TOID(struct pmemfile_dir) tdir = TOID_NULL(struct pmemfile_dir);

		while (dir != NULL) {
			/* should have been catched earlier */
			file_assert_no_dentries(dir);

			TOID(struct pmemfile_dir) next = dir->next;
			if (!TOID_IS_NULL(tdir))
				TX_FREE(tdir);
			tdir = next;
			dir = D_RW(tdir);
		}
	} else if (_file_is_regular_file(inode)) {
		struct pmemfile_block_array *arr = &inode->file_data.blocks;
		TOID(struct pmemfile_block_array) tarr =
				TOID_NULL(struct pmemfile_block_array);

		while (arr != NULL) {
			for (unsigned i = 0; i < arr->length; ++i)
				TX_FREE(arr->blocks[i].data);

			TOID(struct pmemfile_block_array) next = arr->next;
			if (!TOID_IS_NULL(tarr))
				TX_FREE(tarr);
			tarr = next;
			arr = D_RW(tarr);
		}
	} else {
		FATAL("unknown inode type 0x%lx", inode->flags);
	}
	TX_FREE(tinode);
}

static inline struct timespec
pmemfile_time_to_timespec(const struct pmemfile_time *t)
{
	struct timespec tm;
	tm.tv_sec = t->sec;
	tm.tv_nsec = t->nsec;
	return tm;
}

/*
 * file_fill_stat
 */
static int
file_fill_stat(struct pmemfile_vinode *vinode, struct stat *buf)
{
	struct pmemfile_inode *inode = D_RW(vinode->inode);

	memset(buf, 0, sizeof(*buf));
	buf->st_dev = vinode->inode.oid.pool_uuid_lo;
	buf->st_ino = vinode->inode.oid.off;
	buf->st_mode = inode->flags & (S_IFMT | S_IRWXU | S_IRWXG | S_IRWXO);
	buf->st_nlink = inode->nlink;
	buf->st_uid = inode->uid;
	buf->st_gid = inode->gid;
	buf->st_rdev = 0;
	if ((off_t)inode->size < 0) {
		errno = EOVERFLOW;
		return -1;
	}
	buf->st_size = (off_t)inode->size;
	buf->st_blksize = 1;
	if ((blkcnt_t)inode->size < 0) {
		errno = EOVERFLOW;
		return -1;
	}
	buf->st_blocks = (blkcnt_t)inode->size;
	buf->st_atim = pmemfile_time_to_timespec(&inode->atime);
	buf->st_ctim = pmemfile_time_to_timespec(&inode->ctime);
	buf->st_mtim = pmemfile_time_to_timespec(&inode->mtime);

	return 0;
}

/*
 * pmemfile_stat
 */
int
pmemfile_stat(PMEMfilepool *pfp, const char *path, struct stat *buf)
{
	if (!path) {
		errno = ENOENT;
		return -1;
	}

	if (!buf) {
		errno = EFAULT;
		return -1;
	}

	LOG(LDBG, "path %s", path);

	path = file_check_pathname(path);
	if (!path)
		return -1;

	struct pmemfile_vinode *parent_vinode = pfp->root;

	file_inode_ref(pfp, parent_vinode);

	struct pmemfile_vinode *vinode =
			file_lookup_dentry(pfp, parent_vinode, path);

	if (!vinode) {
		int oerrno = errno;
		file_vinode_unref_tx(pfp, parent_vinode);
		errno = oerrno;
		return -1;
	}

	int ret = file_fill_stat(vinode, buf);

	file_vinode_unref_tx(pfp, vinode);
	file_vinode_unref_tx(pfp, parent_vinode);

	return ret;
}

/*
 * pmemfile_fstat
 */
int
pmemfile_fstat(PMEMfilepool *pfp, PMEMfile *file, struct stat *buf)
{
	if (!file) {
		errno = EBADF;
		return -1;
	}

	if (!buf) {
		errno = EFAULT;
		return -1;
	}

	return file_fill_stat(file->vinode, buf);
}

/*
 * pmemfile_lstat
 */
int
pmemfile_lstat(PMEMfilepool *pfp, const char *path, struct stat *buf)
{
	// XXX because symlinks are not yet implemented
	return pmemfile_stat(pfp, path, buf);
}
