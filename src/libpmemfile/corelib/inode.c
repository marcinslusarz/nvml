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

#include "callbacks.h"
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

static void
inode_map_rand_params(struct pmemfile_inode_map *c)
{
	// XXX use independent random pool
	do {
		c->hash_fun_a = (uint32_t)rand();
	} while (c->hash_fun_a == 0);
	c->hash_fun_b = (uint32_t)rand();
}

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

static inline size_t
file_hash_inode(struct pmemfile_inode_map *c, TOID(struct pmemfile_inode) inode)
{
	return (c->hash_fun_a * inode.oid.off + c->hash_fun_b) % c->hash_fun_p;
}

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

#ifdef DEBUG
	/* "path" field is defined only in DEBUG builds */
	Free(vinode->path);
#endif
	util_rwlock_destroy(&vinode->rwlock);
	Free(vinode);
}

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

struct pmemfile_vinode *
file_vinode_get(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) inode, bool ref)
{
	return _file_vinode_get(pfp, inode, ref, false);
}

struct pmemfile_vinode *
file_vinode_ref_new(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) inode)
{
	return _file_vinode_get(pfp, inode, true, true);
}

struct pmemfile_vinode *
file_vinode_ref(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) inode)
{
	return _file_vinode_get(pfp, inode, true, false);
}

void
file_vinode_unref(PMEMfilepool *pfp, struct pmemfile_vinode *vinode)
{
	struct pmemfile_inode_map *c = pfp->inode_map;

	rwlock_tx_wlock(&c->rwlock);
	if (__sync_sub_and_fetch(&vinode->ref, 1) > 0) {
		rwlock_tx_unlock_on_commit(&c->rwlock);
		return;
	}

	struct pmemfile_inode_array *cur = vinode->opened.arr;
	if (cur)
		file_inode_array_unregister(pfp, cur, vinode->opened.idx);

	if (D_RO(vinode->inode)->nlink == 0)
		file_inode_free(pfp, vinode->inode);

	cb_push_back(TX_STAGE_ONCOMMIT,
		(cb_basic)file_vinode_unregister_locked,
		vinode);

	rwlock_tx_unlock_on_commit(&c->rwlock);
}

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
 * file_inode_alloc -- allocates inode
 *
 * Must be called in transaction.
 */
struct pmemfile_vinode *
file_inode_alloc(PMEMfilepool *pfp, uint64_t flags, struct timespec *t)
{
	LOG(LDBG, "flags 0x%lx", flags);

	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	TOID(struct pmemfile_inode) inode = TX_ZNEW(struct pmemfile_inode);

	if (clock_gettime(CLOCK_REALTIME, t)) {
		ERR("!clock_gettime");
		pmemobj_tx_abort(errno);
	}

	D_RW(inode)->flags = flags;
	D_RW(inode)->ctime = *t;
	D_RW(inode)->mtime = *t;
	D_RW(inode)->atime = *t;
	D_RW(inode)->nlink = 0;

	return file_vinode_ref_new(pfp, inode);
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
		TOID(struct pmemfile_dir) dir = inode->file_data.dir;

		while (!TOID_IS_NULL(dir)) {
			/* should have been catched earlier */
			if (D_RW(dir)->used != 0)
				FATAL("Trying to free non-empty directory");

			TOID(struct pmemfile_dir) next = D_RW(dir)->next;
			TX_FREE(dir);
			dir = next;
		}
	} else if (_file_is_regular_file(inode)) {
		TOID(struct pmemfile_block_array) block_arr =
				inode->file_data.blocks;
		while (!TOID_IS_NULL(block_arr)) {
			struct pmemfile_block_array *arr = D_RW(block_arr);

			for (unsigned i = 0; i < arr->blocks_allocated; ++i)
				TX_FREE(arr->blocks[i].data);

			TOID(struct pmemfile_block_array) next = arr->next;
			TX_FREE(block_arr);
			block_arr = next;
		}

		if (!TOID_IS_NULL(inode->file_data.blocks))
			TX_SET_DIRECT(inode, file_data.blocks,
					TOID_NULL(struct pmemfile_block_array));
	} else {
		FATAL("unknown inode type 0x%lx", inode->flags);
	}
	TX_FREE(tinode);
}
