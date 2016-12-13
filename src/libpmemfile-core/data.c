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

#include <fcntl.h>
#include <limits.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include "callbacks.h"
#include "data.h"
#include "inode.h"
#include "internal.h"
#include "locks.h"
#include "out.h"
#include "pool.h"
#include "sys_util.h"
#include "util.h"
#include "valgrind_internal.h"
#include "../libpmemobj/ctree.h"

#define min(a, b) ((a) < (b) ? (a) : (b))

/*
 * block_cache_insert_block -- inserts block into the tree
 */
static void
block_cache_insert_block(struct ctree *c, struct pmemfile_block *block)
{
	ctree_insert_unlocked(c, block->offset, (uintptr_t)block);
}

/*
 * file_rebuild_block_tree -- rebuilds runtime tree of blocks
 */
static void
vinode_rebuild_block_tree(struct pmemfile_vinode *vinode)
{
	struct ctree *c = ctree_new();
	if (!c)
		return;
	struct pmemfile_inode *inode = D_RW(vinode->inode);
	struct pmemfile_block_array *block_array = &inode->file_data.blocks;

	while (block_array != NULL) {
		for (unsigned i = 0; i < block_array->length; ++i) {
			struct pmemfile_block *block = &block_array->blocks[i];

			if (block->size == 0)
				break;
			block_cache_insert_block(c, block);
		}

		block_array = D_RW(block_array->next);
	}

	vinode->blocks = c;
}

static struct pmemfile_block *
find_block(struct pmemfile_vinode *vinode, uint64_t *off)
{
	return (void *)(uintptr_t)ctree_find_le_unlocked(vinode->blocks, off);
}

/*
 * file_destroy_data_state -- destroys file state related to data
 */
void
vinode_destroy_data_state(struct pmemfile_vinode *vinode)
{
	struct ctree *blocks = vinode->blocks;
	if (!blocks)
		return;

	uint64_t key = UINT64_MAX;
	struct pmemfile_block *block;
	while ((block = find_block(vinode, &key))) {
		uint64_t k = ctree_remove_unlocked(blocks, key, 1);
		ASSERTeq(k, key);

		key = UINT64_MAX;
	}

	ctree_delete(blocks);
	vinode->blocks = NULL;

	memset(&vinode->first_free_block, 0, sizeof(vinode->first_free_block));
}

/*
 * file_allocate_block -- allocates new block
 */
static void
file_allocate_block(PMEMfilepool *pfp,
		PMEMfile *file,
		struct pmemfile_inode *inode,
		struct pmemfile_pos *pos,
		struct pmemfile_block *prev,
		struct pmemfile_block *block,
		size_t count)
{
	size_t sz = min(pmemfile_core_block_size, 1U << 31);
	if (sz == 0) {
		if (count <= 4096)
			sz = 16 * 1024;
		else if (count <= 64 * 1024)
			sz = 256 * 1024;
		else if (count <= 1024 * 1024)
			sz = 4 * 1024 * 1024;
		else
			sz = 64 * 1024 * 1024;
	} else if (sz == 1) {
		if (count <= 4096)
			sz = 4096;
		else if (count >= 64 * 1024 * 1024)
			sz = 64 * 1024 * 1024;
		else {
			/* next power of 2 */
			sz = count - 1;
			sz |= sz >> 1;
			sz |= sz >> 2;
			sz |= sz >> 4;
			sz |= sz >> 8;
			sz |= sz >> 16;
			sz++;
		}
	}

	TX_ADD_DIRECT(block);
	block->data = TX_XALLOC(char, sz, POBJ_XALLOC_NO_FLUSH);
	sz = pmemobj_alloc_usable_size(block->data.oid);

#ifdef DEBUG
	/* poison block data */
	void *data = D_RW(block->data);
	VALGRIND_ADD_TO_TX(data, sz);
	pmemobj_memset_persist(pfp->pop, data, 0x66, sz);
	VALGRIND_REMOVE_FROM_TX(data, sz);
	VALGRIND_DO_MAKE_MEM_UNDEFINED(data, sz);
#endif

	ASSERT(sz <= UINT32_MAX);
	block->size = (uint32_t)sz;

	block->flags = 0;
	block->offset = pos->global_offset;
	block->next = TOID_NULL(struct pmemfile_block);

	if (prev) {
		TX_ADD_DIRECT(&prev->next);
		prev->next = (TOID(struct pmemfile_block))pmemobj_oid(block);
	}

	TX_ADD_DIRECT(&inode->last_block_fill);
	inode->last_block_fill = 0;

	block_cache_insert_block(file->vinode->blocks, block);
}

/*
 * inode_extend_block_meta_data -- updates metadata of the current block
 */
static void
inode_extend_block_meta_data(struct pmemfile_inode *inode, uint32_t len)
{
	TX_ADD_FIELD_DIRECT(inode, last_block_fill);
	inode->last_block_fill += len;

	TX_ADD_FIELD_DIRECT(inode, size);
	inode->size += len;
}

/*
 * inode_zero_extend_block -- extends current block with zeroes
 */
static void
inode_zero_extend_block(PMEMfilepool *pfp,
		struct pmemfile_inode *inode,
		struct pmemfile_block *block,
		uint32_t len)
{
	char *addr = D_RW(block->data) + inode->last_block_fill;

	/*
	 * We can safely skip tx_add_range, because there's no user visible
	 * data at this address.
	 */
	VALGRIND_ADD_TO_TX(addr, len);
	pmemobj_memset_persist(pfp->pop, addr, 0, len);
	VALGRIND_REMOVE_FROM_TX(addr, len);

	inode_extend_block_meta_data(inode, len);
}

static struct pmemfile_block *
get_free_block(struct pmemfile_vinode *vinode, bool extend)
{
	struct pmemfile_inode *inode = D_RW(vinode->inode);
	struct block_info *binfo = &vinode->first_free_block;
	struct pmemfile_block_array *prev = NULL;

	if (!binfo->arr) {
		binfo->arr = &inode->file_data.blocks;
		binfo->idx = 0;
	}

	while (binfo->arr) {
		while (binfo->idx < binfo->arr->length) {
			if (binfo->arr->blocks[binfo->idx].size == 0)
				return &binfo->arr->blocks[binfo->idx++];
			binfo->idx++;
		}

		if (!extend && TOID_IS_NULL(binfo->arr->next))
			return NULL;

		prev = binfo->arr;
		binfo->arr = D_RW(binfo->arr->next);
		binfo->idx = 0;
	}

	TOID(struct pmemfile_block_array) next =
			TX_ZALLOC(struct pmemfile_block_array, 4096);
	D_RW(next)->length = (uint32_t)
			((pmemobj_alloc_usable_size(next.oid) -
			sizeof(struct pmemfile_block_array)) /
			sizeof(struct pmemfile_block));
	TX_SET_DIRECT(prev, next, next);

	binfo->arr = D_RW(next);
	binfo->idx = 0;

	return &binfo->arr->blocks[binfo->idx];
}

/*
 * is_last_block -- returns true when specified block is the last one in a file
 */
static bool
is_last_block(struct pmemfile_block *block)
{
	return TOID_IS_NULL(block->next);
}

/*
 * pos_reset -- resets position pointer to the beginning of the file
 */
static void
pos_reset(struct pmemfile_pos *pos, struct pmemfile_vinode *vinode)
{
	uint64_t off = 0;
	pos->block = find_block(vinode, &off);
	if (!pos->block)
		pos->block = get_free_block(vinode, false);
	pos->block_offset = 0;

	pos->global_offset = 0;
}

/*
 * file_seek_within_block -- changes current position pointer within block
 *
 * returns number of bytes
 */
static size_t
file_seek_within_block(PMEMfilepool *pfp,
		PMEMfile *file,
		struct pmemfile_inode *inode,
		struct pmemfile_pos *pos,
		struct pmemfile_block *prev,
		struct pmemfile_block *block,
		size_t offset_left,
		bool extend)
{
	if (block->size == 0) {
		if (extend)
			file_allocate_block(pfp, file, inode, pos, prev, block,
					offset_left);
		else
			return 0;
	}

	uint32_t max_off;
	bool is_last = is_last_block(block);
	if (is_last) {
		ASSERT(inode->last_block_fill >= pos->block_offset);
		max_off = inode->last_block_fill - pos->block_offset;
	} else {
		ASSERT(block->size >= pos->block_offset);
		max_off = block->size - pos->block_offset;
	}
	uint32_t seeked = (uint32_t)min((size_t)max_off, offset_left);

	pos->block_offset += seeked;
	pos->global_offset += seeked;
	offset_left -= seeked;

	if (offset_left == 0 || pos->block_offset == block->size)
		return seeked;

	ASSERTeq(is_last, true);

	uint32_t extended = (uint32_t)min(offset_left,
			(size_t)(block->size - inode->last_block_fill));
	inode_zero_extend_block(pfp, inode, block, extended);

	pos->block_offset += extended;
	pos->global_offset += extended;

	return seeked + extended;
}

/*
 * file_write_within_block -- writes data to current block
 */
static size_t
file_write_within_block(PMEMfilepool *pfp,
		PMEMfile *file,
		struct pmemfile_inode *inode,
		struct pmemfile_pos *pos,
		struct pmemfile_block *prev,
		struct pmemfile_block *block,
		const void *buf,
		size_t count_left)
{
	if (block->size == 0)
		file_allocate_block(pfp, file, inode, pos, prev, block,
				count_left);

	/* How much data should we write to this block? */
	uint32_t len = (uint32_t)min((size_t)block->size - pos->block_offset,
			count_left);

	void *dest = D_RW(block->data) + pos->block_offset;
	VALGRIND_ADD_TO_TX(dest, len);
	pmemobj_memcpy_persist(pfp->pop, dest, buf, len);
	VALGRIND_REMOVE_FROM_TX(dest, len);

	bool is_last = is_last_block(block);
	if (is_last) {
		/*
		 * If new size is beyond the block used size, then we
		 * have to update all metadata.
		 */
		if (pos->block_offset + len > inode->last_block_fill) {
			uint32_t new_used = pos->block_offset + len
					- inode->last_block_fill;

			inode_extend_block_meta_data(inode, new_used);
		}

		ASSERT(inode->last_block_fill <= block->size);
	}

	pos->block_offset += len;
	pos->global_offset += len;

	return len;
}

/*
 * inode_read_from_block -- reads data from current block
 */
static size_t
inode_read_from_block(struct pmemfile_inode *inode,
		struct pmemfile_pos *pos,
		struct pmemfile_block *block,
		void *buf,
		size_t count_left,
		bool is_last)
{
	if (block->size == 0)
		return 0;

	/* How much data should we read from this block? */
	uint32_t len = is_last ? inode->last_block_fill : block->size;
	len = (uint32_t)min((size_t)len - pos->block_offset, count_left);

	if (len == 0)
		return 0;

	memcpy(buf, D_RW(block->data) + pos->block_offset, len);

	pos->block_offset += len;
	pos->global_offset += len;

	return len;
}

/*
 * file_write -- writes to file
 */
static void
file_write(PMEMfilepool *pfp, PMEMfile *file, struct pmemfile_inode *inode,
		const char *buf, size_t count)
{
	/* Position cache. */
	struct pmemfile_pos *pos = &file->pos;
	struct pmemfile_vinode *vinode = file->vinode;

	if (pos->block == NULL)
		pos_reset(pos, vinode);

	if (file->offset != pos->global_offset) {
		size_t block_start = pos->global_offset - pos->block_offset;
		size_t off = file->offset;

		if (off < block_start ||
				off >= block_start + pos->block->size) {
			struct pmemfile_block *block = find_block(vinode, &off);
			if (block) {
				pos->block = block;
				pos->block_offset = 0;
				pos->global_offset = off;
			}
		}
	}

	if (file->offset < pos->global_offset) {
		if (file->offset >= pos->global_offset - pos->block_offset) {
			pos->global_offset -= pos->block_offset;
			pos->block_offset = 0;
		} else {
			pos_reset(pos, vinode);
		}
	}

	size_t offset_left = file->offset - pos->global_offset;

	/*
	 * Find the position, possibly extending and/or zeroing unused space.
	 */

	struct pmemfile_block *prev = NULL;
	while (offset_left > 0) {
		struct pmemfile_block *block = pos->block;

		size_t seeked = file_seek_within_block(pfp, file, inode, pos,
				prev, block, offset_left, true);

		ASSERT(seeked <= offset_left);

		offset_left -= seeked;

		if (offset_left > 0) {
			pos->block = D_RW(block->next);
			pos->block_offset = 0;

			if (pos->block == NULL)
				pos->block = get_free_block(vinode, false);
		}

		prev = block;
	}

	/*
	 * Now file->offset matches cached position in file->pos.
	 *
	 * Let's write the requested data starting from current position.
	 */

	size_t count_left = count;
	while (count_left > 0) {
		struct pmemfile_block *block = pos->block;

		size_t written = file_write_within_block(pfp, file, inode, pos,
				prev, block, buf, count_left);

		ASSERT(written <= count_left);

		buf += written;
		count_left -= written;

		if (count_left > 0) {
			pos->block = D_RW(block->next);
			pos->block_offset = 0;

			if (pos->block == NULL)
				pos->block = get_free_block(vinode, true);
		}

		prev = block;
	}
}

/*
 * pmemfile_write -- writes to file
 */
ssize_t
pmemfile_write(PMEMfilepool *pfp, PMEMfile *file, const void *buf, size_t count)
{
	LOG(LDBG, "file %p buf %p count %zu", file, buf, count);

	if (!vinode_is_regular_file(file->vinode)) {
		errno = EINVAL;
		return -1;
	}

	if (!(file->flags & PFILE_WRITE)) {
		errno = EBADF;
		return -1;
	}

	if ((ssize_t)count < 0)
		count = SSIZE_MAX;

	int error = 0;

	struct pmemfile_vinode *vinode = file->vinode;
	struct pmemfile_inode *inode = D_RW(vinode->inode);
	struct pmemfile_pos pos;

	util_mutex_lock(&file->mutex);

	memcpy(&pos, &file->pos, sizeof(pos));
	int txerrno = 0;

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		rwlock_tx_wlock(&vinode->rwlock);

		if (!vinode->blocks)
			vinode_rebuild_block_tree(vinode);

		if (file->flags & PFILE_APPEND)
			file->offset = D_RO(vinode->inode)->size;

		file_write(pfp, file, inode, buf, count);

		if (count > 0) {
			struct pmemfile_time tm;
			file_get_time(&tm);
			TX_SET(vinode->inode, mtime, tm);
		}

		rwlock_tx_unlock_on_commit(&vinode->rwlock);
	} TX_ONABORT {
		error = 1;
		txerrno = errno;
		memcpy(&file->pos, &pos, sizeof(pos));
	} TX_ONCOMMIT {
		file->offset += count;
	} TX_END

	util_mutex_unlock(&file->mutex);

	if (error) {
		errno = txerrno;
		return -1;
	}

	return (ssize_t)count;
}

/*
 * file_sync_off -- sanitizes file position (internal) WRT file offset (set by
 * user)
 */
static bool
file_sync_off(PMEMfile *file, struct pmemfile_pos *pos,
		struct pmemfile_inode *inode)
{
	size_t block_start = pos->global_offset - pos->block_offset;
	size_t off = file->offset;

	if (off < block_start || off >= block_start + pos->block->size) {
		struct pmemfile_block *block = find_block(file->vinode, &off);
		if (!block)
			return false;

		pos->block = block;
		pos->block_offset = 0;
		pos->global_offset = off;
	}

	if (file->offset < pos->global_offset) {
		if (file->offset >= pos->global_offset - pos->block_offset) {
			pos->global_offset -= pos->block_offset;
			pos->block_offset = 0;
		} else {
			pos_reset(pos, file->vinode);

			if (pos->block == NULL)
				return false;
		}
	}

	return true;
}

/*
 * file_read -- reads file
 */
static size_t
file_read(PMEMfilepool *pfp, PMEMfile *file, struct pmemfile_inode *inode,
		char *buf, size_t count)
{
	struct pmemfile_pos *pos = &file->pos;

	if (unlikely(pos->block == NULL)) {
		pos_reset(pos, file->vinode);

		if (pos->block == NULL)
			return 0;
	}

	/*
	 * Find the position, without modifying file.
	 */

	if (file->offset != pos->global_offset)
		if (!file_sync_off(file, pos, inode))
			return 0;

	size_t offset_left = file->offset - pos->global_offset;

	struct pmemfile_block *prev = NULL;
	while (offset_left > 0) {
		struct pmemfile_block *block = pos->block;

		size_t seeked = file_seek_within_block(pfp, file, inode, pos,
				prev, block, offset_left, false);
		bool is_last = is_last_block(pos->block);

		if (seeked == 0) {
			uint32_t used = is_last ?
					inode->last_block_fill : block->size;
			bool block_boundary =
					block->size > 0 &&
					used == block->size &&
					used == pos->block_offset;
			if (!block_boundary)
				return 0;
		}

		ASSERT(seeked <= offset_left);

		offset_left -= seeked;

		if (offset_left > 0) {
			/* EOF? */
			if (is_last && inode->last_block_fill != block->size)
				return 0;

			pos->block = D_RW(block->next);
			pos->block_offset = 0;

			if (pos->block == NULL) {
				/* EOF */
				return 0;
			}
		}

		prev = block;
	}

	/*
	 * Now file->offset matches cached position in file->pos.
	 *
	 * Let's read the requested data starting from current position.
	 */

	size_t bytes_read = 0;
	size_t count_left = count;
	while (count_left > 0) {
		struct pmemfile_block *block = pos->block;
		bool is_last = is_last_block(pos->block);

		size_t read1 = inode_read_from_block(inode, pos, block, buf,
				count_left, is_last);

		if (read1 == 0) {
			uint32_t used = is_last ?
					inode->last_block_fill : block->size;
			bool block_boundary =
					block->size > 0 &&
					used == block->size &&
					used == pos->block_offset;
			if (!block_boundary)
				break;
		}

		ASSERT(read1 <= count_left);

		buf += read1;
		bytes_read += read1;
		count_left -= read1;

		if (count_left > 0) {
			/* EOF? */
			if (is_last && inode->last_block_fill != block->size)
				break;

			pos->block = D_RW(block->next);
			pos->block_offset = 0;

			if (pos->block == NULL) {
				/* EOF */
				return 0;
			}
		}
	}

	return bytes_read;
}

static int
time_cmp(const struct pmemfile_time *t1, const struct pmemfile_time *t2)
{
	if (t1->sec < t2->sec)
		return -1;
	if (t1->sec > t2->sec)
		return 1;
	if (t1->nsec < t2->nsec)
		return -1;
	if (t1->nsec > t2->nsec)
		return 1;
	return 0;
}

/*
 * pmemfile_read -- reads file
 */
ssize_t
pmemfile_read(PMEMfilepool *pfp, PMEMfile *file, void *buf, size_t count)
{
	LOG(LDBG, "file %p buf %p count %zu", file, buf, count);

	if (!vinode_is_regular_file(file->vinode)) {
		errno = EINVAL;
		return -1;
	}

	if (!(file->flags & PFILE_READ)) {
		errno = EBADF;
		return -1;
	}

	if ((ssize_t)count < 0)
		count = SSIZE_MAX;

	size_t bytes_read = 0;

	struct pmemfile_vinode *vinode = file->vinode;
	struct pmemfile_inode *inode = D_RW(vinode->inode);

	util_mutex_lock(&file->mutex);

	if (!vinode->blocks) {
		util_rwlock_wrlock(&vinode->rwlock);
		vinode_rebuild_block_tree(vinode);
	} else {
		util_rwlock_rdlock(&vinode->rwlock);
	}

	bytes_read = file_read(pfp, file, inode, buf, count);

	bool update_atime = !(file->flags & PFILE_NOATIME);
	struct pmemfile_time tm;

	if (update_atime) {
		struct pmemfile_time tm1d;
		file_get_time(&tm);
		tm1d.nsec = tm.nsec;
		tm1d.sec = tm.sec - 86400;

		/* relatime */
		update_atime =	time_cmp(&inode->atime, &tm1d) < 0 ||
				time_cmp(&inode->atime, &inode->ctime) < 0 ||
				time_cmp(&inode->atime, &inode->mtime) < 0;
	}

	util_rwlock_unlock(&vinode->rwlock);

	if (update_atime) {
		TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
			rwlock_tx_wlock(&vinode->rwlock);

			TX_SET(vinode->inode, atime, tm);

			rwlock_tx_unlock_on_commit(&vinode->rwlock);
		} TX_ONABORT {
			LOG(LINF, "can not update inode atime");
		} TX_END
	}


	file->offset += bytes_read;

	util_mutex_unlock(&file->mutex);

	ASSERT(bytes_read <= count);
	return (ssize_t)bytes_read;
}

/*
 * pmemfile_lseek64 -- changes file current offset
 */
off64_t
pmemfile_lseek64(PMEMfilepool *pfp, PMEMfile *file, off64_t offset, int whence)
{
	LOG(LDBG, "file %p offset %lu whence %d", file, offset, whence);

	if (vinode_is_dir(file->vinode)) {
		if (whence == SEEK_END) {
			errno = EINVAL;
			return -1;
		}
	} else if (vinode_is_regular_file(file->vinode)) {
		/* Nothing to do for now */
	} else {
		errno = EINVAL;
		return -1;
	}

	struct pmemfile_vinode *vinode = file->vinode;
	struct pmemfile_inode *inode = D_RW(vinode->inode);
	off64_t ret;
	int new_errno = EINVAL;

	util_mutex_lock(&file->mutex);

	switch (whence) {
		case SEEK_SET:
			ret = offset;
			break;
		case SEEK_CUR:
			ret = (off64_t)file->offset + offset;
			break;
		case SEEK_END:
			util_rwlock_rdlock(&vinode->rwlock);
			ret = (off64_t)inode->size + offset;
			util_rwlock_unlock(&vinode->rwlock);
			break;
		case SEEK_DATA:
			util_rwlock_rdlock(&vinode->rwlock);
			if (offset < 0) {
				ret = 0;
			} else if ((uint64_t)offset > inode->size) {
				ret = -1;
				new_errno = ENXIO;
			} else {
				ret = offset;
			}
			util_rwlock_unlock(&vinode->rwlock);
			break;
		case SEEK_HOLE:
			util_rwlock_rdlock(&vinode->rwlock);
			if ((uint64_t)offset > inode->size) {
				ret = -1;
				new_errno = ENXIO;
			} else {
				ret = (off64_t)inode->size;
			}
			util_rwlock_unlock(&vinode->rwlock);
			break;
		default:
			ret = -1;
			break;
	}

	if (ret < 0) {
		ret = -1;
		errno = new_errno;
	} else {
		if (file->offset != (size_t)ret)
			LOG(LDBG, "off diff: old %lu != new %lu", file->offset,
					(size_t)ret);
		file->offset = (size_t)ret;
	}

	util_mutex_unlock(&file->mutex);

	return ret;
}

/*
 * pmemfile_lseek -- changes file current offset
 */
off_t
pmemfile_lseek(PMEMfilepool *pfp, PMEMfile *file, off_t offset, int whence)
{
	return pmemfile_lseek64(pfp, file, offset, whence);
}

/*
 * file_truncate -- changes file size to 0
 */
void
vinode_truncate(struct pmemfile_vinode *vinode)
{
	struct pmemfile_block_array *arr =
			&D_RW(vinode->inode)->file_data.blocks;
	TOID(struct pmemfile_block_array) tarr = arr->next;

	TX_MEMSET(&arr->next, 0, sizeof(arr->next));
	for (uint32_t i = 0; i < arr->length; ++i) {
		if (arr->blocks[i].size > 0) {
			TX_FREE(arr->blocks[i].data);
			continue;
		}

		TX_MEMSET(&arr->blocks[0], 0, sizeof(arr->blocks[0]) * i);
		break;
	}

	arr = D_RW(tarr);
	while (arr != NULL) {
		for (uint32_t i = 0; i < arr->length; ++i)
			TX_FREE(arr->blocks[i].data);

		TOID(struct pmemfile_block_array) next = arr->next;
		TX_FREE(tarr);
		tarr = next;
		arr = D_RW(tarr);
	}

	struct pmemfile_inode *inode = D_RW(vinode->inode);

	TX_ADD_DIRECT(&inode->size);
	inode->size = 0;

	TX_ADD_DIRECT(&inode->last_block_fill);
	inode->last_block_fill = 0;

	struct pmemfile_time tm;
	file_get_time(&tm);
	TX_SET(vinode->inode, mtime, tm);

	// we don't have to rollback destroy of data state on abort, because
	// it will be rebuilded when it's needed
	vinode_destroy_data_state(vinode);
}
