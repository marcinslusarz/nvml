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
#include "../../libpmemobj/ctree.h"

#define min(a, b) ((a) < (b) ? (a) : (b))

struct file_block_info {
	struct pmemfile_block_array *arr;
	unsigned id;
};

static void
file_insert_block_to_cache(struct ctree *c,
		struct pmemfile_block_array *block_array,
		unsigned id,
		size_t off)
{
	if (!pmemfile_optimized_tree_walk)
		return;
	struct file_block_info *info = Malloc(sizeof(*info));
	info->arr = block_array;
	info->id = id;

	ctree_insert_unlocked(c, off, (uintptr_t)info);
}

static void
file_rebuild_block_tree(PMEMfile *file)
{
	if (!pmemfile_optimized_tree_walk)
		return;
	struct ctree *c = ctree_new();
	if (!c)
		return;
	struct pmemfile_inode *inode = D_RW(file->vinode->inode);
	struct pmemfile_block_array *block_array =
			D_RW(inode->file_data.blocks);
	size_t off = 0;

	while (block_array != NULL) {
		for (unsigned i = 0; i < block_array->blocks_allocated; ++i) {
			struct pmemfile_block *block = &block_array->blocks[i];

			file_insert_block_to_cache(c, block_array, i, off);

			off += block->allocated;
		}

		block_array = D_RW(block_array->next);
	}

	file->blocks = c;
}

void
file_destroy_data_state(PMEMfile *file)
{
	if (!pmemfile_optimized_tree_walk || !file->blocks)
		return;

	uint64_t key = UINT64_MAX;
	struct file_block_info *info;
	while ((info = (void *)(uintptr_t)ctree_find_le_unlocked(file->blocks,
			&key))) {
		Free(info);
		uint64_t k = ctree_remove_unlocked(file->blocks, key, 1);
		ASSERTeq(k, key);

		key = UINT64_MAX;
	}

	ctree_delete(file->blocks);
}

static void
file_reset_cache(PMEMfile *file, struct pmemfile_inode *inode,
		struct pmemfile_pos *pos, bool alloc)
{
	pos->block_array = D_RW(inode->file_data.blocks);
	if (pos->block_array == NULL && alloc) {
		TX_SET_DIRECT(inode, file_data.blocks,
				TX_ZNEW(struct pmemfile_block_array));
		pos->block_array = D_RW(inode->file_data.blocks);
	}
	pos->block_id = 0;
	pos->block_offset = 0;

	pos->global_offset = 0;
}

static void
file_allocate_block(PMEMfile *file, struct pmemfile_pos *pos,
		struct pmemfile_block *block, size_t count)
{
	struct pmemfile_block_array *block_array = pos->block_array;

	size_t sz = pmemfile_core_block_size;
	if (sz == 0) {
		if (count < 4096)
			sz = 16 * 1024;
		else if (count < 64 * 1024)
			sz = 256 * 1024;
		else if (count < 1024 * 1024)
			sz = 4 * 1024 * 1024;
		else
			sz = 64 * 1024 * 1024;
	}

	TX_ADD_DIRECT(block);
	block->used = 0;
	block->data = TX_XALLOC(char, sz, POBJ_XALLOC_NO_FLUSH);
	block->allocated = pmemobj_alloc_usable_size(block->data.oid);

	TX_ADD_FIELD_DIRECT(block_array, bytes_allocated);
	block_array->bytes_allocated += block->allocated;

	TX_ADD_FIELD_DIRECT(block_array, blocks_allocated);
	block_array->blocks_allocated++;

	file_insert_block_to_cache(file->blocks, block_array,
			(unsigned)(block - &block_array->blocks[0]),
			pos->global_offset);
}

static void
file_extend_block_meta_data(struct pmemfile_inode *inode,
		struct pmemfile_block_array *block_array,
		struct pmemfile_block *block,
		size_t len)
{
	TX_ADD_FIELD_DIRECT(block, used);
	block->used += len;

	TX_ADD_FIELD_DIRECT(block_array, bytes_used);
	block_array->bytes_used += len;

	TX_ADD_FIELD_DIRECT(inode, size);
	inode->size += len;
}

static void
file_zero_extend_block(PMEMfilepool *pfp,
		struct pmemfile_inode *inode,
		struct pmemfile_block_array *block_array,
		struct pmemfile_block *block,
		size_t len)
{
	char *addr = D_RW(block->data) + block->used;

	/*
	 * We can safely skip tx_add_range, because there's no user visible
	 * data at this address.
	 */
	pmemobj_memset_persist(pfp->pop, addr, 0, len);

	file_extend_block_meta_data(inode, block_array, block, len);
}

static bool
file_next_block_array(struct pmemfile_pos *pos, bool extend)
{
	/* Transition to the next block array in block array list. */
	TOID(struct pmemfile_block_array) next = pos->block_array->next;

	if (TOID_IS_NULL(next)) {
		if (!extend)
			return false;

		next = TX_ZNEW(struct pmemfile_block_array);
		TX_SET_DIRECT(pos->block_array, next, next);
	}

	pos->block_array = D_RW(next);

	/* We changed the block array, so we have to reset block position. */
	pos->block_id = 0;
	pos->block_offset = 0;

	return true;
}

/*
 * file_move_within_block -- moves current position pointer within block
 *
 * returns how many bytes were skipped
 */
static size_t
file_move_within_block(PMEMfilepool *pfp,
		PMEMfile *file,
		struct pmemfile_inode *inode,
		struct pmemfile_pos *pos,
		struct pmemfile_block *block,
		size_t offset_left,
		bool extend)
{
	if (block->allocated == 0) {
		if (extend)
			file_allocate_block(file, pos, block, offset_left);
		else
			return 0;
	}

	/*
	 * Is anticipated position within the current block?
	 */
	if (pos->block_offset + offset_left < block->allocated) {
		/*
		 * Is anticipated position between the end of
		 * used space and the end of block?
		 */
		if (pos->block_offset + offset_left > block->used) {
			if (!extend)
				return 0;

			file_zero_extend_block(pfp, inode, pos->block_array,
					block, offset_left - block->used);

			ASSERT(block->used <= block->allocated);
		}

		pos->block_offset += offset_left;
		pos->global_offset += offset_left;

		ASSERTeq(pos->global_offset, file->offset);

		return offset_left;
	}

	/*
	 * Now we know offset lies in one of the consecutive blocks.
	 */


	/*
	 * Is there any space in current block that needs to be
	 * zeroed?
	 */
	if (block->used == block->allocated) {
		/*
		 * No. We can go to the next block.
		 */
		size_t sz = block->used - pos->block_offset;
		pos->global_offset += sz;

		return sz;
	}

	if (!extend)
		return 0;

	/*
	 * Yes. We have to zero the remaining part of the block.
	 */

	size_t len = block->allocated - block->used;
	file_zero_extend_block(pfp, inode, pos->block_array, block, len);

	pos->block_offset += len;
	pos->global_offset += len;

	ASSERTeq(block->used, block->allocated);

	return len;
}

static size_t
file_write_within_block(PMEMfilepool *pfp,
		PMEMfile *file,
		struct pmemfile_inode *inode,
		struct pmemfile_pos *pos,
		struct pmemfile_block *block,
		const void *buf,
		size_t count_left)
{
	if (block->allocated == 0)
		file_allocate_block(file, pos, block, count_left);

	/* How much data should we write to this block? */
	size_t len = min(block->allocated - pos->block_offset, count_left);

	/*
	 * Snapshot data between pos->block_offset and block->used.
	 * Everything above block->used is unused, so we don't
	 * have to restore it on abort.
	 */
	if (pmemfile_track_data && pos->block_offset < block->used) {
		size_t slen = min(len,
				block->used - pos->block_offset);
		if (pmemfile_replace_blocks && slen == block->allocated) {
			TX_FREE(block->data);
			block->data = TX_XALLOC(char, slen,
					PMEMOBJ_FLAG_NO_FLUSH);
		} else {
			pmemobj_tx_add_range_direct(
				D_RW(block->data) + pos->block_offset, slen);
		}
	}

	pmemobj_memcpy_persist(pfp->pop, D_RW(block->data) + pos->block_offset,
			buf, len);

	/*
	 * If new size is beyond the block used size, then we
	 * have to update all metadata.
	 */
	if (pos->block_offset + len > block->used) {
		size_t new_used = pos->block_offset + len - block->used;

		file_extend_block_meta_data(inode, pos->block_array, block,
				new_used);
	}

	ASSERT(block->used <= block->allocated);

	pos->block_offset += len;
	pos->global_offset += len;

	return len;
}

static size_t
file_read_from_block(struct pmemfile_pos *pos,
		struct pmemfile_block *block,
		void *buf,
		size_t count_left)
{
	if (block->allocated == 0)
		return 0;

	/* How much data should we read from this block? */
	size_t len = min(block->used - pos->block_offset, count_left);

	memcpy(buf, D_RW(block->data) + pos->block_offset, len);

	pos->block_offset += len;
	pos->global_offset += len;

	return len;
}

static size_t
file_skip_array_entry(struct pmemfile_pos *pos, size_t offset_left, bool extend)
{
	/* We can start only from the beginning of array */
	if (pos->block_id > 0 || pos->block_offset > 0)
		return 0;

	struct pmemfile_block_array *cur = pos->block_array;
	size_t offset = 0;

	/* Can we skip to the next list entry? */
	while (offset_left > 0 &&
			offset_left >= cur->bytes_used &&
			cur->bytes_allocated == cur->bytes_used &&
			cur->blocks_allocated == MAXNUMBLOCKS) {
		size_t tmp = cur->bytes_used;
		if (!file_next_block_array(pos, extend))
			break;
		offset += tmp;
		offset_left -= tmp;
		pos->global_offset += tmp;
		cur = pos->block_array;
	}

	return offset;
}

static void
file_write(PMEMfilepool *pfp, PMEMfile *file, struct pmemfile_inode *inode,
		const char *buf, size_t count)
{
	/* Position cache. */
	struct pmemfile_pos *pos = &file->pos;

	if (pos->block_array == NULL)
		file_reset_cache(file, inode, pos, true);

	if (pmemfile_optimized_tree_walk &&
			file->offset != pos->global_offset) {
		size_t block_start = pos->global_offset - pos->block_offset;
		size_t off = file->offset;

		if (off < block_start ||
				off >= block_start +
			pos->block_array->blocks[pos->block_id].allocated) {

			struct file_block_info *info = (void *)(uintptr_t)
				ctree_find_le_unlocked(file->blocks, &off);
			if (info) {
				pos->block_array = info->arr;
				pos->block_id = info->id;
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
			file_reset_cache(file, inode, pos, true);
		}
	}

	size_t offset_left = file->offset - pos->global_offset;

	/*
	 * Find the position, possibly extending and/or zeroing unused space.
	 */

	if (pmemfile_optimized_list_walk)
		offset_left -= file_skip_array_entry(pos, offset_left, true);

	while (offset_left > 0) {
		struct pmemfile_block *block =
				&pos->block_array->blocks[pos->block_id];

		size_t offset = file_move_within_block(pfp, file, inode, pos,
				block, offset_left, true);

		ASSERT(offset <= offset_left);

		offset_left -= offset;

		if (offset_left > 0) {
			pos->block_id++;
			pos->block_offset = 0;

			if (pos->block_id == MAXNUMBLOCKS) {
				file_next_block_array(pos, true);

				if (pmemfile_optimized_list_walk)
					offset_left -= file_skip_array_entry(
						pos, offset_left, true);
			}
		}
	}

	/*
	 * Now file->offset matches cached position in file->pos.
	 *
	 * Let's write the requested data starting from current position.
	 */

	size_t count_left = count;
	while (count_left > 0) {
		struct pmemfile_block *block =
				&pos->block_array->blocks[pos->block_id];

		size_t written = file_write_within_block(pfp, file, inode, pos,
				block, buf, count_left);

		ASSERT(written <= count_left);

		buf += written;
		count_left -= written;

		if (count_left > 0) {
			pos->block_id++;
			pos->block_offset = 0;

			if (pos->block_id == MAXNUMBLOCKS)
				file_next_block_array(pos, true);
		}
	}
}

ssize_t
pmemfile_write(PMEMfilepool *pfp, PMEMfile *file, const void *buf, size_t count)
{
	LOG(LDBG, "file %p buf %p count %zu", file, buf, count);

	if (!file_is_regular_file(file->vinode)) {
		errno = EINVAL;
		return -1;
	}

	if (!file->write) {
		errno = EBADF;
		return -1;
	}

	if ((ssize_t)count < 0) {
		errno = EFBIG;
		return -1;
	}

	int error = 0;

	struct pmemfile_vinode *vinode = file->vinode;
	struct pmemfile_inode *inode = D_RW(vinode->inode);
	struct pmemfile_pos pos;

	util_mutex_lock(&file->mutex);

	if (!file->blocks)
		file_rebuild_block_tree(file);

	memcpy(&pos, &file->pos, sizeof(pos));

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		rwlock_tx_wlock(&vinode->rwlock);

		file_write(pfp, file, inode, buf, count);

		rwlock_tx_unlock_on_commit(&vinode->rwlock);
	} TX_ONABORT {
		error = 1;
		memcpy(&file->pos, &pos, sizeof(pos));
	} TX_ONCOMMIT {
		file->offset += count;
	} TX_END

	util_mutex_unlock(&file->mutex);

	if (error)
		return -1;

	return (ssize_t)count;
}

static bool
file_sync_off(PMEMfile *file, struct pmemfile_pos *pos,
		struct pmemfile_inode *inode)
{
	if (pmemfile_optimized_tree_walk) {
		size_t block_start = pos->global_offset - pos->block_offset;
		size_t off = file->offset;

		if (off < block_start ||
				off >= block_start +
			pos->block_array->blocks[pos->block_id].allocated) {

			struct file_block_info *info = (void *)(uintptr_t)
				ctree_find_le_unlocked(file->blocks, &off);
			if (!info)
				return false;

			pos->block_array = info->arr;
			pos->block_id = info->id;
			pos->block_offset = 0;
			pos->global_offset = off;
		}
	}

	if (file->offset < pos->global_offset) {
		if (file->offset >= pos->global_offset - pos->block_offset) {
			pos->global_offset -= pos->block_offset;
			pos->block_offset = 0;
		} else {
			file_reset_cache(file, inode, pos, false);

			if (pos->block_array == NULL)
				return false;
		}
	}

	size_t offset_left = file->offset - pos->global_offset;

	if (offset_left > 0 && pmemfile_optimized_list_walk)
		file_skip_array_entry(pos, offset_left, false);

	return true;
}

static size_t
file_read(PMEMfilepool *pfp, PMEMfile *file, struct pmemfile_inode *inode,
		char *buf, size_t count)
{
	struct pmemfile_pos *pos = &file->pos;

	if (unlikely(pos->block_array == NULL)) {
		file_reset_cache(file, inode, pos, false);

		if (pos->block_array == NULL)
			return 0;
	}

	/*
	 * Find the position, without modifying file.
	 */

	if (file->offset != pos->global_offset)
		if (!file_sync_off(file, pos, inode))
			return 0;

	size_t offset_left = file->offset - pos->global_offset;

	while (offset_left > 0) {
		struct pmemfile_block *block =
				&pos->block_array->blocks[pos->block_id];

		size_t offset = file_move_within_block(pfp, file, inode, pos,
				block, offset_left, false);

		if (offset == 0) {
			bool block_boundary =
					block->allocated > 0 &&
					block->used == block->allocated &&
					block->used == pos->block_offset;
			if (!block_boundary)
				return 0;
		}

		ASSERT(offset <= offset_left);

		offset_left -= offset;

		if (offset_left > 0) {
			/* EOF? */
			if (block->used != block->allocated)
				return 0;

			pos->block_id++;
			pos->block_offset = 0;

			if (pos->block_id == MAXNUMBLOCKS) {
				if (!file_next_block_array(pos, false))
					/* EOF */
					return 0;
				if (pmemfile_optimized_list_walk)
					offset_left -= file_skip_array_entry(
						pos, offset_left, false);
			}
		}
	}

	/*
	 * Now file->offset matches cached position in file->pos.
	 *
	 * Let's read the requested data starting from current position.
	 */

	size_t bytes_read = 0;
	size_t count_left = count;
	while (count_left > 0) {
		struct pmemfile_block *block =
				&pos->block_array->blocks[pos->block_id];

		size_t read1 = file_read_from_block(pos, block, buf,
				count_left);

		if (read1 == 0) {
			bool block_boundary =
					block->allocated > 0 &&
					block->used == block->allocated &&
					block->used == pos->block_offset;
			if (!block_boundary)
				break;
		}

		ASSERT(read1 <= count_left);

		buf += read1;
		bytes_read += read1;
		count_left -= read1;

		if (count_left > 0) {
			/* EOF? */
			if (block->used != block->allocated)
				break;

			pos->block_id++;
			pos->block_offset = 0;

			if (pos->block_id == MAXNUMBLOCKS) {
				if (!file_next_block_array(pos, false))
					/* EOF */
					return 0;
			}
		}
	}

	return bytes_read;
}

ssize_t
pmemfile_read(PMEMfilepool *pfp, PMEMfile *file, void *buf, size_t count)
{
	LOG(LDBG, "file %p buf %p count %zu", file, buf, count);

	if (!file_is_regular_file(file->vinode)) {
		errno = EINVAL;
		return -1;
	}

	if (!file->read) {
		errno = EBADF;
		return -1;
	}

	if ((ssize_t)count < 0) {
		errno = EFBIG;
		return -1;
	}

	size_t bytes_read = 0;

	struct pmemfile_vinode *vinode = file->vinode;
	struct pmemfile_inode *inode = D_RW(vinode->inode);

	util_mutex_lock(&file->mutex);
	util_rwlock_rdlock(&vinode->rwlock);

	if (!file->blocks)
		file_rebuild_block_tree(file);

	bytes_read = file_read(pfp, file, inode, buf, count);

	file->offset += bytes_read;

	util_rwlock_unlock(&vinode->rwlock);
	util_mutex_unlock(&file->mutex);

	ASSERT(bytes_read <= count);
	return (ssize_t)bytes_read;
}

off64_t
pmemfile_lseek64(PMEMfilepool *pfp, PMEMfile *file, off64_t offset, int whence)
{
	LOG(LDBG, "file %p offset %lu whence %d", file, offset, whence);

	if (!file_is_regular_file(file->vinode)) {
		errno = EINVAL;
		return -1;
	}

	struct pmemfile_vinode *vinode = file->vinode;
	struct pmemfile_inode *inode = D_RW(vinode->inode);
	off64_t ret;

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
		default:
			ret = -1;
			break;
	}

	if (ret < 0) {
		ret = -1;
		errno = EINVAL;
	} else {
		if (file->offset != (size_t)ret)
			LOG(LDBG, "off diff: old %lu != new %lu", file->offset,
					(size_t)ret);
		file->offset = (size_t)ret;
	}

	util_mutex_unlock(&file->mutex);

	return ret;
}

off_t
pmemfile_lseek(PMEMfilepool *pfp, PMEMfile *file, off_t offset, int whence)
{
	return pmemfile_lseek64(pfp, file, offset, whence);
}
