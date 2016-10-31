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
	unsigned block_id;
};

/*
 * file_insert_block_to_cache -- inserts block into the tree
 */
static void
file_insert_block_to_cache(struct ctree *c,
		struct pmemfile_block_array *block_array,
		unsigned block_id,
		size_t off)
{
	struct file_block_info *info = Malloc(sizeof(*info));
	info->arr = block_array;
	info->block_id = block_id;

	ctree_insert_unlocked(c, off, (uintptr_t)info);
}

/*
 * file_rebuild_block_tree -- rebuilds runtime tree of blocks
 */
static void
file_rebuild_block_tree(struct pmemfile_vinode *vinode)
{
	struct ctree *c = ctree_new();
	if (!c)
		return;
	struct pmemfile_inode *inode = D_RW(vinode->inode);
	struct pmemfile_block_array *block_array = &inode->file_data.blocks;
	size_t off = 0;

	while (block_array != NULL) {
		for (unsigned i = 0; i < block_array->length; ++i) {
			struct pmemfile_block *block = &block_array->blocks[i];

			if (block->size == 0)
				break;
			file_insert_block_to_cache(c, block_array, i, off);

			off += block->size;
		}

		block_array = D_RW(block_array->next);
	}

	vinode->blocks = c;
}

/*
 * file_destroy_data_state -- destroys file state related to data
 */
void
file_destroy_data_state(struct pmemfile_vinode *vinode)
{
	struct ctree *blocks = vinode->blocks;
	if (!blocks)
		return;

	uint64_t key = UINT64_MAX;
	struct file_block_info *info;
	while ((info = (void *)(uintptr_t)ctree_find_le_unlocked(blocks,
			&key))) {
		Free(info);
		uint64_t k = ctree_remove_unlocked(blocks, key, 1);
		ASSERTeq(k, key);

		key = UINT64_MAX;
	}

	ctree_delete(blocks);
	vinode->blocks = NULL;
}

/*
 * file_reset_cache -- resets position pointer to the beginning of the file
 */
static void
file_reset_cache(PMEMfile *file, struct pmemfile_inode *inode,
		struct pmemfile_pos *pos)
{
	pos->block_array = &inode->file_data.blocks;
	pos->block_id = 0;
	pos->block_offset = 0;

	pos->global_offset = 0;
}

/*
 * file_allocate_block -- allocates new block
 */
static void
file_allocate_block(PMEMfile *file,
		struct pmemfile_inode *inode,
		struct pmemfile_pos *pos,
		struct pmemfile_block *block,
		size_t count)
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
	block->data = TX_XALLOC(char, sz, POBJ_XALLOC_NO_FLUSH);
	block->size = pmemobj_alloc_usable_size(block->data.oid);

	TX_ADD_DIRECT(&inode->last_block_fill);
	inode->last_block_fill = 0;

	file_insert_block_to_cache(file->vinode->blocks, block_array,
			(unsigned)(block - &block_array->blocks[0]),
			pos->global_offset);
}

/*
 * file_extend_block_meta_data -- updates metadata of the current block
 */
static void
file_extend_block_meta_data(struct pmemfile_inode *inode,
		struct pmemfile_block_array *block_array,
		struct pmemfile_block *block,
		size_t len)
{
	TX_ADD_FIELD_DIRECT(inode, last_block_fill);
	inode->last_block_fill += len;

	TX_ADD_FIELD_DIRECT(inode, size);
	inode->size += len;
}

/*
 * file_zero_extend_block -- extends current block with zeroes
 */
static void
file_zero_extend_block(PMEMfilepool *pfp,
		struct pmemfile_inode *inode,
		struct pmemfile_block_array *block_array,
		struct pmemfile_block *block,
		size_t len)
{
	char *addr = D_RW(block->data) + inode->last_block_fill;

	/*
	 * We can safely skip tx_add_range, because there's no user visible
	 * data at this address.
	 */
	pmemobj_memset_persist(pfp->pop, addr, 0, len);

	file_extend_block_meta_data(inode, block_array, block, len);
}

/*
 * file_next_block_array -- changes current block array to the next one
 */
static bool
file_next_block_array(struct pmemfile_pos *pos, bool extend)
{
	/* Transition to the next block array in block array list. */
	TOID(struct pmemfile_block_array) next = pos->block_array->next;

	if (TOID_IS_NULL(next)) {
		if (!extend)
			return false;

		next = TX_ZALLOC(struct pmemfile_block_array, 4096);
		D_RW(next)->length = (uint32_t)
				((pmemobj_alloc_usable_size(next.oid) -
				sizeof(struct pmemfile_block_array)) /
				sizeof(struct pmemfile_block));
		TX_SET_DIRECT(pos->block_array, next, next);
	}

	pos->block_array = D_RW(next);

	/* We changed the block array, so we have to reset block position. */
	pos->block_id = 0;
	pos->block_offset = 0;

	return true;
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
		struct pmemfile_block *block,
		size_t offset_left,
		bool extend,
		bool is_last)
{
	if (block->size == 0) {
		if (extend)
			file_allocate_block(file, inode, pos, block,
					offset_left);
		else
			return 0;
	}

	/*
	 * Is anticipated position within the current block?
	 */
	if (pos->block_offset + offset_left < block->size) {
		/*
		 * Is anticipated position between the end of
		 * used space and the end of block?
		 */
		if (is_last && pos->block_offset + offset_left
				> inode->last_block_fill) {
			if (!extend) {
				size_t sz = inode->last_block_fill -
						pos->block_offset;
				pos->block_offset += sz;
				pos->global_offset += sz;

				return sz;
			}

			file_zero_extend_block(pfp, inode, pos->block_array,
					block, offset_left -
					inode->last_block_fill);

			ASSERT(inode->last_block_fill <= block->size);
		}

		pos->block_offset += offset_left;
		pos->global_offset += offset_left;

		ASSERTeq(pos->global_offset, file->offset);

		return offset_left;
	}

	/*
	 * Now we know offset lies in one of the consecutive blocks.
	 * So we can go to the next block.
	 */
	size_t sz = block->size - pos->block_offset;
	pos->block_offset += sz;
	pos->global_offset += sz;

	return sz;
}

/*
 * file_write_within_block -- writes data to current block
 */
static size_t
file_write_within_block(PMEMfilepool *pfp,
		PMEMfile *file,
		struct pmemfile_inode *inode,
		struct pmemfile_pos *pos,
		struct pmemfile_block *block,
		const void *buf,
		size_t count_left,
		bool is_last)
{
	if (block->size == 0)
		file_allocate_block(file, inode, pos, block, count_left);

	/* How much data should we write to this block? */
	size_t len = min(block->size - pos->block_offset, count_left);

	pmemobj_memcpy_persist(pfp->pop, D_RW(block->data) + pos->block_offset,
			buf, len);

	if (is_last) {
		/*
		 * If new size is beyond the block used size, then we
		 * have to update all metadata.
		 */
		if (pos->block_offset + len > inode->last_block_fill) {
			size_t new_used = pos->block_offset + len
					- inode->last_block_fill;

			file_extend_block_meta_data(inode, pos->block_array,
					block, new_used);
		}

		ASSERT(inode->last_block_fill <= block->size);
	}

	pos->block_offset += len;
	pos->global_offset += len;

	return len;
}

/*
 * file_read_from_block -- reads data from current block
 */
static size_t
file_read_from_block(struct pmemfile_inode *inode,
		struct pmemfile_pos *pos,
		struct pmemfile_block *block,
		void *buf,
		size_t count_left,
		bool is_last)
{
	if (block->size == 0)
		return 0;

	/* How much data should we read from this block? */
	size_t len = is_last ? inode->last_block_fill : block->size;
	len = min(len - pos->block_offset, count_left);

	if (len == 0)
		return 0;

	memcpy(buf, D_RW(block->data) + pos->block_offset, len);

	pos->block_offset += len;
	pos->global_offset += len;

	return len;
}

/*
 * is_last_block -- returns true when specified block is the last one in a file
 */
static bool
is_last_block(unsigned block_id, struct pmemfile_block_array *block_array)
{
	if (block_id == block_array->length - 1)
		return TOID_IS_NULL(block_array->next);
	else
		return block_array->blocks[block_id + 1].size == 0;
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

	if (pos->block_array == NULL)
		file_reset_cache(file, inode, pos);

	if (file->offset != pos->global_offset) {
		size_t block_start = pos->global_offset - pos->block_offset;
		size_t off = file->offset;

		if (off < block_start ||
				off >= block_start +
			pos->block_array->blocks[pos->block_id].size) {

			struct file_block_info *info = (void *)(uintptr_t)
				ctree_find_le_unlocked(file->vinode->blocks,
						&off);
			if (info) {
				pos->block_array = info->arr;
				pos->block_id = info->block_id;
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
			file_reset_cache(file, inode, pos);
		}
	}

	size_t offset_left = file->offset - pos->global_offset;

	/*
	 * Find the position, possibly extending and/or zeroing unused space.
	 */

	while (offset_left > 0) {
		struct pmemfile_block_array *block_array = pos->block_array;
		struct pmemfile_block *block =
				&block_array->blocks[pos->block_id];
		bool is_last = is_last_block(pos->block_id, block_array);

		size_t seeked = file_seek_within_block(pfp, file, inode, pos,
				block, offset_left, true, is_last);

		ASSERT(seeked <= offset_left);

		offset_left -= seeked;

		if (offset_left > 0) {
			pos->block_id++;
			pos->block_offset = 0;

			if (pos->block_id == block_array->length)
				file_next_block_array(pos, true);
		}
	}

	/*
	 * Now file->offset matches cached position in file->pos.
	 *
	 * Let's write the requested data starting from current position.
	 */

	size_t count_left = count;
	while (count_left > 0) {
		struct pmemfile_block_array *block_array = pos->block_array;
		struct pmemfile_block *block =
				&block_array->blocks[pos->block_id];
		bool is_last = is_last_block(pos->block_id, block_array);

		size_t written = file_write_within_block(pfp, file, inode, pos,
				block, buf, count_left, is_last);

		ASSERT(written <= count_left);

		buf += written;
		count_left -= written;

		if (count_left > 0) {
			pos->block_id++;
			pos->block_offset = 0;

			if (pos->block_id == block_array->length)
				file_next_block_array(pos, true);
		}
	}
}

/*
 * pmemfile_write -- writes to file
 */
ssize_t
pmemfile_write(PMEMfilepool *pfp, PMEMfile *file, const void *buf, size_t count)
{
	LOG(LDBG, "file %p buf %p count %zu", file, buf, count);

	if (!file_is_regular_file(file->vinode)) {
		errno = EINVAL;
		return -1;
	}

	if (!(file->flags & PFILE_WRITE)) {
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

	memcpy(&pos, &file->pos, sizeof(pos));

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		rwlock_tx_wlock(&vinode->rwlock);

		if (!vinode->blocks)
			file_rebuild_block_tree(vinode);

		file_write(pfp, file, inode, buf, count);

		if (count > 0) {
			struct pmemfile_time tm;
			file_get_time(&tm);
			TX_SET(vinode->inode, mtime, tm);
		}

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

	if (off < block_start || off >= block_start +
		pos->block_array->blocks[pos->block_id].size) {

		struct file_block_info *info = (void *)(uintptr_t)
			ctree_find_le_unlocked(file->vinode->blocks, &off);
		if (!info)
			return false;

		pos->block_array = info->arr;
		pos->block_id = info->block_id;
		pos->block_offset = 0;
		pos->global_offset = off;
	}

	if (file->offset < pos->global_offset) {
		if (file->offset >= pos->global_offset - pos->block_offset) {
			pos->global_offset -= pos->block_offset;
			pos->block_offset = 0;
		} else {
			file_reset_cache(file, inode, pos);

			if (pos->block_array == NULL)
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

	if (unlikely(pos->block_array == NULL)) {
		file_reset_cache(file, inode, pos);

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
		struct pmemfile_block_array *block_array = pos->block_array;
		struct pmemfile_block *block =
				&block_array->blocks[pos->block_id];
		bool is_last = is_last_block(pos->block_id, block_array);

		size_t seeked = file_seek_within_block(pfp, file, inode, pos,
				block, offset_left, false, is_last);

		if (seeked == 0) {
			uint64_t used = is_last ?
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

			pos->block_id++;
			pos->block_offset = 0;

			if (pos->block_id == block_array->length) {
				if (!file_next_block_array(pos, false))
					/* EOF */
					return 0;
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
		struct pmemfile_block_array *block_array = pos->block_array;
		struct pmemfile_block *block =
				&block_array->blocks[pos->block_id];
		bool is_last = is_last_block(pos->block_id, block_array);

		size_t read1 = file_read_from_block(inode, pos, block, buf,
				count_left, is_last);

		if (read1 == 0) {
			uint64_t used = is_last ?
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

			pos->block_id++;
			pos->block_offset = 0;

			if (pos->block_id == block_array->length) {
				if (!file_next_block_array(pos, false))
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

	if (!file_is_regular_file(file->vinode)) {
		errno = EINVAL;
		return -1;
	}

	if (!(file->flags & PFILE_READ)) {
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

	if (!vinode->blocks) {
		util_rwlock_wrlock(&vinode->rwlock);
		file_rebuild_block_tree(vinode);
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

	if (file_is_dir(file->vinode)) {
		if (whence == SEEK_END) {
			errno = EINVAL;
			return -1;
		}
	} else if (file_is_regular_file(file->vinode)) {
		/* Nothing to do for now */
	} else {
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
file_truncate(struct pmemfile_vinode *vinode)
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
	file_destroy_data_state(vinode);
}
