/*
 * Copyright 2016-2017, Intel Corporation
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
 * vinode_rebuild_block_tree -- rebuilds runtime tree of blocks
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

/*
 * is_offset_in_block -- check if the given offset is in the range
 * specified by the block metadata
 */
static bool
is_offset_in_block(const struct pmemfile_block *block, uint64_t offset)
{
	if (block == NULL)
		return false;

	return block->offset <= offset && offset < block->offset + block->size;
}

/*
 * find_block -- look up block metadata with the highest offset
 * lower than or equal to the offset argument
 */
static struct pmemfile_block *
find_block(struct pmemfile_file *file, uint64_t offset)
{
	if (is_offset_in_block(file->block_pointer_cache, offset))
		return file->block_pointer_cache;

	struct pmemfile_block *block;

	block = (void *)(uintptr_t)ctree_find_le_unlocked(file->vinode->blocks,
	    &offset);

	if (block != NULL)
		file->block_pointer_cache = block;

	return block;
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
 * find_last_block -- lookup the metadata of the block with the highest
 * offset in the file - the metadata of the last block
 */
static struct pmemfile_block *
find_last_block(struct pmemfile_file *file)
{
	if (file->block_pointer_cache != NULL &&
	    is_last_block(file->block_pointer_cache))
		return file->block_pointer_cache;

	uint64_t off = SIZE_MAX;

	return (void *)(uintptr_t)ctree_find_le_unlocked(file->vinode->blocks,
	    &off);
}

/*
 * vinode_destroy_data_state -- destroys file state related to data
 */
void
vinode_destroy_data_state(struct pmemfile_vinode *vinode)
{
	if (vinode->blocks) {
		ctree_delete(vinode->blocks);
		vinode->blocks = NULL;
	}

	memset(&vinode->first_free_block, 0, sizeof(vinode->first_free_block));
}

/*
 * file_allocate_block_data -- allocates new block data.
 * The block metadata must be already allocated, and passed as the block
 * pointer argument.
 */
static void
file_allocate_block_data(PMEMfilepool *pfp,
		PMEMfile *file,
		struct pmemfile_inode *inode,
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

	/* XXX, snapshot separated to let pmemobj use small object cache  */
	pmemobj_tx_add_range_direct(block, 32);
	pmemobj_tx_add_range_direct((char *)block + 32, 16);
	COMPILE_ERROR_ON(sizeof(*block) != 48);

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
}

static struct pmemfile_block *
get_free_block(struct pmemfile_vinode *vinode)
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
	ASSERT(prev != NULL);
	TX_SET_DIRECT(prev, next, next);

	binfo->arr = D_RW(next);
	binfo->idx = 0;

	return &binfo->arr->blocks[binfo->idx];
}

/*
 * expand_file - Optionally append filled blocks to the
 * end of a file. This is going to need important changes when implementing
 * sparse files.
 */
static void
expand_file(PMEMfilepool *pfp, PMEMfile *file, struct pmemfile_inode *inode,
		uint64_t new_size)
{
	ASSERT(inode->size < new_size);

	TX_ADD_FIELD_DIRECT(inode, size);

	struct pmemfile_block *last_block = find_last_block(file);

	if (last_block != NULL) {
		/*
		 * The last block might have some space allocated, but as of
		 * yet unused. Expanding the file to this space does not
		 * require allocating new blocks.
		 */

		uint64_t available = last_block->offset + last_block->size;

		if (available >= new_size) {
			/*
			 * Expanding file within already allocated block,
			 * some free space might still remain in the block.
			 */
			inode->size = new_size;
			return; /* No need to allocate new blocks */
		}

		/*
		 * Expanding file within already allocated block,
		 * using the whole block. It is known that more
		 * blocks are still needed.
		 */
		inode->size = available;
	}

	/*
	 * Number of bytes to be allocated. This is passed down to the
	 * file_allocate_block_data routine, to be used in a heuristic
	 * determining block size. Some might expect this value to be
	 * decreased during the following loop, but that might have undesirable
	 * consequences. Take the case of a 65 kilobyte write for example:
	 * The heuristic would allocate a 64 kilobyte block, followed by a
	 * 1 kilobyte block. Repeated 65 kilobyte writes would result in a file
	 * with 64 and 1 kilobyte blocks interleved. In such a case, a uniform
	 * 64 kilobyte block size seems like a better option.
	 * todo: vehemently argue about the allocation strategy in the office,
	 * and hope for a consensus.
	 *
	 *
	 * Marked as const -- if you try to alter count in the loop, and it
	 * doesn't compile, read the above litany.
	 */
	const uint64_t count = new_size - inode->size;

	/*
	 * By this point, it is known that the file can not be extended
	 * without allocating new block(s).
	 */

	ASSERT(count > 0);

	/*
	 * The pointer last_block always points to the metadata associated with
	 * the last block in the file. This current last block in the loop is
	 * added to the current transaction using the file_allocate_block_data
	 * routine, so there is no need to add the last_block->next field to
	 * the transaction in the two places below, where it is assigned to. An
	 * exception is the case of the first loop iteration, when last_block
	 * points to the previously existing last block ( if there is any ).
	 * Thus its next field must be added to the transaction separately.
	 */
	if (last_block != NULL)
		TX_ADD_DIRECT(&last_block->next);

	/* At what offset in the file the next block should be inserted? */
	uint64_t offset = inode->size;

	while (inode->size < new_size) {
		/* last_block is NULL when the file is empty */
		ASSERT(last_block == NULL || is_last_block(last_block));

		/*
		 * XXX: coalesce get_free_block and file_allocate_block_data
		 * into a single function?
		 */
		struct pmemfile_block *next_block =
			get_free_block(file->vinode);
		file_allocate_block_data(pfp, file, inode, next_block, count);
		next_block->offset = offset;

		/*
		 * Link the new block into the chain of blocks, if there
		 * already was such a chain.
		 */
		if (last_block != NULL)
			last_block->next =
			    (TOID(struct pmemfile_block))
			    pmemobj_oid(next_block);

		uint64_t available = inode->size + next_block->size;
		if (available > new_size)
			inode->size = new_size;
		else
			inode->size = available;

		block_cache_insert_block(file->vinode->blocks, next_block);

		/*
		 * Set up last_block and offset to hold the loops invariant
		 * in the next iteration.
		 * Note: the next_block->next field is left uninitialized.
		 * Everything else in the block metadata is initialized.
		 */
		last_block = next_block;
		offset = inode->size;
	}

	ASSERT(last_block != NULL); /* at least one block was allocated */

	/*
	 * Close the linked list of blocks -- the loop did not initialize
	 * the next field of the last block in the last iteration.
	 */
	last_block->next = TOID_NULL(struct pmemfile_block);
}

static inline void
iterate_on_file_range(PMEMfilepool *pfp, PMEMfile *file,
    uint64_t offset, uint64_t len,
    int (*callback)(PMEMfilepool *, char *, char *chunk,
	uint64_t offset, uint64_t chunk_size),
    void *arg)
{
	struct pmemfile_block *block = find_block(file, offset);

	/*
	 * As long as sparse files are not implemented, a file
	 * with non-zero size must have at least one block, that
	 * can be found with ctree_find_le_unlocked, thus the following assert.
	 */
	ASSERT(block != NULL);

	uint64_t range_offset = 0;

	for (; len > 0; block = D_RW(block->next)) {
		/* Remember the pointer to block used last time */
		file->block_pointer_cache = block;

		/*
		 * Two more asserts, that should stay here as long as
		 * sparse files are not implemented.
		 */
		ASSERT(is_offset_in_block(block, offset));

		/*
		 * Multiple blocks might be used, but the first and last
		 * blocks are special, in the sense that not necesseraly
		 * all of their content is copied.
		 */

		/*
		 * Offset to data used from the block.
		 * It should be zero, unless it is the first block in
		 * the range.
		 */
		uint64_t in_block_start = offset - block->offset;

		/*
		 * The number of bytes used from this block.
		 * Unless it is the last block in the range, all
		 * data till the end of the block is used.
		 */
		uint64_t in_block_len = block->size - in_block_start;

		if (len < in_block_len) {
			/*
			 * Don't need all the data till the end of this block?
			 */
			in_block_len = len;
		}

		ASSERT(in_block_start < block->size);
		ASSERT(in_block_start + in_block_len <= block->size);

		char *data = D_RW(block->data) + in_block_start;

		if (callback(pfp, arg, data, range_offset, in_block_len) != 0)
			return;

		offset += in_block_len;
		range_offset += in_block_len;
		len -= in_block_len;
	}
}

static int
range_zero_fill(PMEMfilepool *pfp, char *arg, char *chunk,
	uint64_t offset, uint64_t chunk_size)
{
	(void) arg;
	(void) offset;

	VALGRIND_ADD_TO_TX(chunk, chunk_size);
	pmemobj_memset_persist(pfp->pop, chunk, 0, chunk_size);
	VALGRIND_REMOVE_FROM_TX(chunk, chunk_size);

	return 0;
}

static int
range_write(PMEMfilepool *pfp, char *buf, char *chunk,
	uint64_t offset, uint64_t chunk_size)
{
	VALGRIND_ADD_TO_TX(chunk, chunk_size);
	pmemobj_memcpy_persist(pfp->pop, chunk, buf + offset, chunk_size);
	VALGRIND_REMOVE_FROM_TX(chunk, chunk_size);

	return 0;
}

/*
 * file_write -- writes to file
 */
static void
file_write(PMEMfilepool *pfp, PMEMfile *file, struct pmemfile_inode *inode,
		const char *buf, size_t count)
{
	ASSERT(count > 0);

	/*
	 * Three steps:
	 * - Append new blocks to end of the file ( optionally )
	 * - Zero Fill some new blocks, in case the file is extended by
	 *   writing to the file after seeking past file size ( optionally )
	 * - Copy the data from the users buffer
	 */

	uint64_t original_size = inode->size;

	if (inode->size < file->offset + count)
		expand_file(pfp, file, inode, file->offset + count);

	if (original_size < file->offset)
		iterate_on_file_range(pfp, file,
		    original_size,
		    file->offset - original_size,
		    range_zero_fill, NULL);

	/* All blocks needed for writing are properly allocated by this point */

	iterate_on_file_range(pfp, file, file->offset, count,
	    range_write, (void *)buf);
}

static ssize_t
pmemfile_write_locked(PMEMfilepool *pfp, PMEMfile *file, const void *buf,
		size_t count)
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

	if ((ssize_t)count < 0)    /* Normally this will still   */
		count = SSIZE_MAX; /* try to write 2^63 bytes... */

	if (file->offset + count < file->offset) /* overflow check */
		count = SIZE_MAX - file->offset;

	if (count == 0)
		return 0;

	int error = 0;

	struct pmemfile_vinode *vinode = file->vinode;
	struct pmemfile_inode *inode = D_RW(vinode->inode);

	util_rwlock_wrlock(&vinode->rwlock);

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
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
	} TX_ONABORT {
		error = errno;
	} TX_ONCOMMIT {
		file->offset += count;
	} TX_END

	util_rwlock_unlock(&vinode->rwlock);

	if (error) {
		errno = error;
		return -1;
	}

	return (ssize_t)count;
}

/*
 * pmemfile_write -- writes to file
 */
ssize_t
pmemfile_write(PMEMfilepool *pfp, PMEMfile *file, const void *buf, size_t count)
{
	ssize_t ret;

	util_mutex_lock(&file->mutex);
	ret = pmemfile_write_locked(pfp, file, buf, count);
	util_mutex_unlock(&file->mutex);

	return ret;
}

static int
file_read_callback(PMEMfilepool *pfp, char *buf,
		char *chunk, uint64_t offset, uint64_t chunk_size)
{
	(void) pfp;

	memcpy(buf + offset, chunk, chunk_size);

	return 0;
}

/*
 * file_read -- reads file
 */
static size_t
file_read(PMEMfilepool *pfp, PMEMfile *file, struct pmemfile_inode *inode,
		char *buf, size_t count)
{
	uint64_t size = inode->size;

	/*
	 * Start reading at file->offset, stop reading
	 * when end of file is reached, or count bytes were read.
	 * The following two branches compute how many bytes are
	 * going to be read.
	 */
	if (file->offset >= size)
		return 0; /* EOF already */

	if (size - file->offset < count)
		count = size - file->offset;

	iterate_on_file_range(pfp, file, file->offset, count,
	    file_read_callback, buf);

	return count;
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

static ssize_t
pmemfile_read_locked(PMEMfilepool *pfp, PMEMfile *file, void *buf, size_t count)
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

	util_rwlock_rdlock(&vinode->rwlock);
	while (!vinode->blocks) {
		util_rwlock_unlock(&vinode->rwlock);
		util_rwlock_wrlock(&vinode->rwlock);
		if (!vinode->blocks)
			vinode_rebuild_block_tree(vinode);
		util_rwlock_unlock(&vinode->rwlock);
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
		util_rwlock_wrlock(&vinode->rwlock);

		TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
			TX_SET(vinode->inode, atime, tm);
		} TX_ONABORT {
			LOG(LINF, "can not update inode atime");
		} TX_END

		util_rwlock_unlock(&vinode->rwlock);
	}


	file->offset += bytes_read;

	ASSERT(bytes_read <= count);
	return (ssize_t)bytes_read;
}

/*
 * pmemfile_read -- reads file
 */
ssize_t
pmemfile_read(PMEMfilepool *pfp, PMEMfile *file, void *buf, size_t count)
{
	ssize_t ret;

	util_mutex_lock(&file->mutex);
	ret = pmemfile_read_locked(pfp, file, buf, count);
	util_mutex_unlock(&file->mutex);

	return ret;
}

/*
 * pmemfile_lseek64 -- changes file current offset
 */
static off64_t
pmemfile_lseek64_locked(PMEMfilepool *pfp, PMEMfile *file, off64_t offset,
		int whence)
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

	return ret;
}

/*
 * pmemfile_lseek64 -- changes file current offset
 */
off64_t
pmemfile_lseek64(PMEMfilepool *pfp, PMEMfile *file, off64_t offset, int whence)
{
	off64_t ret;

	util_mutex_lock(&file->mutex);
	ret = pmemfile_lseek64_locked(pfp, file, offset, whence);
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

ssize_t
pmemfile_pread(PMEMfilepool *pfp, PMEMfile *file, void *buf, size_t count,
		off_t offset)
{
	/* XXX this is hacky implementation */
	ssize_t ret;
	util_mutex_lock(&file->mutex);

	size_t cur_off = file->offset;

	if (pmemfile_lseek64_locked(pfp, file, offset, SEEK_SET) != offset) {
		ret = -1;
		goto end;
	}

	ret = pmemfile_read_locked(pfp, file, buf, count);

	file->offset = cur_off;

end:
	util_mutex_unlock(&file->mutex);

	return ret;
}

ssize_t
pmemfile_pwrite(PMEMfilepool *pfp, PMEMfile *file, const void *buf,
		size_t count, off_t offset)
{
	/* XXX this is hacky implementation */
	ssize_t ret;
	util_mutex_lock(&file->mutex);

	size_t cur_off = file->offset;

	if (pmemfile_lseek64_locked(pfp, file, offset, SEEK_SET) != offset) {
		ret = -1;
		goto end;
	}

	ret = pmemfile_write_locked(pfp, file, buf, count);

	file->offset = cur_off;

end:
	util_mutex_unlock(&file->mutex);

	return ret;
}

/*
 * vinode_truncate -- changes file size to 0
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

	struct pmemfile_time tm;
	file_get_time(&tm);
	TX_SET(vinode->inode, mtime, tm);

	// we don't have to rollback destroy of data state on abort, because
	// it will be rebuilded when it's needed
	vinode_destroy_data_state(vinode);
}
