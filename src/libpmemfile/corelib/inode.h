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

#ifndef PMEMFILE_INODE_H
#define PMEMFILE_INODE_H

#include <pthread.h>
#include <stdint.h>
#include <time.h>

#include "libpmemfile-core.h"
#include "layout.h"
#include "urwlock.h"

/* Inode */
struct pmemfile_vinode {
	uint32_t ref;

	union {
		pthread_spinlock_t spin;
		pthread_rwlock_t rwlock;
		struct urwlock urwlock;
	} lock;
	TOID(struct pmemfile_inode) inode;

#ifdef DEBUG
	/*
	 * One of the full paths inode can be reached from.
	 * Used only for debugging.
	 */
	char *path;
#endif

	/* Pointer to the array of opened inodes. */
	struct {
		struct pmemfile_inode_array *arr;
		unsigned idx;
	} opened;
};

static inline bool _file_is_dir(const struct pmemfile_inode *inode)
{
	return S_ISDIR(inode->flags);
}

static inline bool file_is_dir(struct pmemfile_vinode *vinode)
{
	return _file_is_dir(D_RO(vinode->inode));
}

static inline bool _file_is_regular_file(const struct pmemfile_inode *inode)
{
	return S_ISREG(inode->flags);
}

static inline bool file_is_regular_file(struct pmemfile_vinode *vinode)
{
	return _file_is_regular_file(D_RO(vinode->inode));
}

struct pmemfile_vinode *file_inode_alloc(PMEMfilepool *pfp,
		uint64_t flags, struct timespec *t);

void file_inode_free(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) tinode);

const char *pmfi_path(struct pmemfile_vinode *vinode);

void file_inode_ref(PMEMfilepool *pfp,
		struct pmemfile_vinode *vinode);

struct pmemfile_inode_map *file_inode_map_alloc(void);

void file_inode_map_free(struct pmemfile_inode_map *c);

struct pmemfile_vinode *file_vinode_get(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) inode,
		bool ref);

struct pmemfile_vinode *file_vinode_ref(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) inode);

struct pmemfile_vinode *file_vinode_ref_new(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode) inode);

void file_vinode_unref(PMEMfilepool *pfp,
		struct pmemfile_vinode *vinode,
		bool locked);
void file_vinode_unref_tx(PMEMfilepool *pfp,
		struct pmemfile_vinode *vinode);

#endif
