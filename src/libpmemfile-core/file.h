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
#ifndef PMEMFILE_FILE_H
#define PMEMFILE_FILE_H

/*
 * Runtime state structures.
 */

#include <pthread.h>
#include <stddef.h>
#include "inode.h"
#include "layout.h"

struct ctree;

#define PFILE_READ (1ULL << 0)
#define PFILE_WRITE (1ULL << 1)
#define PFILE_NOATIME (1ULL << 2)
#define PFILE_APPEND (1ULL << 3)

/* File */
struct pmemfile_file {
	/* File inode. */
	struct pmemfile_vinode *vinode;

	/*
	 * Protects against changes to offset / position cache from multiple
	 * threads.
	 */
	pthread_mutex_t mutex;

	/* Flags */
	uint64_t flags;

	/* Requested/current position. */
	size_t offset;

	/* Current position cache, the latest block used. */
	struct pmemfile_block *block_pointer_cache;

	struct pmemfile_dir_pos {
		/* Current directory list */
		struct pmemfile_dir *dir;

		/* Id of the current directory list */
		unsigned dir_id;
	} dir_pos;
};

const char *file_check_pathname(const char *pathname);

#endif
