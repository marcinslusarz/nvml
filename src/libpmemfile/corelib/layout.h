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
#ifndef PMEMFILE_LAYOUT_H
#define PMEMFILE_LAYOUT_H

/*
 * On-media structures.
 */

#include "libpmemobj.h"
#include <stdbool.h>

POBJ_LAYOUT_BEGIN(pmemfile);
POBJ_LAYOUT_ROOT(pmemfile, struct pmemfile_super);
POBJ_LAYOUT_TOID(pmemfile, struct pmemfile_inode);
POBJ_LAYOUT_TOID(pmemfile, struct pmemfile_dir);
POBJ_LAYOUT_TOID(pmemfile, struct pmemfile_block_array);
POBJ_LAYOUT_TOID(pmemfile, struct pmemfile_inode_array);
POBJ_LAYOUT_TOID(pmemfile, char);
POBJ_LAYOUT_END(pmemfile);

/* Inode */
struct pmemfile_inode {
	/* Size of file. */
	uint64_t size;

	/* File flags. */
	uint64_t flags;

	/* Time of last status change. */
	struct timespec ctime;

	/* Time of last modification. */
	struct timespec mtime;

	/* Time of last access. */
	struct timespec atime;

	/* Hard link counter. */
	nlink_t nlink;

	union {
		/* File specific data. */
		TOID(struct pmemfile_block_array) blocks;

		/* Directory specific data. */
		TOID(struct pmemfile_dir) dir;
	} file_data;
};

struct pmemfile_block {
	TOID(char) data;
	size_t allocated;
	size_t used;
};

/* XXX tweak this value */
#define MAXNUMBLOCKS 100
/* File */
struct pmemfile_block_array {
	/* number of allocated chunks, <0, MAXNUMBLOCKS> */
	unsigned blocks_allocated;
	struct pmemfile_block blocks[MAXNUMBLOCKS];

	TOID(struct pmemfile_block_array) next;
};

#define PMEMFILE_MAX_FILE_NAME 255
/* Directory entry */
struct pmemfile_dentry {
	TOID(struct pmemfile_inode) inode;
	char name[PMEMFILE_MAX_FILE_NAME + 1];
};

/*
 * XXX tweak this number when on-media layout will be complete to let inode fit
 * in some nice number (one page?)
 */
#define NUMDENTRIES 100

/* Directory */
struct pmemfile_dir {
	/* Number of used entries, <0, NUMDENTRIES>. */
	uint64_t used;
	struct pmemfile_dentry dentries[NUMDENTRIES];
	TOID(struct pmemfile_dir) next;
};

#define NUMINODES_PER_ENTRY 249

struct pmemfile_inode_array {
	PMEMmutex mtx;
	TOID(struct pmemfile_inode_array) prev;
	TOID(struct pmemfile_inode_array) next;

	/* Number of used entries, <0, NUMINODES_PER_ENTRY>. */
	uint64_t used;

	TOID(struct pmemfile_inode) inodes[NUMINODES_PER_ENTRY];
	char padding[8];
};

/* Superblock */
struct pmemfile_super {
	/* XXX unused */
	uint64_t version;

	/* Root directory inode */
	TOID(struct pmemfile_inode) root_inode;

	/* List of arrays of opened inodes. */
	TOID(struct pmemfile_inode_array) opened_inodes;

	/* Flag indicating mkfs finished its work. */
	char initialized;

	char padding[4096 - 8/*version*/ - 16/*toid*/ - 16/*toid*/ - 1/*init*/];
};

#endif
