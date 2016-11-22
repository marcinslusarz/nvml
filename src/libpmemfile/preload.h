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

#ifndef PMEMFILE_PRELOAD_RESOLVE_PATH_H
#define PMEMFILE_PRELOAD_RESOLVE_PATH_H

#include <sys/stat.h>
#include <stddef.h>

struct PMEMfilepool;

struct pool_description {
	// A path where the mount point is - a directory must exist at this path
	char mount_point[0x1000];

	// The canonical parent directory of the mount point
	char mount_point_parent[0x1000];

	size_t len_mount_point_parent;

	// Where the actual pmemfile pool is
	char poolfile_path[0x1000];

	// Keep the mount point directory open
	long fd;

	/*
	 * The inode number of the mount point, and other information that just
	 * might be useful.
	 */
	struct stat stat;

	/*
	 * The pmemfile pool associated with this mount point.
	 * If this is NULL, the mount point was not used by the application
	 * before in this process. Should be initialized on first use.
	 */
	PMEMfilepool *pool;
};

struct path_component {
	long error_code;

	struct pool_description *pool; // if NULL, path is in kernels FS

	char path[0x1000];
	size_t path_len;
};

enum resolve_last_or_not { resolve_last_slink, no_resolve_last_slink };

struct pool_description *lookup_pd_by_inode(__ino_t inode);
struct pool_description *lookup_pd_by_path(const char *path);

void resolve_path(struct pool_description *in_pool, const char *path,
			struct path_component *result,
			enum resolve_last_or_not);

#endif
