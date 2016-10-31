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
 * stubs.c -- placeholder routines for the functionality of
 * pmemfile that is not yet implemented.
 * All these set errno to ENOTSUP, except for pmemfile_getcwd.
 * Because pmemfile_getcwd is cool.
 */

#include <stddef.h>
#include <errno.h>
#include <stdlib.h>
#include <syscall.h>
#include <sys/mman.h>

#include "libpmemfile-core.h"

static void
check_pfp(PMEMfilepool *pfp)
{
	if (pfp == NULL)
		abort();
}

static void
check_pfp_file(PMEMfilepool *pfp, PMEMfile *file)
{
	check_pfp(pfp);

	// TODO: check that the PMEMfile* belongs to the pool
	if (file == NULL)
		abort();
}

int
pmemfile_chdir(PMEMfilepool *pfp, const char *path)
{
	check_pfp(pfp);

	(void) path;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_readlink(PMEMfilepool *pfp, const char *path,
			char *buf, size_t buf_len)
{
	check_pfp(pfp);

	(void) path;
	(void) buf_len;

	errno = ENOTSUP;
	return -1;
}

char *
pmemfile_getcwd(PMEMfilepool *pfp, char *buf, size_t buf_len)
{
	check_pfp(pfp);

	if (buf != NULL && buf_len == 0) {
		errno = EINVAL;
		return NULL;
	}

	if (buf_len > 1) {
		buf[0] = '/';
		buf[1] = '\0';
		return buf;
	} else {
		errno = EINVAL;
		return NULL;
	}
}

ssize_t
pmemfile_pread(PMEMfilepool *pfp, PMEMfile *file,
			char *buf, size_t count, off_t pos)
{
	check_pfp_file(pfp, file);

	(void) buf;
	(void) count;
	(void) pos;

	errno = ENOTSUP;
	return -1;
}

ssize_t
pmemfile_pwrite(PMEMfilepool *pfp, PMEMfile *file,
			const char *buf, size_t count, off_t pos)

{
	check_pfp_file(pfp, file);

	(void) file;
	(void) buf;
	(void) count;
	(void) pos;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_access(PMEMfilepool *pfp, const char *path, mode_t mode)
{
	check_pfp(pfp);

	(void) path;
	(void) mode;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_sync(PMEMfilepool *pfp)
{
	check_pfp(pfp);

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_fdatasync(PMEMfilepool *pfp, PMEMfile *file)
{
	check_pfp_file(pfp, file);

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_rename(PMEMfilepool *pfp, const char *old_path, const char *new_path)
{
	check_pfp(pfp);

	(void) *old_path;
	(void) *new_path;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_renameat(PMEMfilepool *pfp, PMEMfile *old_at, const char *old_path,
				PMEMfile *new_at, const char *new_path)
{
	check_pfp_file(pfp, old_at);
	check_pfp_file(pfp, new_at);

	(void) old_path;
	(void) new_path;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_openat(PMEMfilepool *pfp, PMEMfile *at,
		const char *path, int oflag, mode_t mode)
{
	check_pfp_file(pfp, at);

	(void) path;
	(void) oflag;
	(void) mode;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_flock(PMEMfilepool *pfp, PMEMfile *file, int operation)
{
	check_pfp_file(pfp, file);

	(void) operation;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_truncate(PMEMfilepool *pfp, const char *path, off_t length)
{
	check_pfp(pfp);

	(void) path;
	(void) length;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_ftruncate(PMEMfilepool *pfp, PMEMfile *file, off_t length)
{
	check_pfp_file(pfp, file);

	(void) length;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_fchdir(PMEMfilepool *pfp, PMEMfile *file)
{
	check_pfp_file(pfp, file);

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_mkdir(PMEMfilepool *pfp, const char *path, mode_t mode)
{
	check_pfp(pfp);

	(void) path;
	(void) mode;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_mkdirat(PMEMfilepool *pfp, PMEMfile *file,
			const char *path, mode_t mode)
{
	check_pfp_file(pfp, file);

	(void) path;
	(void) mode;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_rmdir(PMEMfilepool *pfp, const char *path)
{
	check_pfp(pfp);

	(void) path;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_symlink(PMEMfilepool *pfp, const char *path1, const char *path2)
{
	check_pfp(pfp);

	(void) path1;
	(void) path2;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_symlinkat(PMEMfilepool *pfp, const char *path1,
			PMEMfile *at, const char *path2)
{
	check_pfp_file(pfp, at);

	(void) path1;
	(void) path2;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_chmod(PMEMfilepool *pfp, const char *path, mode_t mode)
{
	check_pfp(pfp);

	(void) path;
	(void) mode;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_fchmod(PMEMfilepool *pfp, PMEMfile *file, mode_t mode)
{
	check_pfp_file(pfp, file);

	(void) mode;

	errno = ENOTSUP;
	return -1;
}

PMEMfile *
pmemfile_dup(PMEMfilepool *pfp, PMEMfile *file)
{
	check_pfp_file(pfp, file);

	errno = ENOTSUP;
	return NULL;
}

PMEMfile *
pmemfile_dup2(PMEMfilepool *pfp, PMEMfile *file, PMEMfile *file2)
{
	check_pfp_file(pfp, file);
	check_pfp_file(pfp, file2);

	errno = ENOTSUP;
	return NULL;
}

void *
pmemfile_mmap(PMEMfilepool *pfp, void *addr, size_t len,
		int prot, int flags, PMEMfile *file, off_t off)
{
	check_pfp_file(pfp, file);

	(void) addr;
	(void) len;
	(void) prot;
	(void) flags;
	(void) off;

	errno = ENOTSUP;
	return MAP_FAILED;
}

int
pmemfile_munmap(PMEMfilepool *pfp, void *addr, size_t len)
{
	check_pfp(pfp);

	(void) addr;
	(void) len;

	errno = ENOTSUP;
	return -1;
}

void *
pmemfile_mremap(PMEMfilepool *pfp, void *old_addr, size_t old_size,
			size_t new_size, int flags, void *new_addr)
{
	check_pfp(pfp);

	(void) old_addr;
	(void) new_addr;
	(void) old_size;
	(void) new_size;
	(void) flags;

	errno = ENOTSUP;
	return MAP_FAILED;
}

int
pmemfile_msync(PMEMfilepool *pfp, void *addr, size_t len, int flags)
{
	check_pfp(pfp);

	(void) addr;
	(void) len;
	(void) flags;

	errno = ENOTSUP;
	return -1;
}

int
pmemfile_mprotect(PMEMfilepool *pfp, void *addr, size_t len, int prot)
{
	check_pfp(pfp);

	(void) addr;
	(void) len;
	(void) prot;

	errno = ENOTSUP;
	return -1;
}
