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
 * libpmemfile-core-stubs.h -- definitions of not yet implemented
 * libpmemfile-core entry points. Do not use these. All the routines
 * just set errno to ENUTSUP.
 * This is header file, and the symbols exported are used while designing
 * the interface of the library.
 * Everything here is subject to change at any time.
 *
 * If/when some pmemfile functionality is implemented, the corresponding
 * header declarations should be moved to the libpmemfile-core.h header file.
 *
 * This file is expected to be removed eventually.
 */
#ifndef LIBPMEMFILE_CORE_H
#error Never include this header file directly
#endif

#ifndef LIBPMEMFILE_CORE_STUBS_H
#define LIBPMEMFILE_CORE_STUBS_H

int pmemfile_chdir(PMEMfilepool *, const char *path);
int pmemfile_fchdir(PMEMfilepool *, PMEMfile *);
int pmemfile_readlink(PMEMfilepool *, const char *path,
			char *buf, size_t buf_len);
char *pmemfile_getcwd(PMEMfilepool *, char *buf, size_t buf_len);
ssize_t pmemfile_pread(PMEMfilepool *, PMEMfile *file,
			char *buf, size_t count, off_t pos);
ssize_t pmemfile_pwrite(PMEMfilepool *, PMEMfile *file,
			const char *buf, size_t count, off_t pos);
int pmemfile_access(PMEMfilepool *, const char *path, mode_t mode);
int pmemfile_sync(PMEMfilepool *);
int pmemfile_fdatasync(PMEMfilepool *, PMEMfile *);
int pmemfile_rename(PMEMfilepool *, const char *old_path, const char *new_path);
int pmemfile_renameat(PMEMfilepool *, PMEMfile *old_at, const char *old_path,
				PMEMfile *new_at, const char *new_path);
int pmemfile_openat(PMEMfilepool *, PMEMfile *at,
		const char *path, int oflag, mode_t mode);
int pmemfile_flock(PMEMfilepool *, PMEMfile *file, int operation);
int pmemfile_truncate(PMEMfilepool *, const char *path, off_t length);
int pmemfile_ftruncate(PMEMfilepool *, PMEMfile *file, off_t length);
int pmemfile_mkdir(PMEMfilepool *, const char *path, mode_t mode);
int pmemfile_mkdirat(PMEMfilepool *, PMEMfile *file,
			const char *path, mode_t mode);
int pmemfile_rmdir(PMEMfilepool *, const char *path);
int pmemfile_symlink(PMEMfilepool *, const char *path1, const char *path2);
int pmemfile_symlinkat(PMEMfilepool *, const char *path1,
				PMEMfile *at, const char *path2);
int pmemfile_chmod(PMEMfilepool *, const char *path, mode_t mode);
int pmemfile_fchmod(PMEMfilepool *, PMEMfile *, mode_t mode);

// De we need dup, dup2 in corelib? Maybe, dunno...
PMEMfile *pmemfile_dup(PMEMfilepool *, PMEMfile *);
PMEMfile *pmemfile_dup2(PMEMfilepool *, PMEMfile *file, PMEMfile *file2);

// Memory mapping pmemfiles, these need extra suppport in the preloadable lib
void *pmemfile_mmap(PMEMfilepool *, void *addr, size_t len,
		int prot, int flags, PMEMfile *file, off_t off);
int pmemfile_munmap(PMEMfilepool *, void *addr, size_t len);
void *pmemfile_mremap(PMEMfilepool *, void *old_addr, size_t old_size,
			size_t new_size, int flags, void *new_addr);
int pmemfile_msync(PMEMfilepool *, void *addr, size_t len, int flags);
int pmemfile_mprotect(PMEMfilepool *, void *addr, size_t len, int prot);

#endif
