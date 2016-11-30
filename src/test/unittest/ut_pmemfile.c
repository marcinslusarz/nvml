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
 * ut_pmemfile.c -- unit test utility functions for pmemfile
 */

#include "pmemfile_test.h"
#include "unittest.h"

PMEMfilepool *
PMEMFILE_MKFS(const char *path)
{
	PMEMfilepool *pfp = pmemfile_mkfs(path,
			1024 * 1024 * 1024 /* PMEMOBJ_MIN_POOL */,
			S_IWUSR | S_IRUSR);
	if (!pfp)
		UT_FATAL("!pmemfile_mkfs: %s", path);
	return pfp;
}

PMEMfile *
PMEMFILE_OPEN(PMEMfilepool *pfp, const char *path, int flags, ...)
{
	va_list ap;
	mode_t mode;

	va_start(ap, flags);
	mode = va_arg(ap, mode_t);
	PMEMfile *f = pmemfile_open(pfp, path, flags, mode);
	va_end(ap);

	UT_ASSERTne(f, NULL);
	return f;
}

PMEMfile *
PMEMFILE_OPENAT(PMEMfilepool *pfp, PMEMfile *dir, const char *path, int flags,
		...)
{
	va_list ap;
	mode_t mode;

	va_start(ap, flags);
	mode = va_arg(ap, mode_t);
	PMEMfile *f = pmemfile_openat(pfp, dir, path, flags, mode);
	va_end(ap);

	UT_ASSERTne(f, NULL);
	return f;
}

ssize_t
PMEMFILE_WRITE(PMEMfilepool *pfp, PMEMfile *file, const void *buf,
		size_t count, ssize_t expected, ...)
{
	ssize_t ret = pmemfile_write(pfp, file, buf, count);
	UT_ASSERTeq(ret, expected);
	if (expected < 0) {
		va_list ap;
		int expected_errno;

		va_start(ap, expected);
		expected_errno = va_arg(ap, int);
		va_end(ap);

		UT_ASSERTeq(errno, expected_errno);
	}

	return ret;
}

void
PMEMFILE_CLOSE(PMEMfilepool *pfp, PMEMfile *file)
{
	pmemfile_close(pfp, file);
}

void
PMEMFILE_CREATE(PMEMfilepool *pfp, const char *path, int flags, mode_t mode)
{
	PMEMFILE_CLOSE(pfp, PMEMFILE_OPEN(pfp, path, flags | O_CREAT, mode));
}

void
PMEMFILE_UNLINK(PMEMfilepool *pfp, const char *path)
{
	int ret = pmemfile_unlink(pfp, path);
	UT_ASSERTeq(ret, 0);
}

ssize_t
PMEMFILE_READ(PMEMfilepool *pfp, PMEMfile *file, void *buf, size_t count,
		ssize_t expected, ...)
{
	ssize_t ret = pmemfile_read(pfp, file, buf, count);
	UT_ASSERTeq(ret, expected);
	if (expected < 0) {
		va_list ap;
		int expected_errno;

		va_start(ap, expected);
		expected_errno = va_arg(ap, int);
		va_end(ap);

		UT_ASSERTeq(errno, expected_errno);
	}

	return ret;
}

off_t
PMEMFILE_LSEEK(PMEMfilepool *pfp, PMEMfile *file, off_t offset, int whence,
		off_t expected)
{
	off_t ret = pmemfile_lseek(pfp, file, offset, whence);

	UT_ASSERTeq(ret, expected);

	return ret;
}

ssize_t
PMEMFILE_FILE_SIZE(PMEMfilepool *pfp, PMEMfile *file, ssize_t expected_size)
{
	struct stat buf;
	int ret = pmemfile_fstat(pfp, file, &buf);
	UT_ASSERTeq(ret, 0);
	if (expected_size >= 0)
		UT_ASSERTeq(buf.st_size, expected_size);
	return buf.st_size;
}

ssize_t
PMEMFILE_PATH_SIZE(PMEMfilepool *pfp, const char *path, ssize_t expected_size)
{
	struct stat buf;
	int ret = pmemfile_stat(pfp, path, &buf);
	UT_ASSERTeq(ret, 0);
	if (expected_size >= 0)
		UT_ASSERTeq(buf.st_size, expected_size);
	return buf.st_size;
}

void
PMEMFILE_STAT(PMEMfilepool *pfp, const char *path, struct stat *buf)
{
	int ret = pmemfile_stat(pfp, path, buf);
	UT_ASSERTeq(ret, 0);
}

void
PMEMFILE_FSTAT(PMEMfilepool *pfp, PMEMfile *file, struct stat *buf)
{
	int ret = pmemfile_fstat(pfp, file, buf);
	UT_ASSERTeq(ret, 0);
}

void
PMEMFILE_FSTATAT(PMEMfilepool *pfp, PMEMfile *dir, const char *path,
		struct stat *buf, int flags)
{
	int ret = pmemfile_fstatat(pfp, dir, path, buf, flags);
	UT_ASSERTeq(ret, 0);
}

void
PMEMFILE_MKDIR(PMEMfilepool *pfp, const char *path, mode_t mode)
{
	int ret = pmemfile_mkdir(pfp, path, mode);
	UT_ASSERTeq(ret, 0);
}

void
PMEMFILE_RMDIR(PMEMfilepool *pfp, const char *path)
{
	int ret = pmemfile_rmdir(pfp, path);
	UT_ASSERTeq(ret, 0);
}

void
PMEMFILE_CHDIR(PMEMfilepool *pfp, const char *path)
{
	int ret = pmemfile_chdir(pfp, path);
	UT_ASSERTeq(ret, 0);
}

void
PMEMFILE_FCHDIR(PMEMfilepool *pfp, PMEMfile *dir)
{
	int ret = pmemfile_fchdir(pfp, dir);
	UT_ASSERTeq(ret, 0);
}

char *
PMEMFILE_GETCWD(PMEMfilepool *pfp, char *buf, size_t size, const char *cmp)
{
	char *ret = pmemfile_getcwd(pfp, buf, size);
	UT_ASSERTne(ret, NULL);
	if (cmp && strcmp(ret, cmp) != 0)
		UT_FATAL("%s != %s", ret, cmp);
	return ret;
}
