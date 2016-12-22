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

void
PMEMFILE_STATS(PMEMfilepool *pfp)
{
	struct pmemfile_stats stats;
	pmemfile_stats(pfp, &stats);

	UT_OUT("inodes %u dirs %u block_arrays %u inode_arrays %u blocks %u",
			stats.inodes, stats.dirs, stats.block_arrays,
			stats.inode_arrays, stats.blocks);
}

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
PMEMFILE_LINK(PMEMfilepool *pfp, const char *oldpath, const char *newpath)
{
	int ret = pmemfile_link(pfp, oldpath, newpath);
	UT_ASSERTeq(ret, 0);
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
PMEMFILE_LSTAT(PMEMfilepool *pfp, const char *path, struct stat *buf)
{
	int ret = pmemfile_lstat(pfp, path, buf);
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

void
PMEMFILE_RENAME(PMEMfilepool *pfp, const char *old_path, const char *new_path)
{
	int ret = pmemfile_rename(pfp, old_path, new_path);
	UT_ASSERTeq(ret, 0);
}

static const char *
timespec_to_str(const struct timespec *t)
{
	char *s = asctime(localtime(&t->tv_sec));
	s[strlen(s) - 1] = 0;
	return s;
}

void
PMEMFILE_PRINT_FILES64(PMEMfilepool *pfp, PMEMfile *dir, void *dirp,
		unsigned length, int print_attrs)
{
	struct stat statbuf;
	char *buf = (void *)dirp;

	for (unsigned i = 0; i < length; ) {
		i += 8;
		i += 8;

		unsigned short int reclen = *(unsigned short *)&buf[i];
		i += 2;

		char type = *(char *)&buf[i];
		i += 1;

		PMEMFILE_FSTATAT(pfp, dir, buf + i, &statbuf, 0);
		if (type == DT_REG)
			UT_ASSERTeq(S_ISREG(statbuf.st_mode), 1);
		else if (type == DT_DIR)
			UT_ASSERTeq(S_ISDIR(statbuf.st_mode), 1);
		else
			UT_ASSERT(0);

		UT_OUT("%c%c%c%c%c%c%c%c%c%c %ld %d %d %6ld %s %s",
				type == DT_DIR ? 'd' : '-',
				statbuf.st_mode & S_IRUSR ? 'r' : '-',
				statbuf.st_mode & S_IWUSR ? 'w' : '-',
				statbuf.st_mode & S_IXUSR ? 'x' : '-',
				statbuf.st_mode & S_IRGRP ? 'r' : '-',
				statbuf.st_mode & S_IWGRP ? 'w' : '-',
				statbuf.st_mode & S_IXGRP ? 'x' : '-',
				statbuf.st_mode & S_IROTH ? 'r' : '-',
				statbuf.st_mode & S_IWOTH ? 'w' : '-',
				statbuf.st_mode & S_IXOTH ? 'x' : '-',
				statbuf.st_nlink,
				print_attrs ? statbuf.st_uid : 0,
				print_attrs ? statbuf.st_gid : 0,
				statbuf.st_size,
				print_attrs ? timespec_to_str(&statbuf.st_mtim)
						: "",
				buf + i);
		i += reclen;
		i -= 8 + 8 + 2 + 1;
	}
}

static void
_PMEMFILE_LIST_FILES(PMEMfilepool *pfp, const char *path, const char *txt,
		int print_attr)
{
	UT_OUT("LIST_FILES start, %s", txt);
	PMEMfile *f = PMEMFILE_OPEN(pfp, path, O_DIRECTORY | O_RDONLY);

	char buf[32758];
	while (1) {
		int r = pmemfile_getdents64(pfp, f, (void *)buf, sizeof(buf));
		UT_ASSERT(r >= 0);
		if (r == 0)
			break;

		PMEMFILE_PRINT_FILES64(pfp, f, buf, (unsigned)r, print_attr);
	}

	PMEMFILE_CLOSE(pfp, f);
	UT_OUT("LIST_FILES end,   %s", txt);
}

void
PMEMFILE_LIST_FILES(PMEMfilepool *pfp, const char *path, const char *txt)
{
	_PMEMFILE_LIST_FILES(pfp, path, txt, 0);
}

void
PMEMFILE_LIST_FILES_WITH_ATTRS(PMEMfilepool *pfp, const char *path,
		const char *txt)
{
	_PMEMFILE_LIST_FILES(pfp, path, txt, 1);
}

void
PMEMFILE_ASSERT_EMPTY_DIR(PMEMfilepool *pfp, const char *path)
{
	PMEMfile *f = PMEMFILE_OPEN(pfp, path, O_DIRECTORY | O_RDONLY);
	int dot = 0, dotdot = 0;

	char buf[32758];
	while (1) {
		int r = pmemfile_getdents64(pfp, f, (void *)buf, sizeof(buf));
		UT_ASSERT(r >= 0);
		if (r == 0)
			break;

		for (unsigned i = 0; i < (unsigned)r; ) {
			i += 8;
			i += 8;

			unsigned short int reclen = *(unsigned short *)&buf[i];
			i += 2;

			i += 1;

			if (strcmp(buf + i, ".") == 0)
				dot++;
			else if (strcmp(buf + i, "..") == 0)
				dotdot++;
			else
				UT_FATAL("unexpected file %s", buf + i);

			i += reclen;
			i -= 8 + 8 + 2 + 1;
		}
	}

	PMEMFILE_CLOSE(pfp, f);

	UT_ASSERTeq(dot, 1);
	UT_ASSERTeq(dotdot, 1);
}
