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
 * file_core_dirs.c -- unit test for directories
 */

#include "pmemfile_test.h"
#include "unittest.h"

static const char *
timespec_to_str(const struct timespec *t)
{
	char *s = asctime(localtime(&t->tv_sec));
	s[strlen(s) - 1] = 0;
	return s;
}

static void
dump_stat(struct stat *st, const char *path)
{
	UT_OUT("path:       %s", path);
	UT_OUT("st_dev:     0x%lx", st->st_dev);
	UT_OUT("st_ino:     %ld", st->st_ino);
	UT_OUT("st_mode:    0%o", st->st_mode);
	UT_OUT("st_nlink:   %lu", st->st_nlink);
	UT_OUT("st_uid:     %u", st->st_uid);
	UT_OUT("st_gid:     %u", st->st_gid);
	UT_OUT("st_rdev:    0x%lx", st->st_rdev);
	UT_OUT("st_size:    %ld", st->st_size);
	UT_OUT("st_blksize: %ld", st->st_blksize);
	UT_OUT("st_blocks:  %ld", st->st_blocks);
	UT_OUT("st_atim:    %ld.%.9ld, %s", st->st_atim.tv_sec,
			st->st_atim.tv_nsec, timespec_to_str(&st->st_atim));
	UT_OUT("st_mtim:    %ld.%.9ld, %s", st->st_mtim.tv_sec,
			st->st_mtim.tv_nsec, timespec_to_str(&st->st_mtim));
	UT_OUT("st_ctim:    %ld.%.9ld, %s", st->st_ctim.tv_sec,
			st->st_ctim.tv_nsec, timespec_to_str(&st->st_ctim));
	UT_OUT("---");
}

static int
stat_and_dump(PMEMfilepool *pfp, const char *path)
{
	struct stat st;
	int ret = pmemfile_stat(pfp, path, &st);
	if (ret)
		return ret;

	dump_stat(&st, path);
	return 0;
}

struct linux_dirent64 {
	uint64_t       d_ino;
	uint64_t       d_off;
	unsigned short d_reclen;
	unsigned char  d_type;
	char           d_name[];
};

static void
list_root(PMEMfilepool *pfp, int expected_files)
{
	PMEMfile *f = PMEMFILE_OPEN(pfp, "/", O_DIRECTORY | O_RDONLY);

	char buf[32 * 1024];
	char path[PATH_MAX];
	struct linux_dirent64 *d = (void *)buf;
	int r = pmemfile_getdents64(pfp, f, (void *)buf, sizeof(buf));
	int num_files = 0;

	while ((uintptr_t)d < (uintptr_t)&buf[r]) {
		num_files++;
		UT_OUT("ino: 0x%lx, off: 0x%lx, len: %d, type: %d, name: \"%s\"",
				d->d_ino, d->d_off, d->d_reclen, d->d_type,
				d->d_name);
		sprintf(path, "/%s", d->d_name);
		stat_and_dump(pfp, path);
		d = (void *)(((char *)d) + d->d_reclen);
	}

	PMEMFILE_CLOSE(pfp, f);

	UT_ASSERTeq(num_files, expected_files);
}

static void
test1(PMEMfilepool *pfp)
{
	PMEMfile *f;
	char buf[1001];
	_pmemfile_list_root(pfp, "before");
	memset(buf, 0xff, sizeof(buf));

	for (int i = 0; i < 100; ++i) {
		sprintf(buf, "/file%04d", i);

		f = PMEMFILE_OPEN(pfp, buf, O_CREAT | O_EXCL | O_WRONLY, 0644);

		PMEMFILE_WRITE(pfp, f, buf, i, i);

		PMEMFILE_CLOSE(pfp, f);

		list_root(pfp, i + 1 + 2);
	}

}

int
main(int argc, char *argv[])
{
	START(argc, argv, "file_core_dirs");

	if (argc < 2)
		UT_FATAL("usage: %s file-name", argv[0]);

	const char *path = argv[1];

	PMEMfilepool *pfp = PMEMFILE_MKFS(path);

	test1(pfp);

	pmemfile_pool_close(pfp);

	DONE(NULL);
}
