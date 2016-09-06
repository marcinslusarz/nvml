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
 * file_core_rw.c -- unit test for pmemfile_read & pmemfile_write
 */

#include "unittest.h"

static PMEMfilepool *
create_pool(const char *path)
{
	PMEMfilepool *pfp = pmemfile_mkfs(path,
			1024 * 1024 * 1024 /* PMEMOBJ_MIN_POOL */,
			S_IWUSR | S_IRUSR);
	if (!pfp)
		UT_FATAL("!pmemfile_mkfs: %s", path);
	return pfp;
}

static void
test1(PMEMfilepool *pfp)
{
	PMEMfile *f = pmemfile_open(pfp, "/file1", O_CREAT | O_EXCL | O_WRONLY,
			0644);
	UT_ASSERTne(f, NULL);

	_pmemfile_list_root(pfp, "/file1 0");

	const char *data = "Marcin S";
	char data2[4096];
	char bufFF[4096], buf00[4096];
	int len = strlen(data) + 1;
	memset(bufFF, 0xff, sizeof(bufFF));
	memset(buf00, 0x00, sizeof(buf00));

	int ret = pmemfile_write(pfp, f, data, len);
	UT_ASSERTeq(ret, len);

	_pmemfile_list_root(pfp, "/file1 9");

	ret = pmemfile_read(pfp, f, data2, len);
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, EBADF); /* file is opened write-only */

	pmemfile_close(pfp, f);



	f = pmemfile_open(pfp, "/file1", O_RDONLY, 0);
	UT_ASSERTne(f, NULL);

	memset(data2, 0xff, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, len);
	UT_ASSERTeq(ret, len);

	UT_ASSERTeq(memcmp(data, data2, len), 0);
	UT_ASSERTeq(memcmp(data2 + len, bufFF, sizeof(data2) - len), 0);

	ret = pmemfile_write(pfp, f, data, len);
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, EBADF); /* file is opened read-only */

	memset(data2, 0, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, len);
	UT_ASSERTeq(ret, 0); /* end of file */

	pmemfile_close(pfp, f);



	f = pmemfile_open(pfp, "/file1", O_RDONLY, 0);
	UT_ASSERTne(f, NULL);

	memset(data2, 0xff, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, sizeof(data2));
	UT_ASSERTeq(ret, len);

	UT_ASSERTeq(memcmp(data, data2, len), 0);
	UT_ASSERTeq(memcmp(data2 + len, bufFF, sizeof(data2) - len), 0);

	pmemfile_close(pfp, f);



	f = pmemfile_open(pfp, "/file1", O_RDONLY, 0);
	UT_ASSERTne(f, NULL);

	memset(data2, 0xff, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, 5);
	UT_ASSERTeq(ret, 5);

	UT_ASSERTeq(memcmp(data, data2, 5), 0);
	UT_ASSERTeq(memcmp(data2 + 5, bufFF, sizeof(data2) - 5), 0);

	memset(data2, 0xff, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, 15);
	UT_ASSERTeq(ret, 4);

	UT_ASSERTeq(memcmp(data + 5, data2, 4), 0);
	UT_ASSERTeq(memcmp(data2 + 4, bufFF, sizeof(data2) - 4), 0);

	pmemfile_close(pfp, f);



	f = pmemfile_open(pfp, "/file1", O_RDWR, 0);
	UT_ASSERTne(f, NULL);

	ret = pmemfile_write(pfp, f, "pmem", 4);
	UT_ASSERTeq(ret, 4);

	memset(data2, 0xff, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, sizeof(data2));
	UT_ASSERTeq(ret, 5);

	UT_ASSERTeq(memcmp(data + 4, data2, 5), 0);
	UT_ASSERTeq(memcmp(data2 + 5, bufFF, sizeof(data2) - 5), 0);

	pmemfile_close(pfp, f);



	_pmemfile_list_root(pfp, "/file1 9");



	f = pmemfile_open(pfp, "/file1", O_RDWR, 0);
	UT_ASSERTne(f, NULL);

	memset(data2, 0xff, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, sizeof(data2));
	UT_ASSERTeq(ret, 9);

	UT_ASSERTeq(memcmp("pmem", data2, 4), 0);
	UT_ASSERTeq(memcmp(data + 4, data2 + 4, 5), 0);
	UT_ASSERTeq(memcmp(data2 + 9, bufFF, sizeof(data2) - 9), 0);

	pmemfile_close(pfp, f);



	f = pmemfile_open(pfp, "/file1", O_RDWR, 0);
	UT_ASSERTne(f, NULL);

	ret = pmemfile_lseek(pfp, f, 0, SEEK_CUR);
	UT_ASSERTeq(ret, 0);


	ret = pmemfile_lseek(pfp, f, 3, SEEK_CUR);
	UT_ASSERTeq(ret, 3);
	memset(data2, 0xff, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, sizeof(data2));
	UT_ASSERTeq(ret, 6);

	UT_ASSERTeq(memcmp("min S\0", data2, 6), 0);
	UT_ASSERTeq(memcmp(data2 + 6, bufFF, sizeof(data2) - 6), 0);

	ret = pmemfile_lseek(pfp, f, 0, SEEK_CUR);
	UT_ASSERTeq(ret, 9);


	ret = pmemfile_lseek(pfp, f, -7, SEEK_CUR);
	UT_ASSERTeq(ret, 2);
	memset(data2, 0xff, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, sizeof(data2));
	UT_ASSERTeq(ret, 7);

	UT_ASSERTeq(memcmp("emin S\0", data2, 7), 0);
	UT_ASSERTeq(memcmp(data2 + 7, bufFF, sizeof(data2) - 7), 0);

	ret = pmemfile_lseek(pfp, f, 0, SEEK_CUR);
	UT_ASSERTeq(ret, 9);


	ret = pmemfile_lseek(pfp, f, -3, SEEK_END);
	UT_ASSERTeq(ret, 6);
	memset(data2, 0xff, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, sizeof(data2));
	UT_ASSERTeq(ret, 3);

	UT_ASSERTeq(memcmp(" S\0", data2, 3), 0);
	UT_ASSERTeq(memcmp(data2 + 3, bufFF, sizeof(data2) - 3), 0);

	ret = pmemfile_lseek(pfp, f, 0, SEEK_CUR);
	UT_ASSERTeq(ret, 9);


	ret = pmemfile_lseek(pfp, f, 100, SEEK_END);
	UT_ASSERTeq(ret, 9 + 100);
	ret = pmemfile_write(pfp, f, "XYZ\0", 4);
	UT_ASSERTeq(ret, 4);

	ret = pmemfile_lseek(pfp, f, 0, SEEK_CUR);
	UT_ASSERTeq(ret, 9 + 100 + 4);

	ret = pmemfile_lseek(pfp, f, 0, SEEK_SET);
	UT_ASSERTeq(ret, 0);
	memset(data2, 0xff, sizeof(data2));
	ret = pmemfile_read(pfp, f, data2, sizeof(data2));
	UT_ASSERTeq(ret, 9 + 100 + 4);

	UT_ASSERTeq(memcmp("pmemin S\0", data2, 9), 0);
	UT_ASSERTeq(memcmp(data2 + 9, buf00, 100), 0);
	UT_ASSERTeq(memcmp("XYZ\0", data2 + 9 + 100, 4), 0);
	UT_ASSERTeq(memcmp(data2 + 9 + 100 + 4, bufFF,
			sizeof(data2) - 9 - 100 - 4), 0);

	ret = pmemfile_lseek(pfp, f, 0, SEEK_CUR);
	UT_ASSERTeq(ret, 9 + 100 + 4);


	pmemfile_close(pfp, f);


	_pmemfile_list_root(pfp, "/file1 9+100+4=113");

	_pmemfile_stats(pfp);

	ret = pmemfile_unlink(pfp, "/file1");
	UT_ASSERTeq(ret, 0);

	_pmemfile_stats(pfp);




	f = pmemfile_open(pfp, "/file1", O_CREAT | O_EXCL | O_RDWR, 0644);
	UT_ASSERTne(f, NULL);

	ret = pmemfile_write(pfp, f, buf00, 4096);
	UT_ASSERTeq(ret, 4096);
	ret = _pmemfile_file_size(pfp, f);
	UT_ASSERTeq(ret, 4096);
	ret = pmemfile_write(pfp, f, bufFF, 4096);
	UT_ASSERTeq(ret, 4096);
	ret = _pmemfile_file_size(pfp, f);
	UT_ASSERTeq(ret, 8192);
	ret = pmemfile_lseek(pfp, f, 0, SEEK_CUR);
	UT_ASSERTeq(ret, 8192);
	ret = pmemfile_lseek(pfp, f, 4096, SEEK_SET);
	UT_ASSERTeq(ret, 4096);
	ret = _pmemfile_file_size(pfp, f);
	UT_ASSERTeq(ret, 8192);
	ret = pmemfile_read(pfp, f, data2, 4096);
	UT_ASSERTeq(ret, 4096);
	ret = _pmemfile_file_size(pfp, f);
	UT_ASSERTeq(ret, 8192);

	pmemfile_close(pfp, f);


	_pmemfile_list_root(pfp, "/file1 8192");
	_pmemfile_stats(pfp);

	ret = pmemfile_unlink(pfp, "/file1");
	UT_ASSERTeq(ret, 0);
}

static void
test2(PMEMfilepool *pfp)
{

	char buf00[128], bufFF[128], bufd[4096 * 4], buftmp[4096 * 4];
	int ret;

	memset(buf00, 0x00, sizeof(buf00));
	memset(bufFF, 0xFF, sizeof(bufFF));

	for (int i = 0; i < sizeof(bufd); ++i)
		bufd[i] = rand() % 255;

	PMEMfile *f = pmemfile_open(pfp, "/file1", O_CREAT | O_EXCL | O_WRONLY,
			0644);
	UT_ASSERTne(f, NULL);

#define LEN (sizeof(bufd) - 1000)
#define LOOPS ((800 * 1024 * 1024) / LEN)
	for (int i = 0; i < LOOPS; ++i) {
		ret = pmemfile_write(pfp, f, bufd, LEN);
		UT_ASSERTeq(ret, LEN);
	}

	pmemfile_close(pfp, f);
	_pmemfile_list_root(pfp, "/file1 ~800MB");
	_pmemfile_stats(pfp);

	f = pmemfile_open(pfp, "/file1", O_RDONLY, 0);
	UT_ASSERTne(f, NULL);

	for (int i = 0; i < LOOPS; ++i) {
		memset(buftmp, 0, sizeof(buftmp));
		ret = pmemfile_read(pfp, f, buftmp, LEN);
		UT_ASSERTeq(ret, LEN);
		if (memcmp(buftmp, bufd, LEN) != 0)
			UT_ASSERT(0);
	}
#undef LEN
	ret = pmemfile_read(pfp, f, buftmp, 1023);
	UT_ASSERTeq(ret, 0);

	pmemfile_close(pfp, f);

	ret = pmemfile_unlink(pfp, "/file1");
	UT_ASSERTeq(ret, 0);
}

int
main(int argc, char *argv[])
{
	START(argc, argv, "file_core_rw");

	if (argc < 2)
		UT_FATAL("usage: %s file-name", argv[0]);

	const char *path = argv[1];

	PMEMfilepool *pfp = create_pool(path);

	_pmemfile_stats(pfp);

	_pmemfile_list_root(pfp, "no files");

	test1(pfp);
	_pmemfile_list_root(pfp, "no files");
	_pmemfile_stats(pfp);

	test2(pfp);
	_pmemfile_list_root(pfp, "no files");
	_pmemfile_stats(pfp);

	pmemfile_pool_close(pfp);

	DONE(NULL);
}
