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
 * file_core_basic.c -- unit test for pmemfile_*
 */

#include "unittest.h"

static PMEMfilepool *
create_pool(const char *path)
{
	PMEMfilepool *pfp = pmemfile_mkfs(path, PMEMOBJ_MIN_POOL,
			S_IWUSR | S_IRUSR);
	if (!pfp)
		UT_FATAL("!pmemfile_mkfs: %s", path);
	return pfp;
}

static PMEMfilepool *
open_pool(const char *path)
{
	PMEMfilepool *pfp = pmemfile_pool_open(path);
	if (!pfp)
		UT_FATAL("!pmemfile_pool_open %s", path);
	return pfp;
}

static void
test_open_create_close(PMEMfilepool *pfp)
{
	PMEMfile *f1, *f2;

	_pmemfile_list_root(pfp, "test_open_create_close start, files: . ..");
	_pmemfile_stats(pfp);

	/* NULL file name */
	errno = 0;
	f1 = pmemfile_open(pfp, NULL, O_CREAT, 0777);
	UT_ASSERTeq(f1, NULL);
	UT_ASSERTeq(errno, EFAULT);

	/* path does not start with "/" */
	errno = 0;
	f1 = pmemfile_open(pfp, "aaa", O_CREAT, 0777);
	UT_ASSERTeq(f1, NULL);
	UT_ASSERTeq(errno, EINVAL);

	/* file does not exist */
	errno = 0;
	f1 = pmemfile_open(pfp, "/aaa", 0, 0);
	UT_ASSERTeq(f1, NULL);
	UT_ASSERTeq(errno, ENOENT);

	/* successful create */
	f1 = pmemfile_open(pfp, "/aaa", O_CREAT | O_EXCL, 0777);
	UT_ASSERTne(f1, NULL);

	pmemfile_close(pfp, f1);

	/* file already exists */
	errno = 0;
	f1 = pmemfile_open(pfp, "/aaa", O_CREAT | O_EXCL, 0777);
	UT_ASSERTeq(f1, NULL);
	UT_ASSERTeq(errno, EEXIST);



	/* file does not exist */
	errno = 0;
	f2 = pmemfile_open(pfp, "/bbb", 0, 0);
	UT_ASSERTeq(f2, NULL);
	UT_ASSERTeq(errno, ENOENT);

	/* successful create */
	f2 = pmemfile_open(pfp, "/bbb", O_CREAT | O_EXCL, 0777);
	UT_ASSERTne(f2, NULL);



	/* successful open */
	f1 = pmemfile_open(pfp, "/aaa", 0, 0);
	UT_ASSERTne(f1, NULL);

	pmemfile_close(pfp, f2);

	pmemfile_close(pfp, f1);

	_pmemfile_list_root(pfp,
			"test_open_create_close end, files: . .. aaa bbb");
	_pmemfile_stats(pfp);

	pmemfile_pool_close(pfp);
}

/*
 * At this point (after test_open_create_close) these files should exist in
 * root:
 * - .
 * - ..
 * - aaa
 * - bbb
 */

static void
test_open_close(const char *path)
{
	PMEMfilepool *pfp = open_pool(path);
	_pmemfile_list_root(pfp, "test_open_close, files: . .. aaa bbb");
	_pmemfile_stats(pfp);
	pmemfile_pool_close(pfp);
}

static void
test_link(const char *path)
{
	PMEMfilepool *pfp = open_pool(path);

	int ret;

	_pmemfile_list_root(pfp,
		"test_link end, files: "
		". .. aaa bbb");

	/* successful link */
	ret = pmemfile_link(pfp, "/aaa", "/aaa.link");
	UT_ASSERTeq(ret, 0);

	_pmemfile_list_root(pfp,
		"test_link end, files: "
		". .. aaa bbb aaa.link");

	/* destination already exists */
	errno = 0;
	ret = pmemfile_link(pfp, "/aaa", "/aaa.link");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, EEXIST);

	_pmemfile_list_root(pfp,
		"test_link end, files: "
		". .. aaa bbb aaa.link");

	/* source does not exist */
	errno = 0;
	ret = pmemfile_link(pfp, "/aaaaaaaaaaaa", "/aaa.linkXXX");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENOENT);

	_pmemfile_list_root(pfp,
		"test_link end, files: "
		". .. aaa bbb aaa.link");

	/* successful link from link */
	ret = pmemfile_link(pfp, "/aaa.link", "/aaa2.link");
	UT_ASSERTeq(ret, 0);

	_pmemfile_list_root(pfp,
		"test_link end, files: "
		". .. aaa bbb aaa.link aaa2.link");

	/* another successful link */
	ret = pmemfile_link(pfp, "/bbb", "/bbb2.link");
	UT_ASSERTeq(ret, 0);

	_pmemfile_list_root(pfp,
		"test_link end, files: "
		". .. aaa bbb aaa.link aaa2.link bbb2.link");
	_pmemfile_stats(pfp);

	pmemfile_pool_close(pfp);
}

/*
 * At this point (after test_link) these files should exist in root:
 * - .
 * - ..
 * - aaa
 * - bbb
 * - aaa.link (hardlink to aaa)
 * - aaa2.link (hardlink to aaa)
 * - bbb2.link (hardlink to bbb)
 */

static void
test_unlink(const char *path)
{
	PMEMfilepool *pfp = open_pool(path);

	int ret;
	PMEMfile *f1;

	f1 = pmemfile_open(pfp, "/bbb2.link", 0, 0);
	UT_ASSERTne(f1, NULL);
	pmemfile_close(pfp, f1);

	ret = pmemfile_unlink(pfp, "/bbb2.link");
	UT_ASSERTeq(ret, 0);

	errno = 0;
	ret = pmemfile_unlink(pfp, "/bbb2.link");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENOENT);

	errno = 0;
	f1 = pmemfile_open(pfp, "/bbb2.link", 0, 0);
	UT_ASSERTeq(f1, NULL);
	UT_ASSERTeq(errno, ENOENT);

	errno = 0;
	ret = pmemfile_unlink(pfp, "/bbb.notexists");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENOENT);


	f1 = pmemfile_open(pfp, "/bbb", 0, 0);
	UT_ASSERTne(f1, NULL);
	ret = pmemfile_unlink(pfp, "/bbb");
	UT_ASSERTeq(ret, 0);
	pmemfile_close(pfp, f1);

	errno = 0;
	f1 = pmemfile_open(pfp, "/bbb", 0, 0);
	UT_ASSERTeq(f1, NULL);
	UT_ASSERTeq(errno, ENOENT);

	errno = 0;
	ret = pmemfile_unlink(pfp, "/..");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, EISDIR);

	errno = 0;
	ret = pmemfile_unlink(pfp, "/.");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, EISDIR);

	_pmemfile_list_root(pfp,
			"test_unlink end, files: . .. aaa aaa.link aaa2.link");
	_pmemfile_stats(pfp);

	pmemfile_pool_close(pfp);
}

/*
 * At this point (after test_unlink) these files should exist in root:
 * - .
 * - ..
 * - aaa
 * - aaa.link
 * - aaa2.link
 *
 * And these files should not exist:
 * - bbb
 * - bbb2.link
 */

int
main(int argc, char *argv[])
{
	START(argc, argv, "file_core_basic");

	if (argc < 2)
		UT_FATAL("usage: %s file-name", argv[0]);

	const char *path = argv[1];

	test_open_create_close(create_pool(path));

	/* open and close pool to test there are no inode leaks */
	test_open_close(path);

	test_link(path);

	test_unlink(path);

	DONE(NULL);
}
