/*
 * Copyright 2017, Intel Corporation
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
 * file_core_symlinks.c -- unit test for pmemfile_symlink and co
 */
#define _GNU_SOURCE
#include "unittest.h"
#include "pmemfile_test.h"

static PMEMfilepool *
create_pool(const char *path)
{
	PMEMfilepool *pfp = pmemfile_mkfs(path, PMEMOBJ_MIN_POOL,
			S_IWUSR | S_IRUSR);
	if (!pfp)
		UT_FATAL("!pmemfile_mkfs: %s", path);
	return pfp;
}

#if 0
static PMEMfilepool *
open_pool(const char *path)
{
	PMEMfilepool *pfp = pmemfile_pool_open(path);
	if (!pfp)
		UT_FATAL("!pmemfile_pool_open %s", path);
	return pfp;
}
#endif

static void
test0(PMEMfilepool *pfp)
{
	PMEMFILE_STATS(pfp);

	PMEMFILE_ASSERT_EMPTY_DIR(pfp, "/");

	PMEMFILE_CREATE(pfp, "/file1", 0, 0644);

	PMEMFILE_MKDIR(pfp, "/dir", 0755);

	PMEMFILE_SYMLINK(pfp, "/file1", "/dir/sym1-exists");
	PMEMFILE_READLINK(pfp, "/dir/sym1-exists", "/file1");
	PMEMFILE_READLINKAT(pfp, "/dir", "sym1-exists", "/file1");
	PMEMFILE_READLINKAT(pfp, "/", "dir/sym1-exists", "/file1");

	PMEMFILE_SYMLINK(pfp, "/file2", "/dir/sym2-not_exists");
	PMEMFILE_READLINK(pfp, "/dir/sym2-not_exists", "/file2");
	PMEMFILE_READLINKAT(pfp, "/dir", "sym2-not_exists", "/file2");

	PMEMFILE_SYMLINK(pfp, "../file1", "/dir/sym3-exists-relative");
	PMEMFILE_READLINK(pfp, "/dir/sym3-exists-relative", "../file1");
	PMEMFILE_READLINKAT(pfp, "/dir", "sym3-exists-relative", "../file1");

	PMEMFILE_SYMLINK(pfp, "../file2", "/dir/sym4-not_exists-relative");
	PMEMFILE_READLINK(pfp, "/dir/sym4-not_exists-relative", "../file2");
	PMEMFILE_READLINKAT(pfp, "/dir", "sym4-not_exists-relative",
			"../file2");

	PMEMFILE_LIST_FILES(pfp, "/", "/");
	PMEMFILE_LIST_FILES(pfp, "/dir", "/dir");

	int ret;

	ret = pmemfile_symlink(pfp, "whatever", "/not-exisiting-dir/xxx");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENOENT);

	ret = pmemfile_symlink(pfp, "whatever", "/file1/xxx");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENOTDIR);

	ret = pmemfile_symlink(pfp, "whatever", "/dir/sym1-exists");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, EEXIST);

	char tmp[4096];
	memset(tmp, '0', 4095);
	tmp[4095] = 0;

	ret = pmemfile_symlink(pfp, tmp, "/dir/lalala");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENAMETOOLONG);

	PMEMFILE_MKDIR(pfp, "/deleted-dir", 0755);
	PMEMfile *deleted_dir = PMEMFILE_OPEN(pfp, "/deleted-dir", O_DIRECTORY);
	PMEMFILE_RMDIR(pfp, "/deleted-dir");

	ret = pmemfile_symlinkat(pfp, "whatever", deleted_dir, "lalala");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENOENT);
	PMEMFILE_CLOSE(pfp, deleted_dir);

	PMEMfile *f = PMEMFILE_OPEN(pfp, "/file1", O_RDONLY);
	ret = pmemfile_symlinkat(pfp, "whatever", f, "lalala");
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENOTDIR);


	char buf[PATH_MAX];

	ret = pmemfile_readlink(pfp, "/not-existing-dir/xxx", buf, PATH_MAX);
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENOENT);

	ret = pmemfile_readlink(pfp, "/file1/xxx", buf, PATH_MAX);
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENOTDIR);

	ret = pmemfile_readlink(pfp, "/file1", buf, PATH_MAX);
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, EINVAL);

	ret = pmemfile_readlinkat(pfp, f, "lalala", buf, PATH_MAX);
	UT_ASSERTeq(ret, -1);
	UT_ASSERTeq(errno, ENOTDIR);

	PMEMFILE_CLOSE(pfp, f);

	PMEMFILE_UNLINK(pfp, "/dir/sym1-exists");
	PMEMFILE_UNLINK(pfp, "/dir/sym2-not_exists");
	PMEMFILE_UNLINK(pfp, "/dir/sym3-exists-relative");
	PMEMFILE_UNLINK(pfp, "/dir/sym4-not_exists-relative");
	PMEMFILE_UNLINK(pfp, "/file1");
	PMEMFILE_RMDIR(pfp, "/dir");

	PMEMFILE_STATS(pfp);

	pmemfile_pool_close(pfp);
}

int
main(int argc, char *argv[])
{
	START(argc, argv, "file_core_symlinks");

	if (argc < 2)
		UT_FATAL("usage: %s file-name", argv[0]);

	const char *path = argv[1];

	test0(create_pool(path));

	DONE(NULL);
}
