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
 * preload.c - The main code controlling the preloadable version of pmemfile. To
 * understand the code start from the routine pmemfile_preload_constructor() -
 * this should run before the application starts, and while there is only a
 * single thread of execution in the process. To understand the syscall
 * routing logic look at the routine hook(...), this is called by the libc
 * syscall intercepting code every time libc would issue a syscall instruction.
 * The hook() routine decides if a syscall should be handled by kernel, or
 * by pmemfile ( and which pmemfile pool ).
 */

#include <assert.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <syscall.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <setjmp.h>
#include <pthread.h>
#include <sys/stat.h>

#include <asm-generic/errno.h>

#include "libcintercept_hook_point.h"
#include "libpmemfile-core.h"
#include "util.h"

#include "fd_pool.h"

#include "preload.h"

#define SYS_CREAT_OFLAGS (O_WRONLY | O_CREAT | O_TRUNC)

static int hook(long syscall_number,
			long arg0, long arg1,
			long arg2, long arg3,
			long arg4, long arg5,
			long *result);

static bool syscall_number_filter[0x100];

static pthread_mutex_t fd_mutex = PTHREAD_MUTEX_INITIALIZER;

static struct pool_description pools[0x100];
static int pool_count;

/*
 * A separate place to keep track of fds used to hold mount points open, in
 * the previous array. The application should not be aware of these, thus
 * whenever these file descriptors are encountered during interposing, -EBADF
 * must be returned. The contents of this array does not change after startup.
 */
static bool mount_point_fds[PMEMFILE_MAX_FD + 1];

/*
 * The array fd_table is used to look up file descriptors, and find a pool, and
 * PMEM file open in that pool. When the 'file' member is NULL, the fd is
 * not used ( but might still be in the fd_pool ).
 */
struct fd_association {
	PMEMfilepool *pool;
	PMEMfile *file;
};

static struct fd_association fd_table[PMEMFILE_MAX_FD + 1];

/*
 * Keeping track of CWD. If cwd_in_pool is not negative, the CWD is inside one
 * the pmemfile pools from the point of view of the application. In this case,
 * the CWD is at the corresponding mount point from the kernel's point of
 * view.
 * If the CWD is not inside any pmemfile pool, the CWD is not being messed with.
 */
static struct pool_description *volatile cwd_pool;

static struct pool_description *
get_cwd_pool(void)
{
	return __atomic_load_n(&cwd_pool, __ATOMIC_ACQUIRE);
}

/*
 * This way the default can be overridden from the command line during
 * a build, without altering the source. As a second option, it can be
 * overridden using a environment variable at runtime.
 */
#ifndef PMEMFILE_DEFAULT_USE_SYMLINK_STRICT
#define PMEMFILE_DEFAULT_USE_SYMLINK_STRICT false
#endif

static bool use_stricter_symlink_resolver = PMEMFILE_DEFAULT_USE_SYMLINK_STRICT;

static void log_init(const char *path);
static void log_write(const char *fmt, ...)
	__attribute__((format(printf, 1, 2)));

static void establish_mount_points(const char *);
static void setup_strict_symlink_flag(const char *);
static void init_hooking(void);

static __attribute__((constructor)) void
pmemfile_preload_constructor(void)
{
	if (!libc_hook_in_process_allowed())
		return;

	log_init(getenv("PMEMFILE_PRELOAD_LOG"));

	// establish_mount_points already needs to have the flag set up
	setup_strict_symlink_flag(getenv("PMEMFILE_USE_SYMLINK_STRICT"));

	establish_mount_points(getenv("PMEMFILE_POOLS"));

	if (pool_count == 0)
		return; // No pools mounted. TODO: prevent syscall interception

	/*
	 * Must be the last step, the callback can be called anytime
	 * after the call to init_hooking()
	 */
	init_hooking();
}

static void
setup_strict_symlink_flag(const char *e)
{
	// Only overwrite the default when explicitly requested by an env var.
	if (e != NULL)
		use_stricter_symlink_resolver = (e[0] != '0');
}

static void
init_hooking(void)
{
	// todo: move this filtering to the intercepting library
	syscall_number_filter[SYS_open] = true;
	syscall_number_filter[SYS_link] = true;
	syscall_number_filter[SYS_unlink] = true;
	syscall_number_filter[SYS_write] = true;
	syscall_number_filter[SYS_read] = true;
	syscall_number_filter[SYS_lseek] = true;
	syscall_number_filter[SYS_close] = true;
	syscall_number_filter[SYS_stat] = true;
	syscall_number_filter[SYS_lstat] = true;
	syscall_number_filter[SYS_access] = true;
	syscall_number_filter[SYS_fstat] = true;
	syscall_number_filter[SYS_getdents] = true;
	syscall_number_filter[SYS_getdents64] = true;
	syscall_number_filter[SYS_getxattr] = true;
	syscall_number_filter[SYS_fgetxattr] = true;
	syscall_number_filter[SYS_lgetxattr] = true;
	syscall_number_filter[SYS_setxattr] = true;
	syscall_number_filter[SYS_fsetxattr] = true;
	syscall_number_filter[SYS_lsetxattr] = true;

	// Install the callback to be calleb by the syscall intercepting library
	intercept_hook_point = &hook;
}

static void
config_error(void)
{
	log_write("invalid config");
	fputs("Invalid pmemfile config\n", stderr);
	syscall_no_intercept(SYS_exit_group, 123);
}

static const char *parse_mount_point(struct pool_description *pool,
					const char *conf);
static const char *parse_pool_path(struct pool_description *pool,
					const char *conf);
static void open_mount_point(struct pool_description *pool);

/*
 * establish_mount_points - parse the configuration, which is expected to be a
 * semicolon separated list of path-pairs:
 * mount_point_path:pool_file_path
 * Mount point path is where the application is meant to observe a pmemfile
 * pool mounted -- this should be an actual directory accessoble by the
 * application. The pool file path should point to the path of the actual
 * pmemfile pool.
 */
static void
establish_mount_points(const char *config)
{
	assert(pool_count == 0);

	if (config == NULL || config[0] == 0) {
		log_write("No mount point");
		return;
	}

	do {
		if ((size_t)pool_count >= sizeof(pools) / sizeof(pools[0]))
			config_error();

		struct pool_description *pool_desc = pools + pool_count;

		// fetch pool_desc->mount_point
		config = parse_mount_point(pool_desc, config);

		// fetch pool_desc->poolfile_path
		config = parse_pool_path(pool_desc, config);

		// fetch pool_desc-fd, pool_desc->stat
		open_mount_point(pool_desc);

		pool_desc->pool = NULL;

		++pool_count;
	} while (config != NULL);
}

static const char *
parse_mount_point(struct pool_description *pool, const char *conf)
{
	if (conf[0] != '/') // Relative path is not allowed
		config_error();

	// There should be a colon separating the mount path from the pool path
	const char *colon = strchr(conf, ':');

	if (colon == NULL || colon == conf)
		config_error();

	if (((size_t)(colon - conf)) >= sizeof(pool->mount_point))
		config_error();

	memcpy(pool->mount_point, conf, (size_t)(colon - conf));
	pool->mount_point[colon - conf] = '\0';

	memcpy(pool->mount_point_parent, conf, (size_t)(colon - conf));
	pool->len_mount_point_parent = (size_t)(colon - conf);

	while (pool->len_mount_point_parent > 1 &&
	    pool->mount_point_parent[pool->len_mount_point_parent] != '/')
		pool->len_mount_point_parent--;

	pool->mount_point_parent[pool->len_mount_point_parent] = '\0';

	// Return a pointer to the char following the colon
	return colon + 1;
}

static const char *
parse_pool_path(struct pool_description *pool, const char *conf)
{
	if (conf[0] != '/') // Relative path is not allowed
		config_error();

	/*
	 * The path should be followed by either with a null character - in
	 * which case this is the last pool in the conf - or a semicolon.
	 */
	size_t i;
	for (i = 0; conf[i] != ';' && conf[i] != '\0'; ++i) {
		if (i >= sizeof(pool->poolfile_path) - 1)
			config_error();
		pool->poolfile_path[i] = conf[i];
	}

	pool->poolfile_path[i] = '\0';

	// Return a pointer to the char following the semicolon, or NULL.
	if (conf[i] == ';')
		return conf + i + 1;
	else
		return NULL;
}

/*
 * open_mount_point - Grab a file descriptor for the mount point, and mark it
 * in the mount_point_fds table.
 */
static void
open_mount_point(struct pool_description *pool)
{
	pool->fd = syscall_no_intercept(SYS_open, pool->mount_point,
					O_DIRECTORY | O_RDONLY, 0);

	if (pool->fd < 0)
		config_error();

	if ((size_t)pool->fd >=
	    sizeof(mount_point_fds) / sizeof(mount_point_fds[0])) {
		log_write("fd too large, sorry mate");
		syscall_no_intercept(SYS_exit_group, 123);
	}

	mount_point_fds[pool->fd] = true;

	if (syscall_no_intercept(SYS_fstat, pool->fd, &pool->stat) != 0)
		config_error();

	if (!S_ISDIR(pool->stat.st_mode))
		config_error();
}

/*
 * Return values expected by libcintercept :
 * A non-zero return value if it should execute the syscall,
 * zero return value if it should not execute the syscall, and
 * use *result value as the syscall's result.
 */
#define NOT_HOOKED 1
#define HOOKED 0

static int hook_open(long *result, long arg0, long arg1, long arg2);
static int hook_link(long *result, long arg0, long arg1);
static int hook_unlink(long *result, long arg0);
static int hook_stat(long *result, long arg0, long arg1);
static int hook_lstat(long *result, long arg0, long arg1);
static int hook_close(long *result, long fd);
static int hook_access(long *result, long arg0, long arg1);
static int hook_getxattr(long *result, long arg0);
static int hook_lgetxattr(long *result, long arg0);
static int hook_setxattr(long *result, long arg0, long arg3);
static int hook_lsetxattr(long *result, long arg0, long arg3);

static long hook_write(struct fd_association *file, char *buffer, size_t count);
static long hook_read(struct fd_association *file, char *buffer, size_t count);
static long hook_lseek(struct fd_association *file, long offset, int whence);
static long hook_fstat(struct fd_association *file, struct stat *buf);
static long hook_pread64(struct fd_association *file,
			char *buf, size_t count, off_t pos);
static long hook_pwrite64(struct fd_association *file,
			const char *buf, size_t count, off_t pos);
static long hook_getdents(struct fd_association *file,
				struct linux_dirent *dirp, unsigned count);
static long hook_getdents64(struct fd_association *file,
				struct linux_dirent64 *dirp, unsigned count);

static long hook_chdir(const char *path);

/*
 * hook_fd_syscalls - a wrapper function for syscalls that take an
 * fd as first argument.
 * This routine expects a few things:
 *
 * The syscall is already known to be one listed here.
 * The fd is already resolved to a PMEMfile pointer.
 * Race conditions are taken care of by the caller --
 *  the fd_mutex is locked while calling hook_fd_syscalls
 */
static long
hook_fd_syscalls(long syscall_number, struct fd_association *file,
			long fd, long arg1,
			long arg2, long arg3,
			long arg4, long arg5)
{
	(void) fd;
	(void) arg4;
	(void) arg5;

	if (syscall_number == SYS_write)
		return hook_write(file, (char *)arg1, (size_t)arg2);
	else if (syscall_number == SYS_read)
		return hook_read(file, (char *)arg1, (size_t)arg2);
	else if (syscall_number == SYS_lseek)
		return hook_lseek(file, arg1, (int)arg2);
	else if (syscall_number == SYS_fstat)
		return hook_fstat(file, (struct stat *)arg1);
	else if (syscall_number == SYS_pread64)
		return hook_pread64(file, (char *)arg1,
					(size_t)arg2, (off_t)arg3);
	else if (syscall_number == SYS_pwrite64)
		return hook_pwrite64(file, (const char *)arg1,
					(size_t)arg2, (off_t)arg3);
	else if (syscall_number == SYS_getdents)
		return hook_getdents(file,
		    (struct linux_dirent *)arg1, (unsigned)arg2);
	else if (syscall_number == SYS_getdents64)
		return hook_getdents64(file,
		    (struct linux_dirent64 *)arg1, (unsigned)arg2);
	else if (syscall_number == SYS_fgetxattr)
		return 0;
	else if (syscall_number == SYS_fsetxattr && arg3 == 0)
		return 0;
	else if (syscall_number == SYS_fsetxattr && arg3 != 0)
		return -ENOTSUP;
	else if (syscall_number == SYS_fsync)
		return 0;
	else
		assert(0);
}

static int
hook(long syscall_number,
			long arg0, long arg1,
			long arg2, long arg3,
			long arg4, long arg5,
			long *result)
{
	assert(pool_count > 0);

	if (syscall_number == SYS_chdir) {
		// todo: handle fchdir, chroot
		*result = hook_chdir((const char *)arg0);
		return HOOKED;
	}

	// todo: move this filtering to the intercepting library
	if (syscall_number < 0 ||
	    (uint64_t)syscall_number >= ARRAY_SIZE(syscall_number_filter) ||
	    !syscall_number_filter[syscall_number])
		return NOT_HOOKED;

	/*
	 * The three syscalls below accept one or more path strings as
	 * arguments. If the pool is not yet initialized, these potentially
	 * trigger an initialization.
	 */
	if (syscall_number == SYS_open)
		return hook_open(result, arg0, arg1, arg2);
	if (syscall_number == SYS_creat)
		return hook_open(result, arg0, SYS_CREAT_OFLAGS, arg2);
	if (syscall_number == SYS_link)
		return hook_link(result, arg0, arg1);
	if (syscall_number == SYS_unlink)
		return hook_unlink(result, arg0);
	if (syscall_number == SYS_stat)
		return hook_stat(result, arg0, arg1);
	if (syscall_number == SYS_lstat)
		return hook_lstat(result, arg0, arg1);
	if (syscall_number == SYS_access)
		return hook_access(result, arg0, arg1);
	if (syscall_number == SYS_getxattr)
		return hook_getxattr(result, arg0);
	if (syscall_number == SYS_lgetxattr)
		return hook_lgetxattr(result, arg0);
	if (syscall_number == SYS_setxattr)
		return hook_setxattr(result, arg0, arg3);
	if (syscall_number == SYS_lsetxattr)
		return hook_lsetxattr(result, arg0, arg3);

	if (syscall_number == SYS_close)
		return hook_close(result, arg0);

	/* The syscalls handled below accept an fd as first argument. */

	long fd = arg0;

	// Is the fd handled by pmemfile? If not, let the kernel handle it.
	if (!fd_pool_has_allocated(fd))
		return NOT_HOOKED;

	pthread_mutex_lock(&fd_mutex);

	/*
	 * Is this fd associated with a PMEMfile pointer, or is
	 * it just sitting in the pool?
	 */
	struct fd_association *file = fd_table + fd;

	if (file != NULL)
		*result = hook_fd_syscalls(syscall_number, file,
				fd, arg1, arg2, arg3, arg4, arg5);
	else
		*result = -EBADF;

	pthread_mutex_unlock(&fd_mutex);

	return HOOKED;
}

static int
hook_open(long *result, long arg0, long flags, long mode)
{
	struct path_component where;
	const char *path_arg = (const char *)arg0;
	enum resolve_last_or_not follow_last;

	log_write("%s(\"%s\")", __func__, path_arg);

	if ((flags & O_NOFOLLOW) != 0)
		follow_last = no_resolve_last_slink;
	else if ((flags & O_CREAT) != 0)
		follow_last = no_resolve_last_slink;
	else
		follow_last = resolve_last_slink;

	resolve_path(get_cwd_pool(), path_arg, &where, follow_last);

	if (where.error_code != 0) {
		*result = where.error_code;
		return HOOKED;
	}

	if (where.pool == NULL)
		return NOT_HOOKED; // Not pmemfile resident path

	pthread_mutex_lock(&fd_mutex); // It is pmemfile resident path

	long fd = fd_pool_fetch_new_fd();

	if (fd < 0) {
		*result = fd;
	} else {
		PMEMfile *file;

		file = pmemfile_open(where.pool->pool, where.path,
				((int)flags) & ~O_NONBLOCK, (mode_t)mode);

		log_write("pmemfile_open(\"%s\") = %p",
		    where.path, (void *)file);

		if (file != NULL) {
			fd_table[fd].pool = where.pool->pool;
			fd_table[fd].file = file;
			*result = fd;
		} else {
			fd_pool_release_fd(fd);
			*result = -errno;
		}
	}

	pthread_mutex_unlock(&fd_mutex);

	return HOOKED;
}

static int
hook_close(long *result, long fd)
{
	int is_hooked;

	if (fd_pool_has_allocated(fd)) {

		pthread_mutex_lock(&fd_mutex);

		is_hooked = HOOKED;

		if (fd_table[fd].file != NULL) {
			assert(fd_table[fd].pool != NULL);

			fd_pool_release_fd(fd);
			pmemfile_close(fd_table[fd].pool, fd_table[fd].file);

			log_write("pmemfile_close(%p, %p)",
			    (void *)fd_table[fd].pool,
			    (void *)fd_table[fd].file);

			fd_table[fd].file = NULL;
			fd_table[fd].pool = NULL;

			*result = 0;
		} else {
			*result = -EBADF;
		}

		pthread_mutex_unlock(&fd_mutex);

	} else {
		/*
		 * It is possible, that we allocate this fd in another thread
		 * right now, we pass it to the kernel, who closes
		 * it ( closes our /dev/null ).
		 *
		 * Or is it??
		 */
		is_hooked = NOT_HOOKED;
	}

	return is_hooked;
}

static long
hook_write(struct fd_association *file, char *buffer, size_t count)
{
	long result = pmemfile_write(file->pool, file->file, buffer, count);

	if (result < 0)
		result = -errno;

	log_write("pmemfile_write(%p, %p, %p, %zu) = %ld",
	    (void *)file->pool, (void *)file->file,
	    (void *)buffer, count, result);

	return result;
}

static long
hook_read(struct fd_association *file, char *buffer, size_t count)
{
	long result = pmemfile_read(file->pool, file->file, buffer, count);

	if (result < 0)
		result = -errno;

	log_write("pmemfile_read(%p, %p, %p, %zu) = %ld",
	    (void *)file->pool, (void *)file->file,
	    (void *)buffer, count, result);

	return result;
}

static long
hook_lseek(struct fd_association *file, long offset, int whence)
{

	long result = pmemfile_lseek(file->pool, file->file, offset, whence);

	log_write("pmemfile_lseek(%p, %p, %lu, %d) = %ld",
	    (void *)file->pool, (void *)file->file, offset, whence, result);

	if (result < 0)
		result = -errno;

	return result;
}

static int
hook_link(long *result, long arg0, long arg1)
{
	struct path_component where_old;
	struct path_component where_new;

	struct pool_description *cwd = get_cwd_pool();

	resolve_path(cwd, (const char *)arg0, &where_old, resolve_last_slink);
	resolve_path(cwd, (const char *)arg1, &where_new, resolve_last_slink);

	if (where_old.pool == NULL || where_new.pool == NULL)
		return NOT_HOOKED;

	if (where_old.pool != where_new.pool) {
		*result = -ENOTSUP;
	}

	*result = pmemfile_link(where_old.pool->pool,
				where_old.path, where_new.path);

	log_write("pmemfile_link(%p, \"%s\", \"%s\") = %ld",
	    (void *)where_old.pool->pool,
	    where_old.path, where_new.path, *result);

	if (*result < 0)
		*result = -errno;

	return HOOKED;
}

static int
hook_unlink(long *result, long arg0)
{
	struct path_component where;

	resolve_path(get_cwd_pool(), (const char *)arg0,
	    &where, resolve_last_slink);

	if (where.pool == NULL)
		return NOT_HOOKED;

	*result = pmemfile_unlink(where.pool->pool, where.path);

	log_write("pmemfile_unlink(%p, \"%s\") = %ld",
	    (void *)where.pool->pool, where.path, *result);

	if (*result < 0)
		*result = -errno;

	return HOOKED;
}

static long
hook_chdir(const char *path)
{
	struct path_component where;

	log_write("%s: \"%s\"", __func__, path);

	pthread_mutex_lock(&fd_mutex);

	resolve_path(cwd_pool, path, &where, resolve_last_slink);
	if (where.pool == NULL) {
		syscall_no_intercept(SYS_chdir, where.path);
	} else {
		pmemfile_chdir(where.pool->pool, where.path);
	}
	cwd_pool = where.pool;

	log_write("%s : \"%s\"", __func__, where.path);

	pthread_mutex_unlock(&fd_mutex);

	return 0;
}

static long log_fd = -1;

static void
log_init(const char *path)
{
	if (path != NULL)
		log_fd = syscall_no_intercept(SYS_open, path,
				O_CREAT | O_RDWR, 0700);
}

static void
log_write(const char *fmt, ...)
{
	if (log_fd < 0)
		return;

	char buf[0x1000];
	int len;
	va_list ap;

	va_start(ap, fmt);
	len = vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
	va_end(ap);


	if (len < 1)
		return;

	buf[len++] = '\n';

	syscall_no_intercept(SYS_write, log_fd, buf, len);
}

static void
open_new_pool(struct pool_description *p)
{
	if (p->pool == NULL) {
		PMEMfilepool *pfp = pmemfile_pool_open(p->poolfile_path);
		__atomic_store_n(&p->pool, pfp, __ATOMIC_SEQ_CST);
	}
}

/*
 * With each virtual mount point an inode number is stored, and this
 * function can be used to lookup a mount point by inode number.
 */
struct pool_description *
lookup_pd_by_inode(__ino_t inode)
{
	for (int i = 0; i < pool_count; ++i) {
		struct pool_description *p = pools + i;
		if (p->stat.st_ino == inode)  {
			PMEMfilepool *pfp;

			pfp = __atomic_load_n(&p->pool, __ATOMIC_SEQ_CST);
			if (pfp == NULL) {
				pthread_mutex_lock(&fd_mutex);
				open_new_pool(p);
				pthread_mutex_unlock(&fd_mutex);
			}
			return p;
		}
	}

	return NULL;
}

struct pool_description *
lookup_pd_by_path(const char *path)
{
	for (int i = 0; i < pool_count; ++i) {
		struct pool_description *p = pools + i;
		// TODO: first compare the lengths of the two strings to
		// avoid strcmp calls
		if (strcmp(p->mount_point, path) == 0)  {
			PMEMfilepool *pfp;

			pfp = __atomic_load_n(&p->pool, __ATOMIC_SEQ_CST);
			if (pfp == NULL) {
				pthread_mutex_lock(&fd_mutex);
				open_new_pool(p);
				pthread_mutex_unlock(&fd_mutex);
			}
			return p;
		}
	}

	return NULL;
}

static int
hook_stat(long *result, long arg0, long arg1)
{
	struct path_component where;

	resolve_path(get_cwd_pool(), (const char *)arg0,
	    &where, resolve_last_slink);

	if (where.pool == NULL)
		return NOT_HOOKED;

	*result = pmemfile_stat(where.pool->pool,
			where.path, (struct stat *)arg1);

	log_write("pmemfile_stat(%p, \"%s\", %p) = %ld",
	    (void *)where.pool->pool, where.path, (void *)arg1, *result);

	if (*result < 0)
		*result = -errno;

	return HOOKED;
}

static int
hook_lstat(long *result, long arg0, long arg1)
{
	struct path_component where;

	resolve_path(get_cwd_pool(), (const char *)arg0,
	    &where, no_resolve_last_slink);

	if (where.pool == NULL)
		return NOT_HOOKED;

	*result = pmemfile_stat(where.pool->pool,
			where.path, (struct stat *)arg1);

	log_write("pmemfile_lstat(%p, \"%s\", %p) = %ld",
	    (void *)where.pool->pool, where.path, (void *)arg1, *result);

	if (*result < 0)
		*result = -errno;

	return HOOKED;
}

static long
hook_fstat(struct fd_association *file, struct stat *buf)
{
	long result = pmemfile_fstat(file->pool, file->file, buf);

	if (result < 0)
		result = -errno;

	log_write("pmemfile_fstat(%p, %p, %p) = %ld",
	    (void *)file->pool, (void *)file->file,
	    (void *)buf, result);

	return result;
}

static long
hook_pread64(struct fd_association *file, char *buf, size_t count, off_t pos)
{
	long result = pmemfile_pread(file->pool, file->file, buf, count, pos);

	if (result < 0)
		result = -errno;

	// format specifier for off_t ??
	// well, it is likely always 64 bit uint here
	log_write("pmemfile_pread(%p, %p, %p, %zu, %zu) = %ld",
	    (void *)file->pool, (void *)file->file,
	    (void *)buf, count, pos, result);

	return result;
}

static long
hook_pwrite64(struct fd_association *file,
		const char *buf, size_t count, off_t pos)
{
	long result = pmemfile_pwrite(file->pool, file->file, buf, count, pos);

	if (result < 0)
		result = -errno;

	log_write("pmemfile_pwrite(%p, %p, %p, %zu, %zu) = %ld",
	    (void *)file->pool, (void *)file->file,
	    (const void *)buf, count, pos, result);

	return result;
}

static int
hook_access(long *result, long arg0, long arg1)
{
	struct path_component where;

	resolve_path(get_cwd_pool(), (const char *)arg0,
	    &where, no_resolve_last_slink);

	if (where.pool == NULL)
		return NOT_HOOKED;

	*result = pmemfile_access(where.pool->pool,
			where.path, (mode_t)arg1);

	log_write("pmemfile_lstat(%p, \"%s\", %ld) = %ld",
	    (void *)where.pool->pool, where.path, arg1, *result);

	if (*result < 0)
		*result = -errno;

	return HOOKED;
}

static long
hook_getdents(struct fd_association *file,
		struct linux_dirent *dirp, unsigned count)
{
	long result = pmemfile_getdents(file->pool, file->file, dirp, count);

	if (result < 0)
		result = -errno;

	log_write("pmemfile_getdents(%p, %p, %p, %u) = %ld",
	    (void *)file->pool, (void *)file->file,
	    (const void *)dirp, count, result);

	return result;
}

static long
hook_getdents64(struct fd_association *file,
		struct linux_dirent64 *dirp, unsigned count)
{
	long result = pmemfile_getdents64(file->pool, file->file, dirp, count);

	if (result < 0)
		result = -errno;

	log_write("pmemfile_getdents64(%p, %p, %p, %u) = %ld",
	    (void *)file->pool, (void *)file->file,
	    (const void *)dirp, count, result);

	return result;
}

static int
hook_getxattr(long *result, long arg0)
{
	struct path_component where;

	resolve_path(get_cwd_pool(), (const char *)arg0,
	    &where, resolve_last_slink);

	if (where.pool == NULL)
		return NOT_HOOKED;

	*result = 0;

	return HOOKED;
}

static int
hook_lgetxattr(long *result, long arg0)
{
	struct path_component where;

	resolve_path(get_cwd_pool(), (const char *)arg0,
	    &where, no_resolve_last_slink);

	if (where.pool == NULL)
		return NOT_HOOKED;

	*result = 0;

	return HOOKED;
}

/*
 * Syscall setxattr is not supported, unless the size argument ( arg3 )
 * is zero. Pretend to support setting an empty string as xattr.
 */
static int
hook_setxattr(long *result, long arg0, long arg3)
{
	struct path_component where;

	resolve_path(get_cwd_pool(), (const char *)arg0,
	    &where, resolve_last_slink);

	if (where.pool == NULL)
		return NOT_HOOKED;

	if (arg3 != 0)
		*result = -ENOTSUP;
	else
		*result = 0;

	return HOOKED;
}

static int
hook_lsetxattr(long *result, long arg0, long arg3)
{
	struct path_component where;

	resolve_path(get_cwd_pool(), (const char *)arg0,
	    &where, no_resolve_last_slink);

	if (where.pool == NULL)
		return NOT_HOOKED;

	if (arg3 != 0)
		*result = -ENOTSUP;
	else
		*result = 0;

	return HOOKED;
}
