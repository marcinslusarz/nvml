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


#define _GNU_SOURCE

#include "intercept_util.h"
#include "intercept.h"


#include <assert.h>
#include <inttypes.h>
#include <ctype.h>
#include <stddef.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <syscall.h>
#include <sys/mman.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <sched.h>

static long log_fd = -1;

void *
xmmap_anon(size_t size)
{
	void *addr = (void *) syscall_no_intercept(SYS_mmap,
				NULL, size,
				PROT_READ | PROT_WRITE,
				MAP_PRIVATE | MAP_ANON, -1, 0);

	if (addr == MAP_FAILED)
		xabort();

	return addr;
}

void *
xmremap(void *addr, size_t old, size_t new)
{
	addr = (void *) syscall_no_intercept(SYS_mremap, addr,
				old, new, MREMAP_MAYMOVE);

	if (addr == MAP_FAILED)
		xabort();

	return addr;
}

long
xlseek(long fd, unsigned long off, int whence)
{
	long result = syscall_no_intercept(SYS_lseek, fd, off, whence);

	if (result < 0)
		xabort();

	return result;
}

void
xread(long fd, void *buffer, size_t size)
{
	if (syscall_no_intercept(SYS_read, fd,
	    (long)buffer, (long)size) != (long)size)
		xabort();
}


void
intercept_setup_log(const char *path_base)
{
	char full_path[0x400];
	const char *path = path_base;

	if (path_base == NULL)
		return;

	if (getenv("INTERCEPT_LOG_NO_PID") == NULL) {
		snprintf(full_path, sizeof(full_path), "%s.%ld",
			path_base,
			syscall_no_intercept(SYS_getpid));

		path = full_path;
	}

	log_fd = syscall_no_intercept(SYS_open, path, O_CREAT | O_RDWR, 0700);

	if (log_fd < 0)
		xabort();
}

static char *
print_open_flags(char *buffer, long flags)
{
	char *c = buffer;

	*c = 0;

	if (flags == 0)
		return c + sprintf(c, "O_RDONLY");

#ifdef O_EXEC
	if ((flags & O_EXEC) == O_EXEC)
		c += sprintf(c, "O_EXEC | ");
#endif
	if ((flags & O_RDWR) == O_RDWR)
		c += sprintf(c, "O_RDWR | ");
	if ((flags & O_WRONLY) == O_WRONLY)
		c += sprintf(c, "O_WRONLY | ");
	if ((flags & (O_WRONLY|O_RDWR)) == 0)
		c += sprintf(c, "O_RDONLY | ");
#ifdef O_SEARCH
	if ((flags & O_SEARCH) = O_SEARCH)
		c += sprintf(c, "O_SEARCH | ");
#endif
	if ((flags & O_APPEND) == O_APPEND)
		c += sprintf(c, "O_APPEND | ");
	if ((flags & O_CLOEXEC) == O_CLOEXEC)
		c += sprintf(c, "O_CLOEXEC | ");
	if ((flags & O_CREAT) == O_CREAT)
		c += sprintf(c, "O_CREAT | ");
	if ((flags & O_DIRECTORY) == O_DIRECTORY)
		c += sprintf(c, "O_DIRECTORY | ");
	if ((flags & O_DSYNC) == O_DSYNC)
		c += sprintf(c, "O_DSYNC | ");
	if ((flags & O_EXCL) == O_EXCL)
		c += sprintf(c, "O_EXCL | ");
	if ((flags & O_NOCTTY) == O_NOCTTY)
		c += sprintf(c, "O_NOCTTY | ");
	if ((flags & O_NOFOLLOW) == O_NOFOLLOW)
		c += sprintf(c, "O_NOFOLLOW | ");
	if ((flags & O_NONBLOCK) == O_NONBLOCK)
		c += sprintf(c, "O_NONBLOCK | ");
	if ((flags & O_RSYNC) == O_RSYNC)
		c += sprintf(c, "O_RSYNC | ");
	if ((flags & O_SYNC) == O_SYNC)
		c += sprintf(c, "O_SYNC | ");
	if ((flags & O_TRUNC) == O_TRUNC)
		c += sprintf(c, "O_TRUNC | ");
#ifdef O_TTY_INIT
	if ((flags & O_TTY_INIT) == O_TTY_INIT)
		c += sprintf(c, "O_TTY_INIT | ");
#endif

#ifdef O_EXEC
	flags &= ~O_EXEC;
#endif
#ifdef O_TTY_INIT
	flags &= ~O_TTY_INIT;
#endif
#ifdef O_SEARCH
	flags &= ~O_SEARCH;
#endif

	flags &= ~(O_RDONLY | O_RDWR | O_WRONLY | O_APPEND |
	    O_CLOEXEC | O_CREAT | O_DIRECTORY | O_DSYNC | O_EXCL |
	    O_NOCTTY | O_NOFOLLOW | O_NONBLOCK | O_RSYNC | O_SYNC |
	    O_TRUNC);

	if (flags != 0) {
		/*
		 * Some values in the flag were not recognized, just print the
		 * raw number.
		 * e.g.: "O_RDONLY | O_NONBLOCK | 0x9876"
		 */
		c += sprintf(c, "0x%lx", flags);
	} else if (c != buffer) {
		/*
		 * All bits in flag were parsed, and the pointer c does not
		 * point to the start of the buffer, therefore some text was
		 * written already, with a separator on the end. Remove the
		 * trailing three characters: " | "
		 *
		 * e.g.: "O_RDONLY | O_NONBLOCK | " -> "O_RDONLY | O_NONBLOCK"
		 */
		c -= 3;
		*c = 0;
	}

	return c;
}

static void
print_clone_flags(char buffer[static 0x100], long flags)
{
	char *c = buffer;

	*c = '\0';

	if ((flags & CLONE_CHILD_CLEARTID) == CLONE_CHILD_CLEARTID)
		c += sprintf(c, "CLONE_CHILD_CLEARTID | ");
	if ((flags & CLONE_CHILD_SETTID) == CLONE_CHILD_SETTID)
		c += sprintf(c, "CLONE_CHILD_SETTID | ");
	if ((flags & CLONE_FILES) == CLONE_FILES)
		c += sprintf(c, "CLONE_FILES | ");
	if ((flags & CLONE_FS) == CLONE_FS)
		c += sprintf(c, "CLONE_FS | ");
	if ((flags & CLONE_IO) == CLONE_IO)
		c += sprintf(c, "CLONE_IO | ");
	if ((flags & CLONE_NEWCGROUP) == CLONE_NEWCGROUP)
		c += sprintf(c, "CLONE_NEWCGROUP | ");
	if ((flags & CLONE_NEWIPC) == CLONE_NEWIPC)
		c += sprintf(c, "CLONE_NEWIPC | ");
	if ((flags & CLONE_NEWNET) == CLONE_NEWNET)
		c += sprintf(c, "CLONE_NEWNET | ");
	if ((flags & CLONE_NEWNS) == CLONE_NEWNS)
		c += sprintf(c, "CLONE_NEWNS | ");
	if ((flags & CLONE_NEWPID) == CLONE_NEWPID)
		c += sprintf(c, "CLONE_NEWPID | ");
	if ((flags & CLONE_NEWUSER) == CLONE_NEWUSER)
		c += sprintf(c, "CLONE_NEWUSER | ");
	if ((flags & CLONE_NEWUTS) == CLONE_NEWUTS)
		c += sprintf(c, "CLONE_NEWUTS | ");
	if ((flags & CLONE_PARENT) == CLONE_PARENT)
		c += sprintf(c, "CLONE_PARENT | ");
	if ((flags & CLONE_PARENT_SETTID) == CLONE_PARENT_SETTID)
		c += sprintf(c, "CLONE_PARENT_SETTID | ");
	if ((flags & CLONE_PTRACE) == CLONE_PTRACE)
		c += sprintf(c, "CLONE_PTRACE | ");
	if ((flags & CLONE_SETTLS) == CLONE_SETTLS)
		c += sprintf(c, "CLONE_SETTLS | ");
	if ((flags & CLONE_SIGHAND) == CLONE_SIGHAND)
		c += sprintf(c, "CLONE_SIGHAND | ");
	if ((flags & CLONE_SYSVSEM) == CLONE_SYSVSEM)
		c += sprintf(c, "CLONE_SYSVSEM | ");
	if ((flags & CLONE_THREAD) == CLONE_THREAD)
		c += sprintf(c, "CLONE_THREAD | ");
	if ((flags & CLONE_UNTRACED) == CLONE_UNTRACED)
		c += sprintf(c, "CLONE_UNTRACED | ");
	if ((flags & CLONE_VFORK) == CLONE_VFORK)
		c += sprintf(c, "CLONE_VFORK | ");
	if ((flags & CLONE_VM) == CLONE_VM)
		c += sprintf(c, "CLONE_VM | ");

	if (c != buffer) {
		c -= 3;
		*c = '\0';
	} else {
		sprintf(buffer, "%ld", flags);
	}
}

/*
 * The formats of syscall arguments, as they should appear in logs.
 */
enum {
	f_dec,		// decimal number
	f_oct_mode,	// mode_t, octal number ( open, chmod, etc.. )
	f_hex,		// hexadecimal number, with zero padding e.g. pointers
	f_str,		// zero terminated string
	f_buf,		// buffer, with a given size
	f_open_flags	// only used for oflags in open, openat
};

static char *print_syscall(char *buffer, const char *name, unsigned args, ...);

/*
 * Log syscalls after intercepting, in a human readable ( as much as possible )
 * format. The format is either:
 *
 * offset -- name(arguments...) = result
 *
 * where the name is known, or
 *
 * offset -- syscall(syscall_number, arguments...) = result
 *
 * where the name is not known.
 *
 * Each line starts with the offset of the syscall instruction in libc's ELF.
 * This should be easy to pass to addr2line, to see in what symbol in libc
 * the syscall was initiated.
 *
 * E.g.:
 * 0xdaea2 -- fstat(1, 0x7ffd115206f0) = 0
 *
 * Each syscall should be logged after being executed, so the result can be
 * logged as well.
 */
void
intercept_log_syscall(const char *libpath, long nr, long arg0, long arg1,
			long arg2, long arg3,
			long arg4, long arg5, unsigned long syscall_offset,
			long result)
{
	if (log_fd < 0)
		return;

	char buffer[0x1000];
	char *buf = buffer;

	buf += sprintf(buf, "%s 0x%lx -- ", libpath, syscall_offset);

	if (nr == SYS_read) {
		buf = print_syscall(buf, "read", 3,
				f_dec, arg0,
				f_buf, arg2, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_write) {
		buf = print_syscall(buf, "write", 3,
				f_dec, arg0,
				f_buf, arg2, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_open) {
		buf = print_syscall(buf, "open", 3,
				f_str, arg0,
				f_open_flags, arg1,
				f_oct_mode, arg2,
				result);
	} else if (nr == SYS_close) {
		buf = print_syscall(buf, "close", 1,
				f_dec, arg0,
				result);
	} else if (nr == SYS_stat) {
		buf = print_syscall(buf, "stat", 2,
				f_str, arg0,
				f_hex, arg1,
				result);
	} else if (nr == SYS_fstat) {
		buf = print_syscall(buf, "fstat", 2,
				f_dec, arg0,
				f_hex, arg1,
				result);
	} else if (nr == SYS_lstat) {
		buf = print_syscall(buf, "lstat", 2,
				f_str, arg0,
				f_hex, arg1,
				result);
	} else if (nr == SYS_lseek) {
		buf = print_syscall(buf, "lstat", 3,
				f_dec, arg0,
				f_dec, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_mmap) {
		buf = print_syscall(buf, "mmap", 6,
				f_hex, arg0,
				f_dec, arg1,
				f_dec, arg2,
				f_dec, arg3,
				f_dec, arg4,
				f_hex, arg5,
				result);
	} else if (nr == SYS_mprotect) {
		buf = print_syscall(buf, "mprotect", 3,
				f_hex, arg0,
				f_dec, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_munmap) {
		buf = print_syscall(buf, "munmap", 2,
				f_hex, arg0,
				f_dec, arg1,
				result);
	} else if (nr == SYS_brk) {
		buf = print_syscall(buf, "brk", 1,
				f_dec, arg0,
				result);
	} else if (nr == SYS_ioctl) {
		buf = print_syscall(buf, "ioctl", 3,
				f_dec, arg0,
				f_dec, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_pread64) {
		buf = print_syscall(buf, "pread64", 4,
				f_dec, arg0,
				f_buf, arg2, arg1,
				f_dec, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_pwrite64) {
		buf = print_syscall(buf, "pwrite64", 4,
				f_dec, arg0,
				f_buf, arg2, arg1,
				f_dec, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_readv) {
		buf = print_syscall(buf, "readv", 3,
				f_dec, arg0,
				f_hex, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_writev) {
		buf = print_syscall(buf, "writev", 3,
				f_dec, arg0,
				f_hex, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_access) {
		buf = print_syscall(buf, "access", 2,
				f_str, arg0,
				f_dec, arg1,
				result);
	} else if (nr == SYS_mremap) {
		buf = print_syscall(buf, "mremap", 5,
				f_hex, arg0,
				f_dec, arg1,
				f_dec, arg2,
				f_dec, arg3,
				f_hex, arg4,
				result);
	} else if (nr == SYS_msync) {
		buf = print_syscall(buf, "msync", 3,
				f_hex, arg0,
				f_dec, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_dup) {
		buf = print_syscall(buf, "dup", 1,
				f_dec, arg0,
				result);
	} else if (nr == SYS_dup2) {
		buf = print_syscall(buf, "dup2", 2,
				f_dec, arg0,
				f_dec, arg1,
				result);
	} else if (nr == SYS_fcntl) {
		buf = print_syscall(buf, "fcntl", 3,
				f_dec, arg0,
				f_dec, arg1,
				f_dec, arg1,
				result);
	} else if (nr == SYS_flock) {
		buf = print_syscall(buf, "flock", 2,
				f_dec, arg0,
				f_dec, arg1,
				result);
	} else if (nr == SYS_fsync) {
		buf = print_syscall(buf, "fsync", 1,
				f_dec, arg0,
				result);
	} else if (nr == SYS_fdatasync) {
		buf = print_syscall(buf, "fdatasync", 1,
				f_dec, arg0,
				result);
	} else if (nr == SYS_truncate) {
		buf = print_syscall(buf, "truncate", 2,
				f_str, arg0,
				f_dec, arg1,
				result);
	} else if (nr == SYS_ftruncate) {
		buf = print_syscall(buf, "ftruncate", 2,
				f_dec, arg0,
				f_dec, arg1,
				result);
	} else if (nr == SYS_getdents) {
		buf = print_syscall(buf, "getdents", 3,
				f_dec, arg0,
				f_hex, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_getcwd) {
		buf = print_syscall(buf, "getcwd", 2,
				f_str, arg0,
				f_dec, arg1,
				result);
	} else if (nr == SYS_chdir) {
		buf = print_syscall(buf, "chdir", 1,
				f_str, arg0,
				result);
	} else if (nr == SYS_fchdir) {
		buf = print_syscall(buf, "fchdir", 1,
				f_dec, arg0,
				result);
	} else if (nr == SYS_rename) {
		buf = print_syscall(buf, "rename", 2,
				f_str, arg0,
				f_str, arg1,
				result);
	} else if (nr == SYS_mkdir) {
		buf = print_syscall(buf, "mkdir", 2,
				f_str, arg0,
				f_oct_mode, arg1,
				result);
	} else if (nr == SYS_rmdir) {
		buf = print_syscall(buf, "rmdir", 1,
				f_str, arg0,
				result);
	} else if (nr == SYS_creat) {
		buf = print_syscall(buf, "creat", 2,
				f_str, arg0,
				f_oct_mode, arg1,
				result);
	} else if (nr == SYS_link) {
		buf = print_syscall(buf, "link", 2,
				f_str, arg0,
				f_str, arg1,
				result);
	} else if (nr == SYS_unlink) {
		buf = print_syscall(buf, "unlink", 1,
				f_str, arg0,
				result);
	} else if (nr == SYS_symlink) {
		buf = print_syscall(buf, "symlink", 2,
				f_str, arg0,
				f_str, arg1,
				result);
	} else if (nr == SYS_readlink) {
		buf = print_syscall(buf, "readlink", 3,
				f_str, arg0,
				f_buf, arg2, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_chmod) {
		buf = print_syscall(buf, "chmod", 2,
				f_str, arg0,
				f_oct_mode, arg2,
				result);
	} else if (nr == SYS_fchmod) {
		buf = print_syscall(buf, "fchmod", 2,
				f_dec, arg0,
				f_oct_mode, arg2,
				result);
	} else if (nr == SYS_chown) {
		buf = print_syscall(buf, "chown", 3,
				f_str, arg0,
				f_dec, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_fchown) {
		buf = print_syscall(buf, "fchown", 3,
				f_dec, arg0,
				f_dec, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_lchown) {
		buf = print_syscall(buf, "lchown", 3,
				f_str, arg0,
				f_dec, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_umask) {
		buf = print_syscall(buf, "umask", 1,
				f_oct_mode, arg0,
				result);
	} else if (nr == SYS_mknod) {
		buf = print_syscall(buf, "mknod", 3,
				f_str, arg0,
				f_oct_mode, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_statfs) {
		buf = print_syscall(buf, "statfs", 2,
				f_str, arg0,
				f_hex, arg1,
				result);
	} else if (nr == SYS_chroot) {
		buf = print_syscall(buf, "chroot", 1,
				f_str, arg0,
				result);
	} else if (nr == SYS_readahead) {
		buf = print_syscall(buf, "readahead", 3,
				f_dec, arg0,
				f_dec, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_getdents64) {
		buf = print_syscall(buf, "getdents64", 3,
				f_dec, arg0,
				f_hex, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_fadvise64) {
		buf = print_syscall(buf, "fadvise64", 4,
				f_dec, arg0,
				f_dec, arg1,
				f_dec, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_openat) {
		buf = print_syscall(buf, "openat", 4,
				f_dec, arg0,
				f_str, arg1,
				f_open_flags, arg2,
				f_oct_mode, arg3,
				result);
	} else if (nr == SYS_mkdirat) {
		buf = print_syscall(buf, "mkdirat", 4,
				f_dec, arg0,
				f_str, arg1,
				f_open_flags, arg2,
				result);
	} else if (nr == SYS_mknodat) {
		buf = print_syscall(buf, "mknodat", 4,
				f_dec, arg0,
				f_str, arg1,
				f_oct_mode, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_fchownat) {
		buf = print_syscall(buf, "fchownat", 5,
				f_dec, arg0,
				f_str, arg1,
				f_dec, arg2,
				f_dec, arg3,
				f_dec, arg4,
				result);
	} else if (nr == SYS_futimesat) {
		buf = print_syscall(buf, "futimesat", 3,
				f_dec, arg0,
				f_str, arg1,
				f_hex, arg2,
				result);
	} else if (nr == SYS_newfstatat) {
		buf = print_syscall(buf, "newfstatat", 4,
				f_dec, arg0,
				f_str, arg1,
				f_hex, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_unlinkat) {
		buf = print_syscall(buf, "unlinkat", 3,
				f_dec, arg0,
				f_str, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_renameat) {
		buf = print_syscall(buf, "renameat", 4,
				f_dec, arg0,
				f_str, arg1,
				f_dec, arg2,
				f_str, arg3,
				result);
	} else if (nr == SYS_linkat) {
		buf = print_syscall(buf, "linkat", 5,
				f_dec, arg0,
				f_str, arg1,
				f_dec, arg2,
				f_str, arg3,
				f_dec, arg4,
				result);
	} else if (nr == SYS_symlinkat) {
		buf = print_syscall(buf, "symlinkat", 3,
				f_str, arg0,
				f_dec, arg1,
				f_str, arg2,
				result);
	} else if (nr == SYS_readlinkat) {
		buf = print_syscall(buf, "readlinkat", 4,
				f_dec, arg0,
				f_str, arg1,
				f_str, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_fchmodat) {
		buf = print_syscall(buf, "fchmodat", 4,
				f_dec, arg0,
				f_str, arg1,
				f_oct_mode, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_faccessat) {
		buf = print_syscall(buf, "faccessat", 4,
				f_dec, arg0,
				f_str, arg1,
				f_oct_mode, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_splice) {
		buf = print_syscall(buf, "splice", 6,
				f_dec, arg0,
				f_hex, arg1,
				f_dec, arg2,
				f_hex, arg3,
				f_dec, arg4,
				f_dec, arg5,
				result);
	} else if (nr == SYS_tee) {
		buf = print_syscall(buf, "tee", 4,
				f_dec, arg0,
				f_dec, arg1,
				f_dec, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_sync_file_range) {
		buf = print_syscall(buf, "sync_file_range", 4,
				f_dec, arg0,
				f_dec, arg1,
				f_dec, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_utimensat) {
		buf = print_syscall(buf, "utimensat", 4,
				f_dec, arg0,
				f_str, arg1,
				f_hex, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_fallocate) {
		buf = print_syscall(buf, "fallocate", 4,
				f_dec, arg0,
				f_dec, arg1,
				f_dec, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_dup3) {
		buf = print_syscall(buf, "dup3", 3,
				f_dec, arg0,
				f_dec, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_preadv) {
		buf = print_syscall(buf, "preadv", 4,
				f_dec, arg0,
				f_hex, arg1,
				f_dec, arg2,
				f_dec, arg3,
				result);
	} else if (nr == SYS_pwritev) {
		buf = print_syscall(buf, "pwritev", 3,
				f_dec, arg0,
				f_hex, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_name_to_handle_at) {
		buf = print_syscall(buf, "name_to_handle_at", 5,
				f_dec, arg0,
				f_str, arg1,
				f_hex, arg2,
				f_hex, arg3,
				f_dec, arg4,
				result);
	} else if (nr == SYS_open_by_handle_at) {
		buf = print_syscall(buf, "open_by_handle_at", 3,
				f_dec, arg0,
				f_hex, arg1,
				f_dec, arg2,
				result);
	} else if (nr == SYS_syncfs) {
		buf = print_syscall(buf, "syncfs", 1,
				f_dec, arg0,
				result);
#ifdef SYS_renameat2
	} else if (nr == SYS_renameat2) {
		buf = print_syscall(buf, "renameat2", 5,
				f_dec, arg0,
				f_str, arg1,
				f_dec, arg2,
				f_str, arg3,
				f_dec, arg4,
				result);
#endif
	} else if (nr == SYS_execve) {
		buf = print_syscall(buf, "execve", 3,
				f_str, arg0,
				f_hex, arg1,
				f_hex, arg2,
				result);
	} else if (nr == SYS_execveat) {
		buf = print_syscall(buf, "execve", 4,
				f_dec, arg0,
				f_str, arg1,
				f_hex, arg2,
				f_hex, arg3,
				result);
	} else if (nr == SYS_exit_group) {
		buf += sprintf(buf, "exit_group(%d)", (int)arg0);
	} else if (nr == SYS_exit) {
		buf += sprintf(buf, "exit(%d)", (int)arg0);
	} else if (nr == SYS_clone) {
		char sflags[0x100];

		print_clone_flags(sflags, arg0);
		buf += sprintf(buf, "clone(%s, %p, %p, %p, %ld)",
		    sflags, (void *)arg1, (void *)arg2, (void *)arg3, arg4);
	} else if (nr == SYS_fork) {
		buf = print_syscall(buf, "fork", 0, result);
	} else if (nr == SYS_vfork) {
		buf += sprintf(buf, "vfork()");
	} else if (nr == SYS_wait4) {
		buf = print_syscall(buf, "wait4", 4,
				f_dec, arg0,
				f_hex, arg1,
				f_hex, arg2,
				f_hex, arg3,
				result);
	} else {
		buf = print_syscall(buf, "syscall", 7,
				f_dec, nr,
				f_hex, arg0,
				f_hex, arg1,
				f_hex, arg2,
				f_hex, arg3,
				f_hex, arg4,
				f_hex, arg5,
				result);
	}

	*buf++ = '\n';

	intercept_log(buffer, (size_t)(buf - buffer));
}

void
intercept_log(const char *buffer, size_t len)
{
	if (log_fd >= 0)
		syscall_no_intercept(SYS_write, log_fd,
		    (long)buffer, (long)len);
}

void
intercept_logs(const char *str)
{
	size_t len = strlen(str) + 1;
	char buffer[len];

	strcpy(buffer, str);
	buffer[len - 1] = '\n';

	if (log_fd >= 0)
		syscall_no_intercept(SYS_write, log_fd,
		    (long)buffer, (long)len);
}

void
intercept_log_close(void)
{
	if (log_fd >= 0)
		syscall_no_intercept(SYS_close, log_fd);
}

static const char xdigit[16] = "0123456789abcdef";

char *
xprint_escape(char *restrict dst, const char *restrict src,
			size_t dst_size, bool zero_term, size_t src_size)
{
	char *dst_end = dst + dst_size - 5;

	if (src == NULL)
		return dst + sprintf(dst, "(null)");

	*dst++ = '"';
	while (dst < dst_end && (zero_term || src_size > 0)) {
		if (zero_term && *src == 0)
			break;

		if (*src == '\"') {
			*dst++ = '\\';
			*dst++ = '"';
		} else if (*src == '\\') {
			*dst++ = '\\';
			*dst++ = '\\';
		} else if (isprint(*src)) {
			*dst++ = *src;
		} else {
			*dst++ = '\\';
			if (*src == '\n') {
				*dst++ = 'n';
			} else if (*src == '\t') {
				*dst++ = 't';
			} else if (*src == '\r') {
				*dst++ = 'r';
			} else if (*src == '\a') {
				*dst++ = 'a';
			} else if (*src == '\b') {
				*dst++ = 'b';
			} else if (*src == '\f') {
				*dst++ = 'f';
			} else if (*src == '\v') {
				*dst++ = 'v';
			} else if (*src == '\0') {
				*dst++ = '0';
			} else {
				*dst++ = 'x';
				*dst++ = xdigit[(unsigned char)(*src) / 16];
				*dst++ = xdigit[(unsigned char)(*src) % 16];
			}

		}

		++src;

		if (!zero_term)
			--src_size;
	}

	if ((src_size > 0 && !zero_term) || (zero_term && *src != 0))
		dst += sprintf(dst, "...");

	*dst++ = '"';
	*dst = 0;

	return dst;
}

static char *
print_syscall(char *b, const char *name, unsigned args, ...)
{
	bool first = true;
	va_list ap;

	b += sprintf(b, "%s(", name);

	va_start(ap, args);

	while (args > 0) {
		int format = va_arg(ap, int);

		if (!first) {
			*b++ = ',';
			*b++ = ' ';
		}

		if (format == f_dec) {
			b += sprintf(b, "%ld", va_arg(ap, long));
		} else if (format == f_oct_mode) {
			b += sprintf(b, "0%lo", va_arg(ap, unsigned long));
		} else if (format == f_hex) {
			b += sprintf(b, "0x%lx", va_arg(ap, unsigned long));
		} else if (format == f_str) {
			b = xprint_escape(b, va_arg(ap, char *), 0x80, true, 0);
		} else if (format == f_buf) {
			size_t size = va_arg(ap, size_t);
			const char *data = va_arg(ap, char *);
			b = xprint_escape(b, data, 0x80, false, size);
		} else if (format == f_open_flags) {
			b = print_open_flags(b, va_arg(ap, long));
		}

		--args;
		first = false;
	}

	b += sprintf(b, ") = %ld", va_arg(ap, long));

	va_end(ap);

	return b;
}
