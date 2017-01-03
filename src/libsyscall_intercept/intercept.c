/*
 * Copyright 2016-2017, Intel Corporation
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
#include <assert.h>
#include <stdbool.h>
#include <elf.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syscall.h>
#include <sys/mman.h>

#include "intercept.h"
#include "intercept_util.h"
#include "libsyscall_intercept_hook_point.h"
#include "disasm_wrapper.h"

int (*intercept_hook_point)(long syscall_number,
			long arg0, long arg1,
			long arg2, long arg3,
			long arg4, long arg5,
			long *result);

static Dl_info self_dlinfo;
static Dl_info libc_dlinfo;
static Dl_info pthreads_dlinfo;

static void find_self_dlinfo(void);

static int find_glibc_dl(void);
static int find_pthreads_dl(void);

static struct intercept_desc glibc_patches;
static struct intercept_desc pthreads_patches;

static void log_header(void);

void __attribute__((noreturn)) xlongjmp(long rip, long rsp, long rax);

static void
intercept_routine(long nr, long arg0, long arg1,
			long arg2, long arg3,
			long arg4, long arg5,
			uint32_t syscall_offset,
			const char *libpath,
			long return_to_asm_wrapper_syscall,
			long return_to_asm_wrapper,
			long rsp_in_asm_wrapper);

static __attribute__((constructor)) void
intercept(void)
{
	if (!libc_hook_in_process_allowed())
		return;

	bool pthreads_available;

	find_self_dlinfo();
	glibc_patches.c_detination = (void *)((uintptr_t)&intercept_routine);
	pthreads_patches.c_detination = (void *)((uintptr_t)&intercept_routine);
	intercept_setup_log(getenv("INTERCEPT_LOG"),
			getenv("INTERCEPT_LOG_TRUNC"));

	if (find_glibc_dl() != 0) {
		intercept_logs("libc not found");
		intercept_log_close();
		return;
	}

	init_patcher();

	log_header();

	find_syscalls(&glibc_patches, &libc_dlinfo);
	create_patch_wrappers(&glibc_patches);

	pthreads_available = (find_pthreads_dl() == 0);

	if (pthreads_available) {
		find_syscalls(&pthreads_patches, &pthreads_dlinfo);
		create_patch_wrappers(&pthreads_patches);
		activate_patches(&pthreads_patches);
	} else {
		intercept_logs("libpthreads not found");
	}

	mprotect_asm_wrappers();
	activate_patches(&glibc_patches);
	if (pthreads_available)
		activate_patches(&pthreads_patches);
}

static void
log_header(void)
{
	static const char self_decoder[] =
		"tempfile=$(mktemp) ; tempfile2=$(mktemp) ; "
		"grep \"^/\" $0 | cut -d \" \" -f 1,2 | "
		"sed \"s/^/addr2line -p -f -e /\" > $tempfile ; "
		"{ echo ; . $tempfile ; echo ; } > $tempfile2 ; "
		"paste $tempfile2 $0 ; exit 0\n";

	intercept_log(self_decoder, sizeof(self_decoder) - 1);
}

static void
find_self_dlinfo(void)
{
	if (!dladdr((void *)((uintptr_t)&intercept), &self_dlinfo))
		syscall_no_intercept(SYS_exit_group, INTERCEPTOR_EXIT_CODE + 1);

	if (self_dlinfo.dli_fbase == NULL)
		syscall_no_intercept(SYS_exit_group, INTERCEPTOR_EXIT_CODE + 2);
}

static int
find_glibc_dl(void)
{
	/* Assume the library that provides fopen is glibc */

	if (!dladdr((void *)((uintptr_t)&fopen), &libc_dlinfo))
		return -1;

	if (libc_dlinfo.dli_fbase == NULL)
		return -1;

	if (libc_dlinfo.dli_fname == NULL)
		return -1;

	return 0;
}

static int
find_pthreads_dl(void)
{
	/*
	 * Assume the library that provides pthread_create is libpthread.
	 * Use dlsym instead of &pthread_create, as that would abort the
	 * program if libpthread is not actually loaded.
	 */

	void *pcreate_addr = dlsym(RTLD_DEFAULT, "pthread_create");

	if (pcreate_addr == NULL)
		return -1;

	if (!dladdr(pcreate_addr, &pthreads_dlinfo))
		return -1;

	if (pthreads_dlinfo.dli_fbase == NULL)
		return -1;

	if (pthreads_dlinfo.dli_fname == NULL)
		return -1;

	return 0;
}

void
xabort(void)
{
	__builtin_trap();
}

/*
 * intercept_routine(...)
 * This is the function called from the asm wrappers,
 * forwarding the syscall parameters to pmemfile.
 *
 * Arguments:
 * nr, arg0 - arg 5 -- syscall number
 *
 * For logging ( debugging, validating ):
 *
 * syscall_offset -- the offset of the original syscall
 *  instruction in the shared object
 * libpath -- the path of the .so being intercepted,
 *  e.g.: "/usr/lib/libc.so.6"
 *
 * For returning to libc:
 * return_to_asm_wrapper_syscall, return_to_asm_wrapper -- the
 *  address to jump to, when this function is done. The function
 *  is called with a faked return address on the stack ( to aid
 *  stack unwinding ). So, instead of just returning from this
 *  function, one must jump to one of these addresses. The first
 *  one triggers the execution of the syscall after restoring all
 *  registers, and before actually jumping back to the subject library.
 *
 * rsp_in_asm_wrapper -- the stack pointer to restore after returning
 *  from this function.
 */
static void
intercept_routine(long nr, long arg0, long arg1,
			long arg2, long arg3,
			long arg4, long arg5,
			uint32_t syscall_offset,
			const char *libpath,
			long return_to_asm_wrapper_syscall,
			long return_to_asm_wrapper,
			long rsp_in_asm_wrapper)
{
	long result;
	int forward_to_kernel = true;

	intercept_log_syscall(libpath, nr,
	    arg0, arg1, arg2, arg3, arg4, arg5,
	    syscall_offset, 0, 0);

	if (intercept_hook_point != NULL)
		forward_to_kernel = intercept_hook_point(nr,
		    arg0, arg1, arg2, arg3, arg4, arg5, &result);

	if (nr == SYS_clone ||
	    nr == SYS_vfork ||
	    nr == SYS_rt_sigreturn) {
		/* can't handle these syscall the normal way */
		xlongjmp(return_to_asm_wrapper_syscall, rsp_in_asm_wrapper, nr);
	}

	if (forward_to_kernel)
		result = syscall_no_intercept(nr,
		    arg0, arg1, arg2, arg3, arg4, arg5);

	intercept_log_syscall(libpath, nr,
	    arg0, arg1, arg2, arg3, arg4, arg5,
	    syscall_offset, 1, result);

	xlongjmp(return_to_asm_wrapper, rsp_in_asm_wrapper, result);
}

int
libc_hook_in_process_allowed(void)
{
	char *c = getenv("LIBC_HOOK_CMDLINE_FILTER");
	if (c == NULL)
		return 1;

	long fd = syscall_no_intercept(SYS_open, "/proc/self/cmdline",
	    O_RDONLY, 0);
	if (fd < 0)
		return 0;

	char buf[0x1000];
	long r = syscall_no_intercept(SYS_read, fd, buf, sizeof(buf));

	syscall_no_intercept(SYS_close, fd);

	if (r <= 1 || buf[0] == '\0')
		return 0;

	char *name = buf;
	while (*name != '\0')
		++name;

	while (*name != '/' && name != buf)
		--name;

	if (*name == '/')
		++name;

	return strcmp(name, c) == 0;
}
