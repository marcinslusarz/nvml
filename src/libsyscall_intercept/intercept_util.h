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

#ifndef INTERCEPT_UTIL_H
#define INTERCEPT_UTIL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

/*
 * syscall_no_intercept - syscall without interception
 *
 * Call syscall_no_intercept to make syscalls
 * from the interceptor library, once glibc is already patched.
 * Don't use the syscall function from glibc, that
 * would just result in an infinite recursion.
 */
long syscall_no_intercept(long syscall_number, ...);

/*
 * xmmap_anon - get new memory mapping
 *
 * Not intercepted - does not call libc.
 * Always succeds if returns - aborts the process on failure.
 */
void *xmmap_anon(size_t size);

void *xmremap(void *addr, size_t old, size_t new);

/*
 * xlseek - get new memory mapping
 *
 * Not intercepted - does not call libc.
 * Always succeds if returns - aborts the process on failure.
 */
long xlseek(long fd, unsigned long off, int whence);

/*
 * xread - get new memory mapping
 *
 * Not intercepted - does not call libc.
 * Always succeds reading size bytes returns - aborts the process on failure.
 */
void xread(long fd, void *buffer, size_t size);

char *xprint_escape(char *restrict dst, const char *restrict src,
			size_t dst_size, bool zero_term, size_t src_size);

void intercept_setup_log(const char *path_base);
void intercept_log(const char *buffer, size_t len);
void intercept_logs(const char *str);
void intercept_log_syscall(const char *libpath, long nr, long arg0, long arg1,
			long arg2, long arg3,
			long arg4, long arg5, uint64_t syscall_offset,
			int result_known, long result);
void intercept_log_close(void);

#endif
