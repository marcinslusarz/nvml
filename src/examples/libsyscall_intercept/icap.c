
#include <stddef.h>
#include <string.h>
#include <syscall.h>

#include "libsyscall_intercept_hook_point.h"

static int hook(long syscall_number,
                        long arg0, long arg1,
                        long arg2, long arg3,
                        long arg4, long arg5,
                        long *result)
{
	if (syscall_number == SYS_write) {
		char buf_copy[0x1000];
		size_t size = (size_t)arg2;

		if (size > sizeof(buf_copy))
			size = sizeof(buf_copy);

		memcpy(buf_copy, (char *)arg1, size);

		// Capitalize the letter 'i', for fun
		for (size_t i = 0; i < size; ++i) {
			if (buf_copy[i] == 'i')
				buf_copy[i] = 'I';
		}
		*result = syscall_no_intercept(SYS_write, arg0, buf_copy, size);
		return 0;
	}
	return 1;
	
}

static __attribute__((constructor)) void
start(void)
{
	intercept_hook_point = &hook;
}
