/*
 * Copyright 2014-2016, Intel Corporation
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
 * pmem.c -- pmem entry points for libpmem
 *
 *
 * PERSISTENT MEMORY INSTRUCTIONS ON X86
 *
 * The primary feature of this library is to provide a way to flush
 * changes to persistent memory as outlined below (note that many
 * of the decisions below are made at initialization time, and not
 * repeated every time a flush is requested).
 *
 * To flush a range to pmem when CLWB is available:
 *
 *	CLWB for each cache line in the given range.
 *
 *	SFENCE to ensure the CLWBs above have completed.
 *
 *	PCOMMIT to mark pmem stores in the memory subsystem.
 *
 *	SFENCE to ensure the stores marked by PCOMMIT above have completed.
 *
 * To flush a range to pmem when CLFLUSHOPT is available and CLWB is not
 * (same as above but issue CLFLUSHOPT instead of CLWB):
 *
 *	CLFLUSHOPT for each cache line in the given range.
 *
 *	SFENCE to ensure the CLWBs above have completed.
 *
 *	PCOMMIT to mark pmem stores in the memory subsystem.
 *
 *	SFENCE to ensure the stores marked by PCOMMIT above have completed.
 *
 * To flush a range to pmem when neither CLFLUSHOPT or CLWB are available
 * (same as above but fences surrounding CLFLUSH are not required):
 *
 *	CLFLUSH for each cache line in the given range.
 *
 *	PCOMMIT to mark pmem stores in the memory subsystem.
 *
 *	SFENCE to ensure the stores marked by PCOMMIT above have completed.
 *
 * To flush a range to pmem when the caller has explicitly assumed
 * responsibility for draining HW stores in the memory subsystem
 * (by choosing to depend on ADR, or by assuming responsibility to issue
 * PCOMMIT/SFENCE at some point):
 *
 *	Same as above flows but omit the final PCOMMIT and SFENCE.
 *
 * To memcpy a range of memory to pmem when MOVNT is available:
 *
 *	Copy any non-64-byte portion of the destination using MOV.
 *
 *	Use the non-PCOMMIT flush flow above for the copied portion.
 *
 *	Copy using MOVNTDQ, up to any non-64-byte aligned end portion.
 *	(The MOVNT instructions bypass the cache, so no flush is required.)
 *
 *	Copy any unaligned end portion using MOV.
 *
 *	Use the flush flow above for the copied portion (including PCOMMIT).
 *
 * To memcpy a range of memory to pmem when MOVNT is not available:
 *
 *	Just pass the call to the normal memcpy() followed by pmem_persist().
 *
 * To memset a non-trivial sized range of memory to pmem:
 *
 *	Same as the memcpy cases above but store the given value instead
 *	of reading values from the source.
 *
 *
 * INTERFACES FOR FLUSHING TO PERSISTENT MEMORY
 *
 * Given the flows above, three interfaces are provided for flushing a range
 * so that the caller has the ability to separate the steps when necessary,
 * but otherwise leaves the detection of available instructions to the libpmem:
 *
 * pmem_persist(addr, len)
 *
 *	This is the common case, which just calls the two other functions:
 *
 *		pmem_flush(addr, len);
 *		pmem_drain();
 *
 * pmem_flush(addr, len)
 *
 *	CLWB or CLFLUSHOPT or CLFLUSH for each cache line
 *
 * pmem_drain()
 *
 *	SFENCE unless using CLFLUSH
 *
 *	PCOMMIT
 *
 *	SFENCE
 *
 * When PCOMMIT is unavailable, either because the platform doesn't support it
 * or because it has been inhibited by the caller by setting PMEM_NO_PCOMMIT=1,
 * the pmem_drain() function degenerates into this:
 *
 * pmem_drain()
 *
 *	SFENCE unless using CLFLUSH
 *
 *
 * INTERFACES FOR COPYING/SETTING RANGES OF MEMORY
 *
 * Given the flows above, the following interfaces are provided for the
 * memmove/memcpy/memset operations to persistent memory:
 *
 * pmem_memmove_nodrain()
 *
 *	Checks for overlapped ranges to determine whether to copy from
 *	the beginning of the range or from the end.  If MOVNT instructions
 *	are available, uses the memory copy flow described above, otherwise
 *	calls the libc memmove() followed by pmem_flush().
 *
 * pmem_memcpy_nodrain()
 *
 *	Just calls pmem_memmove_nodrain().
 *
 * pmem_memset_nodrain()
 *
 *	If MOVNT instructions are available, uses the memset flow described
 *	above, otherwise calls the libc memset() followed by pmem_flush().
 *
 * pmem_memmove_persist()
 * pmem_memcpy_persist()
 * pmem_memset_persist()
 *
 *	Calls the appropriate _nodrain() function followed by pmem_drain().
 *
 *
 * DECISIONS MADE AT INITIALIZATION TIME
 *
 * As much as possible, all decisions described above are made at library
 * initialization time.  This is achieved using two different approaches
 * depending on compiler features:
 * - using function pointers which are setup by pmem_init() when the
 *   library loads,
 * - marking functions as indirect functions using the 'ifunc' function
 *   attribute, this allows the resolution of the symbol value to be
 *   determined dynamically at load time.
 *
 * In both approaches the pmem_init_detect() function detects availability of
 * specified commands and assigns the following function pointers:
 *
 *	Func_drain is used by pmem_drain() to call one of:
 *		drain_no_pcommit_predrain_fence_empty()
 *		drain_no_pcommit_predrain_fence_sfence()
 *		drain_pcommit_predrain_fence_empty()()
 *		drain_pcommit_predrain_fence_sfence()()
 *
 *	Func_persist is used by pmem_persist() to call one of:
 *		persist_clflush_no_pcommit()
 *		persist_clflush_pcommit()
 *		persist_clflushopt_no_pcommit()
 *		persist_clflushopt_pcommit()
 *		persist_clwb_no_pcommit()
 *		persist_clwb_pcommit()
 *
 *	Func_flush is used by pmem_flush() to call one of:
 *		flush_clwb()
 *		flush_clflushopt()
 *		flush_clflush()
 *
 *	Func_memmove_nodrain is used by memmove_nodrain() to call one of:
 *		memmove_nodrain_normal()
 *		memmove_nodrain_movnt()
 *
 *	Func_memset_nodrain is used by memset_nodrain() to call one of:
 *		memset_nodrain_normal()
 *		memset_nodrain_movnt()
 *
 *
 * The function pointers in 1st approach are explicitly called by the
 * corresponding API functions. In the 2nd approach the function pointers are
 * returned by corresponding pmem_*_ifunc() functions which provide the
 * value for symbol resolution at load time.
 *
 * The 'ifunc' approach is used after defining HAS_IFUNC value to 1.
 *
 * DEBUG LOGGING
 *
 * Many of the functions here get called hundreds of times from loops
 * iterating over ranges, making the usual LOG() calls at level 3
 * impractical.  The call tracing log for those functions is set at 15.
 */

#include <sys/mman.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <xmmintrin.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "libpmem.h"

#include "pmem.h"
#include "util.h"
#include "cpu.h"
#include "out.h"
#include "valgrind_internal.h"

/*
 * The x86 memory instructions are new enough that the compiler
 * intrinsic functions are not always available.  The intrinsic
 * functions are defined here in terms of asm statements for now.
 */
#define	_mm_clflushopt(addr)\
	asm volatile(".byte 0x66; clflush %0" : "+m" (*(volatile char *)addr));
#define	_mm_clwb(addr)\
	asm volatile(".byte 0x66; xsaveopt %0" : "+m" (*(volatile char *)addr));
#define	_mm_pcommit()\
	asm volatile(".byte 0x66, 0x0f, 0xae, 0xf8");

#define	FLUSH_ALIGN ((uintptr_t)64)

#define	ALIGN_MASK	(FLUSH_ALIGN - 1)

#define	CHUNK_SIZE	128 /* 16*8 */
#define	CHUNK_SHIFT	7
#define	CHUNK_MASK	(CHUNK_SIZE - 1)

#define	DWORD_SIZE	4
#define	DWORD_SHIFT	2
#define	DWORD_MASK	(DWORD_SIZE - 1)

#define	MOVNT_SIZE	16
#define	MOVNT_MASK	(MOVNT_SIZE - 1)
#define	MOVNT_SHIFT	4

#define	MOVNT_THRESHOLD	256

#define	PROCMAXLEN 2048 /* maximum expected line length in /proc files */

static size_t Movnt_threshold = MOVNT_THRESHOLD;
static int Has_hw_drain;

/*
 * set of flags which determines which functions will be used by library
 */
#define	FF_NO_PCOMMIT		0x00U	/* no pcommit */
#define	FF_PCOMMIT		0x01U	/* pcommit */
#define	FF_MASK_PCOMMIT		0x01U	/* mask for pcommit */
#define	FF_PDF_EMPTY		0x00U	/* pre-drain fence empty */
#define	FF_PDF_SFENCE		0x02U	/* pre-drain fence using sfence */
#define	FF_MASK_PDF		0x02U	/* mask for pre-drain fence */
#define	FF_CLFLUSH		0x04U	/* clflush */
#define	FF_CLFLUSHOPT		0x08U	/* clflushopt */
#define	FF_CLWB			0x0CU	/* clwb */
#define	FF_MASK_FLUSH		0x0CU	/* mask for flush */
#define	FF_NO_MOVNT		0x00U	/* no movnt */
#define	FF_MOVNT		0x10U	/* movnt */
#define	FF_MASK_MOVNT		0x10U	/* mask for movnt */

#define	FF_MASK_DRAIN		(FF_MASK_PCOMMIT | FF_MASK_PDF)
#define	FF_MASK_PERSIST		(FF_MASK_PCOMMIT | FF_MASK_FLUSH)
#define	FF_GET(f, m)		((f) & FF_MASK_##m)
#define	FF_GET_PCOMMIT(f)	FF_GET(f, PCOMMIT)
#define	FF_GET_PDF(f)		FF_GET(f, PDF)
#define	FF_GET_DRAIN(f)		FF_GET(f, DRAIN)
#define	FF_GET_FLUSH(f)		FF_GET(f, FLUSH)
#define	FF_GET_PERSIST(f)	FF_GET(f, PERSIST)
#define	FF_GET_MOVNT(f)		FF_GET(f, MOVNT)
#define	FF_SET(f, m, v)\
	((f) = (((f) & ~(FF_MASK_##m)) | (FF_##v)))
/*
 * default value for function flags:
 * - no pcommit support
 * - pre-drain fence empty
 * - clflush
 * - no movnt support
 */
#define	FF_DEFAULT \
(FF_NO_PCOMMIT | FF_PDF_EMPTY | FF_CLFLUSH | FF_NO_MOVNT)

/*
 * function flags which hold information about supported instructions
 * and are used to determine the function pointers to use
 */
static unsigned Func_flags = FF_DEFAULT;

static void pmem_init_detect(void);

/*
 * pmem_has_hw_drain -- return whether or not HW drain (PCOMMIT) was found
 */
int
pmem_has_hw_drain(void)
{
	return Has_hw_drain;
}

/*
 * predrain_fence_empty -- (internal) issue the pre-drain fence instruction
 */
static inline void
predrain_fence_empty(void)
{
	LOG(15, NULL);

	VALGRIND_DO_FENCE;
	/* nothing to do (because CLFLUSH did it for us) */
}

/*
 * predrain_fence_sfence -- (internal) issue the pre-drain fence instruction
 */
static inline void
predrain_fence_sfence(void)
{
	LOG(15, NULL);

	_mm_sfence();	/* ensure CLWB or CLFLUSHOPT completes before PCOMMIT */
}

/*
 * drain_no_pcommit -- (internal) wait for PM stores to drain, empty version
 */
static inline void
drain_no_pcommit(void)
{
	LOG(15, NULL);

	VALGRIND_DO_COMMIT;
	VALGRIND_DO_FENCE;
	/* caller assumed responsibility for the rest */
}

/*
 * drain_pcommit -- (internal) wait for PM stores to drain, pcommit version
 */
static inline void
drain_pcommit(void)
{
	LOG(15, NULL);

	_mm_pcommit();
	_mm_sfence();
}

/*
 * drain_no_pcommit_predrain_fence_empty -- (internal) wait for PM stores to
 * drain, empty version with empty pre-drain fence
 */
static void
drain_no_pcommit_predrain_fence_empty(void)
{
	LOG(15, NULL);

	predrain_fence_empty();

	drain_no_pcommit();
}

/*
 * drain_pcommit_predrain_fence_empty -- (internal) wait for PM stores to
 * drain, pcommit version with empty pre-drain fence
 */
static void
drain_pcommit_predrain_fence_empty(void)
{
	LOG(15, NULL);

	predrain_fence_empty();

	drain_pcommit();
}

/*
 * drain_no_pcommit_predrain_fence_sfence -- (internal) wait for PM stores to
 * drain, empty version with pre-drain sfence
 */
static void
drain_no_pcommit_predrain_fence_sfence(void)
{
	LOG(15, NULL);

	predrain_fence_sfence();

	drain_no_pcommit();
}

/*
 * drain_pcommit_predrain_fence_sfence -- (internal) wait for PM stores to
 * drain, pcommit version with pre-drain sfence
 */
static void
drain_pcommit_predrain_fence_sfence(void)
{
	LOG(15, NULL);

	predrain_fence_sfence();

	drain_pcommit();
}

/*
 * pmem_drain() calls through Func_drain to do the work.  Although
 * initialized to drain_no_pcommit_predrain_fence_empty(), once the
 * existence of the pcommit feature is confirmed by pmem_init() at
 * library initialization time, Func_drain is set to one of drain_pcommit_*()
 * functions depending on existence of flushing instructions supported by hw.
 * That's the most common case on modern hardware that supports persistent
 * memory.
 */
static void (*Func_drain)(void);

#if HAS_IFUNC
/*
 * pmem_drain_ifunc -- (internal) return pointer to pmem_drain function
 */
static void *
pmem_drain_ifunc(void)
{
	if (!Func_drain)
		pmem_init_detect();

	ASSERT(Func_drain != NULL);

	return Func_drain;
}

void pmem_drain(void) __attribute__((ifunc("pmem_drain_ifunc")));
#else
/*
 * pmem_drain -- wait for any PM stores to drain from HW buffers
 */
void
pmem_drain(void)
{
	LOG(10, NULL);

	Func_drain();
}
#endif

/*
 * flush_clflush -- (internal) flush the CPU cache, using clflush
 */
static inline void
flush_clflush(const void *addr, size_t len)
{
	LOG(10, "addr %p len %zu", addr, len);

	VALGRIND_DO_CHECK_MEM_IS_ADDRESSABLE(addr, len);

	uintptr_t uptr;

	/*
	 * Loop through cache-line-size (typically 64B) aligned chunks
	 * covering the given range.
	 */
	for (uptr = (uintptr_t)addr & ~(FLUSH_ALIGN - 1);
		uptr < (uintptr_t)addr + len; uptr += FLUSH_ALIGN)
		_mm_clflush((char *)uptr);
}

/*
 * flush_clwb -- (internal) flush the CPU cache, using clwb
 */
static inline void
flush_clwb(const void *addr, size_t len)
{
	LOG(10, "addr %p len %zu", addr, len);

	VALGRIND_DO_CHECK_MEM_IS_ADDRESSABLE(addr, len);

	uintptr_t uptr;

	/*
	 * Loop through cache-line-size (typically 64B) aligned chunks
	 * covering the given range.
	 */
	for (uptr = (uintptr_t)addr & ~(FLUSH_ALIGN - 1);
		uptr < (uintptr_t)addr + len; uptr += FLUSH_ALIGN) {
		_mm_clwb((char *)uptr);
	}
}

/*
 * flush_clflushopt -- (internal) flush the CPU cache, using clflushopt
 */
static inline void
flush_clflushopt(const void *addr, size_t len)
{
	LOG(10, "addr %p len %zu", addr, len);

	VALGRIND_DO_CHECK_MEM_IS_ADDRESSABLE(addr, len);

	uintptr_t uptr;

	/*
	 * Loop through cache-line-size (typically 64B) aligned chunks
	 * covering the given range.
	 */
	for (uptr = (uintptr_t)addr & ~(FLUSH_ALIGN - 1);
		uptr < (uintptr_t)addr + len; uptr += FLUSH_ALIGN) {
		_mm_clflushopt((char *)uptr);
	}
}

/*
 * pmem_flush() calls through Func_flush to do the work.  Although
 * initialized to flush_clflush(), once the existence of the clflushopt
 * feature is confirmed by pmem_init() at library initialization time,
 * Func_flush is set to flush_clflushopt().  That's the most common case
 * on modern hardware that supports persistent memory.
 */
static void (*Func_flush)(const void *, size_t);

#if HAS_IFUNC
/*
 * pmem_flush_ifunc -- (internal) return pointer to pmem_flush function
 */
static void *
pmem_flush_ifunc(void)
{
	if (!Func_flush)
		pmem_init_detect();

	ASSERT(Func_flush != NULL);

	return Func_flush;
}

void
pmem_flush(const void *addr, size_t len)
__attribute__((ifunc("pmem_flush_ifunc")));
#else
/*
 * pmem_flush -- flush processor cache for the given range
 */
void
pmem_flush(const void *addr, size_t len)
{
	LOG(10, "addr %p len %zu", addr, len);

	Func_flush(addr, len);
}
#endif

/*
 * persist_clflush_no_pcommit -- (internal) persist function version
 * with clflush and no pcommit
 */
static void
persist_clflush_no_pcommit(void *addr, size_t len)
{
	LOG(15, "addr %p len %zu", addr, len);

	flush_clflush(addr, len);
	drain_no_pcommit_predrain_fence_empty();
}

/*
 * persist_clflush_pcommit -- (internal) persist function version
 * with clflush and pcommit
 */
static void
persist_clflush_pcommit(void *addr, size_t len)
{
	LOG(15, "addr %p len %zu", addr, len);

	flush_clflush(addr, len);
	drain_pcommit_predrain_fence_empty();
}

/*
 * persist_clflushopt_no_pcommit -- (internal) persist function version
 * with clflushopt and no pcommit
 */
static void
persist_clflushopt_no_pcommit(void *addr, size_t len)
{
	LOG(15, "addr %p len %zu", addr, len);

	flush_clflushopt(addr, len);
	drain_no_pcommit_predrain_fence_sfence();
}

/*
 * persist_clflushopt_pcommit -- (internal) persist function version
 * with clflushopt and pcommit
 */
static void
persist_clflushopt_pcommit(void *addr, size_t len)
{
	LOG(15, "addr %p len %zu", addr, len);

	flush_clflushopt(addr, len);
	drain_pcommit_predrain_fence_sfence();
}

/*
 * persist_clwb_no_pcommit -- (internal) persist function version
 * with clwb and no pcommit
 */
static void
persist_clwb_no_pcommit(void *addr, size_t len)
{
	LOG(15, "addr %p len %zu", addr, len);

	flush_clwb(addr, len);
	drain_no_pcommit_predrain_fence_sfence();
}

/*
 * persist_clwb_pcommit -- (internal) persist function version
 * with clwb and pcommit
 */
static void
persist_clwb_pcommit(void *addr, size_t len)
{
	LOG(15, "addr %p len %zu", addr, len);

	flush_clwb(addr, len);
	drain_pcommit_predrain_fence_sfence();
}

/*
 * pmem_flush() calls through Func_flush to do the work.  Although
 * initialized to flush_clflush(), once the existence of the clflushopt
 * feature is confirmed by pmem_init() at library initialization time,
 * Func_flush is set to flush_clflushopt().  That's the most common case
 * on modern hardware that supports persistent memory.
 */
static void (*Func_persist)(void *, size_t);

#if HAS_IFUNC
/*
 * pmem_persist_ifunc -- (internal) return pointer to pmem_persist function
 */
static void *
pmem_persist_ifunc(void)
{
	if (!Func_persist)
		pmem_init_detect();

	ASSERT(Func_persist != NULL);

	return Func_persist;
}

void
pmem_persist(const void *addr, size_t len)
__attribute__((ifunc("pmem_persist_ifunc")));
#else
/*
 * pmem_persist -- make any cached changes to a range of pmem persistent
 */
void
pmem_persist(const void *addr, size_t len)
{
	LOG(15, "addr %p len %zu", addr, len);

	Func_persist(addr, len);
}
#endif

/*
 * pmem_msync -- flush to persistence via msync
 *
 * Using msync() means this routine is less optimal for pmem (but it
 * still works) but it also works for any memory mapped file, unlike
 * pmem_persist() which is only safe where pmem_is_pmem() returns true.
 */
int
pmem_msync(const void *addr, size_t len)
{
	LOG(15, "addr %p len %zu", addr, len);

	VALGRIND_DO_CHECK_MEM_IS_ADDRESSABLE(addr, len);

	/*
	 * msync requires len to be a multiple of pagesize, so
	 * adjust addr and len to represent the full 4k chunks
	 * covering the given range.
	 */

	/* increase len by the amount we gain when we round addr down */
	len += (uintptr_t)addr & (Pagesize - 1);

	/* round addr down to page boundary */
	uintptr_t uptr = (uintptr_t)addr & ~(Pagesize - 1);

	/*
	 * msync accepts addresses aligned to page boundary, so we may sync
	 * more and part of it may have been marked as undefined/inaccessible
	 * Msyncing such memory is not a bug, so as a workaround temporarily
	 * disable error reporting.
	 */
	VALGRIND_DO_DISABLE_ERROR_REPORTING;

	int ret;
	if ((ret = msync((void *)uptr, len, MS_SYNC)) < 0)
		ERR("!msync");

	VALGRIND_DO_ENABLE_ERROR_REPORTING;

	/* full flush, commit */
	VALGRIND_DO_PERSIST(uptr, len);

	return ret;
}

/*
 * is_pmem_always -- (internal) always true version of pmem_is_pmem()
 */
static int
is_pmem_always(const void *addr, size_t len)
{
	LOG(3, NULL);

	return 1;
}

/*
 * is_pmem_never -- (internal) never true version of pmem_is_pmem()
 */
static int
is_pmem_never(const void *addr, size_t len)
{
	LOG(3, NULL);

	return 0;
}

/*
 * is_pmem_proc -- (internal) use /proc to implement pmem_is_pmem()
 *
 * This function returns true only if the entire range can be confirmed
 * as being direct access persistent memory.  Finding any part of the
 * range is not direct access, or failing to look up the information
 * because it is unmapped or because any sort of error happens, just
 * results in returning false.
 *
 * This function works by lookup up the range in /proc/self/smaps and
 * verifying the "mixed map" vmflag is set for that range.  While this
 * isn't exactly the same as direct access, there is no DAX flag in
 * the vmflags and the mixed map flag is only true on regular files when
 * DAX is in-use, so it serves the purpose.
 *
 * The range passed in may overlap with multiple entries in the smaps list
 * so this function loops through the smaps entries until the entire range
 * is verified as direct access, or until it is clear the answer is false
 * in which case it stops the loop and returns immediately.
 */
static int
is_pmem_proc(const void *addr, size_t len)
{
	const char *caddr = addr;

	FILE *fp;
	if ((fp = fopen("/proc/self/smaps", "r")) == NULL) {
		ERR("!/proc/self/smaps");
		return 0;
	}

	int retval = 0;		/* assume false until proven otherwise */
	char line[PROCMAXLEN];	/* for fgets() */
	char *lo = NULL;	/* beginning of current range in smaps file */
	char *hi = NULL;	/* end of current range in smaps file */
	int needmm = 0;		/* looking for mm flag for current range */
	while (fgets(line, PROCMAXLEN, fp) != NULL) {
		static const char vmflags[] = "VmFlags:";
		static const char mm[] = " mm";

		/* check for range line */
		if (sscanf(line, "%p-%p", &lo, &hi) == 2) {
			if (needmm) {
				/* last range matched, but no mm flag found */
				LOG(4, "never found mm flag");
				break;
			} else if (caddr < lo) {
				/* never found the range for caddr */
				LOG(4, "no match for addr %p", caddr);
				break;
			} else if (caddr < hi) {
				/* start address is in this range */
				size_t rangelen = (size_t)(hi - caddr);

				/* remember that matching has started */
				needmm = 1;

				/* calculate remaining range to search for */
				if (len > rangelen) {
					len -= rangelen;
					caddr += rangelen;
					LOG(4, "matched %zu bytes in range "
							"%p-%p, %zu left over",
							rangelen, lo, hi, len);
				} else {
					len = 0;
					LOG(4, "matched all bytes in range "
							"%p-%p", lo, hi);
				}
			}
		} else if (needmm && strncmp(line, vmflags,
					sizeof (vmflags) - 1) == 0) {
			if (strstr(&line[sizeof (vmflags) - 1], mm) != NULL) {
				LOG(4, "mm flag found");
				if (len == 0) {
					/* entire range matched */
					retval = 1;
					break;
				}
				needmm = 0;	/* saw what was needed */
			} else {
				/* mm flag not set for some or all of range */
				LOG(4, "range has no mm flag");
				break;
			}
		}
	}

	fclose(fp);

	LOG(3, "returning %d", retval);
	return retval;
}

/*
 * pmem_is_pmem() calls through Func_is_pmem to do the work.  Although
 * initialized to is_pmem_never(), once the existence of the clflush
 * feature is confirmed by pmem_init() at library initialization time,
 * Func_is_pmem is set to is_pmem_proc().  That's the most common case
 * on modern hardware.
 */
static int (*Func_is_pmem)(const void *addr, size_t len) = is_pmem_never;

/*
 * pmem_is_pmem -- return true if entire range is persistent Memory
 */
int
pmem_is_pmem(const void *addr, size_t len)
{
	LOG(10, "addr %p len %zu", addr, len);

	return Func_is_pmem(addr, len);
}

#define	PMEM_FILE_ALL_FLAGS\
	(PMEM_FILE_CREATE|PMEM_FILE_EXCL|PMEM_FILE_SPARSE|PMEM_FILE_TMPFILE)

#ifndef USE_O_TMPFILE
#ifdef O_TMPFILE
#define	USE_O_TMPFILE 1
#else
#define	USE_O_TMPFILE 0
#endif
#endif

/*
 * pmem_map_file -- create or open the file and map it to memory
 */
void *
pmem_map_file(const char *path, size_t len, int flags, mode_t mode,
	size_t *mapped_lenp, int *is_pmemp)
{
	LOG(3, "path \"%s\" size %zu flags %x mode %o mapped_lenp %p "
		"is_pmemp %p", path, len, flags, mode, mapped_lenp, is_pmemp);

	int oerrno;
	int fd;
	int open_flags = O_RDWR;
	int delete_on_err = 0;

	if (flags & ~(PMEM_FILE_ALL_FLAGS)) {
		ERR("invalid flag specified %x", flags);
		errno = EINVAL;
		return NULL;
	}

	if (flags & PMEM_FILE_CREATE) {
		if ((off_t)len < 0) {
			ERR("invalid file length %zu", len);
			errno = EINVAL;
			return NULL;
		}
		open_flags |= O_CREAT;
	}

	if (flags & PMEM_FILE_EXCL)
		open_flags |= O_EXCL;

	if ((len != 0) && !(flags & PMEM_FILE_CREATE)) {
		ERR("non-zero 'len' not allowed without PMEM_FILE_CREATE");
		errno = EINVAL;
		return NULL;
	}

	if ((len == 0) && (flags & PMEM_FILE_CREATE)) {
		ERR("zero 'len' not allowed with PMEM_FILE_CREATE");
		errno = EINVAL;
		return NULL;
	}

	if ((flags & PMEM_FILE_TMPFILE) && !(flags & PMEM_FILE_CREATE)) {
		ERR("PMEM_FILE_TMPFILE not allowed without PMEM_FILE_CREATE");
		errno = EINVAL;
		return NULL;
	}

#if USE_O_TMPFILE

	if (flags & PMEM_FILE_TMPFILE)
		open_flags |= O_TMPFILE;

	if ((fd = open(path, open_flags, mode)) < 0) {
		ERR("!open %s", path);
		return NULL;
	}

#else

	if (flags & PMEM_FILE_TMPFILE) {
		if ((fd = util_tmpfile(path, "/pmem.XXXXXX")) < 0) {
			return NULL;
		}
	} else {
		if ((fd = open(path, open_flags, mode)) < 0) {
			ERR("!open %s", path);
			return NULL;
		}
		if ((flags & PMEM_FILE_CREATE) && (flags & PMEM_FILE_EXCL))
			delete_on_err = 1;
	}

#endif

	if (flags & PMEM_FILE_CREATE) {
		if (flags & PMEM_FILE_SPARSE) {
			if (ftruncate(fd, (off_t)len) != 0) {
				ERR("!ftruncate");
				goto err;
			}
		} else {
			if ((errno = posix_fallocate(fd, 0, (off_t)len)) != 0) {
				ERR("!posix_fallocate");
				goto err;
			}
		}
	} else {
		struct stat stbuf;

		if (fstat(fd, &stbuf) < 0) {
			ERR("!fstat %s", path);
			goto err;
		}
		if (stbuf.st_size < 0) {
			ERR("stat %s: negative size", path);
			errno = EINVAL;
			goto err;
		}

		len = (size_t)stbuf.st_size;
	}

	void *addr;
	if ((addr = util_map(fd, len, 0, 0)) == NULL)
		goto err;    /* util_map() set errno, called LOG */

	if (mapped_lenp != NULL)
		*mapped_lenp = len;

	if (is_pmemp != NULL)
		*is_pmemp = pmem_is_pmem(addr, len);

	LOG(3, "returning %p", addr);

	VALGRIND_REGISTER_PMEM_MAPPING(addr, len);
	VALGRIND_REGISTER_PMEM_FILE(fd, addr, len, 0);

	(void) close(fd);

	return addr;

err:
	oerrno = errno;
	(void) close(fd);
	if (delete_on_err)
		(void) unlink(path);
	errno = oerrno;
	return NULL;
}

/*
 * pmem_unmap -- unmap the specified region
 */
int
pmem_unmap(void *addr, size_t len)
{
	LOG(3, "addr %p len %zu", addr, len);

	int ret = util_unmap(addr, len);

	VALGRIND_REMOVE_PMEM_MAPPING(addr, len);

	return ret;
}

/*
 * memmove_nodrain_normal -- (internal) memmove to pmem without hw drain
 */
static void *
memmove_nodrain_normal(void *pmemdest, const void *src, size_t len)
{
	LOG(15, "pmemdest %p src %p len %zu", pmemdest, src, len);

	memmove(pmemdest, src, len);
	pmem_flush(pmemdest, len);
	return pmemdest;
}

/*
 * memmove_nodrain_movnt -- (internal) memmove to pmem without hw drain, movnt
 */
static void *
memmove_nodrain_movnt(void *pmemdest, const void *src, size_t len)
{
	LOG(15, "pmemdest %p src %p len %zu", pmemdest, src, len);

	__m128i xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7;
	size_t i;
	__m128i *d;
	__m128i *s;
	void *dest1 = pmemdest;
	size_t cnt;

	if (len == 0 || src == pmemdest)
		return pmemdest;

	if (len < Movnt_threshold) {
		memmove(pmemdest, src, len);
		pmem_flush(pmemdest, len);
		return pmemdest;
	}

	if ((uintptr_t)dest1 - (uintptr_t)src >= len) {
		/*
		 * Copy the range in the forward direction.
		 *
		 * This is the most common, most optimized case, used unless
		 * the overlap specifically prevents it.
		 */

		/* copy up to FLUSH_ALIGN boundary */
		cnt = (uint64_t)dest1 & ALIGN_MASK;
		if (cnt > 0) {
			cnt = FLUSH_ALIGN - cnt;

			/* never try to copy more the len bytes */
			if (cnt > len)
				cnt = len;

			uint8_t *d8 = (uint8_t *)dest1;
			const uint8_t *s8 = (uint8_t *)src;
			for (i = 0; i < cnt; i++) {
				*d8 = *s8;
				d8++;
				s8++;
			}
			pmem_flush(dest1, cnt);
			dest1 = (char *)dest1 + cnt;
			src = (char *)src + cnt;
			len -= cnt;
		}

		d = (__m128i *)dest1;
		s = (__m128i *)src;

		cnt = len >> CHUNK_SHIFT;
		for (i = 0; i < cnt; i++) {
			xmm0 = _mm_loadu_si128(s);
			xmm1 = _mm_loadu_si128(s + 1);
			xmm2 = _mm_loadu_si128(s + 2);
			xmm3 = _mm_loadu_si128(s + 3);
			xmm4 = _mm_loadu_si128(s + 4);
			xmm5 = _mm_loadu_si128(s + 5);
			xmm6 = _mm_loadu_si128(s + 6);
			xmm7 = _mm_loadu_si128(s + 7);
			s += 8;
			_mm_stream_si128(d,	xmm0);
			_mm_stream_si128(d + 1,	xmm1);
			_mm_stream_si128(d + 2,	xmm2);
			_mm_stream_si128(d + 3,	xmm3);
			_mm_stream_si128(d + 4,	xmm4);
			_mm_stream_si128(d + 5, xmm5);
			_mm_stream_si128(d + 6,	xmm6);
			_mm_stream_si128(d + 7,	xmm7);
			VALGRIND_DO_FLUSH(d, 8 * sizeof (*d));
			d += 8;
		}

		/* copy the tail (<128 bytes) in 16 bytes chunks */
		len &= CHUNK_MASK;
		if (len != 0) {
			cnt = len >> MOVNT_SHIFT;
			for (i = 0; i < cnt; i++) {
				xmm0 = _mm_loadu_si128(s);
				_mm_stream_si128(d, xmm0);
				VALGRIND_DO_FLUSH(d, sizeof (*d));
				s++;
				d++;
			}
		}

		/* copy the last bytes (<16), first dwords then bytes */
		len &= MOVNT_MASK;
		if (len != 0) {
			cnt = len >> DWORD_SHIFT;
			int32_t *d32 = (int32_t *)d;
			int32_t *s32 = (int32_t *)s;
			for (i = 0; i < cnt; i++) {
				_mm_stream_si32(d32, *s32);
				VALGRIND_DO_FLUSH(d32, sizeof (*d32));
				d32++;
				s32++;
			}
			cnt = len & DWORD_MASK;
			uint8_t *d8 = (uint8_t *)d32;
			const uint8_t *s8 = (uint8_t *)s32;

			for (i = 0; i < cnt; i++) {
				*d8 = *s8;
				d8++;
				s8++;
			}
			pmem_flush(d32, cnt);
		}
	} else {
		/*
		 * Copy the range in the backward direction.
		 *
		 * This prevents overwriting source data due to an
		 * overlapped destination range.
		 */

		dest1 = (char *)dest1 + len;
		src = (char *)src + len;

		cnt = (uint64_t)dest1 & ALIGN_MASK;
		if (cnt > 0) {
			/* never try to copy more the len bytes */
			if (cnt > len)
				cnt = len;

			uint8_t *d8 = (uint8_t *)dest1;
			const uint8_t *s8 = (uint8_t *)src;
			for (i = 0; i < cnt; i++) {
				d8--;
				s8--;
				*d8 = *s8;
			}
			pmem_flush(d8, cnt);
			dest1 = (char *)dest1 - cnt;
			src = (char *)src - cnt;
			len -= cnt;
		}

		d = (__m128i *)dest1;
		s = (__m128i *)src;

		cnt = len >> CHUNK_SHIFT;
		for (i = 0; i < cnt; i++) {
			xmm0 = _mm_loadu_si128(s - 1);
			xmm1 = _mm_loadu_si128(s - 2);
			xmm2 = _mm_loadu_si128(s - 3);
			xmm3 = _mm_loadu_si128(s - 4);
			xmm4 = _mm_loadu_si128(s - 5);
			xmm5 = _mm_loadu_si128(s - 6);
			xmm6 = _mm_loadu_si128(s - 7);
			xmm7 = _mm_loadu_si128(s - 8);
			s -= 8;
			_mm_stream_si128(d - 1, xmm0);
			_mm_stream_si128(d - 2, xmm1);
			_mm_stream_si128(d - 3, xmm2);
			_mm_stream_si128(d - 4, xmm3);
			_mm_stream_si128(d - 5, xmm4);
			_mm_stream_si128(d - 6, xmm5);
			_mm_stream_si128(d - 7, xmm6);
			_mm_stream_si128(d - 8, xmm7);
			d -= 8;
			VALGRIND_DO_FLUSH(d, 8 * sizeof (*d));
		}

		/* copy the tail (<128 bytes) in 16 bytes chunks */
		len &= CHUNK_MASK;
		if (len != 0) {
			cnt = len >> MOVNT_SHIFT;
			for (i = 0; i < cnt; i++) {
				d--;
				s--;
				xmm0 = _mm_loadu_si128(s);
				_mm_stream_si128(d, xmm0);
				VALGRIND_DO_FLUSH(d, sizeof (*d));
			}
		}

		/* copy the last bytes (<16), first dwords then bytes */
		len &= MOVNT_MASK;
		if (len != 0) {
			cnt = len >> DWORD_SHIFT;
			int32_t *d32 = (int32_t *)d;
			int32_t *s32 = (int32_t *)s;
			for (i = 0; i < cnt; i++) {
				d32--;
				s32--;
				_mm_stream_si32(d32, *s32);
				VALGRIND_DO_FLUSH(d32, sizeof (*d32));
			}

			cnt = len & DWORD_MASK;
			uint8_t *d8 = (uint8_t *)d32;
			const uint8_t *s8 = (uint8_t *)s32;

			for (i = 0; i < cnt; i++) {
				d8--;
				s8--;
				*d8 = *s8;
			}
			pmem_flush(d8, cnt);
		}
	}

	/* serialize non-temporal store instructions */
	predrain_fence_sfence();

	return pmemdest;
}

/*
 * pmem_memmove_nodrain() calls through Func_memmove_nodrain to do the work.
 * Although initialized to memmove_nodrain_normal(), once the existence of the
 * sse2 feature is confirmed by pmem_init() at library initialization time,
 * Func_memmove_nodrain is set to memmove_nodrain_movnt().  That's the most
 * common case on modern hardware that supports persistent memory.
 */
static void *(*Func_memmove_nodrain)
	(void *pmemdest, const void *src, size_t len) = memmove_nodrain_normal;

#if HAS_IFUNC
/*
 * pmem_memmove_nodrain_ifunc -- (internal) return pointer to
 * pmem_memmove_nodrain function
 */
static void *
pmem_memmove_nodrain_ifunc(void)
{
	if (!Func_memmove_nodrain)
		pmem_init_detect();

	ASSERT(Func_memmove_nodrain != NULL);

	return Func_memmove_nodrain;
}

void *
pmem_memmove_nodrain(void *pmemdest, const void *src, size_t len)
__attribute__((ifunc("pmem_memmove_nodrain_ifunc")));
#else
/*
 * pmem_memmove_nodrain -- memmove to pmem without hw drain
 */
void *
pmem_memmove_nodrain(void *pmemdest, const void *src, size_t len)
{
	return Func_memmove_nodrain(pmemdest, src, len);
}
#endif

#if HAS_IFUNC
/*
 * pmem_memcpy_nodrain_ifunc -- (internal) return pointer to
 * pmem_memcpy_nodrain function
 */
static void *
pmem_memcpy_nodrain_ifunc(void)
{
	if (!Func_memmove_nodrain)
		pmem_init_detect();

	ASSERT(Func_memmove_nodrain != NULL);

	return Func_memmove_nodrain;
}

void *
pmem_memcpy_nodrain(void *pmemdest, const void *src, size_t len)
__attribute__((ifunc("pmem_memcpy_nodrain_ifunc")));
#else
/*
 * pmem_memcpy_nodrain -- memcpy to pmem without hw drain
 */
void *
pmem_memcpy_nodrain(void *pmemdest, const void *src, size_t len)
{
	LOG(15, "pmemdest %p src %p len %zu", pmemdest, src, len);

	return Func_memmove_nodrain(pmemdest, src, len);
}
#endif

/*
 * pmem_memmove_persist -- memmove to pmem
 */
void *
pmem_memmove_persist(void *pmemdest, const void *src, size_t len)
{
	LOG(15, "pmemdest %p src %p len %zu", pmemdest, src, len);

	pmem_memmove_nodrain(pmemdest, src, len);
	pmem_drain();
	return pmemdest;
}

/*
 * pmem_memcpy_persist -- memcpy to pmem
 */
void *
pmem_memcpy_persist(void *pmemdest, const void *src, size_t len)
{
	LOG(15, "pmemdest %p src %p len %zu", pmemdest, src, len);

	pmem_memcpy_nodrain(pmemdest, src, len);
	pmem_drain();
	return pmemdest;
}

/*
 * memset_nodrain_normal -- (internal) memset to pmem without hw drain, normal
 */
static void *
memset_nodrain_normal(void *pmemdest, int c, size_t len)
{
	LOG(15, "pmemdest %p c 0x%x len %zu", pmemdest, c, len);

	memset(pmemdest, c, len);
	pmem_flush(pmemdest, len);
	return pmemdest;
}

/*
 * memset_nodrain_movnt -- (internal) memset to pmem without hw drain, movnt
 */
static void *
memset_nodrain_movnt(void *pmemdest, int c, size_t len)
{
	LOG(15, "pmemdest %p c 0x%x len %zu", pmemdest, c, len);

	size_t i;
	void *dest1 = pmemdest;
	size_t cnt;
	__m128i xmm0;
	__m128i *d;

	if (len < Movnt_threshold) {
		memset(pmemdest, c, len);
		pmem_flush(pmemdest, len);
		return pmemdest;
	}

	/* memset up to the next FLUSH_ALIGN boundary */
	cnt = (uint64_t)dest1 & ALIGN_MASK;
	if (cnt != 0) {
		cnt = FLUSH_ALIGN - cnt;

		if (cnt > len)
			cnt = len;

		memset(dest1, c, cnt);
		pmem_flush(dest1, cnt);
		len -= cnt;
		dest1 = (char *)dest1 + cnt;
	}

	xmm0 = _mm_set1_epi8((char)c);

	d = (__m128i *)dest1;
	cnt = len / CHUNK_SIZE;
	if (cnt != 0) {
		for (i = 0; i < cnt; i++) {
			_mm_stream_si128(d, xmm0);
			_mm_stream_si128(d + 1, xmm0);
			_mm_stream_si128(d + 2, xmm0);
			_mm_stream_si128(d + 3, xmm0);
			_mm_stream_si128(d + 4, xmm0);
			_mm_stream_si128(d + 5, xmm0);
			_mm_stream_si128(d + 6, xmm0);
			_mm_stream_si128(d + 7, xmm0);
			VALGRIND_DO_FLUSH(d, 8 * sizeof (*d));
			d += 8;
		}
	}
	/* memset the tail (<128 bytes) in 16 bytes chunks */
	len &= CHUNK_MASK;
	if (len != 0) {
		cnt = len >> MOVNT_SHIFT;
		for (i = 0; i < cnt; i++) {
			_mm_stream_si128(d, xmm0);
			VALGRIND_DO_FLUSH(d, sizeof (*d));
			d++;
		}
	}

	/* memset the last bytes (<16), first dwords then bytes */
	len &= MOVNT_MASK;
	if (len != 0) {
		int32_t *d32 = (int32_t *)d;
		cnt = len >> DWORD_SHIFT;
		if (cnt != 0) {
			for (i = 0; i < cnt; i++) {
				_mm_stream_si32(d32,
					_mm_cvtsi128_si32(xmm0));
				VALGRIND_DO_FLUSH(d32, sizeof (*d32));
				d32++;
			}
		}

		/* at this point the cnt < 16 so use memset */
		cnt = len & DWORD_MASK;
		if (cnt != 0) {
			memset((void *)d32, c, cnt);
			pmem_flush(d32, cnt);
		}
	}

	/* serialize non-temporal store instructions */
	predrain_fence_sfence();

	return pmemdest;
}

/*
 * pmem_memset_nodrain() calls through Func_memset_nodrain to do the work.
 * Although initialized to memset_nodrain_normal(), once the existence of the
 * sse2 feature is confirmed by pmem_init() at library initialization time,
 * Func_memset_nodrain is set to memset_nodrain_movnt().  That's the most
 * common case on modern hardware that supports persistent memory.
 */
static void *(*Func_memset_nodrain)
	(void *pmemdest, int c, size_t len);

#if HAS_IFUNC
/*
 * pmem_memset_nodrain_ifunc -- (internal) return pointer to
 * pmem_memset_nodrain function
 */
static void *
pmem_memset_nodrain_ifunc(void)
{
	if (!Func_memset_nodrain)
		pmem_init_detect();

	ASSERT(Func_memset_nodrain != NULL);

	return Func_memset_nodrain;
}

void *
pmem_memset_nodrain(void *pmemdest, int c, size_t len)
__attribute__((ifunc("pmem_memset_nodrain_ifunc")));
#else
/*
 * pmem_memset_nodrain -- memset to pmem without hw drain
 */
void *
pmem_memset_nodrain(void *pmemdest, int c, size_t len)
{
	return Func_memset_nodrain(pmemdest, c, len);
}
#endif

/*
 * pmem_memset_persist -- memset to pmem
 */
void *
pmem_memset_persist(void *pmemdest, int c, size_t len)
{
	LOG(15, "pmemdest %p c 0x%x len %zu", pmemdest, c, len);

	pmem_memset_nodrain(pmemdest, c, len);
	pmem_drain();
	return pmemdest;
}

/*
 * pmem_get_cpuinfo -- configure libpmem based on CPUID
 */
static void
pmem_get_cpuinfo(void)
{
	if (is_cpu_clflush_present()) {
		Func_is_pmem = is_pmem_proc;
		LOG(3, "clflush supported");
	}

	if (is_cpu_clflushopt_present()) {
		LOG(3, "clflushopt supported");

		char *e = getenv("PMEM_NO_CLFLUSHOPT");
		if (e && strcmp(e, "1") == 0)
			LOG(3, "PMEM_NO_CLFLUSHOPT forced no clflushopt");
		else {
			FF_SET(Func_flags, FLUSH, CLFLUSHOPT);
			FF_SET(Func_flags, PDF, PDF_SFENCE);
		}
	}

	if (is_cpu_clwb_present()) {
		LOG(3, "clwb supported");

		char *e = getenv("PMEM_NO_CLWB");
		if (e && strcmp(e, "1") == 0)
			LOG(3, "PMEM_NO_CLWB forced no clwb");
		else {
			FF_SET(Func_flags, FLUSH, CLWB);
			FF_SET(Func_flags, PDF, PDF_SFENCE);
		}
	}

	if (is_cpu_pcommit_present()) {
		LOG(3, "pcommit supported");

		char *e = getenv("PMEM_NO_PCOMMIT");
		if (e && strcmp(e, "1") == 0)
			LOG(3, "PMEM_NO_PCOMMIT forced no pcommit");
		else {
			FF_SET(Func_flags, PCOMMIT, PCOMMIT);
			Has_hw_drain = 1;
		}
	}

	if (is_cpu_sse2_present()) {
		LOG(3, "movnt supported");

		char *e = getenv("PMEM_NO_MOVNT");
		if (e && strcmp(e, "1") == 0)
			LOG(3, "PMEM_NO_MOVNT forced no movnt");
		else {
			FF_SET(Func_flags, MOVNT, MOVNT);
		}
	}
}

/*
 * pmem_assign_func -- (internal) assigns required function pointers based
 * on values from Func_flags.
 */
static void
pmem_assign_func(void)
{
	/* check and report pcommit support */
	switch (FF_GET_PCOMMIT(Func_flags)) {
	case FF_NO_PCOMMIT:
		LOG(3, "not using pcommit");
		break;
	case FF_PCOMMIT:
		LOG(3, "using pcommit");
		break;
	default:
		ASSERT(0);
	}

	/* check and report pre-drain fence */
	switch (FF_GET_PDF(Func_flags)) {
	case FF_PDF_EMPTY:
		LOG(3, "not using pre-drain fence");
		break;
	case FF_PDF_SFENCE:
		LOG(3, "using pre-drain sfence");
		break;
	default:
		ASSERT(0);
	}

	/* check, report and assign flush function */
	switch (FF_GET_FLUSH(Func_flags)) {
	case FF_CLFLUSH:
		LOG(3, "using clflush");
		Func_flush = flush_clflush;
		break;
	case FF_CLFLUSHOPT:
		LOG(3, "using clflushopt");
		Func_flush = flush_clflushopt;
		break;
	case FF_CLWB:
		LOG(3, "using clwb");
		Func_flush = flush_clwb;
		break;
	default:
		ASSERT(0);
	}

	/* check and assign drain function */
	switch (FF_GET_DRAIN(Func_flags)) {
	case (FF_NO_PCOMMIT | FF_PDF_EMPTY):
		Func_drain = drain_no_pcommit_predrain_fence_empty;
		break;
	case (FF_NO_PCOMMIT | FF_PDF_SFENCE):
		Func_drain = drain_no_pcommit_predrain_fence_sfence;
		break;
	case (FF_PCOMMIT | FF_PDF_EMPTY):
		Func_drain = drain_pcommit_predrain_fence_empty;
		break;
	case (FF_PCOMMIT | FF_PDF_SFENCE):
		Func_drain = drain_pcommit_predrain_fence_sfence;
		break;
	default:
		ASSERT(0);
	}

	/* check and assign persist function */
	switch (FF_GET_PERSIST(Func_flags)) {
	case (FF_CLFLUSH | FF_NO_PCOMMIT):
		Func_persist = persist_clflush_no_pcommit;
		break;
	case (FF_CLFLUSH | FF_PCOMMIT):
		Func_persist = persist_clflush_pcommit;
		break;
	case (FF_CLFLUSHOPT | FF_NO_PCOMMIT):
		Func_persist = persist_clflushopt_no_pcommit;
		break;
	case (FF_CLFLUSHOPT | FF_PCOMMIT):
		Func_persist = persist_clflushopt_pcommit;
		break;
	case (FF_CLWB | FF_NO_PCOMMIT):
		Func_persist = persist_clwb_no_pcommit;
		break;
	case (FF_CLWB | FF_PCOMMIT):
		Func_persist = persist_clwb_pcommit;
		break;
	default:
		ASSERT(0);
	}

	/*
	 * check, report and assign memmove_nodrain and memset_nodrain
	 * functions
	 */
	switch (FF_GET_MOVNT(Func_flags)) {
	case FF_NO_MOVNT:
		LOG(3, "not using movnt");
		Func_memmove_nodrain = memmove_nodrain_normal;
		Func_memset_nodrain = memset_nodrain_normal;
		break;
	case FF_MOVNT:
		LOG(3, "using movnt");
		Func_memmove_nodrain = memmove_nodrain_movnt;
		Func_memset_nodrain = memset_nodrain_movnt;
		break;
	default:
		ASSERT(0);
	}
}

/*
 * pmem_init_detect -- detect supported instructions by hw
 */
static void
pmem_init_detect(void)
{
	/* detect supported cache flush features */
	pmem_get_cpuinfo();

	/*
	 * For testing, allow overriding the default threshold
	 * for using non-temporal stores in pmem_memcpy_*(), pmem_memmove_*()
	 * and pmem_memset_*().
	 * It has no effect if movnt is not supported or disabled.
	 */
	char *ptr = getenv("PMEM_MOVNT_THRESHOLD");
	if (ptr) {
		long long val = atoll(ptr);

		if (val < 0)
			LOG(3, "Invalid PMEM_MOVNT_THRESHOLD");
		else {
			LOG(3, "PMEM_MOVNT_THRESHOLD set to %zu", (size_t)val);
			Movnt_threshold = (size_t)val;
		}
	}

	/*
	 * For debugging/testing, allow pmem_is_pmem() to be forced
	 * to always true or never true using environment variable
	 * PMEM_IS_PMEM_FORCE values of zero or one.
	 *
	 * This isn't #ifdef DEBUG because it has a trivial performance
	 * impact and it may turn out to be useful as a "chicken bit" for
	 * systems where pmem_is_pmem() isn't correctly detecting true
	 * persistent memory.
	 */
	ptr = getenv("PMEM_IS_PMEM_FORCE");
	if (ptr) {
		int val = atoi(ptr);

		if (val == 0)
			Func_is_pmem = is_pmem_never;
		else if (val == 1)
			Func_is_pmem = is_pmem_always;
	}

	pmem_assign_func();
}

/*
 * pmem_init -- load-time initialization for pmem.c
 */
void
pmem_init(void)
{
	out_init(PMEM_LOG_PREFIX, PMEM_LOG_LEVEL_VAR, PMEM_LOG_FILE_VAR,
			PMEM_MAJOR_VERSION, PMEM_MINOR_VERSION);
	LOG(3, NULL);
	util_init();

	pmem_init_detect();
}
