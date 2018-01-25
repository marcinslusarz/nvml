/*
 * Copyright 2015-2018, Intel Corporation
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
 * obj_persist_count.c -- counting number of persists
 */
#define _GNU_SOURCE

#include "obj.h"
#include "pmalloc.h"
#include "unittest.h"

static struct {
	int n_cl_stores;
	int n_drain;
	int n_pmem_persist;
	int n_pmem_msync;
	int n_pmem_flush;
	int n_pmem_drain;
	int n_flush_from_pmem_memcpy;
	int n_flush_from_pmem_memset;
	int n_drain_from_pmem_memcpy;
	int n_drain_from_pmem_memset;
} ops_counter;

#define FLUSH_ALIGN ((uintptr_t)64)

static int
length_aligned(const void *addr, size_t len, uintptr_t alignment)
{
	uintptr_t start = (uintptr_t)addr & ~(alignment - 1);
	uintptr_t end = ((uintptr_t)addr + len + alignment - 1) &
			~(alignment - 1);
	return end - start;
}

static void
flush_cl(const void *addr, size_t len)
{
	ops_counter.n_cl_stores +=
			length_aligned(addr, len, FLUSH_ALIGN) / FLUSH_ALIGN;
}

static void
flush_msync(const void *addr, size_t len)
{
	ops_counter.n_cl_stores +=
			length_aligned(addr, len, Pagesize) / FLUSH_ALIGN;
}

FUNC_MOCK(pmem_persist, void, const void *addr, size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		ops_counter.n_pmem_persist++;
		flush_cl(addr, len);
		ops_counter.n_drain++;

		_FUNC_REAL(pmem_persist)(addr, len);
	}
FUNC_MOCK_END

FUNC_MOCK(pmem_msync, int, const void *addr, size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		ops_counter.n_pmem_msync++;
		flush_msync(addr, len);
		ops_counter.n_drain++;

		return _FUNC_REAL(pmem_msync)(addr, len);
	}
FUNC_MOCK_END

FUNC_MOCK(pmem_flush, void, const void *addr, size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		ops_counter.n_pmem_flush++;
		flush_cl(addr, len);
		_FUNC_REAL(pmem_flush)(addr, len);
	}
FUNC_MOCK_END

FUNC_MOCK(pmem_drain, void, void)
	FUNC_MOCK_RUN_DEFAULT {
		ops_counter.n_pmem_drain++;
		ops_counter.n_drain++;
		_FUNC_REAL(pmem_drain)();
	}
FUNC_MOCK_END

_FUNC_REAL_DECL(pmem_memmove, void *, int flags, void *dest, const void *src,
		size_t len);

static void *
mocked_memmove(int flags, void *dest, const void *src, size_t len)
{
	size_t orig_len = len;
	size_t cnt = (uint64_t)dest & 63;
	if (cnt > 0) {
		if (cnt > len)
			cnt = len;

		len -= cnt;

		ops_counter.n_flush_from_pmem_memcpy++;
		ops_counter.n_cl_stores++;
	}
	ops_counter.n_flush_from_pmem_memcpy +=
				(len + FLUSH_ALIGN - 1) / FLUSH_ALIGN;
	ops_counter.n_cl_stores += (len + FLUSH_ALIGN - 1) / FLUSH_ALIGN;

	if (!(flags & PMEM_MEM_NODRAIN)) {
		ops_counter.n_drain_from_pmem_memcpy++;
		ops_counter.n_drain++;
	}

	return _FUNC_REAL(pmem_memmove)(flags, dest, src, orig_len);
}

FUNC_MOCK(pmem_memcpy_persist, void *, void *dest, const void *src, size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		return mocked_memmove(0, dest, src, len);
	}
FUNC_MOCK_END

FUNC_MOCK(pmem_memcpy_nodrain, void *, void *dest, const void *src, size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		return mocked_memmove(PMEM_MEM_NODRAIN, dest, src, len);
	}
FUNC_MOCK_END

FUNC_MOCK(pmem_memcpy, void *, int flags, void *dest, const void *src,
		size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		return mocked_memmove(flags, dest, src, len);
	}
FUNC_MOCK_END

FUNC_MOCK(pmem_memmove_persist, void *, void *dest, const void *src, size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		return mocked_memmove(0, dest, src, len);
	}
FUNC_MOCK_END

FUNC_MOCK(pmem_memmove_nodrain, void *, void *dest, const void *src, size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		return mocked_memmove(PMEM_MEM_NODRAIN, dest, src, len);
	}
FUNC_MOCK_END

FUNC_MOCK(pmem_memmove, void *, int flags, void *dest, const void *src,
		size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		return mocked_memmove(flags, dest, src, len);
	}
FUNC_MOCK_END

_FUNC_REAL_DECL(pmem_memset, void *, int flags, void *dest, int c, size_t len);

static void *
mocked_memset(int flags, void *dest, int c, size_t len)
{
	size_t orig_len = len;
	size_t cnt = (uint64_t)dest & 63;
	if (cnt > 0) {
		if (cnt > len)
			cnt = len;

		len -= cnt;

		ops_counter.n_flush_from_pmem_memset++;
		ops_counter.n_cl_stores++;
	}
	ops_counter.n_flush_from_pmem_memset +=
				(len + FLUSH_ALIGN - 1) / FLUSH_ALIGN;
	ops_counter.n_cl_stores += (len + FLUSH_ALIGN - 1) / FLUSH_ALIGN;

	if (!(flags & PMEM_MEM_NODRAIN)) {
		ops_counter.n_drain_from_pmem_memset++;
		ops_counter.n_drain++;
	}

	return _FUNC_REAL(pmem_memset)(flags, dest, c, orig_len);
}

FUNC_MOCK(pmem_memset_persist, void *, void *dest, int c, size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		return mocked_memset(0, dest, c, len);
	}
FUNC_MOCK_END

FUNC_MOCK(pmem_memset_nodrain, void *, void *dest, int c, size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		return mocked_memset(PMEM_MEM_NODRAIN, dest, c, len);
	}
FUNC_MOCK_END

FUNC_MOCK(pmem_memset, void *, int flags, void *dest, int c, size_t len)
	FUNC_MOCK_RUN_DEFAULT {
		return mocked_memset(flags, dest, c, len);
	}
FUNC_MOCK_END

/*
 * reset_counters -- zero all counters
 */
static void
reset_counters(void)
{
	memset(&ops_counter, 0, sizeof(ops_counter));
}

/*
 * print_reset_counters -- print and then zero all counters
 */
static void
print_reset_counters(const char *task)
{
	UT_OUT(
		"%-14s %-7d %-10d %-12d %-10d %-10d %-10d %-15d %-17d %-15d %-15d",
		task,
		ops_counter.n_cl_stores,
		ops_counter.n_drain,
		ops_counter.n_pmem_persist,
		ops_counter.n_pmem_msync,
		ops_counter.n_pmem_flush,
		ops_counter.n_pmem_drain,
		ops_counter.n_flush_from_pmem_memcpy,
		ops_counter.n_drain_from_pmem_memcpy,
		ops_counter.n_flush_from_pmem_memset,
		ops_counter.n_drain_from_pmem_memset);

	reset_counters();
}

struct foo {
	int val;
	uint64_t dest;

	PMEMoid bar;
	PMEMoid bar2;
};

int
main(int argc, char *argv[])
{
	START(argc, argv, "obj_persist_count");

	if (argc != 2)
		UT_FATAL("usage: %s file-name", argv[0]);

	const char *path = argv[1];

	PMEMobjpool *pop;
	if ((pop = pmemobj_create(path, "persist_count",
			PMEMOBJ_MIN_POOL, S_IWUSR | S_IRUSR)) == NULL)
		UT_FATAL("!pmemobj_create: %s", path);

	UT_OUT(
		"%-14s %-7s %-10s %-12s %-10s %-10s %-10s %-15s %-17s %-15s %-15s",
		"task",
		"cl(all)",
		"drain(all)",
		"pmem_persist",
		"pmem_msync",
		"pmem_flush",
		"pmem_drain",
		"pmem_memcpy_cls",
		"pmem_memcpy_drain",
		"pmem_memset_cls",
		"pmem_memset_drain");


	print_reset_counters("pool_create");

	/* allocate one structure to create a run */
	pmemobj_alloc(pop, NULL, sizeof(struct foo), 0, NULL, NULL);
	reset_counters();

	PMEMoid root = pmemobj_root(pop, sizeof(struct foo));
	UT_ASSERT(!OID_IS_NULL(root));
	print_reset_counters("root_alloc");

	PMEMoid oid;
	int ret = pmemobj_alloc(pop, &oid, sizeof(struct foo), 0, NULL, NULL);
	UT_ASSERTeq(ret, 0);
	print_reset_counters("atomic_alloc");

	pmemobj_free(&oid);
	print_reset_counters("atomic_free");

	struct foo *f = pmemobj_direct(root);

	TX_BEGIN(pop) {
	} TX_END
	print_reset_counters("tx_begin_end");

	TX_BEGIN(pop) {
		f->bar = pmemobj_tx_alloc(sizeof(struct foo), 0);
		UT_ASSERT(!OID_IS_NULL(f->bar));
	} TX_END
	print_reset_counters("tx_alloc");

	TX_BEGIN(pop) {
		f->bar2 = pmemobj_tx_alloc(sizeof(struct foo), 0);
		UT_ASSERT(!OID_IS_NULL(f->bar2));
	} TX_END
	print_reset_counters("tx_alloc_next");

	TX_BEGIN(pop) {
		pmemobj_tx_free(f->bar);
	} TX_END
	print_reset_counters("tx_free");

	TX_BEGIN(pop) {
		pmemobj_tx_free(f->bar2);
	} TX_END
	print_reset_counters("tx_free_next");

	TX_BEGIN(pop) {
		pmemobj_tx_add_range_direct(&f->val, sizeof(f->val));
	} TX_END
	print_reset_counters("tx_add");

	TX_BEGIN(pop) {
		pmemobj_tx_add_range_direct(&f->val, sizeof(f->val));
	} TX_END
	print_reset_counters("tx_add_next");

	pmalloc(pop, &f->dest, sizeof(f->val), 0, 0);
	print_reset_counters("pmalloc");

	pfree(pop, &f->dest);
	print_reset_counters("pfree");

	uint64_t stack_var;
	pmalloc(pop, &stack_var, sizeof(f->val), 0, 0);
	print_reset_counters("pmalloc_stack");

	pfree(pop, &stack_var);
	print_reset_counters("pfree_stack");

	pmemobj_close(pop);

	DONE(NULL);
}


#ifdef _MSC_VER
/*
 * Since libpmemobj is linked statically, we need to invoke its ctor/dtor.
 */
MSVC_CONSTR(libpmemobj_init)
MSVC_DESTR(libpmemobj_fini)
#endif
