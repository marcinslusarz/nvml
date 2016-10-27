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
 * pool.c -- pool file operations
 */

#include <errno.h>

#include "callbacks.h"
#include "dir.h"
#include "inode.h"
#include "internal.h"
#include "locks.h"
#include "out.h"
#include "pool.h"
#include "sys_util.h"
#include "util.h"

/*
 * file_initialize_super -- (internal) initializes super block
 */
static int
file_initialize_super(PMEMfilepool *pfp)
{
	LOG(LDBG, "pfp %p", pfp);

	int err = 0;
	struct pmemfile_super *super = D_RW(pfp->super);

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		if (super->initialized) {
			pfp->root = file_vinode_ref(pfp, super->root_inode);
#ifdef DEBUG
			pfp->root->path = Strdup("/");
#endif
		} else {
			pfp->root = file_new_dir(pfp, NULL, "/");

			TX_ADD(pfp->super);
			super->root_inode = pfp->root->inode;
			super->initialized = 1;
		}
	} TX_ONABORT {
		err = -1;
	} TX_END

	if (err) {
		ERR("cannot initialize super block");
		return err;
	}

	return 0;
}

/*
 * file_cleanup_inode_array_single -- (internal) cleans up one batch of inodes
 */
static void
file_cleanup_inode_array_single(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode_array) single)
{
	LOG(LDBG, "pfp %p arr 0x%lx", pfp, single.oid.off);

	struct pmemfile_inode_array *op = D_RW(single);
	for (unsigned i = 0; op->used && i < NUMINODES_PER_ENTRY; ++i) {
		if (TOID_IS_NULL(op->inodes[i]))
			continue;

		LOG(LINF, "closing inode left by previous run");

		if (D_RW(op->inodes[i])->nlink == 0)
			file_inode_free(pfp, op->inodes[i]);

		op->inodes[i] = TOID_NULL(struct pmemfile_inode);

		op->used--;
	}

	ASSERTeq(op->used, 0);
}

/*
 * file_cleanup_inode_array -- (internal) removes inodes (and frees if there are
 * no dentries referencing it) from specified list
 */
static void
file_cleanup_inode_array(PMEMfilepool *pfp,
		TOID(struct pmemfile_inode_array) single)
{
	LOG(LDBG, "pfp %p", pfp);

	TX_BEGIN_CB(pfp->pop, cb_queue, pfp) {
		TOID(struct pmemfile_inode_array) last = single;

		for (; !TOID_IS_NULL(single); single = D_RO(single)->next) {
			last = single;

			/*
			 * Both used and unused arrays will be changed. Used
			 * here, unused in the following loop.
			 */
			TX_ADD(single);

			if (D_RO(single)->used > 0)
				file_cleanup_inode_array_single(pfp, single);
		}

		if (!TOID_IS_NULL(last)) {
			TOID(struct pmemfile_inode_array) prev;

			while (!TOID_IS_NULL(D_RO(last)->prev)) {
				prev = D_RO(last)->prev;
				TX_FREE(last);
				last = prev;
			}

			D_RW(last)->next =
					TOID_NULL(struct pmemfile_inode_array);
		}
	} TX_ONABORT {
		FATAL("!cannot cleanup list of previously deleted files");
	} TX_END
}

/*
 * pmemfile_mkfs -- create pmem file system on specified file
 */
PMEMfilepool *
pmemfile_mkfs(const char *pathname, size_t poolsize, mode_t mode)
{
	LOG(LDBG, "pathname %s poolsize %zu mode %o", pathname, poolsize, mode);

	PMEMfilepool *pfp = Zalloc(sizeof(*pfp));
	if (!pfp)
		return NULL;

	int oerrno;
	pfp->pop = pmemobj_create(pathname, POBJ_LAYOUT_NAME(pmemfile),
			poolsize, mode);
	if (!pfp->pop) {
		oerrno = errno;
		ERR("pmemobj_create failed: %s", pmemobj_errormsg());
		goto pool_create;
	}

	pfp->super = POBJ_ROOT(pfp->pop, struct pmemfile_super);
	if (TOID_IS_NULL(pfp->super)) {
		oerrno = ENODEV;
		ERR("cannot initialize super block");
		goto init_super;
	}
	util_rwlock_init(&pfp->rwlock);
	pfp->inode_map = file_inode_map_alloc();

	if (file_initialize_super(pfp)) {
		oerrno = errno;
		goto init_super;
	}

	return pfp;

init_super:
	pmemobj_close(pfp->pop);
pool_create:
	Free(pfp);
	errno = oerrno;
	return NULL;
}

/*
 * pmemfile_pool_open -- open pmem file system
 */
PMEMfilepool *
pmemfile_pool_open(const char *pathname)
{
	LOG(LDBG, "pathname %s", pathname);

	PMEMfilepool *pfp = Zalloc(sizeof(*pfp));
	if (!pfp)
		return NULL;

	int oerrno;
	pfp->pop = pmemobj_open(pathname, POBJ_LAYOUT_NAME(pmemfile));
	if (!pfp->pop) {
		oerrno = errno;
		ERR("pmemobj_open failed: %s", pmemobj_errormsg());
		goto pool_open;
	}

	pfp->super = (TOID(struct pmemfile_super))pmemobj_root(pfp->pop, 0);
	if (pmemobj_root_size(pfp->pop) != sizeof(struct pmemfile_super)) {
		oerrno = ENODEV;
		ERR("pool in file %s is not initialized", pathname);
		goto no_super;
	}
	util_rwlock_init(&pfp->rwlock);
	pfp->inode_map = file_inode_map_alloc();

	if (file_initialize_super(pfp)) {
		oerrno = errno;
		goto no_super;
	}

	file_cleanup_inode_array(pfp, D_RO(pfp->super)->orphaned_inodes);

	return pfp;

no_super:
	pmemobj_close(pfp->pop);
pool_open:
	Free(pfp);
	errno = oerrno;
	return NULL;
}

/*
 * pmemfile_pool_close -- close pmem file system
 */
void
pmemfile_pool_close(PMEMfilepool *pfp)
{
	LOG(LDBG, "pfp %p", pfp);

	file_vinode_unref_tx(pfp, pfp->root);
	file_inode_map_free(pfp->inode_map);
	util_rwlock_destroy(&pfp->rwlock);

	pmemobj_close(pfp->pop);
	pfp->pop = NULL;

	Free(pfp);
}
