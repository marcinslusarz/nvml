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

#include "callbacks.h"
#include "inode.h"
#include "internal.h"
#include "locks.h"
#include "out.h"
#include "sys_util.h"
#include "../../libpmemobj/sync.h"

static void
file_util_rwlock_unlock(PMEMfilepool *pfp, pthread_rwlock_t *arg)
{
	util_rwlock_unlock(arg);
}

/*
 * file_tx_rwlock_wrlock -- transactional read-write lock
 */
void
rwlock_tx_wlock(pthread_rwlock_t *l)
{
	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	cb_push_front(TX_STAGE_ONABORT,
			(cb_basic)file_util_rwlock_unlock, l);

	util_rwlock_wrlock(l);
}

/*
 * file_tx_rwlock_unlock_on_commit -- transactional read-write unlock
 */
void
rwlock_tx_unlock_on_commit(pthread_rwlock_t *l)
{
	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	cb_push_back(TX_STAGE_ONCOMMIT,
			(cb_basic)file_util_rwlock_unlock, l);
}

static void
file_util_spin_unlock(PMEMfilepool *pfp, pthread_spinlock_t *arg)
{
	util_spin_unlock(arg);
}

/*
 * file_tx_spin_lock -- transactional spin lock
 */
void
spin_tx_lock(pthread_spinlock_t *l)
{
	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	cb_push_front(TX_STAGE_ONABORT,
			(cb_basic)file_util_spin_unlock,
			(void *)l);

	util_spin_lock(l);
}

/*
 * file_tx_spin_trylock -- transactional spin lock
 */
int
spin_tx_trylock(pthread_spinlock_t *l)
{
	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	int r = util_spin_trylock(l);
	if (r)
		return r;
	/* XXX should be done before lock */
	cb_push_front(TX_STAGE_ONABORT,
			(cb_basic)file_util_spin_unlock,
			(void *)l);
	return r;
}

/*
 * file_tx_spin_unlock_on_commit -- transactional spin unlock
 */
void
spin_tx_unlock_on_commit(pthread_spinlock_t *l)
{
	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	cb_push_back(TX_STAGE_ONCOMMIT,
			(cb_basic)file_util_spin_unlock,
			(void *)l);
}

static void
file_mutex_unlock_nofail(PMEMfilepool *pfp, PMEMmutex *mutexp)
{
	pmemobj_mutex_unlock_nofail(pfp->pop, mutexp);
}

/*
 * file_tx_pmemobj_mutex_unlock_on_abort -- postpones pmemobj_mutex_unlock on
 * transaction abort
 */
void
mutex_tx_unlock_on_abort(PMEMmutex *mutexp)
{
	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	cb_push_front(TX_STAGE_ONABORT,
			(cb_basic)file_mutex_unlock_nofail,
			mutexp);
}

/*
 * file_tx_pmemobj_mutex_lock -- transactional pmemobj_mutex_lock
 */
void
mutex_tx_lock(PMEMfilepool *pfp, PMEMmutex *mutexp)
{
	cb_push_front(TX_STAGE_ONABORT,
			(cb_basic)file_mutex_unlock_nofail,
			mutexp);

	pmemobj_mutex_lock_nofail(pfp->pop, mutexp);
}

/*
 * file_tx_pmemobj_mutex_unlock_on_commit -- postpones pmemobj_mutex_unlock on
 * transaction commit
 */
void
mutex_tx_unlock_on_commit(PMEMmutex *mutexp)
{
	ASSERTeq(pmemobj_tx_stage(), TX_STAGE_WORK);

	cb_push_back(TX_STAGE_ONCOMMIT,
			(cb_basic)file_mutex_unlock_nofail,
			mutexp);
}

/* struct pmemfile_vinode locking */
void
inode_lock_init(struct pmemfile_vinode *vinode)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
		case 2:
			util_spin_init(&vinode->lock.spin, 0);
			break;
		case 3:
		case 4:
		case 5:
			util_rwlock_init(&vinode->lock.rwlock);
			break;
	}
}

void
inode_rlock(struct pmemfile_vinode *vinode)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
			util_spin_lock(&vinode->lock.spin);
			break;
		case 2:
			while (util_spin_trylock(&vinode->lock.spin))
				sched_yield();
			break;
		case 3:
		case 4:
		case 5:
			util_rwlock_rdlock(&vinode->lock.rwlock);
			break;
	}
}

void
inode_wlock(struct pmemfile_vinode *vinode)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
			util_spin_lock(&vinode->lock.spin);
			break;
		case 2:
			while (util_spin_trylock(&vinode->lock.spin))
				sched_yield();
			break;
		case 3:
		case 4:
		case 5:
			util_rwlock_wrlock(&vinode->lock.rwlock);
			break;
	}
}

void
inode_tx_wlock(struct pmemfile_vinode *vinode)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
			spin_tx_lock(&vinode->lock.spin);
			break;
		case 2:
			while (spin_tx_trylock(&vinode->lock.spin))
				sched_yield();
			break;
		case 3:
		case 4:
		case 5:
			rwlock_tx_wlock(&vinode->lock.rwlock);
			break;
	}
}

void
inode_tx_unlock_on_commit(struct pmemfile_vinode *vinode)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
		case 2:
			spin_tx_unlock_on_commit(&vinode->lock.spin);
			break;
		case 3:
		case 4:
		case 5:
			rwlock_tx_unlock_on_commit(
					&vinode->lock.rwlock);
			break;
	}
}

void
inode_unlock(struct pmemfile_vinode *vinode)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
		case 2:
			util_spin_unlock(&vinode->lock.spin);
			break;
		case 3:
		case 4:
		case 5:
			util_rwlock_unlock(&vinode->lock.rwlock);
			break;
	}
}

void
inode_lock_destroy(struct pmemfile_vinode *vinode)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
		case 2:
			util_spin_destroy(&vinode->lock.spin);
			break;
		case 3:
		case 4:
		case 5:
			util_rwlock_destroy(&vinode->lock.rwlock);
			break;
	}
}

/* struct pmemfilepool locking */
void
pool_lock_init(struct pmemfilepool *pfp)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
		case 2:
			util_spin_init(&pfp->lock.spin, 0);
			break;
		case 3:
		case 4:
		case 5:
			util_rwlock_init(&pfp->lock.rwlock);
			break;
	}
}

void
pool_tx_wlock(struct pmemfilepool *pfp)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
			spin_tx_lock(&pfp->lock.spin);
			break;
		case 2:
			while (spin_tx_trylock(&pfp->lock.spin))
				sched_yield();
			break;
		case 3:
		case 4:
		case 5:
			rwlock_tx_wlock(&pfp->lock.rwlock);
			break;
	}
}

void
pool_tx_unlock_on_commit(struct pmemfilepool *pfp)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
		case 2:
			spin_tx_unlock_on_commit(&pfp->lock.spin);
			break;
		case 3:
		case 4:
		case 5:
			rwlock_tx_unlock_on_commit(&pfp->lock.rwlock);
			break;
	}
}

void
pool_lock_destroy(struct pmemfilepool *pfp)
{
	switch (pmemfile_contention_level) {
		case 0:
			break;
		case 1:
		case 2:
			util_spin_destroy(&pfp->lock.spin);
			break;
		case 3:
		case 4:
		case 5:
			util_rwlock_destroy(&pfp->lock.rwlock);
			break;
	}
}
