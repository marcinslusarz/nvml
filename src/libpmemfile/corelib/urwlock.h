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
#ifndef PMEMFILE_URWLOCK_H
#define PMEMFILE_URWLOCK_H

#include <stdint.h>
#include "out.h"

struct urwlock {
	volatile uint64_t data;
};

static inline void urwlock_init(struct urwlock *lock)
{
	lock->data = 0;
}

static inline void urwlock_rlock(struct urwlock *lock)
{
	uint64_t oldval, newval;
	do {
		oldval = lock->data & 0xffffffff;
		newval = oldval + 1;
	} while (!__sync_bool_compare_and_swap(&lock->data, oldval, newval));
}

static inline void urwlock_wlock(struct urwlock *lock)
{
	while (!__sync_bool_compare_and_swap(&lock->data, 0, 1UL << 32))
		;
}

static inline void urwlock_unlock(struct urwlock *lock)
{
	if (lock->data & (1UL << 32)) {
		if (!__sync_bool_compare_and_swap(&lock->data, 1UL << 32, 0))
			FATAL("impossible");
	} else {
		uint64_t oldval, newval;
		do {
			oldval = lock->data & 0xffffffff;
			newval = oldval - 1;
		} while (!__sync_bool_compare_and_swap(&lock->data, oldval,
				newval));
	}
}

static inline void urwlock_destroy(struct urwlock *lock)
{
}

#endif
