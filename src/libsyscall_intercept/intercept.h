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

#ifndef INTERCEPT_INTERCEPT_H
#define INTERCEPT_INTERCEPT_H

#define _GNU_SOURCE
#include <stdbool.h>
#include <elf.h>
#include <unistd.h>
#include <dlfcn.h>

#include "disasm_wrapper.h"

/*
 * Create wrapper functions to be called from glibc,
 * with an extra instruction taken from glibc
 * from before -- intercept_patch_with_prefix -- the syscall,
 * or after -- intercept_patch_with_postfix -- the syscall.
 */
void intercept_patch_with_prefix(unsigned char *syscall_addr);


void intercept_patch_with_postfix(unsigned char *syscall_addr,
				unsigned postfix_len);

#define INTERCEPTOR_EXIT_CODE 111

__attribute__((noreturn)) void xabort(void);

/*
 * The parch_list array stores some information on
 * whereabouts of patches made to glibc.
 * The syscall_addr pointer points to where a syscall
 *  instruction originally resided in glibc.
 * The asm_wrapper pointer points to the function
 *  called from glibc.
 * The glibc_call_patch pointer points to the exact
 *  location, where the new call instruction should
 *  be written.
 */
struct patch_desc {
	// the original syscall instruction
	unsigned char *syscall_addr;

	// the offset of the original syscall instruction
	unsigned long syscall_offset;

	// the new asm wrapper created
	unsigned char *asm_wrapper;

	// the first byte overwritten in the code
	unsigned char *dst_jmp_patch;

	//  the address to jump back to
	unsigned char *return_address;

	unsigned char *padding_addr;

	struct intercept_disasm_result preceding_ins_2;
	struct intercept_disasm_result preceding_ins;
	struct intercept_disasm_result following_ins;

	bool uses_padding;
	bool uses_prev_ins_2;
	bool uses_prev_ins;
	bool uses_next_ins;
	bool ok;
};

void patch_apply(struct patch_desc *patch);

// The size of a trampoline jump, jmp instruction + pointer
#define TRAMPOLINE_SIZE (6 + 8)

struct intercept_desc {

	/*
	 * uses_trampoline_table - For now this is decided runtime
	 * to make it easy to compare the operation of the library
	 * with and without it. If it is ok, we can remove this
	 * flag, and just always use the trampoline table.
	 */
	bool uses_trampoline_table;

	Dl_info dlinfo;

	unsigned long text_offset;
	unsigned char *text_start;
	unsigned char *text_end;
	Elf64_Half text_section_index;
	Elf64_Shdr sh_text_section;
	bool has_symtab;
	Elf64_Shdr sh_symtab_section;
	bool has_dynsym;
	Elf64_Shdr sh_dynsym_section;

	struct patch_desc *items;
	unsigned count;
	unsigned char *jump_table;

	void *c_detination;

	unsigned char *trampoline_table;
	size_t trampoline_table_size;

	unsigned char *next_trampoline;
};

bool has_jump(const struct intercept_desc *desc, unsigned char *addr);

void find_syscalls(struct intercept_desc *desc, Dl_info *dl_info);

void init_patcher(void);
void create_patch_wrappers(struct intercept_desc *desc);
void mprotect_asm_wrappers(void);

/*
 * Actually overwrite instructions in glibc.
 */
void activate_patches(struct intercept_desc *desc);

#define SYSCALL_INS_SIZE 2
#define JUMP_INS_SIZE 5
#define CALL_OPCODE 0xe8
#define JMP_OPCODE 0xe9
#define SHORT_JMP_OPCODE 0xeb
#define PUSH_IMM_OPCODE 0x68
#define NOP_OPCODE 0x90

void create_jump(unsigned char opcode, unsigned char *from, void *to);

/*
 * Symbols in the glibc text section are expected to
 * start on 16 byte aligned addresses.
 */
#define SALIGNMENT 16

#define unreachable() __builtin_unreachable()
#define return_address() __builtin_return_address(0)

#endif
