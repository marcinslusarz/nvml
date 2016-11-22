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
 * disasm_wrapper.c -- connecting the interceptor code
 * to the disassembler code from the capstone project.
 *
 * See:
 * http://www.capstone-engine.org/lang_c.html
 */

#include "intercept.h"
#include "intercept_util.h"
#include "disasm_wrapper.h"


#include <assert.h>
#include <string.h>
#include <syscall.h>
#include <capstone/capstone.h>

struct intercept_disasm_context {
	csh handle;
	cs_insn *insn;
	const unsigned char *begin;
	const unsigned char *end;
};

struct intercept_disasm_context *
intercept_disasm_init(const unsigned char *begin, const unsigned char *end)
{
	struct intercept_disasm_context *context;

	context = xmmap_anon(sizeof(*context));
	context->begin = begin;
	context->end = end;

	/*
	 * Initialize the disassembler.
	 * The handle here must be passed to capstone each time it is used.
	 */
	if (cs_open(CS_ARCH_X86, CS_MODE_64, &context->handle) != CS_ERR_OK)
		xabort();

	/*
	 * Kindly ask capstone to return some details about the instruction.
	 * Without this, it only prints the instruction, and we would need
	 * to parse the resulting string.
	 */
	cs_option(context->handle, CS_OPT_DETAIL, CS_OPT_ON);

	context->insn = cs_malloc(context->handle);

	return context;
}

void
intercept_disasm_destroy(struct intercept_disasm_context *context)
{
	cs_free(context->insn, 1);
	cs_close(&context->handle);
	(void) syscall_no_intercept(SYS_munmap, context, sizeof(*context));
}

struct intercept_disasm_result
intercept_disasm_next_instruction(struct intercept_disasm_context *context,
					const unsigned char *code)
{
	(void) context;

	struct intercept_disasm_result result;
	const unsigned char *start = code;
	size_t size = (size_t)(context->end - code + 1);
	uint64_t address = (uint64_t)code;

	if (!cs_disasm_iter(context->handle, &start, &size,
	    &address, context->insn)) {
		result.length = 0;
		return result;
	}

	result.length = context->insn->size;

	if (result.length == 0)
		return result;

	result.is_syscall = (context->insn->id == X86_INS_SYSCALL);

	result.is_call = (context->insn->id == X86_INS_CALL);

	result.is_ret = (context->insn->id == X86_INS_RET);

	result.is_rel_jump = false;
	result.is_indirect_jump = false;

	switch (context->insn->id) {
		case X86_INS_JAE:
		case X86_INS_JA:
		case X86_INS_JBE:
		case X86_INS_JB:
		case X86_INS_JCXZ:
		case X86_INS_JECXZ:
		case X86_INS_JE:
		case X86_INS_JGE:
		case X86_INS_JG:
		case X86_INS_JLE:
		case X86_INS_JL:
		case X86_INS_JMP:
		case X86_INS_JNE:
		case X86_INS_JNO:
		case X86_INS_JNP:
		case X86_INS_JNS:
		case X86_INS_JO:
		case X86_INS_JP:
		case X86_INS_JRCXZ:
		case X86_INS_JS:
			result.is_jump = true;
			break;
		default:
			result.is_jump = false;
			break;
	}

	result.has_ip_relative_opr = false;

	for (uint8_t op_i = 0;
	    op_i < context->insn->detail->x86.op_count;
	    ++op_i) {
		cs_x86_op *op = context->insn->detail->x86.operands + op_i;

		switch (op->type) {
			case X86_OP_REG:
				if (op->reg == X86_REG_IP ||
				    op->reg == X86_REG_RIP) {
					result.has_ip_relative_opr = true;
				}
				if (result.is_jump) {
					assert(!result.is_rel_jump);
					result.is_indirect_jump = true;
				}
				break;
			case X86_OP_MEM:
				if (op->mem.base == X86_REG_IP ||
				    op->mem.base == X86_REG_RIP ||
				    op->mem.index == X86_REG_IP ||
				    op->mem.index == X86_REG_RIP) {
					result.has_ip_relative_opr = true;
				}
				if (result.is_jump) {
					assert(!result.is_indirect_jump);
					result.is_rel_jump = true;
					result.jump_delta =
					    (ptrdiff_t)op->mem.disp;
					result.jump_target =
					    (code + result.length) +
					    result.jump_delta;
				}
				break;
			case X86_OP_IMM:
				if (result.is_jump) {
					assert(!result.is_indirect_jump);
					result.is_rel_jump = true;
					result.jump_target = (void *)op->imm;
					result.jump_delta =
					    (unsigned char *)op->imm - code;
				}
				break;
			default:
				break;
		}
	}

	return result;
}
