/* bpf_jit_comp.c : BPF JIT compiler
 *
 * Copyright (C) 2011-2013 Eric Dumazet (eric.dumazet@gmail.com)
 * Internal BPF Copyright (c) 2011-2014 PLUMgrid, http://plumgrid.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/netdevice.h>
#include <linux/filter.h>
#include <linux/if_vlan.h>
#include <asm/cacheflush.h>

int bpf_jit_enable __read_mostly;

/*
 * assembly code in arch/x86/net/bpf_jit.S
 */
extern u8 sk_load_word[], sk_load_half[], sk_load_byte[];
extern u8 sk_load_word_positive_offset[], sk_load_half_positive_offset[];
extern u8 sk_load_byte_positive_offset[];
extern u8 sk_load_word_negative_offset[], sk_load_half_negative_offset[];
extern u8 sk_load_byte_negative_offset[];

static inline u8 *emit_code(u8 *ptr, u32 bytes, unsigned int len)
{
	if (len == 1)
		*ptr = bytes;
	else if (len == 2)
		*(u16 *)ptr = bytes;
	else {
		*(u32 *)ptr = bytes;
		barrier();
	}
	return ptr + len;
}

#define EMIT(bytes, len)	do { prog = emit_code(prog, bytes, len); } while (0)

#define EMIT1(b1)		EMIT(b1, 1)
#define EMIT2(b1, b2)		EMIT((b1) + ((b2) << 8), 2)
#define EMIT3(b1, b2, b3)	EMIT((b1) + ((b2) << 8) + ((b3) << 16), 3)
#define EMIT4(b1, b2, b3, b4)   EMIT((b1) + ((b2) << 8) + ((b3) << 16) + ((b4) << 24), 4)
#define EMIT1_off32(b1, off) \
	do {EMIT1(b1); EMIT(off, 4); } while (0)
#define EMIT2_off32(b1, b2, off) \
	do {EMIT2(b1, b2); EMIT(off, 4); } while (0)
#define EMIT3_off32(b1, b2, b3, off) \
	do {EMIT3(b1, b2, b3); EMIT(off, 4); } while (0)
#define EMIT4_off32(b1, b2, b3, b4, off) \
	do {EMIT4(b1, b2, b3, b4); EMIT(off, 4); } while (0)

static inline bool is_imm8(int value)
{
	return value <= 127 && value >= -128;
}

static inline bool is_simm32(s64 value)
{
	return value == (s64) (s32) value;
}

/* mov dst, src */
#define EMIT_mov(DST, SRC) \
	do {if (DST != SRC) \
		EMIT3(add_2mod(0x48, DST, SRC), 0x89, add_2reg(0xC0, DST, SRC)); \
	} while (0)

static int bpf_size_to_x86_bytes(int bpf_size)
{
	if (bpf_size == BPF_W)
		return 4;
	else if (bpf_size == BPF_H)
		return 2;
	else if (bpf_size == BPF_B)
		return 1;
	else if (bpf_size == BPF_DW)
		return 4; /* imm32 */
	else
		return 0;
}

/* list of x86 cond jumps opcodes (. + s8)
 * Add 0x10 (and an extra 0x0f) to generate far jumps (. + s32)
 */
#define X86_JB  0x72
#define X86_JAE 0x73
#define X86_JE  0x74
#define X86_JNE 0x75
#define X86_JBE 0x76
#define X86_JA  0x77
#define X86_JGE 0x7D
#define X86_JG  0x7F

static inline void bpf_flush_icache(void *start, void *end)
{
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	smp_wmb();
	flush_icache_range((unsigned long)start, (unsigned long)end);
	set_fs(old_fs);
}

#define CHOOSE_LOAD_FUNC(K, func) \
	((int)K < 0 ? ((int)K >= SKF_LL_OFF ? func##_negative_offset : func) : func##_positive_offset)

/* pick a register outside of BPF range for JIT internal work */
#define AUX_REG (MAX_BPF_REG + 1)

/* the following table maps BPF registers to x64 registers.
 * x64 register r12 is unused, since if used as base address register
 * in load/store instructions, it always needs an extra byte of encoding
 */
static const int reg2hex[] = {
	[BPF_REG_0] = 0,  /* rax */
	[BPF_REG_1] = 7,  /* rdi */
	[BPF_REG_2] = 6,  /* rsi */
	[BPF_REG_3] = 2,  /* rdx */
	[BPF_REG_4] = 1,  /* rcx */
	[BPF_REG_5] = 0,  /* r8 */
	[BPF_REG_6] = 3,  /* rbx callee saved */
	[BPF_REG_7] = 5,  /* r13 callee saved */
	[BPF_REG_8] = 6,  /* r14 callee saved */
	[BPF_REG_9] = 7,  /* r15 callee saved */
	[BPF_REG_FP] = 5, /* rbp readonly */
	[AUX_REG] = 3,    /* r11 temp register */
};

/* is_ereg() == true if BPF register 'reg' maps to x64 r8..r15
 * which need extra byte of encoding.
 * rax,rcx,...,rbp have simpler encoding
 */
static inline bool is_ereg(u32 reg)
{
	if (reg == BPF_REG_5 || reg == AUX_REG ||
	    (reg >= BPF_REG_7 && reg <= BPF_REG_9))
		return true;
	else
		return false;
}

/* add modifiers if 'reg' maps to x64 registers r8..r15 */
static inline u8 add_1mod(u8 byte, u32 reg)
{
	if (is_ereg(reg))
		byte |= 1;
	return byte;
}

static inline u8 add_2mod(u8 byte, u32 r1, u32 r2)
{
	if (is_ereg(r1))
		byte |= 1;
	if (is_ereg(r2))
		byte |= 4;
	return byte;
}

/* encode 'dst_reg' register into x64 opcode 'byte' */
static inline u8 add_1reg(u8 byte, u32 dst_reg)
{
	return byte + reg2hex[dst_reg];
}

/* encode 'dst_reg' and 'src_reg' registers into x64 opcode 'byte' */
static inline u8 add_2reg(u8 byte, u32 dst_reg, u32 src_reg)
{
	return byte + reg2hex[dst_reg] + (reg2hex[src_reg] << 3);
}

static void jit_fill_hole(void *area, unsigned int size)
{
	/* fill whole space with int3 instructions */
	memset(area, 0xcc, size);
}

struct jit_context {
	unsigned int cleanup_addr; /* epilogue code offset */
	bool seen_ld_abs;
};

/* maximum number of bytes emitted while JITing one eBPF insn */
#define BPF_MAX_INSN_SIZE	128
#define BPF_INSN_SAFETY		64

static int do_jit(struct bpf_prog *bpf_prog, int *addrs, u8 *image,
		  int oldproglen, struct jit_context *ctx)
{
	struct bpf_insn *insn = bpf_prog->insnsi;
	int insn_cnt = bpf_prog->len;
	bool seen_ld_abs = ctx->seen_ld_abs | (oldproglen == 0);
	u8 temp[BPF_MAX_INSN_SIZE + BPF_INSN_SAFETY];
	int i;
	int proglen = 0;
	u8 *prog = temp;
	int stacksize = MAX_BPF_STACK +
		32 /* space for rbx, r13, r14, r15 */ +
		8 /* space for skb_copy_bits() buffer */;

	EMIT1(0x55); /* push rbp */
	EMIT3(0x48, 0x89, 0xE5); /* mov rbp,rsp */

	/* sub rsp, stacksize */
	EMIT3_off32(0x48, 0x81, 0xEC, stacksize);

	/* all classic BPF filters use R6(rbx) save it */

	/* mov qword ptr [rbp-X],rbx */
	EMIT3_off32(0x48, 0x89, 0x9D, -stacksize);

	/* bpf_convert_filter() maps classic BPF register X to R7 and uses R8
	 * as temporary, so all tcpdump filters need to spill/fill R7(r13) and
	 * R8(r14). R9(r15) spill could be made conditional, but there is only
	 * one 'bpf_error' return path out of helper functions inside bpf_jit.S
	 * The overhead of extra spill is negligible for any filter other
	 * than synthetic ones. Therefore not worth adding complexity.
	 */

	/* mov qword ptr [rbp-X],r13 */
	EMIT3_off32(0x4C, 0x89, 0xAD, -stacksize + 8);
	/* mov qword ptr [rbp-X],r14 */
	EMIT3_off32(0x4C, 0x89, 0xB5, -stacksize + 16);
	/* mov qword ptr [rbp-X],r15 */
	EMIT3_off32(0x4C, 0x89, 0xBD, -stacksize + 24);

	/* clear A and X registers */
	EMIT2(0x31, 0xc0); /* xor eax, eax */
	EMIT3(0x4D, 0x31, 0xED); /* xor r13, r13 */

	if (seen_ld_abs) {
		/* r9d : skb->len - skb->data_len (headlen)
		 * r10 : skb->data
		 */
		if (is_imm8(offsetof(struct sk_buff, len)))
			/* mov %r9d, off8(%rdi) */
			EMIT4(0x44, 0x8b, 0x4f,
			      offsetof(struct sk_buff, len));
		else
			/* mov %r9d, off32(%rdi) */
			EMIT3_off32(0x44, 0x8b, 0x8f,
				    offsetof(struct sk_buff, len));

		if (is_imm8(offsetof(struct sk_buff, data_len)))
			/* sub %r9d, off8(%rdi) */
			EMIT4(0x44, 0x2b, 0x4f,
			      offsetof(struct sk_buff, data_len));
		else
			EMIT3_off32(0x44, 0x2b, 0x8f,
				    offsetof(struct sk_buff, data_len));

		if (is_imm8(offsetof(struct sk_buff, data)))
			/* mov %r10, off8(%rdi) */
			EMIT4(0x4c, 0x8b, 0x57,
			      offsetof(struct sk_buff, data));
		else
			/* mov %r10, off32(%rdi) */
			EMIT3_off32(0x4c, 0x8b, 0x97,
				    offsetof(struct sk_buff, data));
	}

	for (i = 0; i < insn_cnt; i++, insn++) {
		const s32 imm32 = insn->imm;
		u32 dst_reg = insn->dst_reg;
		u32 src_reg = insn->src_reg;
		u8 b1 = 0, b2 = 0, b3 = 0;
		s64 jmp_offset;
		u8 jmp_cond;
		int ilen;
		u8 *func;

		switch (insn->code) {
			/* ALU */
		case BPF_ALU | BPF_ADD | BPF_X:
		case BPF_ALU | BPF_SUB | BPF_X:
		case BPF_ALU | BPF_AND | BPF_X:
		case BPF_ALU | BPF_OR | BPF_X:
		case BPF_ALU | BPF_XOR | BPF_X:
		case BPF_ALU64 | BPF_ADD | BPF_X:
		case BPF_ALU64 | BPF_SUB | BPF_X:
		case BPF_ALU64 | BPF_AND | BPF_X:
		case BPF_ALU64 | BPF_OR | BPF_X:
		case BPF_ALU64 | BPF_XOR | BPF_X:
			switch (BPF_OP(insn->code)) {
			case BPF_ADD: b2 = 0x01; break;
			case BPF_SUB: b2 = 0x29; break;
			case BPF_AND: b2 = 0x21; break;
			case BPF_OR: b2 = 0x09; break;
			case BPF_XOR: b2 = 0x31; break;
			}
			if (BPF_CLASS(insn->code) == BPF_ALU64)
				EMIT1(add_2mod(0x48, dst_reg, src_reg));
			else if (is_ereg(dst_reg) || is_ereg(src_reg))
				EMIT1(add_2mod(0x40, dst_reg, src_reg));
			EMIT2(b2, add_2reg(0xC0, dst_reg, src_reg));
			break;

			/* mov dst, src */
		case BPF_ALU64 | BPF_MOV | BPF_X:
			EMIT_mov(dst_reg, src_reg);
			break;

			/* mov32 dst, src */
		case BPF_ALU | BPF_MOV | BPF_X:
			if (is_ereg(dst_reg) || is_ereg(src_reg))
				EMIT1(add_2mod(0x40, dst_reg, src_reg));
			EMIT2(0x89, add_2reg(0xC0, dst_reg, src_reg));
			break;

			/* neg dst */
		case BPF_ALU | BPF_NEG:
		case BPF_ALU64 | BPF_NEG:
			if (BPF_CLASS(insn->code) == BPF_ALU64)
				EMIT1(add_1mod(0x48, dst_reg));
			else if (is_ereg(dst_reg))
				EMIT1(add_1mod(0x40, dst_reg));
			EMIT2(0xF7, add_1reg(0xD8, dst_reg));
			break;

		case BPF_ALU | BPF_ADD | BPF_K:
		case BPF_ALU | BPF_SUB | BPF_K:
		case BPF_ALU | BPF_AND | BPF_K:
		case BPF_ALU | BPF_OR | BPF_K:
		case BPF_ALU | BPF_XOR | BPF_K:
		case BPF_ALU64 | BPF_ADD | BPF_K:
		case BPF_ALU64 | BPF_SUB | BPF_K:
		case BPF_ALU64 | BPF_AND | BPF_K:
		case BPF_ALU64 | BPF_OR | BPF_K:
		case BPF_ALU64 | BPF_XOR | BPF_K:
			if (BPF_CLASS(insn->code) == BPF_ALU64)
				EMIT1(add_1mod(0x48, dst_reg));
			else if (is_ereg(dst_reg))
				EMIT1(add_1mod(0x40, dst_reg));

			switch (BPF_OP(insn->code)) {
			case BPF_ADD: b3 = 0xC0; break;
			case BPF_SUB: b3 = 0xE8; break;
			case BPF_AND: b3 = 0xE0; break;
			case BPF_OR: b3 = 0xC8; break;
			case BPF_XOR: b3 = 0xF0; break;
			}

			if (is_imm8(imm32))
				EMIT3(0x83, add_1reg(b3, dst_reg), imm32);
			else
				EMIT2_off32(0x81, add_1reg(b3, dst_reg), imm32);
			break;

		case BPF_ALU64 | BPF_MOV | BPF_K:
			/* optimization: if imm32 is positive,
			 * use 'mov eax, imm32' (which zero-extends imm32)
			 * to save 2 bytes
			 */
			if (imm32 < 0) {
				/* 'mov rax, imm32' sign extends imm32 */
				b1 = add_1mod(0x48, dst_reg);
				b2 = 0xC7;
				b3 = 0xC0;
				EMIT3_off32(b1, b2, add_1reg(b3, dst_reg), imm32);
				break;
			}

		case BPF_ALU | BPF_MOV | BPF_K:
			/* mov %eax, imm32 */
			if (is_ereg(dst_reg))
				EMIT1(add_1mod(0x40, dst_reg));
			EMIT1_off32(add_1reg(0xB8, dst_reg), imm32);
			break;

		case BPF_LD | BPF_IMM | BPF_DW:
			if (insn[1].code != 0 || insn[1].src_reg != 0 ||
			    insn[1].dst_reg != 0 || insn[1].off != 0) {
				/* verifier must catch invalid insns */
				pr_err("invalid BPF_LD_IMM64 insn\n");
				return -EINVAL;
			}

			/* movabsq %rax, imm64 */
			EMIT2(add_1mod(0x48, dst_reg), add_1reg(0xB8, dst_reg));
			EMIT(insn[0].imm, 4);
			EMIT(insn[1].imm, 4);

			insn++;
			i++;
			break;

			/* dst %= src, dst /= src, dst %= imm32, dst /= imm32 */
		case BPF_ALU | BPF_MOD | BPF_X:
		case BPF_ALU | BPF_DIV | BPF_X:
		case BPF_ALU | BPF_MOD | BPF_K:
		case BPF_ALU | BPF_DIV | BPF_K:
		case BPF_ALU64 | BPF_MOD | BPF_X:
		case BPF_ALU64 | BPF_DIV | BPF_X:
		case BPF_ALU64 | BPF_MOD | BPF_K:
		case BPF_ALU64 | BPF_DIV | BPF_K:
			EMIT1(0x50); /* push rax */
			EMIT1(0x52); /* push rdx */

			if (BPF_SRC(insn->code) == BPF_X)
				/* mov r11, src_reg */
				EMIT_mov(AUX_REG, src_reg);
			else
				/* mov r11, imm32 */
				EMIT3_off32(0x49, 0xC7, 0xC3, imm32);

			/* mov rax, dst_reg */
			EMIT_mov(BPF_REG_0, dst_reg);

			/* xor edx, edx
			 * equivalent to 'xor rdx, rdx', but one byte less
			 */
			EMIT2(0x31, 0xd2);

			if (BPF_SRC(insn->code) == BPF_X) {
				/* if (src_reg == 0) return 0 */

				/* cmp r11, 0 */
				EMIT4(0x49, 0x83, 0xFB, 0x00);

				/* jne .+9 (skip over pop, pop, xor and jmp) */
				EMIT2(X86_JNE, 1 + 1 + 2 + 5);
				EMIT1(0x5A); /* pop rdx */
				EMIT1(0x58); /* pop rax */
				EMIT2(0x31, 0xc0); /* xor eax, eax */

				/* jmp cleanup_addr
				 * addrs[i] - 11, because there are 11 bytes
				 * after this insn: div, mov, pop, pop, mov
				 */
				jmp_offset = ctx->cleanup_addr - (addrs[i] - 11);
				EMIT1_off32(0xE9, jmp_offset);
			}

			if (BPF_CLASS(insn->code) == BPF_ALU64)
				/* div r11 */
				EMIT3(0x49, 0xF7, 0xF3);
			else
				/* div r11d */
				EMIT3(0x41, 0xF7, 0xF3);

			if (BPF_OP(insn->code) == BPF_MOD)
				/* mov r11, rdx */
				EMIT3(0x49, 0x89, 0xD3);
			else
				/* mov r11, rax */
				EMIT3(0x49, 0x89, 0xC3);

			EMIT1(0x5A); /* pop rdx */
			EMIT1(0x58); /* pop rax */

			/* mov dst_reg, r11 */
			EMIT_mov(dst_reg, AUX_REG);
			break;

		case BPF_ALU | BPF_MUL | BPF_K:
		case BPF_ALU | BPF_MUL | BPF_X:
		case BPF_ALU64 | BPF_MUL | BPF_K:
		case BPF_ALU64 | BPF_MUL | BPF_X:
			EMIT1(0x50); /* push rax */
			EMIT1(0x52); /* push rdx */

			/* mov r11, dst_reg */
			EMIT_mov(AUX_REG, dst_reg);

			if (BPF_SRC(insn->code) == BPF_X)
				/* mov rax, src_reg */
				EMIT_mov(BPF_REG_0, src_reg);
			else
				/* mov rax, imm32 */
				EMIT3_off32(0x48, 0xC7, 0xC0, imm32);

			if (BPF_CLASS(insn->code) == BPF_ALU64)
				EMIT1(add_1mod(0x48, AUX_REG));
			else if (is_ereg(AUX_REG))
				EMIT1(add_1mod(0x40, AUX_REG));
			/* mul(q) r11 */
			EMIT2(0xF7, add_1reg(0xE0, AUX_REG));

			/* mov r11, rax */
			EMIT_mov(AUX_REG, BPF_REG_0);

			EMIT1(0x5A); /* pop rdx */
			EMIT1(0x58); /* pop rax */

			/* mov dst_reg, r11 */
			EMIT_mov(dst_reg, AUX_REG);
			break;

			/* shifts */
		case BPF_ALU | BPF_LSH | BPF_K:
		case BPF_ALU | BPF_RSH | BPF_K:
		case BPF_ALU | BPF_ARSH | BPF_K:
		case BPF_ALU64 | BPF_LSH | BPF_K:
		case BPF_ALU64 | BPF_RSH | BPF_K:
		case BPF_ALU64 | BPF_ARSH | BPF_K:
			if (BPF_CLASS(insn->code) == BPF_ALU64)
				EMIT1(add_1mod(0x48, dst_reg));
			else if (is_ereg(dst_reg))
				EMIT1(add_1mod(0x40, dst_reg));

			switch (BPF_OP(insn->code)) {
			case BPF_LSH: b3 = 0xE0; break;
			case BPF_RSH: b3 = 0xE8; break;
			case BPF_ARSH: b3 = 0xF8; break;
			}
			EMIT3(0xC1, add_1reg(b3, dst_reg), imm32);
			break;

		case BPF_ALU | BPF_LSH | BPF_X:
		case BPF_ALU | BPF_RSH | BPF_X:
		case BPF_ALU | BPF_ARSH | BPF_X:
		case BPF_ALU64 | BPF_LSH | BPF_X:
		case BPF_ALU64 | BPF_RSH | BPF_X:
		case BPF_ALU64 | BPF_ARSH | BPF_X:

			/* check for bad case when dst_reg == rcx */
			if (dst_reg == BPF_REG_4) {
				/* mov r11, dst_reg */
				EMIT_mov(AUX_REG, dst_reg);
				dst_reg = AUX_REG;
			}

			if (src_reg != BPF_REG_4) { /* common case */
				EMIT1(0x51); /* push rcx */

				/* mov rcx, src_reg */
				EMIT_mov(BPF_REG_4, src_reg);
			}

			/* shl %rax, %cl | shr %rax, %cl | sar %rax, %cl */
			if (BPF_CLASS(insn->code) == BPF_ALU64)
				EMIT1(add_1mod(0x48, dst_reg));
			else if (is_ereg(dst_reg))
				EMIT1(add_1mod(0x40, dst_reg));

			switch (BPF_OP(insn->code)) {
			case BPF_LSH: b3 = 0xE0; break;
			case BPF_RSH: b3 = 0xE8; break;
			case BPF_ARSH: b3 = 0xF8; break;
			}
			EMIT2(0xD3, add_1reg(b3, dst_reg));

			if (src_reg != BPF_REG_4)
				EMIT1(0x59); /* pop rcx */

			if (insn->dst_reg == BPF_REG_4)
				/* mov dst_reg, r11 */
				EMIT_mov(insn->dst_reg, AUX_REG);
			break;

		case BPF_ALU | BPF_END | BPF_FROM_BE:
			switch (imm32) {
			case 16:
				/* emit 'ror %ax, 8' to swap lower 2 bytes */
				EMIT1(0x66);
				if (is_ereg(dst_reg))
					EMIT1(0x41);
				EMIT3(0xC1, add_1reg(0xC8, dst_reg), 8);

				/* emit 'movzwl eax, ax' */
				if (is_ereg(dst_reg))
					EMIT3(0x45, 0x0F, 0xB7);
				else
					EMIT2(0x0F, 0xB7);
				EMIT1(add_2reg(0xC0, dst_reg, dst_reg));
				break;
			case 32:
				/* emit 'bswap eax' to swap lower 4 bytes */
				if (is_ereg(dst_reg))
					EMIT2(0x41, 0x0F);
				else
					EMIT1(0x0F);
				EMIT1(add_1reg(0xC8, dst_reg));
				break;
			case 64:
				/* emit 'bswap rax' to swap 8 bytes */
				EMIT3(add_1mod(0x48, dst_reg), 0x0F,
				      add_1reg(0xC8, dst_reg));
				break;
			}
			break;

		case BPF_ALU | BPF_END | BPF_FROM_LE:
			switch (imm32) {
			case 16:
				/* emit 'movzwl eax, ax' to zero extend 16-bit
				 * into 64 bit
				 */
				if (is_ereg(dst_reg))
					EMIT3(0x45, 0x0F, 0xB7);
				else
					EMIT2(0x0F, 0xB7);
				EMIT1(add_2reg(0xC0, dst_reg, dst_reg));
				break;
			case 32:
				/* emit 'mov eax, eax' to clear upper 32-bits */
				if (is_ereg(dst_reg))
					EMIT1(0x45);
				EMIT2(0x89, add_2reg(0xC0, dst_reg, dst_reg));
				break;
			case 64:
				/* nop */
				break;
			}
			break;

			/* ST: *(u8*)(dst_reg + off) = imm */
		case BPF_ST | BPF_MEM | BPF_B:
			if (is_ereg(dst_reg))
				EMIT2(0x41, 0xC6);
			else
				EMIT1(0xC6);
			goto st;
		case BPF_ST | BPF_MEM | BPF_H:
			if (is_ereg(dst_reg))
				EMIT3(0x66, 0x41, 0xC7);
			else
				EMIT2(0x66, 0xC7);
			goto st;
		case BPF_ST | BPF_MEM | BPF_W:
			if (is_ereg(dst_reg))
				EMIT2(0x41, 0xC7);
			else
				EMIT1(0xC7);
			goto st;
		case BPF_ST | BPF_MEM | BPF_DW:
			EMIT2(add_1mod(0x48, dst_reg), 0xC7);

st:			if (is_imm8(insn->off))
				EMIT2(add_1reg(0x40, dst_reg), insn->off);
			else
				EMIT1_off32(add_1reg(0x80, dst_reg), insn->off);

			EMIT(imm32, bpf_size_to_x86_bytes(BPF_SIZE(insn->code)));
			break;

			/* STX: *(u8*)(dst_reg + off) = src_reg */
		case BPF_STX | BPF_MEM | BPF_B:
			/* emit 'mov byte ptr [rax + off], al' */
			if (is_ereg(dst_reg) || is_ereg(src_reg) ||
			    /* have to add extra byte for x86 SIL, DIL regs */
			    src_reg == BPF_REG_1 || src_reg == BPF_REG_2)
				EMIT2(add_2mod(0x40, dst_reg, src_reg), 0x88);
			else
				EMIT1(0x88);
			goto stx;
		case BPF_STX | BPF_MEM | BPF_H:
			if (is_ereg(dst_reg) || is_ereg(src_reg))
				EMIT3(0x66, add_2mod(0x40, dst_reg, src_reg), 0x89);
			else
				EMIT2(0x66, 0x89);
			goto stx;
		case BPF_STX | BPF_MEM | BPF_W:
			if (is_ereg(dst_reg) || is_ereg(src_reg))
				EMIT2(add_2mod(0x40, dst_reg, src_reg), 0x89);
			else
				EMIT1(0x89);
			goto stx;
		case BPF_STX | BPF_MEM | BPF_DW:
			EMIT2(add_2mod(0x48, dst_reg, src_reg), 0x89);
stx:			if (is_imm8(insn->off))
				EMIT2(add_2reg(0x40, dst_reg, src_reg), insn->off);
			else
				EMIT1_off32(add_2reg(0x80, dst_reg, src_reg),
					    insn->off);
			break;

			/* LDX: dst_reg = *(u8*)(src_reg + off) */
		case BPF_LDX | BPF_MEM | BPF_B:
			/* emit 'movzx rax, byte ptr [rax + off]' */
			EMIT3(add_2mod(0x48, src_reg, dst_reg), 0x0F, 0xB6);
			goto ldx;
		case BPF_LDX | BPF_MEM | BPF_H:
			/* emit 'movzx rax, word ptr [rax + off]' */
			EMIT3(add_2mod(0x48, src_reg, dst_reg), 0x0F, 0xB7);
			goto ldx;
		case BPF_LDX | BPF_MEM | BPF_W:
			/* emit 'mov eax, dword ptr [rax+0x14]' */
			if (is_ereg(dst_reg) || is_ereg(src_reg))
				EMIT2(add_2mod(0x40, src_reg, dst_reg), 0x8B);
			else
				EMIT1(0x8B);
			goto ldx;
		case BPF_LDX | BPF_MEM | BPF_DW:
			/* emit 'mov rax, qword ptr [rax+0x14]' */
			EMIT2(add_2mod(0x48, src_reg, dst_reg), 0x8B);
ldx:			/* if insn->off == 0 we can save one extra byte, but
			 * special case of x86 r13 which always needs an offset
			 * is not worth the hassle
			 */
			if (is_imm8(insn->off))
				EMIT2(add_2reg(0x40, src_reg, dst_reg), insn->off);
			else
				EMIT1_off32(add_2reg(0x80, src_reg, dst_reg),
					    insn->off);
			break;

			/* STX XADD: lock *(u32*)(dst_reg + off) += src_reg */
		case BPF_STX | BPF_XADD | BPF_W:
			/* emit 'lock add dword ptr [rax + off], eax' */
			if (is_ereg(dst_reg) || is_ereg(src_reg))
				EMIT3(0xF0, add_2mod(0x40, dst_reg, src_reg), 0x01);
			else
				EMIT2(0xF0, 0x01);
			goto xadd;
		case BPF_STX | BPF_XADD | BPF_DW:
			EMIT3(0xF0, add_2mod(0x48, dst_reg, src_reg), 0x01);
xadd:			if (is_imm8(insn->off))
				EMIT2(add_2reg(0x40, dst_reg, src_reg), insn->off);
			else
				EMIT1_off32(add_2reg(0x80, dst_reg, src_reg),
					    insn->off);
			break;

			/* call */
		case BPF_JMP | BPF_CALL:
			func = (u8 *) __bpf_call_base + imm32;
			jmp_offset = func - (image + addrs[i]);
			if (seen_ld_abs) {
				EMIT2(0x41, 0x52); /* push %r10 */
				EMIT2(0x41, 0x51); /* push %r9 */
				/* need to adjust jmp offset, since
				 * pop %r9, pop %r10 take 4 bytes after call insn
				 */
				jmp_offset += 4;
			}
			if (!imm32 || !is_simm32(jmp_offset)) {
				pr_err("unsupported bpf func %d addr %p image %p\n",
				       imm32, func, image);
				return -EINVAL;
			}
			EMIT1_off32(0xE8, jmp_offset);
			if (seen_ld_abs) {
				EMIT2(0x41, 0x59); /* pop %r9 */
				EMIT2(0x41, 0x5A); /* pop %r10 */
			}
			break;

			/* cond jump */
		case BPF_JMP | BPF_JEQ | BPF_X:
		case BPF_JMP | BPF_JNE | BPF_X:
		case BPF_JMP | BPF_JGT | BPF_X:
		case BPF_JMP | BPF_JGE | BPF_X:
		case BPF_JMP | BPF_JSGT | BPF_X:
		case BPF_JMP | BPF_JSGE | BPF_X:
			/* cmp dst_reg, src_reg */
			EMIT3(add_2mod(0x48, dst_reg, src_reg), 0x39,
			      add_2reg(0xC0, dst_reg, src_reg));
			goto emit_cond_jmp;

		case BPF_JMP | BPF_JSET | BPF_X:
			/* test dst_reg, src_reg */
			EMIT3(add_2mod(0x48, dst_reg, src_reg), 0x85,
			      add_2reg(0xC0, dst_reg, src_reg));
			goto emit_cond_jmp;

		case BPF_JMP | BPF_JSET | BPF_K:
			/* test dst_reg, imm32 */
			EMIT1(add_1mod(0x48, dst_reg));
			EMIT2_off32(0xF7, add_1reg(0xC0, dst_reg), imm32);
			goto emit_cond_jmp;

		case BPF_JMP | BPF_JEQ | BPF_K:
		case BPF_JMP | BPF_JNE | BPF_K:
		case BPF_JMP | BPF_JGT | BPF_K:
		case BPF_JMP | BPF_JGE | BPF_K:
		case BPF_JMP | BPF_JSGT | BPF_K:
		case BPF_JMP | BPF_JSGE | BPF_K:
			/* cmp dst_reg, imm8/32 */
			EMIT1(add_1mod(0x48, dst_reg));

			if (is_imm8(imm32))
				EMIT3(0x83, add_1reg(0xF8, dst_reg), imm32);
			else
				EMIT2_off32(0x81, add_1reg(0xF8, dst_reg), imm32);

emit_cond_jmp:		/* convert BPF opcode to x86 */
			switch (BPF_OP(insn->code)) {
			case BPF_JEQ:
				jmp_cond = X86_JE;
				break;
			case BPF_JSET:
			case BPF_JNE:
				jmp_cond = X86_JNE;
				break;
			case BPF_JGT:
				/* GT is unsigned '>', JA in x86 */
				jmp_cond = X86_JA;
				break;
			case BPF_JGE:
				/* GE is unsigned '>=', JAE in x86 */
				jmp_cond = X86_JAE;
				break;
			case BPF_JSGT:
				/* signed '>', GT in x86 */
				jmp_cond = X86_JG;
				break;
			case BPF_JSGE:
				/* signed '>=', GE in x86 */
				jmp_cond = X86_JGE;
				break;
			default: /* to silence gcc warning */
				return -EFAULT;
			}
			jmp_offset = addrs[i + insn->off] - addrs[i];
			if (is_imm8(jmp_offset)) {
				EMIT2(jmp_cond, jmp_offset);
			} else if (is_simm32(jmp_offset)) {
				EMIT2_off32(0x0F, jmp_cond + 0x10, jmp_offset);
			} else {
				pr_err("cond_jmp gen bug %llx\n", jmp_offset);
				return -EFAULT;
			}

			break;

		case BPF_JMP | BPF_JA:
			jmp_offset = addrs[i + insn->off] - addrs[i];
			if (!jmp_offset)
				/* optimize out nop jumps */
				break;
emit_jmp:
			if (is_imm8(jmp_offset)) {
				EMIT2(0xEB, jmp_offset);
			} else if (is_simm32(jmp_offset)) {
				EMIT1_off32(0xE9, jmp_offset);
			} else {
				pr_err("jmp gen bug %llx\n", jmp_offset);
				return -EFAULT;
			}
			break;

		case BPF_LD | BPF_IND | BPF_W:
			func = sk_load_word;
			goto common_load;
		case BPF_LD | BPF_ABS | BPF_W:
			func = CHOOSE_LOAD_FUNC(imm32, sk_load_word);
common_load:
			ctx->seen_ld_abs = seen_ld_abs = true;
			jmp_offset = func - (image + addrs[i]);
			if (!func || !is_simm32(jmp_offset)) {
				pr_err("unsupported bpf func %d addr %p image %p\n",
				       imm32, func, image);
				return -EINVAL;
			}
			if (BPF_MODE(insn->code) == BPF_ABS) {
				/* mov %esi, imm32 */
				EMIT1_off32(0xBE, imm32);
			} else {
				/* mov %rsi, src_reg */
				EMIT_mov(BPF_REG_2, src_reg);
				if (imm32) {
					if (is_imm8(imm32))
						/* add %esi, imm8 */
						EMIT3(0x83, 0xC6, imm32);
					else
						/* add %esi, imm32 */
						EMIT2_off32(0x81, 0xC6, imm32);
				}
			}
			/* skb pointer is in R6 (%rbx), it will be copied into
			 * %rdi if skb_copy_bits() call is necessary.
			 * sk_load_* helpers also use %r10 and %r9d.
			 * See bpf_jit.S
			 */
			EMIT1_off32(0xE8, jmp_offset); /* call */
			break;

		case BPF_LD | BPF_IND | BPF_H:
			func = sk_load_half;
			goto common_load;
		case BPF_LD | BPF_ABS | BPF_H:
			func = CHOOSE_LOAD_FUNC(imm32, sk_load_half);
			goto common_load;
		case BPF_LD | BPF_IND | BPF_B:
			func = sk_load_byte;
			goto common_load;
		case BPF_LD | BPF_ABS | BPF_B:
			func = CHOOSE_LOAD_FUNC(imm32, sk_load_byte);
			goto common_load;

		case BPF_JMP | BPF_EXIT:
			if (i != insn_cnt - 1) {
				jmp_offset = ctx->cleanup_addr - addrs[i];
				goto emit_jmp;
			}
			/* update cleanup_addr */
			ctx->cleanup_addr = proglen;
			/* mov rbx, qword ptr [rbp-X] */
			EMIT3_off32(0x48, 0x8B, 0x9D, -stacksize);
			/* mov r13, qword ptr [rbp-X] */
			EMIT3_off32(0x4C, 0x8B, 0xAD, -stacksize + 8);
			/* mov r14, qword ptr [rbp-X] */
			EMIT3_off32(0x4C, 0x8B, 0xB5, -stacksize + 16);
			/* mov r15, qword ptr [rbp-X] */
			EMIT3_off32(0x4C, 0x8B, 0xBD, -stacksize + 24);

			EMIT1(0xC9); /* leave */
			EMIT1(0xC3); /* ret */
			break;

		default:
			/* By design x64 JIT should support all BPF instructions
			 * This error will be seen if new instruction was added
			 * to interpreter, but not to JIT
			 * or if there is junk in bpf_prog
			 */
			pr_err("bpf_jit: unknown opcode %02x\n", insn->code);
			return -EINVAL;
		}

		ilen = prog - temp;
		if (ilen > BPF_MAX_INSN_SIZE) {
			pr_err("bpf_jit_compile fatal insn size error\n");
			return -EFAULT;
		}

		if (image) {
			if (unlikely(proglen + ilen > oldproglen)) {
				pr_err("bpf_jit_compile fatal error\n");
				return -EFAULT;
			}
			memcpy(image + proglen, temp, ilen);
		}
		proglen += ilen;
		addrs[i] = proglen;
		prog = temp;
	}
	return proglen;
}

void bpf_jit_compile(struct bpf_prog *prog)
{
}

void bpf_int_jit_compile(struct bpf_prog *prog)
{
	struct bpf_binary_header *header = NULL;
	int proglen, oldproglen = 0;
	struct jit_context ctx = {};
	u8 *image = NULL;
	int *addrs;
	int pass;
	int i;

	if (!bpf_jit_enable)
		return;

	if (!prog || !prog->len)
		return;

	addrs = kmalloc(prog->len * sizeof(*addrs), GFP_KERNEL);
	if (!addrs)
		return;

	/* Before first pass, make a rough estimation of addrs[]
	 * each bpf instruction is translated to less than 64 bytes
	 */
	for (proglen = 0, i = 0; i < prog->len; i++) {
		proglen += 64;
		addrs[i] = proglen;
	}
	ctx.cleanup_addr = proglen;

	/* JITed image shrinks with every pass and the loop iterates
	 * until the image stops shrinking. Very large bpf programs
	 * may converge on the last pass. In such case do one more
	 * pass to emit the final image
	 */
	for (pass = 0; pass < 10 || image; pass++) {
<<<<<<< HEAD
		proglen = do_jit(prog, addrs, image, oldproglen, &ctx);
		if (proglen <= 0) {
			image = NULL;
			if (header)
				bpf_jit_binary_free(header);
			goto out;
		}
		if (image) {
			if (proglen != oldproglen) {
				pr_err("bpf_jit: proglen=%d != oldproglen=%d\n",
				       proglen, oldproglen);
=======
		u8 seen_or_pass0 = (pass == 0) ? (SEEN_XREG | SEEN_DATAREF | SEEN_MEM) : seen;
		/* no prologue/epilogue for trivial filters (RET something) */
		proglen = 0;
		prog = temp;

		if (seen_or_pass0) {
			EMIT4(0x55, 0x48, 0x89, 0xe5); /* push %rbp; mov %rsp,%rbp */
			EMIT4(0x48, 0x83, 0xec, 96);	/* subq  $96,%rsp	*/
			/* note : must save %rbx in case bpf_error is hit */
			if (seen_or_pass0 & (SEEN_XREG | SEEN_DATAREF))
				EMIT4(0x48, 0x89, 0x5d, 0xf8); /* mov %rbx, -8(%rbp) */
			if (seen_or_pass0 & SEEN_XREG)
				CLEAR_X(); /* make sure we dont leek kernel memory */

			/*
			 * If this filter needs to access skb data,
			 * loads r9 and r8 with :
			 *  r9 = skb->len - skb->data_len
			 *  r8 = skb->data
			 */
			if (seen_or_pass0 & SEEN_DATAREF) {
				if (offsetof(struct sk_buff, len) <= 127)
					/* mov    off8(%rdi),%r9d */
					EMIT4(0x44, 0x8b, 0x4f, offsetof(struct sk_buff, len));
				else {
					/* mov    off32(%rdi),%r9d */
					EMIT3(0x44, 0x8b, 0x8f);
					EMIT(offsetof(struct sk_buff, len), 4);
				}
				if (is_imm8(offsetof(struct sk_buff, data_len)))
					/* sub    off8(%rdi),%r9d */
					EMIT4(0x44, 0x2b, 0x4f, offsetof(struct sk_buff, data_len));
				else {
					EMIT3(0x44, 0x2b, 0x8f);
					EMIT(offsetof(struct sk_buff, data_len), 4);
				}

				if (is_imm8(offsetof(struct sk_buff, data)))
					/* mov off8(%rdi),%r8 */
					EMIT4(0x4c, 0x8b, 0x47, offsetof(struct sk_buff, data));
				else {
					/* mov off32(%rdi),%r8 */
					EMIT3(0x4c, 0x8b, 0x87);
					EMIT(offsetof(struct sk_buff, data), 4);
				}
			}
		}

		switch (filter[0].code) {
		case BPF_S_RET_K:
		case BPF_S_LD_W_LEN:
		case BPF_S_ANC_PROTOCOL:
		case BPF_S_ANC_IFINDEX:
		case BPF_S_ANC_MARK:
		case BPF_S_ANC_RXHASH:
		case BPF_S_ANC_CPU:
		case BPF_S_ANC_VLAN_TAG:
		case BPF_S_ANC_VLAN_TAG_PRESENT:
		case BPF_S_ANC_QUEUE:
		case BPF_S_ANC_PKTTYPE:
		case BPF_S_LD_W_ABS:
		case BPF_S_LD_H_ABS:
		case BPF_S_LD_B_ABS:
			/* first instruction sets A register (or is RET 'constant') */
			break;
		default:
			/* make sure we dont leak kernel information to user */
			CLEAR_A(); /* A = 0 */
		}

		for (i = 0; i < flen; i++) {
			unsigned int K = filter[i].k;

			switch (filter[i].code) {
			case BPF_S_ALU_ADD_X: /* A += X; */
				seen |= SEEN_XREG;
				EMIT2(0x01, 0xd8);		/* add %ebx,%eax */
				break;
			case BPF_S_ALU_ADD_K: /* A += K; */
				if (!K)
					break;
				if (is_imm8(K))
					EMIT3(0x83, 0xc0, K);	/* add imm8,%eax */
				else
					EMIT1_off32(0x05, K);	/* add imm32,%eax */
				break;
			case BPF_S_ALU_SUB_X: /* A -= X; */
				seen |= SEEN_XREG;
				EMIT2(0x29, 0xd8);		/* sub    %ebx,%eax */
				break;
			case BPF_S_ALU_SUB_K: /* A -= K */
				if (!K)
					break;
				if (is_imm8(K))
					EMIT3(0x83, 0xe8, K); /* sub imm8,%eax */
				else
					EMIT1_off32(0x2d, K); /* sub imm32,%eax */
				break;
			case BPF_S_ALU_MUL_X: /* A *= X; */
				seen |= SEEN_XREG;
				EMIT3(0x0f, 0xaf, 0xc3);	/* imul %ebx,%eax */
				break;
			case BPF_S_ALU_MUL_K: /* A *= K */
				if (is_imm8(K))
					EMIT3(0x6b, 0xc0, K); /* imul imm8,%eax,%eax */
				else {
					EMIT2(0x69, 0xc0);		/* imul imm32,%eax */
					EMIT(K, 4);
				}
				break;
			case BPF_S_ALU_DIV_X: /* A /= X; */
				seen |= SEEN_XREG;
				EMIT2(0x85, 0xdb);	/* test %ebx,%ebx */
				if (pc_ret0 > 0) {
					/* addrs[pc_ret0 - 1] is start address of target
					 * (addrs[i] - 4) is the address following this jmp
					 * ("xor %edx,%edx; div %ebx" being 4 bytes long)
					 */
					EMIT_COND_JMP(X86_JE, addrs[pc_ret0 - 1] -
								(addrs[i] - 4));
				} else {
					EMIT_COND_JMP(X86_JNE, 2 + 5);
					CLEAR_A();
					EMIT1_off32(0xe9, cleanup_addr - (addrs[i] - 4)); /* jmp .+off32 */
				}
				EMIT4(0x31, 0xd2, 0xf7, 0xf3); /* xor %edx,%edx; div %ebx */
				break;
			case BPF_S_ALU_MOD_X: /* A %= X; */
				seen |= SEEN_XREG;
				EMIT2(0x85, 0xdb);	/* test %ebx,%ebx */
				if (pc_ret0 > 0) {
					/* addrs[pc_ret0 - 1] is start address of target
					 * (addrs[i] - 6) is the address following this jmp
					 * ("xor %edx,%edx; div %ebx;mov %edx,%eax" being 6 bytes long)
					 */
					EMIT_COND_JMP(X86_JE, addrs[pc_ret0 - 1] -
								(addrs[i] - 6));
				} else {
					EMIT_COND_JMP(X86_JNE, 2 + 5);
					CLEAR_A();
					EMIT1_off32(0xe9, cleanup_addr - (addrs[i] - 6)); /* jmp .+off32 */
				}
				EMIT2(0x31, 0xd2);	/* xor %edx,%edx */
				EMIT2(0xf7, 0xf3);	/* div %ebx */
				EMIT2(0x89, 0xd0);	/* mov %edx,%eax */
				break;
			case BPF_S_ALU_MOD_K: /* A %= K; */
				if (K == 1) {
					CLEAR_A();
					break;
				}
				EMIT2(0x31, 0xd2);	/* xor %edx,%edx */
				EMIT1(0xb9);EMIT(K, 4);	/* mov imm32,%ecx */
				EMIT2(0xf7, 0xf1);	/* div %ecx */
				EMIT2(0x89, 0xd0);	/* mov %edx,%eax */
				break;
			case BPF_S_ALU_DIV_K: /* A /= K */
				if (K == 1)
					break;
				EMIT2(0x31, 0xd2);	/* xor %edx,%edx */
				EMIT1(0xb9);EMIT(K, 4);	/* mov imm32,%ecx */
				EMIT2(0xf7, 0xf1);	/* div %ecx */
				break;
			case BPF_S_ALU_AND_X:
				seen |= SEEN_XREG;
				EMIT2(0x21, 0xd8);		/* and %ebx,%eax */
				break;
			case BPF_S_ALU_AND_K:
				if (K >= 0xFFFFFF00) {
					EMIT2(0x24, K & 0xFF); /* and imm8,%al */
				} else if (K >= 0xFFFF0000) {
					EMIT2(0x66, 0x25);	/* and imm16,%ax */
					EMIT(K, 2);
				} else {
					EMIT1_off32(0x25, K);	/* and imm32,%eax */
				}
				break;
			case BPF_S_ALU_OR_X:
				seen |= SEEN_XREG;
				EMIT2(0x09, 0xd8);		/* or %ebx,%eax */
				break;
			case BPF_S_ALU_OR_K:
				if (is_imm8(K))
					EMIT3(0x83, 0xc8, K); /* or imm8,%eax */
				else
					EMIT1_off32(0x0d, K);	/* or imm32,%eax */
				break;
			case BPF_S_ANC_ALU_XOR_X: /* A ^= X; */
			case BPF_S_ALU_XOR_X:
				seen |= SEEN_XREG;
				EMIT2(0x31, 0xd8);		/* xor %ebx,%eax */
				break;
			case BPF_S_ALU_XOR_K: /* A ^= K; */
				if (K == 0)
					break;
				if (is_imm8(K))
					EMIT3(0x83, 0xf0, K);	/* xor imm8,%eax */
				else
					EMIT1_off32(0x35, K);	/* xor imm32,%eax */
				break;
			case BPF_S_ALU_LSH_X: /* A <<= X; */
				seen |= SEEN_XREG;
				EMIT4(0x89, 0xd9, 0xd3, 0xe0);	/* mov %ebx,%ecx; shl %cl,%eax */
				break;
			case BPF_S_ALU_LSH_K:
				if (K == 0)
					break;
				else if (K == 1)
					EMIT2(0xd1, 0xe0); /* shl %eax */
				else
					EMIT3(0xc1, 0xe0, K);
				break;
			case BPF_S_ALU_RSH_X: /* A >>= X; */
				seen |= SEEN_XREG;
				EMIT4(0x89, 0xd9, 0xd3, 0xe8);	/* mov %ebx,%ecx; shr %cl,%eax */
				break;
			case BPF_S_ALU_RSH_K: /* A >>= K; */
				if (K == 0)
					break;
				else if (K == 1)
					EMIT2(0xd1, 0xe8); /* shr %eax */
				else
					EMIT3(0xc1, 0xe8, K);
				break;
			case BPF_S_ALU_NEG:
				EMIT2(0xf7, 0xd8);		/* neg %eax */
				break;
			case BPF_S_RET_K:
				if (!K) {
					if (pc_ret0 == -1)
						pc_ret0 = i;
					CLEAR_A();
				} else {
					EMIT1_off32(0xb8, K);	/* mov $imm32,%eax */
				}
				/* fallinto */
			case BPF_S_RET_A:
				if (seen_or_pass0) {
					if (i != flen - 1) {
						EMIT_JMP(cleanup_addr - addrs[i]);
						break;
					}
					if (seen_or_pass0 & SEEN_XREG)
						EMIT4(0x48, 0x8b, 0x5d, 0xf8);  /* mov  -8(%rbp),%rbx */
					EMIT1(0xc9);		/* leaveq */
				}
				EMIT1(0xc3);		/* ret */
				break;
			case BPF_S_MISC_TAX: /* X = A */
				seen |= SEEN_XREG;
				EMIT2(0x89, 0xc3);	/* mov    %eax,%ebx */
				break;
			case BPF_S_MISC_TXA: /* A = X */
				seen |= SEEN_XREG;
				EMIT2(0x89, 0xd8);	/* mov    %ebx,%eax */
				break;
			case BPF_S_LD_IMM: /* A = K */
				if (!K)
					CLEAR_A();
				else
					EMIT1_off32(0xb8, K); /* mov $imm32,%eax */
				break;
			case BPF_S_LDX_IMM: /* X = K */
				seen |= SEEN_XREG;
				if (!K)
					CLEAR_X();
				else
					EMIT1_off32(0xbb, K); /* mov $imm32,%ebx */
				break;
			case BPF_S_LD_MEM: /* A = mem[K] : mov off8(%rbp),%eax */
				seen |= SEEN_MEM;
				EMIT3(0x8b, 0x45, 0xf0 - K*4);
				break;
			case BPF_S_LDX_MEM: /* X = mem[K] : mov off8(%rbp),%ebx */
				seen |= SEEN_XREG | SEEN_MEM;
				EMIT3(0x8b, 0x5d, 0xf0 - K*4);
				break;
			case BPF_S_ST: /* mem[K] = A : mov %eax,off8(%rbp) */
				seen |= SEEN_MEM;
				EMIT3(0x89, 0x45, 0xf0 - K*4);
				break;
			case BPF_S_STX: /* mem[K] = X : mov %ebx,off8(%rbp) */
				seen |= SEEN_XREG | SEEN_MEM;
				EMIT3(0x89, 0x5d, 0xf0 - K*4);
				break;
			case BPF_S_LD_W_LEN: /*	A = skb->len; */
				BUILD_BUG_ON(FIELD_SIZEOF(struct sk_buff, len) != 4);
				if (is_imm8(offsetof(struct sk_buff, len)))
					/* mov    off8(%rdi),%eax */
					EMIT3(0x8b, 0x47, offsetof(struct sk_buff, len));
				else {
					EMIT2(0x8b, 0x87);
					EMIT(offsetof(struct sk_buff, len), 4);
				}
				break;
			case BPF_S_LDX_W_LEN: /* X = skb->len; */
				seen |= SEEN_XREG;
				if (is_imm8(offsetof(struct sk_buff, len)))
					/* mov off8(%rdi),%ebx */
					EMIT3(0x8b, 0x5f, offsetof(struct sk_buff, len));
				else {
					EMIT2(0x8b, 0x9f);
					EMIT(offsetof(struct sk_buff, len), 4);
				}
				break;
			case BPF_S_ANC_PROTOCOL: /* A = ntohs(skb->protocol); */
				BUILD_BUG_ON(FIELD_SIZEOF(struct sk_buff, protocol) != 2);
				if (is_imm8(offsetof(struct sk_buff, protocol))) {
					/* movzwl off8(%rdi),%eax */
					EMIT4(0x0f, 0xb7, 0x47, offsetof(struct sk_buff, protocol));
				} else {
					EMIT3(0x0f, 0xb7, 0x87); /* movzwl off32(%rdi),%eax */
					EMIT(offsetof(struct sk_buff, protocol), 4);
				}
				EMIT2(0x86, 0xc4); /* ntohs() : xchg   %al,%ah */
				break;
			case BPF_S_ANC_IFINDEX:
				if (is_imm8(offsetof(struct sk_buff, dev))) {
					/* movq off8(%rdi),%rax */
					EMIT4(0x48, 0x8b, 0x47, offsetof(struct sk_buff, dev));
				} else {
					EMIT3(0x48, 0x8b, 0x87); /* movq off32(%rdi),%rax */
					EMIT(offsetof(struct sk_buff, dev), 4);
				}
				EMIT3(0x48, 0x85, 0xc0);	/* test %rax,%rax */
				EMIT_COND_JMP(X86_JE, cleanup_addr - (addrs[i] - 6));
				BUILD_BUG_ON(FIELD_SIZEOF(struct net_device, ifindex) != 4);
				EMIT2(0x8b, 0x80);	/* mov off32(%rax),%eax */
				EMIT(offsetof(struct net_device, ifindex), 4);
				break;
			case BPF_S_ANC_MARK:
				BUILD_BUG_ON(FIELD_SIZEOF(struct sk_buff, mark) != 4);
				if (is_imm8(offsetof(struct sk_buff, mark))) {
					/* mov off8(%rdi),%eax */
					EMIT3(0x8b, 0x47, offsetof(struct sk_buff, mark));
				} else {
					EMIT2(0x8b, 0x87);
					EMIT(offsetof(struct sk_buff, mark), 4);
				}
				break;
			case BPF_S_ANC_RXHASH:
				BUILD_BUG_ON(FIELD_SIZEOF(struct sk_buff, rxhash) != 4);
				if (is_imm8(offsetof(struct sk_buff, rxhash))) {
					/* mov off8(%rdi),%eax */
					EMIT3(0x8b, 0x47, offsetof(struct sk_buff, rxhash));
				} else {
					EMIT2(0x8b, 0x87);
					EMIT(offsetof(struct sk_buff, rxhash), 4);
				}
				break;
			case BPF_S_ANC_QUEUE:
				BUILD_BUG_ON(FIELD_SIZEOF(struct sk_buff, queue_mapping) != 2);
				if (is_imm8(offsetof(struct sk_buff, queue_mapping))) {
					/* movzwl off8(%rdi),%eax */
					EMIT4(0x0f, 0xb7, 0x47, offsetof(struct sk_buff, queue_mapping));
				} else {
					EMIT3(0x0f, 0xb7, 0x87); /* movzwl off32(%rdi),%eax */
					EMIT(offsetof(struct sk_buff, queue_mapping), 4);
				}
				break;
			case BPF_S_ANC_CPU:
#ifdef CONFIG_SMP
				EMIT4(0x65, 0x8b, 0x04, 0x25); /* mov %gs:off32,%eax */
				EMIT((u32)(unsigned long)&cpu_number, 4); /* A = smp_processor_id(); */
#else
				CLEAR_A();
#endif
				break;
			case BPF_S_ANC_VLAN_TAG:
			case BPF_S_ANC_VLAN_TAG_PRESENT:
				BUILD_BUG_ON(FIELD_SIZEOF(struct sk_buff, vlan_tci) != 2);
				if (is_imm8(offsetof(struct sk_buff, vlan_tci))) {
					/* movzwl off8(%rdi),%eax */
					EMIT4(0x0f, 0xb7, 0x47, offsetof(struct sk_buff, vlan_tci));
				} else {
					EMIT3(0x0f, 0xb7, 0x87); /* movzwl off32(%rdi),%eax */
					EMIT(offsetof(struct sk_buff, vlan_tci), 4);
				}
				BUILD_BUG_ON(VLAN_TAG_PRESENT != 0x1000);
				if (filter[i].code == BPF_S_ANC_VLAN_TAG) {
					EMIT3(0x80, 0xe4, 0xef); /* and    $0xef,%ah */
				} else {
					EMIT3(0xc1, 0xe8, 0x0c); /* shr    $0xc,%eax */
					EMIT3(0x83, 0xe0, 0x01); /* and    $0x1,%eax */
				}
				break;
			case BPF_S_ANC_PKTTYPE:
			{
				int off = pkt_type_offset();

				if (off < 0)
					goto out;
				if (is_imm8(off)) {
					/* movzbl off8(%rdi),%eax */
					EMIT4(0x0f, 0xb6, 0x47, off);
				} else {
					/* movbl off32(%rdi),%eax */
					EMIT3(0x0f, 0xb6, 0x87);
					EMIT(off, 4);
				}
				EMIT3(0x83, 0xe0, PKT_TYPE_MAX); /* and    $0x7,%eax */
				break;
			}
			case BPF_S_LD_W_ABS:
				func = CHOOSE_LOAD_FUNC(K, sk_load_word);
common_load:			seen |= SEEN_DATAREF;
				t_offset = func - (image + addrs[i]);
				EMIT1_off32(0xbe, K); /* mov imm32,%esi */
				EMIT1_off32(0xe8, t_offset); /* call */
				break;
			case BPF_S_LD_H_ABS:
				func = CHOOSE_LOAD_FUNC(K, sk_load_half);
				goto common_load;
			case BPF_S_LD_B_ABS:
				func = CHOOSE_LOAD_FUNC(K, sk_load_byte);
				goto common_load;
			case BPF_S_LDX_B_MSH:
				func = CHOOSE_LOAD_FUNC(K, sk_load_byte_msh);
				seen |= SEEN_DATAREF | SEEN_XREG;
				t_offset = func - (image + addrs[i]);
				EMIT1_off32(0xbe, K);	/* mov imm32,%esi */
				EMIT1_off32(0xe8, t_offset); /* call sk_load_byte_msh */
				break;
			case BPF_S_LD_W_IND:
				func = sk_load_word;
common_load_ind:		seen |= SEEN_DATAREF | SEEN_XREG;
				t_offset = func - (image + addrs[i]);
				if (K) {
					if (is_imm8(K)) {
						EMIT3(0x8d, 0x73, K); /* lea imm8(%rbx), %esi */
					} else {
						EMIT2(0x8d, 0xb3); /* lea imm32(%rbx),%esi */
						EMIT(K, 4);
					}
				} else {
					EMIT2(0x89,0xde); /* mov %ebx,%esi */
				}
				EMIT1_off32(0xe8, t_offset);	/* call sk_load_xxx_ind */
				break;
			case BPF_S_LD_H_IND:
				func = sk_load_half;
				goto common_load_ind;
			case BPF_S_LD_B_IND:
				func = sk_load_byte;
				goto common_load_ind;
			case BPF_S_JMP_JA:
				t_offset = addrs[i + K] - addrs[i];
				EMIT_JMP(t_offset);
				break;
			COND_SEL(BPF_S_JMP_JGT_K, X86_JA, X86_JBE);
			COND_SEL(BPF_S_JMP_JGE_K, X86_JAE, X86_JB);
			COND_SEL(BPF_S_JMP_JEQ_K, X86_JE, X86_JNE);
			COND_SEL(BPF_S_JMP_JSET_K,X86_JNE, X86_JE);
			COND_SEL(BPF_S_JMP_JGT_X, X86_JA, X86_JBE);
			COND_SEL(BPF_S_JMP_JGE_X, X86_JAE, X86_JB);
			COND_SEL(BPF_S_JMP_JEQ_X, X86_JE, X86_JNE);
			COND_SEL(BPF_S_JMP_JSET_X,X86_JNE, X86_JE);

cond_branch:			f_offset = addrs[i + filter[i].jf] - addrs[i];
				t_offset = addrs[i + filter[i].jt] - addrs[i];

				/* same targets, can avoid doing the test :) */
				if (filter[i].jt == filter[i].jf) {
					EMIT_JMP(t_offset);
					break;
				}

				switch (filter[i].code) {
				case BPF_S_JMP_JGT_X:
				case BPF_S_JMP_JGE_X:
				case BPF_S_JMP_JEQ_X:
					seen |= SEEN_XREG;
					EMIT2(0x39, 0xd8); /* cmp %ebx,%eax */
					break;
				case BPF_S_JMP_JSET_X:
					seen |= SEEN_XREG;
					EMIT2(0x85, 0xd8); /* test %ebx,%eax */
					break;
				case BPF_S_JMP_JEQ_K:
					if (K == 0) {
						EMIT2(0x85, 0xc0); /* test   %eax,%eax */
						break;
					}
				case BPF_S_JMP_JGT_K:
				case BPF_S_JMP_JGE_K:
					if (K <= 127)
						EMIT3(0x83, 0xf8, K); /* cmp imm8,%eax */
					else
						EMIT1_off32(0x3d, K); /* cmp imm32,%eax */
					break;
				case BPF_S_JMP_JSET_K:
					if (K <= 0xFF)
						EMIT2(0xa8, K); /* test imm8,%al */
					else if (!(K & 0xFFFF00FF))
						EMIT3(0xf6, 0xc4, K >> 8); /* test imm8,%ah */
					else if (K <= 0xFFFF) {
						EMIT2(0x66, 0xa9); /* test imm16,%ax */
						EMIT(K, 2);
					} else {
						EMIT1_off32(0xa9, K); /* test imm32,%eax */
					}
					break;
				}
				if (filter[i].jt != 0) {
					if (filter[i].jf && f_offset)
						t_offset += is_near(f_offset) ? 2 : 5;
					EMIT_COND_JMP(t_op, t_offset);
					if (filter[i].jf)
						EMIT_JMP(f_offset);
					break;
				}
				EMIT_COND_JMP(f_op, f_offset);
				break;
			default:
				/* hmm, too complex filter, give up with jit compiler */
>>>>>>> p9x
				goto out;
			}
			break;
		}
		if (proglen == oldproglen) {
			header = bpf_jit_binary_alloc(proglen, &image,
						      1, jit_fill_hole);
			if (!header)
				goto out;
		}
		oldproglen = proglen;
	}

	if (bpf_jit_enable > 1)
		bpf_jit_dump(prog->len, proglen, 0, image);

	if (image) {
		bpf_flush_icache(header, image + proglen);
		set_memory_ro((unsigned long)header, header->pages);
		prog->bpf_func = (void *)image;
		prog->jited = true;
	}
out:
	kfree(addrs);
}

void bpf_jit_free(struct bpf_prog *fp)
{
	unsigned long addr = (unsigned long)fp->bpf_func & PAGE_MASK;
	struct bpf_binary_header *header = (void *)addr;

	if (!fp->jited)
		goto free_filter;

	set_memory_rw(addr, header->pages);
	bpf_jit_binary_free(header);

free_filter:
	bpf_prog_unlock_free(fp);
}
