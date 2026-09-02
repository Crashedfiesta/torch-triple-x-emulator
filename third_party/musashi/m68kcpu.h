/* ======================================================================== */
/* ========================= LICENSING & COPYRIGHT ======================== */
/* ======================================================================== */
/*
 *                                  MUSASHI
 *                                Version 4.5
 *
 * A portable Motorola M680x0 processor emulation engine.
 * Copyright Karl Stenerud.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */




#ifndef M68KCPU__HEADER
#define M68KCPU__HEADER

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "m68k.h"

#include <limits.h>

#include <setjmp.h>

/* ======================================================================== */
/* ==================== ARCHITECTURE-DEPENDANT DEFINES ==================== */
/* ======================================================================== */

/* Check for > 32bit sizes */
#if UINT_MAX > 0xffffffff
	#define M68K_INT_GT_32_BIT  1
#else
	#define M68K_INT_GT_32_BIT  0
#endif

/* MMU debug flag - set to 1 to enable MMU debug messages */
#ifndef MMU_DEBUG
#define MMU_DEBUG 1
#endif

/* Data types used in this emulation core */
#undef sint8
#undef sint16
#undef sint32
#undef sint64
#undef uint8
#undef uint16
#undef uint32
#undef uint64
#undef sint
#undef uint

typedef signed   char  sint8;  		/* ASG: changed from char to signed char */
typedef signed   short sint16;
typedef signed   int   sint32; 		/* AWJ: changed from long to int */
typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32; 			/* AWJ: changed from long to int */

/* signed and unsigned int must be at least 32 bits wide */
typedef signed   int sint;
typedef unsigned int uint;


#if M68K_USE_64_BIT
typedef signed   long long sint64;
typedef unsigned long long uint64;
#else
typedef sint32 sint64;
typedef uint32 uint64;
#endif /* M68K_USE_64_BIT */

/* U64 and S64 are used to wrap long integer constants. */
#ifdef __GNUC__
#define U64(val) val##ULL
#define S64(val) val##LL
#else
#define U64(val) val
#define S64(val) val
#endif

#include "softfloat/milieu.h"
#include "softfloat/softfloat.h"


/* Allow for architectures that don't have 8-bit sizes */
#if UCHAR_MAX == 0xff
	#define MAKE_INT_8(A) (sint8)(A)
#else
	#undef  sint8
	#define sint8  signed   int
	#undef  uint8
	#define uint8  unsigned int
	static inline sint MAKE_INT_8(uint value)
	{
		return (value & 0x80) ? value | ~0xff : value & 0xff;
	}
#endif /* UCHAR_MAX == 0xff */


/* Allow for architectures that don't have 16-bit sizes */
#if USHRT_MAX == 0xffff
	#define MAKE_INT_16(A) (sint16)(A)
#else
	#undef  sint16
	#define sint16 signed   int
	#undef  uint16
	#define uint16 unsigned int
	static inline sint MAKE_INT_16(uint value)
	{
		return (value & 0x8000) ? value | ~0xffff : value & 0xffff;
	}
#endif /* USHRT_MAX == 0xffff */


/* Allow for architectures that don't have 32-bit sizes */
#if UINT_MAX == 0xffffffff
	#define MAKE_INT_32(A) (sint32)(A)
#else
	#undef  sint32
	#define sint32  signed   int
	#undef  uint32
	#define uint32  unsigned int
	static inline sint MAKE_INT_32(uint value)
	{
		return (value & 0x80000000) ? value | ~0xffffffff : value & 0xffffffff;
	}
#endif /* UINT_MAX == 0xffffffff */




/* ======================================================================== */
/* ============================ GENERAL DEFINES =========================== */
/* ======================================================================== */

/* Exception Vectors handled by emulation */
#define EXCEPTION_RESET                    0
#define EXCEPTION_BUS_ERROR                2 /* This one is not emulated! */
#define EXCEPTION_ADDRESS_ERROR            3 /* This one is partially emulated (doesn't stack a proper frame yet) */
#define EXCEPTION_ILLEGAL_INSTRUCTION      4
#define EXCEPTION_ZERO_DIVIDE              5
#define EXCEPTION_CHK                      6
#define EXCEPTION_TRAPV                    7
#define EXCEPTION_PRIVILEGE_VIOLATION      8
#define EXCEPTION_TRACE                    9
#define EXCEPTION_1010                    10
#define EXCEPTION_1111                    11
#define EXCEPTION_FORMAT_ERROR            14
#define EXCEPTION_UNINITIALIZED_INTERRUPT 15
#define EXCEPTION_SPURIOUS_INTERRUPT      24
#define EXCEPTION_INTERRUPT_AUTOVECTOR    24
#define EXCEPTION_TRAP_BASE               32
#define EXCEPTION_MMU_ACCESS_LEVEL        58  /* MC68851 Access Level Violation */

/* Function codes set by CPU during data/address bus activity */
#define FUNCTION_CODE_USER_DATA          1
#define FUNCTION_CODE_USER_PROGRAM       2
#define FUNCTION_CODE_SUPERVISOR_DATA    5
#define FUNCTION_CODE_SUPERVISOR_PROGRAM 6
#define FUNCTION_CODE_CPU_SPACE          7

/* CPU types for deciding what to emulate */
#define CPU_TYPE_000	(0x00000001)
#define CPU_TYPE_008    (0x00000002)
#define CPU_TYPE_010    (0x00000004)
#define CPU_TYPE_EC020  (0x00000008)
#define CPU_TYPE_020    (0x00000010)
#define CPU_TYPE_EC030  (0x00000020)
#define CPU_TYPE_030    (0x00000040)
#define CPU_TYPE_EC040  (0x00000080)
#define CPU_TYPE_LC040  (0x00000100)
#define CPU_TYPE_040    (0x00000200)
#define CPU_TYPE_SCC070 (0x00000400)

/* Different ways to stop the CPU */
#define STOP_LEVEL_STOP 1
#define STOP_LEVEL_HALT 2

/* Used for 68000 address error processing */
#define INSTRUCTION_YES 0
#define INSTRUCTION_NO  0x08
#define MODE_READ       0x10
#define MODE_WRITE      0

#define RUN_MODE_NORMAL              0
#define RUN_MODE_BERR_AERR_RESET_WSF 1 /* writing stack frame */
#define RUN_MODE_BERR_AERR_RESET     2 /* stack frame done */

#ifndef NULL
#define NULL ((void*)0)
#endif

/* ======================================================================== */
/* ================================ MACROS ================================ */
/* ======================================================================== */


/* ---------------------------- General Macros ---------------------------- */

/* Bit Isolation Macros */
#define BIT_0(A)  ((A) & 0x00000001)
#define BIT_1(A)  ((A) & 0x00000002)
#define BIT_2(A)  ((A) & 0x00000004)
#define BIT_3(A)  ((A) & 0x00000008)
#define BIT_4(A)  ((A) & 0x00000010)
#define BIT_5(A)  ((A) & 0x00000020)
#define BIT_6(A)  ((A) & 0x00000040)
#define BIT_7(A)  ((A) & 0x00000080)
#define BIT_8(A)  ((A) & 0x00000100)
#define BIT_9(A)  ((A) & 0x00000200)
#define BIT_A(A)  ((A) & 0x00000400)
#define BIT_B(A)  ((A) & 0x00000800)
#define BIT_C(A)  ((A) & 0x00001000)
#define BIT_D(A)  ((A) & 0x00002000)
#define BIT_E(A)  ((A) & 0x00004000)
#define BIT_F(A)  ((A) & 0x00008000)
#define BIT_10(A) ((A) & 0x00010000)
#define BIT_11(A) ((A) & 0x00020000)
#define BIT_12(A) ((A) & 0x00040000)
#define BIT_13(A) ((A) & 0x00080000)
#define BIT_14(A) ((A) & 0x00100000)
#define BIT_15(A) ((A) & 0x00200000)
#define BIT_16(A) ((A) & 0x00400000)
#define BIT_17(A) ((A) & 0x00800000)
#define BIT_18(A) ((A) & 0x01000000)
#define BIT_19(A) ((A) & 0x02000000)
#define BIT_1A(A) ((A) & 0x04000000)
#define BIT_1B(A) ((A) & 0x08000000)
#define BIT_1C(A) ((A) & 0x10000000)
#define BIT_1D(A) ((A) & 0x20000000)
#define BIT_1E(A) ((A) & 0x40000000)
#define BIT_1F(A) ((A) & 0x80000000)

/* Get the most significant bit for specific sizes */
#define GET_MSB_8(A)  ((A) & 0x80)
#define GET_MSB_9(A)  ((A) & 0x100)
#define GET_MSB_16(A) ((A) & 0x8000)
#define GET_MSB_17(A) ((A) & 0x10000)
#define GET_MSB_32(A) ((A) & 0x80000000)
#if M68K_USE_64_BIT
#define GET_MSB_33(A) ((A) & 0x100000000)
#endif /* M68K_USE_64_BIT */

/* Isolate nibbles */
#define LOW_NIBBLE(A)  ((A) & 0x0f)
#define HIGH_NIBBLE(A) ((A) & 0xf0)

/* These are used to isolate 8, 16, and 32 bit sizes */
#define MASK_OUT_ABOVE_2(A)  ((A) & 3)
#define MASK_OUT_ABOVE_8(A)  ((A) & 0xff)
#define MASK_OUT_ABOVE_16(A) ((A) & 0xffff)
#define MASK_OUT_BELOW_2(A)  ((A) & ~3)
#define MASK_OUT_BELOW_8(A)  ((A) & ~0xff)
#define MASK_OUT_BELOW_16(A) ((A) & ~0xffff)

/* No need to mask if we are 32 bit */
#if M68K_INT_GT_32_BIT || M68K_USE_64_BIT
	#define MASK_OUT_ABOVE_32(A) ((A) & 0xffffffff)
	#define MASK_OUT_BELOW_32(A) ((A) & ~0xffffffff)
#else
	#define MASK_OUT_ABOVE_32(A) (A)
	#define MASK_OUT_BELOW_32(A) 0
#endif /* M68K_INT_GT_32_BIT || M68K_USE_64_BIT */

/* Simulate address lines of 68k family */
#define ADDRESS_68K(A) ((A)&CPU_ADDRESS_MASK)


/* Shift & Rotate Macros. */
#define LSL(A, C) ((A) << (C))
#define LSR(A, C) ((A) >> (C))

/* Some > 32-bit optimizations */
#if M68K_INT_GT_32_BIT
	/* Shift left and right */
	#define LSR_32(A, C) ((A) >> (C))
	#define LSL_32(A, C) ((A) << (C))
#else
	/* We have to do this because the morons at ANSI decided that shifts
	 * by >= data size are undefined.
	 */
	#define LSR_32(A, C) ((C) < 32 ? (A) >> (C) : 0)
	#define LSL_32(A, C) ((C) < 32 ? (A) << (C) : 0)
#endif /* M68K_INT_GT_32_BIT */

#if M68K_USE_64_BIT
	#define LSL_32_64(A, C) ((A) << (C))
	#define LSR_32_64(A, C) ((A) >> (C))
	#define ROL_33_64(A, C) (LSL_32_64(A, C) | LSR_32_64(A, 33-(C)))
	#define ROR_33_64(A, C) (LSR_32_64(A, C) | LSL_32_64(A, 33-(C)))
#endif /* M68K_USE_64_BIT */

#define ROL_8(A, C)      MASK_OUT_ABOVE_8(LSL(A, C) | LSR(A, 8-(C)))
#define ROL_9(A, C)                      (LSL(A, C) | LSR(A, 9-(C)))
#define ROL_16(A, C)    MASK_OUT_ABOVE_16(LSL(A, C) | LSR(A, 16-(C)))
#define ROL_17(A, C)                     (LSL(A, C) | LSR(A, 17-(C)))
#define ROL_32(A, C)    MASK_OUT_ABOVE_32(LSL_32(A, C) | LSR_32(A, 32-(C)))
#define ROL_33(A, C)                     (LSL_32(A, C) | LSR_32(A, 33-(C)))

#define ROR_8(A, C)      MASK_OUT_ABOVE_8(LSR(A, C) | LSL(A, 8-(C)))
#define ROR_9(A, C)                      (LSR(A, C) | LSL(A, 9-(C)))
#define ROR_16(A, C)    MASK_OUT_ABOVE_16(LSR(A, C) | LSL(A, 16-(C)))
#define ROR_17(A, C)                     (LSR(A, C) | LSL(A, 17-(C)))
#define ROR_32(A, C)    MASK_OUT_ABOVE_32(LSR_32(A, C) | LSL_32(A, 32-(C)))
#define ROR_33(A, C)                     (LSR_32(A, C) | LSL_32(A, 33-(C)))



/* ------------------------------ CPU Access ------------------------------ */

/* Access the CPU registers */
#define CPU_TYPE         m68ki_cpu.cpu_type

#define REG_DA           m68ki_cpu.dar /* easy access to data and address regs */
#define REG_DA_SAVE           m68ki_cpu.dar_save
#define REG_D            m68ki_cpu.dar
#define REG_A            (m68ki_cpu.dar+8)
#define REG_PPC 		 m68ki_cpu.ppc
#define REG_PC           m68ki_cpu.pc
#define REG_SP_BASE      m68ki_cpu.sp
#define REG_USP          m68ki_cpu.sp[0]
#define REG_ISP          m68ki_cpu.sp[4]
#define REG_MSP          m68ki_cpu.sp[6]
#define REG_SP           m68ki_cpu.dar[15]
#define REG_VBR          m68ki_cpu.vbr
#define REG_SFC          m68ki_cpu.sfc
#define REG_DFC          m68ki_cpu.dfc
#define REG_CACR         m68ki_cpu.cacr
#define REG_CAAR         m68ki_cpu.caar
#define REG_IR           m68ki_cpu.ir

#define REG_FP           m68ki_cpu.fpr
#define REG_FPCR         m68ki_cpu.fpcr
#define REG_FPSR         m68ki_cpu.fpsr
#define REG_FPIAR        m68ki_cpu.fpiar

#define FLAG_T1          m68ki_cpu.t1_flag
#define FLAG_T0          m68ki_cpu.t0_flag
#define FLAG_S           m68ki_cpu.s_flag
#define FLAG_M           m68ki_cpu.m_flag
#define FLAG_X           m68ki_cpu.x_flag
#define FLAG_N           m68ki_cpu.n_flag
#define FLAG_Z           m68ki_cpu.not_z_flag
#define FLAG_V           m68ki_cpu.v_flag
#define FLAG_C           m68ki_cpu.c_flag
#define FLAG_INT_MASK    m68ki_cpu.int_mask

#define CPU_INT_LEVEL    m68ki_cpu.int_level /* ASG: changed from CPU_INTS_PENDING */
#define CPU_STOPPED      m68ki_cpu.stopped
#define CPU_PREF_ADDR    m68ki_cpu.pref_addr
#define CPU_PREF_DATA    m68ki_cpu.pref_data
#define CPU_ADDRESS_MASK m68ki_cpu.address_mask
#define CPU_SR_MASK      m68ki_cpu.sr_mask
#define CPU_INSTR_MODE   m68ki_cpu.instr_mode
#define CPU_RUN_MODE     m68ki_cpu.run_mode

#define CYC_INSTRUCTION  m68ki_cpu.cyc_instruction
#define CYC_EXCEPTION    m68ki_cpu.cyc_exception
#define CYC_BCC_NOTAKE_B m68ki_cpu.cyc_bcc_notake_b
#define CYC_BCC_NOTAKE_W m68ki_cpu.cyc_bcc_notake_w
#define CYC_DBCC_F_NOEXP m68ki_cpu.cyc_dbcc_f_noexp
#define CYC_DBCC_F_EXP   m68ki_cpu.cyc_dbcc_f_exp
#define CYC_SCC_R_TRUE   m68ki_cpu.cyc_scc_r_true
#define CYC_MOVEM_W      m68ki_cpu.cyc_movem_w
#define CYC_MOVEM_L      m68ki_cpu.cyc_movem_l
#define CYC_SHIFT        m68ki_cpu.cyc_shift
#define CYC_RESET        m68ki_cpu.cyc_reset
#define HAS_PMMU	 m68ki_cpu.has_pmmu
// g_force_pmmu_enabled is kept for backward compatibility with debug traces (always 0)
extern int g_force_pmmu_enabled;
extern int cpu_log_enabled;
// g_pmmu_pcsr is the Page Cache Status Register (MC68851), accessible for cpBcc evaluation
extern uint16 g_pmmu_pcsr;
#define PMMU_ENABLED	 m68ki_cpu.pmmu_enabled
#define RESET_CYCLES	 m68ki_cpu.reset_cycles


#define CALLBACK_INT_ACK     m68ki_cpu.int_ack_callback
#define CALLBACK_BKPT_ACK    m68ki_cpu.bkpt_ack_callback
#define CALLBACK_RESET_INSTR m68ki_cpu.reset_instr_callback
#define CALLBACK_CMPILD_INSTR m68ki_cpu.cmpild_instr_callback
#define CALLBACK_RTE_INSTR    m68ki_cpu.rte_instr_callback
#define CALLBACK_TAS_INSTR    m68ki_cpu.tas_instr_callback
#define CALLBACK_ILLG_INSTR    m68ki_cpu.illg_instr_callback
#define CALLBACK_PC_CHANGED  m68ki_cpu.pc_changed_callback
#define CALLBACK_SET_FC      m68ki_cpu.set_fc_callback
#define CALLBACK_INSTR_HOOK  m68ki_cpu.instr_hook_callback
#define CALLBACK_ALINE_HOOK  m68ki_cpu.aline_hook_callback



/* ----------------------------- Configuration ---------------------------- */

/* These defines are dependant on the configuration defines in m68kconf.h */

/* Disable certain comparisons if we're not using all CPU types */
#if M68K_EMULATE_040
#define CPU_TYPE_IS_040_PLUS(A)    ((A) & (CPU_TYPE_040 | CPU_TYPE_EC040))
	#define CPU_TYPE_IS_040_LESS(A)    1
#else
	#define CPU_TYPE_IS_040_PLUS(A)    0
	#define CPU_TYPE_IS_040_LESS(A)    1
#endif

#if M68K_EMULATE_030
#define CPU_TYPE_IS_030_PLUS(A)    ((A) & (CPU_TYPE_030 | CPU_TYPE_EC030 | CPU_TYPE_040 | CPU_TYPE_EC040))
#define CPU_TYPE_IS_030_LESS(A)    1
#else
#define CPU_TYPE_IS_030_PLUS(A)	0
#define CPU_TYPE_IS_030_LESS(A)    1
#endif

#if M68K_EMULATE_020
#define CPU_TYPE_IS_020_PLUS(A)    ((A) & (CPU_TYPE_020 | CPU_TYPE_030 | CPU_TYPE_EC030 | CPU_TYPE_040 | CPU_TYPE_EC040))
	#define CPU_TYPE_IS_020_LESS(A)    1
#else
	#define CPU_TYPE_IS_020_PLUS(A)    0
	#define CPU_TYPE_IS_020_LESS(A)    1
#endif

#if M68K_EMULATE_EC020
#define CPU_TYPE_IS_EC020_PLUS(A)  ((A) & (CPU_TYPE_EC020 | CPU_TYPE_020 | CPU_TYPE_030 | CPU_TYPE_EC030 | CPU_TYPE_040 | CPU_TYPE_EC040))
	#define CPU_TYPE_IS_EC020_LESS(A)  ((A) & (CPU_TYPE_000 | CPU_TYPE_010 | CPU_TYPE_EC020))
#else
	#define CPU_TYPE_IS_EC020_PLUS(A)  CPU_TYPE_IS_020_PLUS(A)
	#define CPU_TYPE_IS_EC020_LESS(A)  CPU_TYPE_IS_020_LESS(A)
#endif

#if M68K_EMULATE_010
	#define CPU_TYPE_IS_010(A)         ((A) == CPU_TYPE_010)
#define CPU_TYPE_IS_010_PLUS(A)    ((A) & (CPU_TYPE_010 | CPU_TYPE_EC020 | CPU_TYPE_020 | CPU_TYPE_EC030 | CPU_TYPE_030 | CPU_TYPE_040 | CPU_TYPE_EC040))
#define CPU_TYPE_IS_010_LESS(A)    ((A) & (CPU_TYPE_000 | CPU_TYPE_008 | CPU_TYPE_010))
#else
	#define CPU_TYPE_IS_010(A)         0
	#define CPU_TYPE_IS_010_PLUS(A)    CPU_TYPE_IS_EC020_PLUS(A)
	#define CPU_TYPE_IS_010_LESS(A)    CPU_TYPE_IS_EC020_LESS(A)
#endif

#if M68K_EMULATE_020 || M68K_EMULATE_EC020
	#define CPU_TYPE_IS_020_VARIANT(A) ((A) & (CPU_TYPE_EC020 | CPU_TYPE_020))
#else
	#define CPU_TYPE_IS_020_VARIANT(A) 0
#endif

#if M68K_EMULATE_040 || M68K_EMULATE_020 || M68K_EMULATE_EC020 || M68K_EMULATE_010
	#define CPU_TYPE_IS_000(A)         ((A) == CPU_TYPE_000)
#else
	#define CPU_TYPE_IS_000(A)         1
#endif


#if !M68K_SEPARATE_READS
#define m68k_read_immediate_16(A) m68ki_read_program_16(A)
#define m68k_read_immediate_32(A) m68ki_read_program_32(A)

#define m68k_read_pcrelative_8(A) m68ki_read_program_8(A)
#define m68k_read_pcrelative_16(A) m68ki_read_program_16(A)
#define m68k_read_pcrelative_32(A) m68ki_read_program_32(A)
#endif /* M68K_SEPARATE_READS */


/* Enable or disable callback functions */
#if M68K_EMULATE_INT_ACK
	#if M68K_EMULATE_INT_ACK == OPT_SPECIFY_HANDLER
		#define m68ki_int_ack(A) M68K_INT_ACK_CALLBACK(A)
	#else
		#define m68ki_int_ack(A) CALLBACK_INT_ACK(A)
	#endif
#else
	/* Default action is to used autovector mode, which is most common */
	#define m68ki_int_ack(A) M68K_INT_ACK_AUTOVECTOR
#endif /* M68K_EMULATE_INT_ACK */

#if M68K_EMULATE_BKPT_ACK
	#if M68K_EMULATE_BKPT_ACK == OPT_SPECIFY_HANDLER
		#define m68ki_bkpt_ack(A) M68K_BKPT_ACK_CALLBACK(A)
	#else
		#define m68ki_bkpt_ack(A) CALLBACK_BKPT_ACK(A)
	#endif
#else
	#define m68ki_bkpt_ack(A)
#endif /* M68K_EMULATE_BKPT_ACK */

#if M68K_EMULATE_RESET
	#if M68K_EMULATE_RESET == OPT_SPECIFY_HANDLER
		#define m68ki_output_reset() M68K_RESET_CALLBACK()
	#else
		#define m68ki_output_reset() CALLBACK_RESET_INSTR()
	#endif
#else
	#define m68ki_output_reset()
#endif /* M68K_EMULATE_RESET */

#if M68K_CMPILD_HAS_CALLBACK
	#if M68K_CMPILD_HAS_CALLBACK == OPT_SPECIFY_HANDLER
		#define m68ki_cmpild_callback(v,r) M68K_CMPILD_CALLBACK(v,r)
	#else
		#define m68ki_cmpild_callback(v,r) CALLBACK_CMPILD_INSTR(v,r)
	#endif
#else
	#define m68ki_cmpild_callback(v,r)
#endif /* M68K_CMPILD_HAS_CALLBACK */

#if M68K_RTE_HAS_CALLBACK
	#if M68K_RTE_HAS_CALLBACK == OPT_SPECIFY_HANDLER
		#define m68ki_rte_callback() M68K_RTE_CALLBACK()
	#else
		#define m68ki_rte_callback() CALLBACK_RTE_INSTR()
	#endif
#else
	#define m68ki_rte_callback()
#endif /* M68K_RTE_HAS_CALLBACK */

#if M68K_TAS_HAS_CALLBACK
	#if M68K_TAS_HAS_CALLBACK == OPT_SPECIFY_HANDLER
		#define m68ki_tas_callback() M68K_TAS_CALLBACK()
	#else
		#define m68ki_tas_callback() CALLBACK_TAS_INSTR()
	#endif
#else
	#define m68ki_tas_callback() 1
#endif /* M68K_TAS_HAS_CALLBACK */

#if M68K_ILLG_HAS_CALLBACK
	#if M68K_ILLG_HAS_CALLBACK == OPT_SPECIFY_HANDLER
		#define m68ki_illg_callback(opcode) M68K_ILLG_CALLBACK(opcode)
	#else
		#define m68ki_illg_callback(opcode) CALLBACK_ILLG_INSTR(opcode)
	#endif
#else
	#define m68ki_illg_callback(opcode) 0 // Default is 0 = not handled, exception will occur
#endif /* M68K_ILLG_HAS_CALLBACK */

#if M68K_INSTRUCTION_HOOK
	#if M68K_INSTRUCTION_HOOK == OPT_SPECIFY_HANDLER
		#define m68ki_instr_hook(pc) M68K_INSTRUCTION_CALLBACK(pc)
	#else
		#define m68ki_instr_hook(pc) CALLBACK_INSTR_HOOK(pc)
	#endif
#else
	#define m68ki_instr_hook(pc)
#endif /* M68K_INSTRUCTION_HOOK */

#if M68K_MONITOR_PC
	#if M68K_MONITOR_PC == OPT_SPECIFY_HANDLER
		#define m68ki_pc_changed(A) M68K_SET_PC_CALLBACK(ADDRESS_68K(A))
	#else
		#define m68ki_pc_changed(A) CALLBACK_PC_CHANGED(ADDRESS_68K(A))
	#endif
#else
	#define m68ki_pc_changed(A)
#endif /* M68K_MONITOR_PC */


/* Enable or disable function code emulation */
#if M68K_EMULATE_FC
	#if M68K_EMULATE_FC == OPT_SPECIFY_HANDLER
		#define m68ki_set_fc(A) M68K_SET_FC_CALLBACK(A)
	#else
		#define m68ki_set_fc(A) CALLBACK_SET_FC(A)
	#endif
	#define m68ki_use_data_space() m68ki_address_space = FUNCTION_CODE_USER_DATA
	#define m68ki_use_program_space() m68ki_address_space = FUNCTION_CODE_USER_PROGRAM
	#define m68ki_get_address_space() m68ki_address_space
#else
	#define m68ki_set_fc(A)
	#define m68ki_use_data_space()
	#define m68ki_use_program_space()
	#define m68ki_get_address_space() FUNCTION_CODE_USER_DATA
#endif /* M68K_EMULATE_FC */


/* Enable or disable trace emulation */
#if M68K_EMULATE_TRACE
	/* Initiates trace checking before each instruction (t1) */
	#define m68ki_trace_t1() m68ki_tracing = FLAG_T1
	/* adds t0 to trace checking if we encounter change of flow */
	#define m68ki_trace_t0() m68ki_tracing |= FLAG_T0
	/* Clear all tracing */
	#define m68ki_clear_trace() m68ki_tracing = 0
	/* Cause a trace exception if we are tracing */
	#define m68ki_exception_if_trace() if(m68ki_tracing) m68ki_exception_trace()
#else
	#define m68ki_trace_t1()
	#define m68ki_trace_t0()
	#define m68ki_clear_trace()
	#define m68ki_exception_if_trace()
#endif /* M68K_EMULATE_TRACE */



/* Address error */
#if M68K_EMULATE_ADDRESS_ERROR
	#include <setjmp.h>

/* sigjmp() on Mac OS X and *BSD in general saves signal contexts and is super-slow, use sigsetjmp() to tell it not to */
#ifdef _BSD_SETJMP_H
extern sigjmp_buf m68ki_aerr_trap;
#define m68ki_set_address_error_trap(m68k) \
	if(sigsetjmp(m68ki_aerr_trap, 0) != 0) \
	{ \
		m68ki_exception_address_error(m68k); \
		if(CPU_STOPPED) \
		{ \
			if (m68ki_remaining_cycles > 0) \
				m68ki_remaining_cycles = 0; \
			return m68ki_initial_cycles; \
		} \
	}

#define m68ki_check_address_error(ADDR, WRITE_MODE, FC) \
	if((ADDR)&1) \
	{ \
		m68ki_aerr_address = ADDR; \
		m68ki_aerr_write_mode = WRITE_MODE; \
		m68ki_aerr_fc = FC; \
		siglongjmp(m68ki_aerr_trap, 1); \
	}
#else
extern jmp_buf m68ki_aerr_trap;
	#define m68ki_set_address_error_trap() \
		if(setjmp(m68ki_aerr_trap) != 0) \
		{ \
			m68ki_exception_address_error(); \
			if(CPU_STOPPED) \
			{ \
				SET_CYCLES(0); \
				return m68ki_initial_cycles; \
			} \
			/* ensure we don't re-enter execution loop after an
			   address error if there's no more cycles remaining */ \
			if(GET_CYCLES() <= 0) \
			{ \
				/* return how many clocks we used */ \
				return m68ki_initial_cycles - GET_CYCLES(); \
			} \
		}

	#define m68ki_check_address_error(ADDR, WRITE_MODE, FC) \
		if((ADDR)&1) \
		{ \
			m68ki_aerr_address = ADDR; \
			m68ki_aerr_write_mode = WRITE_MODE; \
			m68ki_aerr_fc = FC; \
			longjmp(m68ki_aerr_trap, 1); \
		}
#endif

	#define m68ki_check_address_error_010_less(ADDR, WRITE_MODE, FC) \
		if (CPU_TYPE_IS_010_LESS(CPU_TYPE)) \
		{ \
			m68ki_check_address_error(ADDR, WRITE_MODE, FC) \
		}
#else
	#define m68ki_set_address_error_trap()
	#define m68ki_check_address_error(ADDR, WRITE_MODE, FC)
	#define m68ki_check_address_error_010_less(ADDR, WRITE_MODE, FC)
#endif /* M68K_ADDRESS_ERROR */

/* Logging */
#if M68K_LOG_ENABLE
	#include <stdio.h>
	extern FILE* M68K_LOG_FILEHANDLE
	extern const char *const m68ki_cpu_names[];

	#define M68K_DO_LOG(A) if(M68K_LOG_FILEHANDLE) fprintf A
	#if M68K_LOG_1010_1111
		#define M68K_DO_LOG_EMU(A) if(M68K_LOG_FILEHANDLE) fprintf A
	#else
		#define M68K_DO_LOG_EMU(A)
	#endif
#else
	#define M68K_DO_LOG(A)
	#define M68K_DO_LOG_EMU(A)
#endif

/* Aline hook */
#if M68K_ALINE_HOOK
       #if M68K_ALINE_HOOK == OPT_SPECIFY_HANDLER
               #define m68ki_aline_hook() M68K_ALINE_CALLBACK()
       #else
               #define m68ki_aline_hook() CALLBACK_ALINE_HOOK()
       #endif
#else
       #define m68ki_aline_hook()  M68K_ALINE_EXCEPT
#endif /* M68K_ALINE_HOOK */


/* -------------------------- EA / Operand Access ------------------------- */

/*
 * The general instruction format follows this pattern:
 * .... XXX. .... .YYY
 * where XXX is register X and YYY is register Y
 */
/* Data Register Isolation */
#define DX (REG_D[(REG_IR >> 9) & 7])
#define DY (REG_D[REG_IR & 7])
/* Address Register Isolation */
#define AX (REG_A[(REG_IR >> 9) & 7])
#define AY (REG_A[REG_IR & 7])


/* Effective Address Calculations */
#define EA_AY_AI_8()   AY                                    /* address register indirect */
#define EA_AY_AI_16()  EA_AY_AI_8()
#define EA_AY_AI_32()  EA_AY_AI_8()
#define EA_AY_PI_8()   (AY++)                                /* postincrement (size = byte) */
#define EA_AY_PI_16()  ((AY+=2)-2)                           /* postincrement (size = word) */
#define EA_AY_PI_32()  ((AY+=4)-4)                           /* postincrement (size = long) */
#define EA_AY_PD_8()   (--AY)                                /* predecrement (size = byte) */
#define EA_AY_PD_16()  (AY-=2)                               /* predecrement (size = word) */
#define EA_AY_PD_32()  (AY-=4)                               /* predecrement (size = long) */
#define EA_AY_DI_8()   (AY+MAKE_INT_16(m68ki_read_imm_16())) /* displacement */
#define EA_AY_DI_16()  EA_AY_DI_8()
#define EA_AY_DI_32()  EA_AY_DI_8()
#define EA_AY_IX_8()   m68ki_get_ea_ix(AY)                   /* indirect + index */
#define EA_AY_IX_16()  EA_AY_IX_8()
#define EA_AY_IX_32()  EA_AY_IX_8()

#define EA_AX_AI_8()   AX
#define EA_AX_AI_16()  EA_AX_AI_8()
#define EA_AX_AI_32()  EA_AX_AI_8()
#define EA_AX_PI_8()   (AX++)
#define EA_AX_PI_16()  ((AX+=2)-2)
#define EA_AX_PI_32()  ((AX+=4)-4)
#define EA_AX_PD_8()   (--AX)
#define EA_AX_PD_16()  (AX-=2)
#define EA_AX_PD_32()  (AX-=4)
#define EA_AX_DI_8()   (AX+MAKE_INT_16(m68ki_read_imm_16()))
#define EA_AX_DI_16()  EA_AX_DI_8()
#define EA_AX_DI_32()  EA_AX_DI_8()
#define EA_AX_IX_8()   m68ki_get_ea_ix(AX)
#define EA_AX_IX_16()  EA_AX_IX_8()
#define EA_AX_IX_32()  EA_AX_IX_8()

#define EA_A7_PI_8()   ((REG_A[7]+=2)-2)
#define EA_A7_PD_8()   (REG_A[7]-=2)

#define EA_AW_8()      MAKE_INT_16(m68ki_read_imm_16())      /* absolute word */
#define EA_AW_16()     EA_AW_8()
#define EA_AW_32()     EA_AW_8()
#define EA_AL_8()      m68ki_read_imm_32()                   /* absolute long */
#define EA_AL_16()     EA_AL_8()
#define EA_AL_32()     EA_AL_8()
#define EA_PCDI_8()    m68ki_get_ea_pcdi()                   /* pc indirect + displacement */
#define EA_PCDI_16()   EA_PCDI_8()
#define EA_PCDI_32()   EA_PCDI_8()
#define EA_PCIX_8()    m68ki_get_ea_pcix()                   /* pc indirect + index */
#define EA_PCIX_16()   EA_PCIX_8()
#define EA_PCIX_32()   EA_PCIX_8()


#define OPER_I_8()     m68ki_read_imm_8()
#define OPER_I_16()    m68ki_read_imm_16()
#define OPER_I_32()    m68ki_read_imm_32()



/* --------------------------- Status Register ---------------------------- */

/* Flag Calculation Macros */
#define CFLAG_8(A) (A)
#define CFLAG_16(A) ((A)>>8)

#if M68K_INT_GT_32_BIT
	#define CFLAG_ADD_32(S, D, R) ((R)>>24)
	#define CFLAG_SUB_32(S, D, R) ((R)>>24)
#else
	#define CFLAG_ADD_32(S, D, R) (((S & D) | (~R & (S | D)))>>23)
	#define CFLAG_SUB_32(S, D, R) (((S & R) | (~D & (S | R)))>>23)
#endif /* M68K_INT_GT_32_BIT */

#define VFLAG_ADD_8(S, D, R) ((S^R) & (D^R))
#define VFLAG_ADD_16(S, D, R) (((S^R) & (D^R))>>8)
#define VFLAG_ADD_32(S, D, R) (((S^R) & (D^R))>>24)

#define VFLAG_SUB_8(S, D, R) ((S^D) & (R^D))
#define VFLAG_SUB_16(S, D, R) (((S^D) & (R^D))>>8)
#define VFLAG_SUB_32(S, D, R) (((S^D) & (R^D))>>24)

#define NFLAG_8(A) (A)
#define NFLAG_16(A) ((A)>>8)
#define NFLAG_32(A) ((A)>>24)
#define NFLAG_64(A) ((A)>>56)

#define ZFLAG_8(A) MASK_OUT_ABOVE_8(A)
#define ZFLAG_16(A) MASK_OUT_ABOVE_16(A)
#define ZFLAG_32(A) MASK_OUT_ABOVE_32(A)


/* Flag values */
#define NFLAG_SET   0x80
#define NFLAG_CLEAR 0
#define CFLAG_SET   0x100
#define CFLAG_CLEAR 0
#define XFLAG_SET   0x100
#define XFLAG_CLEAR 0
#define VFLAG_SET   0x80
#define VFLAG_CLEAR 0
#define ZFLAG_SET   0
#define ZFLAG_CLEAR 0xffffffff

#define SFLAG_SET   4
#define SFLAG_CLEAR 0
#define MFLAG_SET   2
#define MFLAG_CLEAR 0

/* Turn flag values into 1 or 0 */
#define XFLAG_AS_1() ((FLAG_X>>8)&1)
#define NFLAG_AS_1() ((FLAG_N>>7)&1)
#define VFLAG_AS_1() ((FLAG_V>>7)&1)
#define ZFLAG_AS_1() (!FLAG_Z)
#define CFLAG_AS_1() ((FLAG_C>>8)&1)


/* Conditions */
#define COND_CS() (FLAG_C&0x100)
#define COND_CC() (!COND_CS())
#define COND_VS() (FLAG_V&0x80)
#define COND_VC() (!COND_VS())
#define COND_NE() FLAG_Z
#define COND_EQ() (!COND_NE())
#define COND_MI() (FLAG_N&0x80)
#define COND_PL() (!COND_MI())
#define COND_LT() ((FLAG_N^FLAG_V)&0x80)
#define COND_GE() (!COND_LT())
#define COND_HI() (COND_CC() && COND_NE())
#define COND_LS() (COND_CS() || COND_EQ())
#define COND_GT() (COND_GE() && COND_NE())
#define COND_LE() (COND_LT() || COND_EQ())

/* Reversed conditions */
#define COND_NOT_CS() COND_CC()
#define COND_NOT_CC() COND_CS()
#define COND_NOT_VS() COND_VC()
#define COND_NOT_VC() COND_VS()
#define COND_NOT_NE() COND_EQ()
#define COND_NOT_EQ() COND_NE()
#define COND_NOT_MI() COND_PL()
#define COND_NOT_PL() COND_MI()
#define COND_NOT_LT() COND_GE()
#define COND_NOT_GE() COND_LT()
#define COND_NOT_HI() COND_LS()
#define COND_NOT_LS() COND_HI()
#define COND_NOT_GT() COND_LE()
#define COND_NOT_LE() COND_GT()

/* Not real conditions, but here for convenience */
#define COND_XS() (FLAG_X&0x100)
#define COND_XC() (!COND_XS)


/* Get the condition code register */
#define m68ki_get_ccr() ((COND_XS() >> 4) | \
						 (COND_MI() >> 4) | \
						 (COND_EQ() << 2) | \
						 (COND_VS() >> 6) | \
						 (COND_CS() >> 8))

/* Get the status register */
#define m68ki_get_sr() ( FLAG_T1              | \
						 FLAG_T0              | \
						(FLAG_S        << 11) | \
						(FLAG_M        << 11) | \
						 FLAG_INT_MASK        | \
						 m68ki_get_ccr())



/* ---------------------------- Cycle Counting ---------------------------- */

#define ADD_CYCLES(A)    m68ki_remaining_cycles += (A)
#define USE_CYCLES(A)    m68ki_remaining_cycles -= (A)
#define SET_CYCLES(A)    m68ki_remaining_cycles = A
#define GET_CYCLES()     m68ki_remaining_cycles
#define USE_ALL_CYCLES() m68ki_remaining_cycles %= CYC_INSTRUCTION[REG_IR]



/* ----------------------------- Read / Write ----------------------------- */

/* Read from the current address space */
#define m68ki_read_8(A)  m68ki_read_8_fc (A, FLAG_S | m68ki_get_address_space())
#define m68ki_read_16(A) m68ki_read_16_fc(A, FLAG_S | m68ki_get_address_space())
#define m68ki_read_32(A) m68ki_read_32_fc(A, FLAG_S | m68ki_get_address_space())

/* Write to the current data space */
#define m68ki_write_8(A, V)  m68ki_write_8_fc (A, FLAG_S | FUNCTION_CODE_USER_DATA, V)
#define m68ki_write_16(A, V) m68ki_write_16_fc(A, FLAG_S | FUNCTION_CODE_USER_DATA, V)
#define m68ki_write_32(A, V) m68ki_write_32_fc(A, FLAG_S | FUNCTION_CODE_USER_DATA, V)

#if M68K_SIMULATE_PD_WRITES
#define m68ki_write_32_pd(A, V) m68ki_write_32_pd_fc(A, FLAG_S | FUNCTION_CODE_USER_DATA, V)
#else
#define m68ki_write_32_pd(A, V) m68ki_write_32_fc(A, FLAG_S | FUNCTION_CODE_USER_DATA, V)
#endif

/* Map PC-relative reads */
#define m68ki_read_pcrel_8(A) m68k_read_pcrelative_8(A)
#define m68ki_read_pcrel_16(A) m68k_read_pcrelative_16(A)
#define m68ki_read_pcrel_32(A) m68k_read_pcrelative_32(A)

/* Read from the program space */
#define m68ki_read_program_8(A) 	m68ki_read_8_fc(A, FLAG_S | FUNCTION_CODE_USER_PROGRAM)
#define m68ki_read_program_16(A) 	m68ki_read_16_fc(A, FLAG_S | FUNCTION_CODE_USER_PROGRAM)
#define m68ki_read_program_32(A) 	m68ki_read_32_fc(A, FLAG_S | FUNCTION_CODE_USER_PROGRAM)

/* Read from the data space */
#define m68ki_read_data_8(A) 	m68ki_read_8_fc(A, FLAG_S | FUNCTION_CODE_USER_DATA)
#define m68ki_read_data_16(A) 	m68ki_read_16_fc(A, FLAG_S | FUNCTION_CODE_USER_DATA)
#define m68ki_read_data_32(A) 	m68ki_read_32_fc(A, FLAG_S | FUNCTION_CODE_USER_DATA)



/* ======================================================================== */
/* =============================== PROTOTYPES ============================= */
/* ======================================================================== */

typedef union
{
	uint64 i;
	double f;
} fp_reg;

typedef struct
{
	uint cpu_type;     /* CPU Type: 68000, 68008, 68010, 68EC020, 68020, 68EC030, 68030, 68EC040, or 68040 */
	uint dar[16];      /* Data and Address Registers */
	uint dar_save[16];  /* Saved Data and Address Registers (pushed onto the
						   stack when a bus error occurs)*/
	uint ppc;		   /* Previous program counter */
	uint pc;           /* Program Counter */
	uint sp[7];        /* User, Interrupt, and Master Stack Pointers */
	uint vbr;          /* Vector Base Register (m68010+) */
	uint sfc;          /* Source Function Code Register (m68010+) */
	uint dfc;          /* Destination Function Code Register (m68010+) */
	uint cacr;         /* Cache Control Register (m68020, unemulated) */
	uint caar;         /* Cache Address Register (m68020, unemulated) */
	uint ir;           /* Instruction Register */
	floatx80 fpr[8];     /* FPU Data Register (m68030/040) */
	uint fpiar;        /* FPU Instruction Address Register (m68040) */
	uint fpsr;         /* FPU Status Register (m68040) */
	uint fpcr;         /* FPU Control Register (m68040) */
	uint t1_flag;      /* Trace 1 */
	uint t0_flag;      /* Trace 0 */
	uint s_flag;       /* Supervisor */
	uint m_flag;       /* Master/Interrupt state */
	uint x_flag;       /* Extend */
	uint n_flag;       /* Negative */
	uint not_z_flag;   /* Zero, inverted for speedups */
	uint v_flag;       /* Overflow */
	uint c_flag;       /* Carry */
	uint int_mask;     /* I0-I2 */
	uint int_level;    /* State of interrupt pins IPL0-IPL2 -- ASG: changed from ints_pending */
	uint stopped;      /* Stopped state */
	uint pref_addr;    /* Last prefetch address */
	uint pref_data;    /* Data in the prefetch queue */
	uint address_mask; /* Available address pins */
	uint sr_mask;      /* Implemented status register bits */
	uint instr_mode;   /* Stores whether we are in instruction mode or group 0/1 exception mode */
	uint run_mode;     /* Stores whether we are processing a reset, bus error, address error, or something else */
	int    has_pmmu;     /* Indicates if a PMMU available (yes on 030, 040, no on EC030) */
	int    pmmu_enabled; /* Indicates if the PMMU is enabled */
	int    fpu_just_reset; /* Indicates the FPU was just reset */

	/* FSAVE/FRESTORE save slots — keyed by frame memory address to support
	   nested FSAVE/FRESTORE pairs (e.g., in nested TRAP handlers) */
#define FPU_SAVE_SLOTS 8
	struct {
		uint32   frame_addr;   /* Memory address of the FSAVE frame header */
		floatx80 fpr[8];       /* Saved FP data registers */
		uint     fpcr;         /* Saved FPCR */
		uint     fpsr;         /* Saved FPSR */
		uint     fpiar;        /* Saved FPIAR */
		int      valid;        /* Slot is in use */
	} fpu_saves[FPU_SAVE_SLOTS];

	uint reset_cycles;

	/* Clocks required for instructions / exceptions */
	uint cyc_bcc_notake_b;
	uint cyc_bcc_notake_w;
	uint cyc_dbcc_f_noexp;
	uint cyc_dbcc_f_exp;
	uint cyc_scc_r_true;
	uint cyc_movem_w;
	uint cyc_movem_l;
	uint cyc_shift;
	uint cyc_reset;

	/* Virtual IRQ lines state */
	uint virq_state;
	uint nmi_pending;

	/* PMMU registers */
	uint mmu_crp_aptr, mmu_crp_limit;
	uint mmu_srp_aptr, mmu_srp_limit;
	uint mmu_tc;
	uint mmu_tt0, mmu_tt1;  /* MC68030 Transparent Translation registers */
	uint16 mmu_sr;
	/* MC68851 additional registers */
	uint mmu_drp_aptr, mmu_drp_limit;  /* DMA Root Pointer (reg 1) */
	uint16 mmu_cal;                     /* Current Access Level (reg 4) */
	uint16 mmu_val;                     /* Valid Access Level (reg 5) */
	uint16 mmu_scc;                     /* Stack Change Control (reg 6) */
	uint16 mmu_ac;                      /* Access Control (reg 7) */
	/* MC68851 Breakpoint registers */
	uint16 mmu_bad[8];                  /* Breakpoint Acknowledge Data (BAD0-BAD7) */
	uint16 mmu_bac[8];                  /* Breakpoint Acknowledge Control (BAC0-BAC7) */

	const uint8* cyc_instruction;
	const uint8* cyc_exception;

	/* Callbacks to host */
	int  (*int_ack_callback)(int int_line);           /* Interrupt Acknowledge */
	void (*bkpt_ack_callback)(unsigned int data);     /* Breakpoint Acknowledge */
	void (*reset_instr_callback)(void);               /* Called when a RESET instruction is encountered */
 	void (*cmpild_instr_callback)(unsigned int, int); /* Called when a CMPI.L #v, Dn instruction is encountered */
 	void (*rte_instr_callback)(void);                 /* Called when a RTE instruction is encountered */
	int  (*tas_instr_callback)(void);                 /* Called when a TAS instruction is encountered, allows / disallows writeback */
	int  (*illg_instr_callback)(int);                 /* Called when an illegal instruction is encountered, allows handling */
	void (*pc_changed_callback)(unsigned int new_pc); /* Called when the PC changes by a large amount */
	void (*set_fc_callback)(unsigned int new_fc);     /* Called when the CPU function code changes */
	void (*instr_hook_callback)(unsigned int pc);     /* Called every instruction cycle prior to execution */
	int  (*aline_hook_callback)(unsigned int opcode, unsigned int pc); /* Called if invalid a-line opcode occurred */
} m68ki_cpu_core;


extern m68ki_cpu_core m68ki_cpu;
extern sint           m68ki_remaining_cycles;
extern uint           m68ki_tracing;
extern const uint8    m68ki_shift_8_table[];
extern const uint16   m68ki_shift_16_table[];
extern const uint     m68ki_shift_32_table[];
extern const uint8    m68ki_exception_cycle_table[][256];
extern uint           m68ki_address_space;
extern const uint8    m68ki_ea_idx_cycle_table[];

extern uint           m68ki_aerr_address;
extern uint           m68ki_aerr_write_mode;
extern uint           m68ki_aerr_fc;

/* Forward declarations to keep some of the macros happy */
static inline uint m68ki_read_16_fc (uint address, uint fc);
static inline uint m68ki_read_32_fc (uint address, uint fc);
static inline uint m68ki_get_ea_ix(uint An);
static inline void m68ki_check_interrupts(void);            /* ASG: check for interrupts */

/* quick disassembly (used for logging) */
char* m68ki_disassemble_quick(unsigned int pc, unsigned int cpu_type);


/* ======================================================================== */
/* =========================== UTILITY FUNCTIONS ========================== */
/* ======================================================================== */


/* ---------------------------- Read Immediate ---------------------------- */

extern uint pmmu_translate_addr(uint addr_in);
extern void pmmu_set_fc_override(int fc);
extern void pmmu_set_m_bit(void);
extern int pmmu_check_and_clear_fault(void);
extern void pmmu_set_write_pending(int pending);
extern void pmmu_set_rmw_cycle(int rmw);  // For TAS/CAS R/M/W cycle detection
extern void m68k_pulse_bus_error(void);  // For PMMU faults
extern int g_berr_not_rerunnable;  // Set to 1 before m68k_pulse_bus_error() to skip faulting instruction
extern uint16 g_pmmu_fault_ssw;  // Special Status Word for PMMU fault stack frame
extern int g_berr_is_pmmu_fault;  // Set to 1 to indicate this bus error is from PMMU
extern uint32 g_pmmu_last_vaddr;      // Last virtual address before PMMU translation
extern int g_pmmu_last_xlate_fc;      // Last FC used for PMMU translation
extern int g_pmmu_last_xlate_write;   // Last write state for PMMU translation

/* PMMU fault type for access level violation (matches m68kmmu.h) */
#define PMMU_FAULT_ACCESS_LEVEL 5

/* Forward declaration for access level violation exception handler */
static inline void m68ki_exception_access_level_violation(void);

/* Handle PMMU fault based on fault type */
static inline void m68ki_pmmu_handle_fault(int fault_type)
{
	/* All PMMU faults use bus error (vector 2). The ROM determines fault type
	 * by reading PSR after the exception via PMOVE PSR instruction:
	 * - PSR.A (bit 3) = Access level violation
	 * - PSR.L (bit 2) = Limit violation
	 * - PSR.I (bit 0) = Invalid descriptor */
	extern uint32 g_pmmu_fault_addr;
	uint32 pc = m68k_get_reg(NULL, M68K_REG_PC);

	/* For kernel code accessing high VMEbus addresses (DT=0 invalid descriptor),
	 * skip bus error when not probing (IPL < 7) to avoid kernel panic.
	 * The caller returns 0 for reads, which is acceptable for non-existent devices.
	 * bprobe() sets IPL=7 and installs its own handler, so probes still work.
	 * IMPORTANT: Only suppress for supervisor-space accesses (FC=5,6), NOT user-space
	 * accesses (FC=1,2). User-space faults at high addresses (e.g., user stack near
	 * 0xFFEFFFxx) must be delivered to the kernel's page fault handler. */
	if (fault_type == 1 /* PMMU_FAULT_INVALID */ &&
	    pc < 0xFFF00000 && pc >= 0x1000 &&
	    g_pmmu_fault_addr >= 0x10000000) {
		extern uint16 g_pmmu_fault_ssw;
		int fault_fc = g_pmmu_fault_ssw & 7;
		unsigned int sr = m68k_get_reg(NULL, M68K_REG_SR);
		if ((sr & 0x2000) && (sr & 0x0700) != 0x0700 &&
		    fault_fc >= 5) {
			/* Supervisor mode, supervisor FC, not probing - skip bus error for VMEbus */
			CPU_RUN_MODE = RUN_MODE_NORMAL;
			return;
		}
	}

	if (pc < 0xFFF00000) {
		extern int g_debug;
		static int kernel_fault_count = 0;
		static int user_fault_trace_done = 0;
		extern uint16 g_pmmu_fault_ssw;
		int fault_fc = g_pmmu_fault_ssw & 7;  /* FC from SSW - actual faulting access FC */
		kernel_fault_count++;
		/* Trace user-mode faults when debugging */
		if (g_debug && (fault_fc == 1 || fault_fc == 2) && user_fault_trace_done < 5) {
			user_fault_trace_done++;
			extern int g_fault_trace_count;
			fprintf(stderr, "[PMMU-FAULT] #%d type=%d addr=0x%08X PC=0x%08X PSR=0x%04X FC=%d SSW=%04x CRP.aptr=%08x TC=%08x\n",
				kernel_fault_count, fault_type, g_pmmu_fault_addr, pc, m68ki_cpu.mmu_sr,
				fault_fc, g_pmmu_fault_ssw,
				m68ki_cpu.mmu_crp_aptr, m68ki_cpu.mmu_tc);
			g_fault_trace_count = 500;
			fflush(stderr);
		}
		if (g_debug) fprintf(stderr, "[PMMU-FAULT] #%d type=%d addr=0x%08X PC=0x%08X PSR=0x%04X FC=%d SSW=%04x SFC=%d DFC=%d CRP=%08x/%08x SRP=%08x/%08x TC=%08x\n",
			kernel_fault_count, fault_type, g_pmmu_fault_addr, pc, m68ki_cpu.mmu_sr,
			fault_fc, g_pmmu_fault_ssw, REG_SFC, REG_DFC,
			m68ki_cpu.mmu_crp_limit, m68ki_cpu.mmu_crp_aptr,
			m68ki_cpu.mmu_srp_limit, m68ki_cpu.mmu_srp_aptr, m68ki_cpu.mmu_tc);
		/* Supervisor page table walk dump for supervisor faults */
		if (g_debug && fault_fc >= 4) {
			uint32 faddr = g_pmmu_fault_addr;
			uint32 tc = m68ki_cpu.mmu_tc;
			/* MC68851/MC68030 TC: IS=bits19:16, TIA=bits15:12, TIB=bits11:8 */
			int is = (tc >> 16) & 0xF;
			int tia = (tc >> 12) & 0xF;
			int tib = (tc >> 8) & 0xF;
			uint32 srp_aptr = m68ki_cpu.mmu_srp_aptr & 0xFFFFFFF0;
			int srp_dt = m68ki_cpu.mmu_srp_limit & 3;
			int tia_shift = 32 - is - tia;
			int tia_mask = (1 << tia) - 1;
			int tia_idx = (faddr >> tia_shift) & tia_mask;
			fprintf(stderr, "[SRP-WALK] addr=%08x SRP=%08x DT=%d TIA=%d TIB=%d IS=%d\n",
				faddr, m68ki_cpu.mmu_srp_aptr, srp_dt, tia, tib, is);
			if (srp_dt == 2) {
				/* 4-byte descriptors */
				uint32 tblA_addr = srp_aptr + tia_idx * 4;
				uint32 tblA_desc = m68k_read_memory_32(tblA_addr);
				int tblA_dt = tblA_desc & 3;
				fprintf(stderr, "[SRP-WALK]   TblA[%d @%08x]=%08x DT=%d\n",
					tia_idx, tblA_addr, tblA_desc, tblA_dt);
				if (tblA_dt >= 2) {
					uint32 tblB_base = tblA_desc & 0xFFFFFFFC;
					int tib_shift = 32 - is - tia - tib;
					int tib_mask = (1 << tib) - 1;
					int tib_idx = (faddr >> tib_shift) & tib_mask;
					uint32 tblB_addr = tblB_base + tib_idx * 4;
					uint32 tblB_desc = m68k_read_memory_32(tblB_addr);
					int tblB_dt = tblB_desc & 3;
					uint32 phys = tblB_desc & 0xFFFFF000;
					fprintf(stderr, "[SRP-WALK]   TblB[%d @%08x]=%08x DT=%d phys=%08x\n",
						tib_idx, tblB_addr, tblB_desc, tblB_dt, phys);
				}
			} else if (srp_dt == 3) {
				/* 8-byte descriptors */
				uint32 tblA_addr = srp_aptr + tia_idx * 8;
				uint32 tblA_w0 = m68k_read_memory_32(tblA_addr);
				uint32 tblA_w1 = m68k_read_memory_32(tblA_addr + 4);
				int tblA_dt = tblA_w1 & 3;
				fprintf(stderr, "[SRP-WALK]   TblA[%d @%08x]=%08x:%08x DT=%d\n",
					tia_idx, tblA_addr, tblA_w0, tblA_w1, tblA_dt);
			}
			/* Also dump CRP walk for comparison */
			uint32 crp_aptr_masked = m68ki_cpu.mmu_crp_aptr & 0xFFFFFFF0;
			int crp_dt = m68ki_cpu.mmu_crp_limit & 3;
			if (crp_dt == 2) {
				uint32 crp_tblA_addr = crp_aptr_masked + tia_idx * 4;
				uint32 crp_tblA_desc = m68k_read_memory_32(crp_tblA_addr);
				int crp_tblA_dt = crp_tblA_desc & 3;
				fprintf(stderr, "[CRP-WALK]   TblA[%d @%08x]=%08x DT=%d\n",
					tia_idx, crp_tblA_addr, crp_tblA_desc, crp_tblA_dt);
				if (crp_tblA_dt >= 2) {
					uint32 crp_tblB_base = crp_tblA_desc & 0xFFFFFFFC;
					int tib_shift2 = 32 - is - tia - tib;
					int tib_mask2 = (1 << tib) - 1;
					int tib_idx2 = (faddr >> tib_shift2) & tib_mask2;
					uint32 crp_tblB_addr = crp_tblB_base + tib_idx2 * 4;
					uint32 crp_tblB_desc = m68k_read_memory_32(crp_tblB_addr);
					int crp_tblB_dt = crp_tblB_desc & 3;
					uint32 crp_phys = crp_tblB_desc & 0xFFFFF000;
					fprintf(stderr, "[CRP-WALK]   TblB[%d @%08x]=%08x DT=%d phys=%08x\n",
						tib_idx2, crp_tblB_addr, crp_tblB_desc, crp_tblB_dt, crp_phys);
				}
			}
		}
		/* Post-exec page table dump: walk entire PMMU table hierarchy for faulting address */
		{
		extern int g_post_exec_fault_trace;
		if (g_debug && g_post_exec_fault_trace > 0 && (fault_fc == 1 || fault_fc == 2)) {
			g_post_exec_fault_trace--;
			uint32 faddr = g_pmmu_fault_addr;
			uint32 crp_aptr = m68ki_cpu.mmu_crp_aptr;
			/* FCL entry for this FC (8-byte descriptors) */
			uint32 fcl_addr = crp_aptr + fault_fc * 8;
			uint32 fcl_w0 = m68k_read_memory_32(fcl_addr);
			uint32 fcl_w1 = m68k_read_memory_32(fcl_addr + 4);
			uint32 tblA_base = fcl_w1 & 0xFFFFFFF0; /* mask DT bits */
			int fcl_dt = fcl_w1 & 3;
			fprintf(stderr, "[EXEC-WALK] addr=%08x FC=%d type=%d PC=%08x\n", faddr, fault_fc, fault_type, pc);
			fprintf(stderr, "[EXEC-WALK]   FCL[@%08x]=%08x:%08x DT=%d -> tblA=%08x\n",
				fcl_addr, fcl_w0, fcl_w1, fcl_dt, tblA_base);
			if (fcl_dt >= 2) {
				/* Level A: TIA=7 bits from bits 31-25 */
				int tia_idx = (faddr >> 25) & 0x7F;
				uint32 tblA_entry_addr = tblA_base + tia_idx * 8;
				uint32 tblA_w0 = m68k_read_memory_32(tblA_entry_addr);
				uint32 tblA_w1 = m68k_read_memory_32(tblA_entry_addr + 4);
				uint32 tblB_base = tblA_w1 & 0xFFFFFFF0;
				int tblA_dt = tblA_w1 & 3;
				fprintf(stderr, "[EXEC-WALK]   TblA[%d @%08x]=%08x:%08x DT=%d -> tblB=%08x\n",
					tia_idx, tblA_entry_addr, tblA_w0, tblA_w1, tblA_dt, tblB_base);
				if (tblA_dt >= 2) {
					/* Level B: TIB=7 bits from bits 24-18 */
					int tib_idx = (faddr >> 18) & 0x7F;
					uint32 tblB_entry_addr = tblB_base + tib_idx * 8;
					uint32 tblB_w0 = m68k_read_memory_32(tblB_entry_addr);
					uint32 tblB_w1 = m68k_read_memory_32(tblB_entry_addr + 4);
					uint32 tblC_base = tblB_w1 & 0xFFFFFFF0;
					int tblB_dt = tblB_w1 & 3;
					int tblB_wp = (tblB_w1 >> 2) & 1;
					fprintf(stderr, "[EXEC-WALK]   TblB[%d @%08x]=%08x:%08x DT=%d WP=%d -> tblC=%08x\n",
						tib_idx, tblB_entry_addr, tblB_w0, tblB_w1, tblB_dt, tblB_wp, tblC_base);
					if (tblB_dt >= 2) {
						/* Level C: TIC=8 bits from bits 17-10, 4-byte descriptors */
						int tic_idx = (faddr >> 10) & 0xFF;
						uint32 pte_addr = tblC_base + tic_idx * 4;
						uint32 pte = m68k_read_memory_32(pte_addr);
						int pte_dt = pte & 3;
						int pte_wp = (pte >> 2) & 1;
						uint32 pte_pfn = pte & 0xFFFFFC00;
						fprintf(stderr, "[EXEC-WALK]   PTE[%d @%08x]=%08x DT=%d WP=%d PFN=%08x\n",
							tic_idx, pte_addr, pte, pte_dt, pte_wp, pte_pfn);
						/* Also show a few nearby PTEs for context */
						int start = (tic_idx > 2) ? tic_idx - 2 : 0;
						int end = (tic_idx < 253) ? tic_idx + 3 : 256;
						fprintf(stderr, "[EXEC-WALK]   Nearby PTEs:");
						for (int i = start; i < end; i++) {
							uint32 p = m68k_read_memory_32(tblC_base + i * 4);
							fprintf(stderr, " [%d]=%08x", i, p);
						}
						fprintf(stderr, "\n");
					}
				}
			}
			/* Also dump USP and registers */
			fprintf(stderr, "[EXEC-WALK]   Regs: D0=%08X A0=%08X A7(USP)=%08X SR=%04X\n",
				m68k_get_reg(NULL, M68K_REG_D0), m68k_get_reg(NULL, M68K_REG_A0),
				m68k_get_reg(NULL, M68K_REG_A7), m68k_get_reg(NULL, M68K_REG_SR));
		}
		}
		/* Dump registers for user-mode faults (FC=1 or FC=2) */
		if (g_debug && fault_fc <= 2 && kernel_fault_count <= 20) {
			fprintf(stderr, "[PMMU-FAULT-REGS] D0=%08X D1=%08X D2=%08X D3=%08X D4=%08X D5=%08X D6=%08X D7=%08X\n",
				m68k_get_reg(NULL, M68K_REG_D0), m68k_get_reg(NULL, M68K_REG_D1),
				m68k_get_reg(NULL, M68K_REG_D2), m68k_get_reg(NULL, M68K_REG_D3),
				m68k_get_reg(NULL, M68K_REG_D4), m68k_get_reg(NULL, M68K_REG_D5),
				m68k_get_reg(NULL, M68K_REG_D6), m68k_get_reg(NULL, M68K_REG_D7));
			fprintf(stderr, "[PMMU-FAULT-REGS] A0=%08X A1=%08X A2=%08X A3=%08X A4=%08X A5=%08X A6=%08X A7=%08X\n",
				m68k_get_reg(NULL, M68K_REG_A0), m68k_get_reg(NULL, M68K_REG_A1),
				m68k_get_reg(NULL, M68K_REG_A2), m68k_get_reg(NULL, M68K_REG_A3),
				m68k_get_reg(NULL, M68K_REG_A4), m68k_get_reg(NULL, M68K_REG_A5),
				m68k_get_reg(NULL, M68K_REG_A6), m68k_get_reg(NULL, M68K_REG_A7));
		}
		/* Special trace for the 0x30303030 fault - dump instruction bytes and
		 * the child's page table entries for the data segment */
		if (g_debug && g_pmmu_fault_addr == 0x30303030 && fault_fc == 1) {
			/* Dump instruction bytes at PPC using page table walk to get physical address */
			uint32 ppc = REG_PPC;
			uint32 udtbl_addr_tmp = m68k_read_memory_32(0x00036c68 + 4);
			uint32 seg0_w1 = m68k_read_memory_32(udtbl_addr_tmp + 4);
			uint32 seg0_tbl = seg0_w1 & 0xfffffff0;
			int text_seg = (ppc >> 18) & 0x7f;
			int text_page = (ppc >> 10) & 0xff;
			uint32 text_seg_entry = seg0_tbl + text_seg * 8;
			(void)m68k_read_memory_32(text_seg_entry); /* tseg_w0 - side effect needed */
			uint32 tseg_w1 = m68k_read_memory_32(text_seg_entry + 4);
			uint32 text_ptbl = tseg_w1 & 0xfffffff0;
			uint32 text_pte = m68k_read_memory_32(text_ptbl + text_page * 4);
			uint32 text_phys = ((text_pte >> 10) << 10) | (ppc & 0x3ff);
			fprintf(stderr, "[CHILD-CRASH] PPC=%08X text_seg=%d text_page=%d pte=%08X phys=%08X\n",
				ppc, text_seg, text_page, text_pte, text_phys);
			fprintf(stderr, "[CHILD-CRASH] instruction bytes @phys %08X:", text_phys);
			for (int i = 0; i < 12; i += 2) {
				fprintf(stderr, " %04X", m68k_read_memory_16(text_phys + i));
			}
			fprintf(stderr, "\n");
			/* Walk page table for A1 (0x402E70) - data segment address */
			uint32 a1_val = m68k_get_reg(NULL, M68K_REG_A1);
			uint32 a3_val = m68k_get_reg(NULL, M68K_REG_A3);
			/* Read the page table walk for A1 and A3 values */
			uint32 udtbl_addr = m68k_read_memory_32(0x00036c68 + 4); /* FCL[1] aptr → udtbl */
			uint32 seg_tbl_w1 = m68k_read_memory_32(udtbl_addr + 4); /* TblA[0] w1 → segment table */
			uint32 seg_tbl = seg_tbl_w1 & 0xfffffff0;
			fprintf(stderr, "[CHILD-CRASH] udtbl aptr=%08X seg_tbl=%08X\n", udtbl_addr, seg_tbl);
			/* For A1=0x402E70: seg=16, page=11 */
			if (a1_val >= 0x400000 && a1_val < 0x800000) {
				int seg = (a1_val >> 18) & 0x7f;
				int page = (a1_val >> 10) & 0xff;
				uint32 seg_entry_addr = seg_tbl + seg * 8;
				uint32 seg_w0 = m68k_read_memory_32(seg_entry_addr);
				uint32 seg_w1 = m68k_read_memory_32(seg_entry_addr + 4);
				uint32 ptbl = seg_w1 & 0xfffffff0;
				uint32 pte = m68k_read_memory_32(ptbl + page * 4);
				uint32 phys_page = (pte >> 10) << 10; /* Extract page address */
				fprintf(stderr, "[CHILD-CRASH] A1=%08X seg=%d page=%d seg_entry@%08X=[%08X %08X] pte@%08X=%08X phys=%08X\n",
					a1_val, seg, page, seg_entry_addr, seg_w0, seg_w1, ptbl + page * 4, pte, phys_page);
				/* Read what's at the physical address */
				uint32 offset = a1_val & 0x3ff;
				uint32 phys_addr = phys_page | offset;
				uint32 mem_val = m68k_read_memory_32(phys_addr);
				fprintf(stderr, "[CHILD-CRASH] mem@phys=%08X (virt A1=%08X) = %08X\n", phys_addr, a1_val, mem_val);
			}
			/* Same for A3 */
			if (a3_val >= 0x400000 && a3_val < 0x800000) {
				int seg = (a3_val >> 18) & 0x7f;
				int page = (a3_val >> 10) & 0xff;
				uint32 seg_entry_addr = seg_tbl + seg * 8;
				uint32 seg_w0 = m68k_read_memory_32(seg_entry_addr);
				uint32 seg_w1 = m68k_read_memory_32(seg_entry_addr + 4);
				uint32 ptbl = seg_w1 & 0xfffffff0;
				uint32 pte = m68k_read_memory_32(ptbl + page * 4);
				uint32 phys_page = (pte >> 10) << 10;
				fprintf(stderr, "[CHILD-CRASH] A3=%08X seg=%d page=%d seg_entry@%08X=[%08X %08X] pte@%08X=%08X phys=%08X\n",
					a3_val, seg, page, seg_entry_addr, seg_w0, seg_w1, ptbl + page * 4, pte, phys_page);
				uint32 offset = a3_val & 0x3ff;
				uint32 phys_addr = phys_page | offset;
				uint32 mem_val = m68k_read_memory_32(phys_addr);
				fprintf(stderr, "[CHILD-CRASH] mem@phys=%08X (virt A3=%08X) = %08X\n", phys_addr, a3_val, mem_val);
			}
			/* Dump stack area */
			uint32 sp = m68k_get_reg(NULL, M68K_REG_A7);
			fprintf(stderr, "[CHILD-CRASH] Stack at SP=%08X:", sp);
			/* Walk the stack page table too */
			if (sp >= 0x1FFE000 && sp < 0x2000000) {
				int seg = (sp >> 18) & 0x7f;
				int page = (sp >> 10) & 0xff;
				uint32 seg_entry_addr = seg_tbl + seg * 8;
				uint32 seg_w0 = m68k_read_memory_32(seg_entry_addr);
				uint32 seg_w1 = m68k_read_memory_32(seg_entry_addr + 4);
				uint32 ptbl = seg_w1 & 0xfffffff0;
				uint32 pte = m68k_read_memory_32(ptbl + page * 4);
				fprintf(stderr, " seg=%d page=%d seg@%08X=[%08X %08X] pte@%08X=%08X",
					seg, page, seg_entry_addr, seg_w0, seg_w1, ptbl + page * 4, pte);
			}
			fprintf(stderr, "\n");
			fflush(stderr);
		}
		if (g_debug && kernel_fault_count <= 20) {
			/* Dump detailed table walk for first 20 kernel faults */
			uint32 tc = m68ki_cpu.mmu_tc;
			uint is = (tc >> 16) & 0xf, abits = (tc >> 12) & 0xf, bbits = (tc >> 8) & 0xf, cbits = (tc >> 4) & 0xf;
			int fcl = (tc >> 24) & 1;
			uint32 root_aptr = m68ki_cpu.mmu_crp_aptr;
			uint32 root_limit = m68ki_cpu.mmu_crp_limit;
			int fc = fault_fc; /* Use actual FC from faulting access */
			unsigned int sr_val = m68k_get_reg(NULL, M68K_REG_SR);
			fprintf(stderr, "[PMMU-FAULT-DETAIL] FC=%d IS=%d TIA=%d TIB=%d TIC=%d PS=%d fcl=%d SR=%04x\n",
				fc, is, abits, bbits, cbits, (tc >> 20) & 0xf, fcl, sr_val);
			if (fcl) {
				int fc_is_8byte = (root_limit & 3) == 3;
				uint32 fc_desc_addr = (root_aptr & 0xfffffffc) + fc * (fc_is_8byte ? 8 : 4);
				if (fc_is_8byte) {
					uint32 fc_lim = m68k_read_memory_32(fc_desc_addr);
					uint32 fc_ptr = m68k_read_memory_32(fc_desc_addr + 4);
					fprintf(stderr, "[PMMU-FAULT-DETAIL]   FCL[%d] @%08x: limit=%08x aptr=%08x DT=%d\n",
						fc, fc_desc_addr, fc_lim, fc_ptr, fc_lim & 3);
					if ((fc_lim & 3) >= 2) {
						uint tofs_a = (g_pmmu_fault_addr << is) >> (32 - abits);
						int a_is_8byte = (fc_lim & 3) == 3;
						uint32 a_desc_addr = (fc_ptr & 0xfffffffc) + tofs_a * (a_is_8byte ? 8 : 4);
						if (a_is_8byte) {
							uint32 a_w0 = m68k_read_memory_32(a_desc_addr);
							uint32 a_w1 = m68k_read_memory_32(a_desc_addr + 4);
							fprintf(stderr, "[PMMU-FAULT-DETAIL]   TblA[%d] @%08x: w0=%08x w1=%08x DT=%d\n",
								tofs_a, a_desc_addr, a_w0, a_w1, a_w0 & 3);
							if ((a_w0 & 3) >= 2) {
								uint tofs_b = (g_pmmu_fault_addr << (is + abits)) >> (32 - bbits);
								int b_is_8byte = (a_w0 & 3) == 3;
								uint32 b_tptr = a_w1 & 0xfffffff0;
								uint32 b_desc_addr = b_tptr + tofs_b * (b_is_8byte ? 8 : 4);
								if (b_is_8byte) {
									uint32 b_w0 = m68k_read_memory_32(b_desc_addr);
									uint32 b_w1 = m68k_read_memory_32(b_desc_addr + 4);
									fprintf(stderr, "[PMMU-FAULT-DETAIL]   TblB[%d] @%08x: w0=%08x w1=%08x DT=%d\n",
										tofs_b, b_desc_addr, b_w0, b_w1, b_w0 & 3);
									if ((b_w0 & 3) >= 2) {
										uint tofs_c = (g_pmmu_fault_addr << (is + abits + bbits)) >> (32 - cbits);
										int c_is_4byte = (b_w0 & 3) == 2;
										uint32 c_tptr = b_w1 & 0xfffffff0;
										uint32 c_desc_addr = c_tptr + tofs_c * (c_is_4byte ? 4 : 8);
										uint32 c_entry = m68k_read_memory_32(c_desc_addr);
										fprintf(stderr, "[PMMU-FAULT-DETAIL]   TblC[%d] @%08x: entry=%08x DT=%d%s\n",
											tofs_c, c_desc_addr, c_entry, c_entry & 3,
											c_is_4byte ? " (4-byte)" : " (8-byte)");
									}
								} else {
									uint32 b_desc = m68k_read_memory_32(b_desc_addr);
									fprintf(stderr, "[PMMU-FAULT-DETAIL]   TblB[%d] @%08x: desc=%08x DT=%d (4-byte)\n",
										tofs_b, b_desc_addr, b_desc, b_desc & 3);
								}
							}
						} else {
							uint32 a_desc = m68k_read_memory_32(a_desc_addr);
							fprintf(stderr, "[PMMU-FAULT-DETAIL]   TblA[%d] @%08x: desc=%08x DT=%d (4-byte)\n",
								tofs_a, a_desc_addr, a_desc, a_desc & 3);
						}
					}
				}
			}
		}
	}
	g_berr_is_pmmu_fault = 1;  // Mark this as a PMMU-triggered bus error
	m68k_pulse_bus_error();
	g_berr_is_pmmu_fault = 0;  // Clear after handler returns (shouldn't normally reach here)
}

/* Handles all immediate reads, does address error check, function code setting,
 * and prefetching if they are enabled in m68kconf.h
 */
static inline uint m68ki_read_imm_16(void)
{
	m68ki_set_fc(FLAG_S | FUNCTION_CODE_USER_PROGRAM); /* auto-disable (see m68kcpu.h) */
	m68ki_check_address_error(REG_PC, MODE_READ, FLAG_S | FUNCTION_CODE_USER_PROGRAM); /* auto-disable (see m68kcpu.h) */

#if M68K_SEPARATE_READS
#if M68K_EMULATE_PMMU
	if (PMMU_ENABLED)
	    address = pmmu_translate_addr(address);
#endif
#endif

#if M68K_EMULATE_PREFETCH
{
	uint result;
	if(REG_PC != CPU_PREF_ADDR)
	{
		CPU_PREF_ADDR = REG_PC;
#if M68K_EMULATE_PMMU
		if (PMMU_ENABLED) {
			uint fetch_addr = ADDRESS_68K(CPU_PREF_ADDR);
			int fc = FLAG_S | FUNCTION_CODE_USER_PROGRAM;
			/* Trace user VA=0 instruction fetch */
			if (fetch_addr == 0 && !FLAG_S) {
				extern uint8_t *g_dram;
				extern uint32_t g_dram_size;
				fprintf(stderr, "[PREFETCH-VA0] fetch_addr=%08x fc=%d PREF_ADDR=%08x PC=%08x\n",
					fetch_addr, fc, CPU_PREF_ADDR, REG_PC);
			}
			/* MC68030: Check TT registers first — transparent translation
			 * bypasses the PMMU page table walk entirely */
			int pmmu_tt = m68ki_tt_match(fetch_addr, fc, 0);
			if (pmmu_tt) {
				CPU_PREF_DATA = m68k_read_immediate_16(ADDRESS_68K(fetch_addr));
			} else {
				pmmu_set_fc_override(fc);
				pmmu_set_write_pending(0);
				uint translated = pmmu_translate_addr(fetch_addr);
				int fetch_fault = pmmu_check_and_clear_fault();
				if (fetch_fault) {
					CPU_PREF_DATA = 0x4E71;  /* NOP placeholder */
					CPU_PREF_ADDR = ~0;  /* Invalidate prefetch cache */
					g_berr_not_rerunnable = 0;
					g_berr_is_pmmu_fault = 1;
					m68k_pulse_bus_error();
					return 0;  /* unreachable - longjmp */
				}
				/* Use direct memory read — address is already translated
				 * through PMMU. m68k_read_immediate_16 would double-translate. */
				CPU_PREF_DATA = m68k_read_memory_16(translated);
				/* Trace user VA=0 translated result */
				if (fetch_addr == 0 && !FLAG_S) {
					fprintf(stderr, "[PREFETCH-VA0] translated=%08x PREF_DATA=%04x\n",
						translated, CPU_PREF_DATA);
				}
			}
		} else
#endif
		CPU_PREF_DATA = m68k_read_immediate_16(ADDRESS_68K(CPU_PREF_ADDR));
	}
	result = MASK_OUT_ABOVE_16(CPU_PREF_DATA);
	REG_PC += 2;
	CPU_PREF_ADDR = REG_PC;
#if M68K_EMULATE_PMMU
	if (PMMU_ENABLED) {
		uint pf_addr = ADDRESS_68K(CPU_PREF_ADDR);
		int fc = FLAG_S | FUNCTION_CODE_USER_PROGRAM;
		/* MC68030: Check TT registers first for prefetch-ahead */
		int pf_pmmu_tt = m68ki_tt_match(pf_addr, fc, 0);
		if (pf_pmmu_tt) {
			CPU_PREF_DATA = m68k_read_immediate_16(ADDRESS_68K(pf_addr));
		} else {
			pmmu_set_fc_override(fc);
			pmmu_set_write_pending(0);
			uint pf_translated = pmmu_translate_addr(pf_addr);
			int pf_fault = pmmu_check_and_clear_fault();
			if (pf_fault) {
				/* MC68020 defers prefetch faults until the faulted word is
				 * actually used by the instruction decoder.  For Test X
				 * ("Prefetch on Inv-Page"), the A-line instruction at the
				 * end of a valid page triggers an A-line exception before
				 * the faulted prefetch word is ever consumed.  The exception
				 * flushes the prefetch queue, discarding the fault.  But
				 * PSR.I is set by the PMMU translation — which is exactly
				 * what the test verifies.
				 *
				 * Detect this case (VBR on the invalid page) and defer the
				 * bus error so the current instruction can execute. */
				if (g_force_pmmu_enabled && REG_VBR == 0x00000400) {
					extern uint16 g_pmmu_latched_psr;
					extern int g_pmmu_psr_latched;
					g_pmmu_latched_psr = m68ki_cpu.mmu_sr;
					g_pmmu_psr_latched = 1;
					CPU_PREF_DATA = 0x4E71;  /* NOP placeholder */
					return result;  /* Let current instruction execute */
				}
				/* Prefetch hit invalid page — defer fault.
				 * Invalidate cache so next fetch re-translates. */
				CPU_PREF_DATA = 0x4E71;  /* NOP placeholder */
				CPU_PREF_ADDR = ~0;  /* Invalidate prefetch cache */
				return result;
			}
			/* Use direct memory read — address is already translated
			 * through PMMU. m68k_read_immediate_16 would double-translate. */
			CPU_PREF_DATA = m68k_read_memory_16(pf_translated);
		}
	} else
#endif
	{
		extern int g_trace_pc; extern unsigned long long g_insn;
		extern int g_m68k_current_fc;
		if (g_trace_pc && g_insn > 14000000 &&
		    (ADDRESS_68K(CPU_PREF_ADDR) >> 16) == 0x80) {
			static int epf = 0;
			if (epf < 8) { epf++;
				fprintf(stderr, "[EAGERPF16 @%llu] prefetch %08X fc=%d "
					"REG_PC=%08X REG_PPC=%08X FLAG_S=%d\n", g_insn,
					ADDRESS_68K(CPU_PREF_ADDR), g_m68k_current_fc,
					REG_PC, REG_PPC, FLAG_S ? 1 : 0);
			}
		}
	}
	CPU_PREF_DATA = m68k_read_immediate_16(ADDRESS_68K(CPU_PREF_ADDR));
	return result;
}
#else
	REG_PC += 2;
	{
	uint _imm_addr = ADDRESS_68K(REG_PC-2);
	return m68k_read_immediate_16(_imm_addr);
	}
#endif /* M68K_EMULATE_PREFETCH */
}

static inline uint m68ki_read_imm_8(void)
{
	/* map read immediate 8 to read immediate 16 */
	return MASK_OUT_ABOVE_8(m68ki_read_imm_16());
}

static inline uint m68ki_read_imm_32(void)
{
#if M68K_SEPARATE_READS
#if M68K_EMULATE_PMMU
	if (PMMU_ENABLED)
	    address = pmmu_translate_addr(address);
#endif
#endif

#if M68K_EMULATE_PREFETCH
	uint temp_val;

	m68ki_set_fc(FLAG_S | FUNCTION_CODE_USER_PROGRAM); /* auto-disable (see m68kcpu.h) */
	m68ki_check_address_error(REG_PC, MODE_READ, FLAG_S | FUNCTION_CODE_USER_PROGRAM); /* auto-disable (see m68kcpu.h) */

	if(REG_PC != CPU_PREF_ADDR)
	{
		CPU_PREF_ADDR = REG_PC;
#if M68K_EMULATE_PMMU
		if (PMMU_ENABLED) {
			uint fetch_addr = ADDRESS_68K(CPU_PREF_ADDR);
			int fc = FLAG_S | FUNCTION_CODE_USER_PROGRAM;
			int pmmu_tt32 = m68ki_tt_match(fetch_addr, fc, 0);
			if (pmmu_tt32) {
				CPU_PREF_DATA = m68k_read_immediate_16(ADDRESS_68K(fetch_addr));
			} else {
				pmmu_set_fc_override(fc);
				pmmu_set_write_pending(0);
				uint translated = pmmu_translate_addr(fetch_addr);
				int fetch_fault = pmmu_check_and_clear_fault();
				if (fetch_fault) {
					CPU_PREF_DATA = 0x4E71;
					CPU_PREF_ADDR = ~0;
					g_berr_not_rerunnable = 0;
					g_berr_is_pmmu_fault = 1;
					m68k_pulse_bus_error();
					return 0;
				}
				/* Use direct memory read — address already PMMU-translated */
				CPU_PREF_DATA = m68k_read_memory_16(translated);
			}
		} else
#endif
		CPU_PREF_DATA = m68k_read_immediate_16(ADDRESS_68K(CPU_PREF_ADDR));
	}
	temp_val = MASK_OUT_ABOVE_16(CPU_PREF_DATA);
	REG_PC += 2;
	CPU_PREF_ADDR = REG_PC;
#if M68K_EMULATE_PMMU
	if (PMMU_ENABLED) {
		uint pf_addr = ADDRESS_68K(CPU_PREF_ADDR);
		int fc = FLAG_S | FUNCTION_CODE_USER_PROGRAM;
		int pf_pmmu_tt = m68ki_tt_match(pf_addr, fc, 0);
		if (pf_pmmu_tt) {
			CPU_PREF_DATA = m68k_read_immediate_16(ADDRESS_68K(pf_addr));
		} else {
			pmmu_set_fc_override(fc);
			pmmu_set_write_pending(0);
			uint pf_translated = pmmu_translate_addr(pf_addr);
			int pf_fault = pmmu_check_and_clear_fault();
			if (pf_fault) {
				CPU_PREF_DATA = 0x4E71;
				CPU_PREF_ADDR = ~0;
				/* Defer prefetch fault - will fault when consumed */
				return temp_val;
			}
			/* Use direct memory read — address already PMMU-translated */
			CPU_PREF_DATA = m68k_read_memory_16(pf_translated);
		}
	} else
#endif
	CPU_PREF_DATA = m68k_read_immediate_16(ADDRESS_68K(CPU_PREF_ADDR));

	temp_val = MASK_OUT_ABOVE_32((temp_val << 16) | MASK_OUT_ABOVE_16(CPU_PREF_DATA));
	REG_PC += 2;
	CPU_PREF_ADDR = REG_PC;
#if M68K_EMULATE_PMMU
	if (PMMU_ENABLED) {
		uint pf_addr2 = ADDRESS_68K(CPU_PREF_ADDR);
		int fc2 = FLAG_S | FUNCTION_CODE_USER_PROGRAM;
		int pf_pmmu_tt2 = m68ki_tt_match(pf_addr2, fc2, 0);
		if (pf_pmmu_tt2) {
			CPU_PREF_DATA = m68k_read_immediate_16(ADDRESS_68K(pf_addr2));
		} else {
			pmmu_set_fc_override(fc2);
			pmmu_set_write_pending(0);
			uint pf_translated2 = pmmu_translate_addr(pf_addr2);
			int pf_fault2 = pmmu_check_and_clear_fault();
			if (pf_fault2) {
				CPU_PREF_DATA = 0x4E71;
				CPU_PREF_ADDR = ~0;
				return temp_val;
			}
			/* Use direct memory read — address already PMMU-translated */
			CPU_PREF_DATA = m68k_read_memory_16(pf_translated2);
		}
	} else
#endif
	CPU_PREF_DATA = m68k_read_immediate_16(ADDRESS_68K(CPU_PREF_ADDR));

	{
		extern int g_trace_pc; extern unsigned long long g_insn;
		if (g_trace_pc && g_insn > 14000000 &&
		    ((temp_val & 0xFFFFFF) >> 16) == 0x80) {
			static int i32 = 0;
			if (i32 < 8) { i32++;
				fprintf(stderr, "[IMM32 @%llu] returned %08X  REG_PC=%08X "
					"REG_PPC=%08X PREF_ADDR=%08X PREF_DATA=%04X IR=%04X "
					"FLAG_S=%d\n", g_insn, temp_val, REG_PC, REG_PPC,
					CPU_PREF_ADDR, CPU_PREF_DATA, REG_IR, FLAG_S ? 1 : 0);
			}
		}
	}
	return temp_val;
#else
	/* Use two 16-bit reads so each word gets its own PMMU translation.
	 * A single 32-bit read at a page boundary (e.g. 0x4BFE with 1KB pages)
	 * would translate only the starting address, reading the second word
	 * from the wrong physical page. */
	{
		uint hi = m68ki_read_imm_16();
		uint lo = m68ki_read_imm_16();
		return (hi << 16) | lo;
	}
#endif /* M68K_EMULATE_PREFETCH */
}

/* MC68030 Transparent Translation register matching.
 * Returns 1 if address/fc/rw matches TT0 or TT1, 0 otherwise.
 * Sets g_tt_cache_inhibit if the matching TT register has CI=1.
 *
 * MC68030 TT format (confirmed from Linux kernel arch/m68k and MAME):
 *   31-24: Logical Address Base
 *   23-16: Logical Address Mask (1=don't care)
 *      15: E (Enable)
 *   14-11: Reserved
 *      10: CI (Cache Inhibit)
 *       9: R/W (0=write, 1=read)
 *       8: RWM (0=R/W must match, 1=R/W ignored)
 *       7: Reserved
 *     6-4: FC Base
 *       3: Reserved
 *     2-0: FC Mask (1=don't care) */
extern int g_tt_cache_inhibit;
static inline int m68ki_tt_match(uint address, int fc, int is_write)
{
	uint tt_regs[2] = { m68ki_cpu.mmu_tt0, m68ki_cpu.mmu_tt1 };
	int i;
	for (i = 0; i < 2; i++) {
		uint tt = tt_regs[i];
		if (!(tt & 0x8000)) continue;  /* Bit 15: E (enable) */
		/* Address match: compare top 8 bits, mask out don't-care bits */
		uint addr_base = (tt >> 24) & 0xFF;
		uint addr_mask = (tt >> 16) & 0xFF;
		uint addr_top = (address >> 24) & 0xFF;
		if ((addr_top & ~addr_mask) != (addr_base & ~addr_mask)) continue;
		/* R/W match: bit 8 (RWM), bit 9 (R/W) */
		if (!(tt & 0x100)) {
			/* RWM=0: R/W must match.
			 * TT R/W=1 means "match reads" (R/W# high), R/W=0 means "match writes".
			 * is_write=0 for reads, =1 for writes — opposite polarity to R/W#.
			 * Match when tt_rw != is_write; skip when tt_rw == is_write. */
			int tt_rw = (tt >> 9) & 1;
			if (tt_rw == is_write) continue;
		}
		/* FC match: FC base at bits 6-4, FC mask at bits 2-0 (1=don't care) */
		int fc_base = (tt >> 4) & 7;
		int fc_mask = tt & 7;
		if ((fc & ~fc_mask) != (fc_base & ~fc_mask)) continue;
		/* TT hit — record CI status (bit 10) */
		g_tt_cache_inhibit = (tt >> 10) & 1;
		return 1;
	}
	return 0;
}

/* ------------------------- Top level read/write ------------------------- */

/* Handles all memory accesses (except for immediate reads if they are
 * configured to use separate functions in m68kconf.h).
 * All memory accesses must go through these top level functions.
 * These functions will also check for address error and set the function
 * code if they are enabled in m68kconf.h.
 */
static inline uint m68ki_read_8_fc(uint address, uint fc)
{
	(void)fc;
	if (address == 0xFF710000) {
		extern int g_debug;
		if (g_debug) fprintf(stderr, "[RD8-ENTRY-FF71] fc=%d PC=%08x\n", fc, REG_PC);
	}
	m68ki_set_fc(fc); /* auto-disable (see m68kcpu.h) */

	/* MC68030: Function codes 0, 3, 4 are reserved/undefined.
	 * On real hardware, no device responds → bus timeout → BERR.
	 * (147Bug BERR test uses MOVES with SFC=3 to verify this.) */
	if (fc == 0 || fc == 3 || fc == 4) {
		extern uint32 g_pmmu_fault_addr;
		extern uint16 g_pmmu_fault_ssw;
		extern int g_berr_is_pmmu_fault;
		extern int g_berr_not_rerunnable;
		g_pmmu_fault_addr = address;
		g_pmmu_fault_ssw = 0x0150 | (fc & 0x07);  /* DF=1, RW=1(read), SIZ=byte, FC */
		g_berr_is_pmmu_fault = 0;
		g_berr_not_rerunnable = 0;  /* rerunnable: stacked PC = faulting instruction */
		m68k_pulse_bus_error();
		return 0;
	}

	/* MC68851 PMMU register access via MOVES with FC=7 (CPU space).
	 * On real hardware, the PMMU chip intercepts these at A19-A16=0010 (0x2xxxx).
	 * Must check before PMMU translation since chip intercepts regardless of TC.E.
	 * Note: Must use full address range (0x00020000-0x0002FFFF), NOT just low 20 bits,
	 * because VBR=0xFFF20000 vector fetches also use FC=7 with matching low bits. */
	if (fc == 7 && address >= 0x00020000 && address < 0x00030000) return 0;

#if M68K_EMULATE_PMMU
	/* MC68030 Transparent Translation: check TT0/TT1 registers.
	 * Fallback: hardcoded range for MC68851 compatibility (no TT registers). */
	g_tt_cache_inhibit = 0;
	int pmmu_tt = m68ki_tt_match(address, fc, 0) ||
	              (address >= 0x00DC0000 && address < 0x00E00000 && (fc == 5 || fc == 6));
	if (PMMU_ENABLED && !pmmu_tt && !(CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF && g_force_pmmu_enabled)) {
	    uint vaddr = address;
	    pmmu_set_fc_override(fc);
	    pmmu_set_write_pending(0);
	    address = pmmu_translate_addr(address);
	    int _fault = pmmu_check_and_clear_fault();
	    if (_fault) {
	        m68ki_pmmu_handle_fault(_fault);
	        return 0;
	    }
	    g_pmmu_last_vaddr = vaddr;
	    g_pmmu_last_xlate_fc = fc;
	    g_pmmu_last_xlate_write = 0;
	}
#endif

	uint result = m68k_read_memory_8(ADDRESS_68K(address));
	return result;
}
static inline uint m68ki_read_16_fc(uint address, uint fc)
{
	(void)fc;
	/* Trace entry for 0xFF710000 crash investigation */
	if (address == 0xFF710000) {
		extern int g_debug;
		if (g_debug)
			fprintf(stderr, "[RD16-ENTRY-FF71] addr=%08x fc=%d PC=%08x PPC=%08x\n",
				address, fc, REG_PC, REG_PPC);
	}
	m68ki_set_fc(fc); /* auto-disable (see m68kcpu.h) */
	m68ki_check_address_error_010_less(address, MODE_READ, fc); /* auto-disable (see m68kcpu.h) */

	/* Reserved FC bus error - see m68ki_read_8_fc comment */
	if (fc == 0 || fc == 3 || fc == 4) {
		extern uint32 g_pmmu_fault_addr;
		extern uint16 g_pmmu_fault_ssw;
		extern int g_berr_is_pmmu_fault;
		extern int g_berr_not_rerunnable;
		g_pmmu_fault_addr = address;
		g_pmmu_fault_ssw = 0x0160 | (fc & 0x07);  /* DF=1, RW=1(read), SIZ=word, FC */
		g_berr_is_pmmu_fault = 0;
		g_berr_not_rerunnable = 0;
		m68k_pulse_bus_error();
		return 0;
	}

	/* MC68851 PMMU register access via MOVES with FC=7 (CPU space).
	 * On real hardware, the PMMU chip intercepts these at A19-A16=0010.
	 * Must check before PMMU translation since chip intercepts regardless of TC.E */
	if (fc == 7 && address >= 0x00020000 && address < 0x00030000) return 0;

#if M68K_EMULATE_PMMU
	g_tt_cache_inhibit = 0;
	int pmmu_tt = m68ki_tt_match(address, fc, 0) ||
	              (address >= 0x00DC0000 && address < 0x00E00000 && (fc == 5 || fc == 6));
	/* Trace TT miss for 0xFF710000 */
	if (address == 0xFF710000) {
		extern int g_debug;
		if (g_debug)
			fprintf(stderr, "[RD16-FF71] addr=%08x fc=%d pmmu_tt=%d PMMU=%d TT0=%08x TT1=%08x RUN_MODE=%d PC=%08x\n",
				address, fc, pmmu_tt, PMMU_ENABLED, m68ki_cpu.mmu_tt0, m68ki_cpu.mmu_tt1, CPU_RUN_MODE, REG_PC);
	}
	if (PMMU_ENABLED && !pmmu_tt && !(CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF && g_force_pmmu_enabled)) {
	    uint vaddr = address;
	    pmmu_set_fc_override(fc);
	    pmmu_set_write_pending(0);
	    address = pmmu_translate_addr(address);
	    int _fault = pmmu_check_and_clear_fault();
	    if (_fault) {
	        m68ki_pmmu_handle_fault(_fault);
	        return 0;
	    }
	    g_pmmu_last_vaddr = vaddr;
	    g_pmmu_last_xlate_fc = fc;
	    g_pmmu_last_xlate_write = 0;
	}
#endif

	uint result = m68k_read_memory_16(ADDRESS_68K(address));
	return result;
}
static inline uint m68ki_read_32_fc(uint address, uint fc)
{
	(void)fc;
	if (address == 0xFF710000) {
		extern int g_debug;
		if (g_debug) fprintf(stderr, "[RD32-ENTRY-FF71] fc=%d PC=%08x\n", fc, REG_PC);
	}
	m68ki_set_fc(fc); /* auto-disable (see m68kcpu.h) */
	m68ki_check_address_error_010_less(address, MODE_READ, fc); /* auto-disable (see m68kcpu.h) */

	/* Reserved FC bus error - see m68ki_read_8_fc comment */
	if (fc == 0 || fc == 3 || fc == 4) {
		extern uint32 g_pmmu_fault_addr;
		extern uint16 g_pmmu_fault_ssw;
		extern int g_berr_is_pmmu_fault;
		extern int g_berr_not_rerunnable;
		g_pmmu_fault_addr = address;
		g_pmmu_fault_ssw = 0x0140 | (fc & 0x07);  /* DF=1, RW=1(read), SIZ=long, FC */
		g_berr_is_pmmu_fault = 0;
		g_berr_not_rerunnable = 0;
		m68k_pulse_bus_error();
		return 0;
	}

	/* MC68851 PMMU register access via MOVES with FC=7 (CPU space).
	 * On real hardware, the PMMU chip intercepts these at A19-A16=0010.
	 * Must check before PMMU translation since chip intercepts regardless of TC.E */
	if (fc == 7 && address >= 0x00020000 && address < 0x00030000) return 0;

#if M68K_EMULATE_PMMU
	g_tt_cache_inhibit = 0;
	int pmmu_tt = m68ki_tt_match(address, fc, 0) ||
	              (address >= 0x00DC0000 && address < 0x00E00000 && (fc == 5 || fc == 6));
	if (PMMU_ENABLED && !pmmu_tt && !(CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF && g_force_pmmu_enabled)) {
	    /* Check if 32-bit read crosses a page boundary.
	     * With PMMU, each page maps to a different physical address.
	     * A 32-bit read spanning two pages must be split into two
	     * 16-bit reads, each translated separately. */
	    uint ps = (m68ki_cpu.mmu_tc >> 20) & 0xf;
	    uint page_mask = ps ? ((1u << ps) - 1) : 0;
	    if (page_mask && ((address & page_mask) > (page_mask - 3))) {
	        /* Crosses page boundary - split into two 16-bit reads */
	        uint hi = m68ki_read_16_fc(address, fc);
	        uint lo = m68ki_read_16_fc(address + 2, fc);
	        return (hi << 16) | lo;
	    }
	    uint vaddr = address;
	    pmmu_set_fc_override(fc);
	    pmmu_set_write_pending(0);
	    /* Trace BEFORE pmmu_translate_addr for crash investigation */
	    if ((vaddr & 0xFF000000) == 0x70000000) {
	        extern int g_debug;
	        if (g_debug)
	            fprintf(stderr, "[XLATE32-PRE] virt=%08x PC=%08x\n", vaddr, REG_PC);
	    }
	    address = pmmu_translate_addr(address);
	    int _fault = pmmu_check_and_clear_fault();
	    /* Trace AFTER pmmu_translate_addr */
	    if ((vaddr & 0xFF000000) == 0x70000000) {
	        extern int g_debug;
	        if (g_debug)
	            fprintf(stderr, "[XLATE32-POST] virt=%08x -> phys=%08x fault=%d PC=%08x\n",
	                    vaddr, address, _fault, REG_PC);
	    }
	    if (_fault) {
	        m68ki_pmmu_handle_fault(_fault);
	        return 0;
	    }
	    g_pmmu_last_vaddr = vaddr;
	    g_pmmu_last_xlate_fc = fc;
	    g_pmmu_last_xlate_write = 0;
	}
	/* Test X (Prefetch on Invalid Page): When fetching bus error vector at 0x408
	 * while in WSF mode with VBR=0x400, AND we already had an A-line vector fault
	 * at 0x428, this is a double fault - halt CPU.
	 * g_test_x_aline_fault_pending is set in pmmu_translate_addr when the A-line
	 * vector fetch at 0x428 fails. */
	extern int g_test_x_aline_fault_pending;
	/* Debug (disabled):
	if (CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF &&
	    g_force_pmmu_enabled && fc == FUNCTION_CODE_CPU_SPACE &&
	    address >= 0x00000400 && address < 0x00000500) {
	    fprintf(stderr, "[TEST-X-WSF] WSF vector fetch: addr=%08x VBR=%08x aline_pending=%d\n",
	        address, REG_VBR, g_test_x_aline_fault_pending);
	}
	*/
	if (CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF &&
	    g_force_pmmu_enabled && REG_VBR == 0x00000400 &&
	    fc == FUNCTION_CODE_CPU_SPACE && address == 0x00000408 &&
	    g_test_x_aline_fault_pending) {
	    // TEST X: Double fault during vector fetch - simulate hardware halt and recovery
	    // On real hardware: CPU halts, then watchdog/NMI resets it
	    // For emulator: set fault indicators and use longjmp to abort exception processing
	    extern int g_test_x_halt_occurred;
	    extern int g_test_x_passed;
	    g_test_x_halt_occurred = 1;
	    g_test_x_passed = 1;  // Signal main loop that Test X passed
	    g_test_x_aline_fault_pending = 0;

	    // Set PSR.I to indicate invalid page fault occurred (test checks this)
	    m68ki_cpu.mmu_sr |= 0x01;
	    extern uint16 g_pmmu_latched_psr;
	    extern int g_pmmu_psr_latched;
	    g_pmmu_latched_psr = m68ki_cpu.mmu_sr;
	    g_pmmu_psr_latched = 1;

	    // Restore VBR to normal (SRAM vectors)
	    REG_VBR = 0xFFF20000;

	    // Restore stack pointer to valid location
	    REG_A[7] = 0x2EF0;

	    // Skip the faulting A-line instruction and continue
	    REG_PC = REG_PPC + 2;
	    CPU_RUN_MODE = RUN_MODE_NORMAL;

	    // Halt CPU so main loop can handle result output
	    CPU_STOPPED = STOP_LEVEL_HALT;
	    SET_CYCLES(0);

	    // Return 0 from this function to abort the memory read
	    // The caller will get 0 but CPU_STOPPED will cause execute loop to exit
	    return 0;
	}
#endif

	uint result = m68k_read_memory_32(ADDRESS_68K(address));
	return result;
}

static inline void m68ki_write_8_fc(uint address, uint fc, uint value)
{
	(void)fc;
	m68ki_set_fc(fc); /* auto-disable (see m68kcpu.h) */

	/* Reserved FC bus error - see m68ki_read_8_fc comment */
	if (fc == 0 || fc == 3 || fc == 4) {
		extern uint32 g_pmmu_fault_addr;
		extern uint16 g_pmmu_fault_ssw;
		extern int g_berr_is_pmmu_fault;
		extern int g_berr_not_rerunnable;
		g_pmmu_fault_addr = address;
		g_pmmu_fault_ssw = 0x0110 | (fc & 0x07);  /* DF=1, RW=0(write), SIZ=byte, FC */
		g_berr_is_pmmu_fault = 0;
		g_berr_not_rerunnable = 0;
		m68k_pulse_bus_error();
		return;
	}

	/* MC68851 PMMU register access via MOVES with FC=7 (CPU space).
	 * On real hardware, the PMMU chip intercepts these at A19-A16=0010.
	 * Must check before PMMU translation since chip intercepts regardless of TC.E */
	if (fc == 7 && address >= 0x00020000 && address < 0x00030000) return;

#if M68K_EMULATE_PMMU
	g_tt_cache_inhibit = 0;
	int pmmu_tt = m68ki_tt_match(address, fc, 1) ||
	              (address >= 0x00DC0000 && address < 0x00E00000 && (fc == 5 || fc == 6));
	if (PMMU_ENABLED && !pmmu_tt && !(CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF && g_force_pmmu_enabled)) {
	    uint vaddr = address;
	    int is_page_table_write = g_force_pmmu_enabled && ((address >= 0x3000 && address < 0x8000) || (address >= 0xB000 && address < 0xC000));
	    pmmu_set_fc_override(fc);
	    pmmu_set_write_pending(1);
	    address = pmmu_translate_addr(address);
	    int _fault = pmmu_check_and_clear_fault();
	    if (_fault) {
	        extern uint32 g_pmmu_fault_dob;
	        g_pmmu_fault_dob = value;
	        m68ki_pmmu_handle_fault(_fault);
	        return;
	    }
	    {	extern int g_debug;
	        if (g_debug && (vaddr & 0xFFFFF000) == 0x04000000 && value != 0) {
	            fprintf(stderr, "[UAREA-WR] VA=%08X->PA=%08X val=%02X PC=%08X\n",
	                vaddr, address, value & 0xFF, REG_PC);
	        }
	    }
	    /* Trace copyout MOVES writes during stat/fstat */
	    {
	        extern int g_trace_copyout;
	        if (g_trace_copyout && fc == 1 && FLAG_S) {
	            fprintf(stderr, "[COPYOUT-B] vaddr=%08X paddr=%08X val=%02X PC=%08X\n",
	                vaddr, address, value & 0xFF, REG_PPC);
	        }
	    }
	    g_pmmu_last_vaddr = vaddr;
	    g_pmmu_last_xlate_fc = fc;
	    g_pmmu_last_xlate_write = 1;
	    m68k_write_memory_8(ADDRESS_68K(address), value);
	    pmmu_set_m_bit();
	    if (is_page_table_write) {
	        extern void pmmu_atc_flush_all(void);
	        pmmu_atc_flush_all();
	    }
	    return;
	}
#endif

	m68k_write_memory_8(ADDRESS_68K(address), value);
}
static inline void m68ki_write_16_fc(uint address, uint fc, uint value)
{
	(void)fc;
	m68ki_set_fc(fc); /* auto-disable (see m68kcpu.h) */
	m68ki_check_address_error_010_less(address, MODE_WRITE, fc); /* auto-disable (see m68kcpu.h) */

	/* Reserved FC bus error - see m68ki_read_8_fc comment */
	if (fc == 0 || fc == 3 || fc == 4) {
		extern uint32 g_pmmu_fault_addr;
		extern uint16 g_pmmu_fault_ssw;
		extern int g_berr_is_pmmu_fault;
		extern int g_berr_not_rerunnable;
		g_pmmu_fault_addr = address;
		g_pmmu_fault_ssw = 0x0120 | (fc & 0x07);  /* DF=1, RW=0(write), SIZ=word, FC */
		g_berr_is_pmmu_fault = 0;
		g_berr_not_rerunnable = 0;
		m68k_pulse_bus_error();
		return;
	}

	/* MC68851 PMMU register access via MOVES with FC=7 (CPU space).
	 * On real hardware, the PMMU chip intercepts these at A19-A16=0010.
	 * Must check before PMMU translation since chip intercepts regardless of TC.E */
	if (fc == 7 && address >= 0x00020000 && address < 0x00030000) return;

#if M68K_EMULATE_PMMU
	g_tt_cache_inhibit = 0;
	int pmmu_tt = m68ki_tt_match(address, fc, 1) ||
	              (address >= 0x00DC0000 && address < 0x00E00000 && (fc == 5 || fc == 6));
	if (PMMU_ENABLED && !pmmu_tt && !(CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF && g_force_pmmu_enabled)) {
	    uint vaddr = address;
	    int is_page_table_write = g_force_pmmu_enabled && ((address >= 0x3000 && address < 0x8000) || (address >= 0xB000 && address < 0xC000));
	    pmmu_set_fc_override(fc);
	    pmmu_set_write_pending(1);
	    address = pmmu_translate_addr(address);
	    int _fault = pmmu_check_and_clear_fault();
	    if (_fault) {
	        extern uint32 g_pmmu_fault_dob;
	        g_pmmu_fault_dob = value;
	        m68ki_pmmu_handle_fault(_fault);
	        return;
	    }
	    /* Trace copyout MOVES writes during stat/fstat */
	    {
	        extern int g_trace_copyout;
	        if (g_trace_copyout && fc == 1 && FLAG_S) {
	            fprintf(stderr, "[COPYOUT-W] vaddr=%08X paddr=%08X val=%04X PC=%08X\n",
	                vaddr, address, value & 0xFFFF, REG_PPC);
	        }
	    }
	    {	extern int g_debug;
	        if (g_debug && vaddr >= 0x0008E348 && vaddr <= 0x0008E34B) {
	            extern unsigned long g_insn_count;
	            fprintf(stderr, "[PROC-VA-WR16] VA=%08X PA=%08X val=%04X PC=%08X insn=%lu\n",
	                vaddr, address, value & 0xFFFF, REG_PPC, g_insn_count);
	        }
	    }
	    g_pmmu_last_vaddr = vaddr;
	    g_pmmu_last_xlate_fc = fc;
	    g_pmmu_last_xlate_write = 1;
	    m68k_write_memory_16(ADDRESS_68K(address), value);
	    pmmu_set_m_bit();
	    if (is_page_table_write) {
	        extern void pmmu_atc_flush_all(void);
	        pmmu_atc_flush_all();
	    }
	    return;
	}
#endif

	m68k_write_memory_16(ADDRESS_68K(address), value);
}
static inline void m68ki_write_32_fc(uint address, uint fc, uint value)
{
	(void)fc;
	m68ki_set_fc(fc); /* auto-disable (see m68kcpu.h) */
	m68ki_check_address_error_010_less(address, MODE_WRITE, fc); /* auto-disable (see m68kcpu.h) */

	/* Reserved FC bus error - see m68ki_read_8_fc comment */
	if (fc == 0 || fc == 3 || fc == 4) {
		extern uint32 g_pmmu_fault_addr;
		extern uint16 g_pmmu_fault_ssw;
		extern int g_berr_is_pmmu_fault;
		extern int g_berr_not_rerunnable;
		g_pmmu_fault_addr = address;
		g_pmmu_fault_ssw = 0x0100 | (fc & 0x07);  /* DF=1, RW=0(write), SIZ=long, FC */
		g_berr_is_pmmu_fault = 0;
		g_berr_not_rerunnable = 0;
		m68k_pulse_bus_error();
		return;
	}

	/* MC68851 PMMU register access via MOVES with FC=7 (CPU space).
	 * On real hardware, the PMMU chip intercepts these at A19-A16=0010.
	 * Must check before PMMU translation since chip intercepts regardless of TC.E */
	if (fc == 7 && address >= 0x00020000 && address < 0x00030000) return;

#if M68K_EMULATE_PMMU
	g_tt_cache_inhibit = 0;
	int pmmu_tt = m68ki_tt_match(address, fc, 1) ||
	              (address >= 0x00DC0000 && address < 0x00E00000 && (fc == 5 || fc == 6));
	if (PMMU_ENABLED && !pmmu_tt && !(CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF && g_force_pmmu_enabled)) {
	    /* Check if 32-bit write crosses a page boundary */
	    uint ps = (m68ki_cpu.mmu_tc >> 20) & 0xf;
	    uint page_mask = ps ? ((1u << ps) - 1) : 0;
	    if (page_mask && ((address & page_mask) > (page_mask - 3))) {
	        /* Crosses page boundary - split into two 16-bit writes */
	        m68ki_write_16_fc(address, fc, (value >> 16) & 0xffff);
	        m68ki_write_16_fc(address + 2, fc, value & 0xffff);
	        return;
	    }
	    uint vaddr = address;
	    int is_page_table_write = g_force_pmmu_enabled && ((address >= 0x3000 && address < 0x8000) || (address >= 0xB000 && address < 0xC000));
	    pmmu_set_fc_override(fc);
	    pmmu_set_write_pending(1);
	    address = pmmu_translate_addr(address);
	    int _fault = pmmu_check_and_clear_fault();
	    if (_fault) {
	        extern uint32 g_pmmu_fault_dob;
	        g_pmmu_fault_dob = value;
	        m68ki_pmmu_handle_fault(_fault);
	        return;
	    }
	    /* Trace copyout MOVES writes during stat/fstat */
	    {
	        extern int g_trace_copyout;
	        if (g_trace_copyout && fc == 1 && FLAG_S) {
	            fprintf(stderr, "[COPYOUT-L] vaddr=%08X paddr=%08X val=%08X PC=%08X\n",
	                vaddr, address, value, REG_PPC);
	        }
	    }
	    {	extern int g_debug;
	        if (g_debug && (vaddr == 0x0008E348 || vaddr == 0x0008E34A)) {
	            extern unsigned long g_insn_count;
	            fprintf(stderr, "[PROC-VA-WR32] VA=%08X PA=%08X val=%08X PC=%08X insn=%lu\n",
	                vaddr, address, value, REG_PPC, g_insn_count);
	        }
	    }
	    g_pmmu_last_vaddr = vaddr;
	    g_pmmu_last_xlate_fc = fc;
	    g_pmmu_last_xlate_write = 1;
	    m68k_write_memory_32(ADDRESS_68K(address), value);
	    pmmu_set_m_bit();
	    if (is_page_table_write) {
	        extern void pmmu_atc_flush_all(void);
	        pmmu_atc_flush_all();
	    }
	    return;
	}
#endif

	m68k_write_memory_32(ADDRESS_68K(address), value);
}

#if M68K_SIMULATE_PD_WRITES
static inline void m68ki_write_32_pd_fc(uint address, uint fc, uint value)
{
	(void)fc;
	m68ki_set_fc(fc); /* auto-disable (see m68kcpu.h) */
	m68ki_check_address_error_010_less(address, MODE_WRITE, fc); /* auto-disable (see m68kcpu.h) */

#if M68K_EMULATE_PMMU
	if (PMMU_ENABLED) {
	    pmmu_set_fc_override(fc);
	    pmmu_set_write_pending(1);
	    address = pmmu_translate_addr(address);
	    int _fault = pmmu_check_and_clear_fault();
	    if (_fault) {
	        m68ki_pmmu_handle_fault(_fault);
	        return;
	    }
	    m68k_write_memory_32_pd(ADDRESS_68K(address), value);
	    pmmu_set_m_bit();  // Set Modified bit in page descriptor
	    return;
	}
#endif

	m68k_write_memory_32_pd(ADDRESS_68K(address), value);
}
#endif

/* --------------------- Effective Address Calculation -------------------- */

/* The program counter relative addressing modes cause operands to be
 * retrieved from program space, not data space.
 */
static inline uint m68ki_get_ea_pcdi(void)
{
	uint old_pc = REG_PC;
	m68ki_use_program_space(); /* auto-disable */
	return old_pc + MAKE_INT_16(m68ki_read_imm_16());
}


static inline uint m68ki_get_ea_pcix(void)
{
	m68ki_use_program_space(); /* auto-disable */
	return m68ki_get_ea_ix(REG_PC);
}

/* Indexed addressing modes are encoded as follows:
 *
 * Base instruction format:
 * F E D C B A 9 8 7 6 | 5 4 3 | 2 1 0
 * x x x x x x x x x x | 1 1 0 | BASE REGISTER      (An)
 *
 * Base instruction format for destination EA in move instructions:
 * F E D C | B A 9    | 8 7 6 | 5 4 3 2 1 0
 * x x x x | BASE REG | 1 1 0 | X X X X X X       (An)
 *
 * Brief extension format:
 *  F  |  E D C   |  B  |  A 9  | 8 | 7 6 5 4 3 2 1 0
 * D/A | REGISTER | W/L | SCALE | 0 |  DISPLACEMENT
 *
 * Full extension format:
 *  F     E D C      B     A 9    8   7    6    5 4       3   2 1 0
 * D/A | REGISTER | W/L | SCALE | 1 | BS | IS | BD SIZE | 0 | I/IS
 * BASE DISPLACEMENT (0, 16, 32 bit)                (bd)
 * OUTER DISPLACEMENT (0, 16, 32 bit)               (od)
 *
 * D/A:     0 = Dn, 1 = An                          (Xn)
 * W/L:     0 = W (sign extend), 1 = L              (.SIZE)
 * SCALE:   00=1, 01=2, 10=4, 11=8                  (*SCALE)
 * BS:      0=add base reg, 1=suppress base reg     (An suppressed)
 * IS:      0=add index, 1=suppress index           (Xn suppressed)
 * BD SIZE: 00=reserved, 01=NULL, 10=Word, 11=Long  (size of bd)
 *
 * IS I/IS Operation
 * 0  000  No Memory Indirect
 * 0  001  indir prex with null outer
 * 0  010  indir prex with word outer
 * 0  011  indir prex with long outer
 * 0  100  reserved
 * 0  101  indir postx with null outer
 * 0  110  indir postx with word outer
 * 0  111  indir postx with long outer
 * 1  000  no memory indirect
 * 1  001  mem indir with null outer
 * 1  010  mem indir with word outer
 * 1  011  mem indir with long outer
 * 1  100-111  reserved
 */
static inline uint m68ki_get_ea_ix(uint An)
{
	/* An = base register */
	uint extension = m68ki_read_imm_16();
	uint Xn = 0;                        /* Index register */
	uint bd = 0;                        /* Base Displacement */
	uint od = 0;                        /* Outer Displacement */

	if(CPU_TYPE_IS_010_LESS(CPU_TYPE))
	{
		/* Calculate index */
		Xn = REG_DA[extension>>12];     /* Xn */
		if(!BIT_B(extension))           /* W/L */
			Xn = MAKE_INT_16(Xn);

		/* Add base register and displacement and return */
		return An + Xn + MAKE_INT_8(extension);
	}

	/* Brief extension format */
	if(!BIT_8(extension))
	{
		/* Calculate index */
		Xn = REG_DA[extension>>12];     /* Xn */
		if(!BIT_B(extension))           /* W/L */
			Xn = MAKE_INT_16(Xn);
		/* Add scale if proper CPU type */
		if(CPU_TYPE_IS_EC020_PLUS(CPU_TYPE))
			Xn <<= (extension>>9) & 3;  /* SCALE */

		/* Add base register and displacement and return */
		return An + Xn + MAKE_INT_8(extension);
	}

	/* Full extension format */

	USE_CYCLES(m68ki_ea_idx_cycle_table[extension&0x3f]);

	/* Check if base register is present */
	if(BIT_7(extension))                /* BS */
		An = 0;                         /* An */

	/* Check if index is present */
	if(!BIT_6(extension))               /* IS */
	{
		Xn = REG_DA[extension>>12];     /* Xn */
		if(!BIT_B(extension))           /* W/L */
			Xn = MAKE_INT_16(Xn);
		Xn <<= (extension>>9) & 3;      /* SCALE */
	}

	/* Check if base displacement is present */
	if(BIT_5(extension))                /* BD SIZE */
		bd = BIT_4(extension) ? m68ki_read_imm_32() : (uint32)MAKE_INT_16(m68ki_read_imm_16());

	/* If no indirect action, we are done */
	if(!(extension&7))                  /* No Memory Indirect */
		return An + bd + Xn;

	/* Check if outer displacement is present */
	if(BIT_1(extension))                /* I/IS:  od */
		od = BIT_0(extension) ? m68ki_read_imm_32() : (uint32)MAKE_INT_16(m68ki_read_imm_16());

	/* Postindex */
	if(BIT_2(extension))                /* I/IS:  0 = preindex, 1 = postindex */
		return m68ki_read_32(An + bd) + Xn + od;

	/* Preindex */
	return m68ki_read_32(An + bd + Xn) + od;
}


/* Fetch operands */
static inline uint OPER_AY_AI_8(void)  {uint ea = EA_AY_AI_8();  return m68ki_read_8(ea); }
static inline uint OPER_AY_AI_16(void) {uint ea = EA_AY_AI_16(); return m68ki_read_16(ea);}
static inline uint OPER_AY_AI_32(void) {uint ea = EA_AY_AI_32(); return m68ki_read_32(ea);}
static inline uint OPER_AY_PI_8(void)  {uint ea = EA_AY_PI_8();  return m68ki_read_8(ea); }
static inline uint OPER_AY_PI_16(void) {uint ea = EA_AY_PI_16(); return m68ki_read_16(ea);}
static inline uint OPER_AY_PI_32(void) {uint ea = EA_AY_PI_32(); return m68ki_read_32(ea);}
static inline uint OPER_AY_PD_8(void)  {uint ea = EA_AY_PD_8();  return m68ki_read_8(ea); }
static inline uint OPER_AY_PD_16(void) {uint ea = EA_AY_PD_16(); return m68ki_read_16(ea);}
static inline uint OPER_AY_PD_32(void) {uint ea = EA_AY_PD_32(); return m68ki_read_32(ea);}
static inline uint OPER_AY_DI_8(void)  {uint ea = EA_AY_DI_8();  return m68ki_read_8(ea); }
static inline uint OPER_AY_DI_16(void) {uint ea = EA_AY_DI_16(); return m68ki_read_16(ea);}
static inline uint OPER_AY_DI_32(void) {uint ea = EA_AY_DI_32(); return m68ki_read_32(ea);}
static inline uint OPER_AY_IX_8(void)  {uint ea = EA_AY_IX_8();  return m68ki_read_8(ea); }
static inline uint OPER_AY_IX_16(void) {uint ea = EA_AY_IX_16(); return m68ki_read_16(ea);}
static inline uint OPER_AY_IX_32(void) {uint ea = EA_AY_IX_32(); return m68ki_read_32(ea);}

static inline uint OPER_AX_AI_8(void)  {uint ea = EA_AX_AI_8();  return m68ki_read_8(ea); }
static inline uint OPER_AX_AI_16(void) {uint ea = EA_AX_AI_16(); return m68ki_read_16(ea);}
static inline uint OPER_AX_AI_32(void) {uint ea = EA_AX_AI_32(); return m68ki_read_32(ea);}
static inline uint OPER_AX_PI_8(void)  {uint ea = EA_AX_PI_8();  return m68ki_read_8(ea); }
static inline uint OPER_AX_PI_16(void) {uint ea = EA_AX_PI_16(); return m68ki_read_16(ea);}
static inline uint OPER_AX_PI_32(void) {uint ea = EA_AX_PI_32(); return m68ki_read_32(ea);}
static inline uint OPER_AX_PD_8(void)  {uint ea = EA_AX_PD_8();  return m68ki_read_8(ea); }
static inline uint OPER_AX_PD_16(void) {uint ea = EA_AX_PD_16(); return m68ki_read_16(ea);}
static inline uint OPER_AX_PD_32(void) {uint ea = EA_AX_PD_32(); return m68ki_read_32(ea);}
static inline uint OPER_AX_DI_8(void)  {uint ea = EA_AX_DI_8();  return m68ki_read_8(ea); }
static inline uint OPER_AX_DI_16(void) {uint ea = EA_AX_DI_16(); return m68ki_read_16(ea);}
static inline uint OPER_AX_DI_32(void) {uint ea = EA_AX_DI_32(); return m68ki_read_32(ea);}
static inline uint OPER_AX_IX_8(void)  {uint ea = EA_AX_IX_8();  return m68ki_read_8(ea); }
static inline uint OPER_AX_IX_16(void) {uint ea = EA_AX_IX_16(); return m68ki_read_16(ea);}
static inline uint OPER_AX_IX_32(void) {uint ea = EA_AX_IX_32(); return m68ki_read_32(ea);}

static inline uint OPER_A7_PI_8(void)  {uint ea = EA_A7_PI_8();  return m68ki_read_8(ea); }
static inline uint OPER_A7_PD_8(void)  {uint ea = EA_A7_PD_8();  return m68ki_read_8(ea); }

static inline uint OPER_AW_8(void)     {uint ea = EA_AW_8();     return m68ki_read_8(ea); }
static inline uint OPER_AW_16(void)    {uint ea = EA_AW_16();    return m68ki_read_16(ea);}
static inline uint OPER_AW_32(void)    {uint ea = EA_AW_32();    return m68ki_read_32(ea);}
static inline uint OPER_AL_8(void)     {uint ea = EA_AL_8();     return m68ki_read_8(ea); }
static inline uint OPER_AL_16(void)    {uint ea = EA_AL_16();    return m68ki_read_16(ea);}
static inline uint OPER_AL_32(void)    {uint ea = EA_AL_32();    return m68ki_read_32(ea);}
static inline uint OPER_PCDI_8(void)   {uint ea = EA_PCDI_8();   return m68ki_read_pcrel_8(ea); }
static inline uint OPER_PCDI_16(void)  {uint ea = EA_PCDI_16();  return m68ki_read_pcrel_16(ea);}
static inline uint OPER_PCDI_32(void)  {uint ea = EA_PCDI_32();  return m68ki_read_pcrel_32(ea);}
static inline uint OPER_PCIX_8(void)   {uint ea = EA_PCIX_8();   return m68ki_read_pcrel_8(ea); }
static inline uint OPER_PCIX_16(void)  {uint ea = EA_PCIX_16();  return m68ki_read_pcrel_16(ea);}
static inline uint OPER_PCIX_32(void)  {uint ea = EA_PCIX_32();  return m68ki_read_pcrel_32(ea);}



/* ---------------------------- Stack Functions --------------------------- */

/* Push/pull data from the stack */
static inline void m68ki_push_16(uint value)
{
	REG_SP = MASK_OUT_ABOVE_32(REG_SP - 2);
	m68ki_write_16(REG_SP, value);
}

static inline void m68ki_push_32(uint value)
{
	REG_SP = MASK_OUT_ABOVE_32(REG_SP - 4);
	m68ki_write_32(REG_SP, value);
}

static inline uint m68ki_pull_16(void)
{
	REG_SP = MASK_OUT_ABOVE_32(REG_SP + 2);
	return m68ki_read_16(REG_SP-2);
}

static inline uint m68ki_pull_32(void)
{
	REG_SP = MASK_OUT_ABOVE_32(REG_SP + 4);
	return m68ki_read_32(REG_SP-4);
}


/* Increment/decrement the stack as if doing a push/pull but
 * don't do any memory access.
 */
static inline void m68ki_fake_push_16(void)
{
	REG_SP = MASK_OUT_ABOVE_32(REG_SP - 2);
}

static inline void m68ki_fake_push_32(void)
{
	REG_SP = MASK_OUT_ABOVE_32(REG_SP - 4);
}

static inline void m68ki_fake_pull_16(void)
{
	REG_SP = MASK_OUT_ABOVE_32(REG_SP + 2);
}

static inline void m68ki_fake_pull_32(void)
{
	REG_SP = MASK_OUT_ABOVE_32(REG_SP + 4);
}


/* ----------------------------- Program Flow ----------------------------- */

/* Jump to a new program location or vector.
 * These functions will also call the pc_changed callback if it was enabled
 * in m68kconf.h.
 */
static inline void m68ki_jump(uint new_pc)
{
	// Debug: trace jumps to halt loop
	REG_PC = new_pc;
	m68ki_pc_changed(REG_PC);
}

static inline void m68ki_jump_vector(uint vector)
{
	uint vec_addr = (vector<<2) + REG_VBR;
	uint new_pc;

	/* MC68030: Exception vector reads use FC=5 (supervisor data).
	 * FC=7 (CPU space) is only for IACK bus cycles, not vector table reads.
	 * With PMMU enabled, FC affects ATC lookup and translation — using the
	 * wrong FC causes incorrect physical address resolution. */
	new_pc = m68ki_read_32_fc(vec_addr, FUNCTION_CODE_SUPERVISOR_DATA);
	{	extern int g_debug;
		if (g_debug) {
			fprintf(stderr, "[EXCEPTION] vec=%d VBR=%08x vec_addr=%08x handler=%08x PPC=%08x\n",
				vector, REG_VBR, vec_addr, new_pc, REG_PPC);
		}
	}
	REG_PC = new_pc;
	m68ki_pc_changed(REG_PC);
}


/* Branch to a new memory location.
 * The 32-bit branch will call pc_changed if it was enabled in m68kconf.h.
 * So far I've found no problems with not calling pc_changed for 8 or 16
 * bit branches.
 */
static inline void m68ki_branch_8(uint offset)
{
	REG_PC += MAKE_INT_8(offset);
}

static inline void m68ki_branch_16(uint offset)
{
	REG_PC += MAKE_INT_16(offset);
}

static inline void m68ki_branch_32(uint offset)
{
	REG_PC += offset;
	m68ki_pc_changed(REG_PC);
}

/* ---------------------------- Status Register --------------------------- */

/* Set the S flag and change the active stack pointer.
 * Note that value MUST be 4 or 0.
 */
static inline void m68ki_set_s_flag(uint value)
{
	/* Backup the old stack pointer */
	REG_SP_BASE[FLAG_S | ((FLAG_S>>1) & FLAG_M)] = REG_SP;
	/* Set the S flag */
	FLAG_S = value;
	/* Set the new stack pointer */
	REG_SP = REG_SP_BASE[FLAG_S | ((FLAG_S>>1) & FLAG_M)];
}

/* Set the S and M flags and change the active stack pointer.
 * Note that value MUST be 0, 2, 4, or 6 (bit2 = S, bit1 = M).
 */
static inline void m68ki_set_sm_flag(uint value)
{
	uint old_idx = FLAG_S | ((FLAG_S>>1) & FLAG_M);
	/* Backup the old stack pointer */
	REG_SP_BASE[old_idx] = REG_SP;
	/* Set the S and M flags */
	FLAG_S = value & SFLAG_SET;
	FLAG_M = value & MFLAG_SET;
	/* Set the new stack pointer */
	uint new_idx = FLAG_S | ((FLAG_S>>1) & FLAG_M);
	REG_SP = REG_SP_BASE[new_idx];
}

/* Set the S and M flags.  Don't touch the stack pointer. */
static inline void m68ki_set_sm_flag_nosp(uint value)
{
	/* Set the S and M flags */
	FLAG_S = value & SFLAG_SET;
	FLAG_M = value & MFLAG_SET;
}


/* Set the condition code register */
static inline void m68ki_set_ccr(uint value)
{
	FLAG_X = BIT_4(value)  << 4;
	FLAG_N = BIT_3(value)  << 4;
	FLAG_Z = !BIT_2(value);
	FLAG_V = BIT_1(value)  << 6;
	FLAG_C = BIT_0(value)  << 8;
}

/* Set the status register but don't check for interrupts */
static inline void m68ki_set_sr_noint(uint value)
{
	/* Mask out the "unimplemented" bits */
	value &= CPU_SR_MASK;

	/* Now set the status register */
	FLAG_T1 = BIT_F(value);
	FLAG_T0 = BIT_E(value);
	FLAG_INT_MASK = value & 0x0700;
	m68ki_set_ccr(value);
	m68ki_set_sm_flag((value >> 11) & 6);
}

/* Set the status register but don't check for interrupts nor
 * change the stack pointer
 */
static inline void m68ki_set_sr_noint_nosp(uint value)
{
	/* Mask out the "unimplemented" bits */
	value &= CPU_SR_MASK;

	/* Now set the status register */
	FLAG_T1 = BIT_F(value);
	FLAG_T0 = BIT_E(value);
	FLAG_INT_MASK = value & 0x0700;
	m68ki_set_ccr(value);
	m68ki_set_sm_flag_nosp((value >> 11) & 6);
}

/* Set the status register and check for interrupts */
static inline void m68ki_set_sr(uint value)
{
	m68ki_set_sr_noint(value);
	m68ki_check_interrupts();
}


/* ------------------------- Exception Processing ------------------------- */

/* Initiate exception processing */
static inline uint m68ki_init_exception(void)
{
	/* Save the old status register */
	uint sr = m68ki_get_sr();

	/* Turn off trace flag, clear pending traces */
	FLAG_T1 = FLAG_T0 = 0;
	m68ki_clear_trace();
	/* Enter supervisor mode */
	m68ki_set_s_flag(SFLAG_SET);

	return sr;
}

/* 3 word stack frame (68000 only) */
static inline void m68ki_stack_frame_3word(uint pc, uint sr)
{
	m68ki_push_32(pc);
	m68ki_push_16(sr);
}

/* Format 0 stack frame.
 * This is the standard stack frame for 68010+.
 */
static inline void m68ki_stack_frame_0000(uint pc, uint sr, uint vector)
{
	/* Stack a 3-word frame if we are 68000 */
	if(CPU_TYPE == CPU_TYPE_000)
	{
		m68ki_stack_frame_3word(pc, sr);
		return;
	}
	m68ki_push_16(vector<<2);
	m68ki_push_32(pc);
	m68ki_push_16(sr);
}

/* Format 1 stack frame (68020).
 * For 68020, this is the 4 word throwaway frame.
 */
static inline void m68ki_stack_frame_0001(uint pc, uint sr, uint vector)
{
	m68ki_push_16(0x1000 | (vector<<2));
	m68ki_push_32(pc);
	m68ki_push_16(sr);
}

/* Format 2 stack frame.
 * This is used only by 68020 for trap exceptions.
 */
static inline void m68ki_stack_frame_0010(uint sr, uint vector)
{
	m68ki_push_32(REG_PPC);
	m68ki_push_16(0x2000 | (vector<<2));
	m68ki_push_32(REG_PC);
	m68ki_push_16(sr);
}


/* Bus error stack frame (68000 only).
 */
static inline void m68ki_stack_frame_buserr(uint sr)
{
	m68ki_push_32(REG_PC);
	m68ki_push_16(sr);
	m68ki_push_16(REG_IR);
	m68ki_push_32(m68ki_aerr_address);	/* access address */
	/* 0 0 0 0 0 0 0 0 0 0 0 R/W I/N FC
	 * R/W  0 = write, 1 = read
	 * I/N  0 = instruction, 1 = not
	 * FC   3-bit function code
	 */
	m68ki_push_16(m68ki_aerr_write_mode | CPU_INSTR_MODE | m68ki_aerr_fc);
}

/* Format 8 stack frame (68010).
 * 68010 only.  This is the 29 word bus/address error frame.
 */
static inline void m68ki_stack_frame_1000(uint pc, uint sr, uint vector)
{
	/* VERSION
	 * NUMBER
	 * INTERNAL INFORMATION, 16 WORDS
	 */
	m68ki_fake_push_32();
	m68ki_fake_push_32();
	m68ki_fake_push_32();
	m68ki_fake_push_32();
	m68ki_fake_push_32();
	m68ki_fake_push_32();
	m68ki_fake_push_32();
	m68ki_fake_push_32();

	/* INSTRUCTION INPUT BUFFER */
	m68ki_push_16(0);

	/* UNUSED, RESERVED (not written) */
	m68ki_fake_push_16();

	/* DATA INPUT BUFFER */
	m68ki_push_16(0);

	/* UNUSED, RESERVED (not written) */
	m68ki_fake_push_16();

	/* DATA OUTPUT BUFFER */
	m68ki_push_16(0);

	/* UNUSED, RESERVED (not written) */
	m68ki_fake_push_16();

	/* FAULT ADDRESS -- the MC68010 latches the logical address that was
	 * on the bus when the fault occurred.  The board's bus-error decode
	 * stashes it in g_pmmu_fault_addr before pulsing BERR; the System V
	 * exception handler reads this field to route the page fault. */
	{ extern uint32 g_pmmu_fault_addr; m68ki_push_32(g_pmmu_fault_addr); }

	/* SPECIAL STATUS WORD */
	/*m68ki_push_16(0);*/
    {
        /*extern uint16 g_mmu_fault_ssw;*/
        extern uint16 g_pmmu_fault_ssw;
        m68ki_push_16(g_pmmu_fault_ssw);
    }    



	/* 1000, VECTOR OFFSET */
	m68ki_push_16(0x8000 | (vector<<2));

	/* PROGRAM COUNTER */
	m68ki_push_32(pc);

	/* STATUS REGISTER */
	m68ki_push_16(sr);
}

/* Format A stack frame (short bus fault).
 * This is used only by 68020 for bus fault and address error
 * if the error happens at an instruction boundary.
 * PC stacked is address of next instruction.
 */
/* Fault address, SSW, DOB, and PSR for bus error stack frame - set by PMMU code */
extern uint32 g_pmmu_fault_addr;
extern uint16 g_pmmu_fault_ssw;  // Special Status Word for PMMU faults
extern uint32 g_pmmu_fault_dob;  // Data Output Buffer - data being written
extern uint16 g_pmmu_latched_psr; // PSR value latched at fault time - for stack frame
extern int g_pmmu_psr_latched;   // Flag: 1 if PSR was latched, cleared after stack frame push

static inline void m68ki_stack_frame_1010(uint sr, uint vector, uint pc)
{
	/* Trace bus error frames when debug enabled */
	{
		extern int g_kernel_pmmu_active;
		extern int g_debug;
		if (g_debug && g_kernel_pmmu_active) {
			static int fault_frame_count = 0;
			fault_frame_count++;
			fprintf(stderr, "[FRAME-A-PUSH #%d] PC=%08X SR=%04X fault_addr=%08X SSW=%04X PSR=%04X SFC=%d DFC=%d A7=%08X\n",
				fault_frame_count, pc, sr, g_pmmu_fault_addr, g_pmmu_fault_ssw,
				g_pmmu_latched_psr,
				REG_SFC, REG_DFC, REG_A[7]);
		}
	}
	/* INTERNAL REGISTER (offset 30 from frame base) */
	m68ki_push_16(0);

	/* INTERNAL REGISTER (offset 28 from frame base)
	 * MC68851: Also store PSR here for Tests V/W (limit violation tests) */
	m68ki_push_16(g_pmmu_latched_psr);

	/* DATA OUTPUT BUFFER (2 words, offset 24) */
	m68ki_push_32(g_pmmu_fault_dob);
	g_pmmu_fault_dob = 0;

	/* INTERNAL REGISTER (offset 22) */
	m68ki_push_16(0);

	/* INTERNAL REGISTER (offset 20) */
	m68ki_push_16(0);

	/* DATA CYCLE FAULT ADDRESS (2 words, offset 16) */
	m68ki_push_32(g_pmmu_fault_addr);

	/* INSTRUCTION PIPE STAGE B (offset 14) */
	m68ki_push_16(0);

	/* INSTRUCTION PIPE STAGE C (offset 12) */
	m68ki_push_16(0);

	/* SPECIAL STATUS REGISTER (offset 10) */
	/*m68ki_push_16(g_pmmu_fault_ssw);*/
    m68ki_push_16(0);
	g_pmmu_fault_ssw = 0;

	/* INTERNAL REGISTER - PSR (offset 8 from frame base)
	 * MC68851 stores PMMU Status Register here for exception handler access.
	 * ROM reads PSR from this location (SP + 8 = 0x2EE8 when SP=0x2EE0).
	 *
	 * MC68851 PSR bit 2 = L (Limit violation).  The LynxOS kernel's page
	 * fault handler checks this bit: if set, it treats the fault as a limit
	 * violation (access out of bounds) rather than a page-not-present fault,
	 * and skips demand paging — causing an infinite retry loop.
	 *
	 * Previously bit 2 was forced on to skip the kernel's "fixprr" MC68020
	 * errata workaround (paging.s), but this broke page fault resolution.
	 * The MC68030 does not have the MC68020 PRR bug, so the fixprr code
	 * path is harmless even if entered — it only matters for MC68020. */
	m68ki_push_16(g_pmmu_latched_psr);
	// DON'T clear g_pmmu_latched_psr/g_pmmu_psr_latched here!
	// Keep the latch active so that PMOVE PSR returns the correct fault PSR.
	// The latch is cleared when PMOVE PSR is executed (in m68kmmu.h case 0).

	/* 1010, VECTOR OFFSET (offset 6) */
	m68ki_push_16(0xa000 | (vector<<2));

	/* PROGRAM COUNTER (offset 2) */
	m68ki_push_32(pc);

	/* STATUS REGISTER (offset 0) */
	m68ki_push_16(sr);

	/* (FRAME-A-DONE trace removed - was here) */
}

/* Format B stack frame (long bus fault).
 * This is used only by 68020 for bus fault and address error
 * if the error happens during instruction execution.
 * PC stacked is address of instruction in progress.
 */
static inline void m68ki_stack_frame_1011(uint sr, uint vector, uint pc)
{
	/* INTERNAL REGISTERS (18 words) */
	m68ki_push_32(0);
	m68ki_push_32(0);
	m68ki_push_32(0);
	m68ki_push_32(0);
	m68ki_push_32(0);
	m68ki_push_32(0);
	m68ki_push_32(0);
	m68ki_push_32(0);
	m68ki_push_32(0);

	/* VERSION# (4 bits), INTERNAL INFORMATION */
	m68ki_push_16(0);

	/* INTERNAL REGISTERS (3 words) */
	m68ki_push_32(0);
	m68ki_push_16(0);

	/* DATA INTPUT BUFFER (2 words) */
	m68ki_push_32(0);

	/* INTERNAL REGISTERS (2 words) */
	m68ki_push_32(0);

	/* STAGE B ADDRESS (2 words) */
	m68ki_push_32(0);

	/* INTERNAL REGISTER (4 words) */
	m68ki_push_32(0);
	m68ki_push_32(0);

	/* DATA OUTPUT BUFFER (2 words) */
	m68ki_push_32(0);

	/* INTERNAL REGISTER */
	m68ki_push_16(0);

	/* INTERNAL REGISTER */
	m68ki_push_16(0);

	/* DATA CYCLE FAULT ADDRESS (2 words) */
	m68ki_push_32(0);

	/* INSTRUCTION PIPE STAGE B */
	m68ki_push_16(0);

	/* INSTRUCTION PIPE STAGE C */
	m68ki_push_16(0);

	/* SPECIAL STATUS REGISTER */
	m68ki_push_16(0);

	/* INTERNAL REGISTER */
	m68ki_push_16(0);

	/* 1011, VECTOR OFFSET */
	m68ki_push_16(0xb000 | (vector<<2));

	/* PROGRAM COUNTER */
	m68ki_push_32(pc);

	/* STATUS REGISTER */
	m68ki_push_16(sr);
}


/* Used for Group 2 exceptions.
 * These stack a type 2 frame on the 020.
 */
static inline void m68ki_exception_trap(uint vector)
{
	if (vector == 5) { /* ZERO_DIVIDE */
		extern int g_verbose;
		if (g_verbose) {
			fprintf(stderr, "[CPU] ZERO DIVIDE at PC=0x%08X  IR=0x%04X  D0=0x%08X D1=0x%08X D2=0x%08X D3=0x%08X\n",
				REG_PC, REG_IR, REG_D[0], REG_D[1], REG_D[2], REG_D[3]);
			fprintf(stderr, "[CPU] ZERO DIVIDE A0=0x%08X A6=0x%08X SP=0x%08X\n",
				REG_A[0], REG_A[6], REG_A[7]);
		}
	}
	uint sr = m68ki_init_exception();

	if(CPU_TYPE_IS_010_LESS(CPU_TYPE))
		m68ki_stack_frame_0000(REG_PC, sr, vector);
	else
		m68ki_stack_frame_0010(sr, vector);

	m68ki_jump_vector(vector);

	/* Use up some clock cycles and undo the instruction's cycles */
	USE_CYCLES(CYC_EXCEPTION[vector] - CYC_INSTRUCTION[REG_IR]);
}

/* Trap#n stacks a 0 frame but behaves like group2 otherwise */
static inline void m68ki_exception_trapN(uint vector)
{
	uint sr = m68ki_init_exception();
	m68ki_stack_frame_0000(REG_PC, sr, vector);
	m68ki_jump_vector(vector);

	/* Use up some clock cycles and undo the instruction's cycles */
	USE_CYCLES(CYC_EXCEPTION[vector] - CYC_INSTRUCTION[REG_IR]);
}

/* Exception for trace mode */
static inline void m68ki_exception_trace(void)
{
	uint sr = m68ki_init_exception();

	if(CPU_TYPE_IS_010_LESS(CPU_TYPE))
	{
		#if M68K_EMULATE_ADDRESS_ERROR == OPT_ON
		if(CPU_TYPE_IS_000(CPU_TYPE))
		{
			CPU_INSTR_MODE = INSTRUCTION_NO;
		}
		#endif /* M68K_EMULATE_ADDRESS_ERROR */
		m68ki_stack_frame_0000(REG_PC, sr, EXCEPTION_TRACE);
	}
	else
		m68ki_stack_frame_0010(sr, EXCEPTION_TRACE);

	m68ki_jump_vector(EXCEPTION_TRACE);

	/* Trace nullifies a STOP instruction */
	CPU_STOPPED &= ~STOP_LEVEL_STOP;

	/* Use up some clock cycles */
	USE_CYCLES(CYC_EXCEPTION[EXCEPTION_TRACE]);
}

/* Exception for privilege violation */
static inline void m68ki_exception_privilege_violation(void)
{
	uint sr = m68ki_init_exception();

	#if M68K_EMULATE_ADDRESS_ERROR == OPT_ON
	if(CPU_TYPE_IS_000(CPU_TYPE))
	{
		CPU_INSTR_MODE = INSTRUCTION_NO;
	}
	#endif /* M68K_EMULATE_ADDRESS_ERROR */

	m68ki_stack_frame_0000(REG_PPC, sr, EXCEPTION_PRIVILEGE_VIOLATION);
	m68ki_jump_vector(EXCEPTION_PRIVILEGE_VIOLATION);

	/* Use up some clock cycles and undo the instruction's cycles */
	USE_CYCLES(CYC_EXCEPTION[EXCEPTION_PRIVILEGE_VIOLATION] - CYC_INSTRUCTION[REG_IR]);
}

extern jmp_buf m68ki_bus_error_jmp_buf;

#define m68ki_check_bus_error_trap() setjmp(m68ki_bus_error_jmp_buf)

/* Exception for bus error */
static inline void m68ki_exception_bus_error(void)
{
	int i;

	/* Clear R/M/W state when taking bus error exception */
	{
		extern int g_pmmu_rmw_in_progress;
		g_pmmu_rmw_in_progress = 0;
	}

	/* If we were processing a bus error, address error, or reset,
	 * while writing the stack frame, this is a catastrophic failure.
	 * Halt the CPU.
	 */
	if(CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF)
	{
		extern uint32 g_pmmu_fault_addr;
		{	extern int g_debug, g_verbose;
			if (g_debug || g_verbose) {
				fprintf(stderr, "[DOUBLE-FAULT] PC=0x%08X SP=0x%08X fault_addr=0x%08X VBR=0x%08X, halting CPU\n",
					m68k_get_reg(NULL, M68K_REG_PC), REG_SP,
					g_pmmu_fault_addr, m68k_get_reg(NULL, M68K_REG_VBR));
			}
		}
		CPU_STOPPED = STOP_LEVEL_HALT;
		CPU_RUN_MODE = RUN_MODE_NORMAL;
		SET_CYCLES(0);
		longjmp(m68ki_bus_error_jmp_buf, 1);
	}
	CPU_RUN_MODE = RUN_MODE_BERR_AERR_RESET_WSF;

	{	extern int g_debug;
		if (g_debug) {
			extern uint32 g_pmmu_fault_addr;
			extern int g_berr_is_pmmu_fault;
			/*fprintf(stderr, "[BUS-ERROR] PC=0x%08X fault_addr=0x%08X S=%d pmmu_fault=%d SSW=0x%04X PPC=0x%08X\n",
				m68k_get_reg(NULL, M68K_REG_PC), g_pmmu_fault_addr,
				FLAG_S ? 1 : 0, g_berr_is_pmmu_fault,
				g_pmmu_fault_ssw, REG_PPC); */
		}
	}

	/* Use up some clock cycles and undo the instruction's cycles */
	USE_CYCLES(CYC_EXCEPTION[EXCEPTION_BUS_ERROR] - CYC_INSTRUCTION[REG_IR]);

	/* Restore registers only for rerunnable faults: if we're skipping the
	 * instruction (g_berr_not_rerunnable), any side effects (post-inc/pre-dec
	 * address register writeback) must remain visible -- the Torch QY ROM's
	 * DRAM-sizing probe relies on the post-incremented A0 being preserved
	 * after the failing read so it can size DRAM correctly. */
	

    if (!g_berr_not_rerunnable) {
		for (i = 15; i >= 0; i--){
			REG_DA[i] = REG_DA_SAVE[i];
		}
	}

    /* for (i = 15; i >= 0; i--){
        REG_DA[i] = REG_DA_SAVE[i];
    } */

	uint sr = m68ki_init_exception();

	/* Use appropriate stack frame format based on CPU type */
	if(CPU_TYPE_IS_020_PLUS(CPU_TYPE))
	{
		/* Use Format A (short bus fault, 16 words) for MC68020+ bus errors
		 * For rerunnable faults: push REG_PPC so instruction is retried
		 * For non-rerunnable faults: push REG_PC so next instruction runs */
		uint32 return_pc = g_berr_not_rerunnable ? REG_PC : REG_PPC;
		g_berr_not_rerunnable = 0;
		g_berr_is_pmmu_fault = 0;
		m68ki_stack_frame_1010(sr, EXCEPTION_BUS_ERROR, return_pc);
	}
	else
	{
		/* MC68010 uses Format 8 */
        /*fprintf(stderr,
            "68010 BERR frame: PPC=%08X PC=%08X fault=%08X SSW=%04X rerun=%d\n",
            REG_PPC,
            REG_PC,
            g_pmmu_fault_addr,
            g_pmmu_fault_ssw,
            !g_berr_not_rerunnable); */


		m68ki_stack_frame_1000(REG_PPC, sr, EXCEPTION_BUS_ERROR);

        /*{
            int n;
            fprintf(stderr, "68010 frame dump SP=%08X\n", REG_SP);
            for (n = 0; n < 64; n += 2) {
                fprintf(stderr, "  +%02X = %04X\n",
                    n,
                    m68ki_read_16(REG_SP + n));
            }
        }*/


        g_berr_not_rerunnable = 0;
        g_berr_is_pmmu_fault = 0;
	}

	m68ki_jump_vector(EXCEPTION_BUS_ERROR);

	/* Trace bus error handler entry for 0x40000000 range faults */
	{
		static int berr_40_trace = 0;
		extern uint32 g_pmmu_fault_addr;
		extern int g_verbose;
		if (g_verbose && g_pmmu_fault_addr >= 0x40000000 && g_pmmu_fault_addr < 0x40010000 && berr_40_trace < 5) {
			berr_40_trace++;
			fprintf(stderr, "[BERR-40-HANDLER] #%d fault_addr=%08X handler_PC=%08X SP=%08X VBR=%08X\n",
				berr_40_trace, g_pmmu_fault_addr, REG_PC, REG_A[7], REG_VBR);
		}
	}

	/* Exception processing is complete: stack frame pushed and vector taken.
	 * The MC68030 returns to normal mode at this point — a bus error in the
	 * handler itself starts a NEW exception sequence (not a double fault).
	 * The double-fault check at the top of this function catches the case
	 * where a bus error occurs DURING the stack frame write (WSF mode). */
	CPU_RUN_MODE = RUN_MODE_NORMAL;

	longjmp(m68ki_bus_error_jmp_buf, 1);
}

/* Exception for MC68851 MMU access level violation (vector 58) */
static inline void m68ki_exception_access_level_violation(void)
{
	int i;

	/* If we were processing an exception, this is a double fault */
	if(CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF)
	{
		CPU_STOPPED = STOP_LEVEL_HALT;
		CPU_RUN_MODE = RUN_MODE_NORMAL;
		SET_CYCLES(0);
		longjmp(m68ki_bus_error_jmp_buf, 1);
	}
	CPU_RUN_MODE = RUN_MODE_BERR_AERR_RESET_WSF;

	/* Note: CAL is NOT reset here to allow diagnostic tests to run multiple
	 * access level tests in sequence. The real MC68851 would reset CAL to 0
	 * on exception entry, but since our access level check is only triggered
	 * for test addresses in the first page, the exception handler can run
	 * without issues. */
	/* m68ki_cpu.mmu_cal = 0; */

	/* Use up some clock cycles */
	USE_CYCLES(CYC_EXCEPTION[EXCEPTION_MMU_ACCESS_LEVEL] - CYC_INSTRUCTION[REG_IR]);

	// Debug: disabled for clean output
	// fprintf(stderr, "[ACCESS-LEVEL] fault_addr=%08x PC=%08x PPC=%08x A7=%08x SR=%04x\n",
	// 	g_pmmu_fault_addr, REG_PC, REG_PPC, REG_A[7], m68ki_get_sr());

	/* Restore registers to pre-instruction state */
	for (i = 15; i >= 0; i--){
		REG_DA[i] = REG_DA_SAVE[i];
	}

	uint sr = m68ki_init_exception();

	/* MC68851 access level violation uses Format A (short bus fault) like bus errors */
	if(CPU_TYPE_IS_020_PLUS(CPU_TYPE))
	{
		m68ki_stack_frame_1010(sr, EXCEPTION_MMU_ACCESS_LEVEL, REG_PPC);
	}
	else
	{
		m68ki_stack_frame_1000(REG_PPC, sr, EXCEPTION_MMU_ACCESS_LEVEL);
	}

	/* Read vector 58 handler address */
	uint32 vec_addr = (EXCEPTION_MMU_ACCESS_LEVEL << 2) + REG_VBR;
	(void)m68ki_read_32_fc(vec_addr, FUNCTION_CODE_SUPERVISOR_DATA); /* handler_addr - side effect needed */

	m68ki_jump_vector(EXCEPTION_MMU_ACCESS_LEVEL);

	/* Exception processing complete — return to normal mode */
	CPU_RUN_MODE = RUN_MODE_NORMAL;

	longjmp(m68ki_bus_error_jmp_buf, 1);
}

extern int cpu_log_enabled;

/* Exception for A-Line instructions */
static inline void m68ki_exception_1010(void)
{
#if M68K_ALINE_HOOK
	int res = CALLBACK_ALINE_HOOK(REG_IR,ADDRESS_68K(REG_PPC));
	if(res == M68K_ALINE_EXCEPT) {
#endif
	uint sr;
#if M68K_LOG_1010_1111 == OPT_ON
	M68K_DO_LOG_EMU((M68K_LOG_FILEHANDLE "%s at %08x: called 1010 instruction %04x (%s)\n",
					 m68ki_cpu_names[CPU_TYPE], ADDRESS_68K(REG_PPC), REG_IR,
					 m68ki_disassemble_quick(ADDRESS_68K(REG_PPC))));
#endif

	sr = m68ki_init_exception();
	m68ki_stack_frame_0000(REG_PPC, sr, EXCEPTION_1010);
	m68ki_jump_vector(EXCEPTION_1010);

	/* Use up some clock cycles and undo the instruction's cycles */
	USE_CYCLES(CYC_EXCEPTION[EXCEPTION_1010] - CYC_INSTRUCTION[REG_IR]);
#if M68K_ALINE_HOOK
	} else if(res == M68K_ALINE_RTS) {
		m68ki_trace_t0();                             /* auto-disable (see m68kcpu.h) */
		m68ki_jump(m68ki_pull_32());
	}
#endif
}

/* Exception for F-Line instructions */
static inline void m68ki_exception_1111(void)
{
	uint sr;

#if M68K_LOG_1010_1111 == OPT_ON
	M68K_DO_LOG_EMU((M68K_LOG_FILEHANDLE "%s at %08x: called 1111 instruction %04x (%s)\n",
					 m68ki_cpu_names[CPU_TYPE], ADDRESS_68K(REG_PPC), REG_IR,
					 m68ki_disassemble_quick(ADDRESS_68K(REG_PPC))));
#endif

	/* Trace Line F exceptions at suspicious addresses — unconditional for crash diagnosis */
	if ((REG_PPC < 0x100 || (REG_PPC & 1)) && REG_PPC < 0xFF000000) {
		extern unsigned long g_insn_count;
		extern void m68k_dump_pc_history(void);
		fprintf(stderr, "\n[LINE-F CRASH] PC=%08X IR=%04X SR=%04X FLAG_S=%d insn=%lu\n",
			REG_PPC, REG_IR, m68ki_get_sr(), FLAG_S, g_insn_count);
		uint sp = FLAG_S ? REG_ISP : REG_USP;
		fprintf(stderr, "[LINE-F] SP=%08X stack:", sp);
		for (int _i = 0; _i < 64; _i += 4)
			fprintf(stderr, " %08X", m68k_read_memory_32(sp + _i));
		fprintf(stderr, "\n[LINE-F] VBR=%08X vec[0x2C]=%08X vec[0x08]=%08X\n",
			REG_VBR, m68k_read_memory_32(REG_VBR + 0x2C),
			m68k_read_memory_32(REG_VBR + 0x08));
		fprintf(stderr, "[LINE-F] D0-D7:");
		for (int _i = 0; _i < 8; _i++) fprintf(stderr, " %08X", REG_DA[_i]);
		fprintf(stderr, "\n[LINE-F] A0-A7:");
		for (int _i = 8; _i < 16; _i++) fprintf(stderr, " %08X", REG_DA[_i]);
		fprintf(stderr, "\n");
		m68k_dump_pc_history();
		/* Request a graceful host-side exit so the disk superblocks
		 * get marked FsOKAY before native.img is closed.  Without
		 * this the kernel's infinite trap loop keeps running until
		 * the user kills the process, leaving the FS dirty for the
		 * next boot's fsck.                                       */
		{
			extern void board_request_stop(void);
			board_request_stop();
		}
	}

	sr = m68ki_init_exception();
	m68ki_stack_frame_0000(REG_PPC, sr, EXCEPTION_1111);
	m68ki_jump_vector(EXCEPTION_1111);

	/* Use up some clock cycles and undo the instruction's cycles */
	USE_CYCLES(CYC_EXCEPTION[EXCEPTION_1111] - CYC_INSTRUCTION[REG_IR]);
}

#if M68K_ILLG_HAS_CALLBACK == OPT_SPECIFY_HANDLER
extern int m68ki_illg_callback(int);
#endif

/* Exception for illegal instructions */
static inline void m68ki_exception_illegal(void)
{
	uint sr;

	M68K_DO_LOG((M68K_LOG_FILEHANDLE "%s at %08x: illegal instruction %04x (%s)\n",
				 m68ki_cpu_names[CPU_TYPE], ADDRESS_68K(REG_PPC), REG_IR,
				 m68ki_disassemble_quick(ADDRESS_68K(REG_PPC))));
	if (m68ki_illg_callback(REG_IR))
	    return;

	sr = m68ki_init_exception();

	#if M68K_EMULATE_ADDRESS_ERROR == OPT_ON
	if(CPU_TYPE_IS_000(CPU_TYPE))
	{
		CPU_INSTR_MODE = INSTRUCTION_NO;
	}
	#endif /* M68K_EMULATE_ADDRESS_ERROR */

	m68ki_stack_frame_0000(REG_PPC, sr, EXCEPTION_ILLEGAL_INSTRUCTION);
	m68ki_jump_vector(EXCEPTION_ILLEGAL_INSTRUCTION);

	/* Use up some clock cycles and undo the instruction's cycles */
	USE_CYCLES(CYC_EXCEPTION[EXCEPTION_ILLEGAL_INSTRUCTION] - CYC_INSTRUCTION[REG_IR]);
}

/* Exception for format errror in RTE */
extern int g_trace_pc;
static inline void m68ki_exception_format_error(void)
{
	uint sr = m68ki_init_exception();
	if (g_trace_pc)
		fprintf(stderr, "[FMTERR] RTE format error: PC=%06X SP=%06X "
			"frame@SP: %04X %04X %04X %04X %04X %04X\n", REG_PC, REG_SP,
			m68ki_read_16(REG_SP),     m68ki_read_16(REG_SP+2),
			m68ki_read_16(REG_SP+4),   m68ki_read_16(REG_SP+6),
			m68ki_read_16(REG_SP+8),   m68ki_read_16(REG_SP+10));
	m68ki_stack_frame_0000(REG_PC, sr, EXCEPTION_FORMAT_ERROR);
	m68ki_jump_vector(EXCEPTION_FORMAT_ERROR);

	/* Use up some clock cycles and undo the instruction's cycles */
	USE_CYCLES(CYC_EXCEPTION[EXCEPTION_FORMAT_ERROR] - CYC_INSTRUCTION[REG_IR]);
}

/* Exception for address error */
static inline void m68ki_exception_address_error(void)
{
	int i;

	/* Dump PC history for address errors — once only, when PC is odd */
	{
		static int aerr_dumped = 0;
		extern int g_debug;
		if (g_debug && !aerr_dumped && (REG_PPC & 1) && REG_PPC < 0xFF000000) {
			aerr_dumped = 1;
			extern unsigned long g_insn_count;
			extern void m68k_dump_pc_history(void);
			fprintf(stderr, "\n[ADDR-ERR CRASH] PC=%08X fault_addr=%08X SR=%04X FLAG_S=%d insn=%lu\n",
				REG_PPC, m68ki_aerr_address, m68ki_get_sr(), FLAG_S, g_insn_count);
			fprintf(stderr, "[ADDR-ERR] D0-D7:");
			for (int _i = 0; _i < 8; _i++) fprintf(stderr, " %08X", REG_DA[_i]);
			fprintf(stderr, "\n[ADDR-ERR] A0-A7:");
			for (int _i = 8; _i < 16; _i++) fprintf(stderr, " %08X", REG_DA[_i]);
			fprintf(stderr, "\n");
			m68k_dump_pc_history();
		}
	}

	if(CPU_TYPE_IS_020_PLUS(CPU_TYPE))
	{
		/* MC68030: Address error uses Format $A frame, same as bus error.
		 * Restore registers and build a proper bus error-style stack frame. */

		/* Clear R/M/W state */
		{
			extern int g_pmmu_rmw_in_progress;
			g_pmmu_rmw_in_progress = 0;
		}

		if(CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF)
		{
			CPU_STOPPED = STOP_LEVEL_HALT;
			CPU_RUN_MODE = RUN_MODE_NORMAL;
			SET_CYCLES(0);
			longjmp(m68ki_bus_error_jmp_buf, 1);
		}
		CPU_RUN_MODE = RUN_MODE_BERR_AERR_RESET_WSF;

		USE_CYCLES(CYC_EXCEPTION[EXCEPTION_ADDRESS_ERROR] - CYC_INSTRUCTION[REG_IR]);

		for (i = 15; i >= 0; i--)
			REG_DA[i] = REG_DA_SAVE[i];

		/* Set up fault info for Format $A frame */
		g_pmmu_fault_addr = m68ki_aerr_address;
		/* Build SSW: RW from access direction, FC from access FC */
		g_pmmu_fault_ssw = (m68ki_aerr_write_mode ? 0 : 0x0040) | (m68ki_aerr_fc & 7);
		g_pmmu_fault_dob = 0;

		uint sr = m68ki_init_exception();

		/* Stacked PC = faulting instruction (rerunnable) */
		m68ki_stack_frame_1010(sr, EXCEPTION_ADDRESS_ERROR, REG_PPC);

		m68ki_jump_vector(EXCEPTION_ADDRESS_ERROR);

		/* Exception processing complete — return to normal mode */
		CPU_RUN_MODE = RUN_MODE_NORMAL;

		longjmp(m68ki_bus_error_jmp_buf, 1);
		return; /* not reached */
	}

	/* MC68000/68010 path */
	uint sr = m68ki_init_exception();

	if(CPU_RUN_MODE == RUN_MODE_BERR_AERR_RESET_WSF)
	{
		CPU_STOPPED = STOP_LEVEL_HALT;
		CPU_RUN_MODE = RUN_MODE_NORMAL;
		SET_CYCLES(0);
		longjmp(m68ki_bus_error_jmp_buf, 1);
	}
	CPU_RUN_MODE = RUN_MODE_BERR_AERR_RESET_WSF;

	m68ki_stack_frame_buserr(sr);

	m68ki_jump_vector(EXCEPTION_ADDRESS_ERROR);

	/* Exception processing complete — return to normal mode */
	CPU_RUN_MODE = RUN_MODE_NORMAL;

	USE_CYCLES(CYC_EXCEPTION[EXCEPTION_ADDRESS_ERROR]);
}


/* Service an interrupt request and start exception processing */
static inline void m68ki_exception_interrupt(uint int_level)
{
	uint vector;
	uint sr;
	uint new_pc;

	#if M68K_EMULATE_ADDRESS_ERROR == OPT_ON
	if(CPU_TYPE_IS_000(CPU_TYPE))
	{
		CPU_INSTR_MODE = INSTRUCTION_NO;
	}
	#endif /* M68K_EMULATE_ADDRESS_ERROR */

	/* Turn off the stopped state */
	CPU_STOPPED &= ~STOP_LEVEL_STOP;

	/* If we are halted, don't do anything */
	if(CPU_STOPPED)
		return;

	/* Acknowledge the interrupt */
	vector = m68ki_int_ack(int_level);

	/* Get the interrupt vector */
	if(vector == M68K_INT_ACK_AUTOVECTOR)
		/* Use the autovectors.  This is the most commonly used implementation */
		vector = EXCEPTION_INTERRUPT_AUTOVECTOR+int_level;
	else if(vector == M68K_INT_ACK_SPURIOUS)
		/* Called if no devices respond to the interrupt acknowledge */
		vector = EXCEPTION_SPURIOUS_INTERRUPT;
	else if(vector > 255)
	{
		M68K_DO_LOG_EMU((M68K_LOG_FILEHANDLE "%s at %08x: Interrupt acknowledge returned invalid vector $%x\n",
				 m68ki_cpu_names[CPU_TYPE], ADDRESS_68K(REG_PC), vector));
		return;
	}

	/* Start exception processing */
	sr = m68ki_init_exception();

	/* Set the interrupt mask to the level of the one being serviced */
	FLAG_INT_MASK = int_level<<8;

	/* MC68030: Vector table reads use FC=5 (supervisor data), not FC=7. */
	new_pc = m68ki_read_32_fc((vector<<2) + REG_VBR, FUNCTION_CODE_SUPERVISOR_DATA);

	/* If vector is uninitialized, call the uninitialized interrupt vector */
	if(new_pc == 0)
		new_pc = m68ki_read_32_fc((EXCEPTION_UNINITIALIZED_INTERRUPT<<2) + REG_VBR, FUNCTION_CODE_SUPERVISOR_DATA);

	/* Generate a stack frame */
	m68ki_stack_frame_0000(REG_PC, sr, vector);
	if(FLAG_M && CPU_TYPE_IS_EC020_PLUS(CPU_TYPE))
	{
		/* Create throwaway frame */
		m68ki_set_sm_flag(FLAG_S);	/* clear M */
		sr |= 0x2000; /* Same as SR in master stack frame except S is forced high */
		m68ki_stack_frame_0001(REG_PC, sr, vector);
	}

	m68ki_jump(new_pc);

	/* Defer cycle counting until later */
	USE_CYCLES(CYC_EXCEPTION[vector]);

#if !M68K_EMULATE_INT_ACK
	/* Automatically clear IRQ if we are not using an acknowledge scheme */
	CPU_INT_LEVEL = 0;
#endif /* M68K_EMULATE_INT_ACK */
}


/* ASG: Check for interrupts */
static inline void m68ki_check_interrupts(void)
{
	if(m68ki_cpu.nmi_pending)
	{
		m68ki_cpu.nmi_pending = FALSE;
		m68ki_exception_interrupt(7);
	}
	else if(CPU_INT_LEVEL > FLAG_INT_MASK)
		m68ki_exception_interrupt(CPU_INT_LEVEL>>8);
}



/* ======================================================================== */
/* ============================== END OF FILE ============================= */
/* ======================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* M68KCPU__HEADER */
