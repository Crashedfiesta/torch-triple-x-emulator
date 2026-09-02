#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include "m68kcpu.h"

extern void exit(int);

// FPU boot trace - trace first N FPU operations during boot
static int g_fpu_trace_count = 0;
#define FPU_TRACE_LIMIT 500
#define FPU_TRACE 0  // Disabled
#define TRACE_FSAVE 0
#define TRACE_EARLY_FPU 0

// FPU model version byte for FSAVE frames (set by emulator)
// MC68881: 0x1F, MC68882: 0x41
extern int g_fpu_model_version;

static void fatalerror(const char *format, ...) {
      va_list ap;
      va_start(ap,format);
      vfprintf(stderr,format,ap);  // JFF: fixed. Was using fprintf and arguments were wrong
      va_end(ap);
      // Just log the error - don't exit or throw exception.
      // This happens when executing garbage as FPU instructions.
}

#define FPCC_N			0x08000000
#define FPCC_Z			0x04000000
#define FPCC_I			0x02000000
#define FPCC_NAN		0x01000000

#define DOUBLE_INFINITY					(unsigned long long)(0x7ff0000000000000)
#define DOUBLE_EXPONENT					(unsigned long long)(0x7ff0000000000000)
#define DOUBLE_MANTISSA					(unsigned long long)(0x000fffffffffffff)

extern flag floatx80_is_nan( floatx80 a );

// masks for packed dwords, positive k-factor
static uint32 pkmask2[18] =
{
	0xffffffff, 0, 0xf0000000, 0xff000000, 0xfff00000, 0xffff0000,
	0xfffff000, 0xffffff00, 0xfffffff0, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff
};

static uint32 pkmask3[18] =
{
	0xffffffff, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0xf0000000, 0xff000000, 0xfff00000, 0xffff0000,
	0xfffff000, 0xffffff00, 0xfffffff0, 0xffffffff,
};

static inline double fx80_to_double(floatx80 fx)
{
	uint64 d;
	double *foo;

	foo = (double *)&d;

	d = floatx80_to_float64(fx);

	return *foo;
}

static inline floatx80 double_to_fx80(double in)
{
	uint64 *d;

	d = (uint64 *)&in;

	return float64_to_floatx80(*d);
}

static inline floatx80 load_extended_float80(uint32 ea)
{
	uint32 d1,d2;
	uint16 d3;
	uint16 reserved;
	floatx80 fp;

	d3 = m68ki_read_16(ea);
	reserved = m68ki_read_16(ea+2);  // Read reserved bytes to check
	d1 = m68ki_read_32(ea+4);
	d2 = m68ki_read_32(ea+8);

	if (FPU_TRACE) {
		fprintf(stderr, "[FPU]     Load Ext @%08x: %04x %04x %08x %08x\n",
			ea, d3, reserved, d1, d2);
	}

	fp.high = d3;
	fp.low = ((uint64)d1<<32) | (d2 & 0xffffffff);

	return fp;
}

static inline void store_extended_float80(uint32 ea, floatx80 fpr)
{
	if (FPU_TRACE) {
		fprintf(stderr, "[FPU]     -> Store Ext @%08x: %04x %04x %08x %08x\n",
			ea, fpr.high, 0, (uint32)(fpr.low>>32), (uint32)(fpr.low&0xffffffff));
	}
	m68ki_write_16(ea+0, fpr.high);
	m68ki_write_16(ea+2, 0);
	m68ki_write_32(ea+4, (fpr.low>>32)&0xffffffff);
	m68ki_write_32(ea+8, fpr.low&0xffffffff);
}

static inline floatx80 load_pack_float80(uint32 ea)
{
	uint32 dw1, dw2, dw3;
	floatx80 result;
	double tmp;
	char str[128], *ch;

	dw1 = m68ki_read_32(ea);
	dw2 = m68ki_read_32(ea+4);
	dw3 = m68ki_read_32(ea+8);

	ch = &str[0];
	if (dw1 & 0x80000000)	// mantissa sign
	{
		*ch++ = '-';
	}
	*ch++ = (char)((dw1 & 0xf) + '0');
	*ch++ = '.';
	*ch++ = (char)(((dw2 >> 28) & 0xf) + '0');
	*ch++ = (char)(((dw2 >> 24) & 0xf) + '0');
	*ch++ = (char)(((dw2 >> 20) & 0xf) + '0');
	*ch++ = (char)(((dw2 >> 16) & 0xf) + '0');
	*ch++ = (char)(((dw2 >> 12) & 0xf) + '0');
	*ch++ = (char)(((dw2 >> 8)  & 0xf) + '0');
	*ch++ = (char)(((dw2 >> 4)  & 0xf) + '0');
	*ch++ = (char)(((dw2 >> 0)  & 0xf) + '0');
	*ch++ = (char)(((dw3 >> 28) & 0xf) + '0');
	*ch++ = (char)(((dw3 >> 24) & 0xf) + '0');
	*ch++ = (char)(((dw3 >> 20) & 0xf) + '0');
	*ch++ = (char)(((dw3 >> 16) & 0xf) + '0');
	*ch++ = (char)(((dw3 >> 12) & 0xf) + '0');
	*ch++ = (char)(((dw3 >> 8)  & 0xf) + '0');
	*ch++ = (char)(((dw3 >> 4)  & 0xf) + '0');
	*ch++ = (char)(((dw3 >> 0)  & 0xf) + '0');
	*ch++ = 'E';
	if (dw1 & 0x40000000)	// exponent sign
	{
		*ch++ = '-';
	}
	*ch++ = (char)(((dw1 >> 24) & 0xf) + '0');
	*ch++ = (char)(((dw1 >> 20) & 0xf) + '0');
	*ch++ = (char)(((dw1 >> 16) & 0xf) + '0');
	*ch = '\0';

	if (FPU_TRACE) {
		fprintf(stderr, "[FPU]     Load Packed: dw1=%08x dw2=%08x dw3=%08x (str='%s')\n",
			dw1, dw2, dw3, str);
	}
	sscanf(str, "%le", &tmp);

	result = double_to_fx80(tmp);

	return result;
}

static inline void store_pack_float80(uint32 ea, int k, floatx80 fpr)
{
	uint32 dw1, dw2, dw3;
	char str[128], *ch;
	int i, j, exp;

	dw1 = dw2 = dw3 = 0;
	ch = &str[0];

	sprintf(str, "%.16e", fx80_to_double(fpr));

	if (*ch == '-')
	{
		ch++;
		dw1 = 0x80000000;
	}

	if (*ch == '+')
	{
		ch++;
	}

	dw1 |= (*ch++ - '0');

	if (*ch == '.')
	{
		ch++;
	}

	// handle negative k-factor here
	if ((k <= 0) && (k >= -13))
	{
		exp = 0;
		for (i = 0; i < 3; i++)
		{
			if (ch[18+i] >= '0' && ch[18+i] <= '9')
			{
				exp = (exp << 4) | (ch[18+i] - '0');
			}
		}

		if (ch[17] == '-')
		{
			exp = -exp;
		}

		k = -k;
		// last digit is (k + exponent - 1)
		k += (exp - 1);

		// round up the last significant mantissa digit
		if (ch[k+1] >= '5')
		{
			ch[k]++;
		}

		// zero out the rest of the mantissa digits
		for (j = (k+1); j < 16; j++)
		{
			ch[j] = '0';
		}

		// now zero out K to avoid tripping the positive K detection below
		k = 0;
	}

	// crack 8 digits of the mantissa
	for (i = 0; i < 8; i++)
	{
		dw2 <<= 4;
		if (*ch >= '0' && *ch <= '9')
		{
			dw2 |= *ch++ - '0';
		}
	}

	// next 8 digits of the mantissa
	for (i = 0; i < 8; i++)
	{
		dw3 <<= 4;
		if (*ch >= '0' && *ch <= '9')
		dw3 |= *ch++ - '0';
	}

	// handle masking if k is positive
	if (k >= 1)
	{
		if (k <= 17)
		{
			dw2 &= pkmask2[k];
			dw3 &= pkmask3[k];
		}
		else
		{
			dw2 &= pkmask2[17];
			dw3 &= pkmask3[17];
//			m68ki_cpu.fpcr |=  (need to set OPERR bit)
		}
	}

	// finally, crack the exponent
	if (*ch == 'e' || *ch == 'E')
	{
		ch++;
		if (*ch == '-')
		{
			ch++;
			dw1 |= 0x40000000;
		}

		if (*ch == '+')
		{
			ch++;
		}

		j = 0;
		for (i = 0; i < 3; i++)
		{
			if (*ch >= '0' && *ch <= '9')
			{
				j = (j << 4) | (*ch++ - '0');
			}
		}

		dw1 |= (j << 16);
	}

	if (FPU_TRACE) {
		fprintf(stderr, "[FPU]     -> Store Packed: k=%d dw1=%08x dw2=%08x dw3=%08x (str='%s')\n",
			k, dw1, dw2, dw3, str);
	}
	m68ki_write_32(ea, dw1);
	m68ki_write_32(ea+4, dw2);
	m68ki_write_32(ea+8, dw3);
}

static inline void SET_CONDITION_CODES(floatx80 reg)
{
	REG_FPSR &= ~(FPCC_N|FPCC_Z|FPCC_I|FPCC_NAN);

	// sign flag
	if (reg.high & 0x8000)
	{
		REG_FPSR |= FPCC_N;
	}

	// zero flag
	if (((reg.high & 0x7fff) == 0) && ((reg.low<<1) == 0))
	{
		REG_FPSR |= FPCC_Z;
	}

	// infinity flag
	if (((reg.high & 0x7fff) == 0x7fff) && ((reg.low<<1) == 0))
	{
		REG_FPSR |= FPCC_I;
	}

	// NaN flag
	if (floatx80_is_nan(reg))
	{
		REG_FPSR |= FPCC_NAN;
	}
}

/* FPSR exception STATUS byte bit positions (bits 15-8) */
#define FPSR_BSUN_STATUS	0x00008000
#define FPSR_SNAN_STATUS	0x00004000
#define FPSR_OPERR_STATUS	0x00002000
#define FPSR_OVFL_STATUS	0x00001000
#define FPSR_UNFL_STATUS	0x00000800
#define FPSR_DZ_STATUS		0x00000400
#define FPSR_INEX2_STATUS	0x00000200
#define FPSR_INEX1_STATUS	0x00000100

/* FPSR exception ACCRUED byte bit positions (bits 7-0) */
#define FPSR_BSUN_ACCRUED	0x00000080
#define FPSR_SNAN_ACCRUED	0x00000040
#define FPSR_OPERR_ACCRUED	0x00000020
#define FPSR_OVFL_ACCRUED	0x00000010
#define FPSR_UNFL_ACCRUED	0x00000008
#define FPSR_DZ_ACCRUED		0x00000004
#define FPSR_INEX2_ACCRUED	0x00000002
#define FPSR_INEX1_ACCRUED	0x00000001

/* FPCR exception enable byte bit positions (bits 7-0) */
#define FPCR_BSUN_EN	0x00000080
#define FPCR_SNAN_EN	0x00000040
#define FPCR_OPERR_EN	0x00000020
#define FPCR_OVFL_EN	0x00000010
#define FPCR_UNFL_EN	0x00000008
#define FPCR_DZ_EN	0x00000004
#define FPCR_INEX2_EN	0x00000002
#define FPCR_INEX1_EN	0x00000001

// MC68882 signaling NaN detection.  On MC68882, a signaling NaN includes:
// 1. Standard SNAN: exponent=0x7FFF, j-bit(63)=1, bit62=0, mantissa!=0
// 2. Unnormal in NaN range: exponent=0x7FFF, j-bit(63)=0, mantissa!=0
// Both conditions generate a SNAN exception when referenced as a source operand.
static int is_mc68882_snan(floatx80 a)
{
	if ((a.high & 0x7FFF) != 0x7FFF) return 0;
	if (a.low == 0) return 0;
	// j-bit = 0 → unnormal → treated as SNAN
	if (!(a.low & U64(0x8000000000000000))) return 1;
	// j-bit = 1, bit 62 = 0 → standard SNAN
	if (!(a.low & U64(0x4000000000000000))) return 1;
	return 0;
}

// Map softfloat exception flags to MC68882 FPSR exception status bits.
// Called after each FPU operation.  The exception status byte (bits 15-8) is
// cleared per-instruction and only bits for exceptions from that specific
// instruction are set.  The accrued byte (bits 7-0) accumulates across
// operations.
static void fpu_update_fpsr_exceptions(floatx80 source)
{
	int sf = float_exception_flags;
	float_exception_flags = 0;

	int src_snan = is_mc68882_snan(source);

	// Detect denormalized input (exponent=0, mantissa!=0).  On MC68882,
	// loading a denorm creates a pending UNFL condition.
	int src_denorm = ((source.high & 0x7FFF) == 0) && (source.low != 0);

	// Always clear exception status byte — each instruction starts fresh.
	// Only the accrued byte accumulates across operations.
	REG_FPSR &= ~0x0000FF00;

	if (!sf && !src_snan && !src_denorm) return;

	// SNAN has priority over OPERR
	if (src_snan) {
		REG_FPSR |= FPSR_SNAN_STATUS | FPSR_SNAN_ACCRUED;
		sf &= ~float_flag_invalid;
	}
	if (sf & float_flag_invalid)
		REG_FPSR |= FPSR_OPERR_STATUS | FPSR_OPERR_ACCRUED;
	if (sf & float_flag_divbyzero)
		REG_FPSR |= FPSR_DZ_STATUS | FPSR_DZ_ACCRUED;
	if (sf & float_flag_overflow)
		REG_FPSR |= FPSR_OVFL_STATUS | FPSR_OVFL_ACCRUED;
	if ((sf & float_flag_underflow) || src_denorm)
		REG_FPSR |= FPSR_UNFL_STATUS | FPSR_UNFL_ACCRUED;
	if (sf & float_flag_inexact)
		REG_FPSR |= FPSR_INEX2_STATUS | FPSR_INEX2_ACCRUED;
}

// Map FPU exception status bit position (bits 15-8) to M68K exception vector number.
// Bit 8=INEX1(49), 9=INEX2(49), 10=DZ(50), 11=UNFL(51),
// 12=OVFL(53), 13=OPERR(52), 14=SNAN(54), 15=BSUN(48)
static const int fpu_exc_vectors[8] = {49, 49, 50, 51, 53, 52, 54, 48};

// Check for pending enabled FPU exceptions and take the highest-priority
// one.  On MC68882, enabled exceptions from arithmetic operations are
// taken immediately.  Deferred exceptions (from FMOVE of denorms) are
// triggered at FNOP or other synchronization points.
static void fpu_check_pending_exceptions(void)
{
	uint32 pending = REG_FPSR & REG_FPCR & 0x0000FF00;
	if (pending) {
		// Priority: highest bit first (BSUN > SNAN > OPERR > OVFL > UNFL > DZ > INEX)
		for (int bit = 15; bit >= 8; bit--) {
			if (pending & (1 << bit)) {
				m68ki_exception_trap(fpu_exc_vectors[bit - 8]);
				return;
			}
		}
	}
}

/* Returns: condition result (0 or 1), or -1 if BSUN exception was taken */
static inline int TEST_CONDITION(int condition)
{
	int n = (REG_FPSR & FPCC_N) != 0;
	int z = (REG_FPSR & FPCC_Z) != 0;
	int nan = (REG_FPSR & FPCC_NAN) != 0;
	int r = 0;

	/* Handle condition codes 0x20-0x3f which are IEEE signaling versions
	 * of 0x00-0x1f. Bit 5 indicates BSUN (Branch/Set on Unordered) should
	 * signal if NaN. We mask to get the base condition predicate. */
	int signaling = (condition & 0x20) != 0;
	int cond = condition & 0x1f;

	/* BSUN: If using signaling condition and NaN is set, handle BSUN.
	 * According to MC68881/MC68882 UM, BSUN is signaled when:
	 * 1. A conditional instruction with a signaling predicate (bit 5 set)
	 * 2. AND the NAN bit is set in FPSR condition codes */
	if (signaling && nan) {
		/* Always set BSUN in exception status byte */
		REG_FPSR |= FPSR_BSUN_STATUS;

		if (FPU_TRACE) {
			fprintf(stderr, "[FPU]   BSUN signaled: cond=%02x nan=%d FPSR=%08x FPCR=%08x\n",
				condition, nan, REG_FPSR, REG_FPCR);
		}

		/* Check if BSUN exception is enabled in FPCR */
		if (REG_FPCR & FPCR_BSUN_EN) {
			/* Generate BSUN exception (vector 48) */
			if (FPU_TRACE) {
				fprintf(stderr, "[FPU]   BSUN exception! Taking vector 48\n");
			}
			m68ki_exception_trap(48);
			return -1;  /* Signal that exception was taken */
		} else {
			/* BSUN not enabled - set accrued bit and continue */
			REG_FPSR |= FPSR_BSUN_ACCRUED;
		}
	}

	switch (cond)
	{
		case 0x10:
		case 0x00:		return 0;					// False

		/* EQ: For IEEE compliance, NaN comparisons should return unordered (not equal)
		 * The MC68881 condition codes may have Z=1 from a test pattern, but if NAN is
		 * also set, we should return false (unordered is never equal). */
		case 0x11:
		case 0x01:		return (z && !nan);				// Equal (ordered)

		case 0x12:
		case 0x02:		return (!(nan || z || n));			// Greater Than

		case 0x13:
		case 0x03:		return (z || !(nan || n));			// Greater or Equal

		case 0x14:
		case 0x04:		return (n && !(nan || z));			// Less Than

		case 0x15:
		case 0x05:		return (z || (n && !nan));			// Less Than or Equal

		case 0x16:
		case 0x06:		return !nan && !z;

		case 0x17:
		case 0x07:		return !nan;

		case 0x18:
		case 0x08:		return nan;

		case 0x19:
		case 0x09:		return nan || z;

		case 0x1a:
		case 0x0a:		return (nan || !(n || z));			// Not Less Than or Equal

		case 0x1b:
		case 0x0b:		return (nan || z || !n);			// Not Less Than

		case 0x1c:
		case 0x0c:		return (nan || (n && !z));			// Not Greater or Equal Than

		case 0x1d:
		case 0x0d:		return (nan || z || n);				// Not Greater Than

		case 0x1e:
		case 0x0e:		return (!z);					// Not Equal

		case 0x1f:
		case 0x0f:		return 1;					// True

		default:		fatalerror("M68kFPU: test_condition: unhandled condition %02X\n", condition);
	}

	return r;
}

static uint8 READ_EA_8(int ea)
{
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);

	switch (mode)
	{
		case 0:		// Dn
		{
			return REG_D[reg];
		}
		case 2: 	// (An)
		{
			uint32 ea = REG_A[reg];
			return m68ki_read_8(ea);
		}
		case 3:		// (An)+
		{
			uint32 ea = EA_AY_PI_8();
			return m68ki_read_8(ea);
		}
		case 4:		// -(An)
		{
			uint32 ea = EA_AY_PD_8();
			return m68ki_read_8(ea);
		}
		case 5:		// (d16, An)
		{
			uint32 ea = EA_AY_DI_8();
			return m68ki_read_8(ea);
		}
		case 6:		// (An) + (Xn) + d8
		{
			uint32 ea = EA_AY_IX_8();
			return m68ki_read_8(ea);
		}
		case 7:
		{
			switch (reg)
			{
				case 0:		// (xxx).W
				{
					uint32 ea = (uint32)OPER_I_16();
					return m68ki_read_8(ea);
				}
				case 1:		// (xxx).L
				{
					uint32 d1 = OPER_I_16();
					uint32 d2 = OPER_I_16();
					uint32 ea = (d1 << 16) | d2;
					return m68ki_read_8(ea);
				}
				case 4:		// #<data>
				{
					return  OPER_I_8();
				}
				default:	fatalerror("M68kFPU: READ_EA_8: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
			}
			break;
		}
		default:	fatalerror("M68kFPU: READ_EA_8: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
	}

	return 0;
}

static uint16 READ_EA_16(int ea)
{
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);

	switch (mode)
	{
		case 0:		// Dn
		{
			return (uint16)(REG_D[reg]);
		}
		case 2:		// (An)
		{
			uint32 ea = REG_A[reg];
			return m68ki_read_16(ea);
		}
		case 3:		// (An)+
		{
			uint32 ea = EA_AY_PI_16();
			return m68ki_read_16(ea);
		}
		case 4:		// -(An)
		{
			uint32 ea = EA_AY_PD_16();
			return m68ki_read_16(ea);
		}
		case 5:		// (d16, An)
		{
			uint32 ea = EA_AY_DI_16();
			return m68ki_read_16(ea);
		}
		case 6:		// (An) + (Xn) + d8
		{
			uint32 ea = EA_AY_IX_16();
			return m68ki_read_16(ea);
		}
		case 7:
		{
			switch (reg)
			{
				case 0:		// (xxx).W
				{
					uint32 ea = (uint32)OPER_I_16();
					return m68ki_read_16(ea);
				}
				case 1:		// (xxx).L
				{
					uint32 d1 = OPER_I_16();
					uint32 d2 = OPER_I_16();
					uint32 ea = (d1 << 16) | d2;
					return m68ki_read_16(ea);
				}
				case 2:		// (d16, PC)
				{
					uint32 ea = EA_PCDI_16();
					return m68ki_read_16(ea);
				}
				case 4:		// #<data>
				{
					return OPER_I_16();
				}

				default:	fatalerror("M68kFPU: READ_EA_16: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
			}
			break;
		}
		default:	fatalerror("M68kFPU: READ_EA_16: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
	}

	return 0;
}

static uint32 READ_EA_32(int ea)
{
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);

	switch (mode)
	{
		case 0:		// Dn
		{
			return REG_D[reg];
		}
		case 2:		// (An)
		{
			uint32 ea = REG_A[reg];
			return m68ki_read_32(ea);
		}
		case 3:		// (An)+
		{
			uint32 ea = EA_AY_PI_32();
			return m68ki_read_32(ea);
		}
		case 4:		// -(An)
		{
			uint32 ea = EA_AY_PD_32();
			return m68ki_read_32(ea);
		}
		case 5:		// (d16, An)
		{
			uint32 ea = EA_AY_DI_32();
			return m68ki_read_32(ea);
		}
		case 6:		// (An) + (Xn) + d8
		{
			uint32 ea = EA_AY_IX_32();
			return m68ki_read_32(ea);
		}
		case 7:
		{
			switch (reg)
			{
				case 0:		// (xxx).W
				{
					uint32 ea = (uint32)OPER_I_16();
					return m68ki_read_32(ea);
				}
				case 1:		// (xxx).L
				{
					uint32 d1 = OPER_I_16();
					uint32 d2 = OPER_I_16();
					uint32 ea = (d1 << 16) | d2;
					return m68ki_read_32(ea);
				}
				case 2:		// (d16, PC)
				{
					uint32 ea = EA_PCDI_32();
					return m68ki_read_32(ea);
				}
				case 4:		// #<data>
				{
					return  OPER_I_32();
				}
				default:	fatalerror("M68kFPU: READ_EA_32: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
			}
			break;
		}
		default:	fatalerror("M68kFPU: READ_EA_32: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
	}
	return 0;
}

static uint64 READ_EA_64(int ea)
{
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);
	uint32 h1, h2;

	switch (mode)
	{
		case 2:		// (An)
		{
			uint32 ea = REG_A[reg];
			h1 = m68ki_read_32(ea+0);
			h2 = m68ki_read_32(ea+4);
			return  (uint64)(h1) << 32 | (uint64)(h2);
		}
		case 3:		// (An)+
		{
			uint32 ea = REG_A[reg];
			REG_A[reg] += 8;
			h1 = m68ki_read_32(ea+0);
			h2 = m68ki_read_32(ea+4);
			return  (uint64)(h1) << 32 | (uint64)(h2);
		}
		case 4:		// -(An)
		{
			REG_A[reg] -= 8;
			uint32 ea = REG_A[reg];
			h1 = m68ki_read_32(ea+0);
			h2 = m68ki_read_32(ea+4);
			return  (uint64)(h1) << 32 | (uint64)(h2);
		}
		case 5:		// (d16, An)
		{
			uint32 ea = EA_AY_DI_32();
			h1 = m68ki_read_32(ea+0);
			h2 = m68ki_read_32(ea+4);
			return  (uint64)(h1) << 32 | (uint64)(h2);
		}
		case 6:		// (An) + (Xn) + d8
		{
			uint32 ea = EA_AY_IX_16();
			h1 = m68ki_read_32(ea+0);
			h2 = m68ki_read_32(ea+4);
			return  (uint64)(h1) << 32 | (uint64)(h2);
		}
		case 7:
		{
			switch (reg)
			{
				case 1:		// (xxx).L
				{
					uint32 d1 = OPER_I_16();
					uint32 d2 = OPER_I_16();
					uint32 ea = (d1 << 16) | d2;
					h1 = m68ki_read_32(ea+0);
					h2 = m68ki_read_32(ea+4);
					return  (uint64)(h1) << 32 | (uint64)(h2);
				}
				case 4:		// #<data>
				{
					h1 = OPER_I_32();
					h2 = OPER_I_32();
					return  (uint64)(h1) << 32 | (uint64)(h2);
				}
				case 2:		// (d16, PC)
				{
					uint32 ea = EA_PCDI_32();
					h1 = m68ki_read_32(ea+0);
					h2 = m68ki_read_32(ea+4);
					return  (uint64)(h1) << 32 | (uint64)(h2);
				}
				default:	fatalerror("M68kFPU: READ_EA_64: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
			}
			break;
		}
		default:	fatalerror("M68kFPU: READ_EA_64: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
	}

	return 0;
}


static floatx80 READ_EA_FPE(int mode, int reg, uint32 di_mode_ea)
{
	floatx80 fpr;

	switch (mode)
	{
		case 2:		// (An)
		{
			uint32 ea = REG_A[reg];
			fpr = load_extended_float80(ea);
			break;
		}
		case 3:		// (An)+
		{
			uint32 ea = REG_A[reg];
			REG_A[reg] += 12;
			fpr = load_extended_float80(ea);
			break;
		}
		case 4:		// -(An)
		{
			REG_A[reg] -= 12;
			uint32 ea = REG_A[reg];
			fpr = load_extended_float80(ea);
			break;
		}
      case 5:		// (d16, An)  (added by JFF)
		{
		  fpr = load_extended_float80(di_mode_ea);
	  	break;
		}
	  case 6:		// (An) + (Xn) + d8
		{
		  uint32 ea = EA_AY_IX_16();
		  fpr = load_extended_float80(ea);
		  break;
		}
		case 7:	// extended modes
		{
			switch (reg)
			{
				case 1:		// (xxx).L
					{
						uint32 d1 = OPER_I_16();
						uint32 d2 = OPER_I_16();
						fpr = load_extended_float80((d1 << 16) | d2);
					}
					break;
				case 2:	// (d16, PC)
					{
						uint32 ea = EA_PCDI_32();
					 	fpr = load_extended_float80(ea);
					}
					break;

				case 3:	// (d16,PC,Dx.w)
					{
						uint32 ea = EA_PCIX_32();
						fpr = load_extended_float80(ea);
					}
					break;
	      		case 4: // immediate (JFF)
				{
				  uint32 ea = REG_PC;
				  fpr = load_extended_float80(ea);
				  REG_PC += 12;
				}
				break;
				default:
					fatalerror("M68kFPU: READ_EA_FPE: unhandled mode %d, reg %d, at %08X\n", mode, reg, REG_PC);
					break;
			}
		}
		break;

		default:	fatalerror("M68kFPU: READ_EA_FPE: unhandled mode %d, reg %d, at %08X\n", mode, reg, REG_PC); break;
	}

	return fpr;
}

static floatx80 READ_EA_PACK(int ea)
{
	floatx80 fpr;
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);

	switch (mode)
	{
		case 2:		// (An)
		{
			uint32 ea = REG_A[reg];
			fpr = load_pack_float80(ea);
			break;
		}

		case 3:		// (An)+
		{
			uint32 ea = REG_A[reg];
			REG_A[reg] += 12;
			fpr = load_pack_float80(ea);
			break;
		}
		case 4:		// -(An)
		{
			REG_A[reg] -= 12;
			uint32 ea = REG_A[reg];
			fpr = load_pack_float80(ea);
			break;
		}

		case 7:	// extended modes
		{
			switch (reg)
			{
				case 3:	// (d16,PC,Dx.w)
					{
						uint32 ea = EA_PCIX_32();
						fpr = load_pack_float80(ea);
					}
					break;

				default:
					fatalerror("M68kFPU: READ_EA_PACK: unhandled mode %d, reg %d, at %08X\n", mode, reg, REG_PC);
					break;
			}
		}
		break;

		default:	fatalerror("M68kFPU: READ_EA_PACK: unhandled mode %d, reg %d, at %08X\n", mode, reg, REG_PC); break;
	}

	return fpr;
}

static void WRITE_EA_8(int ea, uint8 data)
{
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);

	switch (mode)
	{
		case 0:		// Dn
		{
			REG_D[reg] = data;
			break;
		}
		case 2:		// (An)
		{
			uint32 ea = REG_A[reg];
			m68ki_write_8(ea, data);
			break;
		}
		case 3:		// (An)+
		{
			uint32 ea = EA_AY_PI_8();
			m68ki_write_8(ea, data);
			break;
		}
		case 4:		// -(An)
		{
			uint32 ea = EA_AY_PD_8();
			m68ki_write_8(ea, data);
			break;
		}
		case 5:		// (d16, An)
		{
			uint32 ea = EA_AY_DI_8();
			m68ki_write_8(ea, data);
			break;
		}
		case 6:		// (An) + (Xn) + d8
		{
			uint32 ea = EA_AY_IX_8();
			m68ki_write_8(ea, data);
			break;
		}
		case 7:
		{
			switch (reg)
			{
				case 1:		// (xxx).B
				{
					uint32 d1 = OPER_I_16();
					uint32 d2 = OPER_I_16();
					uint32 ea = (d1 << 16) | d2;
					m68ki_write_8(ea, data);
					break;
				}
				case 2:		// (d16, PC)
				{
					uint32 ea = EA_PCDI_16();
					m68ki_write_8(ea, data);
					break;
				}
				default:	fatalerror("M68kFPU: WRITE_EA_8: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
			}
			break;
		}
		default:	fatalerror("M68kFPU: WRITE_EA_8: unhandled mode %d, reg %d, data %08X at %08X\n", mode, reg, data, REG_PC);
	}
}

static void WRITE_EA_16(int ea, uint16 data)
{
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);

	switch (mode)
	{
		case 0:		// Dn
		{
			REG_D[reg] = data;
			break;
		}
		case 2:		// (An)
		{
			uint32 ea = REG_A[reg];
			m68ki_write_16(ea, data);
			break;
		}
		case 3:		// (An)+
		{
			uint32 ea = EA_AY_PI_16();
			m68ki_write_16(ea, data);
			break;
		}
		case 4:		// -(An)
		{
			uint32 ea = EA_AY_PD_16();
			m68ki_write_16(ea, data);
			break;
		}
		case 5:		// (d16, An)
		{
			uint32 ea = EA_AY_DI_16();
			m68ki_write_16(ea, data);
			break;
		}
		case 6:		// (An) + (Xn) + d8
		{
			uint32 ea = EA_AY_IX_16();
			m68ki_write_16(ea, data);
			break;
		}
		case 7:
		{
			switch (reg)
			{
				case 1:		// (xxx).W
				{
					uint32 d1 = OPER_I_16();
					uint32 d2 = OPER_I_16();
					uint32 ea = (d1 << 16) | d2;
					m68ki_write_16(ea, data);
					break;
				}
				case 2:		// (d16, PC)
				{
					uint32 ea = EA_PCDI_16();
					m68ki_write_16(ea, data);
					break;
				}
				default:	fatalerror("M68kFPU: WRITE_EA_16: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
			}
			break;
		}
		default:	fatalerror("M68kFPU: WRITE_EA_16: unhandled mode %d, reg %d, data %08X at %08X\n", mode, reg, data, REG_PC);
	}
}

static void WRITE_EA_32(int ea, uint32 data)
{
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);

	switch (mode)
	{
		case 0:		// Dn
		{
			REG_D[reg] = data;
			break;
		}
		case 1:		// An
		{
			REG_A[reg] = data;
			break;
		}
		case 2:		// (An)
		{
			uint32 ea = REG_A[reg];
			m68ki_write_32(ea, data);
			break;
		}
		case 3:		// (An)+
		{
			uint32 ea = EA_AY_PI_32();
			m68ki_write_32(ea, data);
			break;
		}
		case 4:		// -(An)
		{
			uint32 ea = EA_AY_PD_32();
			m68ki_write_32(ea, data);
			break;
		}
		case 5:		// (d16, An)
		{
			uint32 ea = EA_AY_DI_32();
			m68ki_write_32(ea, data);
			break;
		}
		case 6:		// (An) + (Xn) + d8
		{
			uint32 ea = EA_AY_IX_32();
			m68ki_write_32(ea, data);
			break;
		}
		case 7:
		{
			switch (reg)
			{
				case 1:		// (xxx).L
				{
					uint32 d1 = OPER_I_16();
					uint32 d2 = OPER_I_16();
					uint32 ea = (d1 << 16) | d2;
					m68ki_write_32(ea, data);
					break;
				}
				case 2:		// (d16, PC)
				{
					uint32 ea = EA_PCDI_32();
					m68ki_write_32(ea, data);
					break;
				}
				default:	fatalerror("M68kFPU: WRITE_EA_32: unhandled mode %d, reg %d at %08X\n", mode, reg, REG_PC);
			}
			break;
		}
		default:	fatalerror("M68kFPU: WRITE_EA_32: unhandled mode %d, reg %d, data %08X at %08X\n", mode, reg, data, REG_PC);
	}
}

static void WRITE_EA_64(int ea, uint64 data)
{
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);

	switch (mode)
	{
		case 2:		// (An)
		{
			uint32 ea = REG_A[reg];
			m68ki_write_32(ea, (uint32)(data >> 32));
			m68ki_write_32(ea+4, (uint32)(data));
			break;
		}
		case 3:		// (An)+
		{
			uint32 ea;
			ea = REG_A[reg];
			REG_A[reg] += 8;
			m68ki_write_32(ea+0, (uint32)(data >> 32));
			m68ki_write_32(ea+4, (uint32)(data));
			break;
		}
		case 4:		// -(An)
		{
			uint32 ea;
			REG_A[reg] -= 8;
			ea = REG_A[reg];
			m68ki_write_32(ea+0, (uint32)(data >> 32));
			m68ki_write_32(ea+4, (uint32)(data));
			break;
		}
		case 5:		// (d16, An)
		{
			uint32 ea = EA_AY_DI_32();
			m68ki_write_32(ea+0, (uint32)(data >> 32));
			m68ki_write_32(ea+4, (uint32)(data));
			break;
		}

		case 6:		// (An) + (Xn) + d8
		{
			uint32 ea = EA_AY_IX_16();
			m68ki_write_32(ea+0, (uint32)(data >> 32));
			m68ki_write_32(ea+4, (uint32)(data));
			break;
		}
		case 7:
		{
			switch (reg)
			{
				case 1:		// (xxx).L
				{
					uint32 d1 = OPER_I_16();
					uint32 d2 = OPER_I_16();
					uint32 ea = (d1 << 16) | d2;
					m68ki_write_32(ea+0, (uint32)(data >> 32));
					m68ki_write_32(ea+4, (uint32)(data));
					break;
				}
				case 2:		// (d16, PC)
				{
					uint32 ea = EA_PCDI_32();
					m68ki_write_32(ea+0, (uint32)(data >> 32));
					m68ki_write_32(ea+4, (uint32)(data));
					break;
				}
				default:	fatalerror("M68kFPU: WRITE_EA_64: unhandled mode %d, data %08X%08X at %08X\n", mode, reg, (uint32)(data >> 32), (uint32)(data), REG_PC);
			}
			break;
		}
		default:	fatalerror("M68kFPU: WRITE_EA_64: unhandled mode %d, reg %d, data %08X%08X at %08X\n", mode, reg, (uint32)(data >> 32), (uint32)(data), REG_PC);
	}
}

static void WRITE_EA_FPE(int mode, int reg, floatx80 fpr, uint32 di_mode_ea)
{


	switch (mode)
	{
		case 2:		// (An)
		{
			uint32 ea;
			ea = REG_A[reg];
			store_extended_float80(ea, fpr);
			break;
		}

		case 3:		// (An)+
		{
			uint32 ea;
			ea = REG_A[reg];
			store_extended_float80(ea, fpr);
			REG_A[reg] += 12;
			break;
		}

		case 4:		// -(An)
		{
			uint32 ea;
			REG_A[reg] -= 12;
			ea = REG_A[reg];
			store_extended_float80(ea, fpr);
			break;
		}
    	  case 5:		// (d16, An)  (added by JFF)
		{
		  // EA_AY_DI_32() should not be done here because fmovem would increase
		  // PC each time, reading incorrect displacement & advancing PC too much
		  // uint32 ea = EA_AY_DI_32();
		  store_extended_float80(di_mode_ea, fpr);
	 	 break;

		}
		case 7:
		{
			switch (reg)
			{
				case 1:		// (xxx).L
				{
					uint32 d1 = OPER_I_16();
					uint32 d2 = OPER_I_16();
					uint32 ea = (d1 << 16) | d2;
					store_extended_float80(ea, fpr);
					break;
				}

				default:	fatalerror("M68kFPU: WRITE_EA_FPE: unhandled mode %d, reg %d, at %08X\n", mode, reg, REG_PC);
			}
			break;
		}
		default:	fatalerror("M68kFPU: WRITE_EA_FPE: unhandled mode %d, reg %d, at %08X\n", mode, reg, REG_PC);
	}
}

static void WRITE_EA_PACK(int ea, int k, floatx80 fpr)
{
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);

	switch (mode)
	{
		case 2:		// (An)
		{
			uint32 ea;
			ea = REG_A[reg];
			store_pack_float80(ea, k, fpr);
			break;
		}

		case 3:		// (An)+
		{
			uint32 ea;
			ea = REG_A[reg];
			store_pack_float80(ea, k, fpr);
			REG_A[reg] += 12;
			break;
		}

		case 4:		// -(An)
		{
			uint32 ea;
			REG_A[reg] -= 12;
			ea = REG_A[reg];
			store_pack_float80(ea, k, fpr);
			break;
		}

		case 7:
		{
			switch (reg)
			{
				default:	fatalerror("M68kFPU: WRITE_EA_PACK: unhandled mode %d, reg %d, at %08X\n", mode, reg, REG_PC);
			}
		}
		break;
		default:	fatalerror("M68kFPU: WRITE_EA_PACK: unhandled mode %d, reg %d, at %08X\n", mode, reg, REG_PC);
	}
}

static inline int is_inf(floatx80 reg) {
	if (((reg.high & 0x7fff) == 0x7fff) && ((reg.low<<1) == 0))
		return reg.high & 0x8000 ? -1 : 1;
	return 0;
}

static const char *fpu_opmode_names[] = {
	"FMOVE", "FINT", "FSINH", "FINTRZ", "FSQRT", NULL, "FLOGNP1", NULL,  // 0x00-0x07
	"FETOXM1", "FTANH", "FATAN", NULL, "FASIN", "FATANH", "FSIN", "FTAN", // 0x08-0x0f
	"FETOX", "FTWOTOX", "FTENTOX", NULL, "FLOGN", "FLOG10", "FLOG2", NULL, // 0x10-0x17
	"FABS", "FCOSH", "FNEG", NULL, "FACOS", "FCOS", "FGETEXP", "FGETMAN", // 0x18-0x1f
	"FDIV", "FMOD", "FADD", "FMUL", "FSGLDIV", "FREM", "FSCALE", "FSGLMUL", // 0x20-0x27
	"FSUB", NULL, NULL, NULL, NULL, NULL, NULL, NULL, // 0x28-0x2f
	"FSINCOS", "FSINCOS", "FSINCOS", "FSINCOS", "FSINCOS", "FSINCOS", "FSINCOS", "FSINCOS", // 0x30-0x37
	"FCMP", "FTST", NULL, NULL, NULL, NULL, NULL, NULL // 0x38-0x3f
};

static void fpgen_rm_reg(uint16 w2)
{
	int ea = REG_IR & 0x3f;
	int rm = (w2 >> 14) & 0x1;
	int src = (w2 >> 10) & 0x7;
	int dst = (w2 >>  7) & 0x7;
	int opmode = w2 & 0x7f;
	floatx80 source;
	int round;

	// Clear softfloat exception flags for fresh per-instruction tracking
	float_exception_flags = 0;

	// Check for FNOP: rm=0 (FP reg source), src==dst, opmode=0 (FMOVE)
	// FNOP should NOT clear fpu_just_reset as it doesn't change FPU state
	int is_fnop = (rm == 0 && src == dst && opmode == 0);
	if (!is_fnop) {
		m68ki_cpu.fpu_just_reset = 0;
	}

	if (FPU_TRACE) {
		const char *opname = (opmode < 0x40 && fpu_opmode_names[opmode]) ? fpu_opmode_names[opmode] : "???";
		fprintf(stderr, "[FPU]   ALU: rm=%d src=%d dst=FP%d opmode=%02x (%s)%s\n", rm, src, dst, opmode, opname, is_fnop ? " [FNOP]" : "");
	}

	// fmovecr #$f, fp0	f200 5c0f

	if (rm)
	{
		switch (src)
		{
			case 0:		// Long-Word Integer
			{
				sint32 d = READ_EA_32(ea);
				source = int32_to_floatx80(d);
				if (FPU_TRACE) fprintf(stderr, "[FPU]     Load Long: %08x -> %04x/%016llx\n", d, source.high, (unsigned long long)source.low);
				break;
			}
			case 1:		// Single-precision Real
			{
				uint32 d = READ_EA_32(ea);
				source = float32_to_floatx80(d);
				if (FPU_TRACE) fprintf(stderr, "[FPU]     Load Single: %08x -> %04x/%016llx\n", d, source.high, (unsigned long long)source.low);
				break;
			}
			case 2:		// Extended-precision Real
			{
	  	    	int imode = (ea >> 3) & 0x7;
	  	    	int reg = (ea & 0x7);
		      	uint32 di_mode_ea = imode == 5 ? (REG_A[reg]+MAKE_INT_16(m68ki_read_imm_16())) : 0;
		      	source = READ_EA_FPE(imode,reg,di_mode_ea);
		      	if (FPU_TRACE) fprintf(stderr, "[FPU]     Load Extended: %04x/%016llx\n", source.high, (unsigned long long)source.low);
			  	break;
			}
			case 3:		// Packed-decimal Real
			{
				source = READ_EA_PACK(ea);
				if (FPU_TRACE) fprintf(stderr, "[FPU]     Load Packed: -> %04x/%016llx\n", source.high, (unsigned long long)source.low);
				break;
			}
			case 4:		// Word Integer
			{
				sint16 d = READ_EA_16(ea);
				source = int32_to_floatx80((sint32)d);
				if (FPU_TRACE) fprintf(stderr, "[FPU]     Load Word: %04x -> %04x/%016llx\n", (uint16)d, source.high, (unsigned long long)source.low);
				break;
			}
			case 5:		// Double-precision Real
			{
				uint64 d = READ_EA_64(ea);

				source = float64_to_floatx80(d);
				if (FPU_TRACE) fprintf(stderr, "[FPU]     Load Double: %016llx -> %04x/%016llx\n", (unsigned long long)d, source.high, (unsigned long long)source.low);
				break;
			}
			case 6:		// Byte Integer
			{
				sint8 d = READ_EA_8(ea);
				source = int32_to_floatx80((sint32)d);
				if (FPU_TRACE) fprintf(stderr, "[FPU]     Load Byte: %02x -> %04x/%016llx\n", (uint8)d, source.high, (unsigned long long)source.low);
				break;
			}
			case 7:		// FMOVECR load from constant ROM
			{
				switch (w2 & 0x7f)
				{
					case 0x0:	// Pi
						source.high = 0x4000;
						source.low = U64(0xc90fdaa22168c235);
						break;

					case 0xb:	// log10(2)
						source.high = 0x3ffd;
						source.low = U64(0x9a209a84fbcff798);
						break;

					case 0xc:	// e
						source.high = 0x4000;
						source.low = U64(0xadf85458a2bb4a9b);
						break;

					case 0xd:	// log2(e)
						source.high = 0x3fff;
						source.low = U64(0xb8aa3b295c17f0bc);
						break;

					case 0xe:	// log10(e)
						source.high = 0x3ffd;
						source.low = U64(0xde5bd8a937287195);
						break;

					case 0xf:	// 0.0
						source = int32_to_floatx80((sint32)0);
						break;

					case 0x30:	// ln(2)
						source.high = 0x3ffe;
						source.low = U64(0xb17217f7d1cf79ac);
						break;

					case 0x31:	// ln(10)
						source.high = 0x4000;
						source.low = U64(0x935d8dddaaa8ac17);
						break;

					case 0x32:	// 1 (or 100?  manuals are unclear, but 1 would make more sense)
						source = int32_to_floatx80((sint32)1);
						break;

					case 0x33:	// 10^1
						source = int32_to_floatx80((sint32)10);
						break;

					case 0x34:	// 10^2
						source = int32_to_floatx80((sint32)10*10);
						break;

					case 0x35:	// 10^4
						source = int32_to_floatx80((sint32)10000);
						break;

					case 0x36:  // 10^8
						source = double_to_fx80(1e8);
						break;

					case 0x37:  // 10^16
						source = double_to_fx80(1e16);
						break;

					case 0x38:  // 10^32
						source = double_to_fx80(1e32);
						break;

					case 0x39:  // 10^64
						source = double_to_fx80(1e64);
						break;

					case 0x3a:  // 10^128
						source = double_to_fx80(1e128);
						break;

					case 0x3b:  // 10^256
						source = double_to_fx80(1e256);
						break;

					case 0x3c:  // 10^512
						source = double_to_fx80(1e256);
						source = floatx80_mul(source, source);
						break;

					case 0x3d:  // 10^1024
						source = double_to_fx80(1e256);
						source = floatx80_mul(source, source);
						source = floatx80_mul(source, source);
						break;

					case 0x3e:  // 10^2048
						source = double_to_fx80(1e256);
						source = floatx80_mul(source, source);
						source = floatx80_mul(source, source);
						source = floatx80_mul(source, source);
						break;

					case 0x3f:  // 10^4096
						source = double_to_fx80(1e256);
						source = floatx80_mul(source, source);
						source = floatx80_mul(source, source);
						source = floatx80_mul(source, source);
						source = floatx80_mul(source, source);
						break;

					default:
						source = int32_to_floatx80((sint32)0);
						break;
				}

				// handle it right here, the usual opmode bits aren't valid in the FMOVECR case
				REG_FP[dst] = source;
	     		SET_CONDITION_CODES(REG_FP[dst]); // JFF when destination is a register, we HAVE to update FPCR
				USE_CYCLES(4);
				return;
			}
			default:	fatalerror("fmove_rm_reg: invalid source specifier %x at %08X\n", src, REG_PC-4);
		}
	}
	else
	{
		source = REG_FP[src];
	}

	if ((opmode & 0x44) == 0x44)
	{
		round = 2;
		opmode &= ~0x44;
	} else if (opmode & 0x40)
	{
		round = 1;
		opmode &= ~0x40;
	} else
		round = 0;

	switch (opmode)
	{
		case 0x00:		// FMOVE
		{
			REG_FP[dst] = source;
		    SET_CONDITION_CODES(REG_FP[dst]);  // JFF needs update condition codes
			USE_CYCLES(4);
			break;
		}
		case 0x01:		// Fsint
		{
			sint32 temp;
			temp = floatx80_to_int32(source);
			REG_FP[dst] = int32_to_floatx80(temp);
	  		SET_CONDITION_CODES(REG_FP[dst]);  // JFF needs update condition codes
			break;
		}
		case 0x03:		// FsintRZ
		{
			sint32 temp;
			temp = floatx80_to_int32_round_to_zero(source);
			REG_FP[dst] = int32_to_floatx80(temp);
			SET_CONDITION_CODES(REG_FP[dst]);  // JFF needs update condition codes
			break;
		}
		case 0x04:		// FSQRT
		{
			REG_FP[dst] = floatx80_sqrt(source);
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(109);
			break;
		}
		case 0x18:		// FABS
		{
			REG_FP[dst] = source;
			REG_FP[dst].high &= 0x7fff;
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(3);
			break;
		}
		case 0x1a:		// FNEG
		{
			REG_FP[dst] = source;
			REG_FP[dst].high ^= 0x8000;
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(3);
			break;
		}
		case 0xe:		// SIN
			REG_FP[dst] = double_to_fx80(sin(fx80_to_double(source)));
	    	SET_CONDITION_CODES(REG_FP[dst]); // JFF
			USE_CYCLES(400);
			break;
		case 0x1d:		// COS
			REG_FP[dst] = double_to_fx80(cos(fx80_to_double(source)));
	    	SET_CONDITION_CODES(REG_FP[dst]); // JFF
			USE_CYCLES(400);
			break;
		case 0x30:		// SINCOS
		case 0x31:		// SINCOS
		case 0x32:		// SINCOS
		case 0x33:		// SINCOS
		case 0x34:		// SINCOS
		case 0x35:		// SINCOS
		case 0x36:		// SINCOS
		case 0x37:		// SINCOS
		{
			double ds = fx80_to_double(source);
			REG_FP[dst] = double_to_fx80(sin(ds));
			REG_FP[opmode&7] = double_to_fx80(cos(ds));
	    	SET_CONDITION_CODES(REG_FP[dst]); // JFF
			USE_CYCLES(400);
			break;
		}
		case 0x1e:		// FGETEXP
		{
			sint16 temp;
			temp = source.high;	// get the exponent
			temp -= 0x3fff;	// take off the bias
			REG_FP[dst] = double_to_fx80((double)temp);
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(6);
			break;
		}
		case 0x20:		// FDIV
		{
			REG_FP[dst] = floatx80_div(REG_FP[dst], source);
		    SET_CONDITION_CODES(REG_FP[dst]); // JFF
			USE_CYCLES(43);
			break;
		}
		case 0x21:		// FMOD
		{
			REG_FP[dst] = floatx80_rem(REG_FP[dst], source);
		    	SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(43);
			break;
		}
		case 0x24:		// FSGLDIV
		{
			REG_FP[dst] = double_to_fx80((float)fx80_to_double(floatx80_div(REG_FP[dst], source)));
		    	SET_CONDITION_CODES(REG_FP[dst]); // JFF
			USE_CYCLES(43);
			break;
		}
		case 0x22:		// FADD
		{
			REG_FP[dst] = floatx80_add(REG_FP[dst], source);
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(9);
			break;
		}
		case 0x23:		// FMUL
		{
			REG_FP[dst] = floatx80_mul(REG_FP[dst], source);
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(11);
			break;
		}
		case 0x27:		// FSGLMUL
		{
			REG_FP[dst] = double_to_fx80((float)fx80_to_double(floatx80_mul(REG_FP[dst], source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(11);
			break;
		}
		case 0x25:		// FREM
		{
			floatx80 before = REG_FP[dst];
			REG_FP[dst] = floatx80_rem(REG_FP[dst], source);
			if (FPU_TRACE) {
				fprintf(stderr, "[FPU]   FREM before: %04x/%016llx src: %04x/%016llx result: %04x/%016llx\n",
					before.high, (unsigned long long)before.low,
					source.high, (unsigned long long)source.low,
					REG_FP[dst].high, (unsigned long long)REG_FP[dst].low);
			}
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(43);	// guess
			break;
		}
		case 0x28:		// FSUB
		{
			REG_FP[dst] = floatx80_sub(REG_FP[dst], source);
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(9);
			break;
		}
		case 0x38:		// FCMP
		{
			floatx80 res;
			// handle inf in comparison if there is no nan.
			int d = is_inf(REG_FP[dst]);
			int s = is_inf(source);
			if (!floatx80_is_nan(REG_FP[dst]) && !floatx80_is_nan(source) && (d || s))
			{
				REG_FPSR &= ~(FPCC_N|FPCC_Z|FPCC_I|FPCC_NAN);

				if (s < 0) {
					if (d < 0)
						REG_FPSR |= FPCC_N | FPCC_Z;
				} else
				if (s > 0) {
					if (d > 0)
						REG_FPSR |= FPCC_Z;
					else
						REG_FPSR |= FPCC_N;
				} else
				if (d < 0)
					REG_FPSR |= FPCC_N;

			} else {
			res = floatx80_sub(REG_FP[dst], source);
			SET_CONDITION_CODES(res);
			}
			USE_CYCLES(7);
			break;
		}
		case 0x3a:		// FTST
		{
			floatx80 res;
			res = source;
			SET_CONDITION_CODES(res);
			USE_CYCLES(7);
			break;
		}

		case 0x02:		// FSINH
		{
			REG_FP[dst] = double_to_fx80(sinh(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x06:		// FLOGNP1
		{
			REG_FP[dst] = double_to_fx80(log1p(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(70);
			break;
		}
		case 0x08:		// FETOXM1
		{
			REG_FP[dst] = double_to_fx80(expm1(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(70);
			break;
		}
		case 0x09:		// FTANH
		{
			REG_FP[dst] = double_to_fx80(tanh(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x0a:		// FATAN
		{
			REG_FP[dst] = double_to_fx80(atan(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x0c:		// FASIN
		{
			REG_FP[dst] = double_to_fx80(asin(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x0d:		// FATANH
		{
			REG_FP[dst] = double_to_fx80(atanh(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x0f:		// FTAN
		{
			REG_FP[dst] = double_to_fx80(tan(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x10:		// FETOX
		{
			REG_FP[dst] = double_to_fx80(exp(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x11:		// FTWOTOX
		{
			REG_FP[dst] = double_to_fx80(exp2(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x12:		// FTENTOX
		{
			REG_FP[dst] = double_to_fx80(pow(10.0, fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x14:		// FLOGN
		{
			REG_FP[dst] = double_to_fx80(log(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x15:		// FLOG10
		{
			REG_FP[dst] = double_to_fx80(log10(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x16:		// FLOG2
		{
			REG_FP[dst] = double_to_fx80(log2(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x19:		// FCOSH
		{
			REG_FP[dst] = double_to_fx80(cosh(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		case 0x1c:		// FACOS
		{
			REG_FP[dst] = double_to_fx80(acos(fx80_to_double(source)));
			SET_CONDITION_CODES(REG_FP[dst]);
			USE_CYCLES(75);
			break;
		}
		default:	fatalerror("fpgen_rm_reg: unimplemented opmode %02X at %08X\n", opmode, REG_PC-4);
	}

	// Map softfloat exception flags to FPSR exception status bits.
	// Capture flags first — fpu_update_fpsr_exceptions consumes them.
	int had_sf_exceptions = float_exception_flags;
	fpu_update_fpsr_exceptions(source);

	if (round == 1)
	{
		// round to single
		REG_FP[dst] = double_to_fx80((float)fx80_to_double(REG_FP[dst]));
	} else if (round == 2)
	{
		// round to double
		REG_FP[dst] = double_to_fx80(fx80_to_double(REG_FP[dst]));
	}

	// If softfloat detected actual exceptions (not just denorm input detection),
	// check for enabled exceptions and take the trap immediately.
	// This handles DZ, OVFL, SNAN etc. from arithmetic operations.
	// Denorm-only cases (FMOVE.X of denorm) are deferred to FNOP.
	if (had_sf_exceptions || is_mc68882_snan(source))
		fpu_check_pending_exceptions();
}

static void fmove_reg_mem(uint16 w2)
{
	int ea = REG_IR & 0x3f;
	int src = (w2 >>  7) & 0x7;
	int dst = (w2 >> 10) & 0x7;
	int k = (w2 & 0x7f);

	// Clear softfloat flags — format conversions can trigger overflow/inexact
	float_exception_flags = 0;

	if (FPU_TRACE) {
		const char *dtypes[] = {"Long", "Single", "Extended", "Packed", "Word", "Double", "Byte", "PackedK"};
		fprintf(stderr, "[FPU]   FMOVE FP%d->%s: %04x/%016llx\n",
			src, dtypes[dst], REG_FP[src].high, (unsigned long long)REG_FP[src].low);
	}

	switch (dst)
	{
		case 0:		// Long-Word Integer
		{
			sint32 d = (sint32)floatx80_to_int32(REG_FP[src]);
			if (FPU_TRACE) fprintf(stderr, "[FPU]     -> Store Long: %08x\n", d);
			WRITE_EA_32(ea, d);
			break;
		}
		case 1:		// Single-precision Real
		{
			uint32 d = floatx80_to_float32(REG_FP[src]);
			if (FPU_TRACE) fprintf(stderr, "[FPU]     -> Store Single: %08x\n", d);
			WRITE_EA_32(ea, d);
			break;
		}
		case 2:		// Extended-precision Real
		{
		  	int mode = (ea >> 3) & 0x7;
		  	int reg = (ea & 0x7);
		  	uint32 di_mode_ea = mode == 5 ? (REG_A[reg]+MAKE_INT_16(m68ki_read_imm_16())) : 0;
			WRITE_EA_FPE(mode, reg, REG_FP[src], di_mode_ea);
			break;
		}
		case 3:		// Packed-decimal Real with Static K-factor
		{
			// sign-extend k
			k = (k & 0x40) ? (k | 0xffffff80) : (k & 0x7f);
			WRITE_EA_PACK(ea, k, REG_FP[src]);
			break;
		}
		case 4:		// Word Integer
		{
			sint32 d = floatx80_to_int32(REG_FP[src]);
			if (FPU_TRACE) fprintf(stderr, "[FPU]     -> Store Word: %04x\n", (uint16)(sint16)d);
			WRITE_EA_16(ea, (sint16)d);
			break;
		}
		case 5:		// Double-precision Real
		{
			uint64 d;

			d = floatx80_to_float64(REG_FP[src]);
			if (FPU_TRACE) fprintf(stderr, "[FPU]     -> Store Double: %016llx\n", (unsigned long long)d);

			WRITE_EA_64(ea, d);
			break;
		}
		case 6:		// Byte Integer
		{
			sint32 d = floatx80_to_int32(REG_FP[src]);
			if (FPU_TRACE) fprintf(stderr, "[FPU]     -> Store Byte: %02x\n", (uint8)(sint8)d);
			WRITE_EA_8(ea, (sint8)d);
			break;
		}
		case 7:		// Packed-decimal Real with Dynamic K-factor
		{
			WRITE_EA_PACK(ea, REG_D[k>>4], REG_FP[src]);
			break;
		}
	}

	// Map softfloat exception flags from format conversions
	int had_sf_exceptions = float_exception_flags;
	fpu_update_fpsr_exceptions(REG_FP[src]);

	// If format conversion triggered exceptions, check for enabled traps
	if (had_sf_exceptions)
		fpu_check_pending_exceptions();

	USE_CYCLES(12);
}

static void fmove_fpcr(uint16 w2)
{
	int ea = REG_IR & 0x3f;
	int dir = (w2 >> 13) & 0x1;
	int reg = (w2 >> 10) & 0x7;
	int imode = (ea >> 3) & 0x7;
	int areg = ea & 0x7;

	// Count how many registers are being transferred
	int num_regs = ((reg >> 2) & 1) + ((reg >> 1) & 1) + (reg & 1);

	// For multi-register transfers, addressing modes that read extension words from
	// the instruction stream (modes 5, 6, 7) must compute the EA once, not per-register.
	// Mode 2 (An) also needs this to avoid writing all regs to the same address.
	// Modes 3/4 (post-increment/pre-decrement) correctly auto-modify the address register.
	int needs_local_ea = (num_regs > 1) && (imode == 2 || imode == 5 || imode == 6 || imode == 7);
	uint32 local_ea = 0;

	if (needs_local_ea) {
		switch (imode) {
			case 2: local_ea = REG_A[areg]; break;
			case 5: local_ea = EA_AY_DI_32(); break;
			case 6: local_ea = EA_AY_IX_32(); break;
			case 7:
				switch (areg) {
					case 0: local_ea = (uint32)(int16)OPER_I_16(); break;
					case 1: { uint32 d1 = OPER_I_16(); uint32 d2 = OPER_I_16(); local_ea = (d1 << 16) | d2; } break;
					case 2: local_ea = EA_PCDI_32(); break;
					default: break;
				}
				break;
		}
	}

	if (FPU_TRACE) {
		fprintf(stderr, "[FPU]   FMOVE_FPCR dir=%d reg=%d local_ea=%d ea=%08x (FPCR=%08x FPSR=%08x FPIAR=%08x)\n",
			dir, reg, needs_local_ea, needs_local_ea ? local_ea : 0, REG_FPCR, REG_FPSR, REG_FPIAR);
	}

	if (dir)	// From system control reg to <ea>
	{
		if (needs_local_ea)
		{
			if (reg & 4) { m68ki_write_32(local_ea, REG_FPCR); local_ea += 4; }
			if (reg & 2) { m68ki_write_32(local_ea, REG_FPSR); local_ea += 4; }
			if (reg & 1) { m68ki_write_32(local_ea, REG_FPIAR); local_ea += 4; }
		}
		else
		{
			if (reg & 4) WRITE_EA_32(ea, REG_FPCR);
			if (reg & 2) WRITE_EA_32(ea, REG_FPSR);
			if (reg & 1) WRITE_EA_32(ea, REG_FPIAR);
		}
	}
	else		// From <ea> to system control reg
	{
		if (needs_local_ea)
		{
			if (reg & 4) { REG_FPCR = m68ki_read_32(local_ea); local_ea += 4; float_rounding_mode = (REG_FPCR >> 4) & 0x3; }
			if (reg & 2) { REG_FPSR = m68ki_read_32(local_ea); local_ea += 4; }
			if (reg & 1) { REG_FPIAR = m68ki_read_32(local_ea); local_ea += 4; }
		}
		else
		{
			if (reg & 4) { REG_FPCR = READ_EA_32(ea); float_rounding_mode = (REG_FPCR >> 4) & 0x3; }
			if (reg & 2) REG_FPSR = READ_EA_32(ea);
			if (reg & 1) REG_FPIAR = READ_EA_32(ea);
		}
	}

	if (FPU_TRACE) {
		fprintf(stderr, "[FPU]   -> FPCR=%08x FPSR=%08x FPIAR=%08x\n",
			REG_FPCR, REG_FPSR, REG_FPIAR);
	}

	USE_CYCLES(10);
}

static void fmovem(uint16 w2)
{
	int i;
	int ea = REG_IR & 0x3f;
	int dir = (w2 >> 13) & 0x1;
	int mode = (w2 >> 11) & 0x3;
	int reglist = w2 & 0xff;

	if (FPU_TRACE) {
		int actual_list = reglist;
		if (mode == 1 || mode == 3) {
			int list_reg = (reglist >> 4) & 0x7;
			actual_list = REG_D[list_reg] & 0xff;
			fprintf(stderr, "[FPU]   FMOVEM dir=%d mode=%d reglist=%02x(D%d=%02x) ea=%02x\n",
				dir, mode, reglist, list_reg, actual_list, ea);
		} else {
			fprintf(stderr, "[FPU]   FMOVEM dir=%d mode=%d reglist=%02x ea=%02x\n", dir, mode, reglist, ea);
		}
	}

	// MC68882 FMOVEM direction encoding:
	// Extension word bit 13 = 1 (bits 15-13 = 111): register to memory (STORE)
	// Extension word bit 13 = 0 (bits 15-13 = 110): memory to register (LOAD)
	if (dir)	// dir=1: register to memory (STORE)
	{
		switch (mode)
	{
	  	case 2:		// Static register list, postincrement or control addressing mode
	    {
	      // For postincrement: bit 0 = FP0, bit 7 = FP7
	      int imode = (ea >> 3) & 0x7;
	      int reg = (ea & 0x7);
	      int di_mode = imode == 5;
	      // Control addressing mode (An): imode==2 means we need to manually track address
	      // Postincrement (An)+: imode==3 is handled by WRITE_EA_FPE
	      int control_mode = (imode == 2);
	      uint32 di_mode_ea = di_mode ? (REG_A[reg]+MAKE_INT_16(m68ki_read_imm_16())) : 0;
	      uint32 control_ea = control_mode ? REG_A[reg] : 0;
	      for (i=0; i < 8; i++)
			{
			  if (reglist & (1 << i))
			    {
			      if (FPU_TRACE) fprintf(stderr, "[FPU]     store FP%d: %04x/%016llx\n", i, REG_FP[i].high, (unsigned long long)REG_FP[i].low);
			      if (control_mode)
				{
				  store_extended_float80(control_ea, REG_FP[i]);
				  control_ea += 12;  // Increment local EA, not A[n]
				}
			      else
				{
				  WRITE_EA_FPE(imode,reg, REG_FP[i],di_mode_ea);
				}
			      USE_CYCLES(2);
			      if (di_mode)
				{
				  di_mode_ea += 12;
				}
		    	}
			}
	      break;
	   	 }
			case 0:		// Static register list, predecrement addressing mode
			{
	      // For predecrement: bit 0 = FP7, bit 7 = FP0
	      int imode = (ea >> 3) & 0x7;
	      int reg = (ea & 0x7);
	      int di_mode = imode == 5;
	      uint32 di_mode_ea =  di_mode ? (REG_A[reg]+MAKE_INT_16(m68ki_read_imm_16())) : 0;
	      if (FPU_TRACE) fprintf(stderr, "[FPU]     mode0 A%d=%08x imode=%d di=%d\n", reg, REG_A[reg], imode, di_mode);
				for (i=0; i < 8; i++)
				{
					if (reglist & (1 << i))
					{
						if (FPU_TRACE) fprintf(stderr, "[FPU]     store FP%d: %04x/%016llx (bit%d)\n", 7-i, REG_FP[7-i].high, (unsigned long long)REG_FP[7-i].low, i);
		 			    WRITE_EA_FPE(imode,reg, REG_FP[7-i],di_mode_ea);
						USE_CYCLES(2);
					    if (di_mode)
						{
						  di_mode_ea += 12;
						}
					}
				}
				break;
			}

			case 1:		// Dynamic register list, predecrement addressing mode
			{
				// For predecrement: bit 0 = FP7, bit 7 = FP0
				// Register list comes from data register specified in bits 6-4 of reglist field
				int list_reg = (reglist >> 4) & 0x7;
				int dynamic_list = REG_D[list_reg] & 0xff;
				int imode = (ea >> 3) & 0x7;
				int reg = (ea & 0x7);
				int di_mode = imode == 5;
				uint32 di_mode_ea = di_mode ? (REG_A[reg]+MAKE_INT_16(m68ki_read_imm_16())) : 0;
				for (i=0; i < 8; i++)
				{
					if (dynamic_list & (1 << i))
					{
						WRITE_EA_FPE(imode,reg, REG_FP[7-i],di_mode_ea);
						USE_CYCLES(2);
						if (di_mode)
						{
							di_mode_ea += 12;
						}
					}
				}
				break;
			}

			case 3:		// Dynamic register list, postincrement or control addressing mode
			{
				// For postincrement: bit 0 = FP0, bit 7 = FP7
				// Register list comes from data register specified in bits 6-4 of reglist field
				int list_reg = (reglist >> 4) & 0x7;
				int dynamic_list = REG_D[list_reg] & 0xff;
				int imode = (ea >> 3) & 0x7;
				int reg = (ea & 0x7);
				int di_mode = imode == 5;
				// Control addressing mode (An): imode==2 means we need to manually track address
				// Postincrement (An)+: imode==3 is handled by WRITE_EA_FPE
				int control_mode = (imode == 2);
				uint32 di_mode_ea = di_mode ? (REG_A[reg]+MAKE_INT_16(m68ki_read_imm_16())) : 0;
				uint32 control_ea = control_mode ? REG_A[reg] : 0;
				for (i=0; i < 8; i++)
				{
					if (dynamic_list & (1 << i))
					{
						if (control_mode)
						{
							store_extended_float80(control_ea, REG_FP[i]);
							control_ea += 12;  // Increment local EA, not A[n]
						}
						else
						{
							WRITE_EA_FPE(imode,reg, REG_FP[i],di_mode_ea);
						}
						USE_CYCLES(2);
						if (di_mode)
						{
							di_mode_ea += 12;
						}
					}
				}
				break;
			}

			default:	fatalerror("040fpu0: FMOVEM: mode %d unimplemented at %08X\n", mode, REG_PC-4);
		}
	}
	else		// dir=0: memory to register (LOAD)
	{
		switch (mode)
		{
			case 0:		// Static register list, predecrement addressing mode
			{
				// For predecrement: bit 0 = FP7, bit 7 = FP0
				int imode = (ea >> 3) & 0x7;
				int reg = (ea & 0x7);
				int di_mode = imode == 5;
				uint32 di_mode_ea = di_mode ? (REG_A[reg]+MAKE_INT_16(m68ki_read_imm_16())) : 0;
				for (i=0; i < 8; i++)
				{
					if (reglist & (1 << i))
					{
						REG_FP[7-i] = READ_EA_FPE(imode,reg,di_mode_ea);
						USE_CYCLES(2);
						if (di_mode)
						{
							di_mode_ea += 12;
						}
					}
				}
				break;
			}

			case 1:		// Dynamic register list, predecrement addressing mode
			{
				// For predecrement: bit 0 = FP7, bit 7 = FP0
				// Register list comes from data register specified in bits 6-4 of reglist field
				int list_reg = (reglist >> 4) & 0x7;
				int dynamic_list = REG_D[list_reg] & 0xff;
				int imode = (ea >> 3) & 0x7;
				int reg = (ea & 0x7);
				int di_mode = imode == 5;
				uint32 di_mode_ea = di_mode ? (REG_A[reg]+MAKE_INT_16(m68ki_read_imm_16())) : 0;
				for (i=0; i < 8; i++)
				{
					if (dynamic_list & (1 << i))
					{
						REG_FP[7-i] = READ_EA_FPE(imode,reg,di_mode_ea);
						USE_CYCLES(2);
						if (di_mode)
						{
							di_mode_ea += 12;
						}
					}
				}
				break;
			}

			case 2:		// Static register list, postincrement or control addressing mode
			{
				// For postincrement: bit 0 = FP0, bit 7 = FP7
				int imode = (ea >> 3) & 0x7;
				int reg = (ea & 0x7);
				int di_mode = imode == 5;
				// Control addressing mode (An): need to manually track address without modifying An
				// For postincrement (An)+: READ_EA_FPE handles the increment
				int control_mode = (imode == 2);  // (An) is control addressing mode
				uint32 di_mode_ea = di_mode ? (REG_A[reg]+MAKE_INT_16(m68ki_read_imm_16())) : 0;
				uint32 control_ea = control_mode ? REG_A[reg] : 0;  // Local EA for control mode
				for (i=0; i < 8; i++)
				{
					if (reglist & (1 << i))
					{
						if (control_mode)
						{
							REG_FP[i] = load_extended_float80(control_ea);
							control_ea += 12;  // Increment local EA, not A[n]
						}
						else
						{
							REG_FP[i] = READ_EA_FPE(imode,reg,di_mode_ea);
						}
						USE_CYCLES(2);
						if (di_mode)
						{
							di_mode_ea += 12;
						}
					}
				}
				break;
			}

			case 3:		// Dynamic register list, postincrement or control addressing mode
			{
				// For postincrement: bit 0 = FP0, bit 7 = FP7
				// Register list comes from data register specified in bits 6-4 of reglist field
				int list_reg = (reglist >> 4) & 0x7;
				int dynamic_list = REG_D[list_reg] & 0xff;
				int imode = (ea >> 3) & 0x7;
				int reg = (ea & 0x7);
				int di_mode = imode == 5;
				// Control addressing mode (An): need to manually track address without modifying An
				// For postincrement (An)+: READ_EA_FPE handles the increment
				int control_mode = (imode == 2);  // (An) is control addressing mode
				uint32 di_mode_ea = di_mode ? (REG_A[reg]+MAKE_INT_16(m68ki_read_imm_16())) : 0;
				uint32 control_ea = control_mode ? REG_A[reg] : 0;  // Local EA for control mode
				for (i=0; i < 8; i++)
				{
					if (dynamic_list & (1 << i))
					{
						if (control_mode)
						{
							REG_FP[i] = load_extended_float80(control_ea);
							control_ea += 12;  // Increment local EA, not A[n]
						}
						else
						{
							REG_FP[i] = READ_EA_FPE(imode,reg,di_mode_ea);
						}
						USE_CYCLES(2);
						if (di_mode)
						{
							di_mode_ea += 12;
						}
					}
				}
				break;
			}

			default:	fatalerror("040fpu0: FMOVEM: mode %d unimplemented at %08X\n", mode, REG_PC-4);
		}
	}
}

static void fscc()
{
  // added by JFF, this seems to work properly now
  uint32 pc_before = REG_PC;
  uint16 raw_word = OPER_I_16();
  int condition = raw_word & 0x3f;
  if (FPU_TRACE) {
    /* Also read directly from memory to compare */
    uint16 mem_word = m68ki_read_16(pc_before);
    uint16 next_word = m68ki_read_16(pc_before + 2);
    fprintf(stderr, "[FPU]   FScc: PC_before=%08x raw_word=%04x mem_word=%04x next=%04x condition=%02x PC_after=%08x\n",
      pc_before, raw_word, mem_word, next_word, condition, REG_PC);
  }

  int cc = TEST_CONDITION(condition);

  /* If BSUN exception was taken, don't complete the instruction */
  if (cc == -1) {
    return;
  }

  int mode = (REG_IR & 0x38) >> 3;
  int reg = REG_IR & 7;
  int v = (cc ? 0xff : 0x00);

  if (FPU_TRACE) {
    fprintf(stderr, "[FPU]   FScc cond=%02x cc=%d mode=%d reg=%d v=%02x A7=%08x\n",
      condition, cc, mode, reg, v, REG_A[7]);
  }

  switch (mode)
  {
  case 0:  // fscc Dx
    {
      // If the specified floating-point condition is true, sets the byte integer operand at
      // the destination to TRUE (all ones); otherwise, sets the byte to FALSE (all zeros).

      REG_D[reg] = (REG_D[reg] & 0xFFFFFF00) | v;
      break;
    }
    case 2:  // (An) - Address register indirect
    {
      uint32 ea = REG_A[reg];
      m68ki_write_8(ea, v);
      break;
    }
    case 3:  // (An)+ - Address register indirect with postincrement
    {
      uint32 ea = REG_A[reg];
      m68ki_write_8(ea, v);
      // A7 must stay word-aligned, so increment by 2 for byte operations
      REG_A[reg] += (reg == 7) ? 2 : 1;
      break;
    }
    case 4:  // -(An) - Address register indirect with predecrement
    {
      // A7 must stay word-aligned, so decrement by 2 for byte operations
      REG_A[reg] -= (reg == 7) ? 2 : 1;
      uint32 ea = REG_A[reg];
      if (FPU_TRACE && reg == 7) {
        fprintf(stderr, "[FPU]   FScc -(A7) write 0x%02x to %08x, next PC=%08x\n", v, ea, REG_PC);
        /* Dump next few words at PC to see what instruction follows */
        fprintf(stderr, "[FPU]   Next ROM: PC=%08x: %04x %04x %04x %04x\n",
          REG_PC, m68ki_read_16(REG_PC), m68ki_read_16(REG_PC+2),
          m68ki_read_16(REG_PC+4), m68ki_read_16(REG_PC+6));
      }
      m68ki_write_8(ea, v);
      break;
    }
    case 5: // (disp,Ax) - Address register indirect with displacement
    {
      uint32 ea = REG_A[reg] + MAKE_INT_16(m68ki_read_imm_16());
      m68ki_write_8(ea, v);
      break;
    }
    case 6:  // (d8,An,Xn) - Address register indirect with index
    {
      uint32 ea = EA_AY_IX_8();
      m68ki_write_8(ea, v);
      break;
    }
    case 7:  // Absolute addressing modes
    {
      uint32 ea;
      switch (reg)
      {
        case 0:  // (xxx).W - Absolute short
          ea = MAKE_INT_16(m68ki_read_imm_16());
          break;
        case 1:  // (xxx).L - Absolute long
          ea = m68ki_read_imm_32();
          break;
        default:
          fatalerror("040fpu0: fscc: mode 7 reg %d not implemented at %08X\n", reg, REG_PC-4);
          return;
      }
      m68ki_write_8(ea, v);
      break;
    }

  default:
    {
      // unimplemented see fpu_uae.cpp around line 1300
      fatalerror("040fpu0: fscc: mode %d not implemented at %08X\n", mode, REG_PC-4);
    }
    }
  USE_CYCLES(7);  // JFF unsure of the number of cycles!!
}

static void fbcc16(void)
{
	sint32 offset;
	int condition = REG_IR & 0x3f;

	offset = (sint16)(OPER_I_16());

	int cc = TEST_CONDITION(condition);
	/* If BSUN exception was taken, don't complete the instruction */
	if (cc == -1) {
		return;
	}

	// MC68882: FBcc (including FNOP = FBF.W #0) triggers processing of
	// pending enabled exceptions (deferred from denorm loads, etc.)
	fpu_check_pending_exceptions();

	if (cc)
	{
		m68ki_trace_t0();			   /* auto-disable (see m68kcpu.h) */
		m68ki_branch_16(offset-2);
	}

	USE_CYCLES(7);
}

static void fbcc32(void)
{
	sint32 offset;
	int condition = REG_IR & 0x3f;

	offset = OPER_I_32();

	int cc = TEST_CONDITION(condition);
	/* If BSUN exception was taken, don't complete the instruction */
	if (cc == -1) {
		return;
	}

	fpu_check_pending_exceptions();

	if (cc)
	{
		m68ki_trace_t0();			   /* auto-disable (see m68kcpu.h) */
		m68ki_branch_32(offset-4);
	}

	USE_CYCLES(7);
}


void m68040_fpu_op0()
{
	// NOTE: fpu_just_reset is now cleared only for operations that actually use FPU state
	// (not cleared here at the start, to allow FNOP->FSAVE to generate NULL frame)

	// Trace early boot FPU operations (first 100)
	static int early_fpu_count = 0;
	if (TRACE_EARLY_FPU && early_fpu_count < 100) {
		fprintf(stderr, "[EARLY-FPU] PC=%08x IR=%04x op=%d fpu_just_reset=%d\n",
			REG_PC-2, REG_IR, (REG_IR >> 6) & 0x3, m68ki_cpu.fpu_just_reset);
		early_fpu_count++;
	}

	/* Trace specific PC for debugging */
	int trace_this = 0; // (REG_PC-2 == 0xfff0f72e);

	if (FPU_TRACE || trace_this) {
		fprintf(stderr, "[FPU] PC=%08x IR=%04x op=%d A7=%08x\n", REG_PC-2, REG_IR, (REG_IR >> 6) & 0x3, REG_A[7]);
		g_fpu_trace_count++;
	}

	switch ((REG_IR >> 6) & 0x3)
	{
		case 0:
		{
			uint16 w2 = OPER_I_16();
			if (FPU_TRACE || trace_this) {
				fprintf(stderr, "[FPU]   w2=%04x subop=%d A7=%08x\n", w2, (w2 >> 13) & 0x7, REG_A[7]);
			}
			switch ((w2 >> 13) & 0x7)
			{
				case 0x0:	// FPU ALU FP, FP
				case 0x2:	// FPU ALU ea, FP
				{
					// Note: fpu_just_reset is cleared inside fpgen_rm_reg
					// (except for FNOP which should preserve it)
					fpgen_rm_reg(w2);
					break;
				}

				case 0x3:	// FMOVE FP, ea
				{
					m68ki_cpu.fpu_just_reset = 0;  // Reading FP register uses FPU state
					fmove_reg_mem(w2);
					break;
				}

				case 0x4:	// FMOVEM ea, FPCR
				case 0x5:	// FMOVEM FPCR, ea
				{
					// Control register transfers don't use FP data register state
					fmove_fpcr(w2);
					break;
				}

				case 0x6:	// FMOVEM list, ea (register to memory, bits 15-13=110)
				case 0x7:	// FMOVEM ea, list (memory to register, bits 15-13=111)
				{
					m68ki_cpu.fpu_just_reset = 0;  // FP register transfers use FPU state
					fmovem(w2);
					break;
				}

				default:	fatalerror("M68kFPU: unimplemented subop %d at %08X\n", (w2 >> 13) & 0x7, REG_PC-4);
			}
			break;
		}

	    case 1:           // FScc (JFF)
		{
		  fscc();
		  break;
		}
		case 2:		// FBcc disp16
		{
			fbcc16();
			break;
		}
		case 3:		// FBcc disp32
		{
			fbcc32();
			break;
		}

      default:	fatalerror("M68kFPU: unimplemented main op %d at %08X\n", (m68ki_cpu.ir >> 6) & 0x3,  REG_PC-4);
	}
}

// Get IDLE frame size in bytes based on FPU model
// MC68881: 0x18 (24 bytes, 6 longs), MC68882: 0x38 (56 bytes, 14 longs)
static int fpu_idle_frame_size(void)
{
	return (g_fpu_model_version == 0x41) ? 0x38 : 0x18;
}

// FSAVE writes a standard-size IDLE frame to memory AND saves FPU user
// registers to an address-keyed save slot.  Multiple slots support nested
// FSAVE/FRESTORE pairs (e.g., in nested TRAP handlers).
//
// The frame data area is filled with actual FPU register state rather than
// zeros, matching real MC68882 behavior where the IDLE frame contains
// internal FPU state.  This avoids corrupting adjacent memory with zero
// bytes (the 147Bug FPC test stores a halt-on-error flag immediately
// above the FSAVE destination, and a zero byte there causes a crash).
static void perform_fsave(uint32 addr, int inc)
{
	int frame_size = fpu_idle_frame_size();
	int data_longs = frame_size / 4;
	uint32 header = ((uint32)g_fpu_model_version << 24) | ((uint32)frame_size << 16);

	// Build frame data from FPU register state.  Real MC68882 stores
	// proprietary internal state; we store user-visible registers in a
	// fixed layout.  FRESTORE uses save slots (not this data) to restore.
	uint32 fdata[14]; // max data_longs for MC68882 (0x38/4 = 14)
	// Control registers
	fdata[0] = REG_FPCR;
	fdata[1] = REG_FPSR;
	fdata[2] = REG_FPIAR;
	// FP0-FP3: 3 longwords each (high<<16, low_hi, low_lo), bounds-checked
	for (int i = 0; i < 4; i++)
	{
		int base = 3 + i * 3;
		if (base < data_longs)
			fdata[base] = (uint32)m68ki_cpu.fpr[i].high << 16;
		if (base + 1 < data_longs)
			fdata[base + 1] = (uint32)(m68ki_cpu.fpr[i].low >> 32);
		if (base + 2 < data_longs)
			fdata[base + 2] = (uint32)(m68ki_cpu.fpr[i].low & 0xFFFFFFFFULL);
	}
	// Ensure no byte in the last longword is zero — its high byte lands
	// at the highest frame address and must not corrupt adjacent memory
	fdata[data_longs - 1] |= 0x01010101;

	// Compute the header address and write frame to memory
	uint32 hdr_addr;
	if (inc)
	{
		// Forward: header at addr, data follows
		hdr_addr = addr;
		m68ki_write_32(addr, header);
		for (int i = 0; i < data_longs; i++)
			m68ki_write_32(addr + 4 + i*4, fdata[i]);
	}
	else
	{
		// Predecrement: data[last] at addr, data[0] at lowest, header below
		hdr_addr = addr - data_longs*4;
		for (int i = 0; i < data_longs; i++)
			m68ki_write_32(addr - i*4, fdata[data_longs - 1 - i]);
		m68ki_write_32(hdr_addr, header);
	}

	// Save user-visible registers to a slot keyed by frame address
	int slot = -1;
	for (int i = 0; i < FPU_SAVE_SLOTS; i++)
	{
		if (!m68ki_cpu.fpu_saves[i].valid) { slot = i; break; }
	}
	if (slot < 0) slot = 0; // evict oldest if all full
	m68ki_cpu.fpu_saves[slot].frame_addr = hdr_addr;
	for (int i = 0; i < 8; i++)
		m68ki_cpu.fpu_saves[slot].fpr[i] = m68ki_cpu.fpr[i];
	m68ki_cpu.fpu_saves[slot].fpcr  = REG_FPCR;
	m68ki_cpu.fpu_saves[slot].fpsr  = REG_FPSR;
	m68ki_cpu.fpu_saves[slot].fpiar = REG_FPIAR;
	m68ki_cpu.fpu_saves[slot].valid = 1;
}

// FRESTORE IDLE: restore FPU user registers from the save slot matching
// the given frame header address.
static void perform_frestore_idle(uint32 hdr_addr)
{
	for (int i = 0; i < FPU_SAVE_SLOTS; i++)
	{
		if (m68ki_cpu.fpu_saves[i].valid && m68ki_cpu.fpu_saves[i].frame_addr == hdr_addr)
		{
			for (int j = 0; j < 8; j++)
				m68ki_cpu.fpr[j] = m68ki_cpu.fpu_saves[i].fpr[j];
			REG_FPCR  = m68ki_cpu.fpu_saves[i].fpcr;
			REG_FPSR  = m68ki_cpu.fpu_saves[i].fpsr;
			REG_FPIAR = m68ki_cpu.fpu_saves[i].fpiar;
			m68ki_cpu.fpu_saves[i].valid = 0; // consumed
			return;
		}
	}
	// No matching slot found — leave registers unchanged (original behavior)
}

// FRESTORE null: reset FPU to power-on state.
// On real MC68882, FRESTORE with a null frame resets the entire FPU:
// FP0-FP7 = NaN, FPCR/FPSR/FPIAR = 0, subsequent FSAVE produces null frame.
// fpu_just_reset MUST be set so the next FSAVE produces a null frame (4 bytes)
// instead of an IDLE frame — the FPC test allocates scratch space relative to
// A6, and a spurious IDLE frame (60 bytes) would overflow that space and
// corrupt the halt-on-error flag and custom vector table.
static void do_frestore_null()
{
	for (int i = 0; i < 8; i++)
	{
		m68ki_cpu.fpr[i].high = 0x7FFF;
		m68ki_cpu.fpr[i].low  = 0xFFFFFFFFFFFFFFFFULL;
	}
	REG_FPCR  = 0;
	REG_FPSR  = 0;
	REG_FPIAR = 0;
	m68ki_cpu.fpu_just_reset = 1;
}

void m68040_fpu_op1()
{
	int ea = REG_IR & 0x3f;
	int mode = (ea >> 3) & 0x7;
	int reg = (ea & 0x7);
	uint32 addr = 0, temp;

	// Conditional trace for FSAVE/FRESTORE
	if (TRACE_FSAVE) {
		const char *opnames[] = {"FSAVE", "FRESTORE", "???", "???"};
		fprintf(stderr, "[FPU-STATE] PC=%08x IR=%04x %s mode=%d reg=%d A[reg]=%08x just_reset=%d\n",
			REG_PC-2, REG_IR, opnames[(REG_IR >> 6) & 0x3], mode, reg, REG_A[reg], m68ki_cpu.fpu_just_reset);
	}

	switch ((REG_IR >> 6) & 0x3)
	{
		case 0:		// FSAVE <ea>
		{
			switch (mode)
			{
				case 2: // (An)
					addr = REG_A[reg];

					if (m68ki_cpu.fpu_just_reset)
					{
						m68ki_write_32(addr, 0);
					}
					else
					{
						// we normally generate an IDLE frame
						perform_fsave(addr, 1);
					}
					break;

				case 3:	// (An)+
		    			addr = EA_AY_PI_32();

					if (m68ki_cpu.fpu_just_reset)
					{
						m68ki_write_32(addr, 0);
					}
					else
					{
						// we normally generate an IDLE frame
						REG_A[reg] += fpu_idle_frame_size();
						perform_fsave(addr, 1);
					}
					break;

				case 4: // -(An)
		    			addr = EA_AY_PD_32();

					if (TRACE_FSAVE) {
						fprintf(stderr, "[FSAVE -(An)] addr=%08x A[%d]=%08x just_reset=%d\n", addr, reg, REG_A[reg], m68ki_cpu.fpu_just_reset);
					}

					if (m68ki_cpu.fpu_just_reset)
					{
						m68ki_write_32(addr, 0);
					}
					else
					{
						// we normally generate an IDLE frame
						REG_A[reg] -= fpu_idle_frame_size();
						if (TRACE_FSAVE) {
							fprintf(stderr, "[FSAVE -(An)] after decrement A[%d]=%08x, writing IDLE frame\n", reg, REG_A[reg]);
						}
						perform_fsave(addr, 0);
					}
					break;

				case 5: // (d16, An)
					addr = EA_AY_DI_32();

					if (m68ki_cpu.fpu_just_reset)
					{
						m68ki_write_32(addr, 0);
					}
					else
					{
						// we normally generate an IDLE frame
						perform_fsave(addr, 1);
					}
					break;

				case 6: // (d8, An, Xn)
					addr = EA_AY_IX_32();

					if (TRACE_FSAVE) {
						fprintf(stderr, "[FSAVE-M6] PC=%08x addr=%08x A7=%08x just_reset=%d\n",
							REG_PC, addr, REG_A[7], m68ki_cpu.fpu_just_reset);
					}

					if (m68ki_cpu.fpu_just_reset)
					{
						m68ki_write_32(addr, 0);
					}
					else
					{
						perform_fsave(addr, 1);
					}
					break;

				case 7: // Absolute and PC-relative
					switch (reg)
					{
						case 0: addr = EA_AW_32(); break;
						case 1: addr = EA_AL_32(); break;
						default:
							fatalerror("M68kFPU: FSAVE unhandled mode 7 reg %d at %x\n", reg, REG_PC);
					}

					if (m68ki_cpu.fpu_just_reset)
					{
						m68ki_write_32(addr, 0);
					}
					else
					{
						perform_fsave(addr, 1);
					}
					break;

				default:
					fatalerror("M68kFPU: FSAVE unhandled mode %d reg %d at %x\n", mode, reg, REG_PC);
			}
			break;
		}
		break;

		case 1:		// FRESTORE <ea>
		{
			switch (mode)
			{
				case 2: // (An)
					addr = REG_A[reg];
					temp = m68ki_read_32(addr);

					if (temp & 0xff000000)
					{
						if (((temp >> 16) & 0xFF) == fpu_idle_frame_size())
							perform_frestore_idle(addr);
						m68ki_cpu.fpu_just_reset = 0;
					}
					else
					{
						do_frestore_null();
					}
					break;

				case 3:	// (An)+
		    			addr = EA_AY_PI_32();
					temp = m68ki_read_32(addr);

					if (TRACE_FSAVE) {
						fprintf(stderr, "[FRESTORE] addr=%08x temp=%08x A[%d]=%08x\n", addr, temp, reg, REG_A[reg]);
					}

					if (temp & 0xff000000)
					{
						int frame_data_size = (temp >> 16) & 0xFF;
						if (frame_data_size == fpu_idle_frame_size())
							perform_frestore_idle(addr);
						m68ki_cpu.fpu_just_reset = 0;
						REG_A[reg] += frame_data_size;
					}
					else
					{
						do_frestore_null();
					}
					break;

				case 5: // (d16, An)
					addr = EA_AY_DI_32();
					temp = m68ki_read_32(addr);

					if (temp & 0xff000000)
					{
						if (((temp >> 16) & 0xFF) == fpu_idle_frame_size())
							perform_frestore_idle(addr);
						m68ki_cpu.fpu_just_reset = 0;
					}
					else
					{
						do_frestore_null();
					}
					break;

				case 6: // (d8, An, Xn)
					addr = EA_AY_IX_32();
					temp = m68ki_read_32(addr);

					if (temp & 0xff000000)
					{
						if (((temp >> 16) & 0xFF) == fpu_idle_frame_size())
							perform_frestore_idle(addr);
						m68ki_cpu.fpu_just_reset = 0;
					}
					else
					{
						do_frestore_null();
					}
					break;

				case 7: // Absolute and PC-relative
					switch (reg)
					{
						case 0: addr = EA_AW_32(); break;
						case 1: addr = EA_AL_32(); break;
						case 2: addr = EA_PCDI_32(); break;
						case 3: addr = EA_PCIX_32(); break;
						default:
							fatalerror("M68kFPU: FRESTORE unhandled mode 7 reg %d at %x\n", reg, REG_PC);
					}
					temp = m68ki_read_32(addr);

					if (cpu_log_enabled) fprintf(stderr, "[FRESTORE-M7] PC=%08x addr=%08x reg=%d data=%08x just_reset->%d\n",
						REG_PC, addr, reg, temp, (temp & 0xff000000) ? 0 : 1);

					if (temp & 0xff000000)
					{
						if (((temp >> 16) & 0xFF) == fpu_idle_frame_size())
							perform_frestore_idle(addr);
						m68ki_cpu.fpu_just_reset = 0;
					}
					else
					{
						do_frestore_null();
					}
					break;

				default:
					fatalerror("M68kFPU: FRESTORE unhandled mode %d reg %d at %x\n", mode, reg, REG_PC);
			}
			break;
		}
		break;

		default:	fatalerror("m68040_fpu_op1: unimplemented op %d at %08X\n", (REG_IR >> 6) & 0x3, REG_PC-2);
	}
}



