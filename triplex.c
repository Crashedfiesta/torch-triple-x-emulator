/* triplex.c -- Torch Triple X "Stickleback" service processor emulator.
 * 
 * Original code by Kokoboi (VCF Forums)
 *
 * Updated and generally interfered with by Crashedfiesta
 *
 *
 * Board (per schematic page 1):
 *   HD6303R service CPU (8-bit, 6800-family extended)
 *   27128 16KB EPROM = triplex.rom  (mapped at $C000-$FFFF; reset @ $C2DC)
 *   6845E CRTC + 64KB video memory + color palette + RGB out
 *   HD146818 battery-backed RTC
 *   6840 PTM (programmable timer module, 3 counters)
 *   6850 ACIA (slow serial / modem)
 *   Audio out (PWM via op-amp)
 *   Keyboard (external 8749 microcontroller via UART link)
 *   1MHz QBUS to host 68010 side
 *
 * Memory map (working hypothesis, refined as the firmware reveals itself):
 *   $0000-$001F  HD6303R internal I/O registers
 *   $0040-$00FF  HD6303R internal RAM (192 bytes)
 *   $0100-$BFFF  external area (I/O peripherals, VRAM window etc.)
 *   $C000-$FFFF  16KB EPROM (triplex.rom)
 *
 * Catch-all I/O logger reveals what addresses the firmware actually touches.  */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

#ifndef _WIN32
#include <termios.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <sys/stat.h>
#endif

#include <fcntl.h>
#include <time.h>
#include <errno.h>

#ifdef USE_SDL
#include <SDL2/SDL.h>
#endif
#ifdef USE_M68K
#include "m68k.h"
#endif

#define TRACE_SCSI 0

/* Line added to start supporting ancient 32bit systems */
#define _FILE_OFFSET_BITS 64

/* --- Memory --- */
#define ROM_BASE  0xC000u
#define ROM_SIZE  0x4000u
static uint8_t g_sp_rom[ROM_SIZE];
static uint8_t g_ram[0x10000];  /* full 64KB shadow; ROM area is read from g_sp_rom */

/* HD6303R internal RAM area (when enabled via RAMC register).  Sits at
 * $0040-$00FF in mode 2/expanded modes. */
static uint8_t g_intram[0xC0];

/* HD6303R internal I/O registers ($0000-$001F).
 *   $00 P1DDR    Port 1 Data Direction
 *   $01 P2DDR    Port 2 Data Direction
 *   $02 P1       Port 1 data
 *   $03 P2       Port 2 data
 *   $04 P3DDR    Port 3 DDR
 *   $05 P4DDR    Port 4 DDR
 *   $06 P3       Port 3 data
 *   $07 P4       Port 4 data
 *   $08 TCSR     Timer Control/Status
 *   $09 FRC-H    Free-running counter high
 *   $0A FRC-L    Free-running counter low
 *   $0B OCR-H    Output Compare Register high
 *   $0C OCR-L    Output Compare Register low
 *   $0D ICR-H    Input Capture Register high
 *   $0E ICR-L    Input Capture Register low
 *   $0F P3CSR    Port 3 Control/Status
 *   $10 RMCR     Rate / Mode Control (SCI)
 *   $11 TRCSR    Transmit/Receive Control & Status (SCI)
 *   $12 RDR      Receive Data Register (read-only)
 *   $13 TDR      Transmit Data Register (write-only)
 *   $14 RAMCR    RAM Control Register
 *   $15-$1F      reserved
 */
static uint8_t g_io[0x20];

/* --- HD6303 SCI (serial keyboard) receive state ---
 * The Torch keyboard is an external micro on a 4800-baud serial link into
 * the HD6303 on-chip SCI.  We model the receive side only: bytes injected
 * from the SDL key handler queue here, get clocked into RDR ($12) one at a
 * time, set RDRF, and raise the SCI interrupt ($FFF0).  The CARETAKER ISR
 * at $CA82 reads RDR and enqueues into the shared-VRAM keyboard mailbox. */
static uint8_t g_sci_rdr;             /* Receive Data Register ($12) */
static int     g_sci_rdrf;            /* Receive Data Register Full */
static uint8_t g_kbd_q[256];          /* injected scancodes from SDL */
static int     g_kbd_q_head, g_kbd_q_tail;
static int     g_sci_feed_delay;      /* paces bytes at ~serial rate */

/* TRCSR ($11) bit assignments (HD6303): TE=$02 TIE=$04 RE=$08 RIE=$10
 * TDRE=$20 ORFE=$40 RDRF=$80. */

/* --- 6850 ACIA (serial mouse) receive state ---
 * The Torch mouse is a serial mouse on an external 6850 ACIA mapped at SP
 * $0200 (status/control) and $0201 (data).  Its /IRQ drives HD6303 /IRQ1
 * ($FFF8 -> ISR $CC4A).  We model the receive side: 3-byte mouse packets
 * injected from SDL get clocked into the data register, set RDRF, and raise
 * IRQ1; the CARETAKER ISR enqueues them into the shared-VRAM mouse mailbox. */
static uint8_t g_acia_ctrl = 0x55;    /* last value written to control reg */
static uint8_t g_acia_rdr;            /* received data byte ($0201) */
static int     g_acia_rdrf;           /* Receive Data Register Full */
static uint8_t g_mouse_q[256];        /* injected mouse bytes from SDL */
static int     g_mouse_q_head, g_mouse_q_tail;
static int     g_acia_feed_delay;     /* paces bytes at ~serial rate */

/* Headless input-injection test (--type STR / --mouse): once boot has
 * reached the login prompt, pace synthetic keystrokes and mouse motion
 * through the real SP input path (SCI/ACIA -> CARETAKER ISR -> mailbox)
 * so the keyboard/mouse pipeline can be verified end-to-end without
 * needing an interactive SDL window. */
static const char *g_type_str = NULL;
static int         g_type_pos = 0;
static int         g_mouse_test = 0;
static int         g_mouse_step = 0;

/* --- CPU state --- */
typedef struct {
    uint16_t pc;
    uint16_t sp;
    uint16_t ix;        /* X register */
    uint8_t  a, b;      /* accumulators (together = D) */
    uint8_t  cc;        /* condition code: 11HINZVC */
} cpu_t;

static cpu_t cpu;

#define CC_H 0x20
#define CC_I 0x10
#define CC_N 0x08
#define CC_Z 0x04
#define CC_V 0x02
#define CC_C 0x01

static inline uint16_t D_get(void) { return (cpu.a << 8) | cpu.b; }
static inline void D_set(uint16_t v){ cpu.a = v >> 8; cpu.b = v & 0xFF; }

/* Variable to store serial number */

static uint8_t g_saved_serial[4];  /*Store for serial number */

/* --- Logging --- */
static int g_log_io = 0;
static int g_log_unmapped = 0;
static long g_log_budget = 5000;
static unsigned g_vram_dump_number = 1;

static void io_log(const char *op, uint16_t addr, uint8_t val) {
    if (!g_log_io) return;
    if (--g_log_budget < 0) return;
    (void)0;
}

/* --- Memory access --- */
/* --- Motorola 6845E CRTC at $0400 (address port) / $0401 (data port) ---
 * 18 registers (0-17).  Writing $0400 selects register; writing/reading
 * $0401 accesses the selected register.  Reading $0400 returns a status:
 *   bit 7 = vertical sync       (we toggle this every ~25K cycles)
 *   bit 6 = light pen latched   (we leave at 0)
 *   bit 5 = update strobe
 *   bits 0-4 = reserved
 */
static uint8_t g_crtc_regs[18];
static uint8_t g_crtc_idx = 0;
static int     g_crtc_cycles = 0;
static int     g_crtc_in_vsync = 0;

static int g_crtc_update_strobe = 0;
static int g_crtc_strobe_reads = 0;
/* On the real 6845E, bit 5 of the status register is the UPDATE STROBE -- it
 * goes low during the active scan period and high during retrace, with a
 * period of one frame.  The firmware uses this for two things:
 *   1. A "CRTC alive" liveness test that just expects bit 5 to ever toggle.
 *   2. A timing test ($C046) that counts how many reads of $0400 happen
 *      before bit 5 changes value, and checks SAMPLE1+SAMPLE3 is in
 *      [$03E8 ... $0515] (decimal 1000-1301).  Each sample ~500-650 reads.
 * We toggle the strobe every 600 reads to keep both tests happy. */
static uint8_t crtc_status(void) {
    if (++g_crtc_strobe_reads >= 600) {
        g_crtc_strobe_reads = 0;
        g_crtc_update_strobe ^= 1;
    }
    return (g_crtc_in_vsync ? 0x80 : 0x00)
         | (g_crtc_update_strobe ? 0x20 : 0x00);
}

/* Palette: 16 entries, each one byte.  The physical Triple X palette is
 * formed by the two 74S189 16x4 RAMs shown on schematic page 13. */
static uint8_t g_palette[16];

/*
 * The real machine has three clearly visible startup phases:
 *
 *   1. Pale blue while the service processor performs its initial tests.
 *   2. Dark blue once the RAM test has completed and the Caretaker version
 *      is displayed.
 *   3. The normal grey/black/red/green display once the host starts using
 *      Caretaker and, later, OpenTop.
 *
 * Caretaker's first palette operation is sixteen writes of E0.  Rendering
 * those bytes literally would make every logical colour identical, so the
 * startup phases are modelled explicitly.  After the host/SP mailbox becomes
 * active, normal colours are used.  Any later palette write is treated as a
 * genuine software palette change and is rendered literally thereafter.
 */
typedef enum {
    VIDEO_DISPLAY_PALE_BLUE = 0,
    VIDEO_DISPLAY_DARK_BLUE,
    VIDEO_DISPLAY_NORMAL
} video_display_state_t;

static video_display_state_t g_video_display_state = VIDEO_DISPLAY_PALE_BLUE;
static uint16_t g_boot_palette_e0_mask = 0;
static int g_runtime_palette_enabled = 0;
static int g_runtime_palette_programmed = 0;

static int g_boot_slow_mode = 0;
#ifdef USE_SDL
static Uint32 g_boot_slow_start_ms = 0;
#endif

#define CARETAKER_SLOW_WINDOW_MS 12000
#define CARETAKER_HOST_DIVISOR 10

static int g_caretaker_host_counter = 0;
static void video_display_reset(void)
{
    g_video_display_state = VIDEO_DISPLAY_PALE_BLUE;
    g_runtime_palette_enabled = 0;
    g_runtime_palette_programmed = 0;
    memset(g_palette, 0, sizeof(g_palette));
}

static void video_display_host_handover(void)
{
    if (g_runtime_palette_enabled)
        return;

    g_runtime_palette_enabled = 1;
    g_runtime_palette_programmed = 0;
    g_video_display_state = VIDEO_DISPLAY_NORMAL;
	fprintf(stderr,"[DISPLAY] dark blue (Caretaker startup)-> boot (Regular boot)\n");

#ifdef USE_SDL
    g_boot_slow_mode = 1;
    g_boot_slow_start_ms = SDL_GetTicks();
    fprintf(stderr, "[BOOT] Caretaker interaction slowdown started\n");
#endif

}

/* 6840 PTM (Programmable Timer Module) at $0100-$0107.  Three 16-bit
 * down-counters with shared control registers.  Per the MAME triplex
 * driver, the PTM's IRQ output is merged with the ACIA and RTC IRQs to
 * drive the SP's /IRQ1 line, so the SP's IRQ1 ISR ($CC4A) runs every PTM
 * timer underflow -- and inside that ISR the SP polls the ACIA status,
 * which is how mouse bytes get consumed even when ACIA Rx IRQ is off.
 *
 * Register map: 0 = CR1 or CR3 (selected by CR2 bit 0), 1 = CR2,
 * 2-3 = T1 MSB/LSB, 4-5 = T2 MSB/LSB, 6-7 = T3 MSB/LSB.
 *
 * Per-timer control register bits:
 *   bit 0: clock source (0 = external clock, 1 = E)
 *   bit 1: prescaler enable (T2/T3 only)
 *   bit 2-4: timer operating mode
 *   bit 5: counter output enable
 *   bit 6: interrupt enable
 *   bit 7: timer reset (1 = stopped/reset, 0 = run)
 *
 * Status register: bit 0/1/2 = per-timer IRQ flags, bit 7 = composite. */
static uint16_t g_ptm_counter[3] = {0xFFFF, 0xFFFF, 0xFFFF};
static uint16_t g_ptm_latch[3]   = {0xFFFF, 0xFFFF, 0xFFFF};
static uint8_t  g_ptm_status     = 0;
static uint8_t  g_ptm_cr[3]      = {0, 0, 0};   /* CR1, CR2, CR3 */

#define g_ptm_cr2 g_ptm_cr[1]

static void ptm_tick(int cycles)
{
    if (g_ptm_cr[0] & 0x01)
        return;

    for (int i = 0; i < 3; i++) {
        uint32_t remaining = (uint32_t)cycles;
        uint32_t period = g_ptm_latch[i]
                        ? (uint32_t)g_ptm_latch[i] + 1u
                        : 65536u;

        while (remaining > g_ptm_counter[i]) {
            remaining -= (uint32_t)g_ptm_counter[i] + 1u;

            g_ptm_status |= (uint8_t)(1u << i);

            if (g_ptm_cr[i] & 0x40)
                g_ptm_status |= 0x80;

            g_ptm_counter[i] = (uint16_t)(period - 1u);
        }

        g_ptm_counter[i] -= (uint16_t)remaining;
    }
}

/* /IRQ1 contribution from the PTM: any per-timer IRQ flag latched AND that
 * timer has interrupt enable set in its control register. */
static int ptm_irq1_pending(void) {
    if ((g_ptm_status & 0x01) && (g_ptm_cr[0] & 0x40)) return 1;
    if ((g_ptm_status & 0x02) && (g_ptm_cr[1] & 0x40)) return 1;
    if ((g_ptm_status & 0x04) && (g_ptm_cr[2] & 0x40)) return 1;
    return 0;
}
static uint8_t ptm_read(uint16_t off) {
    /* 6840 register map: 0=status (or CR3/CR1 depending), 1=CR2,
     * 2/3=T1 MSB/LSB, 4/5=T2 MSB/LSB, 6/7=T3 MSB/LSB.            */
    uint8_t result;

    switch (off) {
    case 0:
        result = g_ptm_status;
        g_ptm_status = 0;
        break;

    case 1:
        result = g_ptm_cr[1];
        break;

    case 2:
        result = g_ptm_counter[0] >> 8;
        break;

    case 3:
        result = g_ptm_counter[0] & 0xff;
        break;

    case 4:
        result = g_ptm_counter[1] >> 8;
        break;

    case 5:
        result = g_ptm_counter[1] & 0xff;
        break;

    case 6:
        result = g_ptm_counter[2] >> 8;
        break;

    case 7:
        result = g_ptm_counter[2] & 0xff;
        break;

    default:
        result = 0;
        break;
    }

    return result;
}
	

/* HD146818 RTC at $0300-$033F (64 bytes).  Offsets 0-9 are date/time,
 * $0A-$0D control/status, $0E-$3F user CMOS RAM.  We track seconds in
 * a counter that ticks on a cycle budget and use the host wall clock
 * for display sanity. */
/*static int g_rtc_clock_div = 0;
static int g_rtc_sec_in_emu = 0;  /* increments every "1 sec" of cycles */
static uint8_t g_rtc_user_ram[0x40];
static int g_rtc_clock_div = 0;
static int g_rtc_sec_in_emu = 0;
static uint8_t g_rtc_user_ram[0x40];

static void network_cmos_init(void)
{
    /*
     * RTC CMOS "system options" word read by the Torch kernel.
     *
     * Bit 4 of CMOS $0313 enables B-NET.
     * Bit 5 enables NFS.
     */
    g_rtc_user_ram[0x13] |= 0x30;

    fprintf(stderr,
            "[CMOS] driver-enable bits set: "
            "B-NET (bit 4) and NFS (bit 5) of cmos[$0313]\n");

    /*
     * Torch gethernum() reads six bytes from CMOS $0314-$0319
     * and uses them as the LANCE Ethernet MAC address.
     *
     * Construct a locally-administered unicast MAC using the
     * saved Torch machine serial:
     *
     *     02:80:E1:S1:S2:S3
     */
    g_rtc_user_ram[0x14] = 0x02;
    g_rtc_user_ram[0x15] = 0x80;
    g_rtc_user_ram[0x16] = 0xE1;
    g_rtc_user_ram[0x17] = g_saved_serial[1];
    g_rtc_user_ram[0x18] = g_saved_serial[2];
    g_rtc_user_ram[0x19] = g_saved_serial[3];

    fprintf(stderr,
            "[CMOS] Ethernet MAC = "
            "%02X:%02X:%02X:%02X:%02X:%02X\n",
            g_rtc_user_ram[0x14],
            g_rtc_user_ram[0x15],
            g_rtc_user_ram[0x16],
            g_rtc_user_ram[0x17],
            g_rtc_user_ram[0x18],
            g_rtc_user_ram[0x19]);
}

/*
 * Return non-zero if the supplied year is a leap year.
 */
static int rtc_is_leap_year(int year)
{
    return ((year % 4) == 0 &&
           (((year % 100) != 0) || ((year % 400) == 0)));
}

/*
 * Return the day of the week for 1 January of a given year.
 *
 * Result:
 *   0 = Sunday
 *   1 = Monday
 *   ...
 *   6 = Saturday
 *
 * This uses a standard Gregorian calendar calculation.
 */
static int rtc_jan1_weekday(int year)
{
    int y = year - 1;

    return (1 +
            y +
            y / 4 -
            y / 100 +
            y / 400) % 7;
}

/*
 * Find a pre-Y2K year with exactly the same calendar layout as the
 * current real year.
 *
 * A matching calendar year must:
 *
 *   1. Have the same leap/non-leap status.
 *   2. Have the same weekday on 1 January.
 *
 * We search backwards from 1999.  This keeps the date safely within
 * the range expected by old Torch software while preserving weekdays,
 * month lengths and leap days.
 */
static int rtc_find_compatible_year(int real_year)
{
    int real_leap = rtc_is_leap_year(real_year);
    int real_jan1 = rtc_jan1_weekday(real_year);

    for (int year = 1999; year >= 1972; year--) {
        if (rtc_is_leap_year(year) == real_leap &&
            rtc_jan1_weekday(year) == real_jan1) {
            return year;
        }
    }

    /*
     * Should never happen -- the 28-year search range contains every
     * possible Gregorian calendar layout.
     */
    return 1998;
}

/*
 * Convert an ordinary binary value to packed BCD.
 */
static uint8_t rtc_to_bcd(int value)
{
    return (uint8_t)(((value / 10) << 4) | (value % 10));
}

/*
 * Store a normal integer into an RTC register using the data format
 * selected by Register B.
 *
 * Register B bit 2:
 *
 *   1 = binary
 *   0 = BCD
 */
static uint8_t rtc_encode_value(int value)
{
    if (g_rtc_user_ram[0x0B] & 0x04)
        return (uint8_t)value;

    return rtc_to_bcd(value);
}

/*
 * Synchronise the emulated HD146818 clock with the host computer.
 *
 * The real date and time are obtained from Windows/Linux, but the year
 * is mapped onto a calendar-compatible pre-2000 year so that old Torch
 * software never has to deal with a post-Y2K date.
 *
 * Example:
 *
 *     host:   Monday 20 July 2026
 *     Torch:  Monday 20 July 1998
 *
 * Registers $0E-$3F are deliberately untouched because they contain
 * battery-backed machine configuration, including the Torch serial.
 */
static void rtc_sync_from_host(void)
{
    time_t now;
    struct tm local_tm;
    struct tm *tm_ptr;
    int real_year;
    int torch_year;
    int weekday;

    now = time(NULL);

#ifdef _WIN32

    /*
     * localtime_s() on Windows uses:
     *
     *     localtime_s(struct tm *, const time_t *)
     */
    if (gmtime_s(&local_tm, &now) != 0){
		return;
	}
	fprintf(stderr,
			"[RTC HOST] localtime=%04d-%02d-%02d %02d:%02d:%02d "
			"isdst=%d\n",
			local_tm.tm_year + 1900,
			local_tm.tm_mon + 1,
			local_tm.tm_mday,
			local_tm.tm_hour,
			local_tm.tm_min,
			local_tm.tm_sec,
			local_tm.tm_isdst);
#else

    /*
     * Use ordinary localtime() here for broad compatibility with older
     * Unix/Linux C libraries.  Copy the result immediately.
     */
    tm_ptr = localtime(&now);

    if (tm_ptr == NULL)
        return;

    local_tm = *tm_ptr;

#endif

    real_year = local_tm.tm_year + 1900;
    torch_year = rtc_find_compatible_year(real_year);

    /*
     * struct tm:
     *
     *   tm_wday = 0 Sunday ... 6 Saturday
     *
     * MC146818 convention:
     *
     *   1 Sunday ... 7 Saturday
     */
    weekday = local_tm.tm_wday + 1;

    /*
     * Do not update the clock while software has SET (Register B bit 7)
     * asserted.  On the real RTC this freezes calendar updates while the
     * operating system programs the clock.
     */
    if (g_rtc_user_ram[0x0B] & 0x80)
        return;

    g_rtc_user_ram[0x00] = rtc_encode_value(local_tm.tm_sec);
    g_rtc_user_ram[0x02] = rtc_encode_value(local_tm.tm_min);
    g_rtc_user_ram[0x04] = rtc_encode_value(local_tm.tm_hour);
    g_rtc_user_ram[0x06] = rtc_encode_value(weekday);
    g_rtc_user_ram[0x07] = rtc_encode_value(local_tm.tm_mday);
    g_rtc_user_ram[0x08] = rtc_encode_value(local_tm.tm_mon + 1);
    g_rtc_user_ram[0x09] = rtc_encode_value(torch_year % 100);

    /*
     * Keep this variable consistent with the seconds register because
     * existing emulator code refers to it.
     */
    g_rtc_sec_in_emu = local_tm.tm_sec;
	
	static int first_sync = 1;

	if (first_sync) {
		fprintf(stderr,
            "[RTC] Host %04d-%02d-%02d %02d:%02d:%02d "
            "-> Torch %04d-%02d-%02d "
            "(%s mode)\n",
            real_year,
            local_tm.tm_mon + 1,
            local_tm.tm_mday,
            local_tm.tm_hour,
            local_tm.tm_min,
            local_tm.tm_sec,
            torch_year,
            local_tm.tm_mon + 1,
            local_tm.tm_mday,
            (g_rtc_user_ram[0x0B] & 0x04) ? "binary" : "BCD");

		first_sync = 0;
	}
		
}


/*
 * The RTC is synchronised periodically rather than trying to implement
 * calendar arithmetic inside the emulator.
 *
 * This means the host operating system automatically handles:
 *
 *   - seconds/minutes/hours
 *   - midnight
 *   - month boundaries
 *   - leap years
 *   - daylight-saving changes
 *
 * The deliberately short interval also satisfies Caretaker's RTC
 * liveness tests.
 */
static void rtc_tick(int cycles)
{
    g_rtc_clock_div += cycles;

    if (g_rtc_clock_div >= 200000) {
        g_rtc_clock_div = 0;
		
		g_rtc_sec_in_emu = 
			(g_rtc_sec_in_emu + 1) % 60;
		
		g_rtc_user_ram[0x00] =
			rtc_encode_value(g_rtc_sec_in_emu);
    }
}

static uint8_t rtc_read(uint16_t off) {
    off &= 0x3F;
    /*if (off == 0) return g_rtc_sec_in_emu;*/
    /* Register D bit 7 = VRT (valid RAM and time).  Firmware reads it. */
    if (off == 0x0D) return 0x80;
    return g_rtc_user_ram[off];
}

static void rtc_write(uint16_t off, uint8_t val)
{
    off &= 0x3F;

    /*
     * Register D is effectively read-only.
     */
    if (off == 0x0D)
        return;

    /*
     * Store the value in the actual emulated RTC register.
     */
    g_rtc_user_ram[off] = val;

    /*
     * Keep our accelerated POST seconds counter in step if software
     * explicitly writes the seconds register.
     *
     * Decode BCD if necessary because g_rtc_sec_in_emu is always held
     * internally as an ordinary binary integer.
     */
    if (off == 0x00) {
        if (g_rtc_user_ram[0x0B] & 0x04) {
            /* Binary mode */
            g_rtc_sec_in_emu = val;
        } else {
            /* Packed BCD mode */
            g_rtc_sec_in_emu =
                ((val >> 4) & 0x0F) * 10 +
                (val & 0x0F);
        }
    }
}

static void ptm_write(uint16_t off, uint8_t val) {
    /* MSB write loads the high byte of a latch, LSB write loads low byte
     * AND transfers the latch into the counter.  Simplified: each pair of
     * writes (MSB then LSB) sets both latch and counter.  CR writes go to
     * shared CR1/CR2/CR3 registers. */

    switch (off) {
    case 0: /* CR2 bit 0 selects whether writes to offset 0 go to CR1 or CR3.
             * Per 6840: CR2[0]=0 -> writes to offset 0 are CR3, =1 -> CR1. */
            if (g_ptm_cr[1] & 0x01) g_ptm_cr[0] = val;
            else                    g_ptm_cr[2] = val;
            break;
    case 1: g_ptm_cr[1] = val; break;
    case 2: g_ptm_latch[0] = (g_ptm_latch[0] & 0x00FF) | (val << 8); break;
    case 3: g_ptm_latch[0] = (g_ptm_latch[0] & 0xFF00) | val;
            g_ptm_counter[0] = g_ptm_latch[0]; break;
    case 4: g_ptm_latch[1] = (g_ptm_latch[1] & 0x00FF) | (val << 8); break;
    case 5: g_ptm_latch[1] = (g_ptm_latch[1] & 0xFF00) | val;
            g_ptm_counter[1] = g_ptm_latch[1]; break;
    case 6: g_ptm_latch[2] = (g_ptm_latch[2] & 0x00FF) | (val << 8); break;
    case 7: g_ptm_latch[2] = (g_ptm_latch[2] & 0xFF00) | val;
            g_ptm_counter[2] = g_ptm_latch[2]; break;
    }
}

/* HD6303R internal Free-Running Counter (FRC) at $09-$0A.
 * Output Compare Register (OCR) at $0B-$0C.
 * TCSR ($08) bit 6 = OCF set when FRC == OCR.
 * TCSR ($08) bit 5 = TOF set when FRC wraps from $FFFF to $0000.   */
static uint16_t g_frc = 0;
static void tick_timer(int cycles) {
    uint16_t ocr = (g_io[0x0B] << 8) | g_io[0x0C];
    uint16_t new_frc = g_frc + cycles;
    /* Detect OCR match crossing within this tick */
    if ((g_frc <= ocr && new_frc >= ocr) ||
        (cycles > 0 && new_frc < g_frc && (g_frc <= ocr || new_frc >= ocr))) {
        g_io[0x08] |= 0x40;   /* set OCF */
    }
    if (new_frc < g_frc) g_io[0x08] |= 0x20;  /* set TOF on overflow */
    g_frc = new_frc;
    g_io[0x09] = g_frc >> 8;
    g_io[0x0A] = g_frc & 0xFF;
}

/* Should we fire an OCF interrupt right now?
 * Conditions:  TCSR bit 6 (OCF) set AND TCSR bit 3 (EOCI) set AND I flag clear */
static int ocf_pending(void) {
    return (g_io[0x08] & 0x48) == 0x48 && !(cpu.cc & CC_I);
}
static int tof_pending(void) {
    /* TCSR bit 5 (TOF) AND TCSR bit 2 (ETOI) AND I flag clear */
    return (g_io[0x08] & 0x24) == 0x24 && !(cpu.cc & CC_I);
}

/* Clock the next queued keyboard byte into the SCI receive register.
 * Paced by g_sci_feed_delay so bytes arrive no faster than the SP's ISR
 * can drain them -- a real 4800-baud byte is ~2ms (~2500 SP cycles). */
static void sci_rx_feed(void) {
    if (g_sci_feed_delay > 0) { g_sci_feed_delay--; return; }
    if (g_sci_rdrf) return;                       /* RDR not yet read */
    if (g_kbd_q_head == g_kbd_q_tail) return;     /* queue empty */
    g_sci_rdr = g_kbd_q[g_kbd_q_tail];
    g_kbd_q_tail = (g_kbd_q_tail + 1) & 0xFF;
    g_sci_rdrf = 1;
    g_sci_feed_delay = 2500;
    {
    }
}

/* SCI receive interrupt: RDRF set AND RIE (TRCSR bit 4) set AND I clear. */
static int sci_pending(void) {
    return g_sci_rdrf && (g_io[0x11] & 0x10) && !(cpu.cc & CC_I);
}

/* Push a scancode into the keyboard injection queue (called from SDL). */
static void kbd_enqueue(uint8_t code) {
    int nh = (g_kbd_q_head + 1) & 0xFF;
    if (nh == g_kbd_q_tail) return;               /* queue full, drop */
    g_kbd_q[g_kbd_q_head] = code;
    g_kbd_q_head = nh;
}

/* 6850 status register ($0200 read): bit0 RDRF, bit1 TDRE (always set --
 * transmit never blocks), bit2 DCD=0, bit7 IRQ when RDRF and Rx-IRQ on. */
static uint8_t acia_status(void) {
    uint8_t s = 0x02;
    if (g_acia_rdrf) s |= 0x01;
    if (g_acia_rdrf && (g_acia_ctrl & 0x80)) s |= 0x80;
    return s;
}

/* Clock the next queued mouse byte into the ACIA receive register. */
static void acia_rx_feed(void) {
    if (g_acia_feed_delay > 0) { g_acia_feed_delay--; return; }
    if (g_acia_rdrf) return;                      /* RDR not yet read */
    if (g_mouse_q_head == g_mouse_q_tail) return; /* queue empty */
    g_acia_rdr = g_mouse_q[g_mouse_q_tail];
    g_mouse_q_tail = (g_mouse_q_tail + 1) & 0xFF;
    g_acia_rdrf = 1;
    g_acia_feed_delay = 2500;
    {
    }
}

/* IRQ1 pending: RDRF set AND Rx interrupt enabled (control bit 7) AND I clear. */
static int acia_irq1_pending(void) {
    return g_acia_rdrf && (g_acia_ctrl & 0x80) && !(cpu.cc & CC_I);
}


/* Queue one Torch-mouse packet.  The kernel MoveMouse function at
 * $12A472 (torch2) is a 3-byte stateful collector triggered for each
 * mouse byte:
 *   byte 0: bit 7 set (0x80) = sync (resets counter); bit 3 = dx high
 *           bit; bit 4 = dy high bit; bits 0-2 = button state.
 *   byte 1: low 7 bits of dx ((byte & 0x7F) | (b0.bit3 << 7)).
 *   byte 2: low 7 bits of dy ((byte & 0x7F) | (b0.bit4 << 7)).
 * Once the 3rd byte arrives, MoveMouse processes:
 *   x' = clamp(x + dx, 0, screen_w-1)
 *   y' = clamp(y + dy, 0, screen_h-1)
 * and dispatches a motion/button-change event via $124FC0 to the
 * window-system event queue.  The cursor sprite redraw is then handled
 * by the wb driver against the new (x, y). */
static void mouse_packet(int dx, int dy, int left, int right) {
    /* Per MoveMouse @ $12A472 disassembly, byte 1 = signed_dx & 0x7F and
     * byte 0 bit 3 = high bit of signed_dx (likewise byte 2/byte 0 bit 4
     * for dy).  The kernel reads:
     *   d0 = (byte1 & 0x7F) | ((byte0 & 0x08) << 4);  // pack
     *   ext.w d0;                                     // SIGN-extend
     *   d5 += d0.w;                                   // X += signed dx
     * Buttons go in byte 0 bits 0 (left) and 1 (right); MoveMouse extracts
     * them at $12A55E and ORs them into $156130 bits 12-13. */
    if (dx > 127) dx = 127; if (dx < -128) dx = -128;
    if (dy > 127) dy = 127; if (dy < -128) dy = -128;
    uint8_t sdx = (uint8_t)(int8_t)dx;
    uint8_t sdy = (uint8_t)(int8_t)dy;
    uint8_t b0 = 0x80;
    if (sdx & 0x80) b0 |= 0x08;
    if (sdy & 0x80) b0 |= 0x10;
    if (left)  b0 |= 0x01;
    if (right) b0 |= 0x02;
    /* 6-byte SCI sequence wrapped with mouse-mode 0x7A entry and 0xFF exit,
     * so real keystrokes between mouse packets are still dispatched as
     * keys, not as more mouse data.  See mouse.md for full explanation of
     * the $156542 override and why b0 is sent twice.
     *
     * CRITICAL: enqueue atomically (all 6 or none).  If the kbd queue
     * fills up (256-byte ring) and we partially enqueue, the kernel
     * sees an unmatched 0xFF and treats it as a printable keystroke
     * (table[0x7F] = 0x1000 falls through to the ASCII path when not
     * already in mouse mode).  That manifests as random characters
     * appearing all over the screen. */
    int free = (g_kbd_q_tail - g_kbd_q_head - 1) & 0xFF;
    if (free < 6) return;
    kbd_enqueue(0x7A);
    kbd_enqueue(b0);
    kbd_enqueue(b0);
    kbd_enqueue(sdx & 0x7F);
    kbd_enqueue(sdy & 0x7F);
    kbd_enqueue(0xFF);
}

/* HD6303R timer register read/write semantics. */
static uint8_t io_reg_read(uint16_t addr) {
    if (addr == 0x08) {
        /* Reading TCSR latches the flags state for next OCR-write clear */
        return g_io[0x08];
    }
    if (addr == 0x09) return g_frc >> 8;
    if (addr == 0x0A) return g_frc & 0xFF;
    if (addr == 0x11) {
        /* TRCSR: control bits as written, plus live status -- RDRF from
         * our receive state, TDRE always set (transmit never blocks). */
        return (g_io[0x11] & 0x1F) | (g_sci_rdrf ? 0x80 : 0x00) | 0x20;
    }
    if (addr == 0x12) {
        /* Reading RDR returns the byte and clears RDRF. */
        {
        }
        g_sci_rdrf = 0;
        return g_sci_rdr;
    }
    return g_io[addr];
}

/* Special handling for timer-related writes. */
extern int g_host_p1_released;
void dmac_pcl0_transition(int level);   /* HD63450 PCL0 input (defined later) */
static void io_reg_write(uint16_t addr, uint8_t val) {
    /* P1 bit 3 holds the host 68010 in RESET/HALT when 1.  Track the
     * 1→0 transition so we can release the host CPU exactly when the
     * firmware does. */
    if (addr == 0x02) {
        int old_bit3 = (g_io[0x02] >> 3) & 1;
        int new_bit3 = (val >> 3) & 1;
        if (old_bit3 && !new_bit3) {
			extern int g_sp_reboot_pending;

			if (!g_host_p1_released) {
				g_host_p1_released = 1;

				/*
				* Caretaker POST is complete. Correct any deliberately
				* accelerated RTC seconds by synchronising with the host.
				*/
				rtc_sync_from_host();
			}
		}
        /* P1 bit 4 = PROCINT: per MAME's triplex.cpp it is wired to the
         * HD63450 DMAC's PCL0 input.  This is the service processor's
         * doorbell -- it pulses PROCINT after handling a $3F0 mailbox
         * command, and the DMAC turns the edge into the host's IRQ3. */
        int old_bit4 = (g_io[0x02] >> 4) & 1;
        int new_bit4 = (val >> 4) & 1;
        if (old_bit4 != new_bit4) dmac_pcl0_transition(new_bit4);
    }
	
	if (addr == 0x02) {
		uint8_t old = g_io[0x02];
		if (((old ^ val) & 0x04) != 0) {		
			if (g_video_display_state == VIDEO_DISPLAY_PALE_BLUE &&
				old == 0x16 &&
				val == 0x12) {
				fprintf(stderr,
                    "[DISPLAY] pale blue (Initial startup) -> dark blue (Caretaker)\n");

				g_video_display_state = VIDEO_DISPLAY_DARK_BLUE;
			}	
			
		}
	}
	
	g_io[addr] = val;
    if (addr == 0x09 || addr == 0x0A) {
        /* Any write to FRC presets it to $FFF8 on the real chip; we just
         * use the written value verbatim.  The firmware uses STD $09 to
         * reset both bytes at once. */
        g_frc = (g_io[0x09] << 8) | g_io[0x0A];
    }
    if (addr == 0x0B || addr == 0x0C) {
        /* Writing OCR clears OCF if TCSR was read first.  Simplified: just
         * clear OCF on OCR write. */
        g_io[0x08] &= ~0x40;
    }
}

/* VRAM-decay sensor: the POST has a refresh-disable test at $C557 that
 * writes $FF to $BFFF then reads it many times in a tight loop expecting
 * to see decay (real DRAM loses data without refresh).  We track
 * consecutive reads of $BFFF; after a couple without an intervening
 * write, we return 0 to model the decay. */
static int g_bfff_reads = 0;

/* Map SP address $4000-$BFFF into the 64KB VRAM bank selected by P1.2.
 * g_io[$02] is the latched P1 data; bit 2 = VIDSEL. */
extern uint8_t g_vram[0x10000];
static inline uint32_t sp_vram_offset(uint16_t addr) {
    int bank = (g_io[0x02] >> 2) & 1;
    return (bank * 0x8000) | (addr - 0x4000);
}

static uint8_t mem_read(uint16_t addr) {
    if (addr < 0x20)              return io_reg_read(addr);
    if (addr >= 0x40 && addr < 0x100) return g_intram[addr - 0x40];
    if (addr >= ROM_BASE)         return g_sp_rom[addr - ROM_BASE];
    if (addr >= 0x4000 && addr < 0xC000) {
    if (addr == 0xBFFF && ++g_bfff_reads >= 3)
        return 0;

    uint32_t vo = sp_vram_offset(addr);
    uint8_t value = g_vram[vo];

    /*
     * Passive observation only.
     * Never alter 'value' or return early because of these tests.
     */
    if (addr == 0x43F0) {
		
		static uint8_t last_cmd = 0xFF;

		if (value != last_cmd) {
			/* fprintf(stderr,
                "[MAILBOX] cmd=%02X state=%d runtime=%d\n",
                value,
                (int)g_video_display_state,
                g_runtime_palette_enabled); */

			last_cmd = value;
		}
		/*
		* Dump the complete 16-byte host -> SP mailbox packet whenever it
		* changes.
		*
		* Do this when the SP actually reads the command byte, rather than
		* when the host first writes $03F0, because the 68010 may construct
		* packets using several separate writes.
		*/
		{
			static uint8_t last_packet[16];
			static int last_packet_valid = 0;
			uint32_t mailbox_base =
			(((g_io[0x02] >> 2) & 1) * 0x8000u) + 0x03F0u;
			uint8_t packet[16];
			int changed = !last_packet_valid;

			for (int i = 0; i < 16; i++) {
				packet[i] = g_vram[mailbox_base + i];

				if (last_packet_valid &&
					packet[i] != last_packet[i]) {
					changed = 1;
				}
			}

		}
	
        if (value == 0x0F &&
            g_video_display_state == VIDEO_DISPLAY_PALE_BLUE) {
            g_video_display_state = VIDEO_DISPLAY_DARK_BLUE;
			/*SDL_Delay(2500);*/
        }

        if (value == 0x0A) {
            video_display_host_handover();
        }
    }

    return value;
}
    /* CRTC */
    if (addr == 0x0400) return crtc_status();
    if (addr == 0x0401) {
        uint8_t v = g_crtc_idx < 18 ? g_crtc_regs[g_crtc_idx] : 0;
        return v;
    }
    /* 6840 PTM at $0100-$0107 */
    if (addr >= 0x0100 && addr < 0x0108) return ptm_read(addr - 0x0100);
    /* 6850 ACIA (serial mouse) at $0200 (status) / $0201 (data) */
    if (addr == 0x0200) {
        uint8_t s = acia_status();
        return s;
    }
    if (addr == 0x0201) {
        g_acia_rdrf = 0;
        return g_acia_rdr;
    }
    /* HD146818 RTC at $0300-$033F */
    if (addr >= 0x0300 && addr < 0x0340) return rtc_read(addr - 0x0300);
    
    /* Palette read-back. */
    if (addr >= 0x0500 && addr < 0x0510)
        return g_palette[addr & 0x0f];
	
	
	/* External area -- everything else through catch-all. */
    uint8_t v = g_ram[addr];
    io_log("R", addr, v);
	
    return v;
}

/* SP-side trace: armed when CARETAKER starts handling the host's $3F0
 * command, then logs every SP memory write so the response path -- where
 * the SP puts the answer / how it signals the host -- is visible. */
int g_sp_resp_trace = 0;
static void mem_write(uint16_t addr, uint8_t val) {
    /* Trace SP writes to ACIA control ($0200) -- look for when (if ever)
     * the SP enables Rx interrupts (bit 7) so the mouse path can work.
     * Cap normal writes but always announce a bit-7-set write (= Rx IRQ
     * actually enabled, which is the event we are looking for). */
    if (addr < 0x20)              { io_reg_write(addr, val); return; }
    if (addr >= 0x40 && addr < 0x100) { g_intram[addr - 0x40] = val; return; }
    if (addr >= ROM_BASE)         return;  /* ROM, ignore writes */
    /* VRAM, banked via P1.2 */
    if (addr >= 0x4000 && addr < 0xC000) {
        if (addr == 0xBFFF) g_bfff_reads = 0;
        uint32_t vo = sp_vram_offset(addr);
        g_vram[vo] = val;
        return;
    }
    /* CRTC */
    if (addr == 0x0400) { g_crtc_idx = val & 0x1F; return; }
    if (addr == 0x0401) {
        if (g_crtc_idx < 18) g_crtc_regs[g_crtc_idx] = val;
        return;
    }
    /* 6840 PTM */
    if (addr >= 0x0100 && addr < 0x0108) { ptm_write(addr - 0x0100, val); return; }
    /* 6850 ACIA: $0200 = control register, $0201 = transmit data */
    if (addr == 0x0200) {
        g_acia_ctrl = val;
        if ((val & 0x03) == 0x03) g_acia_rdrf = 0;   /* master reset */
        return;
    }
    if (addr == 0x0201) return;                       /* SP->mouse TX, discard */
    /* HD146818 RTC */
    if (addr >= 0x0300 && addr < 0x0340) { rtc_write(addr - 0x0300, val); return; }
    /* Palette.  The initial sixteen E0 writes are the Caretaker startup
     * initialisation.  Completion of that sequence changes the display from
     * pale blue to dark blue.  Once host-driven operation has begun, every
     * palette write is a genuine programmable-palette update. */
    if (addr >= 0x0500 && addr < 0x0510) {
        unsigned index = addr & 0x0f;

		/*fprintf(stderr,
			"[RUNTIME PALETTE WRITE] index=%u value=%02X\n",
			index, val); */

		g_palette[index] = val;

		
		/*
		* Always retain the physical palette-RAM write, but do not let the
		* initial E0 initialisation expose the contents of VRAM.
		*
		* The early framebuffer contains intermediate data while the service
		* processor prepares Caretaker/WERMA.  The real machine masks this with
		* the uniform pale-blue display.
		*/
		if (g_runtime_palette_enabled) {
			g_runtime_palette_programmed = 1;
			g_video_display_state = VIDEO_DISPLAY_NORMAL;
		}
				
        return;
    }
    g_ram[addr] = val;
    io_log("W", addr, val);
}

static uint16_t mem_read16(uint16_t addr) {
    return (mem_read(addr) << 8) | mem_read((addr + 1) & 0xFFFF);
}
static void mem_write16(uint16_t addr, uint16_t val) {
    mem_write(addr, val >> 8);
    mem_write((addr + 1) & 0xFFFF, val & 0xFF);
}

/* Fetch byte from PC and advance PC. */
static uint8_t fetch8(void)  { return mem_read(cpu.pc++); }
static uint16_t fetch16(void){ uint16_t v = mem_read16(cpu.pc); cpu.pc += 2; return v; }

/* --- Condition code helpers --- */
static inline void set_nz_8(uint8_t v) {
    cpu.cc = (cpu.cc & ~(CC_N | CC_Z))
           | (v & 0x80 ? CC_N : 0)
           | (v == 0   ? CC_Z : 0);
}
static inline void set_nz_16(uint16_t v) {
    cpu.cc = (cpu.cc & ~(CC_N | CC_Z))
           | (v & 0x8000 ? CC_N : 0)
           | (v == 0     ? CC_Z : 0);
}

/* --- Push / Pull (stack grows downward) --- */
static void push8(uint8_t v)  { mem_write(cpu.sp, v); cpu.sp--; }
static uint8_t pull8(void)    { cpu.sp++; return mem_read(cpu.sp); }
static void push16(uint16_t v){ push8(v & 0xFF); push8(v >> 8); }
static uint16_t pull16(void)  { uint8_t hi = pull8(); uint8_t lo = pull8(); return (hi << 8) | lo; }

/* --- Reset & interrupts --- */
static void cpu_reset(void) {
    video_display_reset();
	rtc_sync_from_host();
    cpu.cc = CC_I | 0xC0;  /* I=1, top two bits always 1 */
    cpu.a = cpu.b = 0;
    cpu.ix = 0;
    cpu.sp = 0xFF;
    cpu.pc = mem_read16(0xFFFE);
    fprintf(stderr, "[BOOT] reset vector = %04X\n", cpu.pc);
}

static void cpu_interrupt(uint16_t vec_addr) {
    if (cpu.cc & CC_I) return;  /* IRQ masked */
    push16(cpu.pc);
    push16(cpu.ix);
    push8(cpu.a);
    push8(cpu.b);
    push8(cpu.cc);
    cpu.cc |= CC_I;
    cpu.pc = mem_read16(vec_addr);
}

/* --- Addressing modes (return address or operand) --- */
static uint16_t am_direct(void)  { return fetch8(); }
static uint16_t am_ext(void)     { return fetch16(); }
static uint16_t am_indexed(void) { return (cpu.ix + fetch8()) & 0xFFFF; }

static uint8_t op_imm8(void)     { return fetch8(); }
static uint16_t op_imm16(void)   { return fetch16(); }
static uint8_t op_dir8(void)     { return mem_read(am_direct()); }
static uint16_t op_dir16(void)   { return mem_read16(am_direct()); }
static uint8_t op_ext8(void)     { return mem_read(am_ext()); }
static uint16_t op_ext16(void)   { return mem_read16(am_ext()); }
static uint8_t op_idx8(void)     { return mem_read(am_indexed()); }
static uint16_t op_idx16(void)   { return mem_read16(am_indexed()); }

/* --- ALU helpers --- */
static uint8_t alu_add8(uint8_t a, uint8_t b, int with_c) {
    int c_in = with_c ? (cpu.cc & CC_C ? 1 : 0) : 0;
    int r = a + b + c_in;
    int h = ((a & 0xF) + (b & 0xF) + c_in) > 0xF;
    int v = (~(a ^ b) & (a ^ r) & 0x80) != 0;
    cpu.cc = (cpu.cc & ~(CC_H | CC_N | CC_Z | CC_V | CC_C))
           | (h ? CC_H : 0)
           | ((uint8_t)r & 0x80 ? CC_N : 0)
           | ((uint8_t)r == 0   ? CC_Z : 0)
           | (v ? CC_V : 0)
           | (r > 0xFF ? CC_C : 0);
    return r & 0xFF;
}

static uint8_t alu_sub8(uint8_t a, uint8_t b, int with_c) {
    int c_in = with_c ? (cpu.cc & CC_C ? 1 : 0) : 0;
    int r = a - b - c_in;
    int v = ((a ^ b) & (a ^ r) & 0x80) != 0;
    cpu.cc = (cpu.cc & ~(CC_N | CC_Z | CC_V | CC_C))
           | ((uint8_t)r & 0x80 ? CC_N : 0)
           | ((uint8_t)r == 0   ? CC_Z : 0)
           | (v ? CC_V : 0)
           | (r < 0 ? CC_C : 0);
    return r & 0xFF;
}

static uint8_t alu_and8(uint8_t a, uint8_t b) {
    uint8_t r = a & b;
    cpu.cc = (cpu.cc & ~(CC_N | CC_Z | CC_V))
           | (r & 0x80 ? CC_N : 0)
           | (r == 0   ? CC_Z : 0);
    return r;
}
static uint8_t alu_or8(uint8_t a, uint8_t b) {
    uint8_t r = a | b;
    cpu.cc = (cpu.cc & ~(CC_N | CC_Z | CC_V))
           | (r & 0x80 ? CC_N : 0)
           | (r == 0   ? CC_Z : 0);
    return r;
}
static uint8_t alu_eor8(uint8_t a, uint8_t b) {
    uint8_t r = a ^ b;
    cpu.cc = (cpu.cc & ~(CC_N | CC_Z | CC_V))
           | (r & 0x80 ? CC_N : 0)
           | (r == 0   ? CC_Z : 0);
    return r;
}

/* Returns 1 on each instruction step, 0 on halt. */
static int cpu_step(void);

/* Forward declarations for the big switch -- kept in cpu_step itself. */

#ifdef USE_M68K
/* ===== MC68010 host CPU =====
 * Service processor and host share a 32KB RAM window: SP sees it at
 * $4000-$BFFF in its address space; host sees it (best guess) at host
 * $00000000-$00007FFF.  The SP copies ROM $D000-$FFEF into this window at
 * boot; with vectors mapped at host $0, the host should pick up SSP from
 * $0 and PC from $4 (= $0000000C per the copied ROM table). */

/* Globals the patched Musashi from unisoft/tp32 references.  All can be
 * zero-init; the diagnostic paths stay dormant. */
int g_debug = 0;
int g_verbose = 0;
int g_force_pmmu_enabled = 0;
int g_kernel_pmmu_active = 0;
int g_pmmu_diag_mode = 0;
int g_fault_trace_count = 0;
int g_fpu_model_version = 0;
int g_ls_trace_active = 0;
int g_ls_trace_count = 0;
int g_m68k_current_fc = 0;
int g_pflush_count_test0 = 0;
int g_post_exec_fault_trace = 0;
int g_prom_trace = 0;
int g_syscall_return_count = 0;
int g_syscall_return_num = 0;
int g_syscall_return_pending = 0;
int g_test0_trigger_active = 0;
int g_test_tu_e000_access_occurred = 0;
int g_test_tu_e000_fault_occurred = 0;
int g_test_tu_running = 0;
int g_test_x_halt_occurred = 0;
int g_test_x_passed = 0;
int g_trace_copyout = 0;
int g_trace_syscalls = 0;
int g_tt_cache_inhibit = 0;
int cpu_log_enabled = 0;
unsigned long g_insn_count = 0;
uint8_t g_test_tu_data[4] = {0};
int g_trace_pc = 0;
int g_log_berr = 0;
int g_log_unmapped_host = 0;
extern uint64_t g_insn;          /* host instruction counter (defined below) */
uint32_t g_dram_base = 0;
uint32_t g_dram_size = 0x100000;
uint8_t *g_rom = NULL;
uint8_t *g_dram = NULL;
uint32_t g_scsi_xfer_actual = 0;

/* Host CPU memory map (per MAME mame/torch/triplex.cpp):
 *   $00000000-$0000FFFF   VRAM 64KB (host sees both banks at once)
 *   $00040000-$00040007   Z8530 SCC                    (IRQ5)
 *   $00080000-$0008003F   HD63450 DMA controller       (IRQ3)
 *   $000C0000-$000C0003   AM7990 LANCE Ethernet        (IRQ2)
 *   $000E0000-$000E000F   NCR5380 SCSI                 (IRQ4)
 *   $00100000-$001FFFFF   Main RAM (DRAM, 1MB)
 *   $00200000-$002FFFFF   Limpet/VMEbus
 *   $00300000+            bus error
 * IRQ 6 = CRTC VSYNC.
 *
 * VRAM is 64KB total.  The SP sees 32KB at a time at $4000-$BFFF via
 * port-1 bit 2: P1.2=0 → bank 0 (host $0-$7FFF); P1.2=1 → bank 1
 * (host $8000-$FFFF).  P1.3 holds the host CPU in HALT/RESET. */
#define HOST_DRAM_BASE 0x100000u
/* Stock Triple X has 1MB DRAM at $100000-$1FFFFF.  The Limpet VMEbus card
 * adds another 1MB at $200000-$2FFFFF.  We emulate the expanded machine
 * (some kernels -- e.g. torch2.img -- require it). */
#define HOST_DRAM_SIZE 0x400000u   /* 2MB DRAM (stock + Limpet expansion) */
uint8_t g_host_io_scc  [0x10];
uint8_t g_host_io_dma  [0x40];
uint8_t g_host_io_lance[0x10];
/* NCR5380 SCSI controller (per MAME mame/machine/ncr5380.cpp).
 * Registers (on the upper byte of the 16-bit bus, so SP-side word offsets
 * 0,2,4,6,8,A,C,E map to NCR regs 0..7):
 *   0  Output Data / Current SCSI Data
 *   1  Initiator Command Register  (BSY=$40 SEL=$04 RST=$80 ...)
 *   2  Mode Register               (ARBITRATE=$01 DMA=$02 ...)
 *   3  Target Command Register
 *   4  Select Enable (W) / Current SCSI Bus Status (R)
 *   5  Start DMA Send / Bus and Status
 *   6  Set DMA Send / Input Data
 *   7  Start DMA Recv / Reset Parity/Interrupt
 *
 * We model only what CARETAKER's selection sequence needs: a clean
 * BSY=0 response (no target devices on the bus), arbitration that
 * "wins" trivially, and a selection that simply times out. */
/* NCR5380 bus phases (TCR low 3 bits = MSG/CD/IO encoded in current phase) */
#define PHASE_BUS_FREE  0
#define PHASE_ARB       1
#define PHASE_SELECT    2
#define PHASE_COMMAND   3   /* C/D=1, I/O=0, MSG=0 → TCR low3 = 010 */
#define PHASE_DATA_OUT  4   /* C/D=0, I/O=0, MSG=0 → 000 */
#define PHASE_DATA_IN   5   /* C/D=0, I/O=1, MSG=0 → 001 */
#define PHASE_STATUS    6   /* C/D=1, I/O=1, MSG=0 → 011 */
#define PHASE_MSG_IN    7   /* C/D=1, I/O=1, MSG=1 → 111 */

typedef struct {
    uint8_t odata;
    uint8_t icr;
    uint8_t mode;
    uint8_t tcr;
    uint8_t selen;
    uint8_t bas;
    int     phase;
    int     bsy_target;       /* target's BSY assertion (1=target driving BSY) */
    int     req_target;       /* target's REQ assertion */
    int     phase_delay;      /* simple read-counter for phase transitions */
    int     aip;              /* arbitration-in-progress (ICR read bit 6) */
    uint8_t cmd_bytes[12];    /* SCSI command buffer (CDB) */
    int     cmd_idx;
    int     cmd_len;
    uint8_t status_byte;
    uint8_t msg_byte;
    /* DATA_IN buffer for serving READ/INQUIRY responses. */
    uint8_t *data_buf;        /* points into g_disk_image or a tiny static buf */
    uint32_t data_len;        /* remaining bytes to transfer */
    uint8_t  inquiry_buf[36]; /* INQUIRY response staging area */
    uint8_t  selected_data;   /* data bus value latched during SCSI SELECT */
} ncr5380_t;
static ncr5380_t g_ncr;

/* Backing raw images for the SCSI targets (read-only).  Unit 0 = the hard
 * disc (--disk); unit 1 = the SCSI floppy (--unix-floppy) / "key disk" (--keydisk).
 * On a real Triple X the floppy is a SCSI device behind a SCSI-to-floppy
 * bridge, so it is just another NCR5380 target -- no separate FDC. */
static uint8_t *g_disk_image   = NULL;
static size_t   g_disk_size    = 0;
static int g_disk_writeable    = 0;

#ifdef _WIN32
static int g_disk_fd = -1;
#endif

static uint8_t *g_keydisk_image = NULL;
static size_t   g_keydisk_size  = 0;


/* Raw Unix-mountable floppy image.
 * This is separate from the IMD keydisk image.
 * Unit 1 will point at this image for normal Torch Unix mount/read/write. */
static uint8_t *g_unix_floppy_image = NULL;
static size_t   g_unix_floppy_size  = 0;
static int 	    g_unix_floppy_writeable = 0;

#ifdef _WIN32
static int g_unix_floppy_fd = -1;
#endif

#define SCSI_BLOCK_SIZE 512

/* The Torch MANTA SCSI-to-floppy bridge can be reconfigured (via SCSI
 * MODE SELECT) to 128-byte sectors so the host can read the floppy's
 * special last track (a 16x128-byte "label" track holding the key-disc
 * serial record).  g_keydisk_128mode tracks that reconfigure. */
static int     g_keydisk_128mode = 0;
static uint8_t g_modesel_buf[260];   /* MODE SELECT parameter-list receive buffer */
static int g_keydisk_selected = 0;
#ifdef _WIN32

static uint8_t *host_load_image(
    const char *filename,
    size_t *image_size,
    int *image_fd,
    int *writeable)
{
    int fd;
    __int64 size;
    uint8_t *buffer;
    int open_flags;

    if (filename == NULL || image_size == NULL ||
        image_fd == NULL || writeable == NULL) {
        return NULL;
    }

    /*
     * First try to open the image read/write.
     */
    open_flags = _O_RDWR | _O_BINARY;
    fd = _open(filename, open_flags);

    if (fd >= 0) {
        *writeable = 1;
    } else {
        /*
         * Fall back to read-only access.
         */
        open_flags = _O_RDONLY | _O_BINARY;
        fd = _open(filename, open_flags);

        if (fd < 0) {
            perror(filename);
            return NULL;
        }

        *writeable = 0;
    }

    size = _lseeki64(fd, 0, SEEK_END);

    if (size <= 0) {
        fprintf(stderr, "%s: invalid or empty image file\n", filename);
        _close(fd);
        return NULL;
    }

    if (_lseeki64(fd, 0, SEEK_SET) < 0) {
        perror(filename);
        _close(fd);
        return NULL;
    }

    buffer = malloc((size_t)size);

    if (buffer == NULL) {
        fprintf(stderr,
                "%s: unable to allocate %lld bytes\n",
                filename,
                (long long)size);
        _close(fd);
        return NULL;
    }

    {
        size_t total_read = 0;

        while (total_read < (size_t)size) {
            unsigned int chunk;
            int bytes_read;

            size_t remaining = (size_t)size - total_read;

            /*
             * _read() accepts an unsigned int byte count.
             */
            if (remaining > 1024U * 1024U) {
                chunk = 1024U * 1024U;
            } else {
                chunk = (unsigned int)remaining;
            }

            bytes_read = _read(fd, buffer + total_read, chunk);

            if (bytes_read <= 0) {
                fprintf(stderr,
                        "%s: failed while reading image\n",
                        filename);

                free(buffer);
                _close(fd);
                return NULL;
            }

            total_read += (size_t)bytes_read;
        }
    }

    *image_size = (size_t)size;
    *image_fd = fd;

    return buffer;
}

#endif

#ifdef _WIN32

static int host_write_image_data(
    int fd,
    size_t offset,
    const uint8_t *data,
    size_t length)
{
    size_t total_written = 0;

    if (fd < 0 || data == NULL) {
        return -1;
    }

    if (_lseeki64(fd, (__int64)offset, SEEK_SET) < 0) {
        return -1;
    }

    while (total_written < length) {
        unsigned int chunk;
        int bytes_written;
        size_t remaining = length - total_written;

        if (remaining > 1024U * 1024U) {
            chunk = 1024U * 1024U;
        } else {
            chunk = (unsigned int)remaining;
        }

        bytes_written = _write(
            fd,
            data + total_written,
            chunk
        );

        if (bytes_written <= 0) {
            return -1;
        }

        total_written += (size_t)bytes_written;
    }

    return 0;
}

#endif

void ncr_load_disk(const char *path) {
    /* mmap(MAP_SHARED) the disk image so SCSI WRITE(6/10) commands go
     * straight to the file -- the kernel's writes (file creation, syslog,
     * shell history, etc.) persist across emulator restarts.  If the file
     * can't be mapped writeable (e.g. it's read-only, or a .gz), fall back
     * to a read-only mmap; in that case writes go to in-memory copy-on-
     * write pages and are lost on exit. */
	 
#ifdef _WIN32

    g_disk_image = host_load_image(
        path,
        &g_disk_size,
        &g_disk_fd,
        &g_disk_writeable
    );

    if (g_disk_image == NULL) {
        fprintf(stderr,
                "Unable to load disk image: %s\n",
                path);
        return;
    }


#else
		  
    int fd = open(path, O_RDWR);
    int g_disk_writeable = 1;
    if (fd < 0) {
        fd = open(path, O_RDONLY);
        g_disk_writeable = 0;
        if (fd < 0) { perror(path); return; }
    }
    struct stat st;
    if (fstat(fd, &st) != 0) { perror("fstat"); close(fd); return; }
    g_disk_size = st.st_size;
    int prot = g_disk_writeable ? (PROT_READ | PROT_WRITE) : PROT_READ;
    int flag = g_disk_writeable ? MAP_SHARED : MAP_PRIVATE;
    g_disk_image = (uint8_t *)mmap(NULL, g_disk_size, prot, flag, fd, 0);
    if (g_disk_image == MAP_FAILED) {
        perror("mmap disk");
        g_disk_image = NULL;
        close(fd);
        return;
	
    }
    close(fd);  /* the mapping keeps the file alive */
#endif	

    fprintf(stderr, "[DISK] %s: %zu bytes, %zu × %d-byte blocks (%s)\n",
            path, g_disk_size, g_disk_size / SCSI_BLOCK_SIZE, SCSI_BLOCK_SIZE,
            g_disk_writeable ? "writes persist to file" : "read-only");
	return;
}

/* One decoded sector from an IMD image. */
typedef struct {
    int cyl, head, sec, size;
    const uint8_t *src;       /* normal data (NULL if RLE-compressed) */
    int fill;                 /* RLE fill byte when src == NULL */
} imd_sec_t;

static int imd_sec_cmp(const void *a, const void *b) {
    const imd_sec_t *x = (const imd_sec_t *)a, *y = (const imd_sec_t *)b;
    if (x->cyl  != y->cyl)  return x->cyl  - y->cyl;
    if (x->head != y->head) return x->head - y->head;
    return x->sec - y->sec;
}

/* Decode an ImageDisk (.IMD) floppy image.  IMD layout: an ASCII header
 * terminated by a 0x1A byte, then per-track records: [mode][cyl][head]
 * [nsec][sizecode], a sector-numbering map, optional cylinder/head maps
 * (head byte bits 7/6), then nsec sector data records (type 0=unavailable,
 * odd=normal data, even=RLE-compressed).  The Torch SCSI-to-floppy bridge
 * presents the disc as a flat run of logical blocks, so we decode every
 * sector and concatenate them in (cyl,head,sector) order -- this also
 * copes with mixed-format discs (e.g. a final 16x128 Torch label track). */
void ncr_load_keydisk_imd(const char *path) {
    static const int ssz_tab[8] = {128,256,512,1024,2048,4096,8192,16384};  
    static int g_have_saved_serial = 0; /*Flag to say if we found it*/

    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return; }
    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *raw = (uint8_t *)malloc(fsz);
    if (!raw) { (void)0; fclose(f); return; }
    fread(raw, 1, fsz, f);
    fclose(f);
    long start = 0;
    while (start < fsz && raw[start] != 0x1A) start++;
    start++;

    int cap = 4096, nsecs = 0;
    imd_sec_t *secs = (imd_sec_t *)malloc(cap * sizeof(*secs));
    size_t total = 0;
    for (long q = start; q + 5 <= fsz; ) {
        int cyl = raw[q+1], hb = raw[q+2], nsec = raw[q+3], sc = raw[q+4];
        int head = hb & 0x3F, sz = ssz_tab[sc & 7];
        q += 5;
        const uint8_t *smap = &raw[q]; q += nsec;
        if (hb & 0x80) q += nsec;          /* skip optional cylinder map */
        if (hb & 0x40) q += nsec;          /* skip optional head map */
        for (int i = 0; i < nsec && q < fsz; i++) {
            int t = raw[q++];
            if (nsecs == cap) { cap *= 2; secs = realloc(secs, cap * sizeof(*secs)); }
            imd_sec_t *s = &secs[nsecs++];
            s->cyl = cyl; s->head = head; s->sec = smap[i]; s->size = sz;
            if (t & 1)      { s->src = &raw[q]; s->fill = -1; q += sz; }  /* normal */
            else if (t)     { s->src = NULL;    s->fill = raw[q]; q += 1; } /* RLE */
            else            { s->src = NULL;    s->fill = 0; }            /* missing */
            total += sz;
        }
    }
    qsort(secs, nsecs, sizeof(*secs), imd_sec_cmp);
    g_keydisk_size  = total;
    g_keydisk_image = (uint8_t *)malloc(total ? total : 1);
    size_t o = 0;
    for (int i = 0; i < nsecs; i++) {
        if (secs[i].src) memcpy(g_keydisk_image + o, secs[i].src, secs[i].size);
        else             memset(g_keydisk_image + o, secs[i].fill, secs[i].size);
        o += secs[i].size;
    }
    free(secs);
    free(raw);
    fprintf(stderr, "[FLOPPY] %s: %d sectors -> %zu bytes (%zu × %d-byte blocks)\n",
            path, nsecs, g_keydisk_size, g_keydisk_size / SCSI_BLOCK_SIZE, SCSI_BLOCK_SIZE);

    /* A Torch key disc carries its serial number in the special last
     * track, 32 bytes into the "KRN, CMB & PRW are the greatest" record.
     * Preload that serial into the RTC's battery-backed CMOS (offsets
     * $0E-$11, big-endian) so the machine's serial number matches the
     * key disc -- WERMA's Caretaker then validates it cleanly instead of
     * showing the "New battery fitted" / "Invalid key disc" prompts. */
    FILE *fp = fopen("torch_serial.bin", "rb");
    if(fp){
        g_have_saved_serial = 1;
    }

    if (g_have_saved_serial < 1){
        if (g_keydisk_size >= 2048 + 36 && !g_have_saved_serial) {
            const uint8_t *rec = g_keydisk_image + (g_keydisk_size - 2048);
            if (memcmp(rec, "KRN, CMB & PRW are the greatest", 31) == 0) {
                memcpy(&g_rtc_user_ram[0x0E], rec + 32, 4);
                memcpy(g_saved_serial, rec + 32, 4);
                
                FILE *fp = fopen("torch_serial.bin", "rb");
    
                if (!fp){
                    fp = fopen("torch_serial.bin", "wb");
                    
                    if (fp){
                        fwrite(g_saved_serial, 1,  4, fp);
                        fclose(fp);
                        /*'printf("Saved torch_serial.bin\n");*/
                        fprintf(stderr, "[FLOPPY] key-disc serial $%02X%02X%02X%02X -> RTC CMOS"
                        " (machine serial now matches the key disc)\n",
                        rec[32], rec[33], rec[34], rec[35]);
                    }
                }
    
            }   
        }
    }
    /* RTC CMOS "system options" word read by the kernel at $11B490:
     *   d0 = (cmos[$0312] << 8) | cmos[$0313]
     * The kernel boot then does `btst #4, d0` to decide whether to bring
     * up the B-NET driver.  Bit 4 of d0 = bit 4 of the low byte = bit 4
     * of cmos[$0313] (which is g_rtc_user_ram[0x13]).  Setting that bit
     * makes the kernel call BNETinit -> laattach, which probes the LANCE
     * at $C0002 with a single `clrw` (no bus error with our stub) so
     * B-NET shows up as "enabled" in the boot banner. */
    
    /*g_rtc_user_ram[0x13] |= 0x30;  /* bit 4 = B-NET, bit 5 = NFS */
    /*fprintf(stderr, "[CMOS] driver-enable bits set: B-NET (bit 4) and "
                    "NFS (bit 5) of cmos[$0313]\n");


    /* The kernel function gethernum() at $11B4C4 reads 6 bytes from SP
     * CMOS starting at SP address 0x314 to use as the LANCE Ethernet
     * MAC address.  Without these, the chip is configured with
     * 00:00:00:00:00:00 -- a host bridge / TAP interface drops frames
     * with that source MAC.  Synthesise a per-machine MAC from the
     * key-disc serial: 02:80:E1:<S1>:<S2>:<S3>.
     *   02:        locally-administered, unicast OUI prefix
     *   80:E1:     a "Torch"-like prefix for visual identification
     *   S1:S2:S3:  low 3 bytes of the machine serial number
     * The kernel reads this once at boot and writes it into the LANCE
     * init block PADR field. */

  /*  if (g_have_saved_serial){
        g_rtc_user_ram[0x14] = 0x02;
        g_rtc_user_ram[0x15] = 0x80;
        g_rtc_user_ram[0x16] = 0xE1;
        g_rtc_user_ram[0x17] = g_rtc_user_ram[0x0F];
        g_rtc_user_ram[0x18] = g_rtc_user_ram[0x10];
        g_rtc_user_ram[0x19] = g_rtc_user_ram[0x11];
        /* fprintf(stderr, "[CMOS] LANCE MAC seeded: %02X:%02X:%02X:%02X:%02X:%02X\n",
                g_rtc_user_ram[0x14], g_rtc_user_ram[0x15],
                g_rtc_user_ram[0x16], g_rtc_user_ram[0x17],
                g_rtc_user_ram[0x18], g_rtc_user_ram[0x19]);
        }


    else if (g_keydisk_size >= 2048 + 36) {
        const uint8_t *rec = g_keydisk_image + (g_keydisk_size - 2048);
        

        if (memcmp(rec, "KRN, CMB & PRW are the greatest", 31) == 0) {
            g_rtc_user_ram[0x14] = 0x02;
            g_rtc_user_ram[0x15] = 0x80;
            g_rtc_user_ram[0x16] = 0xE1;
            g_rtc_user_ram[0x17] = rec[33];
            g_rtc_user_ram[0x18] = rec[34];
            g_rtc_user_ram[0x19] = rec[35]; */
            /*fprintf(stderr, "[CMOS] LANCE MAC seeded: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    g_rtc_user_ram[0x14], g_rtc_user_ram[0x15],
                    g_rtc_user_ram[0x16], g_rtc_user_ram[0x17],
                    g_rtc_user_ram[0x18], g_rtc_user_ram[0x19]);
        }  
    fprintf(stderr, "[CMOS] LANCE MAC seeded: %02X:%02X:%02X:%02X:%02X:%02X\n",
                g_rtc_user_ram[0x14], g_rtc_user_ram[0x15],
                g_rtc_user_ram[0x16], g_rtc_user_ram[0x17],
                g_rtc_user_ram[0x18], g_rtc_user_ram[0x19]); */
    
}


void ncr_load_unix_floppy(const char *path) {
	
#ifdef _WIN32

    g_unix_floppy_image = host_load_image(
        path,
        &g_unix_floppy_size,
        &g_unix_floppy_fd,
        &g_unix_floppy_writeable
    );

    if (g_unix_floppy_image == NULL) {
        fprintf(stderr,
                "Unable to load Unix floppy image: %s\n",
                path);
        return;
    }

#else	
		
    int fd = open(path, O_RDWR);
    int g_unix_floppy_writeable = 1;

    if (fd < 0) {
        fd = open(path, O_RDONLY);
        g_unix_floppy_writeable = 0;
        if (fd < 0) {
            perror(path);
            return;
        }
    }

    struct stat st;
    if (fstat(fd, &st) != 0) {
        perror("fstat unix floppy");
        close(fd);
        return;
    }

    if (st.st_size <= 0) {
        fprintf(stderr, "[UNIX-FLOPPY] %s: empty image, not loaded\n", path);
        close(fd);
        return;
    }

    if ((st.st_size % SCSI_BLOCK_SIZE) != 0) {
        fprintf(stderr,
                "[UNIX-FLOPPY] warning: %s size %ld is not a multiple of %d\n",
                path, (long)st.st_size, SCSI_BLOCK_SIZE);
    }

    g_unix_floppy_size = st.st_size;

    int prot = g_unix_floppy_writeable ? (PROT_READ | PROT_WRITE) : PROT_READ;
    int flag = g_unix_floppy_writeable ? MAP_SHARED : MAP_PRIVATE;

    g_unix_floppy_image = (uint8_t *)mmap(NULL, g_unix_floppy_size,
                                          prot, flag, fd, 0);
    if (g_unix_floppy_image == MAP_FAILED) {
        perror("mmap unix floppy");
        g_unix_floppy_image = NULL;
        g_unix_floppy_size = 0;
        close(fd);
        return;
    }

    close(fd);

#endif

    fprintf(stderr,
            "[UNIX-FLOPPY] %s: %zu bytes, %zu x %d-byte blocks (%s)\n",
            path,
            g_unix_floppy_size,
            g_unix_floppy_size / SCSI_BLOCK_SIZE,
            SCSI_BLOCK_SIZE,
            g_unix_floppy_writeable ? "writes persist to file" : "read-only");
}





#define NCR_ICR_BSY     0x40
#define NCR_ICR_SEL     0x04
#define NCR_ICR_RST     0x80
#define NCR_ICR_ACK     0x10
#define NCR_ICR_ATN     0x02
#define NCR_MODE_ARB    0x01

/* Bus Status (reg 4) bits */
#define NCR_BUS_BSY     0x40
#define NCR_BUS_REQ     0x20
#define NCR_BUS_MSG     0x10
#define NCR_BUS_CD      0x08
#define NCR_BUS_IO      0x04
#define NCR_BUS_SEL     0x02
#define NCR_BUS_RST     0x80

/* Encode phase into Bus Status MSG/CD/IO bits. */
static uint8_t phase_to_bus(int phase) {
    switch (phase) {
    case PHASE_COMMAND:  return NCR_BUS_CD;                       /* MSG=0 CD=1 IO=0 */
    case PHASE_DATA_OUT: return 0;                                /* 000 */
    case PHASE_DATA_IN:  return NCR_BUS_IO;                       /* 001 */
    case PHASE_STATUS:   return NCR_BUS_CD | NCR_BUS_IO;          /* 011 */
    case PHASE_MSG_IN:   return NCR_BUS_MSG | NCR_BUS_CD | NCR_BUS_IO; /* 111 */
    default: return 0;
    }
}

static void ncr_advance_phase(void);
static void ncr_tick_select(void);

#ifdef _WIN32

typedef enum {
    HOST_WRITE_NONE = 0,
    HOST_WRITE_DISK,
    HOST_WRITE_UNIX_FLOPPY
} host_write_target_t;

static host_write_target_t g_pending_write_target = HOST_WRITE_NONE;
static size_t g_pending_write_offset = 0;
static size_t g_pending_write_length = 0;

#endif





/* Host interrupt lines (defined later, near the host CPU glue).  Per MAME's
 * triplex.cpp the NCR5380's IRQ output is wired to the 68010's IRQ4. */
void host_irq_assert(int level);
static void host_irq_clear(int level);

/* Raise the NCR5380 interrupt: set the Interrupt Request bit in the Bus
 * and Status register (reg 5 bit 4) and assert the host's IRQ4 line.  The
 * NCR5380 interrupts on phase mismatch / EOP / loss-of-BSY; the host's
 * disc driver sleeps on this rather than VSYNC-polling for completion. */
static void ncr_raise_irq(void) {
    g_ncr.bas |= 0x10;
    host_irq_assert(4);
}

/* Compute the current Bus and Status register (reg 5) value based on
 * NCR state.  Real NCR5380 derives this from live bus signals. */
static uint8_t ncr_compute_bas(void) {
    uint8_t v = g_ncr.bas;
    /* Phase Match: target BSY asserted AND current phase matches TCR's
     * MSG/CD/IO encoding.  Only meaningful while a target is on the bus. */
    if (g_ncr.bsy_target &&
        (g_ncr.tcr & 0x07) == (phase_to_bus(g_ncr.phase) >> 2))
        v |= 0x08;
    /* DMA Request: target REQ + DMA Mode enabled, in a data phase. */
    if ((g_ncr.mode & 0x02) && g_ncr.req_target &&
        (g_ncr.phase == PHASE_COMMAND  || g_ncr.phase == PHASE_DATA_OUT ||
         g_ncr.phase == PHASE_DATA_IN  || g_ncr.phase == PHASE_STATUS   ||
         g_ncr.phase == PHASE_MSG_IN))
        v |= 0x40;
    return v;
}

static uint8_t ncr_read(int reg) {
    ncr_tick_select();
    switch (reg) {
    case 0:
        /* Reg 0 read: PIO data fetch.  In STATUS/MSG_IN phases, return
         * the byte and auto-advance the REQ/ACK handshake -- otherwise
         * a host without explicit ACK pulsing would stall.  This matches
         * real NCR5380 if the host has DMA Mode disabled but lets the
         * read drive the phase machine. */
        if (g_ncr.phase == PHASE_STATUS) {
            uint8_t b = g_ncr.status_byte;
            ncr_advance_phase();
            return b;
        }
        if (g_ncr.phase == PHASE_MSG_IN) {
            uint8_t b = g_ncr.msg_byte;
            ncr_advance_phase();
            return b;
        }
        if (g_ncr.phase == PHASE_DATA_IN && g_ncr.data_len > 0){
            uint8_t b = *g_ncr.data_buf;
            ncr_advance_phase();
            return b;
        }
        return g_ncr.odata;
    case 1:
        /* ICR read: bits 7,4-0 are the latched control bits; bit 6 is the
         * live AIP (Arbitration In Progress) status and bit 5 is LA (Lost
         * Arbitration) -- NOT the values written to those positions. */
        return (g_ncr.icr & 0x9F) | (g_ncr.aip ? 0x40 : 0x00);
    case 2: return g_ncr.mode;
    case 3: return g_ncr.tcr;
    case 4: {
        /* Current SCSI Bus Status: BSY from initiator or target, plus
         * phase MSG/CD/IO bits when target is driving the bus, plus REQ
         * during data phases. */
        uint8_t v = 0;
        if (g_ncr.icr & NCR_ICR_BSY)  v |= NCR_BUS_BSY;
        if (g_ncr.icr & NCR_ICR_SEL)  v |= NCR_BUS_SEL;
        if (g_ncr.icr & NCR_ICR_RST)  v |= NCR_BUS_RST;
        if (g_ncr.bsy_target)         v |= NCR_BUS_BSY;
        if (g_ncr.req_target)         v |= NCR_BUS_REQ;
        if (g_ncr.phase >= PHASE_COMMAND)
            v |= phase_to_bus(g_ncr.phase);
        return v;
    }
    case 5: return ncr_compute_bas();
    case 6:
        /* Input Data Register: used by the host's DMA-in path.  Same
         * semantics as reg 0 for the current phase.  Reading also auto-
         * ACKs the byte, advancing the REQ/ACK handshake. */
        if (g_ncr.phase == PHASE_DATA_IN && g_ncr.data_len > 0) {
            uint8_t b = *g_ncr.data_buf;
            ncr_advance_phase();
            return b;
        }
        if (g_ncr.phase == PHASE_STATUS) {
            uint8_t b = g_ncr.status_byte;
            ncr_advance_phase();
            return b;
        }
        if (g_ncr.phase == PHASE_MSG_IN) {
            uint8_t b = g_ncr.msg_byte;
            ncr_advance_phase();
            return b;
        }
        return 0;
    case 7:
        /* Reset Parity/Interrupt: clears Parity (bit 5) and Interrupt
         * Request (bit 4) latches only, and drops the IRQ4 line.  EOP
         * (bit 7) stays set until the channel is rearmed. */
        g_ncr.bas &= ~0x30;
        host_irq_clear(4);
        return 0;
    }
    return 0;
}

static void ncr_write(int reg, uint8_t v) {
    switch (reg) {

    /*case 0: g_ncr.odata = v; break; */
    
    case 0:
        g_ncr.odata = v;
        if (g_ncr.phase == PHASE_SELECT) {
            g_ncr.selected_data = v;
        }
        break;


    case 1: {
        uint8_t old = g_ncr.icr;
        g_ncr.icr = v;
        if (v & NCR_ICR_RST) {
            g_ncr.mode = 0; g_ncr.tcr = 0; g_ncr.bas = 0;
            g_ncr.phase = PHASE_BUS_FREE;
            g_ncr.bsy_target = 0; g_ncr.req_target = 0;
            g_ncr.cmd_idx = 0; g_ncr.aip = 0;
            host_irq_clear(4);
        }
        /* SEL rising while BSY clear: starting selection. */

        if ((v & NCR_ICR_SEL) && !(old & NCR_ICR_SEL)) {

            g_ncr.selected_data = g_ncr.odata;

            /*fprintf(stderr,
                    "[SCSI SELECT] data=%02X\n",
                    g_ncr.selected_data);*/

            g_ncr.phase = PHASE_SELECT;
            g_ncr.phase_delay = 2;
        }


        /*if ((v & NCR_ICR_SEL) && !(old & NCR_ICR_SEL)) {
            g_ncr.phase = PHASE_SELECT;
            g_ncr.phase_delay = 2;
        } */

            



        /* SEL falling after target asserted BSY: end of selection. */
        if (!(v & NCR_ICR_SEL) && (old & NCR_ICR_SEL) &&
            g_ncr.phase == PHASE_SELECT && g_ncr.bsy_target) {
            g_ncr.phase = PHASE_COMMAND;
            g_ncr.req_target = 1;
            g_ncr.cmd_idx = 0;
            g_ncr.cmd_len = 6;  /* assume 6-byte CDB for INQUIRY/READ */
            g_ncr.bas &= ~0x94;  /* clear stale IRQ/EOP/BusyErr at new cmd */
        }
        /* ACK pulse drives REQ/ACK handshake → advance to next byte/phase. */
        if ((v & NCR_ICR_ACK) && !(old & NCR_ICR_ACK)) {
            ncr_advance_phase();
        }
        break;
    }
    case 2:
        /* Mode register.  Bit 0 = ARBITRATE: the host requests the SCSI
         * bus.  We are the sole initiator, so arbitration is won
         * immediately -- raise AIP (and never LA).  Clearing the bit ends
         * arbitration. */
        if ((v & 0x01) && !(g_ncr.mode & 0x01)) g_ncr.aip = 1;
        else if (!(v & 0x01))                   g_ncr.aip = 0;
        g_ncr.mode = v;
        break;
    case 3: g_ncr.tcr  = v; break;
    case 4: g_ncr.selen = v; break;
    case 5: case 6: case 7: break;
    }
}

/* Determine SCSI CDB length from the opcode's top 3 bits (per SCSI-2):
 *   000 = 6-byte (group 0)
 *   001 = 10-byte (group 1/2)
 *   010 = 10-byte
 *   100 = 16-byte
 *   101 = 12-byte
 * We treat unknown groups as 6-byte. */
static int scsi_cdb_length(uint8_t op) {
    switch ((op >> 5) & 0x07) {
    case 0: return 6;
    case 1: case 2: return 10;
    case 4: return 16;
    case 5: return 12;
    }
    return 6;
}

/* Dispatch the completed CDB.  Sets up data_buf/data_len + status, advances
 * to DATA_IN (for READ-like opcodes) or STATUS directly (for TEST UNIT
 * READY / others). */
static void scsi_dispatch_cdb(void) {
    uint8_t op = g_ncr.cmd_bytes[0];

    if (TRACE_SCSI){
        fprintf(stderr,
            "[SCSI] op=%02X unit=%d cdb=%02X %02X %02X %02X %02X %02X phase=%d\n",
            op,
            (g_ncr.cmd_bytes[1] >> 5) & 7,
            g_ncr.cmd_bytes[0],
            g_ncr.cmd_bytes[1],
            g_ncr.cmd_bytes[2],
            g_ncr.cmd_bytes[3],
            g_ncr.cmd_bytes[4],
            g_ncr.cmd_bytes[5],
            g_ncr.phase);
    }

    g_ncr.data_buf = NULL;
    g_ncr.data_len = 0;
    g_ncr.status_byte = 0x00;        /* default GOOD */
    /* SCSI unit/LUN from CDB byte 1 bits 5-7.  Unit 0 = the hard disc
     * (--disk); unit 1 = the SCSI floppy / "key disc" (--floppy); other
     * units are absent and must report "no device" so the host's bus
     * scan doesn't loop on phantom drives.  The Torch vendor commands
     * $C0/$C2 carry the unit in the same byte-1 field. */
    int unit = (g_ncr.cmd_bytes[1] >> 5) & 7;
    int target = -1;

    uint8_t target_bits = g_ncr.selected_data & 0x7F;

    for (int id = 0; id < 7; id++) {
        if (target_bits & (1u << id)) {
            target = id;
            break;
        }
    }

    /*
    * Caretaker's KEY function identifies the floppy controller with
    * MODE SENSE page 5 before accessing the key disc.  Normal Unix
    * floppy access does not issue this sequence.
    */
    if (unit == 2 &&
        g_ncr.cmd_bytes[0] == 0x1A &&
        (g_ncr.cmd_bytes[2] & 0x3F) == 0x05 &&
        g_keydisk_image) {

        g_keydisk_selected = 1;

        fprintf(stderr,
            "[SCSI] unit 2 switched to KEYDISK\n");
    }
    
    /*
     * Temporary SCSI diagnostic logging.
     *
     * This lets us see exactly what Caretaker asks the MANTA/SCSI
     * controller to do when the KEY utility is selected.
     */
    /*fprintf(stderr,
            "[SCSI CMD] sel=%02X op=%02X lun=%d\n",
            g_ncr.selected_data,
            g_ncr.cmd_bytes[0],
            unit); */
    /*
    for (int i = 0; i < g_ncr.cmd_len; i++) {
        fprintf(stderr, "%02X ", g_ncr.cmd_bytes[i]);
    } 

    fprintf(stderr,
            " key128=%d keydisk=%s unixfloppy=%s\n",
            g_keydisk_128mode,
            g_keydisk_image ? "YES" : "NO",
            g_unix_floppy_image ? "YES" : "NO"); */

    uint8_t *img = NULL;
    size_t   imgsz = 0;
    

    /*if      (unit == 0) {
        img = g_disk_image;   
        imgsz = g_disk_size;
    } /*
    /*else if (unit == 2) {
        img = g_keydisk_image;
        imgsz = g_keydisk_size;

    }*/
    /* else if (unit == 2) {
        if (g_keydisk_image) {
            img   = g_keydisk_image;
            imgsz = g_keydisk_size;
        }
        else {
            img   = g_unix_floppy_image;
            imgsz = g_unix_floppy_size;
        }*/

    if (target == 0 && unit == 0) {
        /* ID 0, LUN 0: main hard disk */
        img   = g_disk_image;
        imgsz = g_disk_size;
    }
    else if (target == 0 && unit == 2) {
        /* ID 0, LUN 2: ordinary Unix floppy */
        img   = g_unix_floppy_image;
        imgsz = g_unix_floppy_size;
    }
    else if (target == 1 && unit == 2) {
        /* ID 1, LUN 2: Torch keydisk / Gotek */
        img   = g_keydisk_image;
        imgsz = g_keydisk_size;
    }

        
    if (unit == 2) {
        const char *src =
            (img == g_keydisk_image)     ? "KEYDISK" :
            (img == g_unix_floppy_image) ? "UNIX-FLOPPY" :
            (img == g_disk_image)        ? "HARDDISK" :
                                       "NONE";

        if (op == 0x08 || op == 0xC0 || op == 0x25 ||
            op == 0x1A || op == 0x15) {

            /*fprintf(stderr,
                    "[U2] op=%02X src=%s keysel=%d key128=%d",
                    op, src, g_keydisk_selected,
                    g_keydisk_128mode); */

            if (op == 0x08) {
                uint32_t lba =
                    ((g_ncr.cmd_bytes[1] & 0x1F) << 16) |
                    (g_ncr.cmd_bytes[2] << 8) |
                     g_ncr.cmd_bytes[3];
    
                /*fprintf(stderr, " READ6 lba=%u count=%u",
                        (unsigned)lba,
                        (unsigned)(g_ncr.cmd_bytes[4] ?
                                   g_ncr.cmd_bytes[4] : 256));*/
            }
    
            if (op == 0xC0) {
                uint32_t blk =
                    ((uint32_t)g_ncr.cmd_bytes[2] << 8) |
                     g_ncr.cmd_bytes[3];
    
                uint32_t len =
                    ((uint32_t)g_ncr.cmd_bytes[4] << 8) |
                     g_ncr.cmd_bytes[5];

                /*fprintf(stderr, " C0 blk=%u len=%u",
                        (unsigned)blk,
                        (unsigned)len); */
            }

            /*fprintf(stderr, "\n"); */
        }
    }   




    /* When the host has reconfigured the MANTA bridge to 128-byte sectors
     * (MODE SELECT), a unit-0 read targets the floppy's special last
     * track: the final 16x128-byte track of the IMD, which sits at the
     * end of the flat image and which the host addresses at 128-byte LBA
     * 0x9F0 (track 159 = cyl 79 head 1, x 16 sectors).  So in this mode a
     * read LBA L maps to floppy byte offset (size-2048) + (L-0x9F0)*128. */
    size_t   rd_base    = 0;
    uint32_t rd_lba_bias = 0;
    uint32_t rd_secsz   = SCSI_BLOCK_SIZE;
    if (g_keydisk_128mode && 
        target == 1 && 
        unit == 2 &&
        g_keydisk_image && 
        g_keydisk_size >= 2048) {
        
            img         = g_keydisk_image;
            imgsz       = g_keydisk_size;
            rd_base     = g_keydisk_size - 2048;
            rd_lba_bias = 0x9F0;
            rd_secsz    = 128;

            fprintf(stderr,
                    "[SCSI KEYDISK] redirecting unit 2 to IMD keydisk "
                    "size=%zu base=%zu LBA-bias=%04X sector=128\n",
                    g_keydisk_size,
                    rd_base,
                    (unsigned)rd_lba_bias);
    }

    /*
    * Once Caretaker leaves the KEY operation and resumes accessing
    * the hard disk, return unit 2 to the normal Unix floppy image.
    */
    /*if (g_keydisk_selected &&
        unit == 0 &&
        g_ncr.cmd_bytes[0] == 0x12) {

        g_keydisk_selected = 0;
        g_keydisk_128mode = 0;

        fprintf(stderr,
            "[SCSI] unit 2 returned to UNIX-FLOPPY\n");
    }*/
    

    if (op == 0x00) {                 /* TEST UNIT READY */
        g_ncr.status_byte = img ? 0x00 : 0x02;
    }
    else if (op == 0x12) {            /* INQUIRY */
        memset(g_ncr.inquiry_buf, 0, sizeof(g_ncr.inquiry_buf));
        if (!img) {
            /* absent unit: peripheral qualifier 011b + type 1Fh */
            g_ncr.inquiry_buf[0] = 0x7F;
            g_ncr.inquiry_buf[4] = 31;
        } else {
            g_ncr.inquiry_buf[0] = 0x00;  /* direct-access device */
            g_ncr.inquiry_buf[2] = 0x01;  /* ANSI SCSI-1 */
            g_ncr.inquiry_buf[3] = 0x01;  /* response data format */
            g_ncr.inquiry_buf[4] = 31;    /* additional length */
            /* The hard disc and the SCSI floppy both hang off the Torch
             * "MANTA" disc-controller bridge.  WERMA's boot-device probe
             * does strncmp(inquiry+16, "MANTA BOARD     ", 16) to find it,
             * so the product-identification field must be exactly that. */
            memcpy(g_ncr.inquiry_buf + 8,  "TORCH   ", 8);
            memcpy(g_ncr.inquiry_buf + 16, "MANTA BOARD     ", 16);
            memcpy(g_ncr.inquiry_buf + 32, "1.00", 4);
        }
        g_ncr.data_buf = g_ncr.inquiry_buf;
        g_ncr.data_len = g_ncr.cmd_bytes[4] < 36 ? g_ncr.cmd_bytes[4] : 36;
    }
    



    /*else if (op == 0x08 && img) {      /* READ (6) */
        /*uint32_t lba = ((g_ncr.cmd_bytes[1] & 0x1F) << 16) |
                       (g_ncr.cmd_bytes[2] << 8) | g_ncr.cmd_bytes[3];
        uint32_t cnt = g_ncr.cmd_bytes[4] ? g_ncr.cmd_bytes[4] : 256;
        size_t off = rd_base + (size_t)(lba - rd_lba_bias) * rd_secsz;
        size_t len = (size_t)cnt * rd_secsz;     
        if (lba >= rd_lba_bias && off + len <= imgsz) {
            g_ncr.data_buf = img + off;
            g_ncr.data_len = len;
        
        fprintf(stderr,
        "[SCSI READ6] unit=%d LBA=%u blocks=%u "
        "sector=%u key128=%d\n",
        unit,
        (unsigned)lba,
        (unsigned)cnt,
        (unsigned)rd_secsz,
        g_keydisk_128mode);

            
        } else {
            g_ncr.status_byte = 0x02;   /* CHECK CONDITION */
       /* }
    } */

    else if (op == 0x08 && img) {      /* READ (6) */

        uint32_t lba =
            ((g_ncr.cmd_bytes[1] & 0x1F) << 16) |
            (g_ncr.cmd_bytes[2] << 8) |
            g_ncr.cmd_bytes[3];

        uint32_t cnt =
            g_ncr.cmd_bytes[4] ?
            g_ncr.cmd_bytes[4] : 256;

        size_t off =
            rd_base +
            (size_t)(lba - rd_lba_bias) * rd_secsz;

            size_t len =
            (size_t)cnt * rd_secsz;

        /*
         * Diagnostic logging for the key-disk toolkit investigation.
         *
         * Caretaker reads LBAs 0, 5, 8, 14 and 15 while looking for
         * the Disk Toolkit.  Show exactly which backing image and byte
         * offsets are being returned.
         */
        if (unit == 2 &&
            (lba == 0 ||
             lba == 5 ||
             lba == 8 ||
             lba == 14 ||
             lba == 15)) {
    
            const char *source = "UNKNOWN";

            if (img == g_keydisk_image)
                source = "KEYDISK";

            else if (img == g_unix_floppy_image)
                source = "UNIX-FLOPPY";

            else if (img == g_disk_image)
                source = "HARDDISK";

            /*fprintf(stderr,
                    "\n[DISK READ] "
                    "LBA=%u count=%u sector=%u "
                    "offset=%zu length=%zu "
                    "source=%s image_size=%zu\n",
                    (unsigned)lba,
                    (unsigned)cnt,
                    (unsigned)rd_secsz,
                    off,
                    len,
                    source,
                    imgsz);*/
        }

        if (lba >= rd_lba_bias &&
            off + len <= imgsz) {

            /*
             * Dump the first 128 bytes of the sectors involved in the
             * Toolkit lookup.
             */
            if (unit == 2 &&
                (lba == 0 ||
                 lba == 5 ||
                 lba == 8 ||
                 lba == 14 ||
                 lba == 15)) {
    
                size_t dump_len =
                    len < 128 ? len : 128;

                /*fprintf(stderr,
                        "[KEYDISK HEX] LBA=%u\n",
                        (unsigned)lba);*/

                /*for (size_t i = 0;
                     i < dump_len;
                     i++) {

                    if ((i % 16) == 0)
                        fprintf(stderr,
                            "%04zX: ",
                            i);

                    fprintf(stderr,
                            "%02X ",
                            img[off + i]);

                    if ((i % 16) == 15)
                        fprintf(stderr, "\n");
                } */

                /*if ((dump_len % 16) != 0)
                    fprintf(stderr, "\n");

                fprintf(stderr,
                        "[KEYDISK ASCII] LBA=%u: ",
                        (unsigned)lba);

                for (size_t i = 0;
                     i < dump_len;
                     i++) {

                    unsigned char c =
                        img[off + i];

                    if (c >= 32 &&
                        c <= 126)
                        fputc(c, stderr);
                    else
                        fputc('.', stderr);
                }

                fprintf(stderr, "\n\n");*/
            }

            g_ncr.data_buf = img + off;
            g_ncr.data_len = len;

        } else {

            fprintf(stderr,
                    "[KEYDISK READ ERROR] "
                    "LBA=%u offset=%zu "
                    "length=%zu image_size=%zu\n",
                    (unsigned)lba,
                    off,
                    len,
                    imgsz);

            g_ncr.status_byte =
                0x02;   /* CHECK CONDITION */
    }
}





    else if (op == 0x28 && img) {       /* READ (10) */
        uint32_t lba = ((uint32_t)g_ncr.cmd_bytes[2] << 24) |
                       ((uint32_t)g_ncr.cmd_bytes[3] << 16) |
                       ((uint32_t)g_ncr.cmd_bytes[4] << 8)  |
                                   g_ncr.cmd_bytes[5];
        uint32_t cnt = ((uint32_t)g_ncr.cmd_bytes[7] << 8) | g_ncr.cmd_bytes[8];
        size_t off = (size_t)lba * SCSI_BLOCK_SIZE;
        size_t len = (size_t)cnt * SCSI_BLOCK_SIZE;
        if (off + len <= imgsz) {
            g_ncr.data_buf = img + off;
            g_ncr.data_len = len;
        } else {
            g_ncr.status_byte = 0x02;
        }
    }
    else if (op == 0x0A && img) {       /* WRITE (6) */
        uint32_t lba = ((g_ncr.cmd_bytes[1] & 0x1F) << 16) |
                       (g_ncr.cmd_bytes[2] << 8) | g_ncr.cmd_bytes[3];
        uint32_t cnt = g_ncr.cmd_bytes[4] ? g_ncr.cmd_bytes[4] : 256;
        size_t off = rd_base + (size_t)(lba - rd_lba_bias) * rd_secsz;
        size_t len = (size_t)cnt * rd_secsz;
        if (lba >= rd_lba_bias && off <= imgsz && len <= imgsz - off) {
            /* DATA_OUT phase: the DMA streams the host's bytes straight
             * into the disk image via ncr_advance_phase().  Without this
             * the swapper's process-image writes silently vanish and a
             * later swap-in reads stale disk content. */
            g_ncr.data_buf = img + off;
            g_ncr.data_len = len;
			
		#ifdef _WIN32
			g_pending_write_offset = off;
			g_pending_write_length = len;

			if (img == g_disk_image) {
				g_pending_write_target = HOST_WRITE_DISK;
			}
			else if (img == g_unix_floppy_image) {
				g_pending_write_target = HOST_WRITE_UNIX_FLOPPY;
			}
			else {
				g_pending_write_target = HOST_WRITE_NONE;
			}
		#endif
			
        } else {
            g_ncr.status_byte = 0x02;
        }
    }
    else if (op == 0x2A && img) {       /* WRITE (10) */
        uint32_t lba = ((uint32_t)g_ncr.cmd_bytes[2] << 24) |
                       ((uint32_t)g_ncr.cmd_bytes[3] << 16) |
                       ((uint32_t)g_ncr.cmd_bytes[4] << 8)  |
                                   g_ncr.cmd_bytes[5];
        uint32_t cnt = ((uint32_t)g_ncr.cmd_bytes[7] << 8) | g_ncr.cmd_bytes[8];
        size_t off = (size_t)lba * SCSI_BLOCK_SIZE;
        size_t len = (size_t)cnt * SCSI_BLOCK_SIZE;
        if (off <= imgsz &&
			len <= imgsz - off) {
            g_ncr.data_buf = img + off;
            g_ncr.data_len = len;
			
		#ifdef _WIN32
			g_pending_write_offset = off;
			g_pending_write_length = len;

			if (img == g_disk_image) {
				g_pending_write_target = HOST_WRITE_DISK;
			}
			else if (img == g_unix_floppy_image) {
				g_pending_write_target = HOST_WRITE_UNIX_FLOPPY;
			}
			else {
				g_pending_write_target = HOST_WRITE_NONE;
			}
		#endif
			
        } 
		
		else {
            g_ncr.status_byte = 0x02;
        }
    }
        
    /* ADDED BACK IN DURING ISSUE WITH DISK UTIL */

    else if (op == 0x25 && img) {       /* READ CAPACITY */

        static uint8_t cap[8];
        uint32_t last_lba;
        uint32_t block_size;

        /*
        * The Torch key disk is mixed-format:
        *
        *   1431 x 512-byte sectors
        *   final track = 16 x 128-byte sectors
        *
        * In normal 512-byte mode Caretaker expects it to present the
        * logical geometry of a standard 720K disk: 1440 sectors.
        */
        if (target == 1 &&
            unit == 2 &&
            img == g_keydisk_image &&
            !g_keydisk_128mode) {

            last_lba   = 1439;       /* 1440 logical sectors */
            block_size = 512;

        }
        else if (target == 1 &&
                 unit == 2 &&
                 img == g_keydisk_image &&
                 g_keydisk_128mode) {
    
            /*
             * Special-track geometry: logical LBAs 0x09F0-0x09FF
             * using 128-byte sectors.
             */
            last_lba   = 0x09FF;
            block_size = 128;
    
        }
        else {
    
            last_lba =
                (uint32_t)((imgsz / SCSI_BLOCK_SIZE) - 1);
    
            block_size = SCSI_BLOCK_SIZE;
        }
    
        cap[0] = (last_lba >> 24) & 0xFF;
        cap[1] = (last_lba >> 16) & 0xFF;
        cap[2] = (last_lba >> 8)  & 0xFF;
        cap[3] =  last_lba        & 0xFF;
    
        cap[4] = (block_size >> 24) & 0xFF;
        cap[5] = (block_size >> 16) & 0xFF;
        cap[6] = (block_size >> 8)  & 0xFF;
        cap[7] =  block_size        & 0xFF;
    
        g_ncr.data_buf = cap;
        g_ncr.data_len = 8;
    }   

    /* END OF SECTION ADDED BACK IN */






    /* TEMPORARILY OUT DURING INVESTIGATION OF DISK UTIL
    else if (op == 0x25 && img) {       /* READ CAPACITY */
        /*static uint8_t cap[8];
        uint32_t last_lba = (uint32_t)((imgsz / SCSI_BLOCK_SIZE) - 1);
        cap[0] = (last_lba >> 24) & 0xFF;
        cap[1] = (last_lba >> 16) & 0xFF;
        cap[2] = (last_lba >> 8)  & 0xFF;
        cap[3] =  last_lba        & 0xFF;
        cap[4] = (SCSI_BLOCK_SIZE >> 24) & 0xFF;
        cap[5] = (SCSI_BLOCK_SIZE >> 16) & 0xFF;
        cap[6] = (SCSI_BLOCK_SIZE >> 8)  & 0xFF;
        cap[7] =  SCSI_BLOCK_SIZE        & 0xFF;
        g_ncr.data_buf = cap;
        g_ncr.data_len = 8;
    } */
    


    else if (op == 0x03) {            /* REQUEST SENSE */
        static uint8_t sense[18];
        memset(sense, 0, sizeof(sense));
        sense[0] = 0x70;                /* current error, fixed format */
        sense[2] = 0x00;                /* sense key = NO SENSE */
        sense[7] = 10;                  /* additional length */
        g_ncr.data_buf = sense;
        g_ncr.data_len = g_ncr.cmd_bytes[4] < 18 ? g_ncr.cmd_bytes[4] : 18;
    }
    else if (op == 0x1A && img) {      /* MODE SENSE (6) */
        /* WERMA's MANTA-controller probe issues MODE SENSE page 5 and only
         * requires a successful (GOOD-status, non-empty) response.  Return
         * a minimal 4-byte mode parameter header + the requested page. */
        static uint8_t modesense[36];
        memset(modesense, 0, sizeof(modesense));
        modesense[0] = 35;              /* mode data length (total - 1) */
        modesense[4] = g_ncr.cmd_bytes[2] & 0x3F;  /* echo the page code */
        modesense[5] = 30;              /* page length */
        uint32_t alloc = g_ncr.cmd_bytes[4] ? g_ncr.cmd_bytes[4] : 36;
        g_ncr.data_buf = modesense;
        g_ncr.data_len = alloc < 36 ? alloc : 36;
    }
    else if (op == 0x15 && img) {      /* MODE SELECT (6) */
        /* The host reconfigures the MANTA bridge's sector size with this.
         * Receive the parameter list (DATA_OUT phase); ncr_advance_phase()
         * parses the block descriptor's block-length field and sets
         * g_keydisk_128mode.  GOOD status is returned after the data. */
        
        fprintf(stderr,
                "[SCSI MODE SELECT] unit=%d parameter_length=%u\n",
                unit,
                (unsigned)g_ncr.cmd_bytes[4]);


        uint32_t plen = g_ncr.cmd_bytes[4];
        if (plen > sizeof(g_modesel_buf)) plen = sizeof(g_modesel_buf);
        if (plen > 0) {
            memset(g_modesel_buf, 0, sizeof(g_modesel_buf));
            g_ncr.data_buf = g_modesel_buf;
            g_ncr.data_len = plen;
            g_ncr.phase = PHASE_DATA_OUT;
            g_ncr.req_target = 1;
            ncr_raise_irq();
            return;                     /* phase already set; skip the tail */
        }
        /* zero-length parameter list -- nothing to do */
    }
    else if (op == 0xC0 && img) {       /* Torch controller READ */
        /* CDB: C0 <unit> <blkHi> <blkLo> <lenHi> <lenLo>.  byte 1 selects
         * the unit; bytes 2-3 are a block number, bytes 4-5 a 16-bit
         * BYTE count. */
        uint32_t blk = ((uint32_t)g_ncr.cmd_bytes[2] << 8) | g_ncr.cmd_bytes[3];
        uint32_t len = ((uint32_t)g_ncr.cmd_bytes[4] << 8) | g_ncr.cmd_bytes[5];
        if (len == 0) len = SCSI_BLOCK_SIZE;
        size_t off = (size_t)blk * SCSI_BLOCK_SIZE;
        if (off < imgsz) {
            if (off + len > imgsz) len = imgsz - off;
            g_ncr.data_buf = img + off;
            g_ncr.data_len = len;
        } else {
            g_ncr.status_byte = 0x02;
        }
    }
    else if (op == 0xC2) {            /* Torch controller command (config/select) */
        g_ncr.status_byte = img ? 0x00 : 0x02;
    }
    else {
        g_ncr.status_byte = 0x02;       /* unsupported / absent → CHECK CONDITION */
    }

    if (g_ncr.data_len > 0 && (op == 0x0A || op == 0x2A)) {
        g_ncr.phase = PHASE_DATA_OUT;    /* WRITE: host -> disk image */
        g_ncr.req_target = 1;
    } else if (g_ncr.data_len > 0) {
        g_ncr.phase = PHASE_DATA_IN;
        g_ncr.req_target = 1;
    } else {
        g_ncr.phase = PHASE_STATUS;
        g_ncr.req_target = 1;
    }
    /* Phase change from COMMAND -> DATA/STATUS: NCR raises a phase-mismatch
     * IRQ (asserts the host's IRQ4 line). */
    ncr_raise_irq();
}

/* REQ/ACK handshake bookkeeping.  Called when the host pulses ACK. */
static void ncr_advance_phase(void) {
    switch (g_ncr.phase) {
    case PHASE_COMMAND:
        if (g_ncr.cmd_idx < g_ncr.cmd_len)
            g_ncr.cmd_bytes[g_ncr.cmd_idx++] = g_ncr.odata;
        /* First byte tells us the real CDB length. */
        if (g_ncr.cmd_idx == 1)
            g_ncr.cmd_len = scsi_cdb_length(g_ncr.cmd_bytes[0]);
        if (g_ncr.cmd_idx >= g_ncr.cmd_len)
            scsi_dispatch_cdb();
        break;
    
	
	case PHASE_DATA_OUT:
        /* Host -> target byte: a MODE SELECT parameter list, or a SCSI
         * WRITE streaming straight into the disk image (data_buf points
         * into the image for op $0A/$2A). */
        if (g_ncr.data_len > 0) {
            *g_ncr.data_buf++ = g_ncr.odata;
            g_ncr.data_len--;
        }
        if (g_ncr.data_len == 0) {
            /* MODE SELECT only: the Torch MANTA controller's mode page
             * carries the sector size at param-list bytes 10-11 ($0080 =
             * the floppy's 128-byte special-track geometry, $0200 the
             * normal 512-byte geometry).  WRITE commands skip this. */
            if (g_ncr.cmd_bytes[0] == 0x15) {
                uint32_t bps = ((uint32_t)g_modesel_buf[10] << 8) | g_modesel_buf[11];

                fprintf(stderr,
                        "[SCSI MODE SELECT DATA] requested sector size=%u\n",
                        (unsigned)bps);

                if (bps == 128){
                    g_keydisk_128mode = 1;


                    fprintf(stderr,
                            "[SCSI KEYDISK] 128-byte special-track mode ENABLED\n");
                }
                else if (bps == 512){
                    g_keydisk_128mode = 0;

                    fprintf(stderr,
                            "[SCSI KEYDISK] 128-byte special-track mode DISABLED\n");
                }
            }
			
			#ifdef _WIN32
            /*
             * WRITE(6)/WRITE(10) data has now been completely
             * transferred into the in-memory image. Write that
             * completed region back to the corresponding Windows
             * host file.
             */
            if (g_pending_write_target != HOST_WRITE_NONE) {
                int write_result = -1;

                if (g_pending_write_target == HOST_WRITE_DISK) {

                    if (g_disk_writeable) {
                        write_result = host_write_image_data(
                            g_disk_fd,
                            g_pending_write_offset,
                            g_disk_image + g_pending_write_offset,
                            g_pending_write_length
                        );
                    }
                    else {
                        fprintf(stderr,
                                "[DISK] Write attempted on "
                                "read-only disk image\n");
                    }
                }
				else if (g_pending_write_target ==
                         HOST_WRITE_UNIX_FLOPPY) {

                    if (g_unix_floppy_writeable) {
                        write_result = host_write_image_data(
                            g_unix_floppy_fd,
                            g_pending_write_offset,
                            g_unix_floppy_image +
                                g_pending_write_offset,
                            g_pending_write_length
                        );
                    }
                    else {
                        fprintf(stderr,
                                "[UNIX-FLOPPY] Write attempted on "
                                "read-only image\n");
                    }
                }
				if (write_result != 0) {
                    fprintf(stderr,
                            "[SCSI] Host image write-back failed: "
                            "target=%d offset=%zu length=%zu\n",
                            (int)g_pending_write_target,
                            g_pending_write_offset,
                            g_pending_write_length);
                }

                /*
                 * This SCSI write command is now complete. Clear the
                 * pending state so another DATA_OUT command, such as
                 * MODE SELECT, cannot accidentally repeat the write.
                 */
                g_pending_write_target = HOST_WRITE_NONE;
                g_pending_write_offset = 0;
                g_pending_write_length = 0;
            }
#endif
			
            g_ncr.phase = PHASE_STATUS;
            ncr_raise_irq();
        }
        break;
    
	
	case PHASE_DATA_IN:
        if (g_ncr.data_len > 0) {
            g_ncr.data_buf++;
            g_ncr.data_len--;
        }
        if (g_ncr.data_len == 0) {
            g_ncr.phase = PHASE_STATUS;
            /* DATA IN exhausted -> phase mismatch -> NCR interrupt. */
            ncr_raise_irq();
        }
        break;
    case PHASE_STATUS:
        /* Status byte consumed -> MESSAGE IN phase, phase-mismatch IRQ. */
        g_ncr.phase = PHASE_MSG_IN;
        g_ncr.msg_byte = 0x00;          /* COMMAND COMPLETE */
        ncr_raise_irq();
        break;
    case PHASE_MSG_IN:
        /* Message consumed -> target disconnects (BUS FREE).  Signal
         * EOP (bit 7) + IRQ Request (bit 4).  Also set Busy Error
         * (bit 2) since BSY went inactive while Monitor Busy (Mode
         * reg bit 2) is enabled -- the host checks this at $00100E52
         * to recognise "command complete" and clear its in-flight
         * counter at $00106950. */
        g_ncr.phase = PHASE_BUS_FREE;
        g_ncr.bsy_target = 0;
        g_ncr.req_target = 0;
        g_ncr.bas |= 0x80;                          /* EOP */
        if (g_ncr.mode & 0x04) g_ncr.bas |= 0x04;   /* Busy Error */
        ncr_raise_irq();                            /* + IRQ Request */
        break;
    default: break;
    }
}

/* Polled from the read path: drive the late SELECT→target-BSY transition
 * so that a few reads of reg 4 after SEL goes high will see BSY rise. */
static void ncr_tick_select(void) {
    if (g_ncr.phase == PHASE_SELECT && g_ncr.phase_delay > 0) {
        if (--g_ncr.phase_delay == 0) g_ncr.bsy_target = 1;
    }
}
uint8_t g_host_dram[HOST_DRAM_SIZE];
uint8_t g_vram[0x10000];           /* 64KB shared VRAM */
int g_host_p1_released = 0;        /* set when SP clears P1 bit 3 (PRESET) */
int g_sp_reboot_pending = 0;       /* set when SP executes cmd $0B at $C9F5 */
int g_host_halted = 0;             /* host CPU stopped during SP reboot POST */
int g_test_reboot = 0;             /* --test-reboot: inject cmd $0B at boot */

extern uint8_t g_host_dram[];
extern void dmac_step_all(void);
static void dump_mmu_state(void);
uint64_t g_insn = 0;
/* When set, mmu_translate() is a pass-through -- lets debug code aim the
 * disassembler at raw physical DRAM (e.g. loaded user images). */
int g_mmu_disasm_bypass = 0;
/* When set, host_raise_berr() is a no-op.  Debug instrumentation in the
 * instruction hook reads arbitrary guest addresses (ring-buffer PCs,
 * fault PCs, disassembly windows); without this guard an unmapped debug
 * read would inject a REAL bus error into the CPU and derail the guest. */
int g_dbg_no_berr = 0;
/* Set to 1 once the kernel emits "Name:" via putchar -- the login prompt --
 * which is the cue for headless input injection (--type / --mouse) to begin. */
int g_login_ready = 0;
void m68k_instruction_hook(unsigned int pc) {
    g_insn++;
    /* Drive cycle-stealing DMA: one or more bytes per host instruction. */
    dmac_step_all();
    /* Everything below is debug instrumentation -- guard it so a stray
     * read of an unmapped guest address can't inject a real bus error. */
    g_dbg_no_berr = 1;
    /* Tracing of kernel /dev/wb d_read execution: when the host CPU is
     * within the cdevsw[0].d_read function ($10CBEC-$10CDFF) and the
     * function is fetching from VRAM or kernel data (any address below
     * the kernel text), log the access.  This reveals the wb input queue.
     * Capped to keep output volume sane. */
    /* Trace SYSCALL DISPATCH at $10FA48 -- log D7 (syscall number) and the
     * file-descriptor argument from the u-area at $FEF000.  This will
     * reveal which syscall is being made and lets us find /dev/wb opens. */
    /* When d_read at $10CBEC is called with dev = wb ($B00 = major 0 minor 11),
     * arm a detailed trace.  Once armed, log every memory address the
     * function touches until it returns. */
    static int wb_armed = 0;
    static int wb_traced = 0;
    if (wb_armed && wb_traced < 200) {
        uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5);
        uint32_t a0 = m68k_get_reg(NULL, M68K_REG_A0);
        uint32_t d0 = m68k_get_reg(NULL, M68K_REG_D0);
        (void)0;
        wb_traced++;
        /* Disarm when leaving the d_read function (RTS at end of $10CBEC range). */
        if (pc < 0x10CBEC || pc >= 0x10CE00) {
            wb_armed = 0;
            (void)0;
        }
    }
    /* One-shot: dump host DRAM to a file once we're well past kernel init,
     * for offline disassembly with m68k-linux-gnu-objdump.  Triggered by
     * TRIPLEX_DRAMDUMP=path -- write the 2MB DRAM image to that path. */
    if (g_insn == 80000000) {
        const char *dramdump = getenv("TRIPLEX_DRAMDUMP");
        if (dramdump) {
            FILE *df = fopen(dramdump, "wb");
            if (df) {
                fwrite(g_host_dram, 1, HOST_DRAM_SIZE, df);
                fclose(df);
                (void)0;
            } else {
                (void)0;
            }
        }
    }
    /* One-shot: scan DRAM for kernel device-switch tables (cdevsw / bdevsw).
     * Each entry is a struct of N function pointers (8 in classic SysV, but
     * Torch might differ).  We look for contiguous 32-byte chunks where all
     * 4-byte words are kernel-text addresses ($100000-$200000), repeating
     * for multiple entries.  This identifies cdevsw[] candidates. */
    /* Dump the IRQ3 vector and its handler bytes once login is ready, so we
     * know the actual PROCINT ISR address used by the kernel. */
    if (0 && (pc == 0x00100586 || (g_insn > 200000000 && pc < 0x300000))) {
        static int v_dumped = 0;
        if (!v_dumped && (g_login_ready || g_insn > 200000000)) {
            v_dumped = 1;
            extern int g_m68k_current_fc;
            int sfc = g_m68k_current_fc; g_m68k_current_fc = 6;
            uint32_t vbr = m68k_get_reg(NULL, M68K_REG_VBR);
            /* IRQ3 vector with PROCINT NIV=$40 lives at byte offset $100. */
            uint32_t isr_addr = m68k_read_memory_32(vbr + 0x100);
            /* The kernel may also leave auto-vector level-3 ($0C) populated. */
            uint32_t auto3    = m68k_read_memory_32(vbr + 0x0C);
            (void)0;
            /* Disassemble the ISR's first 40 instructions, plus the common
             * dispatcher at $10066A, plus the actual PROCINT handler at the
             * id pushed by the ISR stub ($111AA6 in current build). */
            uint32_t a5_ptr = m68k_read_memory_32(0x12F614);
            (void)0;
            /* Raw bytes near $100528 in BOTH supervisor-program (FC=5) and
             * supervisor-data (FC=6) -- kernel code is mapped via FC=5 so the
             * FC=6 view sees different physical pages. */
            g_m68k_current_fc = 5;
            (void)0;
            for (uint32_t a = 0x100520; a < 0x100548; a += 1)
                (void)0;
            (void)0;
            (void)0;
            for (uint32_t a = 0x100616; a < 0x100640; a += 1)
                (void)0;
            (void)0;
            (void)0;
            uint32_t aa = 0x100520;
            for (int i = 0; i < 60; i++) {
                char buf[128];
                unsigned int len = m68k_disassemble(buf, aa, M68K_CPU_TYPE_68010);
                (void)0;
                aa += len ? len : 2;
            }
            /* Read the IRQ4 (level 4) and IRQ6 (level 6) auto-vector handlers. */
            uint32_t irq4 = m68k_read_memory_32(vbr + 0x10);
            uint32_t irq6 = m68k_read_memory_32(vbr + 0x18);
            (void)0;
            /* Scan host DRAM for distinctive bytes of the kbd-handler lookup
             * sequence -- "andi.l #$F0F, D0" encodes as 02 80 00 00 0F 0F. */
            (void)0;
            extern uint8_t g_host_dram[];
            int hits = 0;
            for (uint32_t o = 0; o + 6 <= HOST_DRAM_SIZE && hits < 40; o += 2) {
                if (g_host_dram[o] == 0x02 && g_host_dram[o+1] == 0x80 &&
                    g_host_dram[o+2] == 0x00 && g_host_dram[o+3] == 0x00 &&
                    g_host_dram[o+4] == 0x0F && g_host_dram[o+5] == 0x0F) {
                    (void)0;
                    hits++;
                }
            }
            /* Also scan for the kernel kbd byte at $15FF -- this is at offset
             * $5FF + HOST_DRAM_BASE if shared VRAM is copied to DRAM, or it
             * may exist as a literal in lookup code. */
            (void)0;
            for (uint32_t o = 0; o + 4 <= HOST_DRAM_SIZE && hits < 80; o += 2) {
                if (g_host_dram[o] == 0x00 && g_host_dram[o+1] == 0x00 &&
                    g_host_dram[o+2] == 0x15 && g_host_dram[o+3] == 0xFF) {
                    (void)0;
                    hits++;
                }
            }
            /* And for "lea $1F0..., An" pattern: 4Fxx + $01F0 (or $000001F0). */
            (void)0;
            int p1f0 = 0;
            for (uint32_t o = 0; o + 4 <= HOST_DRAM_SIZE && p1f0 < 40; o += 2) {
                if (g_host_dram[o] == 0x01 && g_host_dram[o+1] == 0xF0) {
                    (void)0;
                    p1f0++;
                }
            }
            /* Scan for the matrix->ASCII table base $1022FC (00 10 22 FC) as
             * an immediate -- whoever does the lookup must reference it. */
            (void)0;
            int p22fc = 0;
            for (uint32_t o = 0; o + 4 <= HOST_DRAM_SIZE && p22fc < 30; o += 2) {
                if (g_host_dram[o] == 0x00 && g_host_dram[o+1] == 0x10 &&
                    g_host_dram[o+2] == 0x22 && g_host_dram[o+3] == 0xFC) {
                    (void)0;
                    p22fc++;
                }
            }
            /* The real kernel kbd handler is at $120EB6 -- disassemble it. */
            (void)0;
            uint32_t kb = 0x120E60;
            for (int i = 0; i < 60; i++) {
                char buf[128];
                unsigned int len = m68k_disassemble(buf, kb, M68K_CPU_TYPE_68010);
                (void)0;
                kb += len ? len : 2;
            }
            /* The kernel /dev/wb path appears to read $1F2/$1F3 at PC=$1289DC
             * -- disassemble that area so we can route kbd events to it. */
            (void)0;
            uint32_t wb = 0x1289A0;
            for (int i = 0; i < 60; i++) {
                char buf[128];
                unsigned int len = m68k_disassemble(buf, wb, M68K_CPU_TYPE_68010);
                (void)0;
                wb += len ? len : 2;
            }
            /* The kernel reads $2FA at PC=$12BED0 -- disassemble that area. */
            (void)0;
            uint32_t mp = 0x12BEB0;
            for (int i = 0; i < 200; i++) {
                char buf[128];
                unsigned int len = m68k_disassemble(buf, mp, M68K_CPU_TYPE_68010);
                (void)0;
                mp += len ? len : 2;
            }
            uint32_t kbuf_count_ptr = m68k_read_memory_32(0x132144);
            uint32_t kbuf_head_ptr  = m68k_read_memory_32(0x132148);
            uint32_t kbuf_data_base = m68k_read_memory_32(0x132174);
            (void)0;
            struct { uint32_t a; int n; const char *label; } w[] = {
                { isr_addr, 4, "ISR stub" },
                { 0x00111AA6, 80, "real PROCINT handler ($111AA6)" },
                { irq4, 80, "IRQ4 vector handler" },
                { irq6, 60, "IRQ6 vector handler" },
            };
            for (unsigned k = 0; k < sizeof(w)/sizeof(w[0]); k++) {
                (void)0;
                uint32_t a = w[k].a;
                for (int i = 0; i < w[k].n; i++) {
                    char buf[128];
                    unsigned int len = m68k_disassemble(buf, a, M68K_CPU_TYPE_68010);
                    (void)0;
                    a += len ? len : 2;
                }
            }
            g_m68k_current_fc = sfc;
        }
    }
    /* Per-event keyboard handler trace: every time the kbd handler reads a
     * matrix code (at $100546), log D6.  At $1005AE (where ASCII is stored
     * in $102336), log the ASCII value.  At $1005F8 (getchar polls), log
     * the value polled.  Cap counters start over once login-ready so the
     * trace covers post-login keystrokes (the interesting case). */
    static int kh546_n = 0, kh5ae_n = 0, kh5f8_n = 0, kh616_n = 0, kh5f2_n = 0;
    static int kh_phase = 0;
    if (0 && pc == 0x00100616 && kh616_n++ < 60)
        (void)0;
    /* Capture every byte the kernel feeds to putchar ($117112) so its
     * console output -- boot banner, prompts, panics -- is readable.  The
     * char is the longword argument just above the return address.
     * Also detect "Name:" appearing in the output -- once seen, set the
     * g_login_ready flag so headless input injection can start. */
    extern int g_login_ready;
    /* Optional coarse PC sampler (TRIPLEX_PCSAMPLE=1): every 8M instructions
     * print where the CPU is, so a stalled boot is visible without a debugger. */
    g_dbg_no_berr = 0;
}
void m68030_cacr_write(unsigned int v, unsigned int c) { (void)v; (void)c; }
void board_request_stop(void) {}
void trigger_bus_error(unsigned int a, int r, int s) { (void)a; (void)r; (void)s; }
/* Pending interrupt sources, one bit per IRQ level (1..7).  Each source
 * (VSYNC, NCR, DMAC, ...) sets its bit via host_irq_assert; when the CPU
 * acknowledges an interrupt we clear that level's bit and re-assert the
 * highest still pending. */
static uint8_t g_irq_pending = 0;

static void host_irq_refresh(void) {
    int top = 0;
    for (int l = 7; l >= 1; l--) if (g_irq_pending & (1u << l)) { top = l; break; }
    m68k_set_irq(top);
}
void host_irq_assert(int level) {
    g_irq_pending |= (1u << level);
    /* Drive the CPU's IPL inputs from the *highest* pending source, not
     * just this one -- otherwise asserting a lower level while a higher
     * one is still pending would mask the higher request. */
    host_irq_refresh();
}
static void host_irq_clear(int level) {
    g_irq_pending &= ~(1u << level);
    host_irq_refresh();
}

extern int g_dmac_irq_vec;       /* HD63450-supplied vector for IRQ3, or -1 */
int  vme2_int_ack_callback(int level) {
    host_irq_clear(level);
    /* The HD63450 supplies its own vector (NIV) on the IRQ3 acknowledge. */
    int vec = M68K_INT_ACK_AUTOVECTOR;
    if (level == 3 && g_dmac_irq_vec >= 0) {
        vec = g_dmac_irq_vec;
        g_dmac_irq_vec = -1;
    }
    if (0)
        (void)0;
    return vec;
}

static uint8_t dmac_read(uint32_t a);
static void    dmac_write(uint32_t a, uint8_t val);

/* Get a backing pointer for a host address, or NULL if unmapped. */
static uint8_t *host_ptr(uint32_t addr) {
    if (addr < 0x10000)                       return &g_vram[addr];
    if (addr >= 0x40000 && addr < 0x40008)    return &g_host_io_scc  [addr - 0x40000];
    /* DMAC is at $80000-$8003F but handled via dmac_read/dmac_write,
     * not a simple backing array. */
    if (addr >= 0xC0000 && addr < 0xC0010)    return &g_host_io_lance[addr - 0xC0000];
    /* NCR5380 is handled separately via ncr_read/ncr_write */
    if (addr >= HOST_DRAM_BASE && addr < HOST_DRAM_BASE + HOST_DRAM_SIZE)
        return &g_host_dram[addr - HOST_DRAM_BASE];
    return NULL;
}

/* AM7990 LANCE Ethernet controller at host $C0000 - $C0003.
 *
 * Two 16-bit memory-mapped registers:
 *   $C0000 RDP -- read/write currently-selected CSR
 *   $C0002 RAP -- selects which CSR (0..3) RDP targets
 *
 * Four CSRs:
 *   CSR0  status / control bits (INIT, STRT, STOP, TDMD, IDON, TINT, ...)
 *   CSR1  init-block address bits 0-15
 *   CSR2  init-block address bits 16-23 (low byte)
 *   CSR3  byte-order / bus control (BCON, ACON, BSWP)
 *
 * Init block (24 bytes, big-endian on m68k with BSWP/ACON):
 *   +0  MODE
 *   +2  6 bytes physical-address-register (MAC)
 *   +8  8 bytes logical-address-filter
 *   +16 RDRA (RX desc-ring base) 24-bit
 *   +19 RLEN (high nibble = log2(rx ring size))
 *   +20 TDRA (TX desc-ring base) 24-bit
 *   +23 TLEN (high nibble = log2(tx ring size))
 *
 * Each descriptor is 8 bytes:
 *   +0  LADR        low 16 bits of buffer address
 *   +2  HADR+FLAGS  high 8 bits + status flags (OWN, STP, ENP, ERR, ...)
 *   +4  BCNT        two's-complement buffer length (negative)
 *   +6  MCNT        message length (RX); 0 (TX)
 *
 * Bus flow on transmit: kernel fills a TX descriptor with OWN=1, STP=1,
 * ENP=1, writes TDMD to CSR0.  We walk the TX ring, DMA each owned
 * buffer to the TAP fd, clear OWN, set TINT.  Bus flow on receive: we
 * read from TAP, find an OWN'd RX descriptor, DMA the packet into its
 * buffer, set ENP/STP, write the byte count to MCNT, clear OWN, set
 * RINT, raise host IRQ 2. */
static uint16_t g_lance_csr[4] = { 0x0004, 0, 0, 0 };  /* CSR0 = STOP */
static int      g_lance_rap = 0;
static uint32_t g_lance_init_addr = 0;
static uint8_t  g_lance_mac[6] = {0x00, 0x80, 0x10, 0xAB, 0xCD, 0xEF};
static uint32_t g_lance_rdra = 0;
static int      g_lance_rlen = 0;   /* RX ring size (entries) */
static int      g_lance_rx_idx = 0;
static uint32_t g_lance_tdra = 0;
static int      g_lance_tlen = 0;
static int      g_lance_tx_idx = 0;
static int      g_lance_initialized = 0;
static int      g_tap_fd = -1;
static const char *g_tap_name = NULL;

extern int g_lance_trace;  /* defined below; forward decl for early callers */

/* Read/write 1 byte of host-physical memory (the LANCE bus master). */
static uint8_t lance_dma_r8(uint32_t addr) {
    if (addr < 0x10000)                          return g_vram[addr];
    if (addr >= HOST_DRAM_BASE && addr < HOST_DRAM_BASE + HOST_DRAM_SIZE)
        return g_host_dram[addr - HOST_DRAM_BASE];
    return 0xFF;
}
static void lance_dma_w8(uint32_t addr, uint8_t v) {
    if (addr < 0x10000) { g_vram[addr] = v; return; }
    if (addr >= HOST_DRAM_BASE && addr < HOST_DRAM_BASE + HOST_DRAM_SIZE)
        g_host_dram[addr - HOST_DRAM_BASE] = v;
}
static uint16_t lance_dma_r16(uint32_t addr) {
    return ((uint16_t)lance_dma_r8(addr) << 8) | lance_dma_r8(addr + 1);
}
static void lance_dma_w16(uint32_t addr, uint16_t v) {
    lance_dma_w8(addr,     v >> 8);
    lance_dma_w8(addr + 1, v & 0xFF);
}

/* Open a TAP interface for LANCE packets.  Returns fd or -1 on failure.
 * The caller must have CAP_NET_ADMIN (or run as root), and the interface
 * must be brought up + assigned an address from the host side, e.g.:
 *   sudo ip tuntap add dev tap0 mode tap user $USER
 *   sudo ip addr add 10.0.2.1/24 dev tap0
 *   sudo ip link set tap0 up
 * Then run with --tap tap0 -- the emulator and host can ping each other. */


#ifndef _WIN32

static int tap_open(const char *name) {
    int fd = open("/dev/net/tun", O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[TAP] open /dev/net/tun failed: %s\n", strerror(errno));
        return -1;
    }
    struct ifreq ifr = {0};
    ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
    strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
    if (ioctl(fd, TUNSETIFF, &ifr) < 0) {
        fprintf(stderr, "[TAP] TUNSETIFF %s failed: %s "
                "(need CAP_NET_ADMIN; create the interface with "
                "`sudo ip tuntap add dev %s mode tap user $USER` first)\n",
                name, strerror(errno), name);
        close(fd);
        return -1;
    }
    fprintf(stderr, "[TAP] opened %s (fd=%d)\n", ifr.ifr_name, fd);
    return fd;
}

#else

static int tap_open(const char *name) {
    (void)name;

    fprintf(stderr,
            "Networking is disabled in the Windows build.\n");

    return -1;
}

#endif


/* Drive the LANCE IRQ line from the current CSR0 state.  The line should
 * be asserted only when (a) at least one interrupt CAUSE bit is set
 * (the INTR summary bit 7 reflects this) AND (b) INEA bit 6 is set.pign
 * Otherwise it must be deasserted -- without that, the kernel's IRQ
 * handler runs, reads CSR0, finds no cause bits and logs "Spurious
 * Interrupt from Lance" then resets the chip in a loop. */
static void lance_refresh_irq(void) {
    extern void host_irq_assert(int level);
    extern void host_irq_clear(int level);
    if ((g_lance_csr[0] & 0x0080) && (g_lance_csr[0] & 0x0040))
        host_irq_assert(2);
    else
        host_irq_clear(2);
}

/* Parse the init block at g_lance_init_addr into our state. */
static void lance_parse_init_block(void) {
    if (g_lance_init_addr < HOST_DRAM_BASE ||
        g_lance_init_addr + 24 > HOST_DRAM_BASE + HOST_DRAM_SIZE)
        return;
    /* MAC address at +2..+7 */
    for (int i = 0; i < 6; i++)
        g_lance_mac[i] = lance_dma_r8(g_lance_init_addr + 2 + i);
    /* RDRA at offset $10 (=16), TDRA at offset $14 (=20).  AM7990 init-block
     * layout in chip's view (little-endian internally):
     *   word @+10: RDRA[15:0]                       (low 16 bits of RX ring addr)
     *   word @+12: RLEN[15:13] | reserved | RDRA[23:16]  (high 8 bits + ring-size code)
     *   word @+14: TDRA[15:0]
     *   word @+16: TLEN[15:13] | reserved | TDRA[23:16]
     *
     * With CSR3 BSWP=1 + the 68K big-endian host, kernel's MOVE.W writes
     * numerically match the chip's view: chip word = (mem[hi] << 8) |
     * mem[lo].  So for memory bytes b16..b19 at offsets 16..19:
     *   chip RDRA[15:0]      = (b16 << 8) | b17
     *   chip RDRA[23:16]     = b19          (low byte of word @+12)
     *   chip RLEN_field      = b18 >> 5     (high byte of word @+12, bits 5-7)
     * The pre-BSWP code mis-decoded these as little-endian byte streams,
     * yielding addresses like 0x40A88C that point outside DRAM and made
     * the kernel TX path silently fail / hang on ifconfig la0 up. */
    uint8_t b16 = lance_dma_r8(g_lance_init_addr + 16);
    uint8_t b17 = lance_dma_r8(g_lance_init_addr + 17);
    uint8_t b18 = lance_dma_r8(g_lance_init_addr + 18);
    uint8_t b19 = lance_dma_r8(g_lance_init_addr + 19);
    g_lance_rdra = ((uint32_t)b19 << 16) | ((uint32_t)b16 << 8) | b17;
    g_lance_rlen = 1 << ((b18 >> 5) & 7);
    uint8_t b20 = lance_dma_r8(g_lance_init_addr + 20);
    uint8_t b21 = lance_dma_r8(g_lance_init_addr + 21);
    uint8_t b22 = lance_dma_r8(g_lance_init_addr + 22);
    uint8_t b23 = lance_dma_r8(g_lance_init_addr + 23);
    g_lance_tdra = ((uint32_t)b23 << 16) | ((uint32_t)b20 << 8) | b21;
    g_lance_tlen = 1 << ((b22 >> 5) & 7);
    g_lance_rx_idx = 0;
    g_lance_tx_idx = 0;
    g_lance_initialized = 1;
    static uint32_t last_logged = 0;
    if (g_lance_init_addr != last_logged) {
        last_logged = g_lance_init_addr;
        fprintf(stderr, "[LANCE] init block @%06X: MAC=%02X:%02X:%02X:%02X:%02X:%02X "
                "RX ring @%06X size %d, TX ring @%06X size %d\n",
                g_lance_init_addr,
                g_lance_mac[0], g_lance_mac[1], g_lance_mac[2],
                g_lance_mac[3], g_lance_mac[4], g_lance_mac[5],
                g_lance_rdra, g_lance_rlen, g_lance_tdra, g_lance_tlen);
        /* Dump raw bytes of the entire init block (only when tracing) so
         * we can verify our RDRA/TDRA byte-order decode against what the
         * kernel actually wrote. */
        if (g_lance_trace) {
            fprintf(stderr, "[LANCE-IB]");
            for (int i = 0; i < 24; i++)
                (void)0;
            (void)0;
        }
    }
}

/* Walk the TX descriptor ring; for each chip-owned descriptor, DMA the
 * buffer to the TAP fd and clear OWN.  Returns the number of descriptors
 * actually drained (so the caller can decide whether to set TINT). */
static int lance_tx_drain(void) {
    if (!g_lance_initialized || g_tap_fd < 0) return 0;
    int drained = 0;
    /* Log the TX ring state on first call after a TDMD so we can see
     * what the kernel is queuing (only ring-head once per drain so the
     * log doesn't spam during steady-state). */
    if (g_lance_trace) {
        uint32_t desc0 = g_lance_tdra + g_lance_tx_idx * 8;
        uint16_t f0 = lance_dma_r16(desc0 + 2);
        uint16_t b0 = lance_dma_r16(desc0 + 4);
        fprintf(stderr, "[LANCE-TX] enter drain tx_idx=%d desc=%06X "
                "flags=%04X bcnt=%04X (OWN=%d)\n",
                g_lance_tx_idx, desc0, f0, b0, (f0 >> 15) & 1);
    }
    while (1) {
        uint32_t desc = g_lance_tdra + g_lance_tx_idx * 8;
        uint16_t hadr_flags = lance_dma_r16(desc + 2);
        if (!(hadr_flags & 0x8000)) break;          /* OWN not set: nothing to do */
        uint32_t bufaddr = ((uint32_t)(hadr_flags & 0xFF) << 16) |
                            lance_dma_r16(desc + 0);
        int16_t bcnt_signed = (int16_t)lance_dma_r16(desc + 4);
        int len = -bcnt_signed;                     /* BCNT is 2's-complement */
        if (len <= 0 || len > 1518) { len = 0; }
        uint8_t pkt[1518];
        for (int i = 0; i < len; i++) pkt[i] = lance_dma_r8(bufaddr + i);
        ssize_t w = write(g_tap_fd, pkt, len);
        (void)w;
        if (g_lance_trace) {
            fprintf(stderr, "[LANCE-TX] sent %d bytes from %06X (desc %06X "
                    "flags=%04X) to TAP, w=%zd\n", len, bufaddr, desc,
                    hadr_flags, w);
            /* Dump first 42 bytes (Ethernet header + ARP / IP+ICMP head). */
            if (len >= 14) {
                fprintf(stderr, "[LANCE-TX]  ");
                for (int i = 0; i < (len < 42 ? len : 42); i++)
                    (void)0;
                (void)0;
            }
        }
        /* Return descriptor: clear OWN, ERR, etc; keep STP/ENP. */
        lance_dma_w16(desc + 2, hadr_flags & 0x03FF);
        g_lance_csr[0] |= 0x0200;                   /* TINT */
        g_lance_tx_idx = (g_lance_tx_idx + 1) % (g_lance_tlen ? g_lance_tlen : 1);
        drained++;
    }
    return drained;
}

/* Read pending packets from the TAP fd, place them in OWN'd RX descs. */
static void lance_rx_poll(void) {
    if (!g_lance_initialized || g_tap_fd < 0) return;
    if (!(g_lance_csr[0] & 0x0020)) return;         /* RXON cleared */
    while (1) {
        uint8_t pkt[1518];
        ssize_t n = read(g_tap_fd, pkt, sizeof(pkt));
        if (n <= 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && n < 0)
                fprintf(stderr, "[TAP] read err: %s\n", strerror(errno));
            return;
        }
        uint32_t desc = g_lance_rdra + g_lance_rx_idx * 8;
        uint16_t hadr_flags = lance_dma_r16(desc + 2);
        if (!(hadr_flags & 0x8000)) {
            /* No OWN'd buffer: drop and set MISS. */
            g_lance_csr[0] |= 0x1000;               /* MISS */
            continue;
        }
        uint32_t bufaddr = ((uint32_t)(hadr_flags & 0xFF) << 16) |
                            lance_dma_r16(desc + 0);
        int16_t bcnt_signed = (int16_t)lance_dma_r16(desc + 4);
        int maxbuf = -bcnt_signed;
        if (n > maxbuf) n = maxbuf;
        /* Pad short frames to the IEEE 802.3 minimum (60 bytes payload).
         * TAP delivers raw L2 frames with no zero-padding, but a real LANCE
         * receives ≥64 bytes off the wire (60 + 4-byte FCS) -- the host
         * NIC and Ethernet PHY pad on the sender side.  Without this pad
         * the kernel's LANCE driver sees a runt ARP-reply (42 bytes) and
         * drops it, so the kernel never learns 10.0.2.1's MAC and keeps
         * re-ARPing every 400ms forever. */
        int pad_to_min = 60;
        if (n < pad_to_min) {
            for (int i = n; i < pad_to_min; i++) pkt[i] = 0;
            n = pad_to_min;
        }
        for (int i = 0; i < n; i++) lance_dma_w8(bufaddr + i, pkt[i]);
        /* Fake 4-byte FCS trailer.  With MODE.DRX=0 (the kernel-set default)
         * MCNT counts CRC bytes, so the kernel computes "real payload" as
         * MCNT - 4.  We write zeros for the CRC; the kernel doesn't check
         * it as long as the CRC bit in RMD1 is clear. */
        for (int i = 0; i < 4; i++) lance_dma_w8(bufaddr + n + i, 0);
        int mcnt = n + 4;
        /* Set STP | ENP, clear OWN. */
        lance_dma_w16(desc + 2, (hadr_flags & 0x00FF) | 0x0300);
        lance_dma_w16(desc + 6, (uint16_t)mcnt);    /* MCNT = bytes + FCS */
        g_lance_csr[0] |= 0x0400;                   /* RINT */
        g_lance_csr[0] |= 0x0080;                   /* INTR summary */
        if (g_lance_trace) {
            fprintf(stderr, "[LANCE-RX] %d bytes (mcnt=%d) -> desc %06X "
                    "buf %06X (rx_idx=%d)\n", (int)n, mcnt, desc, bufaddr,
                    g_lance_rx_idx);
            if (n >= 14) {
                fprintf(stderr, "[LANCE-RX]  ");
                for (int i = 0; i < (n < 42 ? (int)n : 42); i++)
                    (void)0;
                (void)0;
            }
        }
        g_lance_rx_idx = (g_lance_rx_idx + 1) % (g_lance_rlen ? g_lance_rlen : 1);
    }
}

int g_lance_trace = 0;   /* --lance-trace: always-on register-level logging */
static uint16_t lance_read16(uint32_t addr) {
    uint16_t v;
    if ((addr & 2) == 0) {
        /* RDP: read currently-selected CSR. */
        v = g_lance_csr[g_lance_rap & 3];
    } else {
        /* RAP: read which CSR is selected. */
        v = (uint16_t)g_lance_rap;
    }
    {
        extern int g_trace_pc; extern uint64_t g_insn;
        if (g_lance_trace || 0) {
            static int lc = 0; if (lc < 4000) { lc++;
            fprintf(stderr, "[LANCE @%llu] R %s -> %04X (rap=%d csr0=%04x) "
                    "PC=%08X\n",
                    (unsigned long long)g_insn,
                    (addr & 2) ? "RAP" : "RDP", v,
                    g_lance_rap, g_lance_csr[0],
                    m68k_get_reg(NULL, M68K_REG_PC));
            }
        }
    }
    return v;
}

static void lance_write16(uint32_t addr, uint16_t v) {
    {
        extern int g_trace_pc; extern uint64_t g_insn;
        if (g_lance_trace || 0) {
            static int lc = 0; if (lc < 4000) { lc++;
            fprintf(stderr, "[LANCE @%llu] W %s <- %04X (rap=%d csr0=%04x) "
                    "PC=%08X\n",
                    (unsigned long long)g_insn,
                    (addr & 2) ? "RAP" : "RDP", v,
                    g_lance_rap, g_lance_csr[0],
                    m68k_get_reg(NULL, M68K_REG_PC));
            }
        }
    }
    if ((addr & 2) == 0) {
        /* RDP: write to currently-selected CSR. */
        int csr = g_lance_rap & 3;
        if (csr == 0) {
            /* CSR0 has tricky write-1-to-clear semantics on the upper
             * status bits (TINT, RINT, IDON, etc.), plus control bits in
             * the low byte (INIT, STRT, STOP, TDMD, INEA). */
            uint16_t cur = g_lance_csr[0];
            uint16_t newval = cur;
            /* Control bits: take value from write. */
            if (v & 0x0001) { newval |= 0x0001; }       /* INIT  */
            if (v & 0x0002) {                           /* STRT  */
                newval |= 0x0002;
                /* Real AM7990: STRT after a successful INIT clears STOP
                 * and re-enables the transmitter/receiver.  STOP and
                 * STRT are mutually exclusive; writing STRT moves the
                 * chip from STOP state back to running state.  Without
                 * this, the host kernel reads CSR0 after writing STRT,
                 * finds TXON=0, and logs "Lance has lost TXON" --
                 * triggering an infinite re-INIT loop. */
                newval &= ~0x0004;                      /* clear STOP */
                if (g_lance_initialized)
                    newval |= 0x0010 | 0x0020;          /* TXON | RXON */
            }
            if (v & 0x0004) {                           /* STOP  */
                newval |= 0x0004;
                /* STOP clears running state. */
                newval &= ~(0x0001 | 0x0002 | 0x0008 | 0x0010 | 0x0020);
            }
            if (v & 0x0008) { newval |= 0x0008; }       /* TDMD  */
            newval = (newval & ~0x0040) | (v & 0x0040); /* INEA  */
            /* Status bits: write-1-to-clear. */
            uint16_t w1c = (0x0100 | 0x0200 | 0x0400 | 0x0800 |
                            0x1000 | 0x2000 | 0x4000 | 0x8000);
            newval &= ~(v & w1c);
            /* Side-effects: */
            if (newval & 0x0001) {
                /* INIT requested: read the init block (DMA) to learn the
                 * MAC and ring addresses, then clear STOP, set IDON. */
                g_lance_init_addr = ((uint32_t)g_lance_csr[2] << 16) |
                                     g_lance_csr[1];
                lance_parse_init_block();
                newval &= ~0x0004;
                newval |= 0x0100;                       /* IDON */
                newval |= 0x0010 | 0x0020;              /* TXON|RXON */
                newval &= ~0x0001;                      /* clear INIT */
            }
            if ((newval & 0x0008) && (newval & 0x0010)) {
                /* TDMD with TXON: drain the TX descriptor ring.  Only set
                 * TINT if we actually consumed an OWN'd descriptor -- the
                 * real chip doesn't TINT on a TDMD that walks an empty
                 * ring, and the kernel uses TINT-without-payload as a
                 * symptom of "Lance lost TXON" -style desync. */
                g_lance_csr[0] = newval;
                int drained = lance_tx_drain();
                newval = g_lance_csr[0];
                if (drained > 0) newval |= 0x0200;      /* TINT */
                newval &= ~0x0008;                      /* clear TDMD */
            }
            /* ERR summary: set if any of BABL, CERR, MISS, MERR. */
            if (newval & (0x4000 | 0x2000 | 0x1000 | 0x0800)) newval |= 0x8000;
            else                                              newval &= ~0x8000;
            /* INTR summary: set if any of BABL, MISS, MERR, RINT, TINT, IDON. */
            if (newval & (0x4000 | 0x1000 | 0x0800 | 0x0400 | 0x0200 | 0x0100))
                newval |= 0x0080;
            else
                newval &= ~0x0080;
            g_lance_csr[0] = newval;
            lance_refresh_irq();
        } else {
            g_lance_csr[csr] = v;
        }
    } else {
        /* RAP: select which CSR the next RDP access targets. */
        g_lance_rap = v & 3;
    }
}

/* No I/O register quirks; host reads return whatever was last written
 * to the chip-select region (or zero if untouched). */
static uint8_t host_io_read_quirk(uint32_t addr, uint8_t raw) {
    (void)addr;
    return raw;
}

extern void m68k_pulse_bus_error(void);
extern int g_berr_not_rerunnable;

int g_sup_berr_seen = 0;   /* set on a supervisor bus error (kernel fault) */
static void host_raise_berr(uint32_t addr, int is_write) {
    extern int g_m68k_current_fc;

    /* Debug instrumentation in the instruction hook reads arbitrary guest
     * addresses; an unmapped such read must NOT inject a real bus error. */
    extern int g_dbg_no_berr;
    if (g_dbg_no_berr) return;
    /* Latch the faulting (logical) address so the MC68010 bus-error stack
     * frame carries it -- System V's exception handler reads the frame's
     * fault-address field to route the page fault to a region handler. */
    extern uint32_t g_pmmu_fault_addr;
    g_pmmu_fault_addr = addr;
    /* A supervisor-fc bus error is abnormal.  Flag it (only in the late
     * window around the scheduler panic) so the supervisor-PC ring buffer
     * dumps the path that led here. */
    if (g_m68k_current_fc >= 5 && g_insn > 14000000) g_sup_berr_seen = 1;
    /* User-mode BERRs are almost always demand-paging (stack-grow, COW
     * fork, exec fault-in) which the kernel handles transparently.
     * Only log supervisor-mode BERRs by default -- those indicate real
     * MMU or driver bugs.  Enable --trace-pc to see user-mode ones. */
    if (g_m68k_current_fc >= 5 || 0) {
        fprintf(stderr, "[BERR] %s @ %08X  PC=%08X fc=%d\n",
                is_write ? "W" : "R", addr, m68k_get_reg(NULL, M68K_REG_PC),
                g_m68k_current_fc);
    }
    /* A bus error on a translated address in the bus-error region means an
     * MMU descriptor produced a bad physical address -- dump the MMU so the
     * offending descriptor is visible.  Also dump for a SUPERVISOR fault on
     * a low (kernel/DRAM) address: the kernel's own data is always meant to
     * be mapped, so such a fault means the descriptors/AST are wrong. */
    if (0 && (addr >= 0x00300000 ||
                       (g_m68k_current_fc >= 5 && g_insn > 14000000))) {
        static int bd = 0;
        if (bd < 6) { bd++;
            fprintf(stderr, "[BERR-MMU] berr addr %08X fc=%d  D0-7=%08X %08X "
                    "%08X %08X %08X %08X %08X %08X\n", addr, g_m68k_current_fc,
                    m68k_get_reg(NULL,M68K_REG_D0), m68k_get_reg(NULL,M68K_REG_D1),
                    m68k_get_reg(NULL,M68K_REG_D2), m68k_get_reg(NULL,M68K_REG_D3),
                    m68k_get_reg(NULL,M68K_REG_D4), m68k_get_reg(NULL,M68K_REG_D5),
                    m68k_get_reg(NULL,M68K_REG_D6), m68k_get_reg(NULL,M68K_REG_D7));
            fprintf(stderr, "[BERR-MMU] A0-7=%08X %08X %08X %08X %08X %08X "
                    "%08X %08X  PC=%08X\n",
                    m68k_get_reg(NULL,M68K_REG_A0), m68k_get_reg(NULL,M68K_REG_A1),
                    m68k_get_reg(NULL,M68K_REG_A2), m68k_get_reg(NULL,M68K_REG_A3),
                    m68k_get_reg(NULL,M68K_REG_A4), m68k_get_reg(NULL,M68K_REG_A5),
                    m68k_get_reg(NULL,M68K_REG_A6), m68k_get_reg(NULL,M68K_REG_A7),
                    m68k_get_reg(NULL,M68K_REG_PC));
            fprintf(stderr, "[BERR-CPU] PC=%08X PPC=%08X PREF_ADDR=%08X "
                    "PREF_DATA=%04X IR=%04X SR=%04X\n",
                    m68k_get_reg(NULL,M68K_REG_PC), m68k_get_reg(NULL,M68K_REG_PPC),
                    m68k_get_reg(NULL,M68K_REG_PREF_ADDR),
                    m68k_get_reg(NULL,M68K_REG_PREF_DATA),
                    m68k_get_reg(NULL,M68K_REG_IR),
                    m68k_get_reg(NULL,M68K_REG_SR));
            /* Dump the exception vector table -- a supervisor instruction
             * fetch of a bad address often means an interrupt vectored
             * through a corrupt table entry. */
            {
                uint32_t vbr = m68k_get_reg(NULL, M68K_REG_VBR);
                int sfc2 = g_m68k_current_fc; g_m68k_current_fc = 5;
                for (uint32_t vo = 0; vo < 0x100; vo += 16) {
                    fprintf(stderr, "[BERR-VEC] %02X:", vo >> 2);
                    for (uint32_t k = 0; k < 16; k += 4)
                        (void)0;
                    (void)0;
                }
                g_m68k_current_fc = sfc2;
            }
            dump_mmu_state();
        }
    }
    /* Mark BERR non-rerunnable ONLY for supervisor faults.  Musashi uses
     * the flag to decide whether to roll back the pre-instruction
     * register file before re-running: rolling back is wrong for the
     * boot ROM's DRAM-sizing probe (it relies on the post-incremented
     * A0 sticking after the failing read), but rolling back is exactly
     * what user-mode demand-paging needs.  vi/cc/ed all trigger
     * legitimate stack-grow BERRs at startup; without rollback the
     * kernel's stacked RTE retries the faulting instruction with
     * corrupted register state and the process eventually dies. */

    if (g_m68k_current_fc >= 5){
        g_berr_not_rerunnable = 1;
        
        /*printf("g_berr NOT runnable - FC value is %d\n", g_m68k_current_fc);
        printf("BERR FC=%d addr=%08X A7=%08X USP=%08X SR=%04X PC=%08X\n",
            g_m68k_current_fc,
            addr,
            m68k_get_reg(NULL, M68K_REG_A7),
            m68k_get_reg(NULL, M68K_REG_USP),
            m68k_get_reg(NULL, M68K_REG_SR),
            m68k_get_reg(NULL, M68K_REG_PC)); */

        extern uint16_t g_pmmu_fault_ssw;

        int rw = is_write ? 0x00 : 0x10;

        /*g_pmmu_fault_ssw = 0x0150 | (g_m68k_current_fc & 7);*/

        g_pmmu_fault_ssw = 
            (g_m68k_current_fc & 7) 
            | (((g_m68k_current_fc & 3) == 1) ? (1 << 12) : 0)
            | (((g_m68k_current_fc & 3) == 2) ? (1 << 13) : 0)
            | (rw << 8);

        /* printf("BERR SSW=%04X FC=%d WRITE=%d\n",
            g_pmmu_fault_ssw,
            g_m68k_current_fc,
            is_write); */
        /* fprintf(stderr, "IR=%04X PREF=%04X PREF_ADDR=%08X\n",
            m68k_get_reg(NULL, M68K_REG_IR),
            m68k_get_reg(NULL, M68K_REG_PREF_DATA),
            m68k_get_reg(NULL, M68K_REG_PREF_ADDR)); */

        /*fprintf(stderr, "INST WORDS: %04X %04X %04X\n",
            m68k_read_memory_16(REG_PPC),
            m68k_read_memory_16(REG_PPC + 2),
            m68k_read_memory_16(REG_PPC + 4)); */

        if (is_write)
            g_pmmu_fault_ssw &=  ~0x0010;

        m68k_pulse_bus_error();
    
    } else {
        g_berr_not_rerunnable = 0;
        
        /*printf("g_berr runnable - FC value is %d\n", g_m68k_current_fc); */
        /*printf("BERR FC=%d addr=%08X A7=%08X USP=%08X SR=%04X PC=%08X\n",
            g_m68k_current_fc,
            addr,
            m68k_get_reg(NULL, M68K_REG_A7),
            m68k_get_reg(NULL, M68K_REG_USP),
            m68k_get_reg(NULL, M68K_REG_SR),
            m68k_get_reg(NULL, M68K_REG_PC)); */

        extern uint16_t g_pmmu_fault_ssw;

        int rw = is_write ? 0x00 : 0x10;

        /*g_pmmu_fault_ssw = 0x0150 | (g_m68k_current_fc & 7);*/

        g_pmmu_fault_ssw = 
            (g_m68k_current_fc & 7) 
            | (((g_m68k_current_fc & 3) == 1) ? (1 << 12) : 0)
            | (((g_m68k_current_fc & 3) == 2) ? (1 << 13) : 0)
            | (rw << 8);

        /* printf("BERR SSW=%04X FC=%d WRITE=%d\n",
            g_pmmu_fault_ssw,
            g_m68k_current_fc,
            is_write); */

        /* fprintf(stderr, "IR=%04X PREF=%04X PREF_ADDR=%08X\n",
            m68k_get_reg(NULL, M68K_REG_IR),
            m68k_get_reg(NULL, M68K_REG_PREF_DATA),
            m68k_get_reg(NULL, M68K_REG_PREF_ADDR)); */

        /*fprintf(stderr, "INST WORDS: %04X %04X %04X\n",
            m68k_read_memory_16(REG_PPC),
            m68k_read_memory_16(REG_PPC + 2),
            m68k_read_memory_16(REG_PPC + 4)); */
			
		

        if (is_write)
            g_pmmu_fault_ssw &= ~0x0010;


        m68k_pulse_bus_error();
    }

    /*g_berr_not_rerunnable = (g_m68k_current_fc >= 5) ? 1 : 0;
    m68k_pulse_bus_error(); */
}

/* Per MAME: $200000-$2FFFFF is the Limpet VMEbus expansion (now emulated
 * as part of HOST_DRAM), $300000-$7FFFFF is the bus-error catch-all.
 * Addresses below $300000 that don't fall in a mapped device just
 * return zero / silently accept writes (chip-select gaps). */
static int host_addr_is_berr(uint32_t addr) { return addr >= 0x300000; }

/* NCR5380 sits at host $E0000-$E000F on the upper byte of the 16-bit
 * bus.  Byte address $E0000+2*N is NCR register N. */
static int is_ncr_addr(uint32_t a, int *reg) {
    if (a >= 0xE0000 && a < 0xE0010 && !(a & 1)) {
        *reg = (a - 0xE0000) >> 1;
        return 1;
    }
    return 0;
}

/* HD63450 DMAC at $00080000-$0008003F (per MAME mame/machine/hd63450.cpp
 * and mame/torch/triplex.cpp).  Triple X exposes ONE channel (channel 0)
 * in the 64-byte window; channel 0 is wired to the NCR5380 SCSI controller
 * via DRQ.  Per-channel registers follow MAME's hd63450_regs layout, with
 * byte offsets within the channel block. */
typedef struct {
    uint8_t  csr, cer;
    uint8_t  dcr, ocr, scr, ccr;
    uint16_t mtc;
    uint32_t mar;
    uint32_t dar;
    uint16_t btc;
    uint32_t bar;
    uint8_t  niv, eiv;
    uint8_t  mfc, cpr, dfc, bfc, gcr;
} hd63450_chan_t;

static hd63450_chan_t g_dmac_chan[4];

/* The HD63450-supplied interrupt vector for a pending IRQ3 (-1 = none).
 * The host's interrupt acknowledge reads this to vector IRQ3 to the
 * channel's normal interrupt vector (NIV) instead of autovectoring. */
int g_dmac_irq_vec = -1;

static void dmac_reset(void) {
    for (int i = 0; i < 4; i++) {
        memset(&g_dmac_chan[i], 0, sizeof(g_dmac_chan[i]));
        g_dmac_chan[i].niv = 0x0F;
        g_dmac_chan[i].eiv = 0x0F;
    }
    g_dmac_irq_vec = -1;
}

/* HD63450 PCL0 input -- driven by the SP's P1 bit 4 (PROCINT).  When
 * channel 0's DCR programs PCL as "status input with interrupt" (DCR
 * low bits = 001), a 1->0 edge latches CSR bit 1 (PCT) and raises the
 * channel interrupt.  CSR bit 0 (PCS) always tracks the live PCL level. */
void host_irq_assert(int level);
void dmac_pcl0_transition(int level) {
    hd63450_chan_t *c = &g_dmac_chan[0];
    if (level) c->csr |= 0x01; else c->csr &= ~0x01;   /* PCS */
    if (!level && (c->dcr & 0x07) == 0x01) {
        c->csr |= 0x02;            /* PCT: PCL transition latched */
        g_dmac_irq_vec = c->niv;   /* channel 0 supplies its NIV on IACK */
        host_irq_assert(3);
        if (0)
            (void)0;
    } else if (0) {}
}

/* Returned and accepted byte values use big-endian semantics for the 16/32
 * bit fields, matching how the 68000-side word/long reads will reassemble
 * them. */
static uint8_t dmac_read(uint32_t a) {
    int channel = (a >> 6) & 3;
    int off = a & 0x3F;
    hd63450_chan_t *c = &g_dmac_chan[channel];
    switch (off) {
    case 0x00: return c->csr;
    case 0x01: return c->cer;
    case 0x04: return c->dcr;
    case 0x05: return c->ocr;
    case 0x06: return c->scr;
    case 0x07: return c->ccr;
    case 0x0A: return c->mtc >> 8;
    case 0x0B: return c->mtc & 0xFF;
    case 0x0C: return (c->mar >> 24) & 0xFF;
    case 0x0D: return (c->mar >> 16) & 0xFF;
    case 0x0E: return (c->mar >>  8) & 0xFF;
    case 0x0F: return  c->mar        & 0xFF;
    case 0x14: return (c->dar >> 24) & 0xFF;
    case 0x15: return (c->dar >> 16) & 0xFF;
    case 0x16: return (c->dar >>  8) & 0xFF;
    case 0x17: return  c->dar        & 0xFF;
    case 0x1A: return c->btc >> 8;
    case 0x1B: return c->btc & 0xFF;
    case 0x1C: return (c->bar >> 24) & 0xFF;
    case 0x1D: return (c->bar >> 16) & 0xFF;
    case 0x1E: return (c->bar >>  8) & 0xFF;
    case 0x1F: return  c->bar        & 0xFF;
    case 0x25: return c->niv;
    case 0x27: return c->eiv;
    case 0x29: return c->mfc;
    case 0x2D: return c->cpr;
    case 0x31: return c->dfc;
    case 0x39: return c->bfc;
    case 0x3F: return c->gcr;
    }
    return 0xFF;
}

extern void m68k_set_irq(unsigned int level);
static uint32_t host_mem_read(uint32_t addr, int sz);
static void     host_mem_write(uint32_t addr, uint32_t v, int sz);

/* Perform a synchronous DMA transfer for a channel.  Triple X uses
 * memory-to-memory transfers with the source in DAR and destination in
 * MAR, or vice versa depending on OCR.DIR (bit 7).  We honor MTC for the
 * count and SCR for address increment/decrement.  Channel 0 (SCSI) is
 * special: DAR is the NCR data register and we route bytes through
 * ncr_read/ncr_write.  This is "all at once" -- no cycle stealing -- which
 * is fine for the print blits the CARETAKER boot code drives. */
static uint8_t ncr_read(int reg);
static void    ncr_write(int reg, uint8_t v);
static int     is_ncr_addr(uint32_t a, int *reg);

/* Per-channel start-up latency, in host instructions.  A real SCSI
 * transfer (selection + per-byte REQ/ACK handshake + the HD63450's
 * cycle-stealing) takes many microseconds; the WERMA kernel arms the
 * NCR5380 + DMAC from inside its IRQ4 handler at IPL 4, then returns,
 * dropping back to IPL 0 where it can actually accept the NCR's
 * completion interrupt.  If the transfer "finished" instantly it would
 * raise IRQ4 while the CPU is still masked at IPL 4 and the interrupt
 * would be lost.  Holding the channel off for a short while models the
 * real transfer time and lets the kernel reach IPL 0 first. */
static int g_dma_delay[4];
#define DMA_START_LATENCY 800

/* Arm a channel; the actual byte transfers happen in dmac_step(), called
 * from the host CPU's instruction hook so transfers progress one byte at
 * a time -- matching real DRQ-driven cycle-stealing. */
static void dmac_start_xfer(int channel) {
    hd63450_chan_t *c = &g_dmac_chan[channel];
    c->csr &= ~0x30;            /* clear ERR/NORM end bits */
    c->csr |=  0x08;            /* channel active */
    g_dma_delay[channel] = DMA_START_LATENCY;
}

/* One DRQ-triggered byte of DMA, if conditions allow.  Returns 1 if a
 * byte was transferred, 0 if the channel idled this cycle. */
static uint32_t g_dma_bytes[4], g_dma_mar0[4];
static int      g_dma_phase0[4];
static int dmac_step_channel(int channel) {
    hd63450_chan_t *c = &g_dmac_chan[channel];
    if (!(c->csr & 0x08)) return 0;       /* not active */
    if (g_dma_delay[channel] > 0) {       /* still in transfer-startup latency */
        g_dma_delay[channel]--;
        return 0;
    }
    if (c->mtc == 0) goto end_transfer;

    int dir     = (c->ocr & 0x80) ? 1 : 0;
    int scr_mar = (c->scr >> 2) & 0x03;

    if (channel == 0) {
        if (!(g_ncr.mode & 0x02)) return 0;
        int start_phase = g_ncr.phase;
        if (g_dma_bytes[channel] == 0) {
            g_dma_mar0[channel] = c->mar;
            g_dma_phase0[channel] = start_phase;
        }
        if (dir == 0) {
            if (g_ncr.phase != PHASE_COMMAND &&
                g_ncr.phase != PHASE_DATA_OUT)
                goto end_transfer;
            uint8_t b = host_mem_read(c->mar, 1) & 0xFF;
            g_ncr.odata = b;
            ncr_advance_phase();
        } else {
            if (g_ncr.phase != PHASE_DATA_IN &&
                g_ncr.phase != PHASE_STATUS  &&
                g_ncr.phase != PHASE_MSG_IN)
                goto end_transfer;
            uint8_t b = 0;
            if (g_ncr.phase == PHASE_DATA_IN)
                b = (g_ncr.data_len > 0) ? *g_ncr.data_buf : 0;
            else if (g_ncr.phase == PHASE_STATUS)
                b = g_ncr.status_byte;
            else if (g_ncr.phase == PHASE_MSG_IN)
                b = g_ncr.msg_byte;
            host_mem_write(c->mar, b, 1);
            ncr_advance_phase();
        }
        if      (scr_mar == 1) c->mar += 1;
        else if (scr_mar == 2) c->mar -= 1;
        c->mtc--;
        g_dma_bytes[channel]++;
        /* Stop this DMA session when the SCSI phase changes -- the host
         * re-dispatches a fresh DMA setup for each phase via its phase
         * handler at $00100EC8.  The phase-mismatch IRQ flag (BAS bit 4)
         * is what the host's poll loop watches for. */
        if (g_ncr.phase != start_phase) goto end_transfer;
        return 1;
    }
    /* Other channels not used by Triple X. */
    return 0;

end_transfer:
    g_dma_bytes[channel] = 0;
    c->csr &= ~0x08;
    c->csr |= 0xE0;             /* COC + BTC + NDT */
    /* Signal EOP to NCR.  Real DMAC asserts EOP at end of cycle. */
    if (channel == 0) g_ncr.bas |= 0x80;
    /* Fire the channel's interrupt on a true MTC=0 completion.  Like the
     * PCL0 doorbell, this is a *vectored* interrupt -- the HD63450 supplies
     * the channel's NIV on the IRQ3 acknowledge.  Without the vector the
     * 68010 autovectors to 27 and the kernel panics ("unexpected kernel
     * trap, type 27"). */
    if (c->mtc == 0 && (c->ccr & 0x08)) {
        g_dmac_irq_vec = c->niv;
        host_irq_assert(3);
    }
    return 0;
}

/* Drive a small batch of DMA bytes per host instruction. */
void dmac_step_all(void) {
    for (int ch = 0; ch < 4; ch++) {
        if (g_dmac_chan[ch].csr & 0x08) {
            /* Drain up to a chunk; channel will idle if NCR isn't ready. */
            for (int i = 0; i < 64; i++)
                if (!dmac_step_channel(ch)) break;
        }
    }
}


static void dmac_write(uint32_t a, uint8_t val) {
    int channel = (a >> 6) & 3;
    int off = a & 0x3F;
    hd63450_chan_t *c = &g_dmac_chan[channel];
    switch (off) {
    case 0x00:                                    /* CSR: write-1-to-clear */
        c->csr &= ~(val & 0xF6);
        if (val & 0x10) c->cer = 0;
        break;
    case 0x04: c->dcr = val; break;
    case 0x05: c->ocr = val; break;
    case 0x06: c->scr = val; break;
    case 0x07:
        c->ccr = val;
        if (val & 0x80) dmac_start_xfer(channel);
        if (val & 0x10) { c->csr &= ~0x08; }     /* software abort */
        break;
    case 0x0A: c->mtc = (c->mtc & 0x00FF) | ((uint16_t)val << 8); break;
    case 0x0B: c->mtc = (c->mtc & 0xFF00) | val; break;
    case 0x0C: c->mar = (c->mar & 0x00FFFFFFu) | ((uint32_t)val << 24); break;
    case 0x0D: c->mar = (c->mar & 0xFF00FFFFu) | ((uint32_t)val << 16); break;
    case 0x0E: c->mar = (c->mar & 0xFFFF00FFu) | ((uint32_t)val <<  8); break;
    case 0x0F: c->mar = (c->mar & 0xFFFFFF00u) |             val;       break;
    case 0x14: c->dar = (c->dar & 0x00FFFFFFu) | ((uint32_t)val << 24); break;
    case 0x15: c->dar = (c->dar & 0xFF00FFFFu) | ((uint32_t)val << 16); break;
    case 0x16: c->dar = (c->dar & 0xFFFF00FFu) | ((uint32_t)val <<  8); break;
    case 0x17: c->dar = (c->dar & 0xFFFFFF00u) |             val;       break;
    case 0x1A: c->btc = (c->btc & 0x00FF) | ((uint16_t)val << 8); break;
    case 0x1B: c->btc = (c->btc & 0xFF00) | val; break;
    case 0x1C: c->bar = (c->bar & 0x00FFFFFFu) | ((uint32_t)val << 24); break;
    case 0x1D: c->bar = (c->bar & 0xFF00FFFFu) | ((uint32_t)val << 16); break;
    case 0x1E: c->bar = (c->bar & 0xFFFF00FFu) | ((uint32_t)val <<  8); break;
    case 0x1F: c->bar = (c->bar & 0xFFFFFF00u) |             val;       break;
    case 0x25: c->niv = val; break;
    case 0x27: c->eiv = val; break;
    case 0x29: c->mfc = val; break;
    case 0x2D: c->cpr = val; break;
    case 0x31: c->dfc = val; break;
    case 0x39: c->bfc = val; break;
    case 0x3F: c->gcr = val; break;
    }
}

/* --- MC68451 MMU (host $0A0000-$0A00FF) ---
 * 32 segment descriptors programmed through an 9-byte accumulator + a
 * descriptor pointer.  Datasheet: MC68451 MMU, Motorola Apr 1983.  Only
 * logical address bits A8-A23 are translated (256-byte page granularity);
 * A1-A7 + the data strobes bypass the MMU.  The WERMA System V kernel
 * programs this to run with a virtual supervisor stack at $00FF0000. */
typedef struct {
    uint16_t lba, lam, pba;     /* logical base / logical mask / physical base */
    uint8_t  asn, asmask, ssr;  /* address-space num / mask / status (bit0 = E) */
} mmu_desc_t;

static struct {
    mmu_desc_t desc[32];
    uint8_t    acc[16];         /* accumulator AC0..AC8 at index $00..$08 */
    uint8_t    dp;              /* descriptor pointer (5-bit descriptor number) */
    uint8_t    ast[16];         /* address-space table: the MC68451 AST is
                                 * even-spaced, so MMU register $(fc*2) holds
                                 * function code fc's ASN -> ast[reg>>1]. */
    uint8_t    gsr, lsr, rdp, idp;
    int        enabled;         /* translation active once a descriptor is loaded */
} g_mmu;

extern int g_m68k_current_fc;

static void mmu_reset(void) {
    memset(&g_mmu, 0, sizeof(g_mmu));
    /* Reset state: the master MMU's descriptor 0 identity-maps everything
     * so the CPU runs normally before the OS programs the table. */
    g_mmu.desc[0].lam = 0; g_mmu.desc[0].asmask = 0xFF; g_mmu.desc[0].ssr = 0x01;
    g_mmu.rdp = 0x80;
    g_mmu.enabled = 0;
}

/* The 68451 "Load Descriptor" operation: copy the accumulator into the
 * descriptor selected by DP.  Triggered by a read of register $3F. */
static void mmu_load_descriptor(void) {
    mmu_desc_t *t = &g_mmu.desc[g_mmu.dp & 0x1F];

    /* if (t->lba >= 0xEFC0 && t->lba <= 0xEFF0){

       fprintf(stderr, "STACK-ish MMU LOAD desc=%d E=%d lba=%04X lam=%04X pba=%04X asn=%02X mask=%02X\n",
       g_mmu.dp & 0x1F,
       !!(t->ssr & 1),
       t->lba,
       t->lam,
       t->pba,
       t->asn,
       t->asmask);
    } */


    t->lba    = (g_mmu.acc[0x00] << 8) | g_mmu.acc[0x01];
    t->lam    = (g_mmu.acc[0x02] << 8) | g_mmu.acc[0x03];
    t->pba    = (g_mmu.acc[0x04] << 8) | g_mmu.acc[0x05];
    t->asn    =  g_mmu.acc[0x06];
    t->ssr    =  g_mmu.acc[0x07];        /* bit 0 = E (segment enable) */
    t->asmask =  g_mmu.acc[0x08];
    g_mmu.enabled = 1;                  /* leave reset-identity mode */


    /*if (t->ssr & 1) {
        if (t->lba == 0xEFD8 && t->lam == 0xFFF8) {
            printf("WIDENING STACK DESC: EFD8/FFF8 -> EFD0/FFF0\n");
            t->lba = 0xEFD0;
            t->lam = 0xFFF0;
        }
    }*/


    if ((t->ssr & 1) && t->asn != 0x00 && t->asn != 0xFF) {
        if (t->lba == 0xEFD8 && t->lam == 0xFFF8) {
            t->lba = 0xEFD0;
            t->lam = 0xFFF0;
        }

        if (t->lba == 0xEFD0 && t->lam == 0xFFF0) {
            t->lba = 0xEFC0;
            t->lam = 0xFFE0;
            t->pba -= 0x10;
        }
    }

}

/* MC68451 "Direct Translation" operation (triggered by a read of register
 * $3D).  The host has loaded a logical address into the accumulator
 * (AC0-AC1 = logical page bits A8-A23) and an address-space number into
 * AC6; the MMU searches its descriptors for a match and, on a hit, leaves
 * the translated physical page in AC4-AC5 and reports the result in the
 * LSR/GSR.  WERMA's exec()/page-setup code (e.g. $10561A, $10568E) relies
 * on this to find where a user page is mapped.  Returns the status byte
 * the host reads from $3D: 0 = translation succeeded, non-zero = no
 * descriptor matched. */
static uint8_t mmu_direct_translation(void) {
    uint16_t la16 = (g_mmu.acc[0] << 8) | g_mmu.acc[1];
    uint8_t  asn  =  g_mmu.acc[6];
    for (int d = 0; d < 32; d++) {
        mmu_desc_t *t = &g_mmu.desc[d];
        if (!(t->ssr & 0x01))                   continue;
        if (((la16 ^ t->lba) & t->lam) != 0)    continue;
        /* Masked address-space compare.  A descriptor with ASmask=0 is
         * global (matches any address space); the formula handles that
         * for free.  Per-process isolation depends on this: fork() gives
         * the child its own ASN, so the parent's descriptors must NOT
         * match the child -- never special-case ASN $FF here. */
        if (((asn ^ t->asn) & t->asmask) != 0) continue;
        uint16_t pa16 = (uint16_t)((la16 & ~t->lam) | (t->pba & t->lam));
        g_mmu.acc[4] = pa16 >> 8;          /* AC4-AC5 = translated physical page */
        g_mmu.acc[5] = pa16 & 0xFF;
        g_mmu.dp  = d;                     /* DP points at the matched descriptor */
        g_mmu.lsr = 0x80 | (t->ssr & 0x0F);  /* bit 7 = hit; low nibble = SSR */
        g_mmu.gsr = 0x80;
        return 0x00;                       /* success */
    }
    g_mmu.lsr = 0x00;                      /* no descriptor matched */
    g_mmu.gsr = 0x00;
    return 0xFF;                           /* failure */
}

static uint32_t mmu_reg_read(uint32_t off, int sz) {
    if (off == 0x3F) { mmu_load_descriptor(); return 0; }      /* Load Descriptor */
    if (off == 0x3D) return mmu_direct_translation();          /* Direct Translation */
    if (off == 0x31) return g_mmu.desc[g_mmu.dp & 0x1F].ssr;   /* Transfer Descriptor */
    uint32_t v = 0;
    for (int i = 0; i < sz; i++) {
        uint8_t b = 0, o = (off + i) & 0xFF;
        if      (o <= 0x1E)              b = g_mmu.ast[o >> 1];
        else if (o >= 0x20 && o <= 0x28) b = g_mmu.acc[o - 0x20];
        else if (o == 0x29)              b = g_mmu.dp;
        else if (o == 0x2D)              b = g_mmu.gsr;
        else if (o == 0x2F)              b = g_mmu.lsr;
        else if (o == 0x39)              b = g_mmu.idp;
        else if (o == 0x3B)              b = g_mmu.rdp;
        v = (v << 8) | b;
    }
    return v;
}

extern uint64_t g_insn;
static void mmu_reg_write(uint32_t off, uint32_t v, int sz) {
    if (0 && g_insn > 9470000 && g_insn < 9486000 &&
        (off < 0x20 || off == 0x29 || off == 0x31)) {
        static int aw = 0;
        if (aw < 200) { aw++;
            (void)0;
        }
    }
    for (int i = sz - 1; i >= 0; i--) {
        uint8_t b = (v >> (8 * (sz - 1 - i))) & 0xFF, o = (off + i) & 0xFF;
        if      (o <= 0x1E)              g_mmu.ast[o >> 1] = b;
        else if (o >= 0x20 && o <= 0x28) g_mmu.acc[o - 0x20] = b;
        else if (o == 0x29)              g_mmu.dp  = b & 0x1F;
        else if (o == 0x2D)              g_mmu.gsr = b;
        else if (o == 0x2F)              g_mmu.lsr = b;
        else if (o == 0x31) {            /* Write Segment Status (E bit kept) */
            mmu_desc_t *t = &g_mmu.desc[g_mmu.dp & 0x1F];
            t->ssr = b; /*(t->ssr & 0x01) | (b & 0xFE);*/
        }
        /* writes to AC0..AC8 / DP just stage the accumulator; $3F commits it */
    }
}

/* Translate a 68451 logical address to physical.  Walks the 32 descriptors;
 * a descriptor matches when its enable bit is set, the masked logical-base
 * compare passes, and the masked address-space compare passes. */
static uint32_t mmu_translate(uint32_t logical) {
    if (!g_mmu.enabled || g_mmu_disasm_bypass) return logical;
    uint16_t la16 = (logical >> 8) & 0xFFFF;
    uint8_t  off8 =  logical       & 0xFF;
    uint8_t  casn =  g_mmu.ast[g_m68k_current_fc & 0x0F];

    /*if ((logical & 0xFFFFF000) == 0x00EFD000 ||
    (logical & 0xFFFFF000) == 0x00EFE000) {
    printf("MMU lookup logical=%08X fc=%d casn=%02X\n",
           logical,
           g_m68k_current_fc,
           casn);
    } */



    for (int d = 0; d < 32; d++) {
        mmu_desc_t *t = &g_mmu.desc[d];

        /*if ((logical & 0xFFFFF000) == 0x00EFD000 ||
            (logical & 0xFFFFF000) == 0x00EFE000) {
            printf(" desc=%d E=%d lba=%04X lam=%04X asn=%02X mask=%02X\n",
           d,
           !!(t->ssr & 1),
           t->lba,
           t->lam,
           t->asn,
           t->asmask);
        }*/

        if (!(t->ssr & 0x01))                  continue;  /* E=0: disabled */
        if (((la16 ^ t->lba) & t->lam) != 0)   continue;  /* range mismatch */
        /* Masked address-space compare; ASmask=0 means global.  This is
         * what isolates forked processes -- each has its own ASN, so a
         * parent descriptor must not match the child's address space. */
        if (((casn ^ t->asn) & t->asmask) != 0)
            continue;                                     /* space mismatch */
        uint16_t pa16 = (uint16_t)((la16 & ~t->lam) | (t->pba & t->lam));

        /*if ((logical & 0xFFFFF000) == 0x00EFD000 ||
            (logical & 0xFFFFF000) == 0x00EFE000) {
            printf(" MATCH desc=%d -> pa16=%04X off=%02X phys=%08X\n",
               d,
               pa16,
               off8,
               ((uint32_t)pa16 << 8) | off8);
        } */


        return ((uint32_t)pa16 << 8) | off8;
    }
    /*if ((logical & 0xFFFFF000) == 0x00EFD000 ||
    (logical & 0xFFFFF000) == 0x00EFE000) {
    printf(" NO MATCH logical=%08X fc=%d casn=%02X\n",
           logical,
           g_m68k_current_fc,
           casn);
    }*/

    
    /* host_raise_berr(logical, 0);    */
    return logical;   /* no descriptor matched -> falls through to bus error */
}

/* Debug: dump the live MMU descriptor table + AST, and test-translate a
 * couple of user-space addresses, so a missing user-text mapping shows. */
static void dump_mmu_state(void) {
    extern int g_m68k_current_fc;
    int sfc = g_m68k_current_fc;
    (void)0;
    for (int i = 0; i < 32; i++) {
        mmu_desc_t *t = &g_mmu.desc[i];
        if ((t->ssr & 1) || t->lba || t->lam || t->pba)
            (void)0;
    }
    g_m68k_current_fc = 2;
    (void)0;
    g_m68k_current_fc = 1;
    (void)0;
    g_m68k_current_fc = sfc;
}

static uint32_t host_mem_read(uint32_t addr, int sz) {
    if (addr >= 0xA0000 && addr < 0xA0100)
        return mmu_reg_read(addr & 0xFF, sz);
    uint32_t vaddr = addr;
    addr = mmu_translate(addr);
    /* Full read trace in a tiny window around the scheduler fault: logical
     * -> physical, fc, and the AST entry used -- shows whether the kernel's
     * own code page ($100xxx) is being mis-translated. */
    /* Watch reads of /etc/init's return-address slot $EFFCD0: logging the
     * physical address shows whether the forked child reads the SAME
     * physical page the parent wrote (shared stack page = fork bug). */
    uint32_t v = 0;
    for (int i = 0; i < sz; i++) {
        uint32_t a = addr + i;
        if (host_addr_is_berr(a)) {
            /* Probe: would this logical address map under a USER function
             * code?  If so, the supervisor fault is really an fc-staleness
             * bug (a user fetch translated with the supervisor casn). */
            /*printf("READ BERR: logical=%08X translated=%08X FC=%d\n",
                vaddr, a, g_m68k_current_fc);   */         
            host_raise_berr(vaddr, 0); 
            return 0;
        }
        uint8_t b;
        int reg;
        if (a >= 0x80000 && a < 0x80100) {
            b = dmac_read(a - 0x80000);
        } else if (is_ncr_addr(a, &reg)) {
            b = ncr_read(reg);
        } else if (a >= 0xC0000 && a < 0xC0004) {
            /* LANCE Ethernet: 2 word-wide regs.  Reads are normally word
             * accesses, but if the kernel byte-reads we extract the byte
             * from the 16-bit register. */
            uint16_t w = lance_read16(a & ~1);
            b = (a & 1) ? (uint8_t)(w & 0xFF) : (uint8_t)(w >> 8);
        } else {
            uint8_t *p = host_ptr(a);
            b = p ? host_io_read_quirk(a, *p) : 0;
        }
        /* Host reads of the SP<->host VRAM mailboxes (keyboard $1F0, mouse
         * $2F0, SP-command $3F0): does the host ever poll $3F0 to see the
         * SP's command-complete acknowledgement? */
        if (0 && g_insn > 8550000 &&
            ((a >= 0x1F0 && a < 0x200) || (a >= 0x2F0 && a < 0x300) ||
             (a >= 0x3E0 && a < 0x410) ||
             (a >= 0x15F0 && a < 0x1610) ||  /* SP-stashed kbd matrix code */
             (a >= 0x25F0 && a < 0x2610))) {  /* candidate mouse data slot */
            static int mb_n = 0;
            if (mb_n < 3000) { mb_n++;
                (void)0;
            }
        }
        v = (v << 8) | b;
    }
    return v;
}
static void host_mem_write(uint32_t addr, uint32_t v, int sz) {
    if (addr >= 0xA0000 && addr < 0xA0100) {
        mmu_reg_write(addr & 0xFF, v, sz);
        return;
    }
    uint32_t vaddr = addr;
    addr = mmu_translate(addr);
    /* Watch writes to physical page $154000 -- a kernel stack page that
     * resume() restores a process context from, but which contains 68k
     * code bytes instead.  Shows whether the page is (a) a code page that
     * got double-allocated as a kstack, (b) a kstack never initialised,
     * or (c) a kstack corrupted by a stray write. */
    /* Log writes that land in PID 1's user-text page (physical $176000-
     * $1767FF) -- to see if/when exec() loads the program there. */
    if (0 && addr <= 0x12D7BC && addr + sz > 0x12D7BC)
        (void)0;
    /* Log writes to PID 1's pending-signal mask ($134D28 = proc $134CE8 +
     * $40): shows the moment a signal is posted to PID 1, and from where. */
    /* LANCE writes are word-aligned and should hit lance_write16 with the
     * full word, not be split into bytes -- catch the word case here. */
    if (sz == 2 && addr >= 0xC0000 && addr < 0xC0004) {
        lance_write16(addr, v & 0xFFFF);
        return;
    }
    for (int i = sz - 1; i >= 0; i--) {
        uint32_t a = addr + i;
        if (host_addr_is_berr(a)) { 
            /*printf("WRITE BERR: logical=%08X translated=%08X FC=%d\n",
                vaddr, a, g_m68k_current_fc); */
    
            host_raise_berr(vaddr, 1); 
            return;
        }
        int reg;
        if (a >= 0x80000 && a < 0x80100) {
            dmac_write(a - 0x80000, v & 0xFF);

        /*} else if (is_ncr_addr(a, &reg)) {
            ncr_write(reg, v & 0xFF);*/

        } else if (is_ncr_addr(a, &reg)) {

            /*if (reg == 0) {
                fprintf(stderr,
                        "[NCR WRITE] addr=%08X sz=%d full=%08X byte=%02X\n",
                        (unsigned)addr,
                        sz,
                    (unsigned)v,
                    (unsigned)(v & 0xFF));
            }*/

            ncr_write(reg, v & 0xFF);
        
        } else {
            uint8_t *p = host_ptr(a);
            uint8_t prev = p ? *p : 0;
            if (p) *p = v & 0xFF;
			
			/*
			* Trace host 68010 writes to the 16-byte host -> service processor
			* command mailbox.
			*
			* Host $03F0-$03FF corresponds to SP $43F0-$43FF when VIDSEL selects
			* bank 0.
			*
			* This is particularly useful for finding commands generated by the
			* OpenTop Palette Editor.
			*/

			/*
			* Temporary palette investigation.
			*
			* Log changed bytes written by the 68010 into the lowest 4 KB of
			* shared video RAM.  This includes the known $03F0 mailbox but may
			* reveal another control structure used by the OpenTop palette driver.
			*/

			/*
			* Palette investigation:
			* only watch host writes to $04F0-$04FF.
			*/
			/*if (p &&
				a >= 0x04F0 && a <= 0x04FF &&
				prev != (uint8_t)(v & 0xFF)) {

				fprintf(stderr,
					"[PAL CANDIDATE] PC=%08X addr=%04X old=%02X new=%02X\n",
					m68k_get_reg(NULL, M68K_REG_PC),
					(unsigned)a,
					prev,
					(unsigned)(v & 0xFF));
			} */

            /*if (a >= 0x03F0 && a <= 0x03FF &&
                prev != (uint8_t)(v & 0xFF)) {

                fprintf(stderr,
                        "[MAILBOX] PC=%08X addr=%04X %02X->%02X\n",
                        m68k_get_reg(NULL, M68K_REG_PC),
                        (unsigned)a,
                        prev,
                        (unsigned)(v & 0xFF));
            } */
								
            /* Z8530 SCC at $40000-$40007: log writes to the data registers
             * (odd byte offsets) so the kernel's serial-console output --
             * boot banner, panic messages -- is visible. */
            if (0 && a >= 0x40000 && a < 0x40008 && (a & 1)) {
                uint8_t ch = v & 0xFF;
                (void)0;
            }
            /* Watch host writes to the kbd/mouse mailboxes ($1F0-$1FF /
             * $2F0-$2FF) -- helps confirm whether the host acks/clears
             * the mailbox or whether someone other than the SP is writing
             * the key data. */
            if (0 && p && prev != (v & 0xFF) &&
                ((a >= 0x1F0 && a < 0x200) || (a >= 0x2F0 && a < 0x300))) {
                static int hw = 0;
                if (hw < 400) { hw++;
                    (void)0;
                }
                /* Loud notification for the key event we are watching for. */
                if (a == 0x2FA && (v & 0xFF) != 0) {
                    (void)0;
                }
            }
            /* During the period after kbd-ready, also widely log low-VRAM
             * writes so we can see whether the kernel touches mouse-area
             * bytes via paths other than the direct mailbox addresses. */
            if (0 && p && prev != (v & 0xFF) &&
                a >= 0x200 && a < 0x600 && g_insn > 50000000) {
                static int hwm = 0;
                if (hwm < 80) { hwm++;
                    (void)0;
                }
            }
            /* Kernel-initiated reboot: detect the host CPU writing the
             * cmd-$0B (warm reboot) byte to the host->SP mailbox at host
             * VRAM offset $3F0 (= SP-side $43F0).  Using a host-write
             * intercept rather than SP PC=$C9F5 avoids false positives
             * from cold-boot SP firmware dispatching against stale RAM:
             * the host m68k is not running during the SP cold-boot
             * window, so this branch only fires for a real reboot(2)
             * call from a booted kernel. */
            extern int g_sp_reboot_pending;
            extern int g_host_halted;
            if (a == 0x3F0 && (v & 0xFF) == 0x0B && !g_sp_reboot_pending &&
                g_insn > 20000000ULL) {
                g_sp_reboot_pending = 1;
                g_host_halted = 1;
                fprintf(stderr, "[REBOOT] kernel wrote $0B to $43F0 "
                        "(host PC=%08X, insn=%llu); halting host until "
                        "SP POST releases it\n",
                        m68k_get_reg(NULL, M68K_REG_PC),
                        (unsigned long long)g_insn);
            }
        }
        v >>= 8;
    }
}

/* Debug: which read path touches the $80xxxx user range with a supervisor
 * fc?  Distinguishes a real prefetch/data access from instrumentation. */
static void dbg_read80(const char *who, unsigned int a) {
    extern int g_m68k_current_fc;
    if (0 && g_insn > 14000000 && (a >> 16) == 0x80 &&
        g_m68k_current_fc >= 5) {
        static int r80 = 0;
        if (r80 < 10) { r80++;
            (void)0;
        }
    }
}
unsigned int m68k_read_memory_8 (unsigned int a) { dbg_read80("mem8", a);  return host_mem_read(a, 1); }
unsigned int m68k_read_memory_16(unsigned int a) { dbg_read80("mem16", a); return host_mem_read(a, 2); }
unsigned int m68k_read_memory_32(unsigned int a) { dbg_read80("mem32", a); return host_mem_read(a, 4); }
void m68k_write_memory_8 (unsigned int a, unsigned int v) { host_mem_write(a, v, 1); }
void m68k_write_memory_16(unsigned int a, unsigned int v) { host_mem_write(a, v, 2); }
void m68k_write_memory_32(unsigned int a, unsigned int v) { host_mem_write(a, v, 4); }
unsigned int m68k_read_immediate_8 (unsigned int a) { dbg_read80("imm8", a);  return host_mem_read(a, 1); }
unsigned int m68k_read_immediate_16(unsigned int a) { dbg_read80("imm16", a); return host_mem_read(a, 2); }
unsigned int m68k_read_immediate_32(unsigned int a) { dbg_read80("imm32", a); return host_mem_read(a, 4); }
unsigned int m68k_read_pcrelative_8 (unsigned int a) { return m68k_read_memory_8(a); }
unsigned int m68k_read_pcrelative_16(unsigned int a) { return m68k_read_memory_16(a); }
unsigned int m68k_read_pcrelative_32(unsigned int a) { return m68k_read_memory_32(a); }
unsigned int m68k_read_disassembler_8 (unsigned int a) { return m68k_read_memory_8(a); }
unsigned int m68k_read_disassembler_16(unsigned int a) { return m68k_read_memory_16(a); }
unsigned int m68k_read_disassembler_32(unsigned int a) { return m68k_read_memory_32(a); }
void m68k_set_fc(unsigned int fc) { g_m68k_current_fc = fc; }

/* No host-side sync needed: host_ptr() returns &g_ram[$4000+addr] directly
 * for VRAM accesses, so host and SP share the same byte storage. */
static void host_sync_from_sp(void) {}

#endif

#ifdef USE_SDL
/* SDL framebuffer.  Per MAME's CRTC update row: each scanline reads
 * x_count*2 = 180 bytes from VRAM, each byte expands to 8 pixels (in
 * mode 2 / 1bpp), giving 1440 displayed pixels per scanline.  We render
 * 256 lines (one CRT field; the second interlaced field reads the same
 * data so it's a repeat). */
#define FB_W 720
#define FB_H 256
static SDL_Window   *g_win  = NULL;
static SDL_Renderer *g_ren  = NULL;
static SDL_Texture  *g_tex  = NULL;
static uint32_t      g_fb[FB_W * FB_H];

static int sdl_init(int scale) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        (void)0; return -1;
    }
    g_win = SDL_CreateWindow("Torch Triple X",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        FB_W * scale, FB_H * scale * 2, 0);
    if (!g_win) { (void)0; return -1; }
    g_ren = SDL_CreateRenderer(g_win, -1, SDL_RENDERER_ACCELERATED);
    if (!g_ren) g_ren = SDL_CreateRenderer(g_win, -1, 0);
    if (!g_ren) { (void)0; return -1; }
    g_tex = SDL_CreateTexture(g_ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, FB_W, FB_H);
    SDL_RenderSetIntegerScale(g_ren, SDL_TRUE);
    SDL_RenderSetLogicalSize(g_ren, FB_W, FB_H*2);
    SDL_ShowCursor(SDL_DISABLE);
    return 0;
}

/* Translate a palette byte into 24-bit RGB. The Triple X palette is a
 * 74S189 16x4-bit static RAM (schematic page 12); each entry is a 4-bit
 * field driving R0/R1/G0/G1 (and similar) DACs.  We crack the byte as
 * RRRGGGBB which matches the firmware-side gradients. */
 
/*static uint32_t palette_to_rgb(uint8_t v) {
    uint8_t r = ((v >> 5) & 0x07) * 0x24;
    uint8_t g = ((v >> 2) & 0x07) * 0x24;
    uint8_t b = ( v       & 0x03) * 0x55;
    return 0xFF000000u | (r << 16) | (g << 8) | b;
}*/

static uint32_t palette_to_rgb(uint8_t value)
{
    /*
     * Triple X palette:
     *
     *   bits 7-5: inverted red
     *   bits 4-2: inverted green
     *   bits 1-0: inverted blue
     */
    uint8_t rgb = (uint8_t)~value;

    unsigned r3 = (rgb >> 5) & 0x07;
    unsigned g3 = (rgb >> 2) & 0x07;
    unsigned b2 = rgb & 0x03;

    unsigned r = (r3 * 255U) / 7U;
    unsigned g = (g3 * 255U) / 7U;
    unsigned b = (b2 * 255U) / 3U;

    return 0xff000000U |
           (r << 16) |
           (g << 8) |
           b;
}


/* Render VRAM at $4000+ as a 720x256 mono bitmap.
 * Layout: 256 bytes per scanline (90 displayed, 166 invisible).  Confirmed
 * by the CARETAKER 1.3 firmware drawing "Please insert the key disc" at
 * VRAM offset $1200 with stride 256. */
static void get_display_colours(uint32_t colors[4])
{
    /*
	* TODO: Runtime palette programming.
	*
	* Boot-time display colours are currently emulated using known display
	* states. OpenTop's Palette Editor does not appear to program the SP
	* palette through the currently emulated $03F0 mailbox or direct
	* $0500-$050F SP palette writes.
	*
	* Runtime palette changes require tracing the host video-driver path
	* to determine the original hardware palette programming mechanism.
	*/
	
	
	
	switch (g_video_display_state) {
    case VIDEO_DISPLAY_PALE_BLUE:
        /* Power-on/RAM-test screen: the entire display is cyan. */
        colors[0] = 0xFF00FFFFu;
        colors[1] = 0xFF00FFFFu;
        colors[2] = 0xFF00FFFFu;
        colors[3] = 0xFF00FFFFu;
        break;

    case VIDEO_DISPLAY_DARK_BLUE:
        /* RAM test complete/Caretaker version display.  Index 0 is the
         * background and index 3 is the principal foreground colour.
         * Background is dark blue and text is cyan. */
        colors[0] = 0xFF0000FFu;
        colors[1] = 0xFF49658Fu;
        colors[2] = 0xFF7891B5u;
        colors[3] = 0xFF00FFFFu;
        break;

    case VIDEO_DISPLAY_NORMAL:
    default:
        if (g_runtime_palette_programmed) {
            /* OpenTop or another program has written the palette after
             * startup.  Honour the physical 16-entry palette RAM exactly. */
            colors[0] = palette_to_rgb(g_palette[0]);
            colors[1] = palette_to_rgb(g_palette[1]);
            colors[2] = palette_to_rgb(g_palette[2]);
            colors[3] = palette_to_rgb(g_palette[3]);
        } else {
            /* Normal Caretaker/OpenTop colours observed on real hardware. */
            colors[0] = 0xFFD8D8D8u;  /* grey background */
            colors[1] = 0xFFD00000u;  /* red */
            colors[2] = 0xFF00A000u;  /* green */
            /*colors[2] = 0xFF0000A0u;  /* blue */
            colors[3] = 0xFF000000u;  /* black */
        }
        break;
    }
}

/* Render VRAM as a 720x256, four-colour (2 bits per pixel) display.
 * Each scanline occupies 256 bytes; the first two bytes are not displayed
 * and the following 180 bytes contain 720 visible pixels. */
static void render_frame_to_fb(void)
{
    const int STRIDE = 256;
    const int BYTES_PER_ROW = 180;
    uint32_t colors[4];

    get_display_colours(colors);

    for (int y = 0; y < FB_H; y++) {
        uint32_t base = (uint32_t)y * STRIDE + 2;

        for (int xb = 0; xb < BYTES_PER_ROW; xb++) {
            uint8_t v = g_vram[(base + (uint32_t)xb) & 0xFFFFu];
            int x = xb * 4;

            g_fb[y * FB_W + x + 0] = colors[(v >> 6) & 3];
            g_fb[y * FB_W + x + 1] = colors[(v >> 4) & 3];
            g_fb[y * FB_W + x + 2] = colors[(v >> 2) & 3];
            g_fb[y * FB_W + x + 3] = colors[v & 3];
        }
    }
}

static void render_frame(void) {
    render_frame_to_fb();
    SDL_UpdateTexture(g_tex, NULL, g_fb, FB_W * 4);
    SDL_RenderClear(g_ren);
    SDL_RenderCopy(g_ren, g_tex, NULL, NULL);
    SDL_RenderPresent(g_ren);
}

/* Dump the current framebuffer as a binary PPM -- lets us inspect the
 * screen without a live display (SDL dummy video driver). */
static void dump_ppm(const char *path) {
    render_frame_to_fb();
    FILE *p = fopen(path, "wb");
    if (!p) { perror(path); return; }
    fprintf(p, "P6\n%d %d\n255\n", FB_W, FB_H);
    for (int i = 0; i < FB_W * FB_H; i++) {
        uint32_t c = g_fb[i];
        unsigned char rgb[3] = { (c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF };
        fwrite(rgb, 1, 3, p);
    }
    fclose(p);
    (void)0;
}

/* Translate an SDL keycode into a Torch Triple X keyboard matrix code
 * ($01-$5A).  Returns 0 for keys with no Torch equivalent.  The make
 * (press) code is the matrix code; the break (release) code is matrix|$80. */
static uint8_t torch_matrix_code(SDL_Keycode k) {
    switch (k) {
    case SDLK_1: return 2;  case SDLK_2: return 3;  case SDLK_3: return 4;
    case SDLK_4: return 5;  case SDLK_5: return 6;  case SDLK_6: return 7;
    case SDLK_7: return 8;  case SDLK_8: return 9;  case SDLK_9: return 10;
    case SDLK_0: return 11;
    case SDLK_MINUS: return 12;   case SDLK_EQUALS: return 13;
    case SDLK_BACKSPACE: return 14; case SDLK_TAB: return 15;
    case SDLK_q: return 16; case SDLK_w: return 17; case SDLK_e: return 18;
    case SDLK_r: return 19; case SDLK_t: return 20; case SDLK_y: return 21;
    case SDLK_u: return 22; case SDLK_i: return 23; case SDLK_o: return 24;
    case SDLK_p: return 25;
    case SDLK_LEFTBRACKET: return 26; case SDLK_RIGHTBRACKET: return 27;
    case SDLK_RETURN: return 28;
    case SDLK_LCTRL: case SDLK_RCTRL: return 29;
    case SDLK_a: return 30; case SDLK_s: return 31; case SDLK_d: return 32;
    case SDLK_f: return 33; case SDLK_g: return 34; case SDLK_h: return 35;
    case SDLK_j: return 36; case SDLK_k: return 37; case SDLK_l: return 38;
    case SDLK_SEMICOLON: return 39; case SDLK_QUOTE: return 40;
    case SDLK_BACKQUOTE: return 41;
    case SDLK_LSHIFT: return 42;
    case SDLK_BACKSLASH: return 43;
    case SDLK_z: return 44; case SDLK_x: return 45; case SDLK_c: return 46;
    case SDLK_v: return 47; case SDLK_b: return 48; case SDLK_n: return 49;
    case SDLK_m: return 50;
    case SDLK_COMMA: return 51; case SDLK_PERIOD: return 52;
    case SDLK_SLASH: return 53;
    case SDLK_RSHIFT: return 54;
    case SDLK_F11: case SDLK_HELP: return 55;       /* HELP */
    case SDLK_SPACE: return 57;
    case SDLK_CAPSLOCK: return 58;
    case SDLK_F1: return 59; case SDLK_F2: return 60; case SDLK_F3: return 61;
    case SDLK_F4: return 62; case SDLK_F5: return 63; case SDLK_F6: return 64;
    case SDLK_F7: return 65; case SDLK_F8: return 66; case SDLK_F9: return 67;
    case SDLK_F10: return 68;
    case SDLK_LGUI: case SDLK_RGUI: return 70;      /* DIAMOND */
    case SDLK_KP_7: return 71; case SDLK_KP_8: return 72; case SDLK_KP_9: return 73;
    case SDLK_KP_MINUS: return 74;
    case SDLK_KP_4: return 75; case SDLK_KP_5: return 76; case SDLK_KP_6: return 77;
    case SDLK_KP_PLUS: return 78;
    case SDLK_KP_1: return 79; case SDLK_KP_2: return 80; case SDLK_KP_3: return 81;
    case SDLK_KP_PERIOD: return 82; case SDLK_KP_0: return 83;
    case SDLK_KP_ENTER: return 84;
    case SDLK_KP_MULTIPLY: return 85; case SDLK_KP_DIVIDE: return 86;
    case SDLK_UP: return 87; case SDLK_DOWN: return 88;
    case SDLK_LEFT: return 89; case SDLK_RIGHT: return 90;
    case SDLK_ESCAPE: return 1;
    default: return 0;
    }
}
#endif

int main(int argc, char **argv) {
  #ifdef _WIN32
    _putenv_s("TRIPLEX_SPEED", "16");
  #else  
    setenv("TRIPLEX_SPEED", "16", 1);
  #endif
    const char *rom_path  = "triplex.rom";
    const char *disk_path = NULL;
    const char *g_tap_name = NULL;
    extern int g_lance_trace;  /* defined below; forward decl for early callers */
    int g_test_reboot = 0;             /* --test-reboot: inject cmd $0B at boot */
    long max_steps = 0;          /* 0 = forever */
    int verbose = 0;
    int use_sdl = 0;
    int scale   = 2;
    int use_host = 0;
    const char *shot_path = NULL;
    const char *keydisk_path = NULL;
    const char *unix_floppy_path = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--rom") && i+1 < argc) rom_path = argv[++i];
        else if (!strcmp(argv[i], "--disk") && i+1 < argc) disk_path = argv[++i];
        else if (!strcmp(argv[i], "--keydisk") && i+1 < argc) keydisk_path = argv[++i];

        else if (!strcmp(argv[i], "--unix-floppy") && i+1 < argc) unix_floppy_path = argv[++i];

        else if (!strcmp(argv[i], "--tap") && i+1 < argc) g_tap_name = argv[++i];
        else if (!strcmp(argv[i], "--shot") && i+1 < argc) shot_path = argv[++i];
        else if (!strcmp(argv[i], "--max") && i+1 < argc) max_steps = atol(argv[++i]);
        else if (!strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose")) {
            verbose = 1; g_log_io = 1; g_log_unmapped = 1;
        }
        else if (!strcmp(argv[i], "--sdl")) use_sdl = 1;
        else if (!strcmp(argv[i], "--scale") && i+1 < argc) scale = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--host")) use_host = 1;
        else if (!strcmp(argv[i], "--type") && i+1 < argc) g_type_str = argv[++i];
        else if (!strcmp(argv[i], "--test-reboot")) g_test_reboot = 1;
        else if (!strcmp(argv[i], "--lance-trace")) g_lance_trace = 1;
        else if (!strcmp(argv[i], "--mouse")) g_mouse_test = 1;
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("Torch Triple X emulator (HD6303R service processor)\n"
                   "  --rom PATH          EPROM image (default triplex.rom)\n"
                   "  --disk PATH         raw SCSI hard-disc image (NCR5380 unit 0)\n"
                   "  --keydisk PATH      ImageDisk (.IMD) key disc (NCR5380 unit 1)\n"
                   "  --unix-floppy PATH  Raw 512 byte sector floppy image mounted as SCSI unit 1\n"
                   "  --host              also run the MC68010 host CPU\n"
                   "  --sdl               open an SDL window (720x256), forward kbd/mouse\n"
                   "  --scale N           SDL window scale factor (default 2)\n"
                   "  --shot PATH         periodically dump the screen as a PPM to PATH\n"
                   "  --type STR          headless: inject STR as keystrokes after boot\n"
                   "  --mouse             headless: inject mouse motion after boot\n"
                   "  --max N             stop after N instructions (0 = forever)\n"
                   "  -v --verbose        log I/O accesses\n");
            return 0;
        }
    }
    (void)use_host;
    FILE *f = fopen(rom_path, "rb");
    if (!f) { perror(rom_path); return 1; }
    fread(g_sp_rom, 1, ROM_SIZE, f);
    fclose(f);
    printf("Torch Triple X / Stickleback service processor emulator\n");
    printf("  ROM: %s loaded at 0x%04X-0x%04X\n", rom_path, ROM_BASE, ROM_BASE+ROM_SIZE-1);

    /*FILE *fp = fopen("torch_serial.bin", "rb");
    if (fp){
        fread(&g_rtc_user_ram[0x0E], 1, 4, fp);
        fread(g_saved_serial, 1, 4, fp);
        fclose(fp);
        fprintf(stderr,"[CMOS] Found and loaded pre-stored torch_serial.bin\n");
    } */

    FILE *fp = fopen("torch_serial.bin", "rb");
    if (fp) {
        if (fread(g_saved_serial, 1, 4, fp) == 4) {

        /* Copy the saved serial into the RTC CMOS as well. */
            memcpy(&g_rtc_user_ram[0x0E], g_saved_serial, 4);

            fprintf(stderr,
                "[CMOS] Found and loaded pre-stored torch_serial.bin: "
                "%02X%02X%02X%02X\n",
                g_saved_serial[0],
                g_saved_serial[1],
                g_saved_serial[2],
                g_saved_serial[3]);
        }
        else {
            fprintf(stderr,
                "[CMOS] Failed to read complete serial from torch_serial.bin\n");
        }

        fclose(fp);
    }

    network_cmos_init();

    if (disk_path) {
        extern void ncr_load_disk(const char *);
        ncr_load_disk(disk_path);
    }
    if (keydisk_path) {
        extern void ncr_load_keydisk_imd(const char *);
        ncr_load_keydisk_imd(keydisk_path);
    }
    
    if (unix_floppy_path) {
        extern void ncr_load_unix_floppy(const char *);
        ncr_load_unix_floppy(unix_floppy_path);
    }
    
    if (g_tap_name) g_tap_fd = tap_open(g_tap_name);

#ifdef USE_SDL
    if (use_sdl) {
        if (sdl_init(scale) < 0) return 1;
    }
#else
    (void)use_sdl; (void)scale;
#endif

#ifdef USE_M68K
    int host_started = 0;
    if (use_host) {
        m68k_init();
        m68k_set_cpu_type(M68K_CPU_TYPE_68010);
        dmac_reset();
        mmu_reset();
        /* Don't pulse_reset yet -- we wait until the SP has finished copying
         * ROM $D000-$FFEF to shared RAM at SP $4000-$6FED. */
    }
#endif

    cpu_reset();
    long steps;
    int running = 1;
    long steps_since_frame = 0;
    long shot_ctr = 0;
    for (steps = 0; running && (max_steps == 0 || steps < max_steps); steps++) {
        if (!cpu_step()) break;
        /* Watch for the SP entering its cmd $0B handler at $C9F5, which
         * is reached only when the kernel writes 0x0B to the host->SP
         * mailbox $43F0 -- i.e. an explicit reboot(2) request.  When that
         * happens, the SP firmware:
         *   (a) asserts P1.3 to halt the host,
         *   (b) jumps to $C2E0 and re-runs its POST,
         *   (c) eventually clears P1.3 to release the host again.
         * Mark this pending so the next P1.3 release will trigger a host
         * m68k_pulse_reset() -- not the natural multiple P1.3 toggles
         * the SP does during cold-boot POST. */
        /* Reboot detection moved to the host-write path
         * (cmd-$0B byte to host VRAM $3F0).  No SP-side trigger here;
         * see the m68k_write_memory intercept above.  --test-reboot
         * still synthesises a fake reboot once the host has booted. */
        if (g_test_reboot && !g_sp_reboot_pending && host_started &&
            g_insn > 50000000ULL) {
            g_sp_reboot_pending = 1;
            g_host_halted = 1;
            g_test_reboot = 0;  /* one-shot */
            fprintf(stderr, "[REBOOT] --test-reboot: injecting synthetic "
                    "reboot at insn=%llu\n", (unsigned long long)g_insn);
        }
        /* Sample the SP's PC during the host's hang window, to tell a live
         * CARETAKER (cycling its main loop) from a stuck/crashed one. */
        tick_timer(4);
        ptm_tick(4);
        rtc_tick(4);
        sci_rx_feed();          /* clock queued keyboard bytes into the SCI */
        acia_rx_feed();         /* clock queued mouse bytes into the 6850 */
        /* Poll the TAP fd for incoming Ethernet packets every ~1000 SP
         * cycles (so RX latency is at most ~1 ms simulated).  No-op if
         * --tap wasn't passed. */
        static int rx_poll_ctr = 0;
        if (g_tap_fd >= 0 && ++rx_poll_ctr >= 1000) {
            rx_poll_ctr = 0;
            lance_rx_poll();
            lance_refresh_irq();
        }
        /* HD6303 interrupt dispatch, priority order: OCF > TOF > SCI
         * (keyboard) > IRQ1.  Per the MAME triplex driver, /IRQ1 is the
         * merged output of the PTM, ACIA, and RTC interrupt outputs (any
         * one of them pulling /IRQ1 low fires the SP IRQ1 ISR at $CC4A).
         * The PTM contribution is the periodic poll-driver that lets the
         * ISR consume mouse bytes even when ACIA Rx IRQ is off. */
        if (ocf_pending())            cpu_interrupt(0xFFF4);
        else if (tof_pending())       cpu_interrupt(0xFFF2);
        else if (sci_pending())       cpu_interrupt(0xFFF0);
        else if (acia_irq1_pending() || ptm_irq1_pending())
            cpu_interrupt(0xFFF8);
#ifdef USE_M68K
        /* Once SP has reached its main idle loop, snapshot shared RAM into
         * the host's view and start the host CPU. */
        if (use_host && !host_started && g_host_p1_released) {
            host_sync_from_sp();
            m68k_pulse_reset();
            host_started = 1;
            /* On real hardware the host's stage-1 bootstrap copy
             * (VRAM -> DRAM, MOVE.W loop ending in JMP $001011EA) runs
             * fast enough to finish before the SP touches the shared
             * VRAM bootstrap region again (SP code at $C5B6+ writes a
             * 13-byte struct to VRAM $F0).  Our interleaved scheduler
             * would let the SP corrupt the copy mid-flight, so run the
             * host exclusively until it leaves VRAM and jumps into the
             * DRAM-resident image (host PC reaches the $00100000 range). */
            int guard = 0;
            while (m68k_get_reg(NULL, M68K_REG_PC) < 0x00100000 &&
                   guard++ < 20000000) {
                m68k_execute(64);
            }
        }
        /* Kernel-initiated reboot follow-up.  Once the host wrote cmd $0B
         * to $43F0, halt the host and immediately re-run the entire
         * cold-boot sequence: reset the SP, clear the host-start flags,
         * and let the main loop's existing cold-boot path bring the
         * host back up via the normal SP-POST -> P1.3-release -> host
         * bootstrap chain.  This mirrors real hardware: the SP CPU
         * itself re-runs from its reset vector and refills the shared
         * VRAM bootstrap region from ROM, then releases the host. */
        if (use_host && host_started && g_sp_reboot_pending && g_host_halted) {
            fprintf(stderr, "[REBOOT] re-running cold-boot sequence\n");
            cpu_reset();
            host_started = 0;
            g_host_p1_released = 0;
            g_sp_reboot_pending = 0;
            g_host_halted = 0;
            /* I/O regs (PTM, ACIA, RTC) intentionally retained so the
             * RTC/CMOS settings (B-NET/NFS bits, MAC) survive reboot. */
        }
        if (use_host && host_started && !g_host_halted) {

        #ifdef USE_SDL
        if (g_boot_slow_mode &&
            (SDL_GetTicks() - g_boot_slow_start_ms) >= CARETAKER_SLOW_WINDOW_MS) {

            g_boot_slow_mode = 0;
            g_caretaker_host_counter = 0;

            fprintf(stderr,
                    "[BOOT] Caretaker restored to full speed\n");
        }
        #endif

            if (g_boot_slow_mode) {

                g_caretaker_host_counter++;

                if (g_caretaker_host_counter >= CARETAKER_HOST_DIVISOR) {
                    g_caretaker_host_counter = 0;
                    m68k_execute(200);
                }

            } else {

                g_caretaker_host_counter = 0;
                m68k_execute(200);
            }   

        /* existing VSYNC / IRQ code continues here */

            /* Pulse the host's IRQ6 line on each emulated VSYNC edge.
             * On real Triple X, the 6845E CRTC VSYNC output is routed
             * (via service-bus glue) to the 74148 priority encoder
             * feeding the 68010's IPL pins. */
            static int prev_vsync = 0;
            if (g_crtc_in_vsync && !prev_vsync) host_irq_assert(6);
            prev_vsync = g_crtc_in_vsync;
        }
#endif
        if (++g_crtc_cycles >= 4000) {
            g_crtc_cycles = 0;
            g_crtc_in_vsync = !g_crtc_in_vsync;
        }
#ifdef USE_SDL
        static int fullscreen = 0;
        /* Refresh display ~60 Hz; at ~1MHz CPU, one frame is ~16K cycles. */
        if (use_sdl && ++steps_since_frame >= 16000) {
            steps_since_frame = 0;
            /* Throttle each frame to a wall-clock budget so the kernel
             * tick counter at $16D5C4 (which /bin/dm uses for double-click
             * detection) advances at the rate dm expects.  Without this,
             * the host CPU runs the emulator at 5-10x real time, click
             * timestamps drift far apart in simulated time, and fast
             * wall-clock double-clicks are seen by dm as separate single
             * clicks -- icons can be selected and dragged but never opened.
             *
             * TRIPLEX_SPEED env var sets the frame budget in milliseconds.
             *   default = 16 (60 Hz, ~1:1 real-time on a 1 MHz SP)
             *   24       (~40 Hz, slower; helps if double-click still fails)
             *   32       (~30 Hz, slower still)
             *   higher   = slower
             * Set TRIPLEX_SPEED=0 to disable throttling entirely. */
            {
                static int frame_budget_ms = 16;
                static Uint32 frame_start_ms = 0;

                /*
                 * This is the frame budget actually used for this frame.
                 *
                 * Normally it is frame_budget_ms.
                 * During the Caretaker STOP window it is temporarily increased
                 * to CARETAKER_FRAME_MS, slowing the emulated machine down.
                 */
                int effective_frame_budget = frame_budget_ms;

                /*
                 * Temporary slowdown beginning when Caretaker switches from
                 * the dark-blue startup screen to the normal palette.
                 */

                if (effective_frame_budget > 0) {
                    Uint32 now = SDL_GetTicks();
    
                    if (frame_start_ms) {
                        Uint32 elapsed = now - frame_start_ms;

                        if (elapsed <
                        (Uint32)effective_frame_budget) {

                        SDL_Delay(
                            effective_frame_budget - elapsed);
                        }
                    }

                    frame_start_ms = SDL_GetTicks();
                }
            }
            render_frame();
            /* Drain all SDL events for this frame.  Coalesce motion (many
             * small SDL_MOUSEMOTION events per frame become one net delta)
             * but emit button transitions IMMEDIATELY -- otherwise a quick
             * click (press+release in the same frame) coalesces into a
             * single packet with the final "released" state and the press
             * never reaches the kernel. */
            int mdx = 0, mdy = 0;
            int cur_left = 0, cur_right = 0;
            SDL_Event ev;
            while (SDL_PollEvent(&ev)) {
                if (ev.type == SDL_QUIT) running = 0;
                
                else if (ev.type == SDL_KEYUP) {

					/*
					* F10 and F11 are emulator controls, so do not send
					* their release codes to the emulated Torch.
					*/
					if (ev.key.keysym.sym == SDLK_F10 ||
					ev.key.keysym.sym == SDLK_F11) {
					continue;
				}

				uint8_t mc =
					torch_matrix_code(ev.key.keysym.sym);

				if (mc)
					kbd_enqueue(mc | 0x80);
			}

                
				
				else if (ev.type == SDL_KEYDOWN) {

					if (ev.key.repeat)
						continue;

					/*
					* F10 = dump complete 64 KB VRAM snapshot.
					*
					* This is intercepted before being sent to the emulated
					* Torch keyboard.
					*/
					if (ev.key.keysym.sym == SDLK_F10) {
	
						char filename[64];

						snprintf(filename,
							sizeof(filename),
							"vram_dump_%03u.bin",
							g_vram_dump_number);

						FILE *f = fopen(filename, "wb");

						if (!f) {
							perror(filename);
						} else {
	
							size_t written =
								fwrite(g_vram, 1, 0x10000, f);

							fclose(f);

							if (written == 0x10000) {

								fprintf(stderr,
									"[VRAM SNAPSHOT] saved %s (65536 bytes)\n",
									filename);

							g_vram_dump_number++;

						} else {

							fprintf(stderr,
								"[VRAM SNAPSHOT] ERROR: only wrote %zu bytes to %s\n",
								written,
								filename);
						}
					}

					continue;
				}
	
				/*
				* F11 = fullscreen toggle.
				*/
				if (ev.key.keysym.sym == SDLK_F11) {
	
					fullscreen = !fullscreen;

					SDL_SetWindowFullscreen(
						g_win,
						fullscreen
							? SDL_WINDOW_FULLSCREEN_DESKTOP
							: 0
					);

					continue;
				}
	
				/*
				* Normal Torch keyboard input.
				*/
				uint8_t mc =
					torch_matrix_code(ev.key.keysym.sym);

				if (mc)
					kbd_enqueue(mc);
			}	
							
                else if (ev.type == SDL_MOUSEMOTION) {
                    mdx += ev.motion.xrel;
                    mdy += ev.motion.yrel;
                    /* Remember current button state for any motion-only
                     * packet we emit at end of frame.  Don't emit yet. */
                    int b = SDL_GetMouseState(NULL, NULL);
                    cur_left  = (b & SDL_BUTTON_LMASK) != 0;
                    cur_right = (b & SDL_BUTTON_RMASK) != 0;
                }
                else if (ev.type == SDL_MOUSEBUTTONDOWN ||
                         ev.type == SDL_MOUSEBUTTONUP) {
                    /* Flush any accumulated motion FIRST so the kernel sees
                     * the cursor at the correct position before the click
                     * register fires.  Then emit the button transition. */
                    int b = SDL_GetMouseState(NULL, NULL);
                    int new_left  = (b & SDL_BUTTON_LMASK) != 0;
                    int new_right = (b & SDL_BUTTON_RMASK) != 0;
                    if (mdx || mdy) {
                        mouse_packet(mdx, mdy, cur_left, cur_right);
                        mdx = 0; mdy = 0;
                    }
                    mouse_packet(0, 0, new_left, new_right);
                    cur_left = new_left; cur_right = new_right;
                    /* SDL pre-classifies multi-clicks for us: ev.button.clicks
                     * is the count within SDL's own ~500 ms window.  When the
                     * user just made the SECOND click of a double-click pair
                     * (clicks >= 2 on a press event), emit an extra
                     * release+press right after to give dm a tight
                     * back-to-back triple-click that it will surely recognise
                     * as a double-click open -- bypasses whatever simulated-
                     * time drift exists between the user's two real presses. */
                    if (ev.type == SDL_MOUSEBUTTONDOWN
                        && ev.button.clicks >= 2) {
                        mouse_packet(0, 0, 0, 0);                /* synth release */
                        mouse_packet(0, 0, new_left, new_right); /* synth press */
                    }
                }			
            }
            /* Emit one motion packet at end of frame if there's residual
             * motion the button-flush did not already drain. */
            if (mdx || mdy) {
                mouse_packet(mdx, mdy, cur_left, cur_right);
            }
        }
        /* Periodic headless screenshot dump (overwrites) so the screen can
         * be inspected even with the SDL dummy video driver. */
        if (shot_path && ++shot_ctr >= 4000000) {
            shot_ctr = 0;
            dump_ppm(shot_path);
        }
        /* Headless input-injection.  Keyboard waits for g_login_ready (PUTC
         * "Name:" match).  Mouse only needs the SP to have initialized its
         * ACIA polling loop, which happens very early (~insn 6.5M) -- so we
         * just gate it on a coarse insn threshold.  Pace one keystroke or
         * mouse step per ~80K main-loop iterations.  Bytes flow through the
         * real SP input path (SCI/ACIA -> CARETAKER -> VRAM mailbox ->
         * PROCINT -> host kernel) -- the screen is the verification. */
        /* Kbd injection starts as soon as either the kernel-banner PUTC
         * trace has caught a "Name:" (werma) OR the host has been running
         * long enough for init to have spawned a login process (torch2,
         * which has no PUTC banner).
         *
         * Make/break pacing matters: a real keyboard holds a key for tens
         * of milliseconds between the make and break codes; back-to-back
         * make+break confuses the tty line discipline, so we alternate
         * make and break on different inject ticks. */
        /* Give getty/login a moment to settle after the prompt is on screen
         * before we start typing -- starting injection too fast loses the
         * first few characters (getty's read isn't ready). */
        static uint64_t g_login_ready_at = 0;
        if (g_login_ready && !g_login_ready_at) g_login_ready_at = g_insn;
        int kbd_eligible = (g_login_ready && g_insn > g_login_ready_at + 50000000)
                        || g_insn > 200000000;
        if ((g_type_str && kbd_eligible) ||
            (g_mouse_test && g_insn > 16000000)) {
            static long inj_ctr = 0;
            static int half = 0;        /* 0 = need-make, 1 = need-break */
            static uint8_t cur_mc = 0;
            static char cur_c = 0;
            if (++inj_ctr >= 80000) {
                inj_ctr = 0;
                /* Run mouse-click sequence FIRST when both --mouse and
                 * --type are active.  Mouse opens a Shell window on the
                 * GUI desktop; typing then drops into that window. */
                if (g_mouse_test && g_mouse_step < 40) {
                    g_mouse_step++;
                    int dx = 0, dy = 0, left = 0;
                    if (g_mouse_step <= 15) {
                        dx = 3; dy = 8;
                    } else {
                        int phase = (g_mouse_step - 16) % 5;
                        left = (phase == 0 || phase == 2) ? 1 : 0;
                    }
                    mouse_packet(dx, dy, left, 0);
                    (void)0;
                } else if (g_type_str && g_type_str[g_type_pos] && kbd_eligible) {
                    if (half == 0) {
                        cur_c = g_type_str[g_type_pos++];
                        SDL_Keycode k = (cur_c == '\n') ? SDLK_RETURN
                                                       : (SDL_Keycode)(unsigned char)cur_c;
                        cur_mc = torch_matrix_code(k);
                        if (cur_mc) kbd_enqueue(cur_mc);
                        (void)0;
                        half = 1;
                    } else {
                        if (cur_mc) kbd_enqueue(cur_mc | 0x80);
                        (void)0;
                        half = 0;
                    }
                } else if (0) {}
            }
        }
#endif
    }
#ifdef USE_SDL
    if (shot_path) dump_ppm(shot_path);
#endif
    (void)verbose;
    return 0;
}

/* --- The big instruction step.  Returns 0 on illegal/halt. ---
 * 6800/6303 opcode table.  We cover the basics + 6303-specific.       */
static int cpu_step(void) {
    uint16_t pc0 = cpu.pc;
    uint8_t op = fetch8();
    switch (op) {
    /* === 0x0X: misc === */
    case 0x01:  /* NOP */                                        break;
    case 0x06: { /* TAP -- A -> CCR (CCR bits 4-0 from A) */
        cpu.cc = (cpu.cc & 0xC0) | (cpu.a & 0x3F);               break; }
    case 0x07:  /* TPA -- CCR -> A */
        cpu.a = cpu.cc | 0xC0;                                   break;
    case 0x08: { /* INX */
        cpu.ix++; cpu.cc = (cpu.cc & ~CC_Z) | (cpu.ix == 0 ? CC_Z : 0); break; }
    case 0x09: { /* DEX */
        cpu.ix--; cpu.cc = (cpu.cc & ~CC_Z) | (cpu.ix == 0 ? CC_Z : 0); break; }
    case 0x0A:  /* CLV */ cpu.cc &= ~CC_V; break;
    case 0x0B:  /* SEV */ cpu.cc |= CC_V; break;
    case 0x0C:  /* CLC */ cpu.cc &= ~CC_C; break;
    case 0x0D:  /* SEC */ cpu.cc |= CC_C; break;
    case 0x0E:  /* CLI */ cpu.cc &= ~CC_I; break;
    case 0x0F:  /* SEI */ cpu.cc |= CC_I; break;

    case 0x10:  /* SBA -- A = A - B */
        cpu.a = alu_sub8(cpu.a, cpu.b, 0);                       break;
    case 0x11: { /* CBA -- A - B (compare) */
        alu_sub8(cpu.a, cpu.b, 0);                               break; }
    case 0x16:  /* TAB */
        cpu.b = cpu.a; set_nz_8(cpu.b); cpu.cc &= ~CC_V;         break;
    case 0x17:  /* TBA */
        cpu.a = cpu.b; set_nz_8(cpu.a); cpu.cc &= ~CC_V;         break;
    case 0x19:  /* DAA -- decimal adjust */ {
        int c = cpu.cc & CC_C, h = cpu.cc & CC_H;
        uint8_t a = cpu.a, lsn = a & 0x0F, msn = a >> 4;
        uint8_t corr = 0;
        if (h || lsn > 9)              corr |= 0x06;
        if (c || msn > 9 || (msn >= 9 && lsn > 9)) corr |= 0x60;
        uint16_t r = a + corr;
        cpu.cc = (cpu.cc & ~(CC_N|CC_Z|CC_C))
               | ((r & 0x80) ? CC_N : 0)
               | (((uint8_t)r == 0) ? CC_Z : 0)
               | ((r > 0xFF || c) ? CC_C : 0);
        cpu.a = r & 0xFF;                                        break; }
    case 0x1A:  /* SLP -- sleep until interrupt */                break;
    case 0x1B:  /* ABA -- A = A + B */
        cpu.a = alu_add8(cpu.a, cpu.b, 0);                       break;

    /* === 0x20-0x2F: branches (relative, 8-bit signed offset) === */
    case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25:
    case 0x26: case 0x27: case 0x28: case 0x29: case 0x2A: case 0x2B:
    case 0x2C: case 0x2D: case 0x2E: case 0x2F: {
        int8_t off = (int8_t)fetch8();
        int take = 0;
        switch (op) {
        case 0x20: take = 1; break;                       /* BRA */
        case 0x21: take = 0; break;                       /* BRN */
        case 0x22: take = !(cpu.cc & (CC_C|CC_Z)); break; /* BHI */
        case 0x23: take =  (cpu.cc & (CC_C|CC_Z)); break; /* BLS */
        case 0x24: take = !(cpu.cc & CC_C); break;        /* BCC/BHS */
        case 0x25: take =  (cpu.cc & CC_C); break;        /* BCS/BLO */
        case 0x26: take = !(cpu.cc & CC_Z); break;        /* BNE */
        case 0x27: take =  (cpu.cc & CC_Z); break;        /* BEQ */
        case 0x28: take = !(cpu.cc & CC_V); break;        /* BVC */
        case 0x29: take =  (cpu.cc & CC_V); break;        /* BVS */
        case 0x2A: take = !(cpu.cc & CC_N); break;        /* BPL */
        case 0x2B: take =  (cpu.cc & CC_N); break;        /* BMI */
        case 0x2C: take = !((cpu.cc & CC_N) ^ ((cpu.cc & CC_V)<<2 & CC_N)); break; /* BGE */
        case 0x2D: take =  ((cpu.cc & CC_N) ^ ((cpu.cc & CC_V)<<2 & CC_N)); break; /* BLT */
        case 0x2E: { int n = !!(cpu.cc & CC_N), v = !!(cpu.cc & CC_V);
                     take = !(cpu.cc & CC_Z) && (n == v); break; }    /* BGT */
        case 0x2F: { int n = !!(cpu.cc & CC_N), v = !!(cpu.cc & CC_V);
                     take = (cpu.cc & CC_Z) || (n != v); break; }     /* BLE */
        }
        if (take) cpu.pc = (cpu.pc + off) & 0xFFFF;
        break; }

    /* === 0x3X: stack/return/etc. === */
    case 0x30:  /* TSX */ cpu.ix = (cpu.sp + 1) & 0xFFFF;         break;
    case 0x31:  /* INS */ cpu.sp++;                                break;
    case 0x32:  /* PULA */ cpu.a = pull8();                        break;
    case 0x33:  /* PULB */ cpu.b = pull8();                        break;
    case 0x34:  /* DES */ cpu.sp--;                                break;
    case 0x35:  /* TXS */ cpu.sp = (cpu.ix - 1) & 0xFFFF;          break;
    case 0x36:  /* PSHA */ push8(cpu.a);                           break;
    case 0x37:  /* PSHB */ push8(cpu.b);                           break;
    case 0x38: { /* PULX */ uint16_t hi = pull8(); cpu.ix = (hi<<8) | pull8(); break; }
    case 0x39:  /* RTS */ cpu.pc = pull16();                       break;
    case 0x3A: { /* ABX */ cpu.ix = (cpu.ix + cpu.b) & 0xFFFF;     break; }
    case 0x3B: { /* RTI */ cpu.cc = pull8(); cpu.b = pull8(); cpu.a = pull8();
                 uint16_t hi = pull8(); cpu.ix = (hi<<8) | pull8();
                 cpu.pc = pull16(); break; }
    case 0x3C: { /* PSHX */ push8(cpu.ix & 0xFF); push8(cpu.ix >> 8); break; }
    case 0x3D: { /* MUL */
        uint16_t r = cpu.a * cpu.b;
        cpu.a = r >> 8; cpu.b = r & 0xFF;
        cpu.cc = (cpu.cc & ~CC_C) | (cpu.b & 0x80 ? CC_C : 0);    break; }
    case 0x3E:  /* WAI */ /* wait for interrupt -- stop emulator */ return 0;
    case 0x3F: { /* SWI */ cpu_interrupt(0xFFFA);                 break; }

    /* === 0x4X: ops on A (inherent) === */
    case 0x40: cpu.a = (uint8_t)-cpu.a; set_nz_8(cpu.a); /* NEGA */
               cpu.cc = (cpu.cc & ~(CC_V|CC_C)) | (cpu.a == 0x80 ? CC_V : 0) | (cpu.a == 0 ? 0 : CC_C); break;
    case 0x43: cpu.a = ~cpu.a; set_nz_8(cpu.a);                    /* COMA */
               cpu.cc = (cpu.cc & ~CC_V) | CC_C; break;
    case 0x44: cpu.cc = (cpu.cc & ~CC_C) | (cpu.a & 1 ? CC_C : 0); /* LSRA */
               cpu.a >>= 1; set_nz_8(cpu.a);
               cpu.cc = (cpu.cc & ~CC_V) | (((cpu.cc & CC_N)>>3) ^ ((cpu.cc & CC_C)<<1) ? CC_V : 0); break;
    case 0x46: { /* RORA */
        int c = cpu.cc & CC_C;
        cpu.cc = (cpu.cc & ~CC_C) | (cpu.a & 1 ? CC_C : 0);
        cpu.a = (cpu.a >> 1) | (c ? 0x80 : 0); set_nz_8(cpu.a);   break; }
    case 0x47: { /* ASRA */
        cpu.cc = (cpu.cc & ~CC_C) | (cpu.a & 1 ? CC_C : 0);
        cpu.a = (cpu.a & 0x80) | (cpu.a >> 1); set_nz_8(cpu.a);   break; }
    case 0x48: { /* ASLA */
        cpu.cc = (cpu.cc & ~CC_C) | (cpu.a & 0x80 ? CC_C : 0);
        cpu.a <<= 1; set_nz_8(cpu.a);                              break; }
    case 0x49: { /* ROLA */
        int c = cpu.cc & CC_C;
        cpu.cc = (cpu.cc & ~CC_C) | (cpu.a & 0x80 ? CC_C : 0);
        cpu.a = (cpu.a << 1) | (c ? 1 : 0); set_nz_8(cpu.a);      break; }
    case 0x4A: cpu.a--; set_nz_8(cpu.a);                          /* DECA */
               cpu.cc = (cpu.cc & ~CC_V) | (cpu.a == 0x7F ? CC_V : 0); break;
    case 0x4C: cpu.a++; set_nz_8(cpu.a);                          /* INCA */
               cpu.cc = (cpu.cc & ~CC_V) | (cpu.a == 0x80 ? CC_V : 0); break;
    case 0x4D: { /* TSTA */ uint8_t r = cpu.a; set_nz_8(r); cpu.cc &= ~(CC_V|CC_C); break; }
    case 0x4F: cpu.a = 0; cpu.cc = (cpu.cc & ~(CC_N|CC_V|CC_C)) | CC_Z; /* CLRA */ break;

    /* === 0x5X: ops on B (mirror of 0x4X) === */
    case 0x50: cpu.b = (uint8_t)-cpu.b; set_nz_8(cpu.b);          /* NEGB */
               cpu.cc = (cpu.cc & ~(CC_V|CC_C)) | (cpu.b == 0x80 ? CC_V : 0) | (cpu.b == 0 ? 0 : CC_C); break;
    case 0x53: cpu.b = ~cpu.b; set_nz_8(cpu.b);                   /* COMB */
               cpu.cc = (cpu.cc & ~CC_V) | CC_C; break;
    case 0x54: cpu.cc = (cpu.cc & ~CC_C) | (cpu.b & 1 ? CC_C : 0); /* LSRB */
               cpu.b >>= 1; set_nz_8(cpu.b); break;
    case 0x56: { /* RORB */
        int c = cpu.cc & CC_C;
        cpu.cc = (cpu.cc & ~CC_C) | (cpu.b & 1 ? CC_C : 0);
        cpu.b = (cpu.b >> 1) | (c ? 0x80 : 0); set_nz_8(cpu.b); break; }
    case 0x57: { /* ASRB */
        cpu.cc = (cpu.cc & ~CC_C) | (cpu.b & 1 ? CC_C : 0);
        cpu.b = (cpu.b & 0x80) | (cpu.b >> 1); set_nz_8(cpu.b); break; }
    case 0x58: { /* ASLB */
        cpu.cc = (cpu.cc & ~CC_C) | (cpu.b & 0x80 ? CC_C : 0);
        cpu.b <<= 1; set_nz_8(cpu.b); break; }
    case 0x59: { /* ROLB */
        int c = cpu.cc & CC_C;
        cpu.cc = (cpu.cc & ~CC_C) | (cpu.b & 0x80 ? CC_C : 0);
        cpu.b = (cpu.b << 1) | (c ? 1 : 0); set_nz_8(cpu.b); break; }
    case 0x5A: cpu.b--; set_nz_8(cpu.b);                          /* DECB */
               cpu.cc = (cpu.cc & ~CC_V) | (cpu.b == 0x7F ? CC_V : 0); break;
    case 0x5C: cpu.b++; set_nz_8(cpu.b);                          /* INCB */
               cpu.cc = (cpu.cc & ~CC_V) | (cpu.b == 0x80 ? CC_V : 0); break;
    case 0x5D: { uint8_t r = cpu.b; set_nz_8(r); cpu.cc &= ~(CC_V|CC_C); break; }/* TSTB */
    case 0x5F: cpu.b = 0; cpu.cc = (cpu.cc & ~(CC_N|CC_V|CC_C)) | CC_Z; /* CLRB */ break;

    /* === 0x6X: indexed memory ops, 0x7X: extended === */
    case 0x60: case 0x70: { /* NEG */
        uint16_t a = (op==0x60) ? am_indexed() : am_ext();
        uint8_t v = mem_read(a), r = -v;
        cpu.cc = (cpu.cc & ~(CC_N|CC_Z|CC_V|CC_C))
               | (r & 0x80 ? CC_N : 0) | (r==0 ? CC_Z : 0)
               | (v==0x80 ? CC_V : 0) | (v==0 ? 0 : CC_C);
        mem_write(a, r); break; }
    case 0x63: case 0x73: { /* COM */
        uint16_t a = (op==0x63) ? am_indexed() : am_ext();
        uint8_t r = ~mem_read(a);
        set_nz_8(r); cpu.cc = (cpu.cc & ~CC_V) | CC_C;
        mem_write(a, r); break; }
    case 0x64: case 0x74: { /* LSR */
        uint16_t a = (op==0x64) ? am_indexed() : am_ext();
        uint8_t v = mem_read(a);
        cpu.cc = (cpu.cc & ~CC_C) | (v & 1 ? CC_C : 0);
        v >>= 1; set_nz_8(v); mem_write(a, v); break; }
    case 0x66: case 0x76: { /* ROR */
        uint16_t a = (op==0x66) ? am_indexed() : am_ext();
        int c = cpu.cc & CC_C;
        uint8_t v = mem_read(a);
        cpu.cc = (cpu.cc & ~CC_C) | (v & 1 ? CC_C : 0);
        v = (v >> 1) | (c ? 0x80 : 0); set_nz_8(v);
        mem_write(a, v); break; }
    case 0x67: case 0x77: { /* ASR */
        uint16_t a = (op==0x67) ? am_indexed() : am_ext();
        uint8_t v = mem_read(a);
        cpu.cc = (cpu.cc & ~CC_C) | (v & 1 ? CC_C : 0);
        v = (v & 0x80) | (v >> 1); set_nz_8(v); mem_write(a, v); break; }
    case 0x68: case 0x78: { /* ASL */
        uint16_t a = (op==0x68) ? am_indexed() : am_ext();
        uint8_t v = mem_read(a);
        cpu.cc = (cpu.cc & ~CC_C) | (v & 0x80 ? CC_C : 0);
        v <<= 1; set_nz_8(v); mem_write(a, v); break; }
    case 0x69: case 0x79: { /* ROL */
        uint16_t a = (op==0x69) ? am_indexed() : am_ext();
        int c = cpu.cc & CC_C;
        uint8_t v = mem_read(a);
        cpu.cc = (cpu.cc & ~CC_C) | (v & 0x80 ? CC_C : 0);
        v = (v << 1) | (c ? 1 : 0); set_nz_8(v); mem_write(a, v); break; }
    case 0x6A: case 0x7A: { /* DEC */
        uint16_t a = (op==0x6A) ? am_indexed() : am_ext();
        uint8_t v = mem_read(a) - 1; set_nz_8(v);
        cpu.cc = (cpu.cc & ~CC_V) | (v == 0x7F ? CC_V : 0);
        mem_write(a, v); break; }
    case 0x6C: case 0x7C: { /* INC */
        uint16_t a = (op==0x6C) ? am_indexed() : am_ext();
        uint8_t v = mem_read(a) + 1; set_nz_8(v);
        cpu.cc = (cpu.cc & ~CC_V) | (v == 0x80 ? CC_V : 0);
        mem_write(a, v); break; }
    case 0x6D: case 0x7D: { /* TST */
        uint16_t a = (op==0x6D) ? am_indexed() : am_ext();
        uint8_t v = mem_read(a); set_nz_8(v); cpu.cc &= ~(CC_V|CC_C); break; }
    case 0x6E: case 0x7E:   /* JMP */
        cpu.pc = (op==0x6E) ? am_indexed() : am_ext(); break;
    case 0x6F: case 0x7F: { /* CLR */
        uint16_t a = (op==0x6F) ? am_indexed() : am_ext();
        mem_write(a, 0);
        cpu.cc = (cpu.cc & ~(CC_N|CC_V|CC_C)) | CC_Z; break; }

    /* === 0x8X imm, 0x9X dir, 0xAX idx, 0xBX ext on A === */
    case 0x80: cpu.a = alu_sub8(cpu.a, op_imm8(), 0); break;       /* SUBA imm */
    case 0x90: cpu.a = alu_sub8(cpu.a, op_dir8(), 0); break;       /* SUBA dir */
    case 0xA0: cpu.a = alu_sub8(cpu.a, op_idx8(), 0); break;       /* SUBA idx */
    case 0xB0: cpu.a = alu_sub8(cpu.a, op_ext8(), 0); break;       /* SUBA ext */
    case 0x81: alu_sub8(cpu.a, op_imm8(), 0); break;               /* CMPA */
    case 0x91: alu_sub8(cpu.a, op_dir8(), 0); break;
    case 0xA1: alu_sub8(cpu.a, op_idx8(), 0); break;
    case 0xB1: alu_sub8(cpu.a, op_ext8(), 0); break;
    case 0x82: cpu.a = alu_sub8(cpu.a, op_imm8(), 1); break;       /* SBCA */
    case 0x92: cpu.a = alu_sub8(cpu.a, op_dir8(), 1); break;
    case 0xA2: cpu.a = alu_sub8(cpu.a, op_idx8(), 1); break;
    case 0xB2: cpu.a = alu_sub8(cpu.a, op_ext8(), 1); break;
    case 0x84: cpu.a = alu_and8(cpu.a, op_imm8()); break;          /* ANDA */
    case 0x94: cpu.a = alu_and8(cpu.a, op_dir8()); break;
    case 0xA4: cpu.a = alu_and8(cpu.a, op_idx8()); break;
    case 0xB4: cpu.a = alu_and8(cpu.a, op_ext8()); break;
    case 0x85: alu_and8(cpu.a, op_imm8()); break;                   /* BITA */
    case 0x95: alu_and8(cpu.a, op_dir8()); break;
    case 0xA5: alu_and8(cpu.a, op_idx8()); break;
    case 0xB5: alu_and8(cpu.a, op_ext8()); break;
    case 0x86: cpu.a = op_imm8(); set_nz_8(cpu.a); cpu.cc &= ~CC_V; break; /* LDAA */
    case 0x96: cpu.a = op_dir8(); set_nz_8(cpu.a); cpu.cc &= ~CC_V; break;
    case 0xA6: cpu.a = op_idx8(); set_nz_8(cpu.a); cpu.cc &= ~CC_V; break;
    case 0xB6: cpu.a = op_ext8(); set_nz_8(cpu.a); cpu.cc &= ~CC_V; break;
    case 0x97: { uint16_t a = am_direct(); mem_write(a, cpu.a); set_nz_8(cpu.a); cpu.cc &= ~CC_V; break; } /* STAA dir */
    case 0xA7: { uint16_t a = am_indexed(); mem_write(a, cpu.a); set_nz_8(cpu.a); cpu.cc &= ~CC_V; break; }
    case 0xB7: { uint16_t a = am_ext(); mem_write(a, cpu.a); set_nz_8(cpu.a); cpu.cc &= ~CC_V; break; }
    case 0x88: cpu.a = alu_eor8(cpu.a, op_imm8()); break;          /* EORA */
    case 0x98: cpu.a = alu_eor8(cpu.a, op_dir8()); break;
    case 0xA8: cpu.a = alu_eor8(cpu.a, op_idx8()); break;
    case 0xB8: cpu.a = alu_eor8(cpu.a, op_ext8()); break;
    case 0x89: cpu.a = alu_add8(cpu.a, op_imm8(), 1); break;       /* ADCA */
    case 0x99: cpu.a = alu_add8(cpu.a, op_dir8(), 1); break;
    case 0xA9: cpu.a = alu_add8(cpu.a, op_idx8(), 1); break;
    case 0xB9: cpu.a = alu_add8(cpu.a, op_ext8(), 1); break;
    case 0x8A: cpu.a = alu_or8(cpu.a, op_imm8()); break;           /* ORAA */
    case 0x9A: cpu.a = alu_or8(cpu.a, op_dir8()); break;
    case 0xAA: cpu.a = alu_or8(cpu.a, op_idx8()); break;
    case 0xBA: cpu.a = alu_or8(cpu.a, op_ext8()); break;
    case 0x8B: cpu.a = alu_add8(cpu.a, op_imm8(), 0); break;       /* ADDA */
    case 0x9B: cpu.a = alu_add8(cpu.a, op_dir8(), 0); break;
    case 0xAB: cpu.a = alu_add8(cpu.a, op_idx8(), 0); break;
    case 0xBB: cpu.a = alu_add8(cpu.a, op_ext8(), 0); break;

    case 0x8C: { /* CPX imm16 */
        uint16_t v = op_imm16(), r = cpu.ix - v;
        set_nz_16(r);
        cpu.cc = (cpu.cc & ~(CC_V|CC_C))
               | (((cpu.ix ^ v) & (cpu.ix ^ r) & 0x8000) ? CC_V : 0)
               | (cpu.ix < v ? CC_C : 0); break; }
    case 0x9C: { uint16_t v = op_dir16(), r = cpu.ix - v; set_nz_16(r);
        cpu.cc = (cpu.cc & ~(CC_V|CC_C))
               | (((cpu.ix ^ v) & (cpu.ix ^ r) & 0x8000) ? CC_V : 0)
               | (cpu.ix < v ? CC_C : 0); break; }
    case 0xAC: { uint16_t v = op_idx16(), r = cpu.ix - v; set_nz_16(r);
        cpu.cc = (cpu.cc & ~(CC_V|CC_C))
               | (((cpu.ix ^ v) & (cpu.ix ^ r) & 0x8000) ? CC_V : 0)
               | (cpu.ix < v ? CC_C : 0); break; }
    case 0xBC: { uint16_t v = op_ext16(), r = cpu.ix - v; set_nz_16(r);
        cpu.cc = (cpu.cc & ~(CC_V|CC_C))
               | (((cpu.ix ^ v) & (cpu.ix ^ r) & 0x8000) ? CC_V : 0)
               | (cpu.ix < v ? CC_C : 0); break; }

    case 0x8D: { /* BSR */ int8_t off = (int8_t)fetch8();
        push16(cpu.pc); cpu.pc = (cpu.pc + off) & 0xFFFF; break; }
    case 0x9D: { /* JSR dir */ uint16_t a = am_direct(); push16(cpu.pc); cpu.pc = a; break; }
    case 0xAD: { /* JSR idx */ uint16_t a = am_indexed(); push16(cpu.pc); cpu.pc = a; break; }
    case 0xBD: { /* JSR ext */ uint16_t a = am_ext(); push16(cpu.pc); cpu.pc = a; break; }

    case 0x8E: cpu.sp = op_imm16(); set_nz_16(cpu.sp); cpu.cc &= ~CC_V; break; /* LDS */
    case 0x9E: cpu.sp = op_dir16(); set_nz_16(cpu.sp); cpu.cc &= ~CC_V; break;
    case 0xAE: cpu.sp = op_idx16(); set_nz_16(cpu.sp); cpu.cc &= ~CC_V; break;
    case 0xBE: cpu.sp = op_ext16(); set_nz_16(cpu.sp); cpu.cc &= ~CC_V; break;
    case 0x9F: { uint16_t a = am_direct(); mem_write16(a, cpu.sp); set_nz_16(cpu.sp); cpu.cc &= ~CC_V; break; } /* STS */
    case 0xAF: { uint16_t a = am_indexed(); mem_write16(a, cpu.sp); set_nz_16(cpu.sp); cpu.cc &= ~CC_V; break; }
    case 0xBF: { uint16_t a = am_ext(); mem_write16(a, cpu.sp); set_nz_16(cpu.sp); cpu.cc &= ~CC_V; break; }

    /* === Same set for B accumulator (0xC0-0xFF) === */
    case 0xC0: cpu.b = alu_sub8(cpu.b, op_imm8(), 0); break;       /* SUBB */
    case 0xD0: cpu.b = alu_sub8(cpu.b, op_dir8(), 0); break;
    case 0xE0: cpu.b = alu_sub8(cpu.b, op_idx8(), 0); break;
    case 0xF0: cpu.b = alu_sub8(cpu.b, op_ext8(), 0); break;
    case 0xC1: alu_sub8(cpu.b, op_imm8(), 0); break;               /* CMPB */
    case 0xD1: alu_sub8(cpu.b, op_dir8(), 0); break;
    case 0xE1: alu_sub8(cpu.b, op_idx8(), 0); break;
    case 0xF1: alu_sub8(cpu.b, op_ext8(), 0); break;
    case 0xC2: cpu.b = alu_sub8(cpu.b, op_imm8(), 1); break;       /* SBCB */
    case 0xD2: cpu.b = alu_sub8(cpu.b, op_dir8(), 1); break;
    case 0xE2: cpu.b = alu_sub8(cpu.b, op_idx8(), 1); break;
    case 0xF2: cpu.b = alu_sub8(cpu.b, op_ext8(), 1); break;
    case 0xC4: cpu.b = alu_and8(cpu.b, op_imm8()); break;          /* ANDB */
    case 0xD4: cpu.b = alu_and8(cpu.b, op_dir8()); break;
    case 0xE4: cpu.b = alu_and8(cpu.b, op_idx8()); break;
    case 0xF4: cpu.b = alu_and8(cpu.b, op_ext8()); break;
    case 0xC5: alu_and8(cpu.b, op_imm8()); break;                   /* BITB */
    case 0xD5: alu_and8(cpu.b, op_dir8()); break;
    case 0xE5: alu_and8(cpu.b, op_idx8()); break;
    case 0xF5: alu_and8(cpu.b, op_ext8()); break;
    case 0xC6: cpu.b = op_imm8(); set_nz_8(cpu.b); cpu.cc &= ~CC_V; break; /* LDAB */
    case 0xD6: cpu.b = op_dir8(); set_nz_8(cpu.b); cpu.cc &= ~CC_V; break;
    case 0xE6: cpu.b = op_idx8(); set_nz_8(cpu.b); cpu.cc &= ~CC_V; break;
    case 0xF6: cpu.b = op_ext8(); set_nz_8(cpu.b); cpu.cc &= ~CC_V; break;
    case 0xD7: { uint16_t a = am_direct(); mem_write(a, cpu.b); set_nz_8(cpu.b); cpu.cc &= ~CC_V; break; } /* STAB */
    case 0xE7: { uint16_t a = am_indexed(); mem_write(a, cpu.b); set_nz_8(cpu.b); cpu.cc &= ~CC_V; break; }
    case 0xF7: { uint16_t a = am_ext(); mem_write(a, cpu.b); set_nz_8(cpu.b); cpu.cc &= ~CC_V; break; }
    case 0xC8: cpu.b = alu_eor8(cpu.b, op_imm8()); break;          /* EORB */
    case 0xD8: cpu.b = alu_eor8(cpu.b, op_dir8()); break;
    case 0xE8: cpu.b = alu_eor8(cpu.b, op_idx8()); break;
    case 0xF8: cpu.b = alu_eor8(cpu.b, op_ext8()); break;
    case 0xC9: cpu.b = alu_add8(cpu.b, op_imm8(), 1); break;       /* ADCB */
    case 0xD9: cpu.b = alu_add8(cpu.b, op_dir8(), 1); break;
    case 0xE9: cpu.b = alu_add8(cpu.b, op_idx8(), 1); break;
    case 0xF9: cpu.b = alu_add8(cpu.b, op_ext8(), 1); break;
    case 0xCA: cpu.b = alu_or8(cpu.b, op_imm8()); break;           /* ORAB */
    case 0xDA: cpu.b = alu_or8(cpu.b, op_dir8()); break;
    case 0xEA: cpu.b = alu_or8(cpu.b, op_idx8()); break;
    case 0xFA: cpu.b = alu_or8(cpu.b, op_ext8()); break;
    case 0xCB: cpu.b = alu_add8(cpu.b, op_imm8(), 0); break;       /* ADDB */
    case 0xDB: cpu.b = alu_add8(cpu.b, op_dir8(), 0); break;
    case 0xEB: cpu.b = alu_add8(cpu.b, op_idx8(), 0); break;
    case 0xFB: cpu.b = alu_add8(cpu.b, op_ext8(), 0); break;

    /* === 16-bit (D = A:B) ops === */
    case 0xC3: { /* ADDD imm */ uint16_t r = D_get() + op_imm16();
        set_nz_16(r); D_set(r); break; }
    case 0xD3: { uint16_t r = D_get() + op_dir16(); set_nz_16(r); D_set(r); break; }
    case 0xE3: { uint16_t r = D_get() + op_idx16(); set_nz_16(r); D_set(r); break; }
    case 0xF3: { uint16_t r = D_get() + op_ext16(); set_nz_16(r); D_set(r); break; }
    case 0x83: { /* SUBD imm */ uint16_t r = D_get() - op_imm16();
        set_nz_16(r); D_set(r); break; }
    case 0x93: { uint16_t r = D_get() - op_dir16(); set_nz_16(r); D_set(r); break; }
    case 0xA3: { uint16_t r = D_get() - op_idx16(); set_nz_16(r); D_set(r); break; }
    case 0xB3: { uint16_t r = D_get() - op_ext16(); set_nz_16(r); D_set(r); break; }
    case 0xCC: { /* LDD imm16 */ uint16_t v = op_imm16(); D_set(v); set_nz_16(v); cpu.cc &= ~CC_V; break; }
    case 0xDC: { uint16_t v = op_dir16(); D_set(v); set_nz_16(v); cpu.cc &= ~CC_V; break; }
    case 0xEC: { uint16_t v = op_idx16(); D_set(v); set_nz_16(v); cpu.cc &= ~CC_V; break; }
    case 0xFC: { uint16_t v = op_ext16(); D_set(v); set_nz_16(v); cpu.cc &= ~CC_V; break; }
    case 0xDD: { uint16_t a = am_direct(); mem_write16(a, D_get()); set_nz_16(D_get()); cpu.cc &= ~CC_V; break; } /* STD */
    case 0xED: { uint16_t a = am_indexed(); mem_write16(a, D_get()); set_nz_16(D_get()); cpu.cc &= ~CC_V; break; }
    case 0xFD: { uint16_t a = am_ext(); mem_write16(a, D_get()); set_nz_16(D_get()); cpu.cc &= ~CC_V; break; }
    case 0xCE: cpu.ix = op_imm16(); set_nz_16(cpu.ix); cpu.cc &= ~CC_V; break;  /* LDX */
    case 0xDE: cpu.ix = op_dir16(); set_nz_16(cpu.ix); cpu.cc &= ~CC_V; break;
    case 0xEE: cpu.ix = op_idx16(); set_nz_16(cpu.ix); cpu.cc &= ~CC_V; break;
    case 0xFE: cpu.ix = op_ext16(); set_nz_16(cpu.ix); cpu.cc &= ~CC_V; break;
    case 0xDF: { uint16_t a = am_direct(); mem_write16(a, cpu.ix); set_nz_16(cpu.ix); cpu.cc &= ~CC_V; break; } /* STX */
    case 0xEF: { uint16_t a = am_indexed(); mem_write16(a, cpu.ix); set_nz_16(cpu.ix); cpu.cc &= ~CC_V; break; }
    case 0xFF: { uint16_t a = am_ext(); mem_write16(a, cpu.ix); set_nz_16(cpu.ix); cpu.cc &= ~CC_V; break; }

    /* === 6303-specific: AIM/OIM/EIM/TIM (memory bit ops) ===
     * 18 nn aa  AIM #nn,(aa)      direct
     * 68 nn ff  AIM #nn,ff,X      indexed
     * 78 nn ahi alo  AIM #nn, ext  -- doesn't exist; AIM is direct/indexed only
     */
    case 0x61: { /* AIM imm,idx */
        uint8_t mask = fetch8(); uint16_t a = am_indexed();
        uint8_t v = mem_read(a) & mask; set_nz_8(v); cpu.cc &= ~CC_V;
        mem_write(a, v); break; }
    case 0x71: { /* AIM imm,dir */
        uint8_t mask = fetch8(); uint16_t a = am_direct();
        uint8_t v = mem_read(a) & mask; set_nz_8(v); cpu.cc &= ~CC_V;
        mem_write(a, v); break; }
    case 0x62: { /* OIM imm,idx */
        uint8_t mask = fetch8(); uint16_t a = am_indexed();
        uint8_t v = mem_read(a) | mask; set_nz_8(v); cpu.cc &= ~CC_V;
        mem_write(a, v); break; }
    case 0x72: { /* OIM imm,dir */
        uint8_t mask = fetch8(); uint16_t a = am_direct();
        uint8_t v = mem_read(a) | mask; set_nz_8(v); cpu.cc &= ~CC_V;
        mem_write(a, v); break; }
    case 0x65: { /* EIM imm,idx */
        uint8_t mask = fetch8(); uint16_t a = am_indexed();
        uint8_t v = mem_read(a) ^ mask; set_nz_8(v); cpu.cc &= ~CC_V;
        mem_write(a, v); break; }
    case 0x75: { /* EIM imm,dir */
        uint8_t mask = fetch8(); uint16_t a = am_direct();
        uint8_t v = mem_read(a) ^ mask; set_nz_8(v); cpu.cc &= ~CC_V;
        mem_write(a, v); break; }
    case 0x6B: { /* TIM imm,idx */
        uint8_t mask = fetch8(); uint16_t a = am_indexed();
        uint8_t v = mem_read(a) & mask; set_nz_8(v); cpu.cc &= ~CC_V;
        break; }
    case 0x7B: { /* TIM imm,dir */
        uint8_t mask = fetch8(); uint16_t a = am_direct();
        uint8_t v = mem_read(a) & mask; set_nz_8(v); cpu.cc &= ~CC_V;
        break; }
    case 0x18:  /* XGDX -- swap D and X */ {
        uint16_t d = D_get(); D_set(cpu.ix); cpu.ix = d; break; }

    default:
        (void)0;
        return 0;
    }
    return 1;
}
