#include "rtc.h"

#include <stdio.h>
#include <string.h>
#include <time.h>


/*
 * HD146818 RTC at $0300-$033F.
 *
 * Offsets $00-$09 are date/time registers.
 * $0A-$0D are control/status registers.
 * $0E-$3F are battery-backed CMOS RAM.
 */

static int g_rtc_clock_div = 0;
static int g_rtc_sec_in_emu = 0;
static uint8_t g_rtc_user_ram[0x40];

void rtc_set_machine_serial(const uint8_t serial[4])
{
    memcpy(&g_rtc_user_ram[0x0E], serial, 4);
}

void rtc_network_cmos_init(const uint8_t serial[4])
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
    g_rtc_user_ram[0x17] = serial[1];
    g_rtc_user_ram[0x18] = serial[2];
    g_rtc_user_ram[0x19] = serial[3];

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
void rtc_sync_from_host(void)
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
void rtc_tick(int cycles)
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


uint8_t rtc_read(uint16_t off) {
    off &= 0x3F;
    /*if (off == 0) return g_rtc_sec_in_emu;*/
    /* Register D bit 7 = VRT (valid RAM and time).  Firmware reads it. */
    if (off == 0x0D) return 0x80;
    return g_rtc_user_ram[off];
}

void rtc_write(uint16_t off, uint8_t val)
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
