#ifndef TRIPLEX_RTC_H
#define TRIPLEX_RTC_H

#include <stdint.h>

/*
 * HD146818 real-time clock / CMOS RAM emulation.
 *
 * The Torch maps this device at service-processor addresses
 * $0300-$033F.  Address decoding itself remains in triplex.c;
 * this module deals only with RTC register behaviour.
 */

/*
 * Synchronise the emulated clock registers with the host clock.
 */
void rtc_sync_from_host(void);

/*
 * Advance the emulated RTC timing.
 */
void rtc_tick(int cycles);

/*
 * Read/write an RTC register.
 *
 * 'off' is an RTC-relative register number, $00-$3F,
 * rather than the full Torch address $0300-$033F.
 */
uint8_t rtc_read(uint16_t off);
void rtc_write(uint16_t off, uint8_t val);

/*
 * Store the Torch machine serial number in battery-backed CMOS
 * locations $0E-$11.
 */
void rtc_set_machine_serial(const uint8_t serial[4]);

/*
 * Set the CMOS networking options and construct the Ethernet
 * MAC address from the saved Torch machine serial.
 */
void rtc_network_cmos_init(const uint8_t serial[4]);

#endif
