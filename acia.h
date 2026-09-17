#ifndef TRIPLEX_ACIA_H
#define TRIPLEX_ACIA_H

#include <stdint.h>

/*
 * Motorola 6850 ACIA emulation.
 *
 * The Torch service processor maps the ACIA at:
 *
 *   $0200  status/control
 *   $0201  receive/transmit data
 *
 * Address decoding remains in triplex.c.
 */

/*
 * Advance the receive side of the ACIA.
 *
 * This clocks queued input bytes into the receive data register.
 */
void acia_rx_feed(void);

/*
 * Return the ACIA status register.
 */
uint8_t acia_status(void);

/*
 * Read the receive data register.
 *
 * Reading the data register clears RDRF.
 */
uint8_t acia_read_data(void);

/*
 * Write the control register.
 */
void acia_write_control(uint8_t val);

/*
 * Write the transmit data register.
 *
 * Transmit is not currently emulated, so this is presently a no-op.
 */
void acia_write_data(uint8_t val);

/*
 * Return non-zero if the ACIA itself is requesting an interrupt.
 *
 * This deliberately does NOT consider the HD6303 CPU interrupt-mask flag.
 */
int acia_irq_pending(void);

#endif

