#ifndef TRIPLEX_PTM_H
#define TRIPLEX_PTM_H

#include <stdint.h>

/*
 * Motorola 6840 Programmable Timer Module emulation.
 *
 * The Torch service processor maps the PTM at $0100-$0107.
 * Address decoding remains in triplex.c; this module deals
 * only with PTM register and timer behaviour.
 */

/*
 * Advance the PTM timers by the supplied number of emulated cycles.
 */
void ptm_tick(int cycles);

/*
 * Return non-zero if the PTM is currently requesting IRQ1.
 */
int ptm_irq1_pending(void);

/*
 * Read/write a PTM register.
 *
 * 'off' is the PTM-relative register offset $00-$07,
 * rather than the complete service-processor address
 * $0100-$0107.
 */
uint8_t ptm_read(uint16_t off);
void ptm_write(uint16_t off, uint8_t val);

#endif