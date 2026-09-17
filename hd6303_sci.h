#ifndef TRIPLEX_HD6303_SCI_H
#define TRIPLEX_HD6303_SCI_H

#include <stdint.h>

/*
 * HD6303 on-chip Serial Communications Interface.
 *
 * The Triple X uses the SCI for its keyboard input path.
 * The current emulator also sends its Caretaker mouse-compatibility
 * sequence through the same receive queue.
 */

/*
 * Queue one received byte for delivery to the SCI.
 *
 * Returns non-zero on success, zero if the queue is full.
 */
int sci_enqueue(uint8_t value);

/*
 * Return the number of unused bytes in the receive queue.
 */
int sci_queue_free(void);

/*
 * Advance the receive side of the SCI.
 *
 * A queued byte is transferred to RDR when the current byte has
 * been consumed and the serial pacing delay has expired.
 */
void sci_rx_feed(void);

/*
 * Construct the live TRCSR value from the control bits stored by
 * the HD6303 register block.
 */
uint8_t sci_read_trcsr(uint8_t trcsr);

/*
 * Read the Receive Data Register.
 * Reading RDR clears RDRF.
 */
uint8_t sci_read_data(void);

/*
 * Return non-zero when the SCI receive side is requesting an IRQ.
 *
 * The caller supplies TRCSR because the HD6303 register block remains
 * owned by triplex.c.  CPU interrupt masking is also handled there.
 */
int sci_irq_pending(uint8_t trcsr);

#endif