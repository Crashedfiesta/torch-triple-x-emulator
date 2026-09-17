#include "hd6303_sci.h"

#include <stdint.h>


/*
 * HD6303 on-chip SCI receive state.
 *
 * RDRF is modelled separately from the HD6303's g_io[] register
 * block because it represents live device state rather than simply
 * the last value written to TRCSR.
 */

static uint8_t g_sci_rdr;
static int     g_sci_rdrf;

static uint8_t g_sci_queue[256];
static int     g_sci_queue_head;
static int     g_sci_queue_tail;

static int     g_sci_feed_delay;

int sci_enqueue(uint8_t value)
{
    int next = (g_sci_queue_head + 1) & 0xFF;

    if (next == g_sci_queue_tail)
        return 0;

    g_sci_queue[g_sci_queue_head] = value;
    g_sci_queue_head = next;

    return 1;
}

int sci_queue_free(void)
{
    return (g_sci_queue_tail - g_sci_queue_head - 1) & 0xFF;
}


void sci_rx_feed(void)
{
    if (g_sci_feed_delay > 0) {
        g_sci_feed_delay--;
        return;
    }

    if (g_sci_rdrf)
        return;

    if (g_sci_queue_head == g_sci_queue_tail)
        return;

    g_sci_rdr = g_sci_queue[g_sci_queue_tail];
    g_sci_queue_tail = (g_sci_queue_tail + 1) & 0xFF;

    g_sci_rdrf = 1;
    g_sci_feed_delay = 2500;
}

uint8_t sci_read_trcsr(uint8_t trcsr)
{
    return (trcsr & 0x1F)
         | (g_sci_rdrf ? 0x80 : 0x00)
         | 0x20;
}

uint8_t sci_read_data(void)
{
    g_sci_rdrf = 0;
    return g_sci_rdr;
}

int sci_irq_pending(uint8_t trcsr)
{
    return g_sci_rdrf && (trcsr & 0x10);
}


