#include "acia.h"

#include <stdint.h>

/*
 * Motorola 6850 ACIA.
 *
 * On the Torch Triple X this is used for the serial mouse interface.
 *
 * The device is mapped at service-processor addresses:
 *
 *   $0200  status/control
 *   $0201  receive/transmit data
 *
 * We currently model only the receive side needed by Caretaker.
 */
 
 /* --- 6850 ACIA (serial mouse) receive state ---
 * The Torch mouse is a serial mouse on an external 6850 ACIA mapped at SP
 * $0200 (status/control) and $0201 (data).  Its /IRQ drives HD6303 /IRQ1
 * ($FFF8 -> ISR $CC4A).  We model the receive side: 3-byte mouse packets
 * injected from SDL get clocked into the data register, set RDRF, and raise
 * IRQ1; the CARETAKER ISR enqueues them into the shared-VRAM mouse mailbox. */
 
static uint8_t g_acia_ctrl = 0x55;
static uint8_t g_acia_rdr;
static int     g_acia_rdrf;

static uint8_t g_mouse_q[256];
static int     g_mouse_q_head;
static int     g_mouse_q_tail;

static int     g_acia_feed_delay;


/* 6850 status register ($0200 read): bit0 RDRF, bit1 TDRE (always set --
 * transmit never blocks), bit2 DCD=0, bit7 IRQ when RDRF and Rx-IRQ on. */
uint8_t acia_status(void)
{
    uint8_t s = 0x02;

    if (g_acia_rdrf)
        s |= 0x01;

    if (g_acia_rdrf && (g_acia_ctrl & 0x80))
        s |= 0x80;

    return s;
}

void acia_rx_feed(void)
{
    if (g_acia_feed_delay > 0) {
        g_acia_feed_delay--;
        return;
    }

    if (g_acia_rdrf)
        return;

    if (g_mouse_q_head == g_mouse_q_tail)
        return;

    g_acia_rdr = g_mouse_q[g_mouse_q_tail];
    g_mouse_q_tail = (g_mouse_q_tail + 1) & 0xFF;

    g_acia_rdrf = 1;
    g_acia_feed_delay = 2500;
}


uint8_t acia_read_data(void)
{
    g_acia_rdrf = 0;
    return g_acia_rdr;
}


void acia_write_control(uint8_t val)
{
    g_acia_ctrl = val;

    /*
     * Control bits 0-1 = 11 perform a master reset.
     */
    if ((val & 0x03) == 0x03)
        g_acia_rdrf = 0;
}

void acia_write_data(uint8_t val)
{
    (void)val;

    /*
     * SP -> mouse transmit is not currently emulated.
     */
}

int acia_irq_pending(void)
{
    return g_acia_rdrf && (g_acia_ctrl & 0x80);
}



