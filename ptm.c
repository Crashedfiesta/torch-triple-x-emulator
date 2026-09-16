#include "ptm.h"

#include <stdint.h>

/*
 * Motorola 6840 Programmable Timer Module.
 *
 * The Triple X maps the PTM at service-processor addresses
 * $0100-$0107.
 *
 * It contains three 16-bit down-counters with shared control
 * registers.  Per the MAME Triple X driver, the PTM IRQ output
 * is merged with the ACIA and RTC IRQ outputs to drive the
 * service processor's /IRQ1 line.
 *
 * Register map:
 *
 *   0 = CR1 or CR3 (selected by CR2 bit 0)
 *   1 = CR2
 *   2 = Timer 1 MSB
 *   3 = Timer 1 LSB
 *   4 = Timer 2 MSB
 *   5 = Timer 2 LSB
 *   6 = Timer 3 MSB
 *   7 = Timer 3 LSB
 *
 * Per-timer control register bits:
 *
 *   bit 0     clock source
 *   bit 1     prescaler enable
 *   bits 2-4  timer operating mode
 *   bit 5     counter output enable
 *   bit 6     interrupt enable
 *   bit 7     timer reset
 *
 * Status register:
 *
 *   bits 0-2  timer interrupt flags
 *   bit 7     composite interrupt flag
 */
 
static uint16_t g_ptm_counter[3] = {
    0xFFFF, 0xFFFF, 0xFFFF
};

static uint16_t g_ptm_latch[3] = {
    0xFFFF, 0xFFFF, 0xFFFF
};

static uint8_t g_ptm_status = 0;

static uint8_t g_ptm_cr[3] = {
    0, 0, 0
};


void ptm_tick(int cycles)
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

int ptm_irq1_pending(void) {
    if ((g_ptm_status & 0x01) && (g_ptm_cr[0] & 0x40)) return 1;
    if ((g_ptm_status & 0x02) && (g_ptm_cr[1] & 0x40)) return 1;
    if ((g_ptm_status & 0x04) && (g_ptm_cr[2] & 0x40)) return 1;
    return 0;
}


uint8_t ptm_read(uint16_t off) {
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

void ptm_write(uint16_t off, uint8_t val) {
    /* MSB write loads the high byte of a latch, LSB write loads low byte
     * AND transfers the latch into the counter.  Simplified: each pair of
     * writes (MSB then LSB) sets both latch and counter.  CR writes go to
     * shared CR1/CR2/CR3 registers. */

    switch (off) {
    case 0: /* CR2 bit 0 selects whether writes to offset 0 go to CR1 or CR3.
             * Per 6840: CR2[0]=0 -> writes to offset 0 are CR3, =1 -> CR1. */
        if (g_ptm_cr[1] & 0x01) 
            g_ptm_cr[0] = val;
        else                    
            g_ptm_cr[2] = val;
        break;
        
    case 1: 
        g_ptm_cr[1] = val; 
        break;
        
    case 2:    
        g_ptm_latch[0] = 
            (g_ptm_latch[0] & 0x00FF) | 
            ((uint16_t)val << 8); 
        break;
        
    case 3: 
        g_ptm_latch[0] = 
            (g_ptm_latch[0] & 0xFF00) | val;
            
        g_ptm_counter[0] = g_ptm_latch[0]; 
        break;
        
    case 4: 
        g_ptm_latch[1] = 
            (g_ptm_latch[1] & 0x00FF) | 
            ((uint16_t)val << 8);
            
    case 5: 
        g_ptm_latch[1] = 
            (g_ptm_latch[1] & 0xFF00) | val;
        g_ptm_counter[1] = g_ptm_latch[1]; 
        break;
            
    case 6: 
        g_ptm_latch[2] = 
            (g_ptm_latch[2] & 0x00FF) | 
            ((uint16_t)val << 8);
        break;
        
    case 7: 
        g_ptm_latch[2] = 
            (g_ptm_latch[2] & 0xFF00) | val;
        g_ptm_counter[2] = g_ptm_latch[2]; 
        break;
    }
}

