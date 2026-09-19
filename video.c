#include "video.h"

#include <stdint.h>
#include <string.h>

#ifdef USE_SDL
#include <SDL2/SDL.h>
#endif

/*
 * Motorola 6845E CRTC state.
 */
static uint8_t g_crtc_regs[18];
static uint8_t g_crtc_idx = 0;

static int g_crtc_in_vsync = 0;

static int g_crtc_update_strobe = 0;
static int g_crtc_strobe_reads = 0;

static unsigned int g_crtc_vsync_pending = 0;

uint8_t video_crtc_status(void)
{
    if (++g_crtc_strobe_reads >= 600) {
        g_crtc_strobe_reads = 0;
        g_crtc_update_strobe ^= 1;
    }

    return (g_crtc_in_vsync ? 0x80 : 0x00)
         | (g_crtc_update_strobe ? 0x20 : 0x00);
}

uint8_t video_crtc_read_data(void)
{
    if (g_crtc_idx < 18)
        return g_crtc_regs[g_crtc_idx];

    return 0;
}

void video_crtc_select(uint8_t value)
{
    g_crtc_idx = value & 0x1F;
}

void video_crtc_write_data(uint8_t value)
{
    if (g_crtc_idx < 18)
        g_crtc_regs[g_crtc_idx] = value;
}

void video_crtc_tick(void)
{
#ifdef USE_SDL
    static Uint64 last_counter = 0;
    static Uint64 accumulator = 0;

    Uint64 now = SDL_GetPerformanceCounter();
    Uint64 freq = SDL_GetPerformanceFrequency();
    Uint64 frame_ticks = freq / 50;

    if (last_counter == 0) {
        last_counter = now;
        return;
    }

    accumulator += now - last_counter;
    last_counter = now;

    /*
     * Generate one VSYNC event for every elapsed 20 ms period.
     * This cannot be missed merely because the main loop wasn't
     * running at the instant VSYNC occurred.
     */
    while (accumulator >= frame_ticks) {
        accumulator -= frame_ticks;
        g_crtc_vsync_pending++;
    }

    /*
     * Approximate the actual VSYNC status level separately.
     * This is only for software reading the CRTC status.
     */
    g_crtc_in_vsync =
        (accumulator < (freq / 1000)) ? 1 : 0;
#endif
}

int video_crtc_in_vsync(void)
{
    return g_crtc_in_vsync;
}

static uint8_t g_palette[16];

typedef enum {
    VIDEO_DISPLAY_PALE_BLUE = 0,
    VIDEO_DISPLAY_DARK_BLUE,
    VIDEO_DISPLAY_NORMAL
} video_display_state_t;

static video_display_state_t g_video_display_state =
    VIDEO_DISPLAY_PALE_BLUE;

static uint16_t g_boot_palette_e0_mask = 0;
static int g_runtime_palette_enabled = 0;
static int g_runtime_palette_programmed = 0;

void video_display_reset(void)
{
    g_video_display_state = VIDEO_DISPLAY_PALE_BLUE;
    g_runtime_palette_enabled = 0;
    g_runtime_palette_programmed = 0;

    memset(g_palette, 0, sizeof(g_palette));
}

int video_display_is_pale_blue(void)
{
    return g_video_display_state == VIDEO_DISPLAY_PALE_BLUE;
}

void video_display_set_dark_blue(void)
{
    g_video_display_state = VIDEO_DISPLAY_DARK_BLUE;
}

int video_display_begin_host_mode(void)
{
    if (g_runtime_palette_enabled)
        return 0;

    g_runtime_palette_enabled = 1;
    g_runtime_palette_programmed = 0;
    g_video_display_state = VIDEO_DISPLAY_NORMAL;

    return 1;
}

uint8_t video_palette_read(uint8_t index)
{
    return g_palette[index & 0x0F];
}

void video_palette_write(uint8_t index, uint8_t value)
{
    index &= 0x0F;

    g_palette[index] = value;

    /*
     * During startup the initial palette writes are retained but do not
     * expose VRAM.  Once host-driven video begins, palette writes become
     * genuine runtime palette programming.
     */
    if (g_runtime_palette_enabled) {
        g_runtime_palette_programmed = 1;
        g_video_display_state = VIDEO_DISPLAY_NORMAL;
    }
}

static uint32_t palette_to_rgb(uint8_t value)
{
    uint8_t rgb = (uint8_t)~value;

    unsigned r3 = (rgb >> 5) & 0x07;
    unsigned g3 = (rgb >> 2) & 0x07;
    unsigned b2 = rgb & 0x03;

    unsigned r = (r3 * 255U) / 7U;
    unsigned g = (g3 * 255U) / 7U;
    unsigned b = (b2 * 255U) / 3U;

    return 0xFF000000U |
           (r << 16) |
           (g << 8) |
           b;
}

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



void video_render_framebuffer(const uint8_t *vram,
                              uint32_t *framebuffer)
{
    const int STRIDE = 256;
    const int BYTES_PER_ROW = 180;

    uint32_t colors[4];

    get_display_colours(colors);

    for (int y = 0; y < VIDEO_FB_HEIGHT; y++) {
        uint32_t base = (uint32_t)y * STRIDE + 2;

        for (int xb = 0; xb < BYTES_PER_ROW; xb++) {
            uint8_t v =
                vram[(base + (uint32_t)xb) & 0xFFFFu];

            int x = xb * 4;

            framebuffer[y * VIDEO_FB_WIDTH + x + 0] =
                colors[(v >> 6) & 3];

            framebuffer[y * VIDEO_FB_WIDTH + x + 1] =
                colors[(v >> 4) & 3];

            framebuffer[y * VIDEO_FB_WIDTH + x + 2] =
                colors[(v >> 2) & 3];

            framebuffer[y * VIDEO_FB_WIDTH + x + 3] =
                colors[v & 3];
        }
    }
}

int video_crtc_take_vsync(void)
{
    if (g_crtc_vsync_pending == 0)
        return 0;

    g_crtc_vsync_pending--;
    return 1;
}
