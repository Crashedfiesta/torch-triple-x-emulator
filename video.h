#ifndef TRIPLEX_VIDEO_H
#define TRIPLEX_VIDEO_H

#include <stdint.h>


#define VIDEO_FB_WIDTH  720
#define VIDEO_FB_HEIGHT 256


/*
 * Motorola 6845E CRTC.
 *
 * The Triple X maps the CRTC at:
 *
 *   $0400  status / register select
 *   $0401  selected register data
 */

uint8_t video_crtc_status(void);
uint8_t video_crtc_read_data(void);

void video_crtc_select(uint8_t value);
void video_crtc_write_data(uint8_t value);


/*
 * Advance the simplified CRTC timing model by one emulator step.
 */
void video_crtc_tick(void);

/*
 * Return non-zero while the emulated CRTC VSYNC output is active.
 */
int video_crtc_in_vsync(void);


/*
 * Triple X palette RAM at $0500-$050F.
 */
uint8_t video_palette_read(uint8_t index);
void video_palette_write(uint8_t index, uint8_t value);


/*
 * Boot/display-state handling.
 */
void video_display_reset(void);

int video_display_is_pale_blue(void);
void video_display_set_dark_blue(void);

/*
 * Switch from Caretaker startup display handling to normal host-driven
 * video.  Returns non-zero only when the transition actually occurs.
 */
int video_display_begin_host_mode(void);


/*
 * Convert the Triple X VRAM image into a 720x256 ARGB framebuffer.
 *
 * VRAM remains owned by the machine-level emulator.
 */
void video_render_framebuffer(const uint8_t *vram, uint32_t *framebuffer);

int video_crtc_take_vsync(void);
#endif

