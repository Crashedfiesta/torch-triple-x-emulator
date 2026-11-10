#ifndef TORCH_DISK_MANAGER_H
#define TORCH_DISK_MANAGER_H
#include <SDL2/SDL.h>
/* All calls run on the SDL/main thread. */
int disk_manager_init(void);
void disk_manager_place_beside(SDL_Window *emulator);
void disk_manager_input_begin(void);
int disk_manager_event(SDL_Event *event); /* 1 = consumed */
void disk_manager_input_end(void);
void disk_manager_render(void);
void disk_manager_toggle(void);
void disk_manager_shutdown(void);
#endif
