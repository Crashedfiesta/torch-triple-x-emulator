CC = gcc
CFLAGS = -O2 -Wall -Wno-unused-result -DUSE_SDL -DUSE_M68K $(shell pkg-config --cflags sdl2)
MUSASHI = third_party/musashi
SOFTFLOAT = $(MUSASHI)/softfloat
INCLUDES = -I$(MUSASHI) -I$(SOFTFLOAT)
LIBS = $(shell pkg-config --libs sdl2) -lm

MUSASHI_SRCS = $(MUSASHI)/m68kcpu.c $(MUSASHI)/m68kops.c $(MUSASHI)/m68kdasm.c $(SOFTFLOAT)/softfloat.c

GUI_HEADERS = floppy_media.h floppy_media.inc file_browser.h disk_status.h disk_manager.h third_party/nuklear/nuklear.h third_party/nuklear/nuklear_sdl_renderer.h

all: triplex

triplex: triplex.c rtc.c ptm.c acia.c hd6303_sci.c video.c disk_manager.c file_browser.c $(MUSASHI_SRCS) $(GUI_HEADERS)
	$(CC) $(CFLAGS) -o $@ $(filter %.c,$^) $(INCLUDES) $(LIBS)

clean:
	rm -f triplex

.PHONY: all clean
