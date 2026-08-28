CC = gcc
CFLAGS = -O2 -Wall -Wno-unused-result -DUSE_SDL -DUSE_M68K $(shell pkg-config --cflags sdl2)
MUSASHI = third_party/musashi
SOFTFLOAT = $(MUSASHI)/softfloat
INCLUDES = -I$(MUSASHI) -I$(SOFTFLOAT)
LIBS = $(shell pkg-config --libs sdl2) -lm

MUSASHI_SRCS = $(MUSASHI)/m68kcpu.c $(MUSASHI)/m68kops.c $(MUSASHI)/m68kdasm.c $(SOFTFLOAT)/softfloat.c

all: triplex

triplex: triplex.c $(MUSASHI_SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(INCLUDES) $(LIBS)

clean:
	rm -f triplex

.PHONY: all clean
