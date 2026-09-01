# Torch Triple X Emulator

An emulator for the Torch Triple X workstation.

Original code by Kokoboi.

Endlessly messed about with by Crashedfiesta.

28/08/26 - First proper issue of code and README.

## Current status

- Boots the Torch Caretaker ROM (v1.2 and v1.3 tested)
- Boots Torch Unix
- OpenTop graphical environment works
- SCSI hard disk emulation
- Keydisk support
- Floppy support
- LANCE Ethernet support (Linux only)
- SDL display/input

## Requirements

- C compiler
- SDL2 development libraries
- Musashi 68000 emulator
- Appropriate Torch ROM and key disk image
- Torch Unix hard disk image

## Building on Linux

Ensure that 3rd party libraries are in place and in the case of 'musashi' are in a folder called 'third_party'.

To make 'triplex' run 'make'.

To remove 'triplex' run 'make clean'.

Note that it will compile in Windows but there is no network support (yet).

## Running

Execute from a terminal window:

./triplex --disk HD00.img --unix-floppy FloppyDisk.img --keydisk torch_KEY.imd --sdl --host --scale 1 --tap tap0

## ROMs and disk images

ROM and operating-system images are not supplied with this repository.
Users must provide their own legally obtained copies.

## Networking

Instructions for tap0 - coming soon.

## Known limitations

Crashes within Unix are quite common. Suspect it could be MMU related but I don't know enough about the MMU.. Be prepared!

Crashes at startup also occur more than you'd expect:

- Stuck on dark blue 'Caretaker' screen - close triplex and try again
- Stuck on pale blue screen - close triplex and try again
- Stuck on booting OpenTop (normally the top border of a window is all that is visible) - close triplex and start again
- 'Error on read' message - click 'OK' or '=' on the window until the three boot icons appear. Click on the boot and press enter to err... boot.

## Credits

Original code by Kokoboi
Fiddled with by Crashedfiesta

This just would not work without the amazing Musashi 68k CPU emulator.
