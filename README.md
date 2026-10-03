# Torch Triple X Emulator

An emulator for the Torch Triple X workstation.

This project includes a modified copy of Musashi 4.60 as the Motorola 680x0 CPU emulation core. 
The Musashi sources have been adapted for the Torch Triple X emulator, particularly around MMU and bus-error handling. 
The original Musashi copyright and licence notices are retained in the source files.

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
- Torch specific Musashi 68000 emulator (included in this project)
- Appropriate Torch ROM and key disk image
- Torch Unix hard disk image

## Building on Linux

Ensure that 3rd party libraries are in place, including the Torch specific Musashi code.

To make 'triplex' run 'make'.

To remove 'triplex' run 'make clean'.

Note that the code should compile in Windows but there is no Windows network support (yet).
For best results, use your favourite flavour of Linux. ;)

## Running

Execute 'triplex' from a terminal window. The following options are available:

 - --disk           (the hard disk image to use - uses SCSI ID 0 LUN 0)
 - --disk4          (a second hard disk image to use - uses SCSI ID 1 LUN 0 - the '4' is a long story, just go with it)
 - --unix-floppy    (the regular floppy disk image to use)
 - --key-disk       (the 'key disk' image to use - note this must be in .imd format)
 - --sdl            (use the SDL library to construct the required windows)
 - --host           (create windows on the host system - without this, you won't see much!)
 - --scale          (the scale of the window to use - '1' works best)
 - --tap            (the 'tap' interface that the emulator connects to when talking through the LANCE chips)

For example:

./triplex --disk HD00.img --unix-floppy FloppyDisk.img --keydisk torch_KEY.imd --sdl --host --scale 1 --tap tap0

## ROMs and disk images

ROM, Key Disk and operating system images are not supplied with this repository.
Users must provide their own legally obtained copies.

## Networking

Run the 'torch-network.sh' script before running the emulator. Requires sudo priveleges.
Once in Torch terminal, use 'su' to elevate to root then execute:

ifconfig la0 192.168.2.2

default route add 192.168.2.1 1

Then try and ping '8.8.8.8'. If you get a message that it is alive then network is up and running.

Not available in Windows.

## Known limitations

Crashes within Unix are quite common. Suspect it could be MMU related but I don't know enough about the MMU (I didn't do that code!).. Be prepared!

Crashes at startup also occur more than you'd expect:

- Stuck on dark blue 'Caretaker' screen - close triplex and try again
- Stuck on pale blue screen - close triplex and try again
- Stuck on booting OpenTop (normally the top border of a window is all that is visible) - close triplex and start again
- 'Error on read' message. Click on the '=' button twice to get rid of the error notification. Then, either:
  - click on the 'Go' icon, or
  - click on the boot and press enter to err... boot (you could type 'unix' at this point but that's the default anyway).

## Credits

Original code by Kokoboi.

Endlessly fiddled with by Crashedfiesta (special thanks to Kokoboi for allowing me to do this).

This project makes extensive use of the Musashi project.
