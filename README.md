# First steps with the microcontroler

*2024/01/18*

Microcontroler development board are a huge time saver. Adafruit has a serie of
small form factor called [Feathers](https://www.adafruit.com/category/946)

It can either be powered either by usb or by a battery, and operates a 3.3V
provided by a voltage regulator on the board.

[Adafruit Feather M4 Express](https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51)
looks to be a goot fit because:

- it has plenty (21) input-output pins
- runs at high clock speed (120 MHz) which will make possible
  [bit bang](https://en.wikipedia.org/wiki/Bit_banging) many analog-to-digital
  SPI busses at 3.6 MHz
- it has a single core which keeps the temptation to write concurent code away
- it does not have any unnecessary wireless connectivity

The ATSAMD51J19A controler documentation is available
[there](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ProductDocuments/DataSheets/SAM-D5x-E5x-Family-Data-Sheet-DS60001507.pdf).

The first thing to do it to check the bootloader version.

The bootloader is the very first program that runs on the processor. This
program is stored at the beginning of the embedded flash, mapped at address 0x0
(See `9.2 Physical Memory Map` and `25.6.2 Memory Organization`). It is usually
quite small and responsible to load the main program and start its execution. On
micro controlers they provide a way to update the main program.

Adafruit uses a
[fork of Microsoft UF2 bootloader](https://github.com/adafruit/uf2-samdx1). UF2
bootloader implements Mass Storage Class, that is to say the board is going to
show up as a drive when plugged to USB. The name is a bit confusing because
`UF2` is a file fomat that contains instructions to rewrite the memory on a
device. The file extension contails a serie of memory addresses and content to
be writtent there. By applying this reciepe, this makes it possible to replace
the program by overwriting the memory. This a simple solution that does not
requires any additional harware.

CircuitPython and Arduio workflows are documented. I will stay away from
CircuitPython as I would not like to the the garbage collector add jitter to my
sampling timeseries.

Adafruit and Arduino made such a good job that all the drivers and board
integration in the IDE just works. I had to flash the device twice, but finally
the board led is blinking. Hello word!
