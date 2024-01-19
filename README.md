# First steps with the microcontroler

_2024/01/18_

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

# Emmiting Midi signals

_2024/01/19_

Musical Instrument Digital Interface (a.k.a. midi) is a communication protocol
to send notes over a serial interface. Those notes are received by a synthetizer
that generates a waveform.

There are two hardware solutions 5 pin
[DIN connector](https://fr.wikipedia.org/wiki/Connecteur_DIN) or USB Midi.

## Midi via 5 pin DIN

[5 pin din electrical midi specs](https://www.midi.org/specifications/midi-transports-specifications)
says ut is a UART serial bus plus power. Fortunately most microcontroler now
support UART.

Only four wires out of five are in use for GND, 3V3, UART RX and TX.
[UART](https://en.wikipedia.org/wiki/Universal_asynchronous_receiver-transmitter)
is a thrifty serial that only use two wires TX and RX. In abscence of a clock
the devices synchronize on the transmited signal. For it to work reliably,
parity bit are also transmited.

The
[Arduino Midi library](https://github.com/FortySevenEffects/arduino_midi_library)
works instantly.

I used
[Adafruit's Midi Featherwing](https://learn.adafruit.com/adafruit-midi-featherwing)
for prototyping and it works like a charm.

## Midi via USB

Here the midi information transit throught the same USB chord used pro power,
program and get logs from the board.

It requires use [TinyUSB](https://docs.tinyusb.org/en/latest/index.html) in
`Tools > UsbStack`. By default the USB driver is built with the program is
`ArduinoUSB`. TinyUSB makes possible to support more protocol and act as a Midi
device.

The boards shows up as `Feather M4 Express` in my Digital Audio Workstation and
output midi notes.

This does not prevent the serial loging to work perfectly.
