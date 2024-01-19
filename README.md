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
[there](datasheets\SAM-D5x-E5x-Family-Data-Sheet-DS60001507.pdf).

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

[5 pin din electrical midi specs](datasheets\MCP3004-MCP3008-Data-Sheet-DS20001295.pdf)
says ut is a UART serial bus plus power. Fortunately most microcontroler now
support UART.

Only four wires out of five are in use for GND, 3V3, UART RX and TX.
[UART](https://en.wikipedia.org/wiki/Universal_asynchronous_receiver-transmitter)
is a thrifty serial that only use two wires TX and RX. In abscence of a clock
the devices synchronize on the transmited signal. For it to work reliably,
parity bit are also transmited.

<a href="images\Screen_Shot_2020-07-18_at_12.20.08_PM.png"><img src="images\Screen_Shot_2020-07-18_at_12.20.08_PM.png" height="100" /></a>

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

<a href="images\Screenshot 2024-01-19 183334.png"><img src="images\Screenshot 2024-01-19 183334.png" height="100" /></a>

The boards shows up as `Feather M4 Express` in my Digital Audio Workstation and
output midi notes.

<a href="images\Screenshot 2024-01-19 135625.png"><img src="images\Screenshot 2024-01-19 135625.png" height="100" /></a>
<a href="images\Screenshot 2024-01-19 135826.png"><img src="images\Screenshot 2024-01-19 135826.png" height="100" /></a>

This does not prevent the serial loging to work perfectly.

# Talking to the Analog-Digital-Conveter

Analog to digital converters (a.h.a. ADC) measure a voltage from a pin and
output the result as numerical data on a serial bus.

I selected the [MCP3008](datasheets\MCP3004-MCP3008-Data-Sheet-DS20001295.pdf)
which looks popular. There is a
[tutorial available on Adafruit](https://learn.adafruit.com/reading-a-analog-in-and-controlling-audio-volume-with-the-raspberry-pi),
and the [midi_hammer](https://github.com/aleathwick/midi_hammer) project has
successfully integrated it with hall sensors to create a keyboard
([demo](https://youtu.be/tgWXtYCHDI4)).

The MCP3008 measures one of its 8 inputs and writes the result to an SPI serial
bus. As opposito to UART, the SPI serial bus has an explicit clock. Hence it
uses 3 wires names:

- clock
- MOSI (Master output slave input)
- MISO (Master input slave output)

A fourth pin `CS` is used to activate, or deactivate and silence the peripheral.

The tutorial showcase python code, fortunately Adafruit also made
[an Arduino library](https://github.com/adafruit/Adafruit_MCP3008). And it
commes with examples. I just have to set the number of the `CS` pin and it just
works.

Again a huge timesaver.

I can read the 8 channels:

```
522	0	1023	1	1	0	0	0	[150]
522	0	1023	0	0	0	0	0	[151]
522	0	1023	2	1	0	0	1	[152]
522	0	1023	4	3	3	4	8	[153]
```

- The first is the sensor output. In abscence of manetic fields it returns an
  intermediate value. As the MCP3008 is a 10bit converter, the maximal value is
  1023, and 522 is close to the half.
- The second is grounded, and we can read 0 as expected.
- The third is fed with `3V3`, and we can read the maximal value: 1023.
- With manget touching the sensor, I can reach the value of 788 and 254 when the
  magnet is turned by 180°.

<a href="images\Screenshot 2024-01-19 212708.jpg"><img src="images\Screenshot 2024-01-19 212708.jpg" height="100" /></a>
