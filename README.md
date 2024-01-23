# First steps with the microcontroler

_2024/01/18_

Microcontroler development board are a huge time saver. Adafruit has a serie of
small form factor called [Feathers](https://www.adafruit.com/category/946)

These boards can be powered via USB or battery and utilize a voltage regulator
to maintain a 3.3V operation.

[Adafruit Feather M4 Express](https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51)
appears to be an excellent choice because:

- it has plenty (21) input-output pins
- it runs at high clock speed of 120 MHz which will make possible
  [bitbang](https://en.wikipedia.org/wiki/Bit_banging) many analog-to-digital
  SPI busses at 3.6 MHz
- it has a single core which keeps the temptation to write concurent code away
- it does not have any extraneous wireless features.

The ATSAMD51J19A controler documentation is available
[there](datasheets\SAM-D5x-E5x-Family-Data-Sheet-DS60001507.pdf).

The first thing to do it to check the bootloader version.

The bootloader, the initial program running on the processor, resides at the
start of the embedded flash, mapped at address `0x0` (See
`9.2 Physical Memory Map` and `25.6.2 Memory Organization`). Typically compact,
its role is to load and initiate the main program. In microcontrollers, it also
facilitates updating the main program.

Adafruit employs a
[fork of Microsoft UF2 bootloader](https://github.com/adafruit/uf2-samdx1),
which implements the Mass Storage Class. This means the board appears as a drive
when connected via USB. The term 'UF2' refers to a file format containing
instructions for rewriting device memory. The file format specifies a series of
memory addresses and the content to be written there. By following this process,
the program can be replaced by overwriting the memory, a straightforward method
requiring no extra hardware.

While both CircuitPython and Arduino workflows are well-documented, I will avoid
using CircuitPython to prevent the garbage collector from introducing jitter
into my time series sampling.

Thanks to Adafruit and Arduino's excellent work, the integration of drivers and
boards in the IDE is flawless. It took two attempts to flash the device, but now
the board's LED is blinking. Hello world!

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

_2024/01/19_

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

```c++
adc.begin(13);
```

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

Interestingly it Adafruit_MCP3008 is built on top of
[Adafruit_BusIO](https://github.com/adafruit/Adafruit_BusIO) which abstracts SPI
communication. It is able to implement a software bus, which is exaclt what I
plan to do to read from multime MCP3008.

The
[library source code](https://github.com/adafruit/Adafruit_BusIO/blob/master/Adafruit_SPIDevice.cpp)
shows how to modify multiple output of a given port:

```c++
BusIO_PortReg *mosiPort = (BusIO_PortReg *)portOutputRegister(digitalPinToPort(mosipin));
BusIO_PortMask *mosiPinMask = digitalPinToBitMask(mosipin);

// Write
*mosiPort = *mosiPort | mosiPinMask;
```

That will prove usefull to read measures from multiuple devices in parallel.

The frequency control looks minimal. It just sleeps according the the frequency
period without taking into acount the operations taking time.

```
int bitdelay_us = (1000000 / _freq) / 2;
delayMicroseconds(bitdelay_us);
```

It setup the library to [bit bang](https://en.wikipedia.org/wiki/Bit_banging)
the but by setting each pin:

```c++
 adc.begin(10, 12, 11, 13);
```

Again this works on the first attempt.

# Bit-banging first step

_2024/01/21_

The following source implements software SPI. It writes the content of `wbuf` to
the but while reading the content or `rbuf`. It is inspired from Adafruit
implementation.

```c++
  template <unsigned int L>
  void transfer(const std::array<uint8_t, L> &wbuf,
                std::array<uint8_t, L> &rbuf) {
    // Only MSB first is supported.
    int r_idx = 0;
    for (const uint8_t wbyte : wbuf) {
      uint8_t rbyte;
      for (uint8_t bit = 0x80; bit != 0; bit >>= 1) {
        digitalWrite(mosi_, (wbyte & bit) != 0);
        digitalWrite(clk_, HIGH);
        delayMicroseconds(clk_period_us_);
        rbyte = (rbyte << 1) | digitalRead(miso_[i]);

        digitalWrite(clk_, LOW);
        delayMicroseconds(clk_period_us_);
      }
      rbuf[r_idx++] = rbyte[i];
    }
  }
```

The ADC protocol uses 18 bus clock cycle to get a measure.

<table>
  <tr>
   <td>
   </td>
   <td><strong>Bits</strong>
   </td>
   <td><strong>MOSI</strong>
   </td>
   <td><strong>MISO</strong>
   </td>
  </tr>
  <tr>
   <td>Start bit
   </td>
   <td>1
   </td>
   <td>“1”
   </td>
   <td>don’t care
   </td>
  </tr>
  <tr>
   <td>Single / Diff
   </td>
   <td>1
   </td>
   <td>“1”
   </td>
   <td>don’t care
   </td>
  </tr>
  <tr>
   <td>Channel select
   </td>
   <td>3
   </td>
   <td>D0, D1, D2
   </td>
   <td>don’t care
   </td>
  </tr>
  <tr>
   <td>Sample Period
   </td>
   <td>1
   </td>
   <td>don’t care
   </td>
   <td>don’t care
   </td>
  </tr>
  <tr>
   <td>Null bit
   </td>
   <td>1
   </td>
   <td>don’t care
   </td>
   <td>“0”
   </td>
  </tr>
  <tr>
   <td>Sample
   </td>
   <td>10
   </td>
   <td>don’t care
   </td>
   <td>data
   </td>
  </tr>
</table>

This code runs at 11.48 ksps (87 μs/sample).

There is some headroom for improvement, as the
[MCP3008 datasheet](datasheets\MCP3004-MCP3008-Data-Sheet-DS20001295.pdf)
proposes 120 ksps at 3.3V.

Here are the leads:

- Firstly the most accurate delay function is `delayMicroseconds` which can only
  implement an SPI buss of 500 KHz, wheras we need 2.16 MHz to achieve 120ksps
- Then the current codes transmits 3 bytes (24 bit) per sample, wheras the
  minimal is 18bits, which is 33% more than needed.
- The delays do not take into account the latentency of the operation in between
  the `delays` which comes on top of the delays.
- Control the output using ports registers rahter than `digitalWrite`
- There is fastpath implemented by Adafruit possible when the MOSI pin sends
  consecutive identical values, that consist is bypassing the
  `digitalWrite(mosi_, (wbyte & bit) != 0)`. Surprisingly, this optimization
  makes posible ot run at 12.65 ksp (79 μs per sample), which is a improvement
  of 10%.

# Precision counter on ATSAMD51J19A

_2023/01/21_

`ATSAMD51J19A`
[Arduino API implementation by Adafruit](https://github.com/adafruit/ArduinoCore-samd/blob/e2b78cbd3608fd5969d50c550a314db913a1a9e9/cores/arduino/delay.c#L64)
uses a hardware cycle counter `CYCCNT` of the `Data Watchpoint and Trace`
(a.k.a. DWT). This is exptemely precise has the processor clock runs at 120 MHz.
The processor clock frequency chang be change, and the current value is given by
`SystemCoreClock`.

```c++
void delayCycles(uint32_t count) {
  // This value takes into account the time spent in the function
  // itself. It has been determined experimentally by comparing the
  // delayCycles() function with the micro().
  constexpr uint32_t experimental_bias = 16;
  const uint32_t start = DWT->CYCCNT - experimental_bias;
  while (1) {
    // The DWT->CYCCNT register is a 32 bits counter that counts the
    // number of cycles since the last reset. It is incremented every
    // cycle. It wraps around every 2^32 cycles (~37 secs at 120MHz).
    const uint32_t elapsed = DWT->CYCCNT - start;
    if (elapsed >= count) return;
  }
}
```

Replacing `delayMicros()` with `delayCycles()` improves the sample rate to 23.7
ksps (+87%).

It seems that writing values to the port is time consuming.

```c++
uint32_t start = micros();
for (int i = 0; i < 1000000; i++) {
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  digitalWrite(12, LOW);
  digitalRead(13);
}
uint32_t end = micros();
// Display the cycle frequency.
Serial.print("Cycle frequency: ");
Serial.print(1000000.0 / (end - start));
Serial.println("MHz");
```

This returns 0.70 MHz in `-Ofast` and .61 MHz in `-OSmall`. The low performance
does not make possible to drive the bus at 2.16 MHz.

Controlling the the GPIO via the port is much faster.

```c++
// Get port and mask from pin number.
auto reg = portOutputRegister(digitalPinToPort(12));
auto mask = digitalPinToBitMask(12);
uint32_t start = micros();
for (int i = 0; i < 1000000; i++) {
  *reg |= mask;
}
```

This returns 13.31 MHz `-Ofast`, and without surprise 3.64 MHz for 4 register
update.

Replacing `digital*` with pad manimulation makes possible to reach 90.1 ksps.

# Optimize the IO manipulation further

_2024/01/22_

`*ioreg |= mask;` requires to read modify and write bakc the result. Because
this is time consuming the `ATSAMD51J19A` comes with special register that set
or clear pins when a bit mask is written to them.

I had to read the code to find the member names. Onw Windows it is located in
`\Users\$USER\AppData\Local\Arduino15\packages\adafruit\tools\CMSIS-Atmel\1.2.2\CMSIS\Device\ATMEL\samd51\include\instance\port.h`

For reference:

`digitalWrite(13, HIGH)` takes 41 cycles

`*ioreg |= 0x800000` takes 10 cycles

Here is a verbose way to access `OUTSET`.

```c++
digitalPinToPort(13)->OUTSET.reg = digitalPinToBitMask(13);
```

Runs in 8 cycles, which is slow because the `digital*` takes some time to
execute. However they return a constant which can be cached.

```c++
auto port = digitalPinToPort(13); // Once for all.

port->OUTSET.reg = 0x800000;
```

Is much faster and runs in 3 cycles.

```c++
REG_PORT_OUTSET0 = 0x800000
```

Is the most straight forward, and runs in 3 cycles, which is 3 times faster than
writing to the IO REgister an 13 times faster than the `digitalWrite`! With this
new way to control IO, the SPI bitbang reaches 105.96 ksps which is close to the
ADC specificed limit of 120 ksps at 3.3V. But we are still not running at speed.

I should be able to squeze by avoinding writing and reading the don't care bits
in the protocol, and by rewriting the throttler. Without throttler the bus runs
at 186.96 ksps, and looks to return correct values.

[profiler code here](src\arduino\01_18_blink_sodr_test\01_18_blink_sodr_test.ino)

# Optimizing further

```c++
*(mosi_value ? &REG_PORT_OUTSET0 : &REG_PORT_OUTCLR0) = mosi_mask_;
// 6 cycles

REG_PORT_OUT0 = (REG_PORT_OUT0 & ~mosi_mask_) | (mosi_value << mosi_shift_);
// 15 cycles

*(&REG_PORT_OUTCLR0 + mosi_value) = mosi_mask
// 3 cycles
```

With this new we are stating to see the code requires to be throttled, otherwise
the buss goes too fast for the MPC3008.

# Related project / Further readings

- JS Application to learn the bandoneon layout
  https://github.com/nicokaiser/bandoneon
- Lekker keyboard
  [design note](https://wooting.io/post/validation-tests-lekker-update-8) and
  [teardown video](https://www.youtube.com/watch?v=LpKBC1tWXws).
