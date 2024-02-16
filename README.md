# Bandoneon MIDI

The goal of this DIY project is to build a
[bandoneaon](https://en.wikipedia.org/wiki/Bandoneon) MIDI keyboard with
velocity.

# First steps with the microcontroler

_2024/01/18_

Microcontroler development board are a huge time saver. Adafruit has a serie of
small form factor called [Feathers](https://www.adafruit.com/category/946)

These boards can be powered via USB or battery and utilize a voltage regulator
to maintain a 3.3V operation.

[Adafruit Feather M4 Express](https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51)
is built around the
[ATSAMD51J19A](datasheets\SAM-D5x-E5x-Family-Data-Sheet-DS60001507.pdf). It look
like excellent choice because:

- it has plenty (21) input-output pins
- it runs at high clock speed of 120 MHz which will make possible
  [bitbang](https://en.wikipedia.org/wiki/Bit_banging) many analog-to-digital
  SPI busses at 3.6 MHz. I also have 32 floating point hardware.
- it has a single core which keeps the temptation to write concurent code away
- it does not have any extraneous wireless features.

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

Musical Instrument Digital Interface, better known as MIDI, is a communication
protocol used for transmitting musical notes over a serial interface. These
notes are received by a synthesizer, which then produces the corresponding
waveforms.

There are two hardware solutions: 5 pin
[DIN connector](https://fr.wikipedia.org/wiki/Connecteur_DIN) or USB Midi.

## Midi via 5 pin DIN

The
[electrical specifications for the 5-pin DIN MIDI](datasheets\MCP3004-MCP3008-Data-Sheet-DS20001295.pdf),
reveal that it utilizes a UART serial bus along with power. Fortunately, most
modern microcontrollers now support UART.

Out of the five wires, only four are used: these are for Ground (GND), 3.3 volts
(3V3), UART receive (RX), and UART transmit (TX).

[UART (Universal Asynchronous Receiver-Transmitter)](https://en.wikipedia.org/wiki/Universal_asynchronous_receiver-transmitter)
is an efficient serial communication protocol that uses just two wires: TX
(transmit) and RX (receive). Without a dedicated clock signal, devices
synchronize based on the transmitted signal itself. To ensure reliable
communication, parity bits are also transmitted.

<a href="images\Screen_Shot_2020-07-18_at_12.20.08_PM.png"><img src="images\Screen_Shot_2020-07-18_at_12.20.08_PM.png" height="300" /></a>

The
[Arduino Midi library](https://github.com/FortySevenEffects/arduino_midi_library)
works instantly.

I used
[Adafruit's Midi Featherwing](https://learn.adafruit.com/adafruit-midi-featherwing)
for prototyping and it works like a charm.

## Midi via USB

In this setup, MIDI information travels through the same USB cable that is used
for power, programming, and retrieving logs from the board.

To enable MIDI via USB, you need to use
[TinyUSB](https://docs.tinyusb.org/en/latest/index.html), which can be selected
from the `Tools > USB` Stack menu in the `Arduino IDE`. By default, the USB
driver that is built with the program is ArduinoUSB. TinyUSB allows for the
support of additional protocols and enables the device to function as a MIDI
interface.

<a href="images\Screenshot 2024-01-19 183334.png"><img src="images\Screenshot 2024-01-19 183334.png" height="150" /></a>

The boards shows up as `Feather M4 Express` in my Digital Audio Workstation and
output midi notes.

<a href="images\Screenshot 2024-01-19 135625.png"><img src="images\Screenshot 2024-01-19 135625.png" height="150" /></a>
<a href="images\Screenshot 2024-01-19 135826.png"><img src="images\Screenshot 2024-01-19 135826.png" height="150" /></a>

Even with this setup, serial logging still works just fine.

# Talking to the Analog-Digital-Conveter

_2024/01/19_

Analog to Digital Converters, also known as ADCs, measure voltage from a pin and
convert it to numerical data for transmission over a serial bus. Those
components usualy have multiple input channels connected to a unique
[Successive-approximation ADC](https://en.wikipedia.org/wiki/Successive-approximation_ADC)
by a MUX.

I selected the [MCP3008](datasheets\MCP3004-MCP3008-Data-Sheet-DS20001295.pdf)
which looks popular. There is a
[tutorial available on Adafruit](https://learn.adafruit.com/reading-a-analog-in-and-controlling-audio-volume-with-the-raspberry-pi),
and the [midi_hammer](https://github.com/aleathwick/midi_hammer) project has
successfully integrated it with hall sensors to create a keyboard
([demo](https://youtu.be/tgWXtYCHDI4)).

The [MCP3008](datasheets\MCP3004-MCP3008-Data-Sheet-DS20001295.pdf) can measure
voltage from one of its eight inputs and then transmit the result via an SPI
(Serial Peripheral Interface) serial bus. Unlike UART, SPI includes an explicit
clock signal, which means it uses three wires, named:

- Clock
- MOSI (Master output slave input)
- MISO (Master input slave output)

A fourth pin `CS` (Chip Select), is used to activate or deactivate the
peripheral, effectively controlling when it can send or receive data.

The tutorial features Python code, but luckily, Adafruit also offers
[an Arduino library](https://github.com/adafruit/Adafruit_MCP3008), complete
with examples. All I need to do is set the number for the CS pin, and it works
seamlessly.

```c++
adc.begin(13);
```

Again, this is a huge time-saver.

<a href="images\Screenshot 2024-01-19 212708.jpg"><img src="images\Screenshot 2024-01-19 212708.jpg" height="400" /></a>

I can read the 8 channels:

```
522	0	1023	1	1	0	0	0	[150]
522	0	1023	0	0	0	0	0	[151]
522	0	1023	2	1	0	0	1	[152]
522	0	1023	4	3	3	4	8	[153]
```

- The first channel shows the sensor output. In the absence of magnetic fields,
  it returns a mid-range value. Since the MCP3008 is a 10-bit converter, the
  maximum value is 1023, making 522 close to half of this.
- The second is grounded, and as expected, it reads 0.
- The third channel is connected to 3V3 and shows the maximum reading of 1023.
- When a magnet touches the sensor, the readings vary significantly: I can get a
  value of 788 and, interestingly, 254 when the magnet is rotated 180°.

Interestingly, the
[`Adafruit_MCP3008`](https://github.com/adafruit/Adafruit_MCP3008) library is
built on top of [`Adafruit_BusIO`](https://github.com/adafruit/Adafruit_BusIO)
which abstracts SPI communication. This is particularly useful because it can
implement a software bus, exactly what I need for reading from multiple
`MCP3008` units.

The
[library source code](https://github.com/adafruit/Adafruit_BusIO/blob/master/Adafruit_SPIDevice.cpp)
demonstrates how to modify multiple outputs of a given port by writing to the
register:

```c++
BusIO_PortReg *mosiPort = (BusIO_PortReg *)portOutputRegister(digitalPinToPort(mosipin));
BusIO_PortMask *mosiPinMask = digitalPinToBitMask(mosipin);

// Write
*mosiPort = *mosiPort | mosiPinMask;
```

This technique will be useful for reading measurements from multiple devices
simultaneously.

However, the approach to frequency control appears quite basic. It simply sleeps
according to the frequency period, without accounting for the time taken by
operations.

```c++
int bitdelay_us = (1000000 / _freq) / 2;
delayMicroseconds(bitdelay_us);
```

I configured the library to use
[bit banging](https://en.wikipedia.org/wiki/Bit_banging) for the bus by
specifying each pin:

```c++
 adc.begin(10, 12, 11, 13);
```

Remarkably, this setup worked perfectly on the first try.

It runs at `65300 sps` (samples per second), which is half of the specified
throughtput at 3.3 V.

<a href="images\Screenshot 2024-01-27 221911.png"><img src="images\Screenshot 2024-01-27 221911.png" height="400" /></a>

# Bit-banging first step

_2024/01/21_

The source code below implements software SPI. It writes the content of `wbuf`
to the bus while simultaneously reading into `rbuf`. This approach is inspired
by Adafruit's implementation.

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

The ADC protocol uses 17 bus clocks cycle to get a measure.

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

Your analysis of the code's performance and potential improvements is
insightful. Here's a slightly restructured version for enhanced clarity and
conciseness:

This code currently runs at `11.48 ksps`` (kilosamples per second), which
translates to 87 μs (microseconds) per sample. However, there's room for
improvement. According to the MCP3008 datasheet, it can achieve up to 130 ksps
at 3.3V. Here are some potential optimization strategies:

- The most accurate delay function available is delayMicroseconds, but it limits
  the SPI bus to a maximum of 500 KHz. To reach the desired 120 ksps, a bus
  speed of 2.16 MHz is required.
- The current codes transmits 3 bytes (24 bit) per sample, wheras the minimal is
  17 bits, which is 30% more than needed.
- The existing delays don't account for the latency of operations between the
  delays, which adds to the total delay time.
- Using port registers to control output instead of digitalWrite could be more
  efficient.
- Adafruit has implemented a fast path for cases where the MOSI pin sends
  consecutive identical values. This bypasses
  `digitalWrite(mosi_, (wbyte & bit) != 0)`. Surprisingly, this optimization
  increases the rate to 12.65 ksps (79 μs per sample), an improvement of 10%,
  which makes me think `digitalWrite` is very slow.

# Precision counter on ATSAMD51J19A

_2023/01/21_

`ATSAMD51J19A`
[Arduino API implementation by Adafruit](https://github.com/adafruit/ArduinoCore-samd/blob/e2b78cbd3608fd5969d50c550a314db913a1a9e9/cores/arduino/delay.c#L64)
utilizes a hardware cycle counter `CYCCNT` of the `Data Watchpoint and Trace`
(DWT). This method is extremely precise, as the processor clock operates at 120
MHz. It's important to note that the processor clock frequency can be altered,
and the current frequency is accessible through `SystemCoreCloc`k.

```c++
void delayCycles(uint32_t count) {
  // This value takes into account the time spent in the function
  // itself. It has been determined experimentally by comparing the
  // delayCycles() function with the micro().
  constexpr uint32_t experimental_bias = 16;
  const uint32_t start = DWT->CYCCNT - experimental_bias;
  // The DWT->CYCCNT register is a 32 bits counter that counts the
  // number of cycles since the last reset. It is incremented every
  // cycle. It wraps around every 2^32 cycles (~37 secs at 120MHz).
  while (DWT->CYCCNT - start < count) {
  }
}
```

By replacing `delayMicros()` with `delayCycles()`, the sample rate has been
significantly improved to 23.7 ksps, which is an increase of +87%.

It seems that writing values to the port is time consuming. In a benchmark test
using the following code:

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

The results indicate a cycle frequency of 0.70 MHz when compiled with the
`-Ofast` optimization and 0.61 MHz with `-Os` (optimize for size). These
performance levels are insufficient for driving the bus at the desired speed of
2.16 MHz. However, controlling the GPIO directly via the port registers
demonstrates a much faster performance.

Direct control of the GPIO via port registers significantly boosts performance.
Consider this code snippet:

```c++
// Get port and mask from pin number.
auto reg = portOutputRegister(digitalPinToPort(12));
auto mask = digitalPinToBitMask(12);
uint32_t start = micros();
for (int i = 0; i < 1000000; i++) {
  *reg |= mask;
}
```

Running this code yields a frequency of 13.31 MHz when compiled with `-Ofast``.
As expected, a lower frequency of 3.64 MHz is observed for four register
updates.

By replacing `digital*` functions with direct pad manipulation, the sample rate
can be increased to an impressive 90.1 ksps. However, there's still room for
further improvement to reach our target.

# Optimize the IO manipulation further

_2024/01/22_

The operation `*ioreg |= mask;` involves reading, modifying, and then writing
back the result, which can be time-consuming. To address this, the
[`ATSAMD51J19A`](datasheets\SAM-D5x-E5x-Family-Data-Sheet-DS60001507.pdf)
features specialized registers that allow for setting or clearing pins directly
by writing a bitmask to them. This functionality significantly streamlines the
process.

To find the specific member names required, I had to delve into the code. On
Windows, this can be located in the following path:
`\Users\$USER\AppData\Local\Arduino15\packages\adafruit\tools\CMSIS-Atmel\1.2.2\CMSIS\Device\ATMEL\samd51\include\instance\port.h`.
This file is part of the CMSIS (Cortex Microcontroller Software Interface
Standard) package provided by Adafruit for the SAMD51 microcontroller.

For reference, in terms of performance:

- The function call digitalWrite(13, HIGH) consumes 41 cycles.
- Direct register manipulation with \*ioreg |= 0x800000 takes only 10 cycles.

Here is a verbose way to access `OUTSET`.

```c++
digitalPinToPort(13)->OUTSET.reg = digitalPinToBitMask(13);
```

This operation runs in 8 cycles. While it's slower compared to direct register
manipulation, this is mainly because the digital\* functions take additional
time to execute. However, these functions return a constant value, which offers
the possibility of caching for improved efficiency.

```c++
auto port = digitalPinToPort(13); // Once for all.

port->OUTSET.reg = 0x800000;
```

This method is significantly faster, completing the operation in just 3 cycles.

```c++
REG_PORT_OUTSET0 = 0x800000
```

This is the most straight forward, and it runs in 3 cycles, which is 3 times
faster than writing to the IO Register an 13 times faster than the
`digitalWrite`! . With this efficient method of controlling IO, the SPI bitbang
speed now reaches 105.96 ksps, which is quite close to the MCP3008 ADC's
specified limit of 120 ksps at 3.3V. However, we have yet to achieve the maximum
possible speed.

I should be able to squeze by rewriting the throttler. Without throttler the bus
runs at 186.96 ksps, and looks to return correct values.

Profiler code can be found
[here](src\arduino\01_18_blink_sodr_test\01_18_blink_sodr_test.ino)

# Optimizing further data write

_2024/01/23_

For the `clock` or `CS`, the code involves either clearing or setting a bit. In
contrast, the MOSI pin requires setting it to correspond with a specific bit of
the data. Here are several methods for handling the MOSI pin, by increasing
efficiency:

```c++
*(mosi_value ? &REG_PORT_OUTSET0 : &REG_PORT_OUTCLR0) = mosi_mask_;
// 6 cycles

REG_PORT_OUT0 = (REG_PORT_OUT0 & ~mosi_mask_) | (mosi_value << mosi_shift_);
// 15 cycles

*(&REG_PORT_OUTCLR0 + mosi_value) = mosi_mask
// 3 cycles
```

With this new method, we're beginning to observe that the code needs to be
throttled. Otherwise, the bus operates at a speed that exceeds the capabilities
of the `MCP3008`.

For the record I also considered
[bit-banding](https://developer.arm.com/documentation/ddi0439/b/Programmers-Model/Bit-banding),
but the featue is not available on the
[`ATSAMD51J19A`](datasheets\SAM-D5x-E5x-Family-Data-Sheet-DS60001507.pdf).

Target is 130 KSPS = 1/130e3/18 = 4.27350427E-7 = 427 ns

# Precise timing

Up until now, we've been handling time control with a simple busy loop. The
catch is that this method isn't super precise because each cycle includes
multiple instructions, and it can only wait for multiples of this cycle
duration.

To make things more accurate, we started using the `__NOP();` instruction. It's
like a little pause button for the processor, letting it wait for just one
cycle. But here's the kicker: getting the timing spot on with this method means
doing some manual tuning. You have to measure the timing precisely, which can be
a bit of a hassle.

One way to do this measurement is by keeping an eye on `DWT->CYCCNT`. But
remember, this approach can be a bit intrusive.

I ordered
[`AZ Delivey Logic Analyzer`](https://www.az-delivery.de/en/products/saleae-logic-analyzer).
It is an innexpensive device able to capture the binary state of 8 channels at
24 MHz, the data is then analyzed with
[Salae Logic](https://www.saleae.com/downloads/).

<a href="images\Screenshot 2024-01-27 230825.png"><img src="images\Screenshot 2024-01-27 230825.png" height="400" /></a>

That is a delight to use, the software decodes and displays the SPI data as
hexadecimal, and display precise cycle timings for the clock.

<a href="images\Screenshot 2024-01-24 084300.png"><img src="images\Screenshot 2024-01-24 084300.png" /></a>

After adding a few `__NOP();` instructions and dividing the unpacking of `MOSI`
into two parts, one before and one after clearing the `CLK`:

<a href="images\Screenshot 2024-01-24 085408.png"><img src="images\Screenshot 2024-01-24 085408.png" /></a>

The target is `130 KSPS = 1/130e3/18 = 4.27350427E-7 = 427 ns` is reached. Yay!

# Looking at the generated assembly

I was able to identify the compiler used by enabeling `verbose compiler output`
in `Arduino IDE` preferences:

```
$ Arduino15\packages\adafruit\tools\arm-none-eabi-gcc\9-2019q4/bin/arm-none-eabi-g++ --version
arm-none-eabi-g++ (GNU Tools for Arm Embedded Processors 9-2019-q4-major) 9.2.1 20191025 (release) [ARM/arm-9-branch revision 277599]
```

Looking at https://godbolt.org/ with compiler `ARM gcc 9.2.1 (none)` with
`-Ofast`.

Here is a
[decompiled code sample](https://godbolt.org/#z:OYLghAFBqd5QCxAYwPYBMCmBRdBLAF1QCcAaPECAMzwBtMA7AQwFtMQByARg9KtQYEAysib0QXACx8BBAKoBnTAAUAHpwAMvAFYTStJg1DIApACYAQuYukl9ZATwDKjdAGFUtAK4sGEgKykrgAyeAyYAHI%2BAEaYxBIaAGykAA6oCoRODB7evgGp6ZkCoeFRLLHxXEm2mPaOAkIETMQEOT5%2BXIF2mA5Zjc0EJZExcQnJCk0tbXmdtpODYcPlo1WJAJS2qF7EyOwc5gDMYcjeWADUJgduyBP4gpfYJhoAgofHp5gXV83ETACeDyerxeXjCBAOZgA%2BgQzvhgIQxMowgAVVAWQgAWSYCgA1hAwWcbmsTAB2KwvM6Us7ETAEbYMQkKC7%2BbBnLhmA6XcmvEkAESBoMEEOhsLw8KatCRDFR6OECDwVAI%2BMEjOJZKBVOptPpjOZrPZnIO3NJ/JeQPNHKwNHCZ0hkOeQgxmudLtdbs12JYQIIfxSmCtZwAbp4mI56K7BeCoTCAEqoGOYYBcqkAegAVGnLm4zgmmOgzgJaH8zhCALTRQha%2BETOJnCDBgxhz5oBgTM5eVti8L5sFrM5plPe33%2BzBUIMhpsRsHC2MAdwTSaNqYzWZzmDzpYA6sRCJ8yxWYTTqwRa/WJ3RPh2MsBu2de/3B2bLaPFrbIREAPLKCB991/912g6ToNqGF51uYZgMKgKQQcST4HFar4xtgADikLKB%2BMbIpCH5yMibjBDGGj/hAaYQDG86JpSaZrBoqiSFUGgABwaFIcjBH26aZlczJuNEO6jnWGFYX2vKhkwZwfl4BApNJZwAGpiF4nxuPQzRnMRA4WghL42shaHCdhuHIkI2DIsRf6keRlHANRtH0YxLFcEx7GcSuPEmP4fECWOECGaJ4mSdJskwop3ifEItIaQ%2B2mIXpqHoZh2EAJIRBZ/6UlZcYLnZdEMRozEaGYGiuWcXGrp53l4IJflJQFTRnMlDAhQpSmfJpj7Aq8z7WhFZmJVhkIWMlyJ%2BaQZwsNiOLjYGbW/s6lUarCqAXGSGXrRli0UlSZHmIk%2BkDUZeEETGhyHMoFyWEGc1fLyE1TcmW3PJSJpnLO8rhhAGhwV1J4sCkjafFmBIRICLwnNiTJCMoyXPMQvzFqS3JnCkO6zSeIBLYwPhnBiGBA2tyW8sE2CQsEH6buNRPYMEkIABLJShdOrfyRpLV4GRGGcyh5mFym3e207Rly5rbSmKZnIotao6ggZ4Fg%2BYtjQwDbKBAgAHTs2CTEiicOKQiL22RjrMIsIUBts9ttwgCAPz/FmxvQuNoMHKyLB4AoqAW8jguCCbjLe6Lz1lRLdPNOgs7NJ8k0pCkYTAJr208%2BgfPNgokKTbikLjXrGdTdnE3m5n%2BuG8H1u2/D9tXJGM7Ow8E0e17Cjyoq6elxqsnRLQeDIJjRtCyKCBiFQkJ%2BjuGCQsgfzIIH23Q7Dld/BAjswnr40r4XGTjeXdsAtX2tO2cLtu4368H6vCg/S6ICErQ%2BsQHrGyb3gkIQGbGRP%2B7nuv%2BXZuBpgb9G5rCfjcV%2BRJVo%2B0pOLM4zwFBKBaEyAgQ8YRiFoGcBQfpkAKmqvmOOrYzhRzvAyJBnwFCsE%2BGkFoidg6UlzsXSEAs4QIklCiNEmIpoPzvpCYklsaHPzzlnRhYpmFShlOw3Eb9zY8MgWcfgxA6wEjwALDQyYlGrhdhYO81hrB9iRktTUX8m4twIG3fw6JPJ3UuHdJhEpRFsLlAqJUhjPLmP8LyaR%2BiWaeNAfQoR4pESsNlFiCRoCPHbRNEHSkBIMgAC8AF9hbG2JGWo6TEAZIYyE6tYnxMenyJadpQwEB3NEaSmA7RQDEJHP46cwjd3CMAohdTPjBnlug2oPQlTRFQJ4a64VdFrVMthQyQ0RpfRzuneh40wBgFmn03JpoupRIYE08crSimGAUFQOID8BBthrtGN6JSqA512TCfZIpaAKGiJCA86ciDQl%2BK2d2BBSCeI2s6c5MI0zECOf0zRIdJZKHQQQdANsMGGAdgPF5R967vUYG895iKXTQKYLNOgTAu6YGoZqORCi/Yimbo4gWlzrm3OhF7dZTzKyljZMmQlioviPAOHdFRS56UEFLKWP5CLEkwi6T09%2BL9Zn8ysXWWcRzGX13ZboswiQNKqC4KXF0gzDojNGhocagqBH601ebYVmAwmLOdAdYZxlBnEVFXQ/O7dtqajtJ%2Bb8hqXT2q/D%2BJVzoXWOvdXa98rqnUet9V63hLpeW%2ByjCKMIAsTVJUhKlVlMilkwjUcyjS3qqS4ogKor42YIhlTOGYVR2jLDcttc6UN/K0EHgFsqSVrsG7f3ZaY1x7jLpyrooq4NrofleCoC4vAFjq3dt7WY/tbjs2ri4P07MB400vTyWaUtlJo2DWMidC1KarVZxtXwykGas3qMLZYHRECEW0NOWcCtF7KyiprYCOtGTG0GxHRYmVbaFWzqpEOvtA6b1fufWOrME6p1XoIB%2BiJpbwPB0gxE3h3pMD/UBtmq8XZ/REJhMfIEEM4G4zcMoA4BUmJwwRieueMMiNVzcMfdBcdIS71noszu3de5LQxLh/DzFyNLw3mvMN/tBXbxBTbXekL8XQqo4Y15i6/zccvgim%2BGCX50c4dNZ%2BmrT6qhPfOo1LT8yxHhAwH8EDqOKcXpkvTYQ3VLmg9tHTWo8wQB3ovET4axP9jOExeue19WXxI3wjN3Gh4MHCGg0V8bCSBeC9m9zyZkARdqEWiwJad3Gdo6ZrJ7SHAQCKcpaRFwpOhs%2BW9HcJ5lEKpYgVVa2YH5xZC1cCdZh/WagU6lhGmTKWbO2bOYrmBxpcBJONLzbUFAuNi4YSLaYYVuMa1SZrdH0t1GoGIJQ03d0kDrEwaSK0vNnBvt5pLbpAyttC6oA4VBe2ds1JBy7Wm50LJg8aF4rG8MEc41mfw9c8wz13hAKovWzCSeS0iqkeipNA5dGYLgAOwf/jMP9090OqSBHhwjs4yRkcI4AJxQ5R8Dvk2OcdsgOIaxzCNnO1zOP4NzHn73rgUNsTAw3YMvGiQARy8FHZUMIGA%2BH21ST7LX/jqxpPZtg2J6eyYuyknU3OWBuZl9uk0HANi0E4P4XgfgOBaFIKgTgbgEvoK2DsIGHIeCkAIJoJXGwcQgH8P4dWGhJBMQxxyRIJIMdJCSP1lXHBJDq4t9rzgvAFAgA1ebzXSvSBwFgEgNA/0LxkAoDsuP9B4jNBYM7yHNBaAnmIMHiA0R/cVmYMQP4nBTdF%2BaH8D80RtAdLL7wWPbBBAfmWaX8PpAsAlOAG4VBwfuC8CwJNIw4h2/4BpL0f%2BfeteYFUD0Up9fyCCFqP77u/FK8eCwP7opeAWD142FQAwwAFDyWqrOD8foNem/4IIEQYh2BSBkIIKWah/e6EhwYIwKAEv6DwNEYPkANhoJ6hWxOBSwPwqBsQCBeBZY4gdwsB/8fwag6gsgXAGB3BPB2g9AQhFgygKg9A0gMhgDpgOgChCCsghhcCVgkCOk%2Bh5hiC9BuhegGh5gKCRhKg5gBh6DIcJgBhWDlhKgNhPZthdgJBldVc/d28dcOAYEYwnRgBkBkAzgMd1YzB1YuA6woJ6k6xcBCA1tDhJ1eAw8tBgFSBrdJBJB1YMd/AqgSREgIQNAMcDhHCDhkhvdfdSANctcpCg8Q8zcLcTDvczAJCvDA8/Dw8TD/5c8UDJAgA%3D%3D).

We can see there that:

- `SPIArray::transfer` and `SPIArray::select` inlined.
- The loops that unpack the measures are unrolled:
  ```c++
    for (; i < N / 2; i++) {
      const bool bit = (in >> miso_shifts_[i]) & 0x1;
      rbuf[i] = (rbuf[i] << 1) | bit;
    }
  ```

It is tempting to make all the pin informations such as masks and shift part of
the template argument to turn load instructions into immediates, but oveall the
code is well optimized and run fast enough.

# Procesing samples

_2024/01/28_

I recorded the following samples at 2 KHz.
<a href="images\Screenshot 2024-01-31 201740.png"><img src="images\Screenshot 2024-01-31 201740.png" /></a>

We can see that when it is pressed vigorously, the switch travels in 3.8 ms.
Hence it is nice to be able to sustain a sample rate $>= 2 kHz$

Notes are considered pressed and released on a different threshold to avoid
repeated triggering. The velocity in $[1 .. 127]$ is compiled from a linear
mapping of the discrete time serie $(x_i, t_i)$.

When pressed slowly, the instant derivate
$\dot x_i =\dfrac{ x_i - x_{i-1}}{t_i - t_{i-1}}$ is not accurate because of the
combination of integer aliasing and high sample rated. Indeed, there can be
multiple consecutive identical values. We don't necessarily want to keep
previous sample in memory. To workaround this issue I filter the derivative with
a geometrical decreasing weights based of factor $q$:

$$
\dot x_i = \begin{cases}
0 & \text{for } i = -1 \\
( 1 - q )\dot x_{i-1} + q  \dfrac{ x_i - x_{i-1}}{t_i - t_{i-1}} & \text{with } q \in [0, 1] \end{cases}
$$

- With $q = 1$ this is the instant derivate.

- With $q < 1$ it takes into account the past samples by anoly accessing the
  previous sample and the previous derivate.

# Keyboard switches

_2024/01/31_

I plan to use [Wooting 60 gf](https://wooting.io/product/lekker-switch-linear60)
Hall keyboard switches, with 3d printed caps.

<a href="images\Screenshot 2024-02-16 225720.png"><img src="images\Screenshot 2024-02-16 225720.png"  height="300"/></a>

# ADC source impedence

The ADCs use a capacitor to sample the voltage and employ a dichotomy method to
test various voltages in order to determine the measurement value. The input is
characterised by a resistance and a capacity. The source impedence should be low
enough for the capacity to charge during the sampling period.

- [`MCP3008`](datasheets\MCP3004-MCP3008-Data-Sheet-DS20001295.pdf) input
  impedance 1 kΩ, capacity 20pf.
- [`ATSAMD51J19A`](datasheets\SAM-D5x-E5x-Family-Data-Sheet-DS60001507.pdf)
  input impedance 2kΩ, capacity 3pf

Texas instruments provides a technical document on
[ADC Source Impedance](https://www.ti.com/lit/an/spna061/spna061.pdf).

It shows what the input in voltage would look like if the source impedence is to
high.

<a href="images\Screenshot 2024-02-16 172553.png"><img src="images\Screenshot 2024-02-16 172553.png" height="250" /></a>

# First pinted assembly attempts

I printed caps with inner diameter of $4.2mm$ and $4.5mm$ on an Ultimaker MK3
Extended with AA 0.4 headers and transparent PLA. The fist dimension resulted in
a nice and tight adjustment, and the second was too lose.

As second attempt to print a batch of 12 caps with inner diameter of $4.2mm$
resulted in many learning:

<a href="images\Screenshot 2024-02-16 174339.png"><img src="images\Screenshot 2024-02-16 174339.png" height="200"/></a>

- When printed in batch the seams are not very clean. Stop using batch or ensure
  cura
  [print them one at a time](https://all3dp.com/2/cura-print-one-at-a-time-explained/)
  which will require the part to be smaller than the gantry height ($60mm$) and
  to set the number of extruders to 1 in the printers's settings. I should be
  able to print 9 caps per batch.
- This time the adjustement with the switch is too lose. This is confirmed by
  measurement with a caliper, the outer diameter of $5.6mm$, which is $.1mm$
  larger that the previous print with the same inner inner. I should reduce the
  inner diameter to $4.1mm$.
- Assuming I stick to the .4 header, I should try a thicker sleeve by increasing
  the outer diameter from $5.6mm$ to $4.1+.04 * 2 * 2 = 5.7mm$
- The sleve is slightly too short and should be extended by $.1mm$
- It is unconvenient to remove the brims, and I should switch to skirts.

<a href="images\Screenshot from 2024-02-16 18-17-40.png"><img src="images\Screenshot from 2024-02-16 18-17-40.png" height="180"/></a>
→
<a href="images\Screenshot from 2024-02-16 18-17-31.png"><img src="images\Screenshot from 2024-02-16 18-17-31.png" height="180"/></a>

- The triangular infill looks like a radioactive symbol «☢» and Line, Concentric
  or Gyroid should be prefered.
- Once assembled the caps elevates at $11mm$, versus $9mm$ to $10mm$ for the
  original instrument.

<a href="images\Screenshot 2024-02-16 190442.png"><img src="images\Screenshot 2024-02-16 190442.png" height="180"/></a>
<a href="images\Screenshot 2024-02-16 190515.png"><img src="images\Screenshot 2024-02-16 190515.png" height="180"/></a>

The new button has a long sleeve of $6mm$ which is $.04mm$ more than the measure
from the surface of the keyboard and the pressed pushbutton.

<a href="images\Screenshot 2024-02-16 230251.png"><img src="images\Screenshot 2024-02-16 230251.png" height="300"/></a>

# Resistive touch screen

- Article [source](https://cdn-shop.adafruit.com/datasheets/AVR341.pdf)

Pins are X- Y+ X+ Y- https://www.adafruit.com/product/3575

[Datasheet](datasheets\P333+datasheet+TP035W4L2.pdf)

# Expression pedals

https://line6.com/support/topic/23057-why-is-an-m-audio-ex-p-expression-pedal-incompatible/

# PCB

Shift+S

# Related project

- Inside a bandoneon [video 1](https://www.youtube.com/watch?v=u1KrLr4cBY0),
  [2](https://www.youtube.com/watch?v=JJk-qPO4OpM)
- Bandoneón MIDI [video](https://www.youtube.com/watch?v=HgcwFOrgt_w),
  [site](https://hackaday.io/project/170753-bandomidi)
- bandominedoni [video](https://www.youtube.com/watch?v=WzOsrSpjDgU),
  [github](https://github.com/3araht/bandominedoni)
- Bandonberry [github](https://github.com/jebentancour/Bandonberry)

# Further readings

- JS Application to learn the bandoneon layout
  https://github.com/nicokaiser/bandoneon
- [Hall effect keyboard listing](https://www.hlplanet.com/keyboards-hall-effect-switches)
- Lekker keyboard [design note](https://
  .io/post/validation-tests-lekker-update-8) and
  [teardown video](https://www.youtube.com/watch?v=LpKBC1tWXws).
