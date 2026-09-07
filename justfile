default:
    @just --list

# Monitor the Bandolibre MIDI port, reconnecting across reboots
midimon:
    tools/midimon.sh

# Open UART debug console via STLink VCP (resolves the right /dev/ttyACMx automatically)
console:
    tio -b 921600 /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_*

# Stream SWO ITM port 0 output
trace_swo:
    st-trace --clock=96m

# Stream SWO ITM port 0 output (core clock = 16 MHz HSI)
plotjuggler_bridge:
    st-trace --clock=96m | python3 tools/plotjuggler_bridge.py

plotjuggler:
    ~/bin/plotjuggler -l tools/bellow_layout.xml --start_streamer -n