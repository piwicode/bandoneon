#!/usr/bin/env bash
# Monitor the Bandolibre MIDI port with aseqdump, reconnecting across reboots.
#
# aseqdump exits (or prints "Port unsubscribed") when the device disappears,
# e.g. on a firmware reboot. This wrapper detects that, then polls the ALSA
# sequencer client list until the device re-enumerates and re-attaches.

set -u

CLIENT="${1:-L\'Atelier du Bandonéon Libre}"
POLL_INTERVAL="${POLL_INTERVAL:-0.5}"

# Print the ALSA port (e.g. "20:0") for the named client, empty if absent.
find_port() {
    aseqdump -l 2>/dev/null | awk -v c="$CLIENT" '
        $0 ~ c { print $1; exit }'
}

wait_for_port() {
    local port
    echo "Waiting for client \"$CLIENT\" to appear..." >&2
    while :; do
        port="$(find_port)"
        if [ -n "$port" ]; then
            echo "$port"
            return 0
        fi
        sleep "$POLL_INTERVAL"
    done
}

while :; do
    PORT="$(wait_for_port)"
    echo "Connecting to $CLIENT on $PORT" >&2

    # Read aseqdump output line by line. When the device is unsubscribed
    # (on reboot the line "Port unsubscribed" is printed) we stop reading
    # and fall back to polling for the device to return.
    while IFS= read -r line; do
        printf '%s\n' "$line"
        case "$line" in
            *"Port unsubscribed"*)
                echo "Port unsubscribed — device went away, reconnecting." >&2
                break
                ;;
        esac
    done < <(aseqdump -p "$PORT" 2>&1)

    # aseqdump may also just exit if the port vanished before subscribing.
    echo "aseqdump stopped, re-scanning." >&2
done
