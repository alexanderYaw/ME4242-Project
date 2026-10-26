#!/usr/bin/env bash
# Pair a Bluetooth gamepad (Xbox, PlayStation, ...) with this computer so the motor GUI can use it (Linux, BlueZ).
# Unplug the controller's USB cable, then put it in pairing mode: Xbox = hold the small pair button on top until
# the Xbox logo flashes fast; PlayStation = hold PS + Create/Share until the light bar flashes.
# Usage: ./pair_controller.sh            (or ./pair_controller.sh AA:BB:CC:DD:EE:FF to pick a device)
set -u
PATTERN="xbox|controller|gamepad|dualsense|dualshock"
mac=${1:-}
if [ -z "$mac" ]; then
  echo "Scanning for a controller in pairing mode (up to 40 s)..."
  timeout 40 bluetoothctl scan on >/dev/null 2>&1 &
  scan=$!
  for _ in $(seq 1 40); do
    mac=$(bluetoothctl devices | grep -iE "$PATTERN" | head -1 | awk '{print $2}')
    [ -n "$mac" ] && break
    sleep 1
  done
  if [ -z "$mac" ]; then
    kill $scan 2>/dev/null
    echo "No controller found. Is it in pairing mode (logo flashing fast) and the USB cable unplugged?"
    exit 1
  fi
fi
echo "Found $(bluetoothctl devices | grep "$mac" | cut -d' ' -f3-) ($mac); pairing..."
bluetoothctl pair "$mac" || { echo "Pairing failed; put the controller back in pairing mode and try again."; exit 1; }
bluetoothctl trust "$mac" >/dev/null   # reconnects by itself next time: just press the Xbox button
bluetoothctl connect "$mac"
kill ${scan:-} 2>/dev/null
sleep 2
if bluetoothctl info "$mac" | grep -q "Connected: yes"; then
  echo "Connected. In the GUI, press any button on the controller so the browser sees it."
else
  echo "Paired but not connected yet; press the Xbox button to connect."
fi
