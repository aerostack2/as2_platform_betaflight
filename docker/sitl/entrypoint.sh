#!/bin/sh

# First run: apply sitl.cfg and save it to eeprom.bin. SITL exits after saving,
# and can crash while exiting, so check for the file, not the exit code.
if [ ! -f eeprom.bin ]; then
  betaflight_sitl --config /etc/betaflight/sitl.cfg > /dev/null 2>&1 || true
  [ -f eeprom.bin ] || { echo "SITL did not save eeprom.bin" >&2; exit 1; }
fi

# SITL crashes if simulator packets arrive while it initialises, so start the
# feeder once the MSP port is listening.
betaflight_sitl "$@" &
sitl=$!
trap 'kill $sitl 2>/dev/null' INT TERM
until nc -z 127.0.0.1 5761 2>/dev/null; do sleep 0.2; done
fdm_feed &
echo "Betaflight SITL ready: MSP on tcp://:5761 and :5762"
wait $sitl
