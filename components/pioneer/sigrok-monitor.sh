#!/usr/bin/env bash

FILENAME="${1//.sr/}"
CHANNEL=${2:-"D1"}
# $20 USB Logic Analyzer (Saleae Logic 8 clone)
DRIVER=${3:-"fx2lafw"}

if [ -z "$FILENAME" ]; then
  echo "Usage: $0 <filename>"
  exit 1
fi

# Monitor and timestamp the signals from the Pioneer remote control
while true; do
  sigrok-cli -d "$DRIVER" -C "$CHANNEL" -t "${CHANNEL}=e" -w --time 300 -c samplerate="24 MHz" -o "${FILENAME}.sr" && mv "${FILENAME}.sr" "${FILENAME}-$(date +%s).sr"
  date
done