#!/bin/bash

# The RP2040 device I'm working on doesn't have SWCLK/SWDIO pins.
# Kill me. Please.

# bail on unset vars (u)
set -u

if [ "$EUID" -eq 0 ]; then
  echo "Don't run this as root."
  exit 1
fi

SRC=$1
DST=$2

while :; do
  sleep 3
  if [ -f "$SRC" ]; then
    if [ -d "$DST" ]; then
      cp "$SRC" "$DST"
      echo "COPIED"
    fi
  fi
done
