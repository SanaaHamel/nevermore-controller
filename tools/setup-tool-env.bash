#!/bin/bash

# Original Authors:
#   Sanaa Hamel (trivial, path/filename modifications)

# bail on cmd err (e), unset vars (u), and errors mid-pipe
set -eu
set -o pipefail

if [ "$EUID" -eq 0 ]; then
  echo "[ERROR] This script must not run as root. Exiting."
  exit 1
fi

# Find ROOT_DIR from the pathname of this script
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$ROOT_DIR/.venv"

if [ ! -d "$VENV_DIR" ]; then
  echo "Setting up \`${VENV_DIR}\`..."
  echo "This may take a bit..."
fi

if [ ! -d "$VENV_DIR" ] && ! virtualenv -p python3 "$VENV_DIR"; then
  echo "[ERR] Failed to create \`${VENV_DIR}\`"
  rm -rf "$VENV_DIR"
  exit 1
fi

if ! "$VENV_DIR/bin/pip" install -r "$ROOT_DIR/requirements.txt" ||
  ! "$VENV_DIR/bin/pip" install -r "$ROOT_DIR/../klipper/requirements.txt" ||
  ! "$VENV_DIR/bin/pip" install git+https://github.com/sanaahamel/serial-flash-py; then
  echo "[ERR] Failed to install dependencies."
  echo "If there's a conflict, try deleting \`${VENV_DIR}\` entirely and re-runnning the script."
  exit 1
fi
