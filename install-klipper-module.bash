#!/bin/bash

# Original Authors:
#   Julian Shill (klipper-led_effects)

# Modified By:
#   Sanaa Hamel (trivial, path/filename modifications)

# bail on cmd err (e), unset vars (u), and errors mid-pipe
set -eu
set -o pipefail

if [ "$EUID" -eq 0 ]; then
    echo "[ERROR] This script must not run as root. Exiting."
    exit -1
fi

UNINSTALL=0
KLIPPER_PATH="${HOME}/klipper"
MOONRAKER_CONFIG_DIR="${HOME}/printer_data/config"

# Fall back to old directory for configuration as default
if [ ! -d "${MOONRAKER_CONFIG_DIR}" ]; then
    echo "\"$MOONRAKER_CONFIG_DIR\" does not exist. Falling back to ""${HOME}"/klipper_config" as default."
    MOONRAKER_CONFIG_DIR="${HOME}/klipper_config"
fi

usage(){ echo "Usage: $0 [-k <klipper path>] [-c <configuration path>]" 1>&2; exit 1; }
# Parse command line arguments
while getopts "k:c:uh" arg; do
    case $arg in
        k) KLIPPER_PATH=$OPTARG;;
        c) MOONRAKER_CONFIG_DIR=$OPTARG;;
        u) UNINSTALL=1;;
        h) usage;;
        *) usage;;
    esac
done

# Find ROOT_DIR from the pathname of this script
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Verify Klipper has been installed
check_klipper()
{
    if [ "$(sudo systemctl list-units --full -all -t service --no-legend | grep -F "klipper.service")" ]; then
        echo "Klipper service found."
    else
        echo "[ERROR] Klipper service not found, please install Klipper first"
        exit -1
    fi
}

check_folders()
{
    if [ ! -d "$KLIPPER_PATH/klippy/extras/" ]; then
        echo "[ERROR] Klipper installation not found in directory \"$KLIPPER_PATH\". Exiting"
        exit -1
    fi
    echo "Klipper installation found at $KLIPPER_PATH"

    if [ ! -f "${MOONRAKER_CONFIG_DIR}/moonraker.conf" ]; then
        echo "[ERROR] Moonraker configuration not found in directory \"$MOONRAKER_CONFIG_DIR\". Exiting"
        exit -1
    fi
    echo "Moonraker configuration found at $MOONRAKER_CONFIG_DIR"
}

# Link extension to Klipper
link_extension()
{
    echo -n "Linking extension to Klipper... "
    ln -sf "${ROOT_DIR}/klipper/nevermore.py" "${KLIPPER_PATH}/klippy/extras/nevermore.py"
    echo "[OK]"
    echo "Installing python dependencies... "
    "${HOME}/klippy-env/bin/pip" install bleak janus
}

# Restart moonraker
restart_moonraker()
{
    echo -n "Restarting Moonraker... "
    sudo systemctl restart moonraker
    echo "[OK]"
}

# Add updater to moonraker.conf
add_updater()
{
    echo -e -n "Adding update manager to moonraker.conf... "

    update_section=$(grep -c '\[update_manager nevermore\]' "${MOONRAKER_CONFIG_DIR}"/moonraker.conf || true)
    if [ "${update_section}" -eq 0 ]; then
        echo -e "\n" >> "${MOONRAKER_CONFIG_DIR}/moonraker.conf"
        while read -r line; do
            echo -e "${line}" >> "${MOONRAKER_CONFIG_DIR}/moonraker.conf"
        done < "$ROOT_DIR/klipper/moonraker_update.txt"
        echo -e "\n" >> "${MOONRAKER_CONFIG_DIR}/moonraker.conf"
        echo "[OK]"
        restart_moonraker
        else
        echo -e "[update_manager nevermore] already exists in moonraker.conf [SKIPPED]"
    fi
}

restart_klipper()
{
    echo -n "Restarting Klipper... "
    sudo systemctl restart klipper
    echo "[OK]"
}

start_klipper()
{
    echo -n "Starting Klipper... "
    sudo systemctl start klipper
    echo "[OK]"
}

stop_klipper()
{
    echo -n "Stopping Klipper... "
    sudo systemctl start klipper
    echo "[OK]"
}

uninstall()
{
    if [ -f "${KLIPPER_PATH}/klippy/extras/nevermore.py" ]; then
        echo -n "Uninstalling... "
        rm -f "${KLIPPER_PATH}/klippy/extras/nevermore.py"
        echo "[OK]"
        echo "You can now remove the [update_manager nevermore] section in your moonraker.conf and delete this directory. Also remove all nevermore configurations from your Klipper configuration."
    else
        echo "nevermore.py not found in \"${KLIPPER_PATH}/klippy/extras/\". Is it installed?"
        echo "[FAILED]"
    fi
}

# Run steps
check_klipper
check_folders
stop_klipper
if [[ "$UNINSTALL" != 1 ]]; then
    link_extension
    add_updater
else
    uninstall
fi
start_klipper
