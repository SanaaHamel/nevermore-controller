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
KLIPPER_ENV_PATH="${HOME}/klippy-env"
MOONRAKER_CONFIG_DIR="${HOME}/printer_data/config"

# Fall back to old directory for configuration as default
if [ ! -d "${MOONRAKER_CONFIG_DIR}" ]; then
    echo "\`$MOONRAKER_CONFIG_DIR\` does not exist. Falling back to \`${HOME}/klipper_config\` as default."
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

# Verify Klipper has been installed, is kiauh-like, and is using Python 3
check_klipper()
{
    if [ "$(sudo systemctl list-units --full -all -t service --no-legend | grep -F "klipper.service")" ]; then
        echo "Klipper service found."
    else
        echo "[ERROR] Klipper service not found, please install Klipper first"
        exit -1
    fi

    if [ ! -f "$KLIPPER_ENV_PATH/bin/pip" ]; then
        echo "[ERROR] '$KLIPPER_ENV_PATH/bin/pip' is not a file"
        echo "[ERROR] This can happen if you didn't install Klipper via Kiauh."
        echo "[ERROR] This is not a supported scenario at this time, pardon."
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

fix_mainsail_os_bluetooth()
{
    BOOT_CONFIG="/boot/config.txt"

    echo -n "Checking for Mainsail OS & BlueTooth issue... "
    if [ -f "$BOOT_CONFIG" ] && grep -q "^\s*dtoverlay=disable-bt\s*\(#.*\)\?$" "$BOOT_CONFIG"; then
        echo "[FAILED]"
        echo "It looks like you're using Mainsail OS and the BlueTooth is currently disabled."
        echo "Do you wish to enable it now?"
        echo "WARNING:  Do not do this if you're using the UART to communicate with your board."
        echo "          This will disable the UART on Raspberry Pis."
        echo "Details:  https://docs-os.mainsail.xyz/faq/enable-bluetooth-on-rpi"

        while true; do
            read -r -p "Enable BlueTooth? [yn]" ANSWER
            case $ANSWER in
                [Yy])
                    echo -n "Enabling hciuart service... "
                    sudo systemctl enable hciuart.service
                    echo "[OK]"
                    echo -n "Enabling bluetooth service... "
                    sudo systemctl enable bluetooth.service
                    echo "[OK]"
                    echo -n "Editing \`$BOOT_CONFIG\`... "
                    sudo sed -i -E "s/^(\s*enable_uart=1\s*(#.*)?)$/#\1/g" "$BOOT_CONFIG"
                    sudo sed -i -E "s/^(\s*dtoverlay=disable-bt\s*(#.*)?)$/#\1/g" "$BOOT_CONFIG"
                    echo "[OK]"
                    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                    echo "!!!! REBOOT REQUIRED TO TAKE EFFECT !!!!"
                    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                    break;;
                [Nn])
                    echo "BlueTooth is required for this module. Exiting."
                    exit -1;;
                *) echo "Enable BlueTooth? [yn]";;
            esac
        done
    else
        echo "[OK]"
    fi
}

fix_python2()
{
    KLIPPER_ENV_BACKUP_PATH="$KLIPPER_ENV_PATH-backup-python2"

    echo -n "Checking for Python 3... "
    if ! "$KLIPPER_ENV_PATH/bin/pip" --version | grep "(python 3\(\.[0-9]\+\)*)" >> /dev/null; then
        echo "[FAILED]"
        echo "Klipper appears to be using python 2. This module requires Python 3.7+."
        echo "Do you wish to try to upgrade to Python 3?"
        echo ""
        echo "WARNING:  This will uninstall any dependencies that aren't required by Klipper."
        echo "          You may need to re-run install scripts for other Klipper extensions."

        while true; do
            read -r -p "Upgrade installation to Python 3? [yn]" ANSWER
            case $ANSWER in
                [Yy])
                    echo -n "Installing Python 3... "
                    sudo apt-get install python3-dev python3-matplotlib
                    echo "[OK]"

                    echo -n "Backing up existing \`$KLIPPER_ENV_PATH\` to \`$KLIPPER_ENV_BACKUP_PATH\`... "
                    mv "$KLIPPER_ENV_PATH" "$KLIPPER_ENV_BACKUP_PATH"
                    echo "[OK]"

                    echo -n "Setting up python3 env... "
                    if ! virtualenv -p python3 "$KLIPPER_ENV_PATH" &&
                         "$KLIPPER_ENV_PATH/bin/pip" install -r "$KLIPPER_PATH/scripts/klippy-requirements.txt"; then
                        echo "[ERROR]"
                        echo -n "Reverting env change... "
                        rm -r "$KLIPPER_ENV_PATH" || true
                        mv "$KLIPPER_ENV_BACKUP_PATH" "$KLIPPER_ENV_PATH"
                        echo "[OK]"
                        exit -1
                    fi

                    echo "[OK]"
                    break;;

                [Nn])
                    echo "Python 3.7+ is required for this module. Exiting."
                    exit -1;;

                *) echo "Upgrade installation to Python 3? [yn]";;
            esac
        done
    else
        echo "[OK]"
    fi
}

# Link extension to Klipper
link_extension()
{
    echo -n "Linking extension to Klipper... "
    ln -sf "${ROOT_DIR}/klipper/nevermore.py" "${KLIPPER_PATH}/klippy/extras/nevermore.py"
    echo "[OK]"
    echo "Installing python dependencies... "
    "$KLIPPER_ENV_PATH/bin/pip" install bleak janus typing_extensions
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
    fix_mainsail_os_bluetooth
    fix_python2
    link_extension
    add_updater
else
    uninstall
fi
start_klipper
