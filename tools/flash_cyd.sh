#!/bin/bash
# Flash CYD_STW to FNK0104N over native USB-CDC (ROM download mode).
# Skill rules: --no-stub (stub dies on this USB path), 115200, --before no_reset
# (board already in download mode; avoid reset strobes kicking it out).
set -e
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v 'hermes-agent' | paste -sd:)
. ~/esp/esp-idf/export.sh >/dev/null 2>&1
cd "$(dirname "$0")/../build"
esptool.py --chip esp32s3 -p /dev/ttyACM0 -b 115200 \
  --before no_reset --after hard_reset --no-stub \
  write_flash @flash_args
echo FLASH_DONE
