#!/bin/bash
# CONTROL EXPERIMENT: flash the known-good legacy_cyd PlatformIO build.
# Restores the familiar working UI to prove/disprove current hardware health.
# Set LEGACY_CYD_BUILD_DIR to the legacy build's .pio/build/fnk0104n dir.
set -e
LEGACY_CYD_BUILD_DIR="${LEGACY_CYD_BUILD_DIR:?Set LEGACY_CYD_BUILD_DIR to the legacy_cyd .pio/build/fnk0104n directory}"
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v 'hermes-agent' | paste -sd:)
. ~/esp/esp-idf/export.sh >/dev/null 2>&1
cd "$LEGACY_CYD_BUILD_DIR"
esptool.py --chip esp32s3 -p /dev/ttyACM0 -b 115200 \
  --before no_reset --after no_reset --no-stub \
  write_flash --flash_mode keep --flash_freq keep --flash_size keep \
  0x1000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin
echo CONTROL_FLASH_DONE
