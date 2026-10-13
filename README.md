# CYD_STW

Handheld Wi-Fi CSI (Channel State Information) sensing diagnostic tool for the
Freenove FNK0104N 3.5" Cheap Yellow Display (CYD). Captures per-subcarrier CSI
from the ESP32-S3 Wi-Fi radio, runs a real-time DSP pipeline (amplitude,
moving variance, Hampel noise gate, coarse breathing-rate estimation, movement
spectrogram), and renders a live radar-style UI on the QSPI display. Telemetry
is logged to SD card as CSV.

## Technology Stack

- **MCU framework:** ESP-IDF v5.3 (native, CMake — no Arduino, no PlatformIO)
- **Language:** C (FreeRTOS dual-core task topology)
- **UI:** LVGL 9.3 (`lvgl/lvgl ~9.3.0` via IDF component manager) with
  `espressif/esp_lvgl_port ^2.4.0`
- **DSP:** `espressif/esp-dsp` (vector ops, real FFT, Hann window)
- **CSI:** in-IDF Wi-Fi CSI API (`esp_wifi_set_csi_rx_cb`,
  `CONFIG_ESP_WIFI_CSI_ENABLED`) — the esp-csi repo is not required
- **Storage:** FATFS over SDMMC (4-bit, slot 1), ring-buffer + chunked writer
- **Tooling:** esptool.py over native USB-CDC; helper scripts in `tools/`

## Hardware Dependencies

- **Board:** Freenove FNK0104N 3.5" CYD — ESP32-S3R8 (240 MHz, 8 MB octal
  PSRAM, 4 MB flash on the tested unit; see `sdkconfig.defaults` note)
- **Display:** ST77922 TDDI panel, 320×480 RGB565 over QSPI (no MISO/DC) —
  custom driver in `components/st77922/` (init table ported verbatim from the
  Freenove Arduino class)
- **SD card:** SDMMC 4-bit slot (CLK=5, CMD=4, D0=6, D1=7, D2=2, D3=3) for
  telemetry logging and optional credential provisioning
- **Wi-Fi:** any 2.4 GHz AP as CSI traffic source (the board associates as a
  station; CSI frames flow once traffic does)
- On-board peripherals present but unused by this firmware: ST77922 touch
  (I2C @0x55, deliberately disabled — steals I2C/CPU from the DSP pipeline),
  ES8311 audio codec, expansion I2C header

## Repository Layout

- `main/main.c` — CSI callback → queue → DSP task (core 0) → mailbox → LVGL
  task (core 1, 25 FPS); Hampel filter, breathing-rate extraction, FFT
  spectrogram
- `main/display.c` — LVGL9 glue (PARTIAL mode, native RGB565 + flush byte-swap,
  4-px rounder)
- `main/sd_log.c` — SD telemetry sink + `wifi.cfg` credential provisioning
- `main/radar_bg.c` — radar background bitmap (generated LVGL image array)
- `components/board/include/board_cyd.h` — FNK0104N pinout (single source)
- `components/st77922/` — ST77922 QSPI display driver
- `tools/` — flash / boot-log capture helpers (native USB-CDC, `--no-stub`,
  115200 baud)
- `STATUS.md`, `HANDOVER.md`, `SYSTEM_ARCHITECTURE.md` — project state and
  architecture notes
- `Code_Generated_Image.png` — UI reference render

## Setup & Installation

1. Install ESP-IDF v5.3 (`>=5.3,<6.0`) and activate it:
   ```sh
   . ~/esp/esp-idf/export.sh
   ```
2. Clone and build (component manager fetches LVGL / esp-dsp /
   esp_lvgl_port into `managed_components/` automatically):
   ```sh
   idf.py set-target esp32s3
   idf.py build
   ```
3. Configure Wi-Fi credentials — either at build time
   (`idf.py menuconfig` → *CYD_STW configuration* → SSID/password) or at
   runtime by placing a `wifi.cfg` on the SD card (the firmware writes a
   self-documenting template on first boot and ignores it until filled in):
   ```
   SSID=YourNetworkName
   PASS=YourPassword
   ```
4. Flash over native USB-CDC. Put the board in ROM download mode (hold BOOT,
   tap RST, release BOOT), then:
   ```sh
   bash tools/flash_cyd.sh        # esptool --no-stub, 115200 baud
   ```
   Note: esptool's RTS reset does not boot the app on this board — tap the
   physical RST button after flashing. Capture the boot log with
   `bash tools/capture_boot.sh 90`.

## Usage

On boot the unit associates with the configured AP and starts streaming CSI.
The display shows a radar-style view with the live CSI-derived charts:
per-subcarrier variance, Hampel-filtered movement signal, coarse breathing
rate (BPM + confidence, when a valid 0.1–0.5 Hz rhythm is present), and a
movement spectrogram (real FFT over the filtered variance series). Frame/drop
counters and a periodic heartbeat (`alive t=Ns flush=F tx=T txerr=E`) are
logged to the USB-CDC console.

## Data Formats / Protocols

- **CSI frames:** HT20 LLTF segment, 64 I/Q subcarrier pairs, payload clamped
  to 512 bytes; callback runs in the Wi-Fi driver task (copy-and-push only,
  never blocks).
- **SD telemetry:** `/sdcard/csi_telemetry.csv`, FATFS with long filenames.
  Header: `timestamp_ms,iq_variance,hampel_filtered_val,dropped_frames`,
  appended at CSI frame rate (~15–22 fps) via a 4 KB ring buffer with 2 s
  time-based flush.
- **Credential file:** `/sdcard/wifi.cfg`, `SSID=`/`PASS=` key-value lines.
- **Partitions:** 3 MB factory app + SPIFFS storage (`partitions.csv`), sized
  for the 4 MB flash on tested units.

## Notes

- The CSI frame rate is nominally 15 Hz (observed 15–22 fps); the spectrogram
  frequency axis is labeled from real fs/N spacing — recalibrate if the rate
  is ever measured precisely.
- LVGL framebuffers live in PSRAM; DSP arrays are static `.bss` in internal
  SRAM — the two pools never share an allocation path.
