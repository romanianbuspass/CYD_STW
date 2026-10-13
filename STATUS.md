# CYD_STW — project state (failover handoff doc)

Handheld Wi-Fi CSI sensing diagnostic tool. ESP32-S3 CYD (FNK0104N 3.5"),
ESP-IDF native (NOT PlatformIO — prior CYD work was PlatformIO/Arduino; this
project deliberately starts clean on IDF v5.3 for esp-csi/esp-dsp).

## Stage tracker

- [x] Stage 1 — skeleton: root CMakeLists, sdkconfig.defaults, partitions.csv,
      Kconfig.projbuild, idf_component.yml
- [x] Stage 2 — main.c scaffold: CSI callback -> queue -> DSP task -> LVGL chart
- [x] Stage 3 — board_cyd.h from FNK0104N pinout (skill reference)
- [x] Stage 4 — toolchain + first build: IDF v5.3 @ ~/esp/esp-idf,
      `idf.py build` GREEN (cyd_stw.bin 924 KB, 78% of 4 MB partition free)
- [x] Stage 5 — display bring-up COMPLETE (2026-09-09, visually verified):
      components/st77922 = IDF port of Freenove QSPI class (init table
      verbatim, 40 MHz, IOMUX pins, cs_ena pre/post=2, acquire_bus +
      CS_KEEP_ACTIVE per window); main/display.c = LVGL9 PARTIAL mode,
      2x 320x48 internal DMA SRAM buffers (calloc'd), NATIVE RGB565 +
      byte-swap in flush_cb (LVGL 9.3.0 Kconfig lacks SWAPPED blend
      support — renders zeros), 4-px rounder via LV_EVENT_INVALIDATE_AREA.
      Root causes fixed: CS_KEEP_ACTIVE-without-acquire buslock storm,
      chunk-local px_map OOB indexing (static bands), SWAPPED-render
      compile-out (black screen). Diagnostics removed.
- [-] Stage 5b — touch CANCELLED (2026-09-10): passive CSI monitor; touch
      steals I2C/CPU from DSP. Driver, indev wiring, TOUCH TEST button and
      i2c_probe all removed; I2C bus now completely unused (legacy-driver
      bus-free wedge on cold-boot brown-out eliminated structurally).
      Future input: GPIO0 BOOT button interrupt.
- [x] Stage 5c — SD card data logging COMPLETE (2026-09-10, verified):
      main/sd_log.c — SDMMC 4-bit @/sdcard (slot 1, internal pull-ups;
      pins CLK=5 CMD=4 D0=6 D1=7 D2=2 D3=3 per board JSON — the rotated
      CLK=6/CMD=5/D0=4 variant fails CMD0 with ESP_ERR_TIMEOUT), FATFS
      LFN_STACK (csi_telemetry.csv violates 8.3), wifi.cfg legacy format
      (SSID=/PASS=, template auto-write + reject), ring buffer 4 KB +
      sd_log_task prio 2 → snprintf → 3 KB RAM buffer → chunked
      fwrite/fflush (buffer-full or 2 s). CSV header on empty file;
      verified live rows at ~22 fps, drops=0.
- [x] Stage 6 (core) — Hampel noise gate IMPLEMENTED (2026-09-10): scalar
      iq_variance series, 32-frame raw window (~2 s), median/MAD statistics
      via dsps_addc_f32 + insertion-sort medians (esp-dsp has NO median fn),
      INVERTED decision per operator: inliers clamp to median (baseline
      flattened), outliers pass raw (movement preserved — passive radar).
      Warmup pass-through; sustained >2 s events re-anchor the median
      (documented). Verified dataset: 1776 frames, 0 drops, baseline
      0.5-1.0 vs movement 5-17. Pending: chart overlay, HT-LTF exposure.
- [x] Stage 6b — Coarse breathing-rate extraction IMPLEMENTED (2026-09-13):
      ported ruvnet/RuView `CoarseBreathingExtractor` (2nd-order IIR
      bandpass 0.1-0.5 Hz + zero-crossing BPM + SNR confidence) onto the
      Hampel-filtered iq_variance scalar. NOT the "circular variance on
      wrapped phase" the RuView README implies — that pipeline doesn't
      exist in their shipped code; verified by reading breathing_extractor.rs
      directly. CSI_FRAME_RATE_HZ=15 is a nominal assumption (observed
      15-22 fps); recalibrate if actual rate drifts. 30 s window / 10 s
      min. BPM + confidence surfaced on radar UI, teal 0x00FFCC label
      bottom-right. Build verified GREEN (idf.py build, IDF v5.3, py3.12
      venv), not yet flashed/field-verified against a reference breathing
      rate.
- [ ] Stage 7 — traffic generator for quiet APs (ping/beacon loop)

## Key decisions (do not re-litigate without cause)

- CSI API is in-IDF (`esp_wifi_set_csi_rx_cb`, `CONFIG_ESP_WIFI_CSI_ENABLED`);
  the espressif/esp-csi repo is examples + host tooling, NOT vendored.
- CSI callback runs in the Wi-Fi driver task context: struct-copy payload,
  xQueueSend(tick=0), drop-and-count on full. Never malloc/log/block there.
- DSP arrays = static .bss (internal SRAM). LVGL framebuffers = PSRAM only.
- Queue payload clamp 512 B; LLTF segment = first 128 bytes (64 I/Q pairs).
- Amplitude via esp-dsp vector ops (dsps_mul/add/sqrt). Variance = O(N)
  running sums over a 32-frame ring. Hampel filter is TODO(stage-dsp).
- UI mailbox (len-1 queue, xQueueOverwrite); refresh capped at 25 FPS.

## Verify before trusting

- [x] `wifi_csi_config_t` — v5.3 uses the plain struct on S3 (no HE support);
      field names in main.c match. `dump_ack_en` exists too (left default).
- [x] `wifi_csi_info_t` — rssi is nested: `info.rx_ctrl.rssi` (fixed in main.c).
- [x] LVGL9 chart API compiles: lv_chart_set_value_by_id / lv_screen_active.
- [x] Versions resolved: lvgl 9.3.x, esp_lvgl_port 2.9.0, esp-dsp 1.8.2.
      NOTE: esp_lvgl_port 2.9 REQUIRES lvgl ~9.3 (RGB565_SWAPPED); ~9.2 fails.
      NOTE: REQUIRES name is `esp-dsp` (hyphen), not esp_dsp.

## Environment

- IDF: ~/esp/esp-idf @ v5.3 (shallow). Activate: `. ~/esp/esp-idf/export.sh`
- Flashing FNK0104N over native USB-CDC: use `--no-stub --baud 115200`;
  ROM download mode = hold BOOT, tap RST, release BOOT (`lsusb` 303a:1001).
  See esp32-cyd-platformio skill, flash-and-lvgl-pitfalls reference.
