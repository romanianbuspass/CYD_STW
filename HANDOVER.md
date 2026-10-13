# CYD_STW — Session Handover (2026-09-09, display CLOSED)

Read this + `SYSTEM_ARCHITECTURE.md` + `STATUS.md` first. Skill:
`~/.hermes/skills/software-development/esp32-cyd-espidf/SKILL.md`.

## One-line state
Display bring-up **COMPLETE**: LVGL UI visually confirmed on panel
(chart grid + status text), QSPI 40 MHz, `tx err=0`. Diagnostics removed.
Pipeline resumes at Stage 5b (touch).

## Root causes (fixed 2026-09-09; full detail in SYSTEM_ARCHITECTURE.md §3/§5)
1. `SPI_TRANS_CS_KEEP_ACTIVE` without `spi_device_acquire_bus()` → every
   spi_master transaction failed ESP_ERR_INVALID_ARG ("polling can't get
   buslock"). Panel/pins/protocol were never at fault — the earlier
   "1-line writes land" inference was wrong; NOTHING reached the pins.
2. LVGL 9.3.0 Kconfig has no `LV_DRAW_SW_SUPPORT_RGB565_SWAPPED` → SWAPPED
   blend compiled out → renderer emitted zero buffers → black screen with
   clean TX. Fixed: native RGB565 render + byte-swap in flush_cb (panel is
   hi-byte-first; blue↔green was the oracle).
3. `write_pixels` indexed LVGL PARTIAL chunk buffers with absolute
   coordinates → OOB reads → static bands. Fixed: window-relative.

## Proven facts (do not re-litigate)
- Hardware functional. FPC/module never at fault.
- No MISO routed to ESP32: RDID always 0x00 by design; diagnostics removed.
- TDDI held in reset: pulse GPIO48 (HIGH 10ms→LOW 10ms→HIGH, wait 200ms)
  before QSPI init; I2C 0x55 ACKs after, 0x18 (codec) is bus-health control.
- Init table byte-identical to Freenove `ST77922.cpp` (58 entries).
- All six QSPI pins are native FSPI IOMUX pins; CS is peripheral-owned,
  LL GPIO CS writes are no-ops.
- Solid-color fill tests are shift-invisible — validate with varying
  patterns; one known fullscreen color (blue) is the byte-order oracle.
- Long single-session 480-row raw writes garble at 40 MHz; production
  chunked-window pattern is clean (diagnostic-only observation).

## Next up (pipeline)
- Stage 5b: touch indev (ST77922 TDDI, I2C @0x55 on GPIO39/38, INT=GPIO47)
- Stage 6: Hampel pre-filter, HT-LTF segment exposure
- Stage 7: traffic generator for quiet APs
- CSI live feed → chart path is wired (mailbox); needs live traffic (7)
  for on-screen motion.

## Ops ritual (works every time)
- Build env: `export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v hermes-agent | paste -sd:) && . ~/esp/esp-idf/export.sh` then `idf.py build` in project root. (IDF scripts fail if hermes-agent venv is on PATH.)
- Flash: user does BOOT dance (hold BOOT, tap RST, release BOOT), says
  "boot enabled" → `bash tools/flash_cyd.sh` (esptool.py, /dev/ttyACM0).
- Boot log: esptool RTS reset does NOT boot app on this board; physical RST
  tap required. Arm `bash tools/capture_boot.sh 90 > /tmp/cyd_bootN.log`
  (background) BEFORE asking user to tap RST. Heartbeat logs
  `alive t=Ns flush=F tx=T txerr=E last=WxH` every 5 s.

## Key files
- `components/st77922/st77922.c` — driver (clean; TX counters retained)
- `components/board/include/board_cyd.h` — pinout (single source)
- `main/display.c` — LVGL9 glue (PARTIAL, native RGB565 + flush swap, rounder)
- `main/main.c` — CSI cb → queue → DSP task (core0) → mailbox → LVGL task (core1, 25 FPS)
- `tools/flash_cyd.sh`, `tools/capture_boot.sh`
- Reference (read-only): `<LOCAL_WORKSPACE>/Freenove_ESP32_S3/legacy_cyd/lib/freenove_st77922/`

## Board facts
MAC <DEVICE_MAC>, ROM esp32s3-20210327, flash dio (forced w/ octal
PSRAM), app ~1.05 MB / 4 MB partition (74% free).
