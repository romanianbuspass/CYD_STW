# CYD_STW — System Architecture (source of truth)

Handheld Wi-Fi CSI ("See Through Walls") diagnostic tool.
Freenove FNK0104N 3.5" CYD, ESP32-S3R8 (16 MB flash, 8 MB octal PSRAM).

> **Verification status (2026-09-09):** build green; RTOS/CSI/DSP pipeline
> compile- and runtime-verified (heartbeat + counters); **display pipeline
> visually confirmed end-to-end** (QSPI 40 MHz, LVGL UI renders — chart grid
> + status text on screen). Root causes of the bring-up failure are
> documented in §5.

## 1. Toolchain & Environment

- Framework: native ESP-IDF **v5.3** at `~/esp/esp-idf`. Strictly NO Arduino /
  PlatformIO (prior CYD work was PlatformIO; this project is a clean IDF port).
- Target: `esp32s3`, 240 MHz, QIO-forced-DIO flash (R8 package constraint),
  octal PSRAM (`CONFIG_SPIRAM_MODE_OCT`), custom partitions.csv (4 MB app slot).
- Managed components (`main/idf_component.yml`):
  - `lvgl/lvgl ~9.3` (resolved 9.3.x) — NOTE: esp_lvgl_port 2.9 needs ≥9.3
    (`LV_COLOR_FORMAT_RGB565_SWAPPED`); 9.2 fails.
  - `espressif/esp_lvgl_port ^2.4` (resolved 2.9.0; currently not used by the
    display glue — hand-rolled DIRECT-mode path instead)
  - `espressif/esp-dsp >=1.4,<2` (resolved 1.8.2)
- CSI: **in-IDF API** (`esp_wifi_set_csi_rx_cb`, `esp_wifi_set_csi_config`,
  `esp_wifi_set_csi`) gated by `CONFIG_ESP_WIFI_CSI_ENABLED=y`.
  The `espressif/esp-csi` repo is examples + host tooling — deliberately NOT
  vendored.
- REQUIRES name for esp-dsp is `esp-dsp` (hyphen). Delete `dependencies.lock`
  after version pin changes.
- Shell: strip the Hermes venv from PATH before `install.sh`/`export.sh`
  (`export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v 'hermes-agent' | paste -sd:)`);
  the install check is `sys.prefix`-based.
- Flashing (FNK0104N native USB-CDC): `tools/flash_cyd.sh` — `esptool.py
  --no-stub -b 115200 --before no_reset`. Stub upload dies on this USB path.
  esptool's "hard reset" does NOT boot the app on this board — physical RST
  tap required after flashing. ROM download mode: hold BOOT, tap RST,
  release BOOT (`lsusb` → 303a:1001).
- Console: USB-Serial/JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`). Early
  boot output is lost to USB re-enumeration; firmware heartbeat reprints
  status every 5 s (`alive t=.. flush=.. tx=.. txerr=.. last=WxH`).

## 2. Hardware: Freenove FNK0104N pinout (components/board/include/board_cyd.h)

### ST77922 display — QSPI (no MISO, no DC pin)
| Function | GPIO |
|----------|------|
| CS       | 10   |
| SCLK     | 12   |
| D0 (SDA, lane 0) | 11 |
| D1 (lane 1)      | 13 |
| D2 (lane 2)      | 14 |
| D3 (lane 3)      |  9 |
| Backlight        | 41 |

- Backlight: GPIO41, active HIGH, driven at end of panel init. Floats ON when
  unconfigured — backlight alone is NOT proof of init.
- Resolution 320x480, rotation fixed 0 (MADCTL 0x00; raw touch axes match).

### CRITICAL — TDDI wakeup (GPIO48)
The ST77922 TDDI package (display+touch controller) is **held in reset by
default**. Before ANY QSPI init, firmware MUST:
1. Configure GPIO48 output; drive HIGH (10 ms) → LOW (10 ms) → HIGH, then
   wait ≥200 ms for controller boot.
2. Probe I2C_NUM_0 (SCL=39, SDA=38, 100 kHz) address `0x55` for ACK.
   - Without the reset pulse: `0x55` NACKs and the panel ignores QSPI
     entirely (proven 2026-09-09: NACK → pulse → ACK).
   - `0x18` (ES8311 codec, same bus) ACKs regardless — use it as the
     bus-health control.

### Touch (stage 5b, not yet implemented)
ST77922 TDDI over I2C @ 0x55: 16-bit register reads; STATUS poll until
`data & 0x0F == 0`, then MAX_TOUCHES. RST=GPIO48, INT=GPIO47.

### Other
UART0 header TXD0=43 / RXD0=44 (free; console is USB-CDC). Extended IO
header IO45/IO46 (safe for expansions; never route signals to 38/39 — live
touch bus). WS2812 RGB LED GPIO40. Boot button GPIO0.
SDMMC (internal slot): CLK=5 CMD=4 D0=6 D1=7 D2=2 D3=3 — roles per board
JSON; a rotated CLK=6/CMD=5/D0=4 variant fails card init with
ESP_ERR_TIMEOUT. FATFS needs CONFIG_FATFS_LFN_STACK for csi_telemetry.csv
(exceeds 8.3).

## 3. Display pipeline (ST77922 via QSPI) — VERIFIED 2026-09-09

### Wire-level encapsulation (bypasses missing DC pin)
32-bit command/address preamble, then data. Matches ST77922 spec §7.8.5.1
and the Freenove reference driver byte-for-byte:

- **Register write (init/config), 1-line:**
  instruction `0x02`, then AD[23:0] = `0x00, CMD, 0x00`, then parameters
  (1-line). Equivalently: `ST77922_CMD(cmd) = 0x02000000 | (cmd << 8)`.
- **Pixel write (4-line Quad):** instruction `0x32`, AD[23:0] = `0x00, 0x3C,
  0x00` (`0x32003C00`; 0x3C = write-continue, 0x2C = RAMWR for first frame),
  then RGB565 pixel stream over 4 data lines (`SPI_TRANS_MODE_QIO`).
- **Pixel byte order: high-byte-first.** The panel assembles 16-bit pixels
  MSByte-first (proven: native-order blue 0x001F painted as green 0x1F00).
  See the flush-time byte swap below.
- Read-back is NOT possible: the panel routes no MISO to the ESP32 (RDID
  always returns 0x00 by design; the RDID diagnostics were removed).

### Implementation: raw spi_master (NOT esp_lcd)
`components/st77922/st77922.c` — C port of the Freenove class:
- SPI2_HOST, half-duplex device, mode 0, 40 MHz. All six pins are the native
  FSPI IOMUX pins (CS=10, CLK=12, D0=11, D1=13, D2=14, D3=9) and the bus is
  initialized with `SPICOMMON_BUSFLAG_IOMUX_PINS|QUAD`. Consequence: CS is
  owned by the peripheral — LL GPIO writes to CS are no-ops; CS shaping must
  go through the driver (see CS_KEEP_ACTIVE below).
- `cs_ena_pretrans = 2`, `cs_ena_posttrans = 2` (CS setup/hold margin vs
  panel TRCS 45 ns).
- `spi_transaction_ext_t`: cmd phase 8 bits, addr phase 24 bits
  (`SPI_TRANS_VARIABLE_CMD/ADDR`); devcfg declares `command_bits=8,
  address_bits=24`.
- Init command table verbatim from Freenove `ST77922.cpp` (58 entries,
  incl. 120 ms sleep-out delay after 0x11 — hardware-mandatory).
- `st77922_write_pixels(sx, sy, w, h, fb, stride_px)`: `fb` points at the
  FIRST pixel of the window (LVGL PARTIAL `px_map` is chunk-local — indexing
  with absolute coordinates reads OOB and paints static bands). One CS
  session per call: `spi_device_acquire_bus()` + `SPI_TRANS_CS_KEEP_ACTIVE`
  on all rows but the last, `spi_device_release_bus()` at the end.
  **CS_KEEP_ACTIVE without acquire_bus fails every transaction with
  ESP_ERR_INVALID_ARG ("polling can't get buslock").** First row carries the
  `0x32`+addr preamble, continuation rows are data-only. Long single-session
  full-screen writes (480 rows, one preamble) garble at 40 MHz on this
  panel; the production pattern (set window per chunk + short sessions) is
  clean. TX counters retained for the heartbeat.
- If migrated to esp_lcd later: `dc_gpio_num=-1`, `lcd_cmd_bits=32`,
  `lcd_param_bits=8`, `.flags.quad_mode=1`, command word =
  `0x02000000|(cmd<<8)`, bitmap preamble `0x32003C00` — same wire format.

### LVGL 9.3 glue (main/display.c)
- Render mode: **PARTIAL**, 2 × 320×48 RGB565 bounce buffers (30,720 B each)
  in internal DMA-capable SRAM (`heap_caps_calloc`, zeroed so a failed
  render yields black, not static).
- Color format: **`LV_COLOR_FORMAT_RGB565` (native)** + an in-place byte
  swap of `px_map` in `flush_cb` before transmit. Rationale: LVGL 9.3.0's
  Kconfig has NO `LV_DRAW_SW_SUPPORT_RGB565_SWAPPED` option, and under
  Kconfig `lv_conf_internal.h` defaults it to 0 — the blend-to-SWAPPED path
  compiles OUT, the sw renderer silently produces nothing, and LVGL flushes
  untouched buffers (the "black screen with clean TX" symptom). Native
  RGB565 render (`CONFIG_LV_DRAW_SW_SUPPORT_RGB565=y`) + flush-time swap
  fills the v8 `LV_COLOR_16_SWAP` role.
- `lv_display_flush_ready(disp)` is called only after the blocking
  `spi_device_polling_transmit` sequence returns — no DMA race by
  construction.
- 4-px rounder via `LV_EVENT_INVALIDATE_AREA` event cb (LVGL9 has no
  rounder_cb). Confirmed: 10 clean 320×48 chunks per full render.
- Opaque base background on `lv_screen_active()` so LVGL owns every pixel.
- `lv_tick_inc(2)` from a periodic esp_timer; all LVGL calls on the LVGL task.
- DSP arrays stay in internal SRAM (static .bss). Pools never mix.

## 4. CSI & DSP RTOS pipeline (main/main.c)

1. **Wi-Fi driver task → `csi_rx_cb`:** fires in Wi-Fi task context (not a
   hard ISR). Non-blocking struct-copy of `wifi_csi_info_t` + payload clamp
   512 B (LLTF segment = first 128 B = 64 int8 I/Q pairs; HT20 bandwidth,
   AMPDU on). `xQueueSend(tick=0)`; drop-and-count on full. No malloc/log.
   Note: v5.3 nests radio metadata — `info.rx_ctrl.rssi`.
2. **`csi_dsp_task` (core 0, prio 5, 8 KB stack):** deinterleaves I/Q,
   amplitude `sqrt(I²+Q²)` via esp-dsp vector ops (`dsps_mul_f32` ×2,
   `dsps_add_f32`, `dsps_sqrt_f32` — S3 SIMD dispatch). O(N) running
   variance over a 32-frame ring buffer (static .bss, internal SRAM).
   Pushes `dsp_result_t` to a len-1 mailbox queue (`xQueueOverwrite`).
   Hampel pre-filter: TODO(stage-dsp).
3. **`lvgl_task` (core 1, prio 3, 8 KB stack):** 25 FPS cap
   (`pdMS_TO_TICKS(40)` receive timeout) — SPI bus starvation guard.
   `lv_chart` 64-point line series updated via `lv_chart_set_value_by_id`;
   status label shows frames/drops. Heartbeat log every 5 s.

Queues: CSI queue 16 × ~580 B frames; result mailbox 1 × dsp_result_t.
Wi-Fi: STA mode, creds via Kconfig (`CYD_STW_WIFI_SSID/PASSWORD`); CSI
requires live traffic (stage 7: ping/beacon generator for quiet APs).

## 5. Known pitfalls (cost us time; do not rediscover)

- **Display bring-up failure triad (root-caused 2026-09-09):**
  1. `SPI_TRANS_CS_KEEP_ACTIVE` without `spi_device_acquire_bus()` → every
     transaction fails with ESP_ERR_INVALID_ARG, logged as "polling can't
     get buslock". acquire/release must bracket any KEEP_ACTIVE session.
  2. LVGL 9.3.0 Kconfig lacks `LV_DRAW_SW_SUPPORT_RGB565_SWAPPED` →
     SWAPPED blend compiles out → renderer emits zeros → black screen with
     clean TX. Render native RGB565, byte-swap px_map in flush_cb (panel
     wants hi-byte-first; blue 0x001F vs green 0x1F00 was the tell).
  3. LVGL PARTIAL `px_map` is chunk-local — indexing it with absolute
     coordinates reads OOB and paints static bands matching chunk height.
- Solid-color fill tests are shift-invisible: any bit/byte/lane phase
  error still paints a clean solid. Validate pixel paths with VARYING
  patterns (color bars), and prefer one known fullscreen color (blue) as
  the byte-order oracle.
- Panel black + backlight on ⇒ check TDDI reset (GPIO48) first. RDID via
  any path reads 0x00 by design (no MISO routed) — never use it as a
  diagnostic.
- esptool "hard reset" doesn't boot the app on this board; RTS pulses can
  drop it INTO download mode. Physical RST tap is the reliable path.
- Early boot logs vanish into USB re-enumeration — rely on the 5 s
  heartbeat, not on catching boot output.
- PlatformIO-built images are not bootable by the IDF v5.3 bootloader
  ("Invalid image block") — cross-flashing old artifacts is not a valid
  hardware control experiment without the matching bootloader.
