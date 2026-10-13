#pragma once
/*
 * ST77922 QSPI display driver — ESP-IDF port of the Freenove FNK0104N class.
 *
 * Source: Freenove_ESP32_S3/legacy_cyd/lib/freenove_st77922/ST77922.{h,cpp}
 * (Arduino bits removed; spi_master half-duplex QIO core kept verbatim).
 *
 * Rotation: fixed 0 (portrait 320x480, MADCTL 0x00). Rotations 1/3 used an
 * Arduino ps_malloc transpose path in the original and are NOT ported.
 * Touch axes match raw at MADCTL 0x00 on this panel (skill-verified).
 */

#include <stdint.h>
#include <stdbool.h>

#define ST77922_WIDTH   320
#define ST77922_HEIGHT  480

/* SPI bus + device + panel init sequence + backlight ON. */
void st77922_init(void);

/* ex/ey are EXCLUSIVE (matches Freenove Set_Windows semantics). */
void st77922_set_window(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey);

/*
 * Transmit a w x h pixel window from a framebuffer with arbitrary row stride.
 * One CS session (spi_device_acquire_bus + SPI_TRANS_CS_KEEP_ACTIVE);
 * each row is a separate QIO DMA transaction (rows after the first carry
 * no cmd/addr, exactly like the original).
 * fb points at the FIRST pixel of the window (LVGL PARTIAL mode passes the
 * chunk-local px_map directly; DIRECT mode callers add the base offset
 * themselves: fb = framebuf + sy*stride_px + sx). Pixels are RGB565
 * byte-swapped.
 */
void st77922_write_pixels(uint16_t sx, uint16_t sy, uint16_t w, uint16_t h,
                          const uint16_t *fb, uint32_t stride_px);

void st77922_backlight(bool on);

/* Diagnostics (polled by the app heartbeat) */
uint32_t st77922_tx_count(void);
uint32_t st77922_tx_errors(void);
