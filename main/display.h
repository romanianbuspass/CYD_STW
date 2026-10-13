#pragma once
#include "lvgl.h"

/* ST77922 panel + LVGL9 PARTIAL-mode display, internal-SRAM bounce buffers.
 * Call once from the LVGL task, after lv_init(). */
lv_display_t *cyd_display_init(void);

/* Allocate the draw buffers in app_main BEFORE the SDMMC mount — LVGL gets
 * first claim on contiguous internal DMA SRAM. Asserts on failure. */
void cyd_display_prealloc(void);

/* Diagnostics (polled by the app heartbeat) */
uint32_t cyd_display_flush_count(void);
void cyd_display_last_flush(uint16_t *w, uint16_t *h);
