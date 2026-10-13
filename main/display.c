/*
 * LVGL9 display glue for the ST77922 QSPI panel.
 *
 * Render mode: PARTIAL with two 320x48 RGB565 bounce buffers in INTERNAL
 * DMA-capable SRAM (never PSRAM — mirrors the proven legacy_cyd
 * architecture).
 *
 * 4-px rounder: LVGL9 removed disp_drv.rounder_cb; the equivalent is an
 * LV_EVENT_INVALIDATE_AREA event callback that expands the invalidated
 * area to 4-px boundaries BEFORE rendering (same semantics as the
 * legacy_cyd disp_rounder that eliminated QSPI pixel-stream shear).
 */

#include "display.h"
#include "st77922.h"
#include "board_cyd.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "display";

#define CYD_PARTIAL_ROWS 48
#define FB_BYTES (CYD_LCD_H_RES * CYD_PARTIAL_ROWS * 2)

/* TDDI reset: the ST77922 controller is held on GPIO48. Without this pulse
 * the controller sits in reset and the panel ignores QSPI entirely while
 * the backlight stays on. (Stage 5b touch was dropped: no I2C traffic on
 * this board — the legacy-I2C bus-free wedge on cold-boot brown-out dies
 * with it.) */
static void tddi_reset(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << CYD_TOUCH_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(CYD_TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CYD_TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CYD_TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));  /* controller boot time after reset release */
    ESP_LOGI(TAG, "TDDI reset pulsed on GPIO%d", CYD_TOUCH_RST);
}

static uint8_t *s_buf1, *s_buf2;
static uint32_t s_flush_count;
static uint16_t s_last_w, s_last_h;

uint32_t cyd_display_flush_count(void) { return s_flush_count; }
void cyd_display_last_flush(uint16_t *w, uint16_t *h) { *w = s_last_w; *h = s_last_h; }

/* LVGL9 rounder: expand invalidated areas to 4-px boundaries before render
 * (ST77922 QSPI constraint — unaligned partial windows shear the stream) */
static void rounder_cb(lv_event_t *e)
{
    lv_area_t *area = lv_event_get_param(e);
    area->x1 &= ~(CYD_QSPI_ALIGN - 1);
    area->y1 &= ~(CYD_QSPI_ALIGN - 1);
    area->x2 |= (CYD_QSPI_ALIGN - 1);
    area->y2 |= (CYD_QSPI_ALIGN - 1);
    if (area->x2 >= CYD_LCD_H_RES) area->x2 = CYD_LCD_H_RES - 1;
    if (area->y2 >= CYD_LCD_V_RES) area->y2 = CYD_LCD_V_RES - 1;
}

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    int32_t w = lv_area_get_width(area);
    int32_t h = lv_area_get_height(area);

    /* Alignment watchdog — should never fire with rounder_cb active */
    if ((area->x1 & 3) || (area->y1 & 3) || ((area->x2 + 1) & 3) || ((area->y2 + 1) & 3)) {
        ESP_LOGW(TAG, "misaligned flush %ld,%ld %ldx%ld", (long)area->x1,
                 (long)area->y1, (long)w, (long)h);
    }

    /* Byte-swap in place before transmit: the panel assembles 16-bit pixels
     * high-byte-first (proven: native-order blue 0x001F painted as green
     * 0x1F00). LVGL 9.3.0's Kconfig lacks LV_DRAW_SW_SUPPORT_RGB565_SWAPPED,
     * so the SWAPPED render path is compiled out; we render native RGB565
     * (supported) and swap here — the v8 LV_COLOR_16_SWAP role. */
    uint16_t *px = (uint16_t *)px_map;
    const size_t n = (size_t)w * (size_t)h;
    for (size_t i = 0; i < n; i++) px[i] = (uint16_t)((px[i] << 8) | (px[i] >> 8));

    /* PARTIAL mode: px_map is the packed area block, stride = area width */
    st77922_write_pixels((uint16_t)area->x1, (uint16_t)area->y1,
                         (uint16_t)w, (uint16_t)h,
                         (const uint16_t *)px_map, (uint32_t)w);
    s_flush_count++;
    s_last_w = (uint16_t)w;
    s_last_h = (uint16_t)h;
    lv_display_flush_ready(disp);
}

/* Allocate the LVGL draw buffers FIRST (app_main, before the SDMMC mount):
 * LVGL gets first claim on contiguous internal DMA SRAM; a failure here is
 * a loud assert at boot, never a NULL pointer handed to the SPI DMA. */
void cyd_display_prealloc(void)
{
    s_buf1 = heap_caps_calloc(1, FB_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_buf2 = heap_caps_calloc(1, FB_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(s_buf1 != NULL && "Display buffer 1 allocation failed!");
    assert(s_buf2 != NULL && "Display buffer 2 allocation failed!");
    ESP_LOGI(TAG, "draw buffers preallocated: 2 x %u B internal DMA SRAM @%p/%p",
             (unsigned)FB_BYTES, s_buf1, s_buf2);
}

lv_display_t *cyd_display_init(void)
{
    tddi_reset();
    st77922_init();

    lv_display_t *disp = lv_display_create(CYD_LCD_H_RES, CYD_LCD_V_RES);
    /* NATIVE RGB565, not RGB565_SWAPPED: LVGL 9.3.0's Kconfig lacks
     * LV_DRAW_SW_SUPPORT_RGB565_SWAPPED, so under Kconfig the SWAPPED blend
     * compiles OUT (lv_conf_internal.h defaults it to 0) and the sw renderer
     * silently renders NOTHING (flush probe: sum256=0). Native RGB565 is
     * supported (CONFIG_LV_DRAW_SW_SUPPORT_RGB565=y) and matches the panel:
     * the bars smoke test sent native uint16 RGB565 in memory order and the
     * colors were correct (0xF800 -> red, 0x07E0 -> green). */
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);

    /* Buffers preallocated by cyd_display_prealloc() in app_main BEFORE the
     * SDMMC mount — LVGL gets first claim on contiguous internal DMA SRAM. */
    assert(s_buf1 != NULL && "Display buffer 1 allocation failed!");
    assert(s_buf2 != NULL && "Display buffer 2 allocation failed!");

    lv_display_set_buffers(disp, s_buf1, s_buf2, FB_BYTES,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, flush_cb);
    lv_display_add_event_cb(disp, rounder_cb, LV_EVENT_INVALIDATE_AREA, disp);

    /* Opaque base background: LVGL owns every pixel; a failed render can
     * never pass stale buffer content to the panel. */
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x1E1E1E), 0);
    lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, 0);
    return disp;
}
