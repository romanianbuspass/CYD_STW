/*
 * ST77922 QSPI display driver — ESP-IDF port of the Freenove FNK0104N class.
 * Init command table and QSPI transaction structure kept verbatim from
 * Freenove_ESP32_S3/legacy_cyd/lib/freenove_st77922/ST77922.cpp.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "hal/gpio_ll.h"
#include "esp_check.h"
#include "esp_log.h"

#include "st77922.h"
#include "board_cyd.h"

static const char *TAG = "st77922";

#define WR_RAM_C_CMD 0x3C
#define WR_RAM_CMD   0x2C
#define SET_X_CMD    0x2A
#define SET_Y_CMD    0x2B
#define MADCTL_CMD   0x36
#define QSPI_1W_CMD  0x02
#define QSPI_4W_CMD  0x32

#define QSPI_PORT      SPI2_HOST
#define QSPI_FREQUENCY 40000000  /* 40 MHz (bisect value; Freenove default 80) */
#define QSPI_MODE      0  /* SPI mode 0 (Freenove source used Arduino SPI_MODE0) */

/* Fast pin ops for the backlight (BL is plain GPIO; CS is owned by the SPI
 * peripheral via IOMUX — GPIO10 is the native FSPI CS0 — so LL CS writes
 * would be no-ops and were removed with the bring-up diagnostics). */
#if ((CYD_LCD_BL>=0) && (CYD_LCD_BL<32))
    #define LCD_BL_LOW  GPIO.out_w1tc = (1 << CYD_LCD_BL)
    #define LCD_BL_HIGH GPIO.out_w1ts = (1 << CYD_LCD_BL)
#else
    #define LCD_BL_LOW  GPIO.out1_w1tc.val = (1 << (CYD_LCD_BL - 32))
    #define LCD_BL_HIGH GPIO.out1_w1ts.val = (1 << (CYD_LCD_BL - 32))
#endif

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t len;
    uint32_t delay_ms;
} lcd_init_cmd_t;

/* Init sequence verbatim from Freenove ST77922.cpp (data arrays inlined) */
static const lcd_init_cmd_t st77922_lcd_init[] = {
    {0xF1, {0x00}, 1, 0},
    {0x60, {0x00, 0x00, 0x00}, 3, 0},
    {0x65, {0x80}, 1, 0},
    {0x79, {0x06}, 1, 0},
    {0x7B, {0x00, 0x08, 0x08}, 3, 0},
    {0x80, {0x55, 0x62, 0x2F, 0x17, 0xF0, 0x52, 0x70, 0xD2, 0x52, 0x62, 0xEA}, 11, 0},
    {0x81, {0x26, 0x52, 0x72, 0x27}, 4, 0},
    {0x84, {0x92, 0x25}, 2, 0},
    {0x87, {0x10, 0x10, 0x58, 0x00, 0x02, 0x3A}, 6, 0},
    {0x88, {0x00, 0x00, 0x2C, 0x10, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x06}, 15, 0},
    {0x89, {0x00, 0x00, 0x00}, 3, 0},
    {0x8A, {0x13, 0x00, 0x2C, 0x00, 0x00, 0x2C, 0x10, 0x10, 0x00, 0x3E, 0x19}, 11, 0},
    {0x8B, {0x15, 0xB1, 0xB1, 0x44, 0x96, 0x2C, 0x10, 0x97, 0x8E}, 9, 0},
    {0x8C, {0x1D, 0xB1, 0xB1, 0x44, 0x96, 0x2C, 0x10, 0x50, 0x0F, 0x01, 0xC5, 0x12, 0x09}, 13, 0},
    {0x8D, {0x0C}, 1, 0},
    {0x8E, {0x33, 0x01, 0x0C, 0x13, 0x01, 0x01}, 6, 0},
    {0xB3, {0x00, 0x30}, 2, 0},
    {0xF1, {0x00}, 1, 0},
    {0x71, {0xD0}, 1, 0},
    {0x66, {0x02, 0x3F}, 2, 0},
    {0xBE, {0x26, 0x00, 0x9D}, 3, 0},
    {0x70, {0x01, 0xA0, 0x11, 0x40, 0xE0, 0x00, 0x11, 0x69, 0x11, 0x00, 0x00, 0x1A}, 12, 0},
    {0x90, {0x04, 0x04, 0x55, 0x74, 0x00, 0x40, 0x43, 0x27, 0x27}, 9, 0},
    {0x91, {0x04, 0x04, 0x55, 0x75, 0x00, 0x40, 0x42, 0x27, 0x27}, 9, 0},
    {0x92, {0x04, 0x44, 0x55, 0xC0, 0x06, 0x00, 0x07, 0x05, 0x90, 0x27}, 10, 0},
    {0x93, {0x04, 0x43, 0x11, 0x00, 0x00, 0x00, 0x00, 0x05, 0x90, 0x27}, 10, 0},
    {0x94, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 6, 0},
    {0x95, {0x96, 0x16, 0x00, 0x00, 0xFF}, 5, 0},
    {0x96, {0x44, 0x53, 0x03, 0x12, 0x23, 0x24, 0x06, 0x05, 0x94, 0x27, 0x00, 0x44}, 12, 0},
    {0x97, {0x44, 0x53, 0x47, 0x56, 0x20, 0x20, 0x02, 0x01, 0x94, 0x27, 0x00, 0x44}, 12, 0},
    {0xBA, {0x55, 0x94, 0x2D, 0x94, 0x27}, 5, 0},
    {0x9A, {0x40, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00}, 7, 0},
    {0x9B, {0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00}, 7, 0},
    {0x9C, {0x5C, 0x12, 0x00, 0x00, 0x10, 0x12, 0x00, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00}, 13, 0},
    {0x9D, {0x8A, 0x51, 0x00, 0x00, 0x00, 0x80, 0x1E, 0x01}, 8, 0},
    {0x9E, {0x51, 0x00, 0x00, 0x00, 0x80, 0x1E, 0x01}, 7, 0},
    {0xB4, {0x1D, 0x1C, 0x1E, 0x0B, 0x14, 0x02, 0x13, 0x09, 0x1E, 0x00, 0x1E, 0x10}, 12, 0},
    {0xB5, {0x1D, 0x1C, 0x1E, 0x0A, 0x15, 0x03, 0x11, 0x08, 0x1E, 0x01, 0x1E, 0x12}, 12, 0},
    {0xB6, {0x77, 0x77, 0x00, 0x0A, 0xFF, 0x0A, 0xFF}, 7, 0},
    {0x86, {0xCD, 0x04, 0xB1, 0x02, 0x58, 0x12, 0x58, 0x0C, 0x13, 0x01, 0xA5, 0x00, 0xA5, 0xA5}, 14, 0},
    {0xB7, {0x07, 0x0A, 0x0E, 0x06, 0x05, 0x03, 0x2B, 0x03, 0x03, 0x42, 0x07, 0x10, 0x10, 0x2E, 0x3F, 0x0D}, 16, 0},
    {0xB8, {0x07, 0x0A, 0x0D, 0x05, 0x05, 0x02, 0x2B, 0x02, 0x03, 0x42, 0x06, 0x10, 0x0F, 0x2E, 0x3F, 0x0D}, 16, 0},
    {0xB9, {0x23, 0x23}, 2, 0},
    {0xBF, {0x10, 0x14, 0x14, 0x0B, 0x0B, 0x0B}, 6, 0},
    {0xF2, {0x00}, 1, 0},
    {0x73, {0x04, 0xDA, 0x12, 0x54, 0x47}, 5, 0},
    {0x77, {0x6B, 0x5B, 0xFD, 0xC3, 0xC5}, 5, 0},
    {0x7A, {0x15, 0x27}, 2, 0},
    {0x7B, {0x04, 0x57}, 2, 0},
    {0x7E, {0x01, 0x0E}, 2, 0},
    {0xBF, {0x36}, 1, 0},
    {0xE3, {0x40, 0x40}, 2, 0},
    {0xF0, {0x00}, 1, 0},
    {0xD0, {0x00}, 1, 0},
    {0x2A, {0x00, 0x00, 0x01, 0x3F}, 4, 0},
    {0x2B, {0x00, 0x00, 0x01, 0xDF}, 4, 0},
    {0x21, {0x00}, 0, 0},
    {0x11, {0x00}, 0, 120},
    {0x29, {0x00}, 0, 0},
    {0x2C, {0x00}, 0, 0},
    {0x3A, {0x01}, 1, 0},
    {0x36, {0x00}, 1, 0},
    {0x35, {0x01}, 1, 20},
};

static spi_device_handle_t s_qspi;
static uint32_t s_tx_count, s_tx_errs;

uint32_t st77922_tx_count(void) { return s_tx_count; }
uint32_t st77922_tx_errors(void) { return s_tx_errs; }

static void write_reg(uint32_t cmd, const void *data, uint8_t len)
{
    spi_transaction_ext_t t;
    memset(&t, 0, sizeof(t));
    t.base.flags = (SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR);
    t.base.cmd = QSPI_1W_CMD;
    t.base.addr = cmd << 8;
    t.command_bits = 8;
    t.address_bits = 24;
    t.base.tx_buffer = (len != 0) ? data : NULL;
    t.base.length = 8 * len;
    spi_device_polling_transmit(s_qspi, (spi_transaction_t *)&t);
}

void st77922_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << CYD_LCD_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    LCD_BL_LOW;

    const spi_bus_config_t buscfg = {
        .data0_io_num = CYD_LCD_D0,
        .data1_io_num = CYD_LCD_D1,
        .sclk_io_num  = CYD_LCD_SCLK,
        .data2_io_num = CYD_LCD_D2,
        .data3_io_num = CYD_LCD_D3,
        .max_transfer_sz = (ST77922_WIDTH * 2) + 8,  /* one full row + slack */
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_IOMUX_PINS
               | SPICOMMON_BUSFLAG_QUAD,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(QSPI_PORT, &buscfg, SPI_DMA_CH_AUTO));

    const spi_device_interface_config_t devcfg = {
        .command_bits = 8,    /* IDF5: declare real phase widths (4.4 tolerated 0 here
                               * with VARIABLE_CMD/ADDR overrides; 5.x is stricter) */
        .address_bits = 24,
        .cs_ena_pretrans = 2,   /* CS setup margin (panel TRCS 45 ns) */
        .cs_ena_posttrans = 2,  /* CS hold margin */
        .mode = QSPI_MODE,
        .clock_speed_hz = QSPI_FREQUENCY,
        .spics_io_num = CYD_LCD_CS,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 17,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(QSPI_PORT, &devcfg, &s_qspi));
    int actual_khz = 0;
    ESP_ERROR_CHECK(spi_device_get_actual_freq(s_qspi, &actual_khz));
    ESP_LOGI(TAG, "spi dev added, actual clock %d kHz", actual_khz);

    for (size_t i = 0; i < sizeof(st77922_lcd_init) / sizeof(lcd_init_cmd_t); i++) {
        write_reg(st77922_lcd_init[i].cmd, st77922_lcd_init[i].data,
                  st77922_lcd_init[i].len);
        if (st77922_lcd_init[i].delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(st77922_lcd_init[i].delay_ms));
        }
    }

    /* Rotation 0: MADCTL 0x00 (portrait; raw touch axes match) */
    uint8_t madctl = 0x00;
    write_reg(MADCTL_CMD, &madctl, 1);

    LCD_BL_HIGH;
    ESP_LOGI(TAG, "panel up: %dx%d QSPI @%d MHz", ST77922_WIDTH, ST77922_HEIGHT,
             QSPI_FREQUENCY / 1000000);
}

void st77922_set_window(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey)
{
    uint8_t x_data[] = {
        (uint8_t)(sx >> 8), (uint8_t)(sx & 0xFF),
        (uint8_t)((ex - 1) >> 8), (uint8_t)((ex - 1) & 0xFF),
    };
    write_reg(SET_X_CMD, x_data, 4);
    uint8_t y_data[] = {
        (uint8_t)(sy >> 8), (uint8_t)(sy & 0xFF),
        (uint8_t)((ey - 1) >> 8), (uint8_t)((ey - 1) & 0xFF),
    };
    write_reg(SET_Y_CMD, y_data, 4);
}

void st77922_write_pixels(uint16_t sx, uint16_t sy, uint16_t w, uint16_t h,
                          const uint16_t *fb, uint32_t stride_px)
{
    if ((sx >= ST77922_WIDTH) || (sy >= ST77922_HEIGHT)) return;
    if (((sx + w) > ST77922_WIDTH) || ((sy + h) > ST77922_HEIGHT)) return;
    if ((w < 1) || (h < 1)) return;

    st77922_set_window(sx, sy, sx + w, sy + h);

    ESP_ERROR_CHECK(spi_device_acquire_bus(s_qspi, portMAX_DELAY));
    for (uint16_t r = 0; r < h; r++) {
        /* fb points at the FIRST pixel of the window (chunk-local in LVGL
         * PARTIAL mode; caller adds the base offset in DIRECT mode).
         * Indexing with absolute (sy,sx) here read OOB past chunk buffers
         * and produced static bands. */
        const uint16_t *line = fb + (size_t)r * stride_px;
        spi_transaction_ext_t t;
        memset(&t, 0, sizeof(t));
        t.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD
                     | SPI_TRANS_VARIABLE_ADDR;
        if (r == 0) {
            t.base.cmd = QSPI_4W_CMD;
            t.base.addr = WR_RAM_C_CMD << 8;
            t.command_bits = 8;
            t.address_bits = 24;
        }
        if (r < h - 1) t.base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
        t.base.tx_buffer = line;
        t.base.length = (size_t)w * 16;
        esp_err_t err = spi_device_polling_transmit(s_qspi, (spi_transaction_t *)&t);
        s_tx_count++;
        if (err != ESP_OK) {
            s_tx_errs++;
            if (s_tx_errs <= 5) {
                ESP_LOGW(TAG, "tx err %d row %d (%ux%u @%u,%u)", err, r, w, h, sx, sy);
            }
        }
    }
    spi_device_release_bus(s_qspi);
}

void st77922_backlight(bool on)
{
    if (on) LCD_BL_HIGH; else LCD_BL_LOW;
}
