#pragma once
/*
 * Freenove FNK0104N 3.5" CYD (ESP32-S3R8, 16 MB flash, 8 MB octal PSRAM, qio_opi)
 *
 * Pinout source: esp32-cyd-platformio skill, references/fnk0104n-pinout.md
 * (consensus of Freenove ST77922.h / ST77922_Touch.h / demo sketches).
 */

/* ST77922 display — QSPI (no MISO/DC; needs a Freenove-style driver port,
 * plain esp_lcd_panel_io 4-wire SPI will NOT work) */
#define CYD_LCD_CS      10
#define CYD_LCD_SCLK    12
#define CYD_LCD_D0      11
#define CYD_LCD_D1      13
#define CYD_LCD_D2      14
#define CYD_LCD_D3      9
#define CYD_LCD_BL      41
#define CYD_LCD_H_RES   320
#define CYD_LCD_V_RES   480
/* Partial-flush areas must align to 4 px (ST77922 QSPI rounder fix,
 * confirmed root cause of drag/dropdown corruption on legacy_cyd) */
#define CYD_QSPI_ALIGN  4

/* ST77922 TDDI touch — I2C_NUM_0 @ 0x55, 100 kHz.
 * Shared bus with ES8311 audio codec and the expansion I2C header. */
#define CYD_TOUCH_SCL   39
#define CYD_TOUCH_SDA   38
#define CYD_TOUCH_RST   48
#define CYD_TOUCH_INT   47
#define CYD_TOUCH_ADDR  0x55

/* Extended IO header — safe for UART/PPS expansions.
 * Do NOT route signals to 38/39; that is the live touch I2C bus. */
#define CYD_IO45        45
#define CYD_IO46        46

/* UART0 header (free; debug console is USB-CDC) */
#define CYD_UART0_TX    43
#define CYD_UART0_RX    44

/* Misc */
#define CYD_RGB_LED     40  /* WS2812 */
#define CYD_BOOT_BTN    0

/* SDMMC (unused in STW scaffold, documented for future logging) */
#define CYD_SD_CMD      4
#define CYD_SD_CLK      5
#define CYD_SD_D0       6
#define CYD_SD_D1       7
#define CYD_SD_D2       2
#define CYD_SD_D3       3
