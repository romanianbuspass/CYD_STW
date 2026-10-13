#pragma once
/*
 * Stage 5c — SD card (SDMMC 4-bit) + CSV telemetry logging + wifi.cfg.
 *
 * Resilience contract: a missing/failed SD card must never block the rest
 * of the firmware (display, CSI) — sd_log_init returns an error and every
 * other entry point degrades to a no-op.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* One telemetry record; pushed BY VALUE into the ring buffer by the DSP
 * task (cheap struct copy, no formatting in the hot path). */
typedef struct {
    uint32_t timestamp_ms;
    float iq_variance;          /* mean of the 64-bin LLTF magnitude variance */
    float hampel_filtered_val;  /* 0.0f until Stage 6 Hampel lands */
    uint32_t dropped_frames;
} sd_log_rec_t;

/* Mount the SD card at /sdcard (SDMMC 4-bit: CLK=6 CMD=5 D0=4 D1=7 D2=2
 * D3=3), create the ring buffer + sd_log_task, and open
 * /sdcard/csi_telemetry.csv (writes the header row on a new file).
 * Returns ESP_OK when the card is live; any failure is non-fatal. */
esp_err_t sd_log_init(void);

/* Read /sdcard/wifi.cfg (SSID=/PASS= lines, '#' comments). Writes a
 * self-documenting template when the file is absent. Returns true only
 * when usable (non-template) credentials were parsed. */
bool sd_log_wifi_creds(char *ssid, size_t ssid_len, char *pass, size_t pass_len);

/* Non-blocking record push from the DSP task. No-op without a card;
 * ring-full drops are counted (see sd_log_push_drops). */
void sd_log_push(const sd_log_rec_t *rec);

uint32_t sd_log_push_drops(void);
