/*
 * Stage 5c — SD card (SDMMC 4-bit) + CSV telemetry logging + wifi.cfg.
 *
 * Topology: DSP task (hot path) --sd_log_push(struct by value)--> ring
 * buffer --> sd_log_task (low prio) --snprintf + chunked fwrite/fflush-->
 * /sdcard/csi_telemetry.csv. Formatting and SD latency never touch the
 * DSP task.
 */

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

#include "sd_log.h"

static const char *TAG = "sd_log";

#define SD_MOUNT_POINT   "/sdcard"
#define WIFI_CFG_PATH    SD_MOUNT_POINT "/wifi.cfg"
#define CSV_PATH         SD_MOUNT_POINT "/csi_telemetry.csv"
#define CSV_HEADER       "timestamp_ms,iq_variance,hampel_filtered_val,dropped_frames\n"

/* wifi.cfg template — mirrors the legacy legacy_cyd strategy: write a
 * self-documenting template, reject it until the user fills it in. */
static const char WIFI_TEMPLATE[] =
    "# CYD_STW Wi-Fi credentials\n"
    "# Fill in SSID and PASS, save, then power-cycle the unit.\n"
    "SSID=YourNetworkName\n"
    "PASS=YourPassword\n";

#define RING_BYTES      (4 * 1024)   /* ~85 records of 16 B + alignment */
#define IO_BUF_BYTES    (3 * 1024)   /* CSV accumulation before fwrite */
#define FLUSH_MS        2000         /* time-based flush even if buffer not full */
#define LOG_TASK_PRIO   2
#define LOG_TASK_STACK  4096
#define LOG_TASK_CORE   0

static RingbufHandle_t s_ring;
static FILE *s_file;
static volatile uint32_t s_push_drops;

uint32_t sd_log_push_drops(void) { return s_push_drops; }

/* ================================================================
 * Telemetry sink task
 * ================================================================ */
static void sd_log_task(void *arg)
{
    static char buf[IO_BUF_BYTES];
    size_t used = 0;
    uint32_t last_flush = esp_log_timestamp();

    for (;;) {
        size_t sz = 0;
        sd_log_rec_t *rec = xRingbufferReceive(s_ring, &sz, pdMS_TO_TICKS(FLUSH_MS));
        uint32_t now = esp_log_timestamp();
        if (rec) {
            int n = snprintf(buf + used, sizeof(buf) - used,
                             "%lu,%.6f,%.6f,%lu\n",
                             (unsigned long)rec->timestamp_ms,
                             (double)rec->iq_variance,
                             (double)rec->hampel_filtered_val,
                             (unsigned long)rec->dropped_frames);
            if (n > 0 && (size_t)n < sizeof(buf) - used) used += (size_t)n;
            vRingbufferReturnItem(s_ring, rec);
        }
        /* Chunked write: buffer-full or idle-timeout, whichever first.
         * fsync, NOT fflush: FATFS only updates the directory entry (file
         * size) on f_sync/f_close — with fflush alone every other reader
         * (and stat() at next boot) sees a 0-byte file forever. Observed
         * 2026-09-10: "CSV header written" logged at every boot, card
         * always empty. */
        if (used > 0 && ((sizeof(buf) - used < 96) || (now - last_flush >= FLUSH_MS))) {
            fwrite(buf, 1, used, s_file);
            fsync(fileno(s_file));
            used = 0;
            last_flush = now;
            /* One-time persistence proof: directory entry must show the
             * committed size (the fflush-era bug always reported 0). */
            static bool s_size_logged;
            if (!s_size_logged) {
                s_size_logged = true;
                struct stat fst;
                if (stat(CSV_PATH, &fst) == 0)
                    ESP_LOGI(TAG, "csv on disk: %ld B after first sync", (long)fst.st_size);
            }
        }
    }
}

/* ================================================================
 * Init: mount + csv + ring + task (all optional-failure tolerant)
 * ================================================================ */
esp_err_t sd_log_init(void)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();  /* 4-bit capable, 20 MHz */
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;
    /* FNK0104N internal slot pin map — per board JSON / legacy
     * legacy_cyd storage.cpp (verified on hardware). NOTE: CLK=5/CMD=4/
     * D0=6; a rotated CLK=6/CMD=5/D0=4 variant times out at card init. */
    slot.clk = 5;
    slot.cmd = 4;
    slot.d0 = 6;
    slot.d1 = 7;
    slot.d2 = 2;
    slot.d3 = 3;
    /* CYD has no external pull-ups on the SDMMC lines */
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    const esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_card_t *card = NULL;
    esp_err_t err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot, &mount, &card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed (%d) — logging disabled, boot continues", err);
        return err;
    }
    ESP_LOGI(TAG, "SD mounted: %s, %llu MB", card->cid.name,
             ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));

    /* CSV: append; write the header only when the file starts empty */
    s_file = fopen(CSV_PATH, "a");
    if (!s_file) {
        ESP_LOGW(TAG, "cannot open %s — logging disabled", CSV_PATH);
        return ESP_FAIL;
    }
    struct stat st;
    if (stat(CSV_PATH, &st) == 0 && st.st_size == 0) {
        fputs(CSV_HEADER, s_file);
        fsync(fileno(s_file));  /* persist the directory entry now */
        ESP_LOGI(TAG, "CSV header written to %s", CSV_PATH);
    }

    s_ring = xRingbufferCreate(RING_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (!s_ring) {
        ESP_LOGW(TAG, "ring buffer alloc failed — logging disabled");
        fclose(s_file);
        s_file = NULL;
        return ESP_ERR_NO_MEM;
    }
    xTaskCreatePinnedToCore(sd_log_task, "sd_log", LOG_TASK_STACK, NULL,
                            LOG_TASK_PRIO, NULL, LOG_TASK_CORE);
    return ESP_OK;
}

/* ================================================================
 * wifi.cfg (legacy-compatible SSID=/PASS= format)
 * ================================================================ */
bool sd_log_wifi_creds(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    ssid[0] = pass[0] = '\0';

    FILE *f = fopen(WIFI_CFG_PATH, "r");
    if (!f) {
        f = fopen(WIFI_CFG_PATH, "w");
        if (f) {
            fputs(WIFI_TEMPLATE, f);
            fclose(f);
            ESP_LOGI(TAG, "wrote %s template — fill in and reboot", WIFI_CFG_PATH);
        }
        return false;
    }
    char line[96];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#') continue;
        char *nl = strpbrk(line, "\r\n");
        if (nl) *nl = '\0';
        if (strncmp(line, "SSID=", 5) == 0) {
            strlcpy(ssid, line + 5, ssid_len);
        } else if (strncmp(line, "PASS=", 5) == 0) {
            strlcpy(pass, line + 5, pass_len);
        }
    }
    fclose(f);

    /* Reject empty or untouched template */
    if (ssid[0] == '\0' || strcmp(ssid, "YourNetworkName") == 0) return false;
    ESP_LOGI(TAG, "wifi.cfg: SSID \"%s\" loaded", ssid);
    return true;
}

/* ================================================================
 * Hot-path push (DSP task): struct by value, never blocks
 * ================================================================ */
void sd_log_push(const sd_log_rec_t *rec)
{
    if (!s_ring) return;
    if (xRingbufferSend(s_ring, rec, sizeof(*rec), 0) != pdTRUE) {
        s_push_drops++;
    }
}
