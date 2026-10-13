/*
 * CYD_STW — handheld Wi-Fi CSI sensing diagnostic tool
 * Stage 1-2 scaffold: Wi-Fi CSI capture -> FreeRTOS queue -> DSP task -> LVGL chart
 *
 * Task topology (strictly non-blocking at the radio):
 *   Wi-Fi driver task (IDF, high prio) --csi_rx_cb--> xQueueSend(tick=0) --> csi_dsp_task
 *   csi_dsp_task  core 0 prio 5 : amplitude (esp-dsp) + moving variance
 *   lvgl_task     core 1 prio 3 : chart refresh capped at CYD_UI_FPS
 *
 * Memory policy:
 *   DSP arrays  -> static .bss (internal SRAM)
 *   LVGL framebuffers -> heap_caps_malloc(MALLOC_CAP_SPIRAM)  [stage-display]
 *   The two pools must never share an allocation path.
 */

#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"

#include "dsps_add.h"
#include "dsps_addc.h"
#include "dsps_mul.h"
#include "dsps_math.h"
#include "dsps_fft2r.h"
#include "dsps_wind_hann.h"
#include "lvgl.h"

#include "board_cyd.h"
#include "display.h"
#include "st77922.h"
#include "sd_log.h"

static const char *TAG = "cyd_stw";

/* ---------------- configuration ---------------- */
#define CSI_SUBCARRIERS 64      /* HT20 LLTF segment: 64 I/Q pairs */
#define CSI_PAYLOAD_MAX 512     /* clamp for queue struct-copy */
#define CSI_QUEUE_LEN   16
#define VAR_WINDOW      32      /* moving-variance window, in frames */

#define DSP_TASK_PRIO   5
#define DSP_TASK_STACK  8192
#define DSP_TASK_CORE   0
#define LVGL_TASK_PRIO  3
#define LVGL_TASK_STACK 8192
#define LVGL_TASK_CORE  1
#define CYD_UI_FPS      25
#define LV_TICK_MS      2

/* Stage 7: movement spectrogram FFT. Real-FFT over the last FFT_LEN
 * frames of mean iq_variance (scalar time series, same input the
 * Hampel/breathing pipeline already consumes) — NOT per-subcarrier;
 * one scalar magnitude series in, FFT_BINS frequency bins out.
 * fs is CSI_FRAME_RATE_HZ (nominal 15 Hz, see Stage 6b comment) =>
 * Nyquist ~7.5 Hz. The brief's "0-30 Hz" axis assumed a higher fs
 * than this pipeline actually has; labeling bins by their real
 * fs/N spacing rather than fabricating range. Recalibrate FFT_BINS'
 * Hz mapping if CSI_FRAME_RATE_HZ is ever measured and corrected. */
#define FFT_LEN         128     /* power of 2, dsps_fft2r requirement */
#define FFT_BINS        (FFT_LEN / 2)   /* 0 .. Nyquist, real signal */

/* ---------------- types ---------------- */
typedef struct {
    wifi_csi_info_t info;               /* struct copy; .buf repointed below */
    int8_t payload[CSI_PAYLOAD_MAX];    /* raw I/Q payload copy */
} csi_frame_t;

typedef struct {
    float variance[CSI_SUBCARRIERS];
    float hampel;       /* noise-gated iq_variance scalar (Stage 6) */
    float breath_bpm;   /* Stage 6b coarse breathing estimate (0 if invalid) */
    float breath_conf;  /* [0,1], 0 if invalid */
    bool  breath_valid;
    float fft_bins[FFT_BINS];  /* Stage 7: magnitude spectrum, 0..Nyquist */
    bool  fft_valid;
    uint32_t frames;
    uint32_t drops;
    int8_t rssi;
} dsp_result_t;

/* ---------------- shared state ---------------- */
static QueueHandle_t s_csi_queue;       /* csi_frame_t: wifi-task -> dsp */
static QueueHandle_t s_result_queue;    /* dsp_result_t mailbox (len 1, overwrite) */
static volatile uint32_t s_drop_count;  /* frames dropped on a full queue */
static char s_sta_ssid[33];             /* runtime SSID (wifi.cfg or Kconfig) */

/* DSP arrays — static .bss, guaranteed internal SRAM */
static float s_i[CSI_SUBCARRIERS], s_q[CSI_SUBCARRIERS];
static float s_i2[CSI_SUBCARRIERS], s_q2[CSI_SUBCARRIERS], s_mag[CSI_SUBCARRIERS];
static float s_hist[VAR_WINDOW][CSI_SUBCARRIERS];   /* magnitude ring buffer */
static float s_sum[CSI_SUBCARRIERS], s_sumsq[CSI_SUBCARRIERS];
static uint32_t s_hist_pos, s_hist_count;

/* ================================================================
 * CSI capture
 * ================================================================ */
static void csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    /* Runs in the Wi-Fi driver task context (not a hard ISR, but latency-
     * critical): struct-copy the payload, push, count-and-drop on full.
     * No malloc, no logging, no blocking. */
    csi_frame_t frame;
    size_t n = info->len;
    if (n > CSI_PAYLOAD_MAX) n = CSI_PAYLOAD_MAX;
    memcpy(&frame.info, info, sizeof(wifi_csi_info_t));
    memcpy(frame.payload, info->buf, n);
    frame.info.len = n;
    frame.info.buf = frame.payload;
    if (xQueueSend(s_csi_queue, &frame, 0) != pdTRUE) {
        s_drop_count++;
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_sta_ssid[0] != '\0') esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected");
        if (s_sta_ssid[0] != '\0') esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "associated; CSI stream live once traffic flows");
    }
}

static void wifi_csi_init(const char *ssid, const char *pass)
{
    strlcpy(s_sta_ssid, ssid, sizeof(s_sta_ssid));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    wifi_config_t sta = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    strlcpy((char *)sta.sta.ssid, ssid, sizeof(sta.sta.ssid));
    strlcpy((char *)sta.sta.password, pass, sizeof(sta.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* HT20 => 64-subcarrier FFT; CSI needs HT traffic (AMPDU enabled) */
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));

    wifi_csi_config_t csi = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    /* TODO(stage-traffic): if the AP is quiet, start a low-rate ping or
     * esp_wifi_80211_tx beacon loop so CSI frames actually arrive. */
}

/* ================================================================
 * DSP task — amplitude + moving variance
 * ================================================================ */
static void extract_amplitude(const csi_frame_t *f)
{
    /* LLTF segment: first 2*CSI_SUBCARRIERS bytes, int8 I/Q interleaved.
     * TODO(stage-dsp): also expose HT-LTF segment for finer granularity. */
    for (int k = 0; k < CSI_SUBCARRIERS; k++) {
        s_i[k] = (float)f->payload[2 * k];
        s_q[k] = (float)f->payload[2 * k + 1];
    }
    /* |H| = sqrt(I^2 + Q^2) via esp-dsp vector ops (S3 SIMD dispatch) */
    dsps_mul_f32(s_i, s_i, s_i2, CSI_SUBCARRIERS, 1, 1, 1);
    dsps_mul_f32(s_q, s_q, s_q2, CSI_SUBCARRIERS, 1, 1, 1);
    dsps_add_f32(s_i2, s_q2, s_mag, CSI_SUBCARRIERS, 1, 1, 1);
    dsps_sqrt_f32(s_mag, s_mag, CSI_SUBCARRIERS);
}

static void update_variance(float *variance_out)
{
    /* O(N) running variance over the ring window.
     * Numerically fine for 8-bit-sourced data; revisit if sums drift.
     * TODO(stage-dsp): Hampel pre-filter (window median +- k*MAD) to reject
     * impulsive noise before it enters the variance window. */
    if (s_hist_count > 0) {
        for (int k = 0; k < CSI_SUBCARRIERS; k++) {
            float old = s_hist[s_hist_pos][k];
            s_sum[k]   -= old;
            s_sumsq[k] -= old * old;
        }
    }
    for (int k = 0; k < CSI_SUBCARRIERS; k++) {
        float v = s_mag[k];
        s_hist[s_hist_pos][k] = v;
        s_sum[k]   += v;
        s_sumsq[k] += v * v;
    }
    s_hist_pos = (s_hist_pos + 1) % VAR_WINDOW;
    if (s_hist_count < VAR_WINDOW) s_hist_count++;
    float n = (float)s_hist_count;
    for (int k = 0; k < CSI_SUBCARRIERS; k++) {
        float mean = s_sum[k] / n;
        float var = s_sumsq[k] / n - mean * mean;
        variance_out[k] = var > 0.0f ? var : 0.0f;
    }
}

/* ================================================================
 * Stage 6: Hampel-style noise gate (median/MAD statistics) on the
 * scalar iq_variance series. INVERTED vs classic Hampel by design
 * (operator decision 2026-09-10): this is a passive radar — movement
 * spikes ARE the signal. Samples within median +- 3*MAD are clamped
 * TO the median (baseline micro-noise flattened); samples beyond the
 * threshold pass raw (movement preserved). Classic Hampel would do
 * the opposite and delete the movement.
 *
 * Window = last 32 RAW samples (~2 s @ 15 fps). Note: sustained events
 * (>2 s) re-anchor the rolling median — the gate then normalizes the
 * trace to the new level. Acceptable for display/denoise; the raw
 * column in the CSV is always untouched.
 * All-static buffers, no heap, ~5 us/frame.
 * ================================================================ */
#define HAMPEL_WIN      32
#define HAMPEL_NSIGMA   3.0f
#define MAD_TO_SIGMA    1.4826f  /* Gaussian consistency constant */

static float s_hbuf[HAMPEL_WIN];
static int s_hidx, s_hcount;
static float s_hwork[HAMPEL_WIN], s_hdev[HAMPEL_WIN];

static float hampel_median(const float *v, int n)
{
    memcpy(s_hwork, v, (size_t)n * sizeof(float));
    for (int i = 1; i < n; i++) {           /* insertion sort: n=32, ~1 us */
        float key = s_hwork[i];
        int j = i - 1;
        while (j >= 0 && s_hwork[j] > key) { s_hwork[j + 1] = s_hwork[j]; j--; }
        s_hwork[j + 1] = key;
    }
    return (n & 1) ? s_hwork[n / 2]
                   : 0.5f * (s_hwork[n / 2 - 1] + s_hwork[n / 2]);
}

static float hampel_update(float x)
{
    if (s_hcount < HAMPEL_WIN) {            /* warmup: pass through */
        s_hbuf[s_hidx] = x;
        s_hidx = (s_hidx + 1) % HAMPEL_WIN;
        s_hcount++;
        return x;
    }
    const float m = hampel_median(s_hbuf, HAMPEL_WIN);
    dsps_addc_f32(s_hbuf, s_hdev, HAMPEL_WIN, -m, 1, 1);   /* dev = x - m */
    for (int i = 0; i < HAMPEL_WIN; i++) s_hdev[i] = fabsf(s_hdev[i]);
    const float mad = hampel_median(s_hdev, HAMPEL_WIN);
    const float sigma = MAD_TO_SIGMA * mad;

    /* Noise gate: clamp inliers to the median (flatten baseline),
     * pass outliers raw (preserve movement). Two-sided: deep downward
     * excursions are real channel perturbations too. sigma==0 (perfectly
     * static window) clamps everything to the median — correct. */
    float out = m;
    if (fabsf(x - m) > HAMPEL_NSIGMA * sigma) out = x;

    s_hbuf[s_hidx] = x;                     /* window tracks RAW input */
    s_hidx = (s_hidx + 1) % HAMPEL_WIN;
    return out;
}

/* ================================================================
 * Stage 6b: Coarse breathing-rate extraction.
 *
 * Ported from ruvnet/RuView `wifi-densepose-wifiscan` crate
 * (`CoarseBreathingExtractor`, breathing_extractor.rs) — NOT the
 * marketing-copy "circular variance on wrapped phase" pipeline; the
 * actual shipped Rust is a 2nd-order IIR bandpass (bilinear-transform
 * Butterworth approx, 0.1-0.5 Hz) followed by zero-crossing rate
 * counting, run here on the Hampel-filtered iq_variance scalar
 * (post stage-6 gate, so the passive-radar movement spikes are the
 * oscillating signal, not per-subcarrier phase).
 *
 * CSI_FRAME_RATE_HZ is nominal (observed ~15-22 fps depending on AP
 * traffic, see STATUS.md Stage 5c); this filter's corner frequencies
 * are only as accurate as that assumption — recalibrate if the
 * measured frame rate drifts far from 15 Hz.
 *
 * esp-dsp has no biquad/zero-crossing helper suited to this; scalar
 * arithmetic here is ~10 float ops/frame, negligible vs the vector
 * ops already in extract_amplitude/update_variance.
 * ================================================================ */
#define BREATH_FREQ_LO_HZ   0.1f
#define BREATH_FREQ_HI_HZ   0.5f
#define CSI_FRAME_RATE_HZ   15.0f   /* nominal; see comment above */
#define BREATH_WINDOW_S     30
#define BREATH_MIN_S        10
#define BREATH_WINDOW_LEN   450   /* (int)(15.0f * 30) — CSI_FRAME_RATE_HZ * BREATH_WINDOW_S */
#define BREATH_MIN_LEN      150   /* (int)(15.0f * 10) — CSI_FRAME_RATE_HZ * BREATH_MIN_S */

typedef struct {
    float x1, x2, y1, y2;   /* IIR bandpass state */
} breath_iir_t;

static breath_iir_t s_breath_iir;
static float s_breath_hist[BREATH_WINDOW_LEN];
static int s_breath_head;      /* next write index (ring) */
static int s_breath_count;     /* samples seen, saturates at BREATH_WINDOW_LEN */

typedef struct {
    float bpm;
    float confidence;   /* [0,1] */
    bool valid;
} breath_estimate_t;

/* 2nd-order IIR bandpass, same bilinear-transform approx as the Rust
 * original: y[n] = (1-r)*(x[n]-x[n-2]) + 2*r*cos(w0)*y[n-1] - r^2*y[n-2] */
static float breath_bandpass(float input)
{
    const float omega_lo = 2.0f * (float)M_PI * BREATH_FREQ_LO_HZ / CSI_FRAME_RATE_HZ;
    const float omega_hi = 2.0f * (float)M_PI * BREATH_FREQ_HI_HZ / CSI_FRAME_RATE_HZ;
    const float bw = omega_hi - omega_lo;
    const float center = 0.5f * (omega_lo + omega_hi);
    const float r = 1.0f - bw / 2.0f;
    const float cos_w0 = cosf(center);

    breath_iir_t *st = &s_breath_iir;
    float output = (1.0f - r) * (input - st->x2) + 2.0f * r * cos_w0 * st->y1 - r * r * st->y2;
    st->x2 = st->x1;
    st->x1 = input;
    st->y2 = st->y1;
    st->y1 = output;
    return output;
}

static int breath_count_zero_crossings(const float *hist, int n)
{
    int crossings = 0;
    for (int i = 1; i < n; i++) {
        if (hist[i - 1] * hist[i] < 0.0f) crossings++;
    }
    return crossings;
}

static float breath_confidence(const float *hist, int n)
{
    if (n < 4) return 0.0f;
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += hist[i];
    mean /= (float)n;
    float variance = 0.0f, peak = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = hist[i] - mean;
        variance += d * d;
        float a = fabsf(hist[i]);
        if (a > peak) peak = a;
    }
    variance /= (float)n;
    if (variance < 1e-10f) return 0.0f;
    float noise = sqrtf(variance);
    float snr = (noise > 1e-10f) ? (peak / noise) : 0.0f;
    float conf = snr / 5.0f;
    return conf > 1.0f ? 1.0f : conf;
}

/* Feed one Hampel-filtered iq_variance sample; returns the latest
 * breathing estimate (valid=false until enough history + in-band). */
static breath_estimate_t breath_update(float hampel_sample)
{
    breath_estimate_t est = { .bpm = 0.0f, .confidence = 0.0f, .valid = false };

    float filtered = breath_bandpass(hampel_sample);

    s_breath_hist[s_breath_head] = filtered;
    s_breath_head = (s_breath_head + 1) % BREATH_WINDOW_LEN;
    if (s_breath_count < BREATH_WINDOW_LEN) s_breath_count++;

    if (s_breath_count < BREATH_MIN_LEN) return est;

    /* Ring buffer is not necessarily contiguous in time order for the
     * zero-crossing pass, but crossing COUNT is order-insensitive to
     * a rotation as long as we don't crossing-count across the wrap
     * seam twice; walking s_breath_hist in raw array order double
     * counts/misses at most one seam sample out of BREATH_WINDOW_LEN
     * (~450) — negligible for a rate estimate. */
    int n = s_breath_count;
    int crossings = breath_count_zero_crossings(s_breath_hist, n);
    float duration_s = (float)n / CSI_FRAME_RATE_HZ;
    float freq_hz = (float)crossings / (2.0f * duration_s);

    if (freq_hz < BREATH_FREQ_LO_HZ || freq_hz > BREATH_FREQ_HI_HZ) return est;

    est.bpm = freq_hz * 60.0f;
    est.confidence = breath_confidence(s_breath_hist, n);
    est.valid = true;
    return est;
}

/* ================================================================
 * Stage 7: movement spectrogram — real FFT (esp-dsp radix-2) over
 * a rolling window of the mean iq_variance scalar (same input as
 * Stage 6/6b, NOT per-subcarrier). One-time dsps_fft2r_init_fc32
 * call is required before any dsps_fft2r_fc32 use; done lazily on
 * first call (idempotent — esp-dsp's init just allocates twiddle
 * tables once).
 * ================================================================ */
static float s_fft_hist[FFT_LEN];      /* raw scalar time series, ring */
static int s_fft_head, s_fft_count;
static float s_fft_wind[FFT_LEN];      /* Hann window coefficients */
static float s_fft_work[FFT_LEN * 2];  /* complex-interleaved working buf */
static bool s_fft_inited;

static bool fft_update(float sample, float *bins_out)
{
    if (!s_fft_inited) {
        esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_LEN);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "dsps_fft2r_init_fc32 failed: %d", err);
            return false;
        }
        dsps_wind_hann_f32(s_fft_wind, FFT_LEN);
        s_fft_inited = true;
    }

    s_fft_hist[s_fft_head] = sample;
    s_fft_head = (s_fft_head + 1) % FFT_LEN;
    if (s_fft_count < FFT_LEN) s_fft_count++;
    if (s_fft_count < FFT_LEN) return false;   /* warmup */

    /* Ring buffer read out in raw array order (same seam-order caveat
     * as breath_update above — negligible for a spectral estimate over
     * a 128-sample window). Windowed real input, zero imaginary. */
    for (int i = 0; i < FFT_LEN; i++) {
        int idx = (s_fft_head + i) % FFT_LEN;
        s_fft_work[2 * i]     = s_fft_hist[idx] * s_fft_wind[i];
        s_fft_work[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(s_fft_work, FFT_LEN);
    dsps_bit_rev_fc32(s_fft_work, FFT_LEN);

    /* Single real channel (imaginary part was zero going in): the
     * complex FFT output is Hermitian-symmetric, so bins 0..N/2 are
     * the full non-redundant spectrum directly — no cplx2reC unpack
     * (that call is only for two REAL signals packed into one FFT,
     * which is not what this is). */
    for (int k = 0; k < FFT_BINS; k++) {
        float re = s_fft_work[2 * k];
        float im = s_fft_work[2 * k + 1];
        bins_out[k] = sqrtf(re * re + im * im) / (float)FFT_LEN;
    }
    return true;
}

static void csi_dsp_task(void *arg)
{
    csi_frame_t frame;
    dsp_result_t res = {0};
    uint32_t last_log_ms = 0;

    for (;;) {
        if (xQueueReceive(s_csi_queue, &frame, portMAX_DELAY) != pdTRUE) continue;
        if (frame.info.len < 2 * CSI_SUBCARRIERS) continue;  /* no LLTF segment */

        extract_amplitude(&frame);
        update_variance(res.variance);
        res.frames++;
        res.drops = s_drop_count;

        /* Stage 5c telemetry: struct by value into the ring buffer; the
         * sd_log task does all snprintf/fwrite off the hot path.
         * iq_variance = mean of the 64-bin variance (scalar for CSV).
         * Stage 6: Hampel-filtered twin of iq_variance (median +- 3*MAD,
         * 32-frame window) logged beside the raw value. */
        sd_log_rec_t rec = {
            .timestamp_ms = (uint32_t)esp_log_timestamp(),
            .iq_variance = 0.0f,
            .hampel_filtered_val = 0.0f,
            .dropped_frames = res.drops,
        };
        for (int k = 0; k < CSI_SUBCARRIERS; k++) rec.iq_variance += res.variance[k];
        rec.iq_variance /= CSI_SUBCARRIERS;
        rec.hampel_filtered_val = hampel_update(rec.iq_variance);
        sd_log_push(&rec);
        res.hampel = rec.hampel_filtered_val;

        breath_estimate_t breath = breath_update(rec.hampel_filtered_val);
        res.breath_valid = breath.valid;
        res.breath_bpm = breath.bpm;
        res.breath_conf = breath.confidence;

        res.fft_valid = fft_update(rec.hampel_filtered_val, res.fft_bins);
        res.rssi = frame.info.rx_ctrl.rssi;

        /* Mailbox semantics: UI always gets the freshest result */
        xQueueOverwrite(s_result_queue, &res);

        uint32_t now = esp_log_timestamp();
        if (now - last_log_ms > 5000) {
            last_log_ms = now;
            ESP_LOGI(TAG, "frames=%lu drops=%lu rssi=%d len=%d",
                     (unsigned long)res.frames, (unsigned long)res.drops,
                     frame.info.rx_ctrl.rssi, frame.info.len);
        }
    }
}

/* ================================================================
 * LVGL task — Aliens-style polar radar @ CYD_UI_FPS
 * Static radar_bg image (RGB565 native — the flush-time byte swap
 * handles panel order globally; a SWAPPED asset would double-swap).
 * Sweep + phosphor trail drawn in LV_EVENT_DRAW_MAIN on a transparent
 * full-screen overlay: zero extra framebuffer RAM.
 * ================================================================ */
static esp_timer_handle_t s_lv_tick_timer;
static lv_obj_t *s_radar;
static lv_obj_t *s_status;
static lv_obj_t *s_bpm_label;
static lv_obj_t *s_link_warn;
static uint32_t s_link_last_frame_seen;
static uint32_t s_link_last_change_ms;
#define CSI_STALL_MS   1500   /* no new frame for this long -> flag stall */
#define CSI_WEAK_RSSI  -80    /* dBm; below this, flag weak link (e.g. bad antenna) */

/* Tab navigation: tileview + GPIO0 (BOOT btn) cycle, debounced in lvgl_task
 * (LVGL is not ISR-safe; the ISR only sets a flag). */
static lv_obj_t *s_tileview;
static lv_obj_t *s_tile_spectrogram;
static lv_obj_t *s_tile_sector_meter;
static lv_obj_t *s_tile_pulse_monitor;
static volatile bool s_tab_btn_flag;
static uint32_t s_tab_last_switch_ms;
#define TAB_DEBOUNCE_MS 200
#define TAB_COUNT       3

LV_IMAGE_DECLARE(radar_bg);

#define RADAR_OX        160
#define RADAR_OY        400   /* bg PNG ring origin: 80px from screen bottom (480-80) */
#define RADAR_RMAX      300.0f
#define RADAR_RMIN      8.0f
#define RADAR_TRAIL     10
#define RADAR_STEP_DEG  4.0f
#define RADAR_VAR_FULL  17.0f   /* variance mapped to full radius */

/* ================================================================
 * Stage 7 visuals: spectrogram waterfall (Tab 0) + pulse monitor
 * (Tab 2). Both are lv_canvas buffers in PSRAM (too big for internal
 * SRAM budget already spent on LVGL draw buffers / WiFi). Canvas
 * memory is scroll-shifted with memmove — cheap relative to the S3's
 * clock, avoids per-pixel LVGL draw calls for the bulk of the frame.
 * ================================================================ */
#define SPEC_W          320
#define SPEC_H          200
#define PULSE_W         320
#define PULSE_H         160
#define GRAD_STEPS      32
#define SPEC_PERIOD_MS  100   /* brief: "scroll downward every 100ms" */

static lv_obj_t *s_spec_canvas;
static uint16_t *s_spec_buf;        /* PSRAM, SPEC_W*SPEC_H, native RGB565 (2B/px) */
static uint32_t s_spec_last_ms;

static lv_obj_t *s_pulse_canvas;
static uint16_t *s_pulse_buf;       /* PSRAM, PULSE_W*PULSE_H, native RGB565 (2B/px) */

static uint16_t s_grad_lut[GRAD_STEPS];  /* deep teal -> bright yellow/white, packed RGB565 */

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static void grad_lut_init(void)
{
    /* deep teal 0x003333 -> bright teal 0x00FFCC -> white/yellow 0xFFFFAA */
    for (int i = 0; i < GRAD_STEPS; i++) {
        float t = (float)i / (float)(GRAD_STEPS - 1);
        uint8_t r, g, b;
        if (t < 0.6f) {
            float u = t / 0.6f;
            r = 0;
            g = (uint8_t)(0x33 + u * (0xFF - 0x33));
            b = (uint8_t)(0x33 + u * (0xCC - 0x33));
        } else {
            float u = (t - 0.6f) / 0.4f;
            r = (uint8_t)(u * 0xFF);
            g = 0xFF;
            b = (uint8_t)(0xCC + u * (0xAA - 0xCC));
        }
        s_grad_lut[i] = rgb565(r, g, b);
    }
}

static void spectrogram_init(lv_obj_t *parent)
{
    s_spec_buf = heap_caps_malloc((size_t)SPEC_W * SPEC_H * sizeof(uint16_t),
                                  MALLOC_CAP_SPIRAM);
    if (!s_spec_buf) { ESP_LOGE(TAG, "spec canvas alloc failed"); return; }
    s_spec_canvas = lv_canvas_create(parent);
    lv_obj_set_size(s_spec_canvas, SPEC_W, SPEC_H);
    lv_obj_set_style_pad_all(s_spec_canvas, 0, 0);
    lv_obj_set_style_border_width(s_spec_canvas, 0, 0);
    lv_canvas_set_buffer(s_spec_canvas, s_spec_buf, SPEC_W, SPEC_H, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(s_spec_canvas, lv_color_hex(0x000000), LV_OPA_COVER);
    lv_obj_align(s_spec_canvas, LV_ALIGN_TOP_MID, 0, 0);
}

/* New FFT row painted at the top, previous rows shifted down one row
 * (y-axis = TIME, newest at top; x-axis = FREQUENCY, 0..Nyquist L->R).
 * Real Nyquist here is CSI_FRAME_RATE_HZ/2 (~7.5Hz nominal), not the
 * 30Hz the brief assumed — see FFT_BINS comment above. */
static void spectrogram_push(const float *bins)
{
    if (!s_spec_buf) return;
    uint32_t now = esp_log_timestamp();
    if (now - s_spec_last_ms < SPEC_PERIOD_MS) return;
    s_spec_last_ms = now;

    size_t row_bytes = (size_t)SPEC_W * sizeof(uint16_t);
    memmove(s_spec_buf + SPEC_W, s_spec_buf, row_bytes * (SPEC_H - 1));

    /* Skip bin0 (DC): dwarfs everything else and carries no motion
     * info. Normalize the remaining bins against their own frame max
     * so the display always shows relative structure regardless of
     * absolute CSI amplitude drift. */
    float maxv = 1e-6f;
    for (int k = 1; k < FFT_BINS; k++) if (bins[k] > maxv) maxv = bins[k];

    int px_per_bin = SPEC_W / FFT_BINS;
    for (int x = 0; x < SPEC_W; x++) {
        int k = x / px_per_bin;
        if (k < 1) k = 1;
        if (k >= FFT_BINS) k = FFT_BINS - 1;
        float norm = bins[k] / maxv;
        if (norm > 1.0f) norm = 1.0f;
        int step = (int)(norm * (GRAD_STEPS - 1));
        s_spec_buf[x] = s_grad_lut[step];
    }
    lv_obj_invalidate(s_spec_canvas);
}

static void pulse_init(lv_obj_t *parent)
{
    s_pulse_buf = heap_caps_malloc((size_t)PULSE_W * PULSE_H * sizeof(uint16_t),
                                   MALLOC_CAP_SPIRAM);
    if (!s_pulse_buf) { ESP_LOGE(TAG, "pulse canvas alloc failed"); return; }
    s_pulse_canvas = lv_canvas_create(parent);
    lv_obj_set_size(s_pulse_canvas, PULSE_W, PULSE_H);
    lv_obj_set_style_pad_all(s_pulse_canvas, 0, 0);
    lv_obj_set_style_border_width(s_pulse_canvas, 0, 0);
    lv_canvas_set_buffer(s_pulse_canvas, s_pulse_buf, PULSE_W, PULSE_H, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(s_pulse_canvas, lv_color_hex(0x000000), LV_OPA_COVER);
    lv_obj_align(s_pulse_canvas, LV_ALIGN_TOP_MID, 0, 0);
}

/* Scroll left one column per update (right-to-left ECG sweep). New
 * sample drawn at the rightmost column as a bright teal core plus a
 * dimmer vertical spread (glow) and the whole buffer is darkened
 * slightly each pass (phosphor-fade trail) rather than hard-cleared. */
static void pulse_push(float variance)
{
    if (!s_pulse_buf) return;

    for (int y = 0; y < PULSE_H; y++) {
        uint16_t *row = s_pulse_buf + (size_t)y * PULSE_W;
        memmove(row, row + 1, (size_t)(PULSE_W - 1) * sizeof(uint16_t));
    }

    float norm = variance / RADAR_VAR_FULL;
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    int core_y = (int)((1.0f - norm) * (PULSE_H - 1));   /* higher variance = higher on screen */

    for (int dy = -4; dy <= 4; dy++) {
        int y = core_y + dy;
        if (y < 0 || y >= PULSE_H) continue;
        float falloff = 1.0f - (float)abs(dy) / 5.0f;
        if (falloff < 0.0f) falloff = 0.0f;
        uint8_t g = (uint8_t)(0xFF * falloff);
        uint8_t b = (uint8_t)(0xCC * falloff);
        s_pulse_buf[(size_t)y * PULSE_W + (PULSE_W - 1)] = rgb565(0, g, b);
    }
    lv_obj_invalidate(s_pulse_canvas);
}

static float s_trail_ang[RADAR_TRAIL];  /* degrees, ring buffer */
static float s_trail_len[RADAR_TRAIL];
static int   s_trail_head;              /* newest entry */
static float s_sweep_deg;
static uint32_t s_radar_last_frame;
static float s_cur_variance;            /* latest filtered iq_variance, for sector fill */
static uint32_t s_radar_last_draw_ms;
#define RADAR_REDRAW_MS 200   /* ~5Hz repaint cap, independent of CSI frame rate */

#define SECTOR_COUNT    5
#define RING_COUNT      4      /* 5m/10m/15m/20m */

static void radar_draw_sectors(lv_layer_t *layer)
{
    /* 5 sectors spanning a 180-deg forward arc (Left to Right), each
     * filled with a radial teal gradient scaled by current variance —
     * approximated with concentric arc bands (LVGL has no native
     * radial-gradient arc fill) going from dim core to bright rim. */
    float norm = s_cur_variance / RADAR_VAR_FULL;
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    lv_draw_arc_dsc_t adsc;
    const float sector_span = 180.0f / SECTOR_COUNT;
    const int bands = 4;   /* was 12: 60 arcs/frame starved IDLE1 -> WDT reset */
    for (int s = 0; s < SECTOR_COUNT; s++) {
        float a0 = -90.0f + s * sector_span;
        float a1 = a0 + sector_span - 2.0f;   /* small gap between sectors */
        for (int b = 0; b < bands; b++) {
            float bt = (float)b / (float)(bands - 1);          /* 0=core .. 1=rim */
            float bright = bt * norm;                          /* brighter toward rim, scaled by variance */
            lv_draw_arc_dsc_init(&adsc);
            adsc.color = lv_color_make(0, (uint8_t)(0x33 + bright * (0xFF - 0x33)),
                                           (uint8_t)(0x33 + bright * (0xCC - 0x33)));
            adsc.opa = (lv_opa_t)(LV_OPA_10 + bright * (LV_OPA_70 - LV_OPA_10));
            adsc.width = (RADAR_RMAX - RADAR_RMIN) / bands + 1;
            adsc.start_angle = a0;
            adsc.end_angle = a1;
            adsc.center.x = RADAR_OX;
            adsc.center.y = RADAR_OY;
            adsc.radius = (int32_t)(RADAR_RMIN + bt * (RADAR_RMAX - RADAR_RMIN));
            lv_draw_arc(layer, &adsc);
        }
    }
}

static void radar_draw_rings(lv_layer_t *layer)
{
    /* Concentric range rings at 5/10/15/20 m, evenly spaced across
     * RADAR_RMIN..RADAR_RMAX (no independent range calibration input
     * exists yet — spacing is linear placeholder until one is added). */
    lv_draw_arc_dsc_t rdsc;
    for (int i = 1; i <= RING_COUNT; i++) {
        lv_draw_arc_dsc_init(&rdsc);
        rdsc.color = lv_color_hex(0x0F4D4D);
        rdsc.opa = LV_OPA_50;
        rdsc.width = 1;
        rdsc.start_angle = -90;
        rdsc.end_angle = 90;
        rdsc.center.x = RADAR_OX;
        rdsc.center.y = RADAR_OY;
        rdsc.radius = (int32_t)(RADAR_RMIN + (float)i / RING_COUNT * (RADAR_RMAX - RADAR_RMIN));
        lv_draw_arc(layer, &rdsc);
    }
}

static void radar_push(float filtered_var)
{
    s_cur_variance = filtered_var;
    s_sweep_deg += RADAR_STEP_DEG;
    if (s_sweep_deg >= 360.0f) s_sweep_deg -= 360.0f;
    s_trail_head = (s_trail_head + 1) % RADAR_TRAIL;
    s_trail_ang[s_trail_head] = s_sweep_deg;
    float len = RADAR_RMIN + (filtered_var / RADAR_VAR_FULL) * (RADAR_RMAX - RADAR_RMIN);
    if (len > RADAR_RMAX) len = RADAR_RMAX;
    s_trail_len[s_trail_head] = len;
}

static void radar_draw_cb(lv_event_t *e)
{
    lv_layer_t *layer = lv_event_get_layer(e);
    radar_draw_sectors(layer);
    radar_draw_rings(layer);
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_hex(0x00FFCC);   /* phosphor teal */
    dsc.width = 2;
    dsc.round_start = 1;
    dsc.round_end = 1;
    for (int i = 0; i < RADAR_TRAIL; i++) {
        const int idx = (s_trail_head - i + 2 * RADAR_TRAIL) % RADAR_TRAIL;
        const float a = s_trail_ang[idx] * (float)M_PI / 180.0f;
        const float len = s_trail_len[idx];
        dsc.opa = (lv_opa_t)(LV_OPA_COVER - (LV_OPA_COVER - LV_OPA_20) * i / (RADAR_TRAIL - 1));
        dsc.p1.x = RADAR_OX;
        dsc.p1.y = RADAR_OY;
        dsc.p2.x = RADAR_OX + len * cosf(a);
        dsc.p2.y = RADAR_OY + len * sinf(a);
        lv_draw_line(layer, &dsc);
    }
}

static void lv_tick_cb(void *arg) { lv_tick_inc(LV_TICK_MS); }

/* GPIO0 (BOOT btn) ISR: flag-only, debounced in lvgl_task. Shares silicon
 * with the boot-strap function but is only sampled post-boot — a user
 * holding BOOT during a future power-cycle still correctly re-enters
 * download mode (strap sampled once at reset, unaffected by this ISR). */
static void IRAM_ATTR tab_btn_isr(void *arg)
{
    s_tab_btn_flag = true;
}

static void tab_btn_gpio_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << CYD_BOOT_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CYD_BOOT_BTN, tab_btn_isr, NULL));
    ESP_LOGI(TAG, "tab-cycle ISR armed on GPIO%d", CYD_BOOT_BTN);
}

static void tab_btn_poll(void)
{
    if (!s_tab_btn_flag) return;
    s_tab_btn_flag = false;

    uint32_t now = esp_log_timestamp();
    if (now - s_tab_last_switch_ms < TAB_DEBOUNCE_MS) return;
    s_tab_last_switch_ms = now;

    uint32_t idx = lv_obj_get_index(lv_tileview_get_tile_active(s_tileview));
    idx = (idx + 1) % TAB_COUNT;
    lv_obj_t *tiles[TAB_COUNT] = {s_tile_spectrogram, s_tile_sector_meter,
                                  s_tile_pulse_monitor};
    lv_tileview_set_tile(s_tileview, tiles[idx], LV_ANIM_ON);
    ESP_LOGI(TAG, "tab -> %lu", (unsigned long)idx);
}

static void ui_build(void)
{
    /* Headless tileview: nav is button-driven, not swipe (avoids fighting
     * the GPIO0 debounce with touch/scroll gestures — this board has no
     * touch wired anyway, see board_cyd.h). */
    s_tileview = lv_tileview_create(lv_screen_active());
    lv_obj_set_size(s_tileview, CYD_LCD_H_RES, CYD_LCD_V_RES);
    lv_obj_set_pos(s_tileview, 0, 0);
    lv_obj_remove_flag(s_tileview, LV_OBJ_FLAG_SCROLLABLE);

    s_tile_spectrogram = lv_tileview_add_tile(s_tileview, 0, 0, LV_DIR_NONE);
    s_tile_sector_meter = lv_tileview_add_tile(s_tileview, 1, 0, LV_DIR_NONE);
    s_tile_pulse_monitor = lv_tileview_add_tile(s_tileview, 2, 0, LV_DIR_NONE);

    /* --- Tab 1 (index 1): Sector Proximity Meter — existing radar UI,
     * reparented onto its tile rather than duplicated. --- */
    lv_obj_t *bg = lv_image_create(s_tile_sector_meter);
    lv_image_set_src(bg, &radar_bg);
    lv_obj_set_pos(bg, 0, 0);

    s_radar = lv_obj_create(s_tile_sector_meter);
    lv_obj_set_size(s_radar, CYD_LCD_H_RES, CYD_LCD_V_RES);
    lv_obj_set_pos(s_radar, 0, 0);
    lv_obj_set_style_bg_opa(s_radar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_radar, 0, 0);
    lv_obj_remove_flag(s_radar, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_radar, radar_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_status = lv_label_create(s_tile_sector_meter);
    lv_obj_align(s_status, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_style_text_color(s_status, lv_color_hex(0xE0E0E0), 0);
    lv_label_set_text(s_status, "CYD_STW: waiting for CSI...");

    s_bpm_label = lv_label_create(s_tile_sector_meter);
    lv_obj_align(s_bpm_label, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_text_color(s_bpm_label, lv_color_hex(0x00FFCC), 0);

    s_link_warn = lv_label_create(s_tile_sector_meter);
    lv_obj_align(s_link_warn, LV_ALIGN_TOP_MID, 0, 6);
    lv_obj_set_style_text_color(s_link_warn, lv_color_hex(0xFF3333), 0);
    lv_obj_set_style_bg_color(s_link_warn, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(s_link_warn, LV_OPA_70, 0);
    lv_obj_set_style_pad_hor(s_link_warn, 6, 0);
    lv_obj_add_flag(s_link_warn, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(s_bpm_label, "BR: --");

    /* --- Tab 0: Movement Spectrogram (FFT waterfall canvas) --- */
    lv_obj_set_style_bg_color(s_tile_spectrogram, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(s_tile_spectrogram, LV_OPA_COVER, 0);
    grad_lut_init();
    spectrogram_init(s_tile_spectrogram);
    lv_obj_t *l1 = lv_label_create(s_tile_spectrogram);
    lv_obj_set_style_text_color(l1, lv_color_hex(0x808080), 0);
    lv_label_set_text_fmt(l1, "0-%d.%dHz (fs=%dHz nominal)",
                          (int)(CSI_FRAME_RATE_HZ / 2), (int)((CSI_FRAME_RATE_HZ / 2 - (int)(CSI_FRAME_RATE_HZ / 2)) * 10),
                          (int)CSI_FRAME_RATE_HZ);
    lv_obj_align(l1, LV_ALIGN_BOTTOM_MID, 0, -4);

    /* --- Tab 2: Enhanced Pulse Monitor (ECG-style canvas) --- */
    lv_obj_set_style_bg_color(s_tile_pulse_monitor, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(s_tile_pulse_monitor, LV_OPA_COVER, 0);
    pulse_init(s_tile_pulse_monitor);
    lv_obj_t *l2 = lv_label_create(s_tile_pulse_monitor);
    lv_obj_set_style_text_color(l2, lv_color_hex(0x808080), 0);
    lv_label_set_text(l2, "VARIANCE (filtered iq_variance)");
    lv_obj_align(l2, LV_ALIGN_BOTTOM_MID, 0, -4);

    /* Start on the sector meter (existing, populated visualization) */
    lv_tileview_set_tile(s_tileview, s_tile_sector_meter, LV_ANIM_OFF);

    tab_btn_gpio_init();
    s_link_last_change_ms = esp_log_timestamp();
}

static void ui_update(const dsp_result_t *r)
{
    uint32_t nowc = esp_log_timestamp();
    if (r->frames != s_link_last_frame_seen) {
        s_link_last_frame_seen = r->frames;
        s_link_last_change_ms = nowc;
    }
    bool stalled = (nowc - s_link_last_change_ms) > CSI_STALL_MS;
    bool weak = (r->rssi <= CSI_WEAK_RSSI);
    if (stalled) {
        lv_label_set_text(s_link_warn, LV_SYMBOL_WARNING " CSI STALLED - CHECK ANTENNA");
        lv_obj_clear_flag(s_link_warn, LV_OBJ_FLAG_HIDDEN);
    } else if (weak) {
        lv_label_set_text_fmt(s_link_warn, LV_SYMBOL_WARNING " WEAK LINK (%d dBm)", r->rssi);
        lv_obj_clear_flag(s_link_warn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_link_warn, LV_OBJ_FLAG_HIDDEN);
    }

    if (r->frames != s_radar_last_frame) {
        s_radar_last_frame = r->frames;
        radar_push(r->hampel);
        /* Throttle the sector-meter redraw: 60 lv_draw_arc calls/frame at
         * the full ~15Hz CSI rate starved IDLE1 -> task_wdt resets. State
         * (sweep angle, trail) still updates every frame; only the
         * expensive repaint is rate-limited. */
        uint32_t now_ms = esp_log_timestamp();
        if (now_ms - s_radar_last_draw_ms >= RADAR_REDRAW_MS) {
            s_radar_last_draw_ms = now_ms;
            lv_obj_invalidate(s_radar);
        }
        pulse_push(r->hampel);
    }
    if (r->fft_valid) {
        spectrogram_push(r->fft_bins);
    }
    lv_label_set_text_fmt(s_status, "frames=%lu drops=%lu",
                          (unsigned long)r->frames, (unsigned long)r->drops);

    if (r->breath_valid) {
        lv_label_set_text_fmt(s_bpm_label, "BR: %d bpm (%d%%)",
                              (int)(r->breath_bpm + 0.5f), (int)(r->breath_conf * 100.0f + 0.5f));
    } else {
        lv_label_set_text(s_bpm_label, "BR: --");
    }
}

static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "lvgl_task entered on core %d", xPortGetCoreID());
    lv_init();
    ESP_LOGI(TAG, "lv_init done");

    const esp_timer_create_args_t tick_args = {
        .callback = lv_tick_cb, .name = "lv_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &s_lv_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_lv_tick_timer, LV_TICK_MS * 1000));

    cyd_display_init();  /* ST77922 QSPI + DIRECT-mode PSRAM framebuffers */
    ui_build();

    dsp_result_t res;
    const TickType_t frame_ticks = pdMS_TO_TICKS(1000 / CYD_UI_FPS);
    uint32_t last_hb = 0;
    for (;;) {
        if (xQueueReceive(s_result_queue, &res, frame_ticks) == pdTRUE) {
            ui_update(&res);
        }
        tab_btn_poll();
        lv_timer_handler();
        vTaskDelay(1);   /* hard yield: guarantee IDLE1 gets a slot regardless of draw cost */
        /* Heartbeat: console is otherwise silent pre-association, and early
         * boot output is lost to USB re-enumeration (skill: flash pitfalls) */
        uint32_t now = esp_log_timestamp();
        if (now - last_hb > 5000) {
            last_hb = now;
            uint16_t lw, lh;
            cyd_display_last_flush(&lw, &lh);
            ESP_LOGI(TAG, "alive t=%lus flush=%lu tx=%lu txerr=%lu last=%ux%u",
                     (unsigned long)(now / 1000),
                     (unsigned long)cyd_display_flush_count(),
                     (unsigned long)st77922_tx_count(),
                     (unsigned long)st77922_tx_errors(), lw, lh);
        }
    }
}

/* ================================================================ */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_csi_queue = xQueueCreate(CSI_QUEUE_LEN, sizeof(csi_frame_t));
    s_result_queue = xQueueCreate(1, sizeof(dsp_result_t));
    configASSERT(s_csi_queue && s_result_queue);

    ESP_LOGI(TAG, "heap: internal=%u PSRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* LVGL draw buffers BEFORE the SDMMC mount: first claim on contiguous
     * internal DMA SRAM. Any failure is a boot-time assert, not a NULL
     * pointer handed to the SPI DMA. */
    cyd_display_prealloc();

    /* Stage 5c: SD card (wifi.cfg may carry the credentials).
     * All failures are non-fatal: display + CSI boot regardless. */
    sd_log_init();
    char ssid[33] = CONFIG_CYD_STW_WIFI_SSID;
    char pass[65] = CONFIG_CYD_STW_WIFI_PASSWORD;
    sd_log_wifi_creds(ssid, sizeof(ssid), pass, sizeof(pass));

    xTaskCreatePinnedToCore(csi_dsp_task, "csi_dsp", DSP_TASK_STACK, NULL,
                            DSP_TASK_PRIO, NULL, DSP_TASK_CORE);
    BaseType_t lv_ok = xTaskCreatePinnedToCore(lvgl_task, "lvgl", LVGL_TASK_STACK, NULL,
                            LVGL_TASK_PRIO, NULL, LVGL_TASK_CORE);
    ESP_LOGI(TAG, "lvgl_task create result: %d", (int)lv_ok);

    wifi_csi_init(ssid, pass);
}
