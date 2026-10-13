idf_component_register(
    SRCS "main.c" "display.c" "sd_log.c" "radar_bg.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_timer lvgl esp-dsp esp_lvgl_port st77922 board driver fatfs sdmmc
)
