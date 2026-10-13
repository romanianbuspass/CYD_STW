idf_component_register(
    SRCS "st77922.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos board
)
