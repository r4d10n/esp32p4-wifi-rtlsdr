idf_component_register(
    SRCS "ui_hw.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_common audio_out
)
