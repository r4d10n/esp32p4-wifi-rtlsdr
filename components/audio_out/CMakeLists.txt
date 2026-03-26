idf_component_register(
    SRCS "audio_out.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_common espressif__esp_codec_dev
)
