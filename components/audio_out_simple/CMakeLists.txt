idf_component_register(
    SRCS "audio_out_simple.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_common espressif__esp_codec_dev
)
