idf_component_register(
    SRCS "fm_stereo.c" "rds_decoder.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES dsp esp_common esp_timer
)
