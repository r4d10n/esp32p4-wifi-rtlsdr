idf_component_register(
    SRCS "decoder_adsb.c" "decoder_ais.c" "decoder_aprs.c" "decoder_gsm.c" "decoder_ft8.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log
)
