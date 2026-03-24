idf_component_register(
    SRCS "decoder_registry.c"
         "decoder_bus.c"
         "decoder_tracking.c"
         "decoder_channel.c"
         "decoder_adsb.c"
         "decoder_ais.c"
         "decoder_gsm.c"
         "decoder_ft8.c"
         "decoder_ax25.c"
         "decoder_pocsag.c"
         "decoder_dtmf.c"
         "decoder_ctcss.c"
         "decoder_cw.c"
         "dsp_ddc.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log json esp_timer esp_http_client lwip
)
