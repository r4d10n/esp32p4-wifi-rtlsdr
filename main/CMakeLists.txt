idf_component_register(
    SRCS "main.c" "../test/bench_fft.c" "../test/bench_dsp_pipeline.c" "../test/test_dsp_kernels.c" "../test/test_fm_stereo.c" "../test/test_fm_demod.c" "../test/test_pipeline_e2e.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb rtlsdr dsp fm_demod fm_stereo audio_out web_radio ui_hw esp_wifi esp_netif nvs_flash esp_event esp_timer mdns espressif__esp_hosted esp_eth driver espressif__esp-dsp
)
