idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb rtlsdr fm_demod_simple audio_out_simple web_radio_simple esp_wifi esp_netif nvs_flash esp_event esp_timer mdns espressif__esp_hosted driver
)
