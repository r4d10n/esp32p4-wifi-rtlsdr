idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb rtlsdr gsm_kalibrate esp_wifi esp_netif nvs_flash esp_event esp_timer mdns espressif__esp_hosted esp_eth driver
)
