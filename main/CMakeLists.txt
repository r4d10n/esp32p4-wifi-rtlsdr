idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb rtlsdr rtltcp esp_wifi esp_netif nvs_flash esp_event mdns
)
