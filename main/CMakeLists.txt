idf_component_register(
    SRCS "main.c" "../test/bench_fft.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb rtlsdr rtltcp websdr wifimgr spyserver soapyremote esp_wifi esp_netif nvs_flash esp_event esp_timer mdns espressif__esp_hosted esp_eth driver espressif__esp-dsp json
)
