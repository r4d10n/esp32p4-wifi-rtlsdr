idf_component_register(
    SRCS "soapyremote.c"
    INCLUDE_DIRS "include"
    REQUIRES lwip freertos log esp_ringbuf mdns rtlsdr
)
