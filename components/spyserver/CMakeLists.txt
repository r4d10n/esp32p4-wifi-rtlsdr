idf_component_register(
    SRCS "spyserver.c"
    INCLUDE_DIRS "include"
    REQUIRES lwip freertos log esp_ringbuf rtlsdr
)
