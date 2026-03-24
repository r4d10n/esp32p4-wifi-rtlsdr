idf_component_register(
    SRCS "rtlpower.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log json esp_http_client esp_ringbuf
)
