idf_component_register(
    SRCS "rtltcp.c" "rtludp.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES lwip rtlsdr esp_event esp_ringbuf esp_timer
)
