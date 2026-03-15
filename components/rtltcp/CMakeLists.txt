idf_component_register(
    SRCS "rtltcp.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES lwip rtlsdr esp_event
)
