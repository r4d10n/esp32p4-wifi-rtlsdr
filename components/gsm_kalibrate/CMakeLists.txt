idf_component_register(
    SRCS "gsm_kalibrate.c"
    INCLUDE_DIRS "include"
    REQUIRES rtlsdr esp_http_server esp_timer
    PRIV_REQUIRES freertos heap log json
)
