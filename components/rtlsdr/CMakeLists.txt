idf_component_register(
    SRCS "rtlsdr.c" "tuner_r82xx.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES usb esp_timer
)
