idf_component_register(
    SRCS "rtlsdr.c" "tuner_r82xx.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES usb esp_timer freertos
)

# Suppress format warnings from uint16_t→long promotion on RISC-V with GCC 14
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format)
