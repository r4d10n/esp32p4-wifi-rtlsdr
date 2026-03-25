idf_component_register(
    SRCS "usb_audio.c"
    INCLUDE_DIRS "include" "."
    REQUIRES esp_common
    PRIV_REQUIRES espressif__tinyusb
)

# TinyUSB's source files #include "tusb_config.h" — inject our directory
# into tinyusb's include path so it can find our tusb_config.h.
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
