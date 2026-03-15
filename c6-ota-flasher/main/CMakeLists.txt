idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_timer nvs_flash esp_event espressif__esp_hosted
    EMBED_FILES "network_adapter.bin"
)
