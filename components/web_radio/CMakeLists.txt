idf_component_register(
    SRCS "web_radio.c" "nvs_settings.c"
    INCLUDE_DIRS "include" "."
    PRIV_REQUIRES esp_https_server esp_http_server json esp_timer esp_event nvs_flash app_update
    EMBED_FILES "www/index.html"
    EMBED_TXTFILES "certs/servercert.pem" "certs/prvtkey.pem"
)
