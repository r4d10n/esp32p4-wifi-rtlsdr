idf_component_register(
    SRCS "web_radio.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_http_server json esp_timer esp_event
    EMBED_FILES "www/index.html"
)
