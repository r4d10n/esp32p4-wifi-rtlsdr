idf_component_register(
    SRCS "websdr.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_http_server dsp rtlsdr esp_timer json
    EMBED_FILES "www/index.html" "www/sdr.js" "www/sdr.css"
)
