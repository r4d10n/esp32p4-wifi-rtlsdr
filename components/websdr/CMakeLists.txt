idf_component_register(
    SRCS "websdr.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_https_server esp_http_server dsp rtlsdr esp_timer json
    EMBED_FILES "www/index.html" "www/sdr.js" "www/sdr.css" "www/dseg7.woff2" "www/fm_player.html"
    EMBED_TXTFILES "certs/servercert.pem" "certs/prvtkey.pem"
)
