idf_component_register(
    SRCS "websdr.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_https_server esp_http_server dsp rtlsdr esp_timer json
    # UI assets come from web/ via Vite build → components/websdr/dist/.
    # fm_player.html and dseg7.woff2 remain under www/ (legacy, served at /fm).
    EMBED_FILES "dist/index.html" "dist/sdr.js" "dist/sdr.css" "www/dseg7.woff2" "www/fm_player.html"
    EMBED_TXTFILES "certs/servercert.pem" "certs/prvtkey.pem"
)
