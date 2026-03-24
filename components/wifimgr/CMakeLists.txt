set(srcs
    "wifimgr.c"
    "wifimgr_config.c"
    "wifimgr_api.c"
    "wifimgr_portal.c"
    "wifimgr_notify.c"
    "wifimgr_ethernet.c"
    "wifimgr_chatbot.c"
    "wifimgr_ota.c"
)

set(embed_files
    "www/portal.html"
    "www/portal.js"
    "www/portal.css"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash esp_wifi esp_netif esp_http_server esp_http_client
             esp_event lwip mdns json driver esp_eth mbedtls decoders
    PRIV_REQUIRES esp_https_server app_update esp_app_format
    EMBED_FILES ${embed_files}
)
