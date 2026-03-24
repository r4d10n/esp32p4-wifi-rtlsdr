/*
 * WiFi Manager — OTA Firmware Update
 *
 * HTTP endpoint for uploading new firmware via the config portal.
 * Uses ESP-IDF's OTA API for safe dual-partition flashing.
 *
 * NOTE: For OTA to work, the partition table must have ota_0 and ota_1
 * partitions instead of a single factory partition. Migration partition table:
 *
 *   # Name,    Type, SubType,  Offset,    Size
 *   nvs,       data, nvs,      0x9000,    0x6000,
 *   otadata,   data, ota,      0xf000,    0x2000,
 *   phy_init,  data, phy,      0x11000,   0x1000,
 *   ota_0,     app,  ota_0,    0x20000,   0xE0000,
 *   ota_1,     app,  ota_1,    0x100000,  0xE0000,
 *   storage,   data, littlefs, 0x1E0000,  0x20000,
 *
 * With the current factory partition, OTA will validate but not flash.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_server.h"
#include "esp_app_format.h"
#include "esp_system.h"

static const char *TAG = "wifimgr_ota";

/* OTA upload handler — receives firmware binary via HTTP POST */
esp_err_t wifimgr_ota_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA update started, content length: %d", req->content_len);

    if (req->content_len <= 0 || req->content_len > 2 * 1024 * 1024) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Invalid firmware size\"}");
        return ESP_OK;
    }

    /* Find the next OTA partition */
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA partition found. Is partition table configured for OTA?");
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"No OTA partition available. Partition table needs ota_0/ota_1.\"}");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Writing to partition: %s (offset 0x%lx, size 0x%lx)",
             update_partition->label,
             (unsigned long)update_partition->address,
             (unsigned long)update_partition->size);

    /* Begin OTA */
    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"OTA begin failed\"}");
        return ESP_OK;
    }

    /* Receive and write firmware in chunks */
    char *buf = malloc(4096);
    if (!buf) {
        esp_ota_abort(ota_handle);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Out of memory\"}");
        return ESP_OK;
    }

    int received = 0;
    int total = req->content_len;
    bool header_checked = false;

    while (received < total) {
        int ret = httpd_req_recv(req, buf, 4096);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
            ESP_LOGE(TAG, "Receive error at %d/%d bytes", received, total);
            break;
        }

        /* Validate firmware header on first chunk */
        if (!header_checked && ret >= sizeof(esp_image_header_t)) {
            esp_image_header_t *hdr = (esp_image_header_t *)buf;
            if (hdr->magic != ESP_IMAGE_HEADER_MAGIC) {
                ESP_LOGE(TAG, "Invalid firmware image (bad magic: 0x%02x)", hdr->magic);
                esp_ota_abort(ota_handle);
                free(buf);
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Invalid firmware image\"}");
                return ESP_OK;
            }
            header_checked = true;
        }

        err = esp_ota_write(ota_handle, buf, ret);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA write failed at %d bytes: %s", received, esp_err_to_name(err));
            break;
        }

        received += ret;

        /* Progress logging every 64KB */
        if (received % (64 * 1024) < 4096) {
            ESP_LOGI(TAG, "OTA progress: %d/%d (%d%%)", received, total, received * 100 / total);
        }
    }

    free(buf);

    if (received != total || err != ESP_OK) {
        esp_ota_abort(ota_handle);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"OTA write incomplete\"}");
        return ESP_OK;
    }

    /* Finalize OTA */
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"OTA validation failed\"}");
        return ESP_OK;
    }

    /* Set boot partition */
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set boot partition failed: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Failed to set boot partition\"}");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "OTA successful! Rebooting to new firmware...");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"OTA complete. Rebooting...\"}");

    /* Delay to allow response to be sent, then reboot */
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK; /* unreachable */
}

/* Get OTA info endpoint */
esp_err_t wifimgr_ota_info_handler(httpd_req_t *req)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);

    esp_app_desc_t app_desc;
    esp_ota_get_partition_description(running, &app_desc);

    char resp[512];
    snprintf(resp, sizeof(resp),
        "{\"running_partition\":\"%s\","
        "\"boot_partition\":\"%s\","
        "\"next_ota_partition\":\"%s\","
        "\"app_version\":\"%s\","
        "\"idf_version\":\"%s\","
        "\"compile_date\":\"%s\","
        "\"compile_time\":\"%s\","
        "\"ota_available\":%s}",
        running ? running->label : "unknown",
        boot ? boot->label : "unknown",
        next ? next->label : "none",
        app_desc.version,
        app_desc.idf_ver,
        app_desc.date,
        app_desc.time,
        next ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}
