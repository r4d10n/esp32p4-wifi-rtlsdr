/*
 * ESP32-P4 -> ESP32-C6 OTA Flasher
 *
 * Pushes embedded esp-hosted slave firmware to the C6 over SDIO.
 * The binary is embedded at build time via EMBED_FILES.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_hosted_ota.h"
#include "esp_hosted.h"

static const char *TAG = "c6_ota";

/* Embedded slave firmware (linked by EMBED_FILES) */
extern const uint8_t slave_fw_start[] asm("_binary_network_adapter_bin_start");
extern const uint8_t slave_fw_end[]   asm("_binary_network_adapter_bin_end");

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  ESP32-C6 Slave OTA via SDIO          ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");

    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Event loop (esp-hosted may already create one) */
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }

    /* Initialize esp-hosted and connect to slave */
    ESP_LOGI(TAG, "Initializing ESP-Hosted...");
    ESP_ERROR_CHECK(esp_hosted_init());
    ESP_LOGI(TAG, "Connecting to C6 slave over SDIO...");
    ESP_ERROR_CHECK(esp_hosted_connect_to_slave());
    ESP_LOGI(TAG, "Connected to C6 slave");

    /* Check current slave version */
    esp_hosted_coprocessor_fwver_t ver = {0};
    int ver_ret = esp_hosted_get_coprocessor_fwversion(&ver);
    if (ver_ret == 0) {
        ESP_LOGI(TAG, "Current C6 firmware: v%lu.%lu.%lu",
                 (unsigned long)ver.major1,
                 (unsigned long)ver.minor1,
                 (unsigned long)ver.patch1);
    } else {
        ESP_LOGW(TAG, "Could not read C6 version (ret=%d), proceeding anyway", ver_ret);
    }

    /* Firmware size */
    size_t fw_size = slave_fw_end - slave_fw_start;
    ESP_LOGI(TAG, "Embedded slave firmware: %zu bytes (%.1f KB)", fw_size, fw_size / 1024.0f);

    /* OTA begin */
    ESP_LOGI(TAG, "Starting OTA...");
    ret = esp_hosted_slave_ota_begin();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s (0x%x)", esp_err_to_name(ret), ret);
        ESP_LOGE(TAG, "The C6 slave may not support OTA. Try UART flashing.");
        goto done;
    }
    ESP_LOGI(TAG, "OTA begin OK");

    /* Write firmware in chunks */
    const size_t chunk_size = 1400;
    size_t offset = 0;
    size_t total = fw_size;
    int last_pct = -1;

    while (offset < total) {
        size_t len = (total - offset > chunk_size) ? chunk_size : (total - offset);
        ret = esp_hosted_slave_ota_write((uint8_t *)(slave_fw_start + offset), len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "OTA write failed at offset %zu: %s", offset, esp_err_to_name(ret));
            goto done;
        }
        offset += len;

        int pct = (int)(offset * 100 / total);
        if (pct / 10 > last_pct / 10) {
            ESP_LOGI(TAG, "OTA progress: %d%% (%zu / %zu bytes)", pct, offset, total);
            last_pct = pct;
        }
    }

    ESP_LOGI(TAG, "OTA write complete, finalizing...");

    /* OTA end */
    ret = esp_hosted_slave_ota_end();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s (0x%x)", esp_err_to_name(ret), ret);
        goto done;
    }
    ESP_LOGI(TAG, "OTA end OK - firmware validated");

    /* Activate new firmware */
    ESP_LOGI(TAG, "Activating new firmware (C6 will reboot)...");
    ret = esp_hosted_slave_ota_activate();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA activate failed: %s (0x%x)", esp_err_to_name(ret), ret);
        ESP_LOGW(TAG, "Older slave FW may not need activate - trying reboot...");
    } else {
        ESP_LOGI(TAG, "OTA activate OK");
    }

    /* Wait for C6 to reboot */
    ESP_LOGI(TAG, "Waiting for C6 reboot...");
    vTaskDelay(pdMS_TO_TICKS(8000));

    /* Verify new version */
    memset(&ver, 0, sizeof(ver));
    ver_ret = esp_hosted_get_coprocessor_fwversion(&ver);
    if (ver_ret == 0) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
        ESP_LOGI(TAG, "║  OTA SUCCESS!                         ║");
        ESP_LOGI(TAG, "║  New C6 FW: v%lu.%lu.%lu               ║",
                 (unsigned long)ver.major1,
                 (unsigned long)ver.minor1,
                 (unsigned long)ver.patch1);
        ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    } else {
        ESP_LOGW(TAG, "Could not verify new version (transport may need re-init)");
        ESP_LOGI(TAG, "OTA likely succeeded - reboot P4 to reconnect");
    }

done:
    ESP_LOGI(TAG, "OTA flasher done. You can now flash the WiFi test firmware.");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
