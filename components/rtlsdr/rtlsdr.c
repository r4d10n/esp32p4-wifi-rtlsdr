/*
 * ESP32-P4 RTL-SDR USB Host Driver
 *
 * Drives RTL2832U via USB 2.0 High-Speed vendor control transfers
 * and bulk IN reads. Ported from librtlsdr.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "usb/usb_host.h"
#include "rtlsdr.h"

static const char *TAG = "rtlsdr";

/* RTL2832U crystal frequency */
#define RTL_XTAL_FREQ   28800000

/* USB control transfer directions */
#define CTRL_IN     (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_IN)
#define CTRL_OUT    (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_OUT)

/* Number of async bulk transfer buffers */
#define DEFAULT_BUF_NUM     8
#define DEFAULT_BUF_LEN     (16 * 512)  /* 8KB per transfer */

/* Device state */
struct rtlsdr_dev {
    usb_host_client_handle_t client_hdl;
    usb_device_handle_t      dev_hdl;
    uint8_t                  iface_num;
    bool                     iface_claimed;

    /* Device info */
    rtlsdr_tuner_t  tuner_type;
    uint32_t        center_freq;
    uint32_t        sample_rate;
    int             gain;
    bool            agc_mode;
    int             freq_correction;
    int             direct_sampling;
    bool            offset_tuning;
    bool            bias_tee;

    /* Async streaming */
    volatile bool       async_running;
    rtlsdr_read_cb_t    async_cb;
    void               *async_ctx;
    usb_transfer_t     **xfers;
    uint32_t            xfer_num;
    uint32_t            xfer_len;
    SemaphoreHandle_t   xfer_sem;

    /* Control transfer synchronization */
    SemaphoreHandle_t   ctrl_sem;
    usb_transfer_t     *ctrl_xfer;
    esp_err_t           ctrl_status;
};

/* ──────────────────────── USB Control Transfers ──────────────────────── */

static void ctrl_xfer_cb(usb_transfer_t *xfer)
{
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)xfer->context;
    dev->ctrl_status = (xfer->status == USB_TRANSFER_STATUS_COMPLETED)
                       ? ESP_OK : ESP_FAIL;
    xSemaphoreGive(dev->ctrl_sem);
}

static esp_err_t rtlsdr_ctrl_transfer(rtlsdr_dev_t *dev, uint8_t bmRequestType,
                                       uint16_t wValue, uint16_t wIndex,
                                       uint8_t *data, uint16_t wLength)
{
    usb_transfer_t *xfer = dev->ctrl_xfer;

    xfer->device_handle = dev->dev_hdl;
    xfer->callback = ctrl_xfer_cb;
    xfer->context = dev;
    xfer->bEndpointAddress = 0;
    xfer->timeout_ms = RTLSDR_CTRL_TIMEOUT_MS;

    /* Fill setup packet */
    usb_setup_packet_t *setup = (usb_setup_packet_t *)xfer->data_buffer;
    setup->bmRequestType = bmRequestType;
    setup->bRequest = 0;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = wLength;

    if ((bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) == 0 && wLength > 0) {
        memcpy(xfer->data_buffer + sizeof(usb_setup_packet_t), data, wLength);
    }

    xfer->num_bytes = sizeof(usb_setup_packet_t) + wLength;

    esp_err_t ret = usb_host_transfer_submit_control(dev->client_hdl, xfer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Control transfer submit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (xSemaphoreTake(dev->ctrl_sem, pdMS_TO_TICKS(RTLSDR_CTRL_TIMEOUT_MS + 100)) != pdTRUE) {
        ESP_LOGE(TAG, "Control transfer timeout");
        return ESP_ERR_TIMEOUT;
    }

    if (dev->ctrl_status != ESP_OK) {
        return ESP_FAIL;
    }

    if ((bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) && wLength > 0) {
        memcpy(data, xfer->data_buffer + sizeof(usb_setup_packet_t), wLength);
    }

    return ESP_OK;
}

/* Read a register block */
static esp_err_t rtlsdr_read_reg(rtlsdr_dev_t *dev, uint16_t block,
                                  uint16_t addr, uint8_t *val, uint16_t len)
{
    uint16_t wIndex = block;
    uint16_t wValue = addr;
    return rtlsdr_ctrl_transfer(dev, CTRL_IN, wValue, wIndex, val, len);
}

/* Write a register block */
static esp_err_t rtlsdr_write_reg(rtlsdr_dev_t *dev, uint16_t block,
                                   uint16_t addr, const uint8_t *val, uint16_t len)
{
    uint16_t wIndex = (block) | 0x10;
    uint16_t wValue = addr;
    return rtlsdr_ctrl_transfer(dev, CTRL_OUT, wValue, wIndex, (uint8_t *)val, len);
}

/* Write a single 16-bit value to demod register */
static esp_err_t rtlsdr_demod_write_reg(rtlsdr_dev_t *dev, uint8_t page,
                                         uint16_t addr, uint16_t val, uint8_t len)
{
    uint8_t data[2];
    /* Set demod page */
    uint8_t page_val = page;
    esp_err_t ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_DEMOD, 0x00, &page_val, 1);
    if (ret != ESP_OK) return ret;

    if (len == 1) {
        data[0] = val & 0xFF;
    } else {
        data[0] = (val >> 8) & 0xFF;
        data[1] = val & 0xFF;
    }
    return rtlsdr_write_reg(dev, RTLSDR_BLOCK_DEMOD, addr, data, len);
}

static esp_err_t rtlsdr_demod_read_reg(rtlsdr_dev_t *dev, uint8_t page,
                                        uint16_t addr, uint8_t *val, uint8_t len)
{
    uint8_t page_val = page;
    esp_err_t ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_DEMOD, 0x00, &page_val, 1);
    if (ret != ESP_OK) return ret;
    return rtlsdr_read_reg(dev, RTLSDR_BLOCK_DEMOD, addr, val, len);
}

/* ──────────────────────── I2C Repeater (Tuner Access) ──────────────────────── */

static esp_err_t rtlsdr_set_i2c_repeater(rtlsdr_dev_t *dev, bool on)
{
    return rtlsdr_demod_write_reg(dev, 1, 0x01, on ? 0x18 : 0x10, 1);
}

/* ──────────────────────── RTL2832U Baseband Init ──────────────────────── */

/* FIR filter coefficients (default from librtlsdr) */
static const int fir_default[] = {
    -54, -36, -41, -40, -32, -14, 14, 53,
    101, 156, 215, 273, 327, 372, 404, 421
};

static esp_err_t rtlsdr_init_baseband(rtlsdr_dev_t *dev)
{
    esp_err_t ret;

    /* Initialize USB interface */
    uint8_t val;

    /* Dummy write to test USB link */
    val = 0x09;
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, 0x2000, &val, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "USB test write failed");

    /* Demod init */
    ret = rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1);  /* enable I2C repeater */
    ESP_RETURN_ON_ERROR(ret, TAG, "Demod init 1 failed");

    ret = rtlsdr_demod_write_reg(dev, 1, 0x15, 0x01, 1);  /* IF mode = zero-IF */
    ESP_RETURN_ON_ERROR(ret, TAG, "Demod init 2 failed");

    ret = rtlsdr_demod_write_reg(dev, 1, 0x16, 0x32, 1);  /* digital AGC */
    ESP_RETURN_ON_ERROR(ret, TAG, "Demod init 3 failed");

    /* Load FIR filter coefficients */
    uint8_t fir[20];
    for (int i = 0; i < 8; i++) {
        fir[i] = fir_default[i] & 0xFF;
    }
    for (int i = 0; i < 8; i += 2) {
        fir[8 + i] = ((fir_default[8 + i] >> 4) & 0x0F) | ((fir_default[8 + i + 1] << 4) & 0xF0);
        fir[8 + i + 1] = (fir_default[8 + i + 1] >> 4) & 0xFF;
    }
    /* Write FIR coefficients in chunks */
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_DEMOD, 0x1C, fir, 16);
    ESP_RETURN_ON_ERROR(ret, TAG, "FIR write failed");

    /* Enable spectrum inversion */
    ret = rtlsdr_demod_write_reg(dev, 1, 0x19, 0x05, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Spectrum inv failed");

    /* Enable digital AGC */
    ret = rtlsdr_demod_write_reg(dev, 1, 0x11, 0x03, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "DAGC enable failed");

    /* Set initial sample rate to 1.024 MSPS */
    uint32_t rsamp_ratio = ((uint64_t)RTL_XTAL_FREQ * (1 << 22)) / 1024000;
    ret = rtlsdr_demod_write_reg(dev, 1, 0x9F, (rsamp_ratio >> 16) & 0xFFFF, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Sample rate hi failed");
    ret = rtlsdr_demod_write_reg(dev, 1, 0xA1, rsamp_ratio & 0xFFFF, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Sample rate lo failed");

    ESP_LOGI(TAG, "Baseband initialized");
    return ESP_OK;
}

/* ──────────────────────── Tuner Detection ──────────────────────── */

/* Forward declaration for tuner init (in tuner_r82xx.c) */
extern esp_err_t r82xx_init(rtlsdr_dev_t *dev);
extern esp_err_t r82xx_set_freq(rtlsdr_dev_t *dev, uint32_t freq);
extern esp_err_t r82xx_set_gain(rtlsdr_dev_t *dev, int gain);
extern esp_err_t r82xx_set_gain_mode(rtlsdr_dev_t *dev, int manual);
extern esp_err_t r82xx_set_bandwidth(rtlsdr_dev_t *dev, uint32_t bw);

/* R820T I2C address */
#define R820T_I2C_ADDR  0x34
#define R828D_I2C_ADDR  0x74

static esp_err_t rtlsdr_probe_tuner(rtlsdr_dev_t *dev)
{
    esp_err_t ret;
    uint8_t val;

    /* Enable I2C repeater */
    ret = rtlsdr_set_i2c_repeater(dev, true);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2C repeater enable failed");

    /* Try R820T at address 0x34 */
    ret = rtlsdr_read_reg(dev, RTLSDR_BLOCK_IIC, R820T_I2C_ADDR, &val, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Found R820T/R820T2 tuner (reg0=0x%02x)", val);
        dev->tuner_type = RTLSDR_TUNER_R820T;
        rtlsdr_set_i2c_repeater(dev, false);
        return ESP_OK;
    }

    /* Try R828D at address 0x74 */
    ret = rtlsdr_read_reg(dev, RTLSDR_BLOCK_IIC, R828D_I2C_ADDR, &val, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Found R828D tuner (reg0=0x%02x)", val);
        dev->tuner_type = RTLSDR_TUNER_R828D;
        rtlsdr_set_i2c_repeater(dev, false);
        return ESP_OK;
    }

    /* TODO: probe E4000, FC0012, FC0013, FC2580 */
    ESP_LOGW(TAG, "No supported tuner found");
    dev->tuner_type = RTLSDR_TUNER_UNKNOWN;
    rtlsdr_set_i2c_repeater(dev, false);
    return ESP_ERR_NOT_FOUND;
}

/* ──────────────────────── USB Host Client ──────────────────────── */

static bool rtlsdr_is_target_device(const usb_device_desc_t *desc)
{
    if (desc->idVendor != RTLSDR_USB_VID) return false;
    return (desc->idProduct == RTLSDR_USB_PID_GENERIC ||
            desc->idProduct == RTLSDR_USB_PID_BLOG);
}

static void usb_client_event_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)arg;
    switch (msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(TAG, "New USB device connected (addr=%d)", msg->new_dev.address);
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGW(TAG, "USB device disconnected");
            if (dev->async_running) {
                dev->async_running = false;
            }
            break;
        default:
            break;
    }
}

/* ──────────────────────── Async Bulk Reads ──────────────────────── */

static void bulk_xfer_cb(usb_transfer_t *xfer)
{
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)xfer->context;

    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED && xfer->actual_num_bytes > 0) {
        if (dev->async_cb) {
            dev->async_cb(xfer->data_buffer, xfer->actual_num_bytes, dev->async_ctx);
        }
    } else if (xfer->status != USB_TRANSFER_STATUS_CANCELED) {
        ESP_LOGW(TAG, "Bulk IN status=%d bytes=%d", xfer->status, xfer->actual_num_bytes);
    }

    /* Resubmit if still running */
    if (dev->async_running) {
        esp_err_t ret = usb_host_transfer_submit(xfer);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Bulk resubmit failed: %s", esp_err_to_name(ret));
        }
    } else {
        xSemaphoreGive(dev->xfer_sem);
    }
}

/* ──────────────────────── Public API ──────────────────────── */

esp_err_t rtlsdr_init(rtlsdr_dev_t **out_dev)
{
    esp_err_t ret;

    rtlsdr_dev_t *dev = calloc(1, sizeof(rtlsdr_dev_t));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "alloc failed");

    dev->ctrl_sem = xSemaphoreCreateBinary();
    dev->xfer_sem = xSemaphoreCreateCounting(DEFAULT_BUF_NUM, 0);

    /* Allocate control transfer buffer */
    ret = usb_host_transfer_alloc(64 + sizeof(usb_setup_packet_t), 0, &dev->ctrl_xfer);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "ctrl xfer alloc failed");

    /* Register USB host client */
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usb_client_event_cb,
            .callback_arg = dev,
        },
    };
    ret = usb_host_client_register(&client_config, &dev->client_hdl);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "USB client register failed");

    /* Wait for device to connect and find RTL-SDR */
    ESP_LOGI(TAG, "Waiting for RTL-SDR device...");
    bool found = false;
    while (!found) {
        usb_host_client_handle_events(dev->client_hdl, pdMS_TO_TICKS(500));

        /* Enumerate connected devices */
        uint8_t dev_addr_list[8];
        int num_devs = 0;
        ret = usb_host_device_addr_list_fill(sizeof(dev_addr_list), dev_addr_list, &num_devs);
        if (ret != ESP_OK || num_devs == 0) continue;

        for (int i = 0; i < num_devs; i++) {
            usb_device_handle_t test_dev;
            ret = usb_host_device_open(dev->client_hdl, dev_addr_list[i], &test_dev);
            if (ret != ESP_OK) continue;

            const usb_device_desc_t *desc;
            ret = usb_host_get_device_desc(test_dev, &desc);
            if (ret == ESP_OK && rtlsdr_is_target_device(desc)) {
                ESP_LOGI(TAG, "RTL-SDR found! VID=0x%04x PID=0x%04x",
                         desc->idVendor, desc->idProduct);
                dev->dev_hdl = test_dev;
                found = true;
                break;
            }
            usb_host_device_close(dev->client_hdl, test_dev);
        }
    }

    /* Claim interface 0 */
    dev->iface_num = 0;
    ret = usb_host_interface_claim(dev->client_hdl, dev->dev_hdl, dev->iface_num, 0);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Interface claim failed");
    dev->iface_claimed = true;
    ESP_LOGI(TAG, "Interface 0 claimed");

    /* Initialize baseband */
    ret = rtlsdr_init_baseband(dev);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Baseband init failed");

    /* Detect tuner */
    ret = rtlsdr_probe_tuner(dev);
    if (ret == ESP_OK && (dev->tuner_type == RTLSDR_TUNER_R820T ||
                          dev->tuner_type == RTLSDR_TUNER_R828D)) {
        ret = r82xx_init(dev);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "Tuner init failed");
    }

    /* Set default config */
    dev->sample_rate = 1024000;
    dev->center_freq = 100000000;

    *out_dev = dev;
    ESP_LOGI(TAG, "RTL-SDR initialized (tuner=%d)", dev->tuner_type);
    return ESP_OK;

err:
    if (dev->iface_claimed) {
        usb_host_interface_release(dev->client_hdl, dev->dev_hdl, dev->iface_num);
    }
    if (dev->dev_hdl) {
        usb_host_device_close(dev->client_hdl, dev->dev_hdl);
    }
    if (dev->client_hdl) {
        usb_host_client_deregister(dev->client_hdl);
    }
    if (dev->ctrl_xfer) {
        usb_host_transfer_free(dev->ctrl_xfer);
    }
    if (dev->ctrl_sem) vSemaphoreDelete(dev->ctrl_sem);
    if (dev->xfer_sem) vSemaphoreDelete(dev->xfer_sem);
    free(dev);
    return ret;
}

esp_err_t rtlsdr_deinit(rtlsdr_dev_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    rtlsdr_stop_async(dev);

    if (dev->iface_claimed) {
        usb_host_interface_release(dev->client_hdl, dev->dev_hdl, dev->iface_num);
    }
    if (dev->dev_hdl) {
        usb_host_device_close(dev->client_hdl, dev->dev_hdl);
    }
    if (dev->client_hdl) {
        usb_host_client_deregister(dev->client_hdl);
    }
    if (dev->ctrl_xfer) {
        usb_host_transfer_free(dev->ctrl_xfer);
    }
    vSemaphoreDelete(dev->ctrl_sem);
    vSemaphoreDelete(dev->xfer_sem);
    free(dev);
    return ESP_OK;
}

esp_err_t rtlsdr_set_center_freq(rtlsdr_dev_t *dev, uint32_t freq)
{
    esp_err_t ret;

    ret = rtlsdr_set_i2c_repeater(dev, true);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2C repeater on failed");

    if (dev->tuner_type == RTLSDR_TUNER_R820T || dev->tuner_type == RTLSDR_TUNER_R828D) {
        ret = r82xx_set_freq(dev, freq);
    } else {
        ret = ESP_ERR_NOT_SUPPORTED;
    }

    rtlsdr_set_i2c_repeater(dev, false);
    if (ret == ESP_OK) {
        dev->center_freq = freq;
        ESP_LOGI(TAG, "Center freq: %lu Hz", (unsigned long)freq);
    }
    return ret;
}

esp_err_t rtlsdr_set_sample_rate(rtlsdr_dev_t *dev, uint32_t rate)
{
    /* Compute resampling ratio */
    uint32_t rsamp_ratio = ((uint64_t)RTL_XTAL_FREQ * (1 << 22)) / rate;
    uint32_t real_rate = ((uint64_t)RTL_XTAL_FREQ * (1 << 22)) / rsamp_ratio;

    esp_err_t ret;
    ret = rtlsdr_demod_write_reg(dev, 1, 0x9F, (rsamp_ratio >> 16) & 0xFFFF, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Sample rate hi failed");
    ret = rtlsdr_demod_write_reg(dev, 1, 0xA1, rsamp_ratio & 0xFFFF, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Sample rate lo failed");

    dev->sample_rate = real_rate;
    ESP_LOGI(TAG, "Sample rate: %lu Hz (requested %lu)", (unsigned long)real_rate, (unsigned long)rate);

    /* Set IF bandwidth on tuner */
    ret = rtlsdr_set_i2c_repeater(dev, true);
    if (ret == ESP_OK && (dev->tuner_type == RTLSDR_TUNER_R820T || dev->tuner_type == RTLSDR_TUNER_R828D)) {
        r82xx_set_bandwidth(dev, rate);
    }
    rtlsdr_set_i2c_repeater(dev, false);

    return ESP_OK;
}

esp_err_t rtlsdr_set_tuner_gain_mode(rtlsdr_dev_t *dev, int manual)
{
    esp_err_t ret = rtlsdr_set_i2c_repeater(dev, true);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2C repeater on failed");

    if (dev->tuner_type == RTLSDR_TUNER_R820T || dev->tuner_type == RTLSDR_TUNER_R828D) {
        ret = r82xx_set_gain_mode(dev, manual);
    }

    rtlsdr_set_i2c_repeater(dev, false);
    return ret;
}

esp_err_t rtlsdr_set_tuner_gain(rtlsdr_dev_t *dev, int gain)
{
    esp_err_t ret = rtlsdr_set_i2c_repeater(dev, true);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2C repeater on failed");

    if (dev->tuner_type == RTLSDR_TUNER_R820T || dev->tuner_type == RTLSDR_TUNER_R828D) {
        ret = r82xx_set_gain(dev, gain);
        if (ret == ESP_OK) dev->gain = gain;
    }

    rtlsdr_set_i2c_repeater(dev, false);
    return ret;
}

esp_err_t rtlsdr_set_agc_mode(rtlsdr_dev_t *dev, bool on)
{
    dev->agc_mode = on;
    return rtlsdr_demod_write_reg(dev, 0, 0x19, on ? 0x25 : 0x05, 1);
}

esp_err_t rtlsdr_set_freq_correction(rtlsdr_dev_t *dev, int ppm)
{
    dev->freq_correction = ppm;
    /* Re-apply center frequency with correction */
    if (dev->center_freq > 0) {
        return rtlsdr_set_center_freq(dev, dev->center_freq);
    }
    return ESP_OK;
}

esp_err_t rtlsdr_set_direct_sampling(rtlsdr_dev_t *dev, int on)
{
    dev->direct_sampling = on;
    /* TODO: implement direct sampling mux control */
    ESP_LOGW(TAG, "Direct sampling mode %d (not yet implemented)", on);
    return ESP_OK;
}

esp_err_t rtlsdr_set_offset_tuning(rtlsdr_dev_t *dev, bool on)
{
    dev->offset_tuning = on;
    /* TODO: implement offset tuning */
    ESP_LOGW(TAG, "Offset tuning %d (not yet implemented)", on);
    return ESP_OK;
}

esp_err_t rtlsdr_set_bias_tee(rtlsdr_dev_t *dev, bool on)
{
    /* GPIO 0 control via USB register */
    uint8_t val;
    esp_err_t ret = rtlsdr_read_reg(dev, RTLSDR_BLOCK_SYS, 0x0000, &val, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "GPIO read failed");

    if (on) {
        val |= 0x08;  /* Set GPIO 0 high */
    } else {
        val &= ~0x08; /* Set GPIO 0 low */
    }

    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, 0x0000, &val, 1);
    if (ret == ESP_OK) {
        dev->bias_tee = on;
        ESP_LOGI(TAG, "Bias tee: %s", on ? "ON" : "OFF");
    }
    return ret;
}

esp_err_t rtlsdr_set_if_gain(rtlsdr_dev_t *dev, int stage, int gain)
{
    /* TODO: implement IF gain control */
    ESP_LOGW(TAG, "IF gain stage=%d gain=%d (not yet implemented)", stage, gain);
    return ESP_OK;
}

esp_err_t rtlsdr_set_test_mode(rtlsdr_dev_t *dev, bool on)
{
    return rtlsdr_demod_write_reg(dev, 0, 0x19, on ? 0x03 : 0x05, 1);
}

esp_err_t rtlsdr_configure(rtlsdr_dev_t *dev, const rtlsdr_config_t *config)
{
    esp_err_t ret;

    ret = rtlsdr_set_sample_rate(dev, config->sample_rate);
    ESP_RETURN_ON_ERROR(ret, TAG, "Set sample rate failed");

    ret = rtlsdr_set_center_freq(dev, config->center_freq);
    ESP_RETURN_ON_ERROR(ret, TAG, "Set center freq failed");

    ret = rtlsdr_set_agc_mode(dev, config->agc_mode);
    ESP_RETURN_ON_ERROR(ret, TAG, "Set AGC failed");

    if (config->gain > 0) {
        rtlsdr_set_tuner_gain_mode(dev, 1);
        rtlsdr_set_tuner_gain(dev, config->gain);
    } else {
        rtlsdr_set_tuner_gain_mode(dev, 0);
    }

    rtlsdr_set_bias_tee(dev, config->bias_tee);
    rtlsdr_set_freq_correction(dev, config->freq_correction);

    return ESP_OK;
}

uint32_t rtlsdr_get_center_freq(rtlsdr_dev_t *dev) { return dev->center_freq; }
uint32_t rtlsdr_get_sample_rate(rtlsdr_dev_t *dev) { return dev->sample_rate; }
int      rtlsdr_get_tuner_gain(rtlsdr_dev_t *dev)  { return dev->gain; }
rtlsdr_tuner_t rtlsdr_get_tuner_type(rtlsdr_dev_t *dev) { return dev->tuner_type; }

/* R820T gain table (tenths of dB) */
static const int r82xx_gains[] = {
    0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
    166, 197, 207, 229, 254, 280, 297, 328,
    338, 364, 372, 386, 402, 421, 434, 439,
    445, 480, 496
};

const int *rtlsdr_get_tuner_gains(rtlsdr_dev_t *dev, int *count)
{
    if (dev->tuner_type == RTLSDR_TUNER_R820T || dev->tuner_type == RTLSDR_TUNER_R828D) {
        *count = sizeof(r82xx_gains) / sizeof(r82xx_gains[0]);
        return r82xx_gains;
    }
    *count = 0;
    return NULL;
}

esp_err_t rtlsdr_reset_buffer(rtlsdr_dev_t *dev)
{
    esp_err_t ret;
    ret = rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Reset buffer write failed");
    ret = rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Reset buffer clear failed");
    return ESP_OK;
}

esp_err_t rtlsdr_read_async(rtlsdr_dev_t *dev, rtlsdr_read_cb_t cb,
                            void *ctx, uint32_t buf_num, uint32_t buf_len)
{
    if (buf_num == 0) buf_num = DEFAULT_BUF_NUM;
    if (buf_len == 0) buf_len = DEFAULT_BUF_LEN;

    /* Ensure buf_len is multiple of 512 (HS bulk MPS) */
    buf_len = (buf_len / 512) * 512;
    if (buf_len == 0) buf_len = 512;

    dev->async_cb = cb;
    dev->async_ctx = ctx;
    dev->xfer_num = buf_num;
    dev->xfer_len = buf_len;

    /* Allocate transfers */
    dev->xfers = calloc(buf_num, sizeof(usb_transfer_t *));
    ESP_RETURN_ON_FALSE(dev->xfers, ESP_ERR_NO_MEM, TAG, "xfer array alloc failed");

    for (uint32_t i = 0; i < buf_num; i++) {
        esp_err_t ret = usb_host_transfer_alloc(buf_len, 0, &dev->xfers[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Bulk xfer %lu alloc failed", (unsigned long)i);
            /* Free already allocated */
            for (uint32_t j = 0; j < i; j++) {
                usb_host_transfer_free(dev->xfers[j]);
            }
            free(dev->xfers);
            dev->xfers = NULL;
            return ret;
        }
        dev->xfers[i]->device_handle = dev->dev_hdl;
        dev->xfers[i]->bEndpointAddress = RTLSDR_BULK_EP;
        dev->xfers[i]->num_bytes = buf_len;
        dev->xfers[i]->callback = bulk_xfer_cb;
        dev->xfers[i]->context = dev;
    }

    /* Reset buffer and start streaming */
    rtlsdr_reset_buffer(dev);
    dev->async_running = true;

    /* Submit all transfers */
    for (uint32_t i = 0; i < buf_num; i++) {
        esp_err_t ret = usb_host_transfer_submit(dev->xfers[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Bulk xfer %lu submit failed: %s",
                     (unsigned long)i, esp_err_to_name(ret));
            dev->async_running = false;
            break;
        }
    }

    /* Process USB events until stopped */
    while (dev->async_running) {
        usb_host_client_handle_events(dev->client_hdl, pdMS_TO_TICKS(100));
    }

    /* Wait for all transfers to complete/cancel */
    for (uint32_t i = 0; i < buf_num; i++) {
        xSemaphoreTake(dev->xfer_sem, pdMS_TO_TICKS(1000));
    }

    /* Free transfers */
    for (uint32_t i = 0; i < buf_num; i++) {
        usb_host_transfer_free(dev->xfers[i]);
    }
    free(dev->xfers);
    dev->xfers = NULL;

    return ESP_OK;
}

esp_err_t rtlsdr_stop_async(rtlsdr_dev_t *dev)
{
    dev->async_running = false;
    return ESP_OK;
}
