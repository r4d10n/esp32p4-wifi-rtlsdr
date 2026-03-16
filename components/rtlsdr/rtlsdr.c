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

/* Max retries after EP0 STALL before giving up */
#define CTRL_STALL_RETRIES  2

/* Number of async bulk transfer buffers */
#define DEFAULT_BUF_NUM     6
#define DEFAULT_BUF_LEN     (32 * 512)  /* 16KB per transfer */

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
    SemaphoreHandle_t   ctrl_mutex;
    usb_transfer_t     *ctrl_xfer;
    esp_err_t           ctrl_status;

    /* Client event processing task */
    TaskHandle_t        client_task_hdl;
    volatile bool       client_task_running;
};

/* Background task to pump USB client events (needed for control transfer callbacks) */
static void usb_client_event_task(void *arg)
{
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)arg;
    while (dev->client_task_running) {
        usb_host_client_handle_events(dev->client_hdl, pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "USB client event task stopped");
    vTaskDelete(NULL);
}

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
    xSemaphoreTake(dev->ctrl_mutex, portMAX_DELAY);
    usb_transfer_t *xfer = dev->ctrl_xfer;

    for (int attempt = 0; attempt <= CTRL_STALL_RETRIES; attempt++) {
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
            xSemaphoreGive(dev->ctrl_mutex);
            return ret;
        }

        if (xSemaphoreTake(dev->ctrl_sem, pdMS_TO_TICKS(RTLSDR_CTRL_TIMEOUT_MS + 100)) != pdTRUE) {
            ESP_LOGE(TAG, "Control transfer timeout");
            xSemaphoreGive(dev->ctrl_mutex);
            return ESP_ERR_TIMEOUT;
        }

        if (dev->ctrl_status == ESP_OK) {
            /* Success — copy IN data and return */
            if ((bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) && wLength > 0) {
                memcpy(data, xfer->data_buffer + sizeof(usb_setup_packet_t), wLength);
            }
            xSemaphoreGive(dev->ctrl_mutex);
            return ESP_OK;
        }

        /* Transfer failed */
        if (xfer->status == USB_TRANSFER_STATUS_STALL) {
            /* Clear the STALL condition so EP0 can be used again */
            usb_host_endpoint_halt(dev->dev_hdl, 0);
            usb_host_endpoint_flush(dev->dev_hdl, 0);
            usb_host_endpoint_clear(dev->dev_hdl, 0);

            if (attempt < CTRL_STALL_RETRIES) {
                ESP_LOGW(TAG, "EP0 STALL (attempt %d/%d), retrying...",
                         attempt + 1, CTRL_STALL_RETRIES + 1);
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;  /* retry the transfer */
            }
        }

        /* Non-STALL failure or retries exhausted */
        ESP_LOGE(TAG, "Ctrl xfer fail: bmReqType=0x%02x wValue=0x%04x wIndex=0x%04x wLen=%d status=%d",
                 bmRequestType, wValue, wIndex, wLength, xfer->status);
        xSemaphoreGive(dev->ctrl_mutex);
        return ESP_FAIL;
    }

    xSemaphoreGive(dev->ctrl_mutex);
    return ESP_FAIL;
}

/*
 * Register access — matches librtlsdr encoding exactly:
 *   write_reg: wIndex = (block << 8) | 0x10, wValue = addr
 *   read_reg:  wIndex = (block << 8),         wValue = addr
 *   demod_write_reg: wIndex = page | 0x10,    wValue = (addr << 8) | 0x20
 */

/* Read a register block */
esp_err_t rtlsdr_read_reg(rtlsdr_dev_t *dev, uint16_t block,
                           uint16_t addr, uint8_t *val, uint16_t len)
{
    uint16_t wIndex = (block << 8);
    uint16_t wValue = addr;
    return rtlsdr_ctrl_transfer(dev, CTRL_IN, wValue, wIndex, val, len);
}

/* Write a register block */
esp_err_t rtlsdr_write_reg(rtlsdr_dev_t *dev, uint16_t block,
                            uint16_t addr, const uint8_t *val, uint16_t len)
{
    uint16_t wIndex = (block << 8) | 0x10;
    uint16_t wValue = addr;
    ESP_LOGD(TAG, "write_reg: block=%d addr=0x%04x len=%d => wValue=0x%04x wIndex=0x%04x data[0]=0x%02x",
             block, addr, len, wValue, wIndex, val[0]);
    return rtlsdr_ctrl_transfer(dev, CTRL_OUT, wValue, wIndex, (uint8_t *)val, len);
}

/* Write to demod register — uses separate encoding from write_reg */
static esp_err_t rtlsdr_demod_write_reg(rtlsdr_dev_t *dev, uint8_t page,
                                         uint16_t addr, uint16_t val, uint8_t len)
{
    uint8_t data[2];
    uint16_t wValue = (addr << 8) | 0x20;
    uint16_t wIndex = page | 0x10;

    if (len == 1) {
        data[0] = val & 0xff;
    } else {
        data[0] = (val >> 8) & 0xff;
    }
    data[1] = val & 0xff;

    ESP_LOGD(TAG, "demod_write: page=%d addr=0x%04x val=0x%04x len=%d => wValue=0x%04x wIndex=0x%04x data=%02x%02x",
             page, addr, val, len, wValue, wIndex, data[0], data[1]);

    /* Always send 2 data bytes (as librtlsdr allocates data[2]) */
    esp_err_t ret = rtlsdr_ctrl_transfer(dev, CTRL_OUT, wValue, wIndex, data, len);

    /* Dummy read after demod write (as librtlsdr does) */
    if (ret == ESP_OK) {
        uint8_t dummy[2];
        rtlsdr_ctrl_transfer(dev, CTRL_IN, (0x01 << 8) | 0x20, 0x0a, dummy, 1);
    }

    return ret;
}

/* Read from demod register */
static esp_err_t rtlsdr_demod_read_reg(rtlsdr_dev_t *dev, uint8_t page,
                                        uint16_t addr, uint8_t *val, uint8_t len)
{
    uint16_t wValue = (addr << 8) | 0x20;
    uint16_t wIndex = page;
    return rtlsdr_ctrl_transfer(dev, CTRL_IN, wValue, wIndex, val, len);
}

/* ──────────────────────── I2C Repeater (Tuner Access) ──────────────────────── */

esp_err_t rtlsdr_set_i2c_repeater(rtlsdr_dev_t *dev, bool on)
{
    return rtlsdr_demod_write_reg(dev, 1, 0x01, on ? 0x18 : 0x10, 1);
}

/* Public wrapper for demod_write_reg (used by tuner during calibration) */
esp_err_t rtlsdr_demod_write_reg_ext(rtlsdr_dev_t *dev, uint8_t page,
                                      uint16_t addr, uint16_t val, uint8_t len)
{
    return rtlsdr_demod_write_reg(dev, page, addr, val, len);
}

/* ──────────────────────── IF Frequency ──────────────────────── */

#define TWO_POW_22  (1 << 22)

esp_err_t rtlsdr_set_if_freq(rtlsdr_dev_t *dev, uint32_t freq)
{
    uint32_t rtl_xtal = RTL_XTAL_FREQ;
    int32_t if_freq;
    uint8_t tmp;
    esp_err_t ret;

    if_freq = ((int64_t)freq * TWO_POW_22 / rtl_xtal) * (-1);

    tmp = (if_freq >> 16) & 0x3f;
    ret = rtlsdr_demod_write_reg(dev, 1, 0x19, tmp, 1);
    if (ret != ESP_OK) return ret;

    tmp = (if_freq >> 8) & 0xff;
    ret = rtlsdr_demod_write_reg(dev, 1, 0x1a, tmp, 1);
    if (ret != ESP_OK) return ret;

    tmp = if_freq & 0xff;
    ret = rtlsdr_demod_write_reg(dev, 1, 0x1b, tmp, 1);

    ESP_LOGI(TAG, "IF freq: %lu Hz (if_reg=0x%06x)", (unsigned long)freq, if_freq & 0x3fffff);
    return ret;
}

/* ──────────────────────── GPIO Control ──────────────────────── */

/* SYS block GPIO register addresses */
#define GPO   0x3001
#define GPOE  0x3003
#define GPD   0x3004

esp_err_t rtlsdr_set_gpio_output(rtlsdr_dev_t *dev, uint8_t gpio)
{
    uint8_t val;
    uint8_t mask = 1 << gpio;
    esp_err_t ret;

    /* Set GPIO direction to output: clear bit in GPD */
    ret = rtlsdr_read_reg(dev, RTLSDR_BLOCK_SYS, GPD, &val, 1);
    if (ret != ESP_OK) return ret;
    val &= ~mask;
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, GPD, &val, 1);
    if (ret != ESP_OK) return ret;

    /* Enable GPIO output: set bit in GPOE */
    ret = rtlsdr_read_reg(dev, RTLSDR_BLOCK_SYS, GPOE, &val, 1);
    if (ret != ESP_OK) return ret;
    val |= mask;
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, GPOE, &val, 1);

    return ret;
}

esp_err_t rtlsdr_set_gpio_bit(rtlsdr_dev_t *dev, uint8_t gpio, int on)
{
    uint8_t val;
    uint8_t mask = 1 << gpio;
    esp_err_t ret;

    ret = rtlsdr_read_reg(dev, RTLSDR_BLOCK_SYS, GPO, &val, 1);
    if (ret != ESP_OK) return ret;

    if (on) {
        val |= mask;
    } else {
        val &= ~mask;
    }

    return rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, GPO, &val, 1);
}

esp_err_t rtlsdr_set_bias_tee_gpio(rtlsdr_dev_t *dev, int gpio, int on)
{
    esp_err_t ret;
    ret = rtlsdr_set_gpio_output(dev, (uint8_t)gpio);
    if (ret != ESP_OK) return ret;
    return rtlsdr_set_gpio_bit(dev, (uint8_t)gpio, on);
}

/* ──────────────────────── Blog V4 Detection ──────────────────────── */

/* TODO: Read USB string descriptors for proper detection.
 * For now, all R828D dongles are treated as Blog V4 (28.8 MHz xtal).
 * This works for Blog V4 but is wrong for standard R828D (16 MHz xtal). */
int rtlsdr_is_blog_v4(rtlsdr_dev_t *dev)
{
    /* R828D on Blog V4 uses same 28.8 MHz xtal as RTL2832U */
    return (rtlsdr_get_tuner_type(dev) == RTLSDR_TUNER_R828D) ? 1 : 0;
}

/* ──────────────────────── Sample Freq Correction ──────────────────────── */

static esp_err_t rtlsdr_set_sample_freq_correction(rtlsdr_dev_t *dev, int ppm)
{
    int16_t offs = (int16_t)(ppm * (-1) * TWO_POW_22 / 1000000);
    esp_err_t ret;

    ret = rtlsdr_demod_write_reg(dev, 1, 0x3f, offs & 0xff, 1);
    if (ret != ESP_OK) return ret;
    ret = rtlsdr_demod_write_reg(dev, 1, 0x3e, (offs >> 8) & 0x3f, 1);
    return ret;
}

/* ──────────────────────── RTL2832U Baseband Init ──────────────────────── */

/* FIR filter coefficients (default from librtlsdr) */
static const int fir_default[] = {
    -54, -36, -41, -40, -32, -14, 14, 53,
    101, 156, 215, 273, 327, 372, 404, 421
};

static esp_err_t rtlsdr_set_fir(rtlsdr_dev_t *dev)
{
    /* Pack FIR coefficients: 8 x 8-bit then 8 x 12-bit */
    uint8_t fir[20];
    int i;

    for (i = 0; i < 8; i++) {
        fir[i] = (uint8_t)(fir_default[i] & 0xff);
    }
    for (i = 0; i < 8; i += 2) {
        int val0 = fir_default[8 + i];
        int val1 = fir_default[8 + i + 1];
        fir[8 + i * 3 / 2]     = (val0 >> 4) & 0xff;
        fir[8 + i * 3 / 2 + 1] = ((val0 << 4) & 0xf0) | ((val1 >> 8) & 0x0f);
        fir[8 + i * 3 / 2 + 2] = val1 & 0xff;
    }

    /* Write FIR one byte at a time (as librtlsdr does) */
    for (i = 0; i < (int)sizeof(fir); i++) {
        esp_err_t ret = rtlsdr_demod_write_reg(dev, 1, 0x1c + i, fir[i], 1);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

static esp_err_t rtlsdr_init_baseband(rtlsdr_dev_t *dev)
{
    esp_err_t ret;
    uint8_t val;

    /* USB_SYSCTL = 0x09 (test / reset) */
    val = 0x09;
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, 0x2000, &val, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "USB SYSCTL write failed");

    /* USB_EPA_MAXPKT = 0x0002 (512 bytes for HS, librtlsdr default) */
    uint8_t maxpkt[2] = {0x00, 0x02};
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, 0x2158, maxpkt, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "EPA MAXPKT write failed");

    /* USB_EPA_CTL = 0x1002 */
    uint8_t epa_ctl[2] = {0x10, 0x02};
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, 0x2148, epa_ctl, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "EPA CTL write failed");

    /* DEMOD_CTL_1 (0x300b) = 0x22 */
    val = 0x22;
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, 0x300b, &val, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "DEMOD_CTL_1 failed");

    /* DEMOD_CTL (0x3000) = 0xe8 (release reset, enable ADC-Q) */
    val = 0xe8;
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, 0x3000, &val, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "DEMOD_CTL failed");

    /* Allow demod to come out of reset */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Verify SYS block by reading back DEMOD_CTL */
    {
        uint8_t rd_val = 0;
        ret = rtlsdr_read_reg(dev, RTLSDR_BLOCK_SYS, 0x3000, &rd_val, 1);
        ESP_LOGI(TAG, "DEMOD_CTL readback: ret=%d val=0x%02x (expect 0xe8)", ret, rd_val);

        rd_val = 0;
        ret = rtlsdr_read_reg(dev, RTLSDR_BLOCK_SYS, 0x300b, &rd_val, 1);
        ESP_LOGI(TAG, "DEMOD_CTL_1 readback: ret=%d val=0x%02x (expect 0x22)", ret, rd_val);

        /* Also try reading USB_SYSCTL to verify reads work */
        rd_val = 0;
        ret = rtlsdr_read_reg(dev, RTLSDR_BLOCK_USB, 0x2000, &rd_val, 1);
        ESP_LOGI(TAG, "USB_SYSCTL readback: ret=%d val=0x%02x (expect 0x09)", ret, rd_val);
    }

    /* Reset demod (reg 1:0x01 = 0x14, then 0x10) */
    ret = rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Demod reset 1 failed (ret=%d), continuing...", ret);
    }
    ret = rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Demod reset 2 failed (ret=%d), continuing...", ret);
    }

    /* Disable spectrum inversion and adjacent channel rejection */
    ret = rtlsdr_demod_write_reg(dev, 1, 0x15, 0x00, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Spectrum inv/adj disable failed");
    ret = rtlsdr_demod_write_reg(dev, 1, 0x16, 0x0000, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "ADC disable failed");

    /* Clear both DDC shift and IF frequency registers */
    for (int i = 0; i < 6; i++) {
        rtlsdr_demod_write_reg(dev, 1, 0x16 + i, 0x00, 1);
    }

    /* Load FIR filter */
    ret = rtlsdr_set_fir(dev);
    ESP_RETURN_ON_ERROR(ret, TAG, "FIR set failed");

    /* Enable SDR mode, disable DAGC (bit 5) */
    ret = rtlsdr_demod_write_reg(dev, 0, 0x19, 0x05, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "SDR mode failed");

    /* Init FSM state-holding register */
    ret = rtlsdr_demod_write_reg(dev, 1, 0x93, 0xf0, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "FSM 1 failed");
    ret = rtlsdr_demod_write_reg(dev, 1, 0x94, 0x0f, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "FSM 2 failed");

    /* Disable AGC */
    rtlsdr_demod_write_reg(dev, 1, 0x11, 0x00, 1);

    /* Disable RF and IF AGC loop */
    rtlsdr_demod_write_reg(dev, 1, 0x04, 0x00, 1);

    /* Disable PID filter */
    rtlsdr_demod_write_reg(dev, 0, 0x61, 0x60, 1);

    /* Default ADC_I/ADC_Q datapath */
    rtlsdr_demod_write_reg(dev, 0, 0x06, 0x80, 1);

    /* Enable Zero-IF mode, DC cancellation, IQ estimation/compensation */
    rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1b, 1);

    /* Disable 4.096 MHz clock output on TP_CK0 */
    rtlsdr_demod_write_reg(dev, 0, 0x0d, 0x83, 1);

    /* Set initial sample rate to 1.024 MSPS */
    uint32_t rsamp_ratio = ((uint64_t)RTL_XTAL_FREQ * (1 << 22)) / 1024000;
    rsamp_ratio &= 0x0ffffffc;
    ret = rtlsdr_demod_write_reg(dev, 1, 0x9f, (rsamp_ratio >> 16) & 0xffff, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Sample rate hi failed");
    ret = rtlsdr_demod_write_reg(dev, 1, 0xa1, rsamp_ratio & 0xffff, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Sample rate lo failed");

    /* Reset demod again */
    ret = rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Demod re-reset 1 failed");
    ret = rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Demod re-reset 2 failed");

    ESP_LOGI(TAG, "Baseband initialized (match librtlsdr sequence)");
    return ESP_OK;
}

/* ──────────────────────── Tuner Detection ──────────────────────── */

/* Forward declaration for tuner init (in tuner_r82xx.c) */
extern esp_err_t r82xx_init(rtlsdr_dev_t *dev);
extern esp_err_t r82xx_set_freq(rtlsdr_dev_t *dev, uint32_t freq);
extern esp_err_t r82xx_set_gain(rtlsdr_dev_t *dev, int gain);
extern esp_err_t r82xx_set_gain_mode(rtlsdr_dev_t *dev, int manual);
extern int       r82xx_set_bandwidth(rtlsdr_dev_t *dev, int bw, uint32_t rate);

/* R820T I2C address */
#define R820T_I2C_ADDR  0x34
#define R828D_I2C_ADDR  0x74

/* R82XX check register for tuner detection */
#define R82XX_CHECK_ADDR    0x00
#define R82XX_CHECK_VAL     0x69

static esp_err_t rtlsdr_i2c_read_reg(rtlsdr_dev_t *dev, uint8_t i2c_addr,
                                      uint8_t reg, uint8_t *val)
{
    /* For I2C read on RTL2832U: write addr byte, then read data */
    esp_err_t ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_IIC, i2c_addr, &reg, 1);
    if (ret != ESP_OK) return ret;
    return rtlsdr_read_reg(dev, RTLSDR_BLOCK_IIC, i2c_addr, val, 1);
}

static esp_err_t rtlsdr_probe_tuner(rtlsdr_dev_t *dev)
{
    esp_err_t ret;
    uint8_t val;

    /* Enable I2C repeater */
    ret = rtlsdr_set_i2c_repeater(dev, true);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2C repeater enable failed");

    /* Try R828D at address 0x74 (check first — RTL-SDR V4 uses this) */
    ret = rtlsdr_i2c_read_reg(dev, R828D_I2C_ADDR, R82XX_CHECK_ADDR, &val);
    if (ret == ESP_OK && val == R82XX_CHECK_VAL) {
        ESP_LOGI(TAG, "Found R828D tuner (reg0=0x%02x)", val);
        dev->tuner_type = RTLSDR_TUNER_R828D;
        rtlsdr_set_i2c_repeater(dev, false);
        return ESP_OK;
    }

    /* Try R820T at address 0x34 */
    ret = rtlsdr_i2c_read_reg(dev, R820T_I2C_ADDR, R82XX_CHECK_ADDR, &val);
    if (ret == ESP_OK && val == R82XX_CHECK_VAL) {
        ESP_LOGI(TAG, "Found R820T/R820T2 tuner (reg0=0x%02x)", val);
        dev->tuner_type = RTLSDR_TUNER_R820T;
        rtlsdr_set_i2c_repeater(dev, false);
        return ESP_OK;
    }

    /* TODO: probe E4000, FC0012, FC0013, FC2580 */
    ESP_LOGW(TAG, "No supported tuner found (last read=0x%02x)", val);
    dev->tuner_type = RTLSDR_TUNER_UNKNOWN;
    rtlsdr_set_i2c_repeater(dev, false);
    return ESP_ERR_NOT_FOUND;
}

/* ──────────────────────── USB Host Client ──────────────────────── */

static bool rtlsdr_is_target_device(const usb_device_desc_t *desc)
{
    if (desc->idVendor == RTLSDR_USB_VID) {
        /* Match any known RTL2832U PID */
        return (desc->idProduct == RTLSDR_USB_PID_GENERIC ||
                desc->idProduct == RTLSDR_USB_PID_BLOG ||
                desc->idProduct == 0x2838);  /* ezcap / Blog V4 */
    }
    return false;
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

static uint32_t bulk_cb_count = 0;
static uint32_t bulk_bytes_total = 0;
static uint32_t bulk_start_tick = 0;

static void bulk_xfer_cb(usb_transfer_t *xfer)
{
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)xfer->context;

    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED && xfer->actual_num_bytes > 0) {
        bulk_cb_count++;
        bulk_bytes_total += xfer->actual_num_bytes;
        if (bulk_cb_count == 1) {
            bulk_start_tick = xTaskGetTickCount();
        }
        if (bulk_cb_count <= 3 || (bulk_cb_count % 5000) == 0) {
            uint32_t elapsed_ms = (xTaskGetTickCount() - bulk_start_tick) * portTICK_PERIOD_MS;
            if (elapsed_ms > 0) {
                uint32_t rate_kbps = (uint32_t)((uint64_t)bulk_bytes_total * 1000 / 1024 / elapsed_ms);
                ESP_LOGI(TAG, "USB: #%lu %dB (total %lu KB in %lums = %lu KB/s = %lu kSPS)",
                         (unsigned long)bulk_cb_count, xfer->actual_num_bytes,
                         (unsigned long)(bulk_bytes_total / 1024),
                         (unsigned long)elapsed_ms,
                         (unsigned long)rate_kbps,
                         (unsigned long)(rate_kbps * 1024 / 2 / 1000));
            }
        }
        if (dev->async_cb) {
            dev->async_cb(xfer->data_buffer, xfer->actual_num_bytes, dev->async_ctx);
        }
    } else if (xfer->status != USB_TRANSFER_STATUS_CANCELED) {
        ESP_LOGW(TAG, "Bulk IN status=%d bytes=%d (cb#%lu)",
                 xfer->status, xfer->actual_num_bytes, (unsigned long)bulk_cb_count);
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
    dev->ctrl_mutex = xSemaphoreCreateMutex();
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

    /* Start background task to pump client events (needed for ctrl xfer callbacks) */
    dev->client_task_running = true;
    xTaskCreatePinnedToCore(usb_client_event_task, "usb_client", 4096,
                            dev, 10, &dev->client_task_hdl, 0);

    /* Wait for device to connect and find RTL-SDR */
    ESP_LOGI(TAG, "Waiting for RTL-SDR device...");
    bool found = false;
    while (!found) {
        vTaskDelay(pdMS_TO_TICKS(500));

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
            ret = usb_host_get_device_descriptor(test_dev, &desc);
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

    /* Print device descriptors for debugging */
    {
        const usb_device_desc_t *ddesc;
        usb_host_get_device_descriptor(dev->dev_hdl, &ddesc);
        ESP_LOGI(TAG, "Device: VID=0x%04x PID=0x%04x class=%d subclass=%d proto=%d bcdUSB=0x%04x",
                 ddesc->idVendor, ddesc->idProduct, ddesc->bDeviceClass,
                 ddesc->bDeviceSubClass, ddesc->bDeviceProtocol, ddesc->bcdUSB);
        ESP_LOGI(TAG, "  bNumConfigurations=%d bMaxPacketSize0=%d",
                 ddesc->bNumConfigurations, ddesc->bMaxPacketSize0);

        const usb_config_desc_t *cdesc;
        ret = usb_host_get_active_config_descriptor(dev->dev_hdl, &cdesc);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Config: bNumInterfaces=%d bConfigurationValue=%d wTotalLength=%d",
                     cdesc->bNumInterfaces, cdesc->bConfigurationValue, cdesc->wTotalLength);
            /* Walk interfaces */
            int offset = 0;
            const uint8_t *p = (const uint8_t *)cdesc;
            while (offset < cdesc->wTotalLength) {
                uint8_t bLength = p[offset];
                uint8_t bDescType = p[offset + 1];
                if (bLength == 0) break;
                if (bDescType == 0x04) { /* Interface */
                    ESP_LOGI(TAG, "  Interface: num=%d alt=%d numEP=%d class=%d",
                             p[offset+2], p[offset+3], p[offset+4], p[offset+5]);
                } else if (bDescType == 0x05) { /* Endpoint */
                    ESP_LOGI(TAG, "  Endpoint: addr=0x%02x attr=0x%02x maxpkt=%d",
                             p[offset+2], p[offset+3], p[offset+4] | (p[offset+5] << 8));
                }
                offset += bLength;
            }
        }
    }

    /* Claim both interfaces (RTL2832U has 2 interfaces) */
    dev->iface_num = 0;
    ret = usb_host_interface_claim(dev->client_hdl, dev->dev_hdl, 0, 0);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Interface 0 claim failed");
    ESP_LOGI(TAG, "Interface 0 claimed");

    /* Also claim interface 1 (needed for demod register access on some variants) */
    ret = usb_host_interface_claim(dev->client_hdl, dev->dev_hdl, 1, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Interface 1 claimed");
    } else {
        ESP_LOGW(TAG, "Interface 1 claim failed (ret=%d), continuing...", ret);
    }
    dev->iface_claimed = true;

    /* Initialize baseband */
    ret = rtlsdr_init_baseband(dev);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Baseband init failed");

    /* Detect tuner */
    ret = rtlsdr_probe_tuner(dev);
    if (ret == ESP_OK && (dev->tuner_type == RTLSDR_TUNER_R820T ||
                          dev->tuner_type == RTLSDR_TUNER_R828D)) {

        /* R820T/R828D post-detection demod config (from rtl-sdr-blog librtlsdr.c) */
        rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1a, 1);  /* disable Zero-IF */
        rtlsdr_demod_write_reg(dev, 0, 0x08, 0x4d, 1);  /* only In-phase ADC input */
        rtlsdr_set_if_freq(dev, 3570000);                 /* R82XX IF = 3.57 MHz */
        rtlsdr_demod_write_reg(dev, 1, 0x15, 0x01, 1);  /* enable spectrum inversion */

        /* Initialize GPIO 4 as output (used by some dongles) */
        rtlsdr_set_gpio_output(dev, 4);

        /* Enable I2C repeater for tuner access */
        rtlsdr_set_i2c_repeater(dev, true);
        ret = r82xx_init(dev);
        rtlsdr_set_i2c_repeater(dev, false);
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
    if (dev->ctrl_mutex) vSemaphoreDelete(dev->ctrl_mutex);
    if (dev->ctrl_sem) vSemaphoreDelete(dev->ctrl_sem);
    if (dev->xfer_sem) vSemaphoreDelete(dev->xfer_sem);
    free(dev);
    return ret;
}

esp_err_t rtlsdr_deinit(rtlsdr_dev_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    rtlsdr_stop_async(dev);

    /* Stop client event task */
    dev->client_task_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));

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
    vSemaphoreDelete(dev->ctrl_mutex);
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
    esp_err_t ret;

    if (rate == 0 || rate > 3200000) {
        ESP_LOGE(TAG, "Invalid sample rate: %lu", (unsigned long)rate);
        return ESP_ERR_INVALID_ARG;
    }

    /* Set tuner bandwidth BEFORE sample rate (reference order) */
    ret = rtlsdr_set_i2c_repeater(dev, true);
    if (ret == ESP_OK && (dev->tuner_type == RTLSDR_TUNER_R820T || dev->tuner_type == RTLSDR_TUNER_R828D)) {
        int if_freq = r82xx_set_bandwidth(dev, rate > 0 ? (int)rate : 1024000, rate);
        rtlsdr_set_i2c_repeater(dev, false);

        /* Update IF frequency if bandwidth changed it */
        if (if_freq > 0) {
            rtlsdr_set_if_freq(dev, (uint32_t)if_freq);
        }
    } else {
        rtlsdr_set_i2c_repeater(dev, false);
    }

    /* Compute resampling ratio */
    uint32_t rsamp_ratio = ((uint64_t)RTL_XTAL_FREQ * (1 << 22)) / rate;
    rsamp_ratio &= 0x0ffffffc;  /* Low 2 bits must be zero per hardware spec */

    uint32_t real_rsamp_ratio = rsamp_ratio | ((rsamp_ratio & 0x08000000) << 1);
    uint32_t real_rate = ((uint64_t)RTL_XTAL_FREQ * (1 << 22)) / real_rsamp_ratio;

    ret = rtlsdr_demod_write_reg(dev, 1, 0x9f, (rsamp_ratio >> 16) & 0xffff, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Sample rate hi failed");
    ret = rtlsdr_demod_write_reg(dev, 1, 0xa1, rsamp_ratio & 0xffff, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Sample rate lo failed");

    /* Apply sample frequency correction */
    rtlsdr_set_sample_freq_correction(dev, dev->freq_correction);

    /* Demod soft-reset to apply new rate */
    rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1);
    rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);

    dev->sample_rate = real_rate;
    ESP_LOGI(TAG, "Sample rate: %lu Hz (requested %lu)", (unsigned long)real_rate, (unsigned long)rate);

    /* Retune center freq after rate change */
    if (dev->center_freq > 0) {
        rtlsdr_set_center_freq(dev, dev->center_freq);
    }

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
    /* Bias tee is on GPIO 0 */
    esp_err_t ret = rtlsdr_set_bias_tee_gpio(dev, 0, on ? 1 : 0);
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
    /* Reset EPA (USB endpoint) buffer — matches librtlsdr */
    uint8_t epa_ctl[2];

    epa_ctl[0] = 0x10; epa_ctl[1] = 0x02;
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, 0x2148, epa_ctl, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "EPA CTL reset set failed");

    epa_ctl[0] = 0x00; epa_ctl[1] = 0x00;
    ret = rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, 0x2148, epa_ctl, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "EPA CTL reset clear failed");

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
    ESP_LOGI(TAG, "Submitting %lu bulk IN transfers (%lu bytes each, EP=0x%02x)",
             (unsigned long)buf_num, (unsigned long)buf_len, RTLSDR_BULK_EP);
    for (uint32_t i = 0; i < buf_num; i++) {
        esp_err_t ret = usb_host_transfer_submit(dev->xfers[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Bulk xfer %lu submit failed: %s",
                     (unsigned long)i, esp_err_to_name(ret));
            dev->async_running = false;
            break;
        }
    }
    ESP_LOGI(TAG, "All %lu bulk transfers submitted, streaming active", (unsigned long)buf_num);

    /* Wait until stopped (USB events pumped by background client task) */
    while (dev->async_running) {
        vTaskDelay(pdMS_TO_TICKS(100));
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
