/*
 * WiFi Manager — Ethernet Manager
 *
 * RMII Ethernet with IP101/LAN8720/RTL8201 PHY support.
 * DHCP or static IP. Link status callbacks.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_eth_phy.h"
#include "driver/gpio.h"

#include "wifimgr_config.h"
#include "wifimgr_notify.h"

static const char *TAG = "wifimgr_eth";

static esp_eth_handle_t s_eth_handle;
static esp_netif_t *s_eth_netif;
static volatile bool s_link_up;

/* ── Event handlers ─────────────────────────────────────────── */

static void eth_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    switch (id) {
    case ETHERNET_EVENT_CONNECTED:
        s_link_up = true;
        ESP_LOGI(TAG, "Ethernet link up");
        wifimgr_notify_text("system", "eth_link_change",
                             "Ethernet Connected", "Link up");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        s_link_up = false;
        ESP_LOGW(TAG, "Ethernet link down");
        wifimgr_notify_text("system", "eth_link_change",
                             "Ethernet Disconnected", "Link down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet stopped");
        break;
    default:
        break;
    }
}

static void got_ip_handler(void *arg, esp_event_base_t base,
                            int32_t id, void *data)
{
    if (id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = data;
        ESP_LOGI(TAG, "Ethernet IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/* ── Public API ─────────────────────────────────────────────── */

esp_err_t wifimgr_ethernet_init(void)
{
    ethernet_config_t cfg;
    wifimgr_config_load_ethernet(&cfg);

    if (!cfg.enable) {
        ESP_LOGI(TAG, "Ethernet disabled in config");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing Ethernet (PHY: %s, addr: %d, MDC: %d, MDIO: %d)",
             cfg.phy_type, cfg.phy_addr, cfg.mdc_gpio, cfg.mdio_gpio);

    /* MAC config */
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = cfg.mdc_gpio;
    emac_config.smi_gpio.mdio_num = cfg.mdio_gpio;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    if (!mac) {
        ESP_LOGE(TAG, "Failed to create MAC");
        return ESP_FAIL;
    }

    /* PHY config */
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = cfg.phy_addr;
    phy_config.reset_gpio_num = -1;

    esp_eth_phy_t *phy = NULL;
    if (strcmp(cfg.phy_type, "IP101") == 0) {
        phy = esp_eth_phy_new_ip101(&phy_config);
    } else if (strcmp(cfg.phy_type, "LAN8720") == 0) {
        phy = esp_eth_phy_new_lan87xx(&phy_config);
    } else if (strcmp(cfg.phy_type, "RTL8201") == 0) {
        phy = esp_eth_phy_new_rtl8201(&phy_config);
    } else {
        ESP_LOGE(TAG, "Unknown PHY type: %s", cfg.phy_type);
        mac->del(mac);
        return ESP_ERR_INVALID_ARG;
    }

    if (!phy) {
        ESP_LOGE(TAG, "Failed to create PHY");
        mac->del(mac);
        return ESP_FAIL;
    }

    /* Install Ethernet driver */
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_err_t err = esp_eth_driver_install(&eth_config, &s_eth_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet driver install failed: %s", esp_err_to_name(err));
        phy->del(phy);
        mac->del(mac);
        return err;
    }

    /* Create netif */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);
    esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle));

    /* Static IP if not DHCP */
    if (!cfg.dhcp) {
        esp_netif_dhcpc_stop(s_eth_netif);

        esp_netif_ip_info_t ip_info = {0};
        ip_info.ip.addr = esp_ip4addr_aton(cfg.static_ip);
        ip_info.netmask.addr = esp_ip4addr_aton(cfg.static_mask);
        ip_info.gw.addr = esp_ip4addr_aton(cfg.static_gw);
        esp_netif_set_ip_info(s_eth_netif, &ip_info);

        /* Set DNS */
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = esp_ip4addr_aton(cfg.static_dns);
        dns.ip.type = ESP_IPADDR_TYPE_V4;
        esp_netif_set_dns_info(s_eth_netif, ESP_NETIF_DNS_MAIN, &dns);

        ESP_LOGI(TAG, "Static IP: %s/%s gw %s dns %s",
                 cfg.static_ip, cfg.static_mask, cfg.static_gw, cfg.static_dns);
    }

    /* Register event handlers */
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, got_ip_handler, NULL);

    /* Start */
    err = esp_eth_start(s_eth_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet start failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Ethernet initialized and started");
    return ESP_OK;
}

bool wifimgr_ethernet_link_up(void)
{
    return s_link_up;
}

esp_netif_t *wifimgr_ethernet_get_netif(void)
{
    return s_eth_netif;
}
