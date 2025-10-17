/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include "protocol_examples_common.h"
#include "example_common_private.h"
#include "esp_event.h"
#include "esp_eth.h"
#if CONFIG_ETH_USE_SPI_ETHERNET
#include "driver/spi_master.h"
#endif // CONFIG_ETH_USE_SPI_ETHERNET
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/ip.h"
#if (CONFIG_EDGE_PLUS == 1) || (CONFIG_EDGE_LPR == 1) || (CONFIG_EDGE_V2 == 1) || (CONFIG_EDGE_HCB2 == 1) || (CONFIG_EDGE_FB == 1) || (CONFIG_EDGE_QR == 1) || (CONFIG_EDGE_O4 == 1) || (CONFIG_EDGE_O12 == 1)
#warning "Please set the correct path to the Edge Workspace"
#include "../../../../../components/Database/include/nvs_handler.h"
// #include "D:/Projects/EntryPass/Firmware/EDGE-V2/hcb-edge-plus/components/Database/include/nvs_handler.h"
#endif

#define EXAMPLE_MAXIMUM_RETRY 3
#define EXAMPLE_STATIC_IP_ADDR "192.168.4.100"
#define EXAMPLE_STATIC_NETMASK_ADDR "255.255.255.0"
#define EXAMPLE_STATIC_GW_ADDR "192.168.4.1"

#define EXAMPLE_MAIN_DNS_SERVER "8.8.8.8"
#define EXAMPLE_BACKUP_DNS_SERVER "8.8.4.4"

static const char *TAG = "ethernet_connect";
static SemaphoreHandle_t s_semph_get_ip_addrs = NULL;
#if CONFIG_EXAMPLE_CONNECT_IPV6
static SemaphoreHandle_t s_semph_get_ip6_addrs = NULL;
#endif

static esp_netif_t *eth_start(void);
static void eth_stop(void);

/** Event handler for Ethernet events */

static void eth_on_got_ip(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if (!example_is_our_netif(EXAMPLE_NETIF_DESC_ETH, event->esp_netif))
    {
        return;
    }
    ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
    xSemaphoreGive(s_semph_get_ip_addrs);
}

static esp_err_t set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != IPADDR_NONE))
    {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}

static void set_static_ip(esp_netif_t *netif)
{
#if (CONFIG_EDGE_PLUS == 1) || (CONFIG_EDGE_LPR == 1) || (CONFIG_EDGE_V2 == 1) || (CONFIG_EDGE_HCB2 == 1) || (CONFIG_EDGE_FB == 1) || (CONFIG_EDGE_QR == 1) || (CONFIG_EDGE_O4 == 1) || (CONFIG_EDGE_O12 == 1)
    S_DEV_NETWORK_SETTINGS dev_net_settings;
#endif

    if (esp_netif_dhcpc_stop(netif) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to stop dhcp client");
        return;
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));

#if (CONFIG_EDGE_PLUS == 1) || (CONFIG_EDGE_LPR == 1) || (CONFIG_EDGE_V2 == 1) || (CONFIG_EDGE_HCB2 == 1) || (CONFIG_EDGE_FB == 1) || (CONFIG_EDGE_QR == 1) || (CONFIG_EDGE_O4 == 1) || (CONFIG_EDGE_O12 == 1)
    ESP_ERROR_CHECK(get_dev_network_settings(&dev_net_settings, E_GET_DEV_NETWORK_MODE_NORMAL));
    ip.ip.addr = ipaddr_addr(dev_net_settings.static_ip_v4_addr);
    ip.netmask.addr = ipaddr_addr(dev_net_settings.static_ip_netmask);
    ip.gw.addr = ipaddr_addr(dev_net_settings.static_gw_ip);
#else
    ip.ip.addr = ipaddr_addr(EXAMPLE_STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(EXAMPLE_STATIC_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(EXAMPLE_STATIC_GW_ADDR);
#endif
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set ip info");
        return;
    }
#if (CONFIG_EDGE_PLUS == 1) || (CONFIG_EDGE_LPR == 1) || (CONFIG_EDGE_V2 == 1) || (CONFIG_EDGE_HCB2 == 1) || (CONFIG_EDGE_FB == 1) || (CONFIG_EDGE_QR == 1) || (CONFIG_EDGE_O4 == 1) || (CONFIG_EDGE_O12 == 1)
    ESP_ERROR_CHECK(set_dns_server(netif, ipaddr_addr(dev_net_settings.dns_ip[0]), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(set_dns_server(netif, ipaddr_addr(dev_net_settings.dns_ip[1]), ESP_NETIF_DNS_BACKUP));
    ESP_LOGI(TAG, "Success to set static ip from nvs: %s, netmask: %s, gw: %s dns1: %s dns2: %s", 
        dev_net_settings.static_ip_v4_addr, dev_net_settings.static_ip_netmask, dev_net_settings.static_gw_ip,
        dev_net_settings.dns_ip[0], dev_net_settings.dns_ip[1]);
#else
    ESP_LOGI(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", EXAMPLE_STATIC_IP_ADDR, EXAMPLE_STATIC_NETMASK_ADDR, EXAMPLE_STATIC_GW_ADDR);
    ESP_ERROR_CHECK(set_dns_server(netif, ipaddr_addr(EXAMPLE_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(set_dns_server(netif, ipaddr_addr(EXAMPLE_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
#endif
}

static void on_eth_event(void *esp_netif, esp_event_base_t event_base,
                         int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id)
    {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Up");
#if CONFIG_EXAMPLE_CONNECT_IPV6
        ESP_ERROR_CHECK(esp_netif_create_ip6_linklocal(esp_netif));
#endif
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

#if (CONFIG_USE_DHCP_CLIENT_ETH == 0)
        set_static_ip(esp_netif);
#else
        S_DEV_NETWORK_SETTINGS dev_net_settings;
        esp_err_t ret = get_dev_network_settings(&dev_net_settings, E_GET_DEV_NETWORK_MODE_NORMAL);

        ESP_LOGI(TAG, "get_dev_network_settings ret: %s", esp_err_to_name(ret));

        if ((dev_net_settings.dev_flag & DEV_MODE_BM_DHCP_EN) == 0)
        {
            ESP_LOGI(TAG, "Static IP mode");
            set_static_ip(esp_netif);
        }
        else
        {
            ESP_LOGI(TAG, "DHCP mode");
            esp_netif_dhcpc_start(esp_netif);
        }
#endif
        break;
    default:
        break;
    }
}

#if CONFIG_EXAMPLE_CONNECT_IPV6

static void eth_on_got_ipv6(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
    if (!example_is_our_netif(EXAMPLE_NETIF_DESC_ETH, event->esp_netif))
    {
        return;
    }
    esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&event->ip6_info.ip);
    ESP_LOGI(TAG, "Got IPv6 event: Interface \"%s\" address: " IPV6STR ", type: %s", esp_netif_get_desc(event->esp_netif),
             IPV62STR(event->ip6_info.ip), example_ipv6_addr_types_to_str[ipv6_type]);
    if (ipv6_type == EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE)
    {
        xSemaphoreGive(s_semph_get_ip6_addrs);
    }
}

#endif // CONFIG_EXAMPLE_CONNECT_IPV6

static esp_eth_handle_t s_eth_handle = NULL;
static esp_eth_mac_t *s_mac = NULL;
static esp_eth_phy_t *s_phy = NULL;
static esp_eth_netif_glue_handle_t s_eth_glue = NULL;

static esp_netif_t *eth_start(void)
{
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
    esp_netif_config.if_desc = EXAMPLE_NETIF_DESC_ETH;
    esp_netif_config.route_prio = 64;
    esp_netif_config_t netif_config = {
        .base = &esp_netif_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH};
    esp_netif_t *netif = esp_netif_new(&netif_config);
    assert(netif);

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.rx_task_stack_size = CONFIG_EXAMPLE_ETHERNET_EMAC_TASK_STACK_SIZE;
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_EXAMPLE_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_EXAMPLE_ETH_PHY_RST_GPIO;
#if CONFIG_EXAMPLE_USE_INTERNAL_ETHERNET
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_mdc_gpio_num = CONFIG_EXAMPLE_ETH_MDC_GPIO;
    esp32_emac_config.smi_mdio_gpio_num = CONFIG_EXAMPLE_ETH_MDIO_GPIO;
    s_mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
#if CONFIG_EXAMPLE_ETH_PHY_IP101
    s_phy = esp_eth_phy_new_ip101(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_RTL8201
    s_phy = esp_eth_phy_new_rtl8201(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_LAN87XX
    s_phy = esp_eth_phy_new_lan87xx(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_DP83848
    s_phy = esp_eth_phy_new_dp83848(&phy_config);
#elif CONFIG_EXAMPLE_ETH_PHY_KSZ80XX
    s_phy = esp_eth_phy_new_ksz80xx(&phy_config);
#endif
#elif CONFIG_EXAMPLE_USE_SPI_ETHERNET
    gpio_install_isr_service(0);
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_EXAMPLE_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_EXAMPLE_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_EXAMPLE_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_EXAMPLE_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = CONFIG_EXAMPLE_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = CONFIG_EXAMPLE_ETH_SPI_CS_GPIO,
        .queue_size = 20};
#if CONFIG_EXAMPLE_USE_DM9051
    /* dm9051 ethernet driver is based on spi driver */
    eth_dm9051_config_t dm9051_config = ETH_DM9051_DEFAULT_CONFIG(CONFIG_EXAMPLE_ETH_SPI_HOST, &spi_devcfg);
    dm9051_config.int_gpio_num = CONFIG_EXAMPLE_ETH_SPI_INT_GPIO;
    s_mac = esp_eth_mac_new_dm9051(&dm9051_config, &mac_config);
    s_phy = esp_eth_phy_new_dm9051(&phy_config);
#elif CONFIG_EXAMPLE_USE_W5500
    /* w5500 ethernet driver is based on spi driver */
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(CONFIG_EXAMPLE_ETH_SPI_HOST, &spi_devcfg);
    w5500_config.int_gpio_num = CONFIG_EXAMPLE_ETH_SPI_INT_GPIO;
    s_mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    s_phy = esp_eth_phy_new_w5500(&phy_config);
#endif
#elif CONFIG_EXAMPLE_USE_OPENETH
    phy_config.autonego_timeout_ms = 100;
    s_mac = esp_eth_mac_new_openeth(&mac_config);
    s_phy = esp_eth_phy_new_dp83848(&phy_config);
#endif

    // Install Ethernet driver
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(s_mac, s_phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &s_eth_handle));
#if !CONFIG_EXAMPLE_USE_INTERNAL_ETHERNET
    /* The SPI Ethernet module might doesn't have a burned factory MAC address, we cat to set it manually.
       We set the ESP_MAC_ETH mac address as the default, if you want to use ESP_MAC_EFUSE_CUSTOM mac address, please enable the
       configuration: `ESP_MAC_USE_CUSTOM_MAC_AS_BASE_MAC`
    */
    uint8_t eth_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(eth_mac, ESP_MAC_ETH));
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, eth_mac));
#endif
    // combine driver with netif
    s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    esp_netif_attach(netif, s_eth_glue);

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_on_got_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_CONNECTED, &on_eth_event, netif));
#ifdef CONFIG_EXAMPLE_CONNECT_IPV6
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, &eth_on_got_ipv6, NULL));
#endif

    esp_eth_start(s_eth_handle);
    return netif;
}

static void eth_stop(void)
{
    esp_netif_t *eth_netif = get_example_netif_from_desc(EXAMPLE_NETIF_DESC_ETH);
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_on_got_ip));
#if CONFIG_EXAMPLE_CONNECT_IPV6
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_GOT_IP6, &eth_on_got_ipv6));
    ESP_ERROR_CHECK(esp_event_handler_unregister(ETH_EVENT, ETHERNET_EVENT_CONNECTED, &on_eth_event));
#endif
    ESP_ERROR_CHECK(esp_eth_stop(s_eth_handle));
    ESP_ERROR_CHECK(esp_eth_del_netif_glue(s_eth_glue));
    ESP_ERROR_CHECK(esp_eth_driver_uninstall(s_eth_handle));
    s_eth_handle = NULL;
    ESP_ERROR_CHECK(s_phy->del(s_phy));
    ESP_ERROR_CHECK(s_mac->del(s_mac));

    esp_netif_destroy(eth_netif);
}

esp_eth_handle_t get_example_eth_handle(void)
{
    return s_eth_handle;
}

/* tear down connection, release resources */
void example_ethernet_shutdown(void)
{
    if (s_semph_get_ip_addrs == NULL)
    {
        return;
    }
    vSemaphoreDelete(s_semph_get_ip_addrs);
    s_semph_get_ip_addrs = NULL;
#if CONFIG_EXAMPLE_CONNECT_IPV6
    vSemaphoreDelete(s_semph_get_ip6_addrs);
    s_semph_get_ip6_addrs = NULL;
#endif
    eth_stop();
}

esp_err_t example_ethernet_connect(void)
{
#if CONFIG_EXAMPLE_CONNECT_IPV4
    s_semph_get_ip_addrs = xSemaphoreCreateBinary();
    if (s_semph_get_ip_addrs == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
#endif
#if CONFIG_EXAMPLE_CONNECT_IPV6
    s_semph_get_ip6_addrs = xSemaphoreCreateBinary();
    if (s_semph_get_ip6_addrs == NULL)
    {
        vSemaphoreDelete(s_semph_get_ip_addrs);
        return ESP_ERR_NO_MEM;
    }
#endif
    eth_start();
    ESP_LOGI(TAG, "Waiting for IP(s).");
#if CONFIG_EXAMPLE_CONNECT_IPV4
    xSemaphoreTake(s_semph_get_ip_addrs, portMAX_DELAY);
#endif
#if CONFIG_EXAMPLE_CONNECT_IPV6
    xSemaphoreTake(s_semph_get_ip6_addrs, portMAX_DELAY);
#endif
    return ESP_OK;
}
