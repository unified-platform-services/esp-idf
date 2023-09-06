/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Common functions for protocol examples, to establish Wi-Fi or Ethernet connection.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#include <string.h>
#include "protocol_examples_common.h"
#include "example_common_private.h"
#include "esp_log.h"

#if CONFIG_HCB_MODEL_N5200 || CONFIG_HCB_MODEL_N5400 || CONFIG_HCB_MODEL_N5150 || CONFIG_ESP32
#include "../../../../ESP-WS/HCB-EdgePlus/components/Database/include/nvs_handler.h"
#endif

#include <netdb.h>

#define EXAMPLE_STATIC_IP_ADDR CONFIG_EXAMPLE_STATIC_IP_ADDR
#define EXAMPLE_STATIC_NETMASK_ADDR CONFIG_EXAMPLE_STATIC_NETMASK_ADDR
#define EXAMPLE_STATIC_GW_ADDR CONFIG_EXAMPLE_STATIC_GW_ADDR
#ifdef CONFIG_EXAMPLE_STATIC_DNS_AUTO
#define EXAMPLE_MAIN_DNS_SERVER EXAMPLE_STATIC_GW_ADDR
#define EXAMPLE_BACKUP_DNS_SERVER "0.0.0.0"
#else
#define EXAMPLE_MAIN_DNS_SERVER CONFIG_EXAMPLE_STATIC_DNS_SERVER_MAIN
#define EXAMPLE_BACKUP_DNS_SERVER CONFIG_EXAMPLE_STATIC_DNS_SERVER_BACKUP
#endif

#if CONFIG_EXAMPLE_CONNECT_WIFI

static const char *TAG = "example_connect";
static esp_netif_t *s_example_sta_netif = NULL;
static SemaphoreHandle_t s_semph_get_ip_addrs = NULL;
#if CONFIG_EXAMPLE_CONNECT_IPV6
static SemaphoreHandle_t s_semph_get_ip6_addrs = NULL;
#endif

#if CONFIG_EXAMPLE_WIFI_SCAN_METHOD_FAST
#define EXAMPLE_WIFI_SCAN_METHOD WIFI_FAST_SCAN
#elif CONFIG_EXAMPLE_WIFI_SCAN_METHOD_ALL_CHANNEL
#define EXAMPLE_WIFI_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#endif

#if CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SIGNAL
#define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SECURITY
#define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#endif

#if CONFIG_EXAMPLE_WIFI_AUTH_OPEN
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_EXAMPLE_WIFI_AUTH_WEP
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA2_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA_WPA2_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA2_ENTERPRISE
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_ENTERPRISE
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA3_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA2_WPA3_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WAPI_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

static int s_retry_num = 0;

static void example_handler_on_wifi_disconnect(void *arg, esp_event_base_t event_base,
                                               int32_t event_id, void *event_data)
{
  s_retry_num++;
  if (s_retry_num > CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY)
  {
    ESP_LOGI(TAG, "WiFi Connect failed %d times, stop reconnect.", s_retry_num);
    /* let example_wifi_sta_do_connect() return */
    if (s_semph_get_ip_addrs)
    {
      xSemaphoreGive(s_semph_get_ip_addrs);
    }
#if CONFIG_EXAMPLE_CONNECT_IPV6
    if (s_semph_get_ip6_addrs)
    {
      xSemaphoreGive(s_semph_get_ip6_addrs);
    }
#endif
    return;
  }
  ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
  esp_err_t err = esp_wifi_connect();
  if (err == ESP_ERR_WIFI_NOT_STARTED)
  {
    return;
  }
  ESP_ERROR_CHECK(err);
}

static void get_mac_address(void)
{
  uint8_t mac[6];
  esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
  ESP_LOGI(TAG, "MAC address: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

#if CONFIG_EXAMPLE_SET_STATIC_IP
static esp_err_t example_set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
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

static void example_set_static_ip(esp_netif_t *netif)
{
#if CONFIG_HCB_MODEL_N5200 || CONFIG_HCB_MODEL_N5400 || CONFIG_HCB_MODEL_N5150 || CONFIG_ESP32
  TDevNetworkSettings dev_net_settings;
#endif

  if (esp_netif_dhcpc_stop(netif) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to stop dhcp client");
    return;
  }
  esp_netif_ip_info_t ip;
  memset(&ip, 0, sizeof(esp_netif_ip_info_t));

#if CONFIG_HCB_MODEL_N5200 || CONFIG_HCB_MODEL_N5400 || CONFIG_HCB_MODEL_N5150 || CONFIG_ESP32
  ESP_ERROR_CHECK(get_dev_network_settings(&dev_net_settings));
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
#if CONFIG_HCB_MODEL_N5200 || CONFIG_HCB_MODEL_N5400 || CONFIG_HCB_MODEL_N5150 || CONFIG_ESP32
  ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", dev_net_settings.static_ip_v4_addr, dev_net_settings.static_ip_netmask, dev_net_settings.static_gw_ip);
  ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(dev_net_settings.dns_ip[0]), ESP_NETIF_DNS_MAIN));
  ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(dev_net_settings.dns_ip[1]), ESP_NETIF_DNS_BACKUP));
#else  
  ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", EXAMPLE_STATIC_IP_ADDR, EXAMPLE_STATIC_NETMASK_ADDR, EXAMPLE_STATIC_GW_ADDR);
  ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
  ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
#endif
}
#endif

static void example_handler_on_wifi_connect(void *esp_netif, esp_event_base_t event_base,
                                            int32_t event_id, void *event_data)
{
#if CONFIG_EXAMPLE_SET_STATIC_IP
  example_set_static_ip(esp_netif);
#endif

#if CONFIG_EXAMPLE_CONNECT_IPV6
  esp_netif_create_ip6_linklocal(esp_netif);
#endif // CONFIG_EXAMPLE_CONNECT_IPV6
}

static void example_handler_on_sta_got_ip(void *arg, esp_event_base_t event_base,
                                          int32_t event_id, void *event_data)
{
  s_retry_num = 0;
  ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
  if (!example_is_our_netif(EXAMPLE_NETIF_DESC_STA, event->esp_netif))
  {
    return;
  }
  ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
  if (s_semph_get_ip_addrs)
  {
    xSemaphoreGive(s_semph_get_ip_addrs);
  }
  else
  {
    ESP_LOGI(TAG, "- IPv4 address: " IPSTR ",", IP2STR(&event->ip_info.ip));
  }
  get_mac_address();
}

#if CONFIG_EXAMPLE_CONNECT_IPV6
static void example_handler_on_sta_got_ipv6(void *arg, esp_event_base_t event_base,
                                            int32_t event_id, void *event_data)
{
  ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
  if (!example_is_our_netif(EXAMPLE_NETIF_DESC_STA, event->esp_netif))
  {
    return;
  }
  esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&event->ip6_info.ip);
  ESP_LOGI(TAG, "Got IPv6 event: Interface \"%s\" address: " IPV6STR ", type: %s", esp_netif_get_desc(event->esp_netif),
           IPV62STR(event->ip6_info.ip), example_ipv6_addr_types_to_str[ipv6_type]);

  if (ipv6_type == EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE)
  {
    if (s_semph_get_ip6_addrs)
    {
      xSemaphoreGive(s_semph_get_ip6_addrs);
    }
    else
    {
      ESP_LOGI(TAG, "- IPv6 address: " IPV6STR ", type: %s", IPV62STR(event->ip6_info.ip), example_ipv6_addr_types_to_str[ipv6_type]);
    }
  }
}
#endif // CONFIG_EXAMPLE_CONNECT_IPV6

void example_wifi_start(void)
{
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
  // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
  esp_netif_config.if_desc = EXAMPLE_NETIF_DESC_STA;
  esp_netif_config.route_prio = 128;
  s_example_sta_netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);

  esp_netif_set_hostname(s_example_sta_netif, CONFIG_ESP32_HOSTNAME);

  esp_wifi_set_default_wifi_sta_handlers();

  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
}

void example_wifi_stop(void)
{
  esp_err_t err = esp_wifi_stop();
  if (err == ESP_ERR_WIFI_NOT_INIT)
  {
    return;
  }
  ESP_ERROR_CHECK(err);
  ESP_ERROR_CHECK(esp_wifi_deinit());
  ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(s_example_sta_netif));
  esp_netif_destroy(s_example_sta_netif);
  s_example_sta_netif = NULL;
}

esp_err_t example_wifi_sta_do_connect(wifi_config_t wifi_config, bool wait)
{
  if (wait)
  {
    s_semph_get_ip_addrs = xSemaphoreCreateBinary();
    if (s_semph_get_ip_addrs == NULL)
    {
      return ESP_ERR_NO_MEM;
    }
#if CONFIG_EXAMPLE_CONNECT_IPV6
    s_semph_get_ip6_addrs = xSemaphoreCreateBinary();
    if (s_semph_get_ip6_addrs == NULL)
    {
      vSemaphoreDelete(s_semph_get_ip_addrs);
      return ESP_ERR_NO_MEM;
    }
#endif
  }
  s_retry_num = 0;
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &example_handler_on_wifi_disconnect, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &example_handler_on_sta_got_ip, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &example_handler_on_wifi_connect, s_example_sta_netif));
#if CONFIG_EXAMPLE_CONNECT_IPV6
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, &example_handler_on_sta_got_ipv6, NULL));
#endif

  ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  esp_err_t ret = esp_wifi_connect();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "WiFi connect failed! ret:%x", ret);
    return ret;
  }
  if (wait)
  {
    ESP_LOGI(TAG, "Waiting for IP(s)");
    xSemaphoreTake(s_semph_get_ip_addrs, portMAX_DELAY);
#if CONFIG_EXAMPLE_CONNECT_IPV6
    xSemaphoreTake(s_semph_get_ip6_addrs, portMAX_DELAY);
#endif
    if (s_retry_num > CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY)
    {
      return ESP_FAIL;
    }
  }
  return ESP_OK;
}

esp_err_t example_wifi_sta_do_disconnect(void)
{
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &example_handler_on_wifi_disconnect));
  ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &example_handler_on_sta_got_ip));
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &example_handler_on_wifi_connect));
#if CONFIG_EXAMPLE_CONNECT_IPV6
  ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_GOT_IP6, &example_handler_on_sta_got_ipv6));
#endif
  if (s_semph_get_ip_addrs)
  {
    vSemaphoreDelete(s_semph_get_ip_addrs);
  }
#if CONFIG_EXAMPLE_CONNECT_IPV6
  if (s_semph_get_ip6_addrs)
  {
    vSemaphoreDelete(s_semph_get_ip6_addrs);
  }
#endif
  return esp_wifi_disconnect();
}

void example_wifi_shutdown(void)
{
  example_wifi_sta_do_disconnect();
  example_wifi_stop();
}

esp_err_t example_wifi_connect(void)
{
  ESP_LOGI(TAG, "Start example_connect.");
  example_wifi_start();
  wifi_config_t wifi_config = {
      .sta = {
#if !CONFIG_EXAMPLE_WIFI_SSID_PWD_FROM_STDIN
          .ssid = CONFIG_EXAMPLE_WIFI_SSID,
          .password = CONFIG_EXAMPLE_WIFI_PASSWORD,
#endif
          .scan_method = EXAMPLE_WIFI_SCAN_METHOD,
          .sort_method = EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD,
          .threshold.rssi = CONFIG_EXAMPLE_WIFI_SCAN_RSSI_THRESHOLD,
          .threshold.authmode = EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD,
      },
  };
#if CONFIG_EXAMPLE_WIFI_SSID_PWD_FROM_STDIN
  example_configure_stdin_stdout();
  char buf[sizeof(wifi_config.sta.ssid) + sizeof(wifi_config.sta.password) + 2] = {0};
  ESP_LOGI(TAG, "Please input ssid password:");
  fgets(buf, sizeof(buf), stdin);
  int len = strlen(buf);
  buf[len - 1] = '\0'; /* removes '\n' */
  memset(wifi_config.sta.ssid, 0, sizeof(wifi_config.sta.ssid));

  char *rest = NULL;
  char *temp = strtok_r(buf, " ", &rest);
  strncpy((char *)wifi_config.sta.ssid, temp, sizeof(wifi_config.sta.ssid));
  memset(wifi_config.sta.password, 0, sizeof(wifi_config.sta.password));
  temp = strtok_r(NULL, " ", &rest);
  if (temp)
  {
    strncpy((char *)wifi_config.sta.password, temp, sizeof(wifi_config.sta.password));
  }
  else
  {
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
  }
#endif
  return example_wifi_sta_do_connect(wifi_config, true);
}

#endif /* CONFIG_EXAMPLE_CONNECT_WIFI */
