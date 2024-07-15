// Copyright 2021-2022 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_pm.h"
#include "esp_event.h"
#include "esp_smartconfig.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "lwip/err.h"
#include "lwip/sys.h"

static const char *TAG = "app_wifi";

#include "app_ui.h"

#if (ESP_IDF_VERSION_MAJOR >= 5)
#include "esp_mac.h"
#include "lwip/ip_addr.h"
#endif

#define ESP_MAXIMUM_RETRY 2

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define ESPTOUCH_DONE_BIT BIT1

enum
{
    WIFI_UNCONNECT,
    WIFI_CONNECTED
};

esp_netif_t *ap_netif = NULL;
nvs_handle_t wifi_nvs_handle;

static int s_retry_num = 0;
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group = NULL;
char wifi_ip_address[16] = {0};

static bool wifi_connected_flag = false;

static void wifi_smartconfig_task(void *parm);

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    /* Sta mode */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            ESP_LOGE(TAG, "connect ap fail for 4 times, now start smartconfig...");
            xTaskCreate(wifi_smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            // show to lcd
            ui_set_info_to_statusbar("start smartconfig");

            wifi_connected_flag = WIFI_UNCONNECT;
            lv_event_send(ui_Screen1, LV_EVENT_VALUE_CHANGED, (bool *)wifi_connected_flag);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        memset(wifi_ip_address, 0x0, sizeof(wifi_ip_address));
        strcpy(wifi_ip_address, ip4addr_ntoa((const ip4_addr_t *)&event->ip_info.ip));
        // show ip address to lcd
        ui_set_ip_address(wifi_ip_address);

        ui_set_info_to_statusbar((char *)wifi_ip_address);

        wifi_connected_flag = WIFI_CONNECTED;
        lv_event_send(ui_Screen1, LV_EVENT_VALUE_CHANGED, (bool *)wifi_connected_flag);

        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED");
        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
        uint8_t mac[6];
        esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
        // show mac address to lcd
        ui_set_mac_address(mac);

        // show wifi info to lcd
        char connect_text[48] = {0};
        sprintf(connect_text, "connected:%s", (const char *)event->ssid);
        ui_set_wifi_ssid((const char *)event->ssid);
    }
    /* Smartconfig mode */
    else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD)
    {
        ESP_LOGI(TAG, "SC_EVENT_GOT_SSID_PSWD");
        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = {0};
        uint8_t password[65] = {0};
        uint8_t rvd_data[33] = {0};

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));

        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true)
        {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2)
        {
            ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
            ESP_LOGI(TAG, "RVD_DATA:");
            for (int i = 0; i < 33; i++)
            {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }

        /* save wifi info to flash */
        ESP_ERROR_CHECK(nvs_open("wifi_info", NVS_READWRITE, &wifi_nvs_handle));
        ESP_ERROR_CHECK(nvs_set_str(wifi_nvs_handle, "wifi_ssid", (const char *)ssid));
        ESP_ERROR_CHECK(nvs_set_str(wifi_nvs_handle, "wifi_pswd", (const char *)password));
        ESP_ERROR_CHECK(nvs_set_u8(wifi_nvs_handle, "wifi_info_flag", 1));
        ESP_ERROR_CHECK(nvs_commit(wifi_nvs_handle));
        nvs_close(wifi_nvs_handle);

        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        esp_wifi_connect();

        // show to lcd
        char connect_text[48] = {0};
        sprintf(connect_text, "connecting:%s", (const char *)ssid);
        ui_set_info_to_statusbar((char *)connect_text);
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE)
    {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

static void wifi_smartconfig_task(void *parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_AIRKISS));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    while (1)
    {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if (uxBits & WIFI_CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if (uxBits & ESPTOUCH_DONE_BIT)
        {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

static void check_wifi_info(void)
{
    uint8_t has_conf_flag = 0;
    wifi_config_t wifi_config;

    memset(&wifi_config, 0, sizeof(wifi_config_t));

    nvs_open("wifi_info", NVS_READONLY, &wifi_nvs_handle);
    nvs_get_u8(wifi_nvs_handle, "wifi_info_flag", &has_conf_flag);

    if (has_conf_flag == 1)
    {
        ESP_LOGI(TAG, "flash have wifi Info, now try to connect wifi...");

        char flash_ssid[33] = {0};
        size_t ssid_length = sizeof(flash_ssid);
        esp_err_t ssid_err = nvs_get_str(wifi_nvs_handle, "wifi_ssid", flash_ssid, &ssid_length);
        if (ssid_err == ESP_OK)
        {
            ESP_LOGI(TAG, "Flash Wi-Fi SSID = %s", flash_ssid);
            memcpy((char *)wifi_config.sta.ssid, flash_ssid, 32);
        }

        char flash_passwd[65] = {0};
        size_t passwd_length = sizeof(flash_passwd);
        esp_err_t passwd_err = nvs_get_str(wifi_nvs_handle, "wifi_pswd", flash_passwd, &passwd_length);
        if (passwd_err == ESP_OK)
        {
            ESP_LOGI(TAG, "Flash Wi-Fi pswd = %s", flash_passwd);
            memcpy((char *)wifi_config.sta.password, flash_passwd, 64);
        }

        ESP_LOGI(TAG, "connect wifi ssid = %s, pswd = %s", wifi_config.sta.ssid, wifi_config.sta.password);
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
    else
    {
        ESP_LOGI(TAG, "No wifi info in the nvs, now start smartconfig...");
        xTaskCreate(wifi_smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
        // set ui to lcd
        ui_set_info_to_statusbar("start smartconfig");
    }

    // close nvs
    nvs_close(wifi_nvs_handle);
}

esp_err_t App_Wifi_run(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi init finished.");

    ESP_LOGI(TAG, "esp_wifi_set_ps().");
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

    check_wifi_info();

    xEventGroupWaitBits(s_wifi_event_group,
                        WIFI_CONNECTED_BIT,
                        pdFALSE,
                        pdTRUE,
                        portMAX_DELAY);

    // vEventGroupDelete(s_wifi_event_group);
    // s_wifi_event_group = NULL;

    return ESP_OK;
}
