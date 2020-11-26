
/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "user_wifi.h"
#include "user_common.h"

EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
//#define WIFI_CONNECTED_BIT BIT0
//#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "user_wifi";
uint8_t staModleStartFlg = DISABLE;
static int s_retry_num = 0;
tApInfo aucApInfo = {0};

static wifi_scan_config_t scanConf  = {
	.ssid = NULL,
	.bssid = NULL,
	.channel = 0,
	.show_hidden = true
};

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (ENBLE == staModleStartFlg)
        {
            ESP_ERROR_CHECK( esp_wifi_connect() );
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            if (ENBLE == staModleStartFlg)
            {
                ESP_ERROR_CHECK( esp_wifi_connect() );
            }
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        staModleStartFlg = DISABLE;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_wifi_stop());
    esp_netif_create_default_wifi_ap();
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_AP_SSID,
            .ssid_len = strlen(ESP_AP_SSID),
            .channel = ESP_AP_CHANNEL,
            .password = ESP_AP_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(ESP_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_AP_SSID, ESP_AP_PASS, ESP_AP_CHANNEL);
}

void wifi_init_sta(void)
{
    staModleStartFlg = ENBLE;
    ESP_ERROR_CHECK(esp_wifi_stop());
    esp_netif_create_default_wifi_sta();
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_STA_SSID,
            .password = ESP_STA_PASS,
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    
    ESP_LOGI(TAG, "wifi_init_sta finished. SSID:%s password:%s",
             ESP_STA_SSID, ESP_STA_PASS);
}

void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
#if 1
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
#endif
}

void wifi_scan(void)
{
	uint16_t apCount;
	int i = 0;
    char fifo[70] = {0};
    int count = 0;
	
	//ESP_ERROR_CHECK(esp_wifi_disconnect());
	//ESP_ERROR_CHECK(esp_wifi_scan_stop());
	//The true parameter cause the function to block until the scan is done.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));
	apCount = 0;
	esp_wifi_scan_get_ap_num(&apCount);
	//printf("Number of access points found: %d\n", apCount);
	if (apCount == 0) {
		printf("00:00:00:00:00:00|0|0|\n");
		return;
	}
	wifi_ap_record_t *list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);
	ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, list));
#if OPEN_PRINTF
	printf("----------scandone\n");
#endif
	memset(&aucApInfo, 0, sizeof (aucApInfo));

	// A8:0C:CA:01:76:86|-56|1|WAYZ Guest

	aucApInfo.count = apCount;
#if OPEN_PRINTF
	printf("-----find ap number: %d ---------\r\n", aucApInfo.count);
#endif
	for (i = 0; i < apCount; i++) {
		//printf(""MACSTR"|%d|%d|%s\n", MAC2STR(list[i].bssid), list[i].rssi, list[i].primary, list[i].ssid);
		aucApInfo.tinfoAp[i].channel = list[i].primary;
		aucApInfo.tinfoAp[i].rssi = list[i].rssi;
		strncpy((char *)aucApInfo.tinfoAp[i].mac, (char *)list[i].bssid, 6 );
		strcpy( (char *)aucApInfo.tinfoAp[i].ssid, (char *)list[i].ssid );
#if OPEN_PRINTF
		printf("%02X:%02X:%02X:%02X:%02X:%02X|%02d|%02d|%s\n", MAC2STR(aucApInfo.tinfoAp[i].mac), 
				aucApInfo.tinfoAp[i].rssi, aucApInfo.tinfoAp[i].channel, aucApInfo.tinfoAp[i].ssid);
#endif
        sprintf(fifo, "%02X:%02X:%02X:%02X:%02X:%02X|%02d|%02d|%s\n", MAC2STR(aucApInfo.tinfoAp[i].mac), 
				aucApInfo.tinfoAp[i].rssi, aucApInfo.tinfoAp[i].channel, aucApInfo.tinfoAp[i].ssid);
        count += strlen(fifo);
	}
    aucApInfo.length = count;
	free(list);
    //NVS_Write(AP_INFO_KEY, &aucApInfo, sizeof(aucApInfo));
}


