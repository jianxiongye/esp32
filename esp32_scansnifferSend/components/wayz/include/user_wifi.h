
#ifndef  __USER_WIFI_H
#define  __USER_WIFI_H

#include "esp_system.h"


#define  ESP_STA_SSID        CONFIG_ESP_STA_SSID
#define  ESP_STA_PASS        CONFIG_ESP_STA_PASSWORD
#define  ESP_MAXIMUM_RETRY   CONFIG_ESP_MAXIMUM_RETRY

#define  ESP_AP_SSID         CONFIG_ESP_AP_SSID
#define  ESP_AP_PASS         CONFIG_ESP_AP_PASSWORD
#define  ESP_AP_CHANNEL      CONFIG_ESP_AP_CHANNEL
#define  MAX_STA_CONN        CONFIG_ESP_MAX_STA_CONN

#define  AP_MAX_NUMBER       100

#pragma  pack(push)
#pragma  pack(1)

typedef struct _ap_struct_Info_
{
    uint8_t mac[6];
    int8_t rssi;
    uint8_t channel;
    uint8_t ssid[33];
}tApStructInfo;

typedef struct _apInfo_
{
    tApStructInfo tinfoAp[AP_MAX_NUMBER];
    int8_t count;
    int32_t length;
}tApInfo;

#pragma  pack(pop)

void wifi_init(void);
void wifi_init_sta(void);
void wifi_init_softap(void);
void wifi_scan(void);



#endif

