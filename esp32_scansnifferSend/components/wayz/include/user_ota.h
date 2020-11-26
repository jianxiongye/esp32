
#ifndef  __USER_OTA_H
#define  __USER_OTA_H

#include "esp_system.h"
#include "user_common.h"

#define EXAMPLE_SERVER_IP       CONFIG_SERVER_IP
#define EXAMPLE_SERVER_PORT     CONFIG_SERVER_PORT
#define EXAMPLE_FILENAME        CONFIG_EXAMPLE_FILENAME
#define BUFFSIZE                1500
#define TEXT_BUFFSIZE           1024

#pragma  pack(push)
#pragma  pack(1)

typedef enum esp_ota_firm_state {
    ESP_OTA_INIT = 0,
    ESP_OTA_PREPARE,
    ESP_OTA_START,
    ESP_OTA_RECVED,
    ESP_OTA_FINISH,
} esp_ota_firm_state_t;

typedef struct esp_ota_firm {
    uint8_t             ota_num;
    uint8_t             update_ota_num;

    esp_ota_firm_state_t    state;

    size_t              content_len;

    size_t              read_bytes;
    size_t              write_bytes;

    size_t              ota_size;
    size_t              ota_offset;

    const char          *buf;
    size_t              bytes;
} esp_ota_firm_t;


#pragma  pack(pop)

void create_ota_updata();
void ota_suspend();
void ota_resume();


#endif
