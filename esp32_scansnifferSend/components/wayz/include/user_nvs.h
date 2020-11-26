
#ifndef __USER_NVS_
#define __USER_NVS_
#include "esp_system.h"
#include "stdio.h"

#define  TB_SELF            "Tb_Self" //数据库的表名

#define  WIFI_INFO_KEY      "wifi_info"
#define  AP_INFO_KEY        "ap_info"

#define  KEY_LEN             25  
//#define  SNIFFER_KEY(x)      #x             

#define  USER_FLASH_SIZE     100 // 单位k


void NVS_Read(const char* key, void* value, size_t size);
void NVS_Write(const char* key, void* value, size_t length);
void NVS_erase_all(void);
void flash_erase(void);
extern uint32_t secB_addr;

#endif

