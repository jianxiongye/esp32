

#include <stdio.h>
#include "esp_system.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_spi_flash.h"

#include "user_nvs.h"

static const char *TAG = "user_nvs";
uint32_t secB_addr = 0x330000;
/**
 * @description:  读取数据进去nvs里面的任务
 * @param {type} 
 * @return: 
 */
void NVS_Read(const char* key, void* value, size_t size)
{
    //ESP_LOGI(TAG, "--------------- Start NVS_Read ---------------");

    //NVS操作的句柄，类似于 rtos系统的任务创建返回的句柄！
    nvs_handle mHandleNvsRead;

    esp_err_t err = nvs_open(TB_SELF, NVS_READWRITE, &mHandleNvsRead);
    //打开数据库，打开一个数据库就相当于会返回一个句柄
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Open NVS Table fail");
        return ;
    }

    //读取数组
    err = nvs_get_blob(mHandleNvsRead, key, value, &size);

    if (err == ESP_OK)
    {
#if 0
        ESP_LOGI(TAG, "get group_myself_read data OK !");
        for (uint32_t i = 0; i < size; i++)
        {
            ESP_LOGI(TAG, "get group_myself_read data : [%d] =%02x", i, ((char *)value)[i]);
        }
#endif
    }

    //关闭数据库，关闭面板！
    nvs_close(mHandleNvsRead);

    //ESP_LOGI(TAG, "--------------- End NVS_Read ---------------");
}

/**
 * @description:  创建一个写数据进去nvs里面的任务
 * @param {type} 
 * @return: 
 */
void NVS_Write(const char* key, void* value, size_t length)
{
    //ESP_LOGI(TAG, "--------------- Start NVS_Write ---------------");

    //NVS操作的句柄，类似于 rtos系统的任务创建返回的句柄！
    nvs_handle mHandleNvs;

    //打开数据库，打开一个数据库就相当于会返回一个句柄
    if (nvs_open(TB_SELF, NVS_READWRITE, &mHandleNvs) != ESP_OK)
    {
        ESP_LOGE(TAG, "Open NVS Table fail");
        return ;
    }

    if (nvs_set_blob(mHandleNvs, key, value, length) != ESP_OK)
        ESP_LOGE(TAG, "Save group  Fail !! key: %s  ", key);
    

    //提交下！相当于软件面板的 “应用” 按钮，并没关闭面板！
    nvs_commit(mHandleNvs);

    //关闭数据库，关闭面板！
    nvs_close(mHandleNvs);

    //ESP_LOGI(TAG, "--------------- End NVS_Write ---------------\n");
}

void NVS_erase_all(void)
{
    nvs_handle mHandleNvs;
    if (nvs_open(TB_SELF, NVS_READWRITE, &mHandleNvs) != ESP_OK)
    {
        ESP_LOGE(TAG, "Open NVS Table fail");
        return ;
    }
    nvs_erase_all(mHandleNvs);
    //提交下！相当于软件面板的 “应用” 按钮，并没关闭面板！
    nvs_commit(mHandleNvs);

    //关闭数据库，关闭面板！
    nvs_close(mHandleNvs);
}


void flash_erase(void)
{
    int i = 0;
    ESP_LOGE(TAG, "erase %x size %d", secB_addr, USER_FLASH_SIZE);
    for(i = 0; i < USER_FLASH_SIZE; i ++)
    {
        spi_flash_erase_sector((secB_addr/SPI_FLASH_SEC_SIZE) +i);
	}
}




