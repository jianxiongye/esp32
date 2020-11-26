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
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "user_wifi.h"
#include "user_sniffer.h"
#include "user_uart.h"
#include "user_sim800c.h"
#include "user_radar.h"
#include "user_bmp.h"
#include "user_bt.h"

static const char *TAG = "user_main";
uint8_t staMacAddr[6];
uint8_t apMacAddr[6];

#if 0
esp_err_t get_chip_id(uint32_t* chip_id){
    esp_err_t status = ESP_OK;
    *chip_id = (REG_READ(0x3FF00050) & 0xFF000000) |
                         (REG_READ(0x3ff0005C) & 0xFFFFFF);
    return status;
}
#endif
void printfChipInfo(void)
{
    uint32_t id = 0;
    //get_chip_id(&id);
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("------------------------------------------------------\r\n");
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    printf("File: %s, \nLine: %d, Date: %s, Time: %s, Timestamp: %s\n", __FILE__, __LINE__, __DATE__, __TIME__, __TIMESTAMP__);    
    //获取IDF版本
    printf("SDK version: %s, chip id: %u\n", esp_get_idf_version(), id);

    esp_read_mac(staMacAddr, ESP_MAC_WIFI_STA);
    printf(" Station esp_wifi_get_mac(): %02x:%02x:%02x:%02x:%02x:%02x \n", 
            staMacAddr[0], staMacAddr[1], staMacAddr[2], staMacAddr[3], staMacAddr[4], staMacAddr[5]);
    esp_read_mac(apMacAddr, ESP_MAC_WIFI_SOFTAP);
    printf(" AP esp_wifi_get_mac(): %02x:%02x:%02x:%02x:%02x:%02x \n", 
        apMacAddr[0], apMacAddr[1], apMacAddr[2], apMacAddr[3], apMacAddr[4], apMacAddr[5]);
    printf("------------------------------------------------------\r\n");
}


void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    gpio_init();
    create_gpio_task();
    uart0_init(115200);
    uart2_init(115200);
    printfChipInfo();
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    BMP280Init();
    wifi_init();
    bt_init();
    sim800c_init();
//    wifi_scan();
    //wifi_init_sta();
   
    //vTaskDelay(10000/portTICK_PERIOD_MS );
    // esp_wifi_disconnect();
    // esp_wifi_stop();
    // ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    //creater_sniffer_task();
    //create_sim800c_task();
    create_uart_task();
    //xTaskCreate(i2c_task_bme280_bmp280_bmp180, "i2c_task_bme280_bmp280_bmp180", 2048, NULL, 10, NULL);
    //create_bt_task();
}
