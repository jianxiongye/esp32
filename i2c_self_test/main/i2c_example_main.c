/* main
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "BME280_BMP280_BMP180.h"




void app_main(void)
{
    //start i2c task
    xTaskCreate(i2c_task_bme280_bmp280_bmp180, "i2c_task_bme280_bmp280_bmp180", 4096, NULL, 10, NULL);
}
