
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "user_radar.h"

#define GPIO_OUTPUT_IO_4    4
#define GPIO_PWK            ((1ULL << GPIO_OUTPUT_IO_4))
#define GPIO_OUTPUT_IO_0    32
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | GPIO_PWK | (1ULL<<18) | (1ULL<<19))
#define GPIO_INPUT_IO_0     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;
int led_sleep_time = LED_LIGHT_MIN; // ms

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void radar_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

static void gpio_task(void* arg)
{
    while(1) {
        gpio_set_level(GPIO_OUTPUT_IO_0, LED_ON);
        vTaskDelay(1000 / portTICK_RATE_MS);
        gpio_set_level(GPIO_OUTPUT_IO_0, LED_OFF);
        vTaskDelay(led_sleep_time / portTICK_RATE_MS);
    }
}

void gpio_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_IO_4, 1);
    
    //gpio_set_level(18, 1);
    //gpio_set_level(19, 1);
    
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(GPIO_OUTPUT_IO_4, 0);
    //change gpio intrrupt type for one pin
    //gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);
}

void create_radar_task(void)
{
    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(radar_task, "radar_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
}

void create_gpio_task()
{
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
}
