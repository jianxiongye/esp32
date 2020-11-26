
#ifndef __USER_RADAR_H
#define __USER_RADAR_H

#include "esp_system.h"

#define  LED_ON             1
#define  LED_OFF            0
#define  LED_LIGHT_MIN      1000


void gpio_init(void);
void create_radar_task(void);
void create_gpio_task(void);

#endif

