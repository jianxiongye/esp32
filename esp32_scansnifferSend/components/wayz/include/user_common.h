

#ifndef  __USER_COMMON_H
#define  __USER_COMMON_H

#include "stdint.h"
#include "freertos/event_groups.h"

#define  ENBLE                      (1)
#define  DISABLE                    (0)

#define  OPEN_PRINTF                (1)
#define  PRINT_LOG_EN               (1)

#if PRINT_LOG_EN
#define  qDebug(...)                printf(__VA_ARGS__)
#else
#define  qDebug(...)  
#endif


/*------------------事件通知 start----------------------*/
#define  START_BIT                  BIT0
#define  WIFI_CONNECTED_BIT         BIT1
#define  OTA_BIT                    BIT2
#define  WIFI_FAIL_BIT              BIT3
#define  BT_SCAN_BIT                BIT4
#define  HYTERA_DATA_FAIL_BIT       BIT5
#define  HYTERA_CLOSE_ACK_BIT       BIT6
/*------------------事件通知 end----------------------*/

/*------------------优先级定义 start----------------------*/
#define  UART_PRIORITY              5
#define  CJSON_PRIORITY             6
#define  HTTP_PRIORITY              7
/*------------------优先级定义 end----------------------*/

/*------------------堆栈大小 start----------------------*/
#define  UART_STACK                 2048
#define  CJSON_STACK                8192
#define  HTTP_STACK                 8192
/*------------------堆栈大小 end----------------------*/

extern EventGroupHandle_t wifi_event_group;

#endif
