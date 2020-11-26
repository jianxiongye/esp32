
#ifndef  USER_SIM800C_H
#define  USER_SIM800C_H

#include "esp_system.h"


#define     TCP_MODE            0
#define     TCP_SERVER          "120.76.100.197"  // 网络上的公共TCP
#define     TCP_PORT            "10002"

#define     UDP_MODE            1
#define     UDP_SERVER          "52.82.44.25"
#define     UDP_PORT            "26200"

#define     TCP_OPEN            1

#if TCP_OPEN
#define     SIM800C_MODE        TCP_MODE
#define     SIM800C_SERVER      TCP_SERVER
#define     SIM800C_PORT        TCP_PORT
#else
#define     SIM800C_MODE        UDP_MODE
#define     SIM800C_SERVER      UDP_SERVER
#define     SIM800C_PORT        UDP_PORT
#endif

#define     SEND_COUNT          13
#pragma pack(push)
#pragma pack(1)

typedef struct _localtime_
{
    uint16_t ALT_year;
    uint8_t  ALT_month;
    uint8_t  ALT_day;
    uint8_t  ALT_hour;
    uint8_t  ALT_minute;
    uint8_t  ALT_second;
}tLocalTime;


#pragma pack(pop)

void sim800c_init(void);
void create_sim800c_task(void);
void getTime(void);
void sim800c_suspend(void);
void sim800c_resume(void);

#endif
