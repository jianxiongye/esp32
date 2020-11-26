
#ifndef  __USER_BT_H
#define  __USER_BT_H

#include "esp_system.h"

#define BT_GAP_MAX_BDNAME_LEN             (248)
#define BT_GAP_EIR_DATA_LEN               240
/// Bluetooth address length
#define ESP_BD_ADDR_LEN                   6
#define ESP_BT_ADDR_LEN                   18
#define SCAN_MAX_COUNT                    20


/// Bluetooth device address
typedef uint8_t esp_bd_addr_t[ESP_BD_ADDR_LEN];
#pragma  pack(push)
#pragma  pack(1)

typedef enum {
    APP_GAP_STATE_IDLE = 0,
    APP_GAP_STATE_DEVICE_DISCOVERING,
    APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE,
    APP_GAP_STATE_SERVICE_DISCOVERING,
    APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE,
} app_gap_state_t;

typedef struct {
    bool dev_found;
    uint8_t bdname_len;
    uint8_t eir_len;
    uint8_t rssi;
    uint32_t cod;
    uint8_t eir[BT_GAP_EIR_DATA_LEN];
    uint8_t bdname[BT_GAP_MAX_BDNAME_LEN + 1];
    esp_bd_addr_t bda;
    app_gap_state_t state;
} app_gap_cb_t;

typedef struct {
    uint8_t bt_mac[ESP_BT_ADDR_LEN];
    uint32_t cod;
    int32_t rssi;
    uint8_t bdname[BT_GAP_MAX_BDNAME_LEN + 1];
}tBTScanInfo;


#pragma  pack(pop)

void bt_init(void);
void create_bt_task(void);
void bt_suspend();
void bt_resume();


extern tBTScanInfo gBtScanInfo[SCAN_MAX_COUNT];

#endif


