
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <sys/unistd.h>
#include <sys/fcntl.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_partition.h"
#include "esp_spi_flash.h"

#include "user_sniffer.h"
#include "user_sim800c.h"
#include "user_nvs.h"

esp_timer_handle_t periodic_timer;

uint8_t temp_mac1[6] = {0xc4, 0x6a, 0xb7, 0x9f, 0xcc, 0x34};
uint8_t staMacAddr[6];
uint8_t apMacAddr[6];
tMiscInfo tInfoMisc = {0};
//uint32_t secB_addr = 0x330000;
char des_addr[(SNIFFER_DATA_LEN / 4 + 1) * 4] = {0};
char nvsData[FIFO_COUNT * SNIFFER_DATA_LEN] = {0};
static const char *TAG = "user_sniffer";
xTaskHandle snifferHandle;

void diable_channel_hop(void);

void writeSniffDataToFlash(char* data)
{
#if 1
    int i = 0;
    //char aucMacBroadCast[] = "FF:FF:FF:FF:FF:FF"; // 广播的mac地址
	//char aucMacMultiCast[] = "01:00:5E"; // 多播的mac地址

    if (tInfoMisc.record < OVERFLOW)
    {
        for (i = 0; i < tInfoMisc.record; i ++)
        {
		#if 0
			if (0 == strncmp(nvsData + i * SNIFFER_DATA_LEN, data, 35))
			{
				printf("-------- data are equal -------------\r\n");
				return ;
			}
		#endif
			if (ENBLE == tInfoMisc.fifoFlag)
			{
				spi_flash_read(secB_addr + (sizeof (des_addr) * i), des_addr, sizeof (des_addr));
			}
			else
			{
				memcpy(des_addr, nvsData + i * SNIFFER_DATA_LEN, SNIFFER_DATA_LEN);
			}
			if (0 == strncmp(des_addr + MAC_LEN, data + MAC_LEN, 35))
			{
				if (ENBLE == tInfoMisc.fifoFlag)
				{
					spi_flash_write(secB_addr + (sizeof (des_addr) * i), data, \
							(uint32_t)((strlen(data) / 4) + 1) * 4);
				}
				else
				{
					memcpy(nvsData + i * SNIFFER_DATA_LEN, des_addr, SNIFFER_DATA_LEN);
				}
				qDebug("-------- data are equal -------------\r\n");
				return ;
			}

#if 0
			if (0 == strncmp(aucMacBroadCast, data, strlen(aucMacBroadCast)) ||  
				0 == strncmp(aucMacMultiCast, data, 8) ||
				0 == strncmp(aucMacBroadCast, data + MAC_LEN, strlen(aucMacBroadCast)) ||  
				0 == strncmp(aucMacMultiCast, data + MAC_LEN, 8))
			{
				printf("-------- data are equal broadcast -------------\r\n");
				return ;
			}
#endif
        }

		if (ENBLE == tInfoMisc.fifoFlag)
		{
			spi_flash_write(secB_addr + (sizeof (des_addr) * tInfoMisc.record), data, \
							(uint32_t)((strlen(data) / 4) + 1) * 4);
			//spi_flash_read(secB_addr + (sizeof (des_addr) * tInfoMisc.record), des_addr, sizeof (des_addr));
			//ESP_LOGI(TAG, "%s", des_addr);
			//spi_flash_read(secB_addr + ((uint32_t)((os_strlen(data)/4)+1)*4*record), des_addr,         (uint32_t)((os_strlen(data)/4)+1)*4);
		}
		else
		{
			strcpy(nvsData + tInfoMisc.record * SNIFFER_DATA_LEN, data);
		}
		

        tInfoMisc.record ++;
    }
    else
    {
        diable_channel_hop();
    }
#endif
}

static void sniffer_cb(void* buf, wifi_promiscuous_pkt_type_t type)
{
    wifi_pkt_rx_ctrl_t* rx_ctrl = (wifi_pkt_rx_ctrl_t*)buf;
    uint8_t* frame = (uint8_t*)(rx_ctrl + 1);
    uint8_t i = 0; 
    char fifo[SNIFFER_DATA_LEN + 2] = {0};
	struct framecontrol_t *framecontrol;

	framecontrol = (struct framecontrol_t*)(buf + 12);

    switch (type) {
        case WIFI_PKT_MGMT:
			if(framecontrol->subtype == SUBTYPE_PRBREQ \
				|| framecontrol->subtype == SUBTYPE_ASSREQ \
				|| framecontrol->subtype == SUBTYPE_RESREQ)//00|0d  00|00
				//|| framecontrol->subtype == SUBTYPE_ACTION 00|13 判断不了发送和接受端
				goto printDS;
			else if(framecontrol->subtype == SUBTYPE_RESREP \
				  ||framecontrol->subtype == SUBTYPE_ASSREP \
				  ||framecontrol->subtype == SUBTYPE_PRBREP)//00|01
				goto printSD;
			//认证的时候，ap也会发送数据，需要通过BSSID判断发送方
			else if(framecontrol->subtype == SUBTYPE_AUTH ){ 
				if(memcmp(&frame[4], &frame[16], 6)){
					goto printSD;
				}else
				{
					goto printDS;
				}	
			}
			return;

        case WIFI_PKT_CTRL:
            return;

        case WIFI_PKT_DATA:
            	//ap发出的帧,可以用来当作探针数据
				//datadir为0时为IBSS，设备都是对等的，无需此模式;为3时是WDS中继，也不需要的
			switch(framecontrol->datadir){
				case 0:
					break;
				case DATA_DIR_TOAP:
					goto printDS;
					break;
				case DATA_DIR_FROMAP:
					if(!memcmp(&frame[10], &frame[16], 6))
					{
						goto printSD;
					}	
					else
					{
						goto print32;
					}	
					break;
				default:
					break;
			}
            break;

        case WIFI_PKT_MISC:
            return;
            break;

        default :
			return;
    }
	return;
				
printSD:
	//打印本机stamac
	// 缓存上次的MAC，避免重复打印
	if(0 != frame[4] % 4)//如果不能被4整除除，那就不是我们需要的mac
		return;
	// 如果MAC地址和上一次一样就返回
	if(0==memcmp(temp_mac1, &frame[4], 6)){
		return;
	}
	for (i=0; i<6; i++){
		temp_mac1[i] = frame[i+4];
	}
	//check 真实mac地址

#if 0	
	sprintf(fifo, ""MACPRINT"|"MACPRINT"|%02d|%02d|%02d|%d|0|%d|%d\n", PRINT(frame,4), PRINT(frame,10),\
            framecontrol->frametype, framecontrol->subtype, rx_ctrl->channel, rx_ctrl->rssi, framecontrol->datadir, tInfoMisc.minute);
#endif
	sprintf(fifo, "%02X:%02X:%02X:%02X:%02X:%02X|%02X:%02X:%02X:%02X:%02X:%02X|%02X:%02X:%02X:%02X:%02X:%02X|%02d|%02d|%02d|%d|0|%d|0\n", \
            PRINT(staMacAddr, 0), PRINT(frame,4), PRINT(frame,10),\
            framecontrol->frametype, framecontrol->subtype, rx_ctrl->channel, rx_ctrl->rssi, framecontrol->datadir);

	//memcpy(mstore, &frame[4], 6);
    qDebug("printSD: %s", fifo);
	writeSniffDataToFlash(fifo);
	return;

print32:
	if(0 != frame[16] % 4)//如果不能被4整除除，那就不是我们需要的mac
		return;

	// 缓存上次的MAC，避免重复打印
	if(0==memcmp(temp_mac1, &frame[16], 6)){
		return;
	}
	for (i = 0; i < 6; i ++){
		temp_mac1[i] = frame[i+16];
	}
#if 0
	sprintf(fifo, ""MACPRINT"|"MACPRINT"|%02d|%02d|%02d|%d|0|%d|%d\n", PRINT(frame,16), PRINT(frame,10),\
            framecontrol->frametype, framecontrol->subtype, rx_ctrl->channel, rx_ctrl->rssi, framecontrol->datadir, tInfoMisc.minute);
#endif
	sprintf(fifo, "%02X:%02X:%02X:%02X:%02X:%02X|%02X:%02X:%02X:%02X:%02X:%02X|%02X:%02X:%02X:%02X:%02X:%02X|%02d|%02d|%02d|%d|0|%d|0\n", \
        PRINT(staMacAddr, 0), PRINT(frame,16), PRINT(frame,10),\
		framecontrol->frametype, framecontrol->subtype, rx_ctrl->channel, rx_ctrl->rssi, framecontrol->datadir);

	//memcpy(mstore, &frame[16], 6);
    qDebug("print32: %s", fifo);
	writeSniffDataToFlash(fifo);
	return ;

printDS:
	if(0 != frame[10] % 4)//如果不能被4整除除，那就不是我们需要的mac
		return;
	if(0 == memcmp(temp_mac1, &frame[10], 6)){
		return;
	}
	for (i = 0; i < 6; i ++){
		temp_mac1[i] = frame[i+10];
	}
#if 0
	sprintf(fifo, ""MACPRINT"|"MACPRINT"|%02d|%02d|%02d|%d|0|%d|%d\n", PRINT(frame,10), PRINT(frame,4),\
            framecontrol->frametype, framecontrol->subtype, rx_ctrl->channel, rx_ctrl->rssi, framecontrol->datadir, tInfoMisc.minute);
#endif
	sprintf(fifo, ""MACPRINT"|"MACPRINT"|"MACPRINT"|%02d|%02d|%02d|%d|0|%d|0\n", PRINT(staMacAddr, 0), PRINT(frame,10), PRINT(frame,4),\
		framecontrol->frametype, framecontrol->subtype, rx_ctrl->channel, rx_ctrl->rssi, framecontrol->datadir);
	//memcpy(mstore, &frame[10], 6);
    qDebug("printDS: %s", fifo);
	writeSniffDataToFlash(fifo);
	return;
}

void diable_channel_hop()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));

	sim800c_resume(); // 唤醒传输数据

#if PRINT_TIME_EN
	time_t now;
	struct tm timeinfo;

	time(&now);
	localtime_r(&now, &timeinfo);
	printf("%04d-%02d-%02d %02d:%02d:%02d sniffer_task end\n", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
				timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
#endif

	ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    ESP_ERROR_CHECK(esp_timer_delete(periodic_timer));
	qDebug("diable_channel_hop: %d \n", tInfoMisc.record);
	//xEventGroupSetBits(wifi_event_group, SEND_BIT);
}

void channelHopFunc(void *arg)
{
    static uint8_t new_channel = CONFIG_CHANNEL;
    uint8_t sendDataTime = 0;
	new_channel ++;
	if(new_channel > MAX_CHN)
	{
		new_channel = CONFIG_CHANNEL;
	}

    esp_wifi_set_channel(new_channel, WIFI_SECOND_CHAN_NONE);
	tInfoMisc.count ++;
	if(tInfoMisc.count % TIMEOUT_COUNT == 0){
		tInfoMisc.minute ++;
		if (tInfoMisc.minute < SNIFFER_MIN_TIME)
		{
			qDebug("time is up %d seconds\n", tInfoMisc.minute);
		}
	}

    sendDataTime = tInfoMisc.tuartWifiInfo.mSniffSendMin >= SNIFFER_MIN_TIME ? tInfoMisc.tuartWifiInfo.mSniffSendMin : SNIFFER_MIN_TIME;
	if(tInfoMisc.minute >= sendDataTime){
		qDebug("time is up %d seconds send data now\n", tInfoMisc.minute);
		tInfoMisc.minute = 0;
		tInfoMisc.count = 0;
		diable_channel_hop();
	}
}

static void sniffer_task(void* pvParameters)
{
    wifi_promiscuous_filter_t sniffer_filter = {0};
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &channelHopFunc,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic"
    };
	while(1){
		qDebug("----start------sniffer\n");
		//esp_wifi_disconnect();
		//ESP_ERROR_CHECK(esp_wifi_scan_stop());
        //wifi_scan();

		//if (ENBLE == tInfoMisc.fifoFlag)
		{
			//flash_erase();
		}
#if 1
        sniffer_filter.filter_mask |= WIFI_PROMIS_FILTER_MASK_ALL;
#endif

		if (sniffer_filter.filter_mask == 0) {
			ESP_LOGI(TAG, "Please add one filter at least!");
			vTaskDelete(NULL);
		}

		//xEventGroupWaitBits(wifi_event_group, START_BIT, false, true, portMAX_DELAY);
		//ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_CHANNEL, 0));
		ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(sniffer_cb));
		ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&sniffer_filter));
		ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
        esp_wifi_set_channel(CONFIG_CHANNEL, WIFI_SECOND_CHAN_NONE);
#if PRINT_TIME_EN
		time_t now;
		struct tm timeinfo;

        time(&now);
        localtime_r(&now, &timeinfo);
        printf("%04d-%02d-%02d %02d:%02d:%02d sniffer_task start\n", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
		            timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
#endif

	#if HOP_JUMP_ENABLE // 1min 切换一个通道

        ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMEOUT * 1000));
	#endif
        qDebug("sniffer_task resume!\n");
		vTaskSuspend(NULL);
	}

	while(1)
    {
		vTaskDelay(1000/portTICK_PERIOD_MS );
		qDebug("working...\n");
	}
	vTaskDelete(NULL);
}

void sniffer_suspend(void)
{
    vTaskSuspend(snifferHandle);
}

void sniffer_resume(void)
{
    vTaskResume(snifferHandle);
}

void creater_sniffer_task(void)
{
    esp_read_mac(staMacAddr, ESP_MAC_WIFI_STA);
    esp_read_mac(apMacAddr, ESP_MAC_WIFI_SOFTAP);
    xTaskCreate(sniffer_task, "sniffer_task", 2048, NULL, 11, &snifferHandle);
}

