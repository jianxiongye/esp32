
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_spi_flash.h"

#include "user_uart.h"
#include "user_sim800c.h"
#include "user_sniffer.h"
#include "user_nvs.h"

#define GPIO_OUTPUT_IO_4    4
#define GPIO_PWK            ((1ULL << GPIO_OUTPUT_IO_4))

static const char *TAG = "user_sim800c";

static tLocalTime gLocalTime = {0};

const char *modetbl[2]={ "TCP", "UDP" };//连接模式
#define  SEND_BUF_LEN       4096
#define  SEND_FLAG          1
static char sendBuf[SEND_BUF_LEN] = {0};

static TaskHandle_t sim800cHandle;

#define  START_STNP_EN      1

#if START_STNP_EN

#define  TIME_LEN   64
char bufTime[TIME_LEN] = {0};

#endif


uint8_t* sim800c_check_cmd(char *str)
{
	char *strx = 0;
    uart2_recv();
	strx = strstr((const char*)UART2_RX_BUF, str);

	return (uint8_t *)strx;
}

uint8_t sim800c_send_cmd(char *cmd, char *ack, int waittime)
{
    uart2_rx_flag = 0;
    uint8_t ret = 0;
    uart2_send(cmd, strlen(cmd));
    uart2_send("\r\n", strlen("\r\n"));

    if (ack && waittime)
    {
        while (-- waittime)
        {
            vTaskDelay( 10 / portTICK_PERIOD_MS );
            if (sim800c_check_cmd(ack))
            {
                uart2_rx_flag = 0;
                break;
            }
        }
        if(waittime == 0)
            ret = 1; 
    }
    return ret;
}

//NTP网络同步时间
void ntp_update(void)
{  
	sim800c_send_cmd("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 200);//配置承载场景1
	sim800c_send_cmd("AT+SAPBR=3,1,\"APN\",\"UNIWAP\"", "OK", 200);
	sim800c_send_cmd("AT+SAPBR=1,1", "OK", 200);                        //激活一个GPRS上下文
    //sim800c_send_cmd("AT+SAPBR=2,1", "OK", 200);                        //获取分配IP地址
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    sim800c_send_cmd("AT+CNTPCID=1", "OK", 200);                     //设置CNTP使用的CID
	sim800c_send_cmd("AT+CNTP=\"ntp1.aliyun.com\",32", "OK", 200);     //设置NTP服务器和本地时区(32时区 时间最准确)
    sim800c_send_cmd("AT+CNTP", "+CNTP:", 600);                    //同步网络时间
    sim800c_send_cmd("AT+CCLK?", "OK", 200);                       // 获取时间
}

void sim800c_product_info(void)
{
    uint8_t *p = NULL;
    p = (uint8_t *)malloc(50);
    if(sim800c_send_cmd("AT+CGMI", "OK", 200) == 0)				// 查询制造商名称
	{
		sprintf((char *)p, "manufacturers:%s", UART2_RX_BUF + 2); // 前两个字节为\r\n,需要去除掉
        printf("2G %s \r\n", p);
		uart2_rx_flag = 0;
	} 
    if(sim800c_send_cmd("AT+CGMM", "OK", 200) == 0)// 查询模块的名字
	{ 
		sprintf((char*)p, "module type:%s", UART2_RX_BUF + 2);
        printf("2G %s \r\n", p);
		uart2_rx_flag = 0;
	}
    if(sim800c_send_cmd("AT+CGSN", "OK", 200)==0)//查询产品序列号
	{ 
		sprintf((char*)p, "serial number:%s", UART2_RX_BUF + 2);
        printf("2G %s \r\n", p);
		uart2_rx_flag = 0;		
	}

    free(p);
    p = NULL;
}


static uint8_t sendto(char *data)
{
    char endFlag = 0X1A;
    if(sim800c_send_cmd("AT+CIPSEND", ">", 200) == 0)//发送数据
    {
        sim800c_send_cmd(data, "0", 0);	//发送数据:0X00  
        vTaskDelay( 50 / portTICK_PERIOD_MS );			//必须加延时
        sim800c_send_cmd((char *)&endFlag, "SEND OK", 500);	//CTRL+Z,结束数据发送,启动一次传输	
        //uart2_send((char *)&endFlag, 1);
        printf("send data success\r\n");
    }else 
    {
        sim800c_send_cmd((char *)0X1B, 0, 0);	//ESC,取消发送 	
    }
    return 1;
}

static void send_data(void)
{
    int i = 0, j = 0;
    
    int cycles = 0;
    int div = tInfoMisc.record / SEND_COUNT;
    int mod = tInfoMisc.record % SEND_COUNT;

    memset(sendBuf, 0, sizeof (sendBuf));
    ESP_LOGI(TAG, "----------------------------------");

#if START_STNP_EN
    getTime();
    strncpy(sendBuf, bufTime, TIME_LEN);
#endif

    for (i = 0; i <= div; i ++)
    {
        if (i != div)
        {
            cycles = SEND_COUNT;
        }
        else if (i == div && 0 != mod)
        {
            cycles = mod;
        }
        else if (i == div && 0 == mod)
        {
            cycles = 0;
            break;
        }

        //memset(sendBuf, 0, sizeof (sendBuf));

        for (j = 0; j < cycles; j ++)
        {
            //strncpy(sendBuf + j * SNIFFER_DATA_LEN, nvsData + i * SEND_COUNT * SNIFFER_DATA_LEN  + j * SNIFFER_DATA_LEN, SNIFFER_DATA_LEN);
            if (ENBLE == tInfoMisc.fifoFlag)
            {
                spi_flash_read(secB_addr + ((uint32_t)sizeof(des_addr) * (i * SEND_COUNT + j)), des_addr, (uint32_t)sizeof(des_addr));
            }
            else
            {
                memcpy(des_addr, nvsData + (i * SEND_COUNT + j) * SNIFFER_DATA_LEN, SNIFFER_DATA_LEN);
            }
#if START_STNP_EN
            strncpy(sendBuf + j * SNIFFER_DATA_LEN + TIME_LEN, des_addr, SNIFFER_DATA_LEN);
#else
            strncpy(sendBuf + j * SNIFFER_DATA_LEN, des_addr, SNIFFER_DATA_LEN);
#endif
            
            //vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        

#if 0
        ESP_LOGI(TAG, "----------%d------", i);
        for (j = 0; j < cycles; j ++)
            ESP_LOGI(TAG, "%s", sendBuf + j * SNIFFER_DATA_LEN);
#else
        //ESP_LOGI(TAG, "%s", sendBuf);
#endif
#if SEND_FLAG
        //int err = send(sock, sendBuf, cycles * SNIFFER_DATA_LEN, 0);
    #if START_STNP_EN
            //int err = sendto(sendBuf, cycles * SNIFFER_DATA_LEN + TIME_LEN);
            int err = sendto(sendBuf);
    #else
            int err = sendto(sendBuf, cycles * SNIFFER_DATA_LEN);
    #endif
        if (err < 0) {
            ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "send %d data success by udp\r\n", i);
#endif
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    tInfoMisc.record = 0;
    memset(nvsData, 0, sizeof (nvsData));
}


//tcp/udp测试
//带心跳功能,以维持连接
//mode:0:TCP测试;1,UDP测试)
//ipaddr:ip地址
//port:端口 
// 最大可以一次发送 1352 字节
void sim800c_tcpudp_test(uint8_t mode, char *ipaddr, char *port)
{
    uint8_t *p;
    uint8_t connectsta = 0; // 0,正在连接; 1,连接成功; 2,连接关闭; 
    p = (uint8_t *)malloc(100);
    sprintf((char *)p, "AT+CIPSTART=\"%s\",\"%s\",\"%s\"", modetbl[mode], ipaddr, port);
	if(sim800c_send_cmd((char *)p, "CONNECT OK", 500)) return;		//发起连接

    sim800c_send_cmd("AT+CIPSTATUS", "OK", 500);	//查询连接状态
    if(strstr((const char*)UART2_RX_BUF, "CLOSED"))
        connectsta = 2;
    if(strstr((const char*)UART2_RX_BUF, "CONNECT OK"))
        connectsta = 1;
    printf("connectsta: %d\r\n", connectsta);
    if (1 == connectsta)
    {
        printf("---------\r\n");
       
        send_data();

    }

    sim800c_send_cmd("AT+CIPCLOSE=1", "CLOSE OK", 500);	//关闭连接
	sim800c_send_cmd("AT+CIPSHUT", "SHUT OK", 500);		//关闭移动场景 
    free(p);
    p = NULL;
}

uint8_t sim800c_gprs_init(void)
{
    //uint8_t ipbuf[16]; 		//IP缓存
    //sim800c_send_cmd("AT+CIPCLOSE=1", "CLOSE OK", 100);	//关闭连接
	sim800c_send_cmd("AT+CIPSHUT", "SHUT OK", 100);		//关闭移动场景 
	if(sim800c_send_cmd("AT+CGCLASS=\"B\"", "OK", 1000)) 
        return 1;				//设置GPRS移动台类别为B,支持包交换和数据交换 
	if(sim800c_send_cmd("AT+CGDCONT=1,\"IP\",\"CMNET\"", "OK",1000)) 
        return 2;//设置PDP上下文,互联网接协议,接入点等信息
	if(sim800c_send_cmd("AT+CGATT=1", "OK", 500)) 
        return 3;					//附着GPRS业务
	if(sim800c_send_cmd("AT+CIPCSGP=1,\"CMNET\"", "OK", 500)) 
        return 4;	 	//设置为GPRS连接模式
	//if(sim800c_send_cmd("AT+CIPHEAD=1", "OK", 500)) 
    //    return 5;	 				//设置接收数据显示IP头(方便判断数据来源)
	//ipbuf[0]=0; 	
    //sim800c_tcpudp_test(SIM800C_MODE, SIM800C_SERVER, SIM800C_PORT);
    return 0;
}

void getTime(void)
{
    sim800c_send_cmd("AT+CCLK?", "OK", 200);                       // 获取时间
    //分解时间
    char *timePtr = strstr((const char*)UART2_RX_BUF,(const char*)"CCLK:");
    if (NULL == timePtr)
    {
        printf("---parse time info failure.---\r\n");
        return ;
    }
    gLocalTime.ALT_year    = 2000 + 10 * (timePtr[7] - '0') + timePtr[8] - '0';
    gLocalTime.ALT_month   = 10 * (timePtr[10] - '0') + timePtr[11] - '0';
    gLocalTime.ALT_day     = 10 * (timePtr[13] - '0') + timePtr[14] - '0';
    gLocalTime.ALT_hour    = 10 * (timePtr[16] - '0') + timePtr[17] - '0';
    gLocalTime.ALT_minute  = 10 * (timePtr[19] - '0') + timePtr[20] - '0';
    gLocalTime.ALT_second  = 10 * (timePtr[22] - '0') + timePtr[23] - '0';

    printf("-----%04d-%02d-%02d %02d:%02d:%02d \r\n", gLocalTime.ALT_year, gLocalTime.ALT_month, gLocalTime.ALT_day, \
                                                        gLocalTime.ALT_hour, gLocalTime.ALT_minute, gLocalTime.ALT_second);
    sprintf(bufTime, "%04d-%02d-%02d %02d:%02d:%02d\n", gLocalTime.ALT_year, gLocalTime.ALT_month, gLocalTime.ALT_day, \
                                                        gLocalTime.ALT_hour, gLocalTime.ALT_minute, gLocalTime.ALT_second);
}

void sim800c_init(void)
{
#if 0
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_PWK;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_IO_4, 0);
#endif

    while (sim800c_send_cmd("AT", "OK", 100))
    {
        ESP_LOGW(TAG, "no 2G module detected");
        vTaskDelay( 500 / portTICK_PERIOD_MS );
        ESP_LOGW(TAG, "try to connect 2G modules");
        vTaskDelay( 500 / portTICK_PERIOD_MS );
    }
    //sim800c_send_cmd("ATE0", "OK", 200);//关闭回显
    sim800c_product_info();
    ntp_update(); // 同步时间
    vTaskDelay( 10 / portTICK_PERIOD_MS );
    getTime();
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    sim800c_gprs_init();
}


static void sim800c_event_task(void *pvParameters)
{
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    sim800c_gprs_init();
    
    while (1)
    {
        sim800c_suspend();
        sim800c_tcpudp_test(SIM800C_MODE, SIM800C_SERVER, SIM800C_PORT);
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
        sniffer_resume();
    }

    vTaskDelete(NULL);
}

void sim800c_suspend(void)
{
    vTaskSuspend(sim800cHandle);
}

void sim800c_resume(void)
{
    vTaskResume(sim800cHandle);
}

void create_sim800c_task(void)
{
    xTaskCreate(sim800c_event_task, "sim800c_event_task", 2048, NULL, 7, &sim800cHandle);
}

