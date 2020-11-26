
#ifndef  __USER_BMP_H
#define  __USER_BMP_H

#include "esp_system.h"

/*****************************配置区****************************/
//I2C超时
#define  I2C_TIMEOUT    1000
#define  ADDR           0x76//0xec
//温度过采样
#define  osrs_t         1
//湿度过采样
#define  osrs_h         1
//压力过采样
#define  osrs_p         1
//工作模式(1或2:被动模式，测量完毕进入休眠,需要再次测量时重新设置model=1)
#define  mode           3
//主动模式时的转换间隔(4:0.5s)
#define  t_sb           4
//滤波器(0:关闭)
#define  filter         0
/*******************************配置区*****************************/
//速率/滤波控制寄存器
#define  config         (t_sb << 5 | filter << 2)
//测量控制寄存器
#define ctrl_meas       (osrs_t << 5 | osrs_p << 2 | mode)


void BMP280Init(void);
uint8_t BMP280CheckStatus(void);
void BMP280ReadTrim(void);
void BMP280StartMeasure(void);
double BMP280CompensateP(int32_t adc_P);
double BMP280ReadP(void);
double BME280CompensateT(int32_t adc_T);
double BMP280ReadT(void);
double BMP280CompensateH(int32_t adc_H);
double BMP280ReadH(void);

#endif

