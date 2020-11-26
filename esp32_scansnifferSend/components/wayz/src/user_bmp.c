#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "esp_system.h"

#include "user_bmp.h"
#include "user_i2c.h"

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;
int32_t t_fine;

void BMP280Init(void)
{
	uint8_t tData;

    esp_err_t ret = i2c_master_init();
	printf("ret : %d \r\n", ret);
	i2c_mem_read(I2C_MASTER_NUM, ADDR, 0xD0, &tData, 1, I2C_TIMEOUT);
	printf("id : 0x%02x \r\n", tData);
	//软件复位
	tData = 0xb6;
	i2c_mem_write(I2C_MASTER_NUM, ADDR, 0xe0, &tData, 1, I2C_TIMEOUT);
	vTaskDelay( 50 / portTICK_PERIOD_MS );
	
	//id校验 0x58
	//uint8_t id=0;
	//HAL_I2C_Mem_Read(&hi2c2,ADDR,0xd0,1,&id,1,I2C_TIMEOUT);
  
	if(osrs_t != 0)
	{
		tData = osrs_h;
		i2c_mem_write(I2C_MASTER_NUM, ADDR, 0xf2, &tData, 1, I2C_TIMEOUT);
	}
	
	tData = ctrl_meas;
	i2c_mem_write(I2C_MASTER_NUM, ADDR, 0xf4, &tData, 1, I2C_TIMEOUT);
	
	if(mode == 3)
	{
		tData = config;
		i2c_mem_write(I2C_MASTER_NUM, ADDR, 0xf5, &tData, 1, I2C_TIMEOUT);
	}
	BMP280ReadTrim();
	vTaskDelay( 50 / portTICK_PERIOD_MS );
	printf("BMP280ReadP : %f, BMP280ReadT : %f, BMP280ReadH : %f\r\n", BMP280ReadP(), BMP280ReadT(), BMP280ReadH());
}
//检查设备转换状态(1:转换完毕，0:正在转换)
uint8_t BMP280CheckStatus(void)
{
	uint8_t rData;
	i2c_mem_read(I2C_MASTER_NUM, ADDR, 0xf3, &rData, 1, I2C_TIMEOUT);
	return (rData & 0x40) ? 0 : 1;
}
//读取校准值
void BMP280ReadTrim(void)
{
	uint8_t rData[24];
	i2c_mem_read(I2C_MASTER_NUM, ADDR, 0x88, rData, 24, I2C_TIMEOUT);
	dig_T1 = (rData[1] << 8) | rData[0];
	dig_T2 = (rData[3] << 8) | rData[2];
	dig_T3 = (rData[5] << 8) | rData[4];
	dig_P1 = (rData[7] << 8) | rData[6];
	dig_P2 = (rData[9] << 8) | rData[8];
	dig_P3 = (rData[11]<< 8) | rData[10];
	dig_P4 = (rData[13]<< 8) | rData[12];
	dig_P5 = (rData[15]<< 8) | rData[14];
	dig_P6 = (rData[17]<< 8) | rData[16];
	dig_P7 = (rData[19]<< 8) | rData[18];
	dig_P8 = (rData[21]<< 8) | rData[20];
	dig_P9 = (rData[23]<< 8) | rData[22];
	i2c_mem_read(I2C_MASTER_NUM, ADDR, 0xa1, rData, 1, I2C_TIMEOUT);
	dig_H1 = rData[0];
	i2c_mem_read(I2C_MASTER_NUM, ADDR, 0xe1, rData, 7, I2C_TIMEOUT);
  
	dig_H2 = (rData[1]<< 8) | rData[0];
	dig_H3 = rData[2];
	dig_H4 = (rData[4]<< 4) | (0x0F & rData[3]);
	dig_H5 = (rData[5] << 4) | ((rData[4] >> 4) & 0x0F);
	dig_H6 = rData[6];  
}

void BMP280StartMeasure(void)
{
	uint8_t tData;
	uint8_t rData;
	if(mode == 1 || mode == 2)
	{
		
		if(osrs_t != 0)
		{
			i2c_mem_read(I2C_MASTER_NUM, ADDR, 0xf2, &rData, 1, I2C_TIMEOUT);
			tData = osrs_h;
			i2c_mem_write(I2C_MASTER_NUM, ADDR, 0xf2, &tData, 1, I2C_TIMEOUT);
			i2c_mem_read(I2C_MASTER_NUM, ADDR, 0xf2, &rData, 1, I2C_TIMEOUT);
		}
		tData=ctrl_meas;
		i2c_mem_write(I2C_MASTER_NUM, ADDR, 0xf4, &tData, 1, I2C_TIMEOUT);
		
	}
}
//气压计算
double BMP280CompensateP(int32_t adc_P)
{
	//浮点数计算
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}
//气压读取
double BMP280ReadP(void)
{
	int32_t temp;
	uint8_t rData[3];
	while(!BMP280CheckStatus());
	i2c_mem_read(I2C_MASTER_NUM, ADDR, 0xf7, rData, 3, I2C_TIMEOUT);
	
	temp=(rData[0] << 12) | (rData[1] << 4) | (rData[2] >> 4);
	return BMP280CompensateP(temp);
}
//温度计算
double BME280CompensateT(int32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (int32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}
//温度读取
double BMP280ReadT(void)
{
	int32_t temp;
	uint8_t rData[3];
	while(!BMP280CheckStatus());
	i2c_mem_read(I2C_MASTER_NUM, ADDR, 0xfa, rData, 3, I2C_TIMEOUT);
	
	temp = (rData[0] << 12) | (rData[1] << 4) | (rData[2] >> 4);
	return BME280CompensateT(temp);
}
//湿度计算
double BMP280CompensateH(int32_t adc_H)
{
	double var_H;
	var_H = (((double)t_fine) - 76800.0);
	var_H = (adc_H - (((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * var_H)) *
	(((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) / 67108864.0 * var_H *
	(1.0 + ((double)dig_H3) / 67108864.0 * var_H)));
	var_H = var_H * (1.0 - ((double)dig_H1) * var_H / 524288.0);
	if (var_H > 100.0)
	var_H = 100.0;
	else if (var_H < 0.0)
	var_H = 0.0;
	return var_H;
}
//湿度读取
double BMP280ReadH(void)
{
	int32_t temp;
	uint8_t rData[2];
	while(!BMP280CheckStatus());
	i2c_mem_read(I2C_MASTER_NUM, ADDR, 0xfd, rData, 2, I2C_TIMEOUT);
	
	temp = (rData[0] << 8) | rData[1];
	return BMP280CompensateH(temp);
}


