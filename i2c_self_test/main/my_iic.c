/* BMP280&BME280 sensor pressure example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "my_iic.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#if 1
/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init()
{
    i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = 18,
            .scl_io_num = 19,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 1000
    };
    i2c_param_config(I2C_NUM_1, &i2c_config);
    i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);

    esp_err_t esp_retval;  
 
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((0x76) << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_retval = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
    if (esp_retval != ESP_OK) {
        printf("I2C slave NOT working or wrong I2C slave address - error (%i)", esp_retval);
        // LABEL
 
    }
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}


/**
*    @brief follow the NXP I2C protocal write
*
* 1. send data
* ___________________________________________________________________________________________________
* | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
* --------|---------------------------|-------------------------|----------------------------|------|
*
* @param i2c_num I2C port number
* @param reg_address slave reg address
* @param data data to send
* @param data_len data length
*
* @return
*     - ESP_OK Success
*     - ESP_ERR_INVALID_ARG Parameter error
*     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
*     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
*     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
*/
esp_err_t i2c_m_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}




/**
 *  @brief follow the NXP I2C protocal read
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t i2c_m_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 5000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        printf("read 1 %x -- %d \r\n", ret, ret);
        //return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_addr << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 5000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}
#else 
#define SDA_GPIO    18
#define SCL_GPIO    19
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<SDA_GPIO) | (1ULL<<SCL_GPIO))

#define IIC_SDA(x)  gpio_set_level(SDA_GPIO, x)
#define IIC_SCL(x)  gpio_set_level(SCL_GPIO, x)

#define IIC_SDA_GET gpio_get_level(SDA_GPIO)
#define READ_SDA    gpio_get_level(SDA_GPIO)
void SDA_IN(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<SDA_GPIO);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}
void SDA_OUT(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<SDA_GPIO);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}
void IIC_Init(void)
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
    
    IIC_SDA(1);
    IIC_SCL(1);  
}
void delay_us(int us)
{
    vTaskDelay((us * 20) / portTICK_RATE_MS);
    // int a = us * 100;
    // while (a > 0)
    // {
    //     a --;
    // }
    
}

//²úÉúIICÆðÊ¼ÐÅºÅ
void IIC_Start(void)
{
	SDA_OUT();     //sdaÏßÊä³ö
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	delay_us(4);
 	IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL(0);//Ç¯×¡I2C×ÜÏß£¬×¼±¸·¢ËÍ»ò½ÓÊÕÊý¾Ý 

    delay_us(10);
    IIC_SDA(1);	  	  
	IIC_SCL(1);
}

void IIC_Stop(void)
{
	SDA_OUT();//sdaÏßÊä³ö
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL(1); 
	delay_us(4);			
	IIC_SDA(1);//·¢ËÍI2C×ÜÏß½áÊøÐÅºÅ				   	
}

uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDAÉèÖÃÎªÊäÈë  
	IIC_SDA(1);delay_us(1);	   
	IIC_SCL(1);delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
            printf("wait ack fail\r\n");
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(0);//Ê±ÖÓÊä³ö0 	   
	return 0;  
} 
void IIC_Ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	delay_us(5);
	IIC_SCL(1);
	delay_us(5);
	IIC_SCL(0);

    delay_us(5);
    IIC_SDA(1);	  	  
	IIC_SCL(1);
}

void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	delay_us(5);
	IIC_SCL(1);
	delay_us(5);
	IIC_SCL(0);

    delay_us(5);
    IIC_SDA(1);	  	  
	IIC_SCL(1);
}	

void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL(0);//À­µÍÊ±ÖÓ¿ªÊ¼Êý¾Ý´«Êä
    for(t=0;t<8;t++)
    {              
        IIC_SDA((txd&0x80)>>7);
        txd<<=1; 	  
		delay_us(5);   //¶ÔTEA5767ÕâÈý¸öÑÓÊ±¶¼ÊÇ±ØÐëµÄ
		IIC_SCL(1);
		delay_us(5); 
		IIC_SCL(0);	
		delay_us(5);
    }

    delay_us(5);
    IIC_SCL(1);
    IIC_SDA(1);
} 

uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDAÉèÖÃÎªÊäÈë
    delay_us(5);
    for(i=0;i<8;i++ )
	{
        IIC_SCL(0); 
        delay_us(5);
		IIC_SCL(1);
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(5); 
    }
    if (!ack)
        IIC_NAck();//·¢ËÍnACK
    else
        IIC_Ack(); //·¢ËÍACK   
    return receive;
}

esp_err_t i2c_master_init()
{
    IIC_Init();
    
    return ESP_OK;
}

esp_err_t i2c_m_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{

    IIC_Start();
    IIC_Send_Byte(i2c_addr << 1 | WRITE_BIT);
    IIC_Ack();

    while (data_len > 0)
    {
        IIC_Send_Byte(reg_address ++);
        IIC_Ack();
        data_len --;
        IIC_Send_Byte(*data ++);
        IIC_Ack();
    }
    IIC_Stop();
    
    return ESP_OK;
}

esp_err_t i2c_m_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    uint8_t ack = 0;
    IIC_Start();
    IIC_Send_Byte(i2c_addr << 1 | WRITE_BIT);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg_address);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(i2c_addr << 1 | READ_BIT);
    IIC_Wait_Ack();
    while (data_len > 0)
    {
        ack = (data_len == 1 ? 0 : 1);
        *data = IIC_Read_Byte(ack);
        data ++;
        data_len --;
    }
    IIC_Stop();
    return ESP_OK;
}

#endif



