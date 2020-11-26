
#ifndef  __USER_I2C_H
#define  __USER_I2C_H

#include "esp_system.h"

#define _I2C_NUMBER(num)                I2C_NUM_##num
#define I2C_NUMBER(num)                 _I2C_NUMBER(num)

#define DATA_LENGTH                     512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH                  128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS     1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO                5               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO                4               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM                   I2C_NUMBER(0) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN            (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN            (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO               19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO               18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM                  I2C_NUM_1//I2C_NUMBER(1) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ              100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE       0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE       0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT                       I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT                        I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN                    0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                   0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL                         0x0                             /*!< I2C ack value */
#define NACK_VAL                        0x1                            /*!< I2C nack value */
#define ESP_SLAVE_ADDR                  0x00

esp_err_t i2c_mem_write(i2c_port_t i2c_num, uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
esp_err_t i2c_mem_read(i2c_port_t i2c_num, uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
esp_err_t i2c_master_init(void);
esp_err_t i2c_slave_init(void);

#endif
