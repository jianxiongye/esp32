
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "user_i2c.h"

static const char *TAG = "i2c-driver";


esp_err_t i2c_mem_write(i2c_port_t i2c_num, uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint8_t data = 0;
    int i = 0;
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        printf("operation i2c start failure\r\n");
        return ret;
    }
    ret = i2c_master_write_byte(cmd, DevAddress << 1 | WRITE_BIT, ACK_CHECK_EN);
    if (ret != ESP_OK) {
        printf("operation i2c write 1 failure\r\n");
        return ret;
    }
#if 0
    while (Size > 0)
    {
        ret = i2c_master_write_byte(cmd, MemAddress + i, ACK_CHECK_EN);
        if (ret != ESP_OK) {
            printf("operation i2c write 2 failure\r\n");
            return ret;
        }
        data = *pData ++;
        Size --;
        i ++;
        i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    }
 #endif
    i2c_master_write_byte(cmd, MemAddress, ACK_CHECK_EN);   
    i2c_master_write(cmd, pData, Size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, Timeout / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        printf("operation i2c failure\r\n");
        return ret;
    }
    return ret;
}

esp_err_t i2c_mem_read(i2c_port_t i2c_num, uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint8_t data = 0;
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DevAddress << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MemAddress, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, Timeout / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        printf("operation i2c write start failure\r\n");
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DevAddress << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, pData, Size, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, Timeout / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


esp_err_t i2c_master_init(void)
{
    i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000
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
 * @brief i2c slave initialization
 */
esp_err_t i2c_slave_init(void)
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}


