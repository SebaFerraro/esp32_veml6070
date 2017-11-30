/*
  Integration Times and UVA Sensitivity:
    Rset=240k -> 1T=100.0ms ->   5.000 uW/cm²/step
    Rset=270k -> 1T=112.5ms ->   5.625 uW/cm²/step
    Rset=300k -> 1T=125.0ms ->   6.250 uW/cm²/step
    Rset=600k -> 1T=250.0ms ->  12.500 uW/cm²/step
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include "sdkconfig.h"

#define I2C_VEML6070_ADDR_CMD 0x38
#define I2C_VEML6070_ADDR1 0x38
#define I2C_VEML6070_ADDR2 0x39

//Integration Time
#define IT_1_2 0x0 //1/2T
#define IT_1   0x1 //1T
#define IT_2   0x2 //2T
#define IT_4   0x3 //4T


#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

#define SDA_PIN GPIO_NUM_18
#define SCL_PIN GPIO_NUM_19

static char tag[] = "veml6070";

uint16_t i2c_veml6070_uv(uint8_t cmd_conf) {
	uint16_t uv=0;
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_VEML6070_ADDR_CMD << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    	i2c_master_write_byte(cmd, cmd_conf, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (espRc != ESP_OK) {
                printf("Error write I2C_VEML6070_ADDR_CMD: %d\n",espRc);
		return 0;
	}

        vTaskDelay(500 / portTICK_RATE_MS);
		
    	uint8_t msb=0, lsb=0;
        cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_VEML6070_ADDR1 << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &msb, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (espRc != ESP_OK) {
                printf("Error read I2C_VEML6070_ADDR1: %d\n",espRc);
		return 0;
	}
               
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_VEML6070_ADDR2 << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &lsb, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (espRc != ESP_OK) {
                printf("Error read I2C_VEML6070_ADDR2: %d\n",espRc);
		return 0;
	}
	uv=((uint16_t) msb<<8) | lsb;
	//printf("Datos msb lsb : %d  %d  UV: %d\n", msb, lsb,uv);
        return uv;

}

void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
	
}

