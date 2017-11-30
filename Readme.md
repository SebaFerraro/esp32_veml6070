/**
 * Main file for running the BLE samples.
 */
//extern "C" {
//	void app_main(void);
//}
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include "sdkconfig.h"
#include "Veml6070.h"
//void Veml6070(void);


//
// Un-comment ONE of the following
//            ---
void app_main(void) {
i2c_master_init();
uint8_t sal=0;
uint8_t dat=0;
uint16_t uv=0;
dat=((IT_1<<2) | 0x02);
while(1){
        uv=i2c_veml6070_uv(dat);
        //uv=I2C_VEML6070_READ(I2C_VEML6070_ADDR1);
        printf("READ %d UV: %d\n",sal,uv);
        vTaskDelay(2000 / portTICK_RATE_MS);
        }
}
