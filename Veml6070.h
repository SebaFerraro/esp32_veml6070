/* VEML6070 SENSOR UV
   Sebastian Ferraro 
   Noviembre 2017
   Este codigo es GPLv3
*/

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

#define SDA_PIN GPIO_NUM_12
#define SCL_PIN GPIO_NUM_13

// Prototipos de Funciones

uint16_t i2c_veml6070_uv(uint8_t cmd_conf);
void i2c_master_init();
uint8_t i2c_veml6070_indexuv(uint16_t uv, uint8_t T);
