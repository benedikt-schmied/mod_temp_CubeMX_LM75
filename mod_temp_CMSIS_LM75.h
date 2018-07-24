/* I2C to use for communications with LM75 */

#define I2C_PORT         I2C1
#define I2C_SCL_PIN      GPIO_PIN_8     // PB6
#define I2C_SDA_PIN      GPIO_PIN_9    // PB7
#define I2C_GPIO_PORT    GPIOB
#define I2C_CLOCK        RCC_APB1Periph_I2C1


/* LM75 defines */
#define LM75_ADDR                     0x90 // LM75 address

/* LM75 registers */
#define LM75_REG_TEMP                 0x00 // Temperature
#define LM75_REG_CONF                 0x01 // Configuration
#define LM75_REG_THYS                 0x02 // Hysteresis
#define LM75_REG_TOS                  0x03 // Overtemperature shutdown


int dev_temperature_LM75__init(uint32_t _i2c_clk_speed);

void LM75_WriteReg(uint8_t reg, uint16_t value);

uint16_t LM75_ReadReg(uint8_t reg);

uint8_t LM75_ReadConf(void);

void LM75_WriteConf(uint8_t value);

void LM75_Shutdown(FunctionalState newstate);

int16_t LM75_Temperature(void);
