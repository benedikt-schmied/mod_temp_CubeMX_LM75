/***************************************************************************//**
 * mod_temp_CubeMX_LM75.c
 *
 * LM75 driver which is based on a current implementation of ST's CubeMX
 * framework

 *  Copyright (C) 2018  Benedikt Schmied
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  github
 *  email: github-ce@benedikt-schmied.de
 *
 ******************************************************************************/

/***************************************************************************//**
 *                                 CHANGELOG
 * YYYY-MM-DD       AUTHOR  Comment
 * 2017-07-17       BS      Initial Creation
 *
 ******************************************************************************/


////////////////////////////////////////////////////////////////////////////////
/// Includes
////////////////////////////////////////////////////////////////////////////////

/* c - runtime */
#include <stdint.h>
#include <stddef.h>

/* system */
#include <stm32l0xx_hal.h>

/* own libs */

/* module (this project) */
#include "mod_temp_CubeMX_LM75.h"
#include "mod_temp_CubeMX_LM75_cfg.h"

////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////

#define M_MOD_TEMP_CUBEMX_LM75_IS_REG(_reg) \
    ( \
        (mod_temp_CubeMX_LM75_reg_temperature <= _reg) && \
        (mod_temp_CubeMX_LM75_reg_cnt > _reg) \
    )

////////////////////////////////////////////////////////////////////////////////
/// Type definitions, structures and unions
////////////////////////////////////////////////////////////////////////////////

/**
 * enumeration
 *
 * @remark there is a macro for checking consistency
 */
enum mod_temp_CubeMX_LM75_reg_selector {
    mod_temp_CubeMX_LM75_reg_temperature,
    mod_temp_CubeMX_LM75_reg_configuration,
    mod_temp_CubeMX_LM75_reg_hyst,
    mod_temp_CubeMX_LM75_reg_os,
    mod_temp_CubeMX_LM75_reg_cnt
};

/**
 * structure: for register accesses (holds the configuration)
 */
struct temp_reg_cfg_attr {
    uint8_t addr;           /*!< absolute address */
    uint8_t width;          /*!< in bytes */
    union {
        uint8_t _ui8;       /*!< unsigned character*/
        uint16_t _ui16;     /*!< unsigned half word */
        uint32_t _ui32;     /*!< unsigned word */
    } data;
};

/**
 * union: holds the return values
 */
union mod_temp_CubeMX_LM75__reg_attr {
    uint8_t _ui8;       /*!< unsigned character*/
    uint16_t _ui16;     /*!< unsigned half word */
    uint32_t _ui32;     /*!< unsigned word */
};

////////////////////////////////////////////////////////////////////////////////
/// (static) variables
////////////////////////////////////////////////////////////////////////////////

I2C_HandleTypeDef hi2c1;

/**
 * @brief configuration
 *
 * @remark this is C99 notation
 */
static struct temp_reg_cfg_attr temp_reg_cfg[4] = {
        {
            .addr   = 0,
            .width  = 2
        },
        {
            .addr   = 1,
            .width  = 1
        },
        {
            .addr   = 2,
            .width  = 2
        },
        {
            .addr   = 3,
            .width  = 2
        }
};

////////////////////////////////////////////////////////////////////////////////
/// (static) function defintions
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief error handler
 */
extern void _Error_Handler(char *, int);

void LM75_WriteReg(uint8_t reg, uint16_t value);

uint16_t LM75_ReadReg(uint8_t reg);

uint8_t LM75_ReadConf(void);

void LM75_WriteConf(uint8_t value);

void LM75_Shutdown(FunctionalState newstate);

int16_t LM75_Temperature(void);

/**
 * @brief internal 'read a register' function
 */
static int mod_temp_CubeMX_LM75__read_reg(enum mod_temp_CubeMX_LM75_reg_selector _sel, union mod_temp_CubeMX_LM75__reg_attr **_reg);


////////////////////////////////////////////////////////////////////////////////
/// (global) function definition
////////////////////////////////////////////////////////////////////////////////


/**
 * mod_temp_CubeMX_LM75__init
 */
int mod_temp_CubeMX_LM75__init(uint32_t _i2c_clk_speed)
{
    /* automatic variables */

    /* executable statements */

    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x00922BFF;
    hi2c1.Init.OwnAddress1      = 0x5A;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0x5A;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
    return 0;
}

/**
 * mod_temp_CubeMX_LM75__get_temperatue
 */
int mod_temp_CubeMX_LM75__get_temperatue(int32_t *_temp)
{
    /* automatic variables */
    int ret;
    union mod_temp_CubeMX_LM75__reg_attr *reg;

    /* executable statements */
    if (NULL == _temp) {
        return -1;
    }

    ret = mod_temp_CubeMX_LM75__read_reg(mod_temp_CubeMX_LM75_reg_temperature, &reg);
    if (0 == ret) {

        /* convert the bit stream for the calleee */
        *_temp  = (reg->_ui16 & 0xFF) << 1;
        *_temp |= (reg->_ui16 >> 8) & 0x1;
        *_temp >>= 1;

        /* return Ok */
        return ret;
    } else {

        /* map all values */
        return -1;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// (static ) function definition
////////////////////////////////////////////////////////////////////////////////

/**
 *
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(hi2c->Instance==I2C1)
    {
        /* USER CODE BEGIN I2C1_MspInit 0 */

        /* USER CODE END I2C1_MspInit 0 */

        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**I2C1 GPIO Configuration
        PB8     ------> I2C1_SCL
        PB9     ------> I2C1_SDA
        */
        GPIO_InitStruct.Pin         = I2C_SCL_PIN | I2C_SDA_PIN;
        GPIO_InitStruct.Mode        = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull        = GPIO_PULLUP;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate   = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Peripheral clock enable */
        __HAL_RCC_I2C1_CLK_ENABLE();
        /* USER CODE BEGIN I2C1_MspInit 1 */

        /* USER CODE END I2C1_MspInit 1 */
    }

}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

    if(hi2c->Instance==I2C1)
    {
    /* USER CODE BEGIN I2C1_MspDeInit 0 */

    /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* USER CODE BEGIN I2C1_MspDeInit 1 */

    /* USER CODE END I2C1_MspDeInit 1 */
  }

}

/**
 * @brief mod_temp_CubeMX_LM75__read_reg
 */
static int mod_temp_CubeMX_LM75__read_reg(enum mod_temp_CubeMX_LM75_reg_selector _sel, union mod_temp_CubeMX_LM75__reg_attr **_reg)
{
    /* automatic variables */
    uint8_t addr;

    /* executable statements */

    /* check, whether this is a valid arguments */
    if (!M_MOD_TEMP_CUBEMX_LM75_IS_REG(_sel)) {
        return -1;
    }
    if (_reg == NULL) {
        return -1;
    }

    /* everything seems to valid, hence start the read procedure */
    addr = temp_reg_cfg[_sel].addr;

    /* first, select the required register (write operation) */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Transmit(&hi2c1, M_MOD_TEMP_CUBEMX_LM75__ADDR, &addr, sizeof(addr), 1000);

    /* second, read from the slave */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Receive(&hi2c1, M_MOD_TEMP_CUBEMX_LM75__ADDR, (void *)&temp_reg_cfg[_sel].data, temp_reg_cfg[_sel].width, 1000);

    *_reg =  &temp_reg_cfg[_sel].data;
    return 0;
}




//// Write 16-bit LM75 register
//void LM75_WriteReg(uint8_t reg, uint16_t value) {
//	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledgment
//	I2C_GenerateSTART(I2C_PORT,ENABLE);
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
//	I2C_Send7bitAddress(I2C_PORT,M_MOD_TEMP_CUBEMX_LM75__ADDR,I2C_Direction_Transmitter); // Send slave address
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
//	I2C_SendData(I2C_PORT,reg); // Send register address
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
//	I2C_SendData(I2C_PORT,(uint8_t)(value >> 8)); // Send high byte
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
//	I2C_SendData(I2C_PORT,(uint8_t)value); // Send low byte
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
//	I2C_GenerateSTOP(I2C_PORT,ENABLE);
//}
//
//// Read value from LM75 configuration register (8 bit)
//uint8_t LM75_ReadConf(void) {
//	uint8_t value;
//
//	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledgment
//	I2C_GenerateSTART(I2C_PORT,ENABLE);
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
//	I2C_Send7bitAddress(I2C_PORT,M_MOD_TEMP_CUBEMX_LM75__ADDR,I2C_Direction_Transmitter); // Send slave address
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
//	I2C_SendData(I2C_PORT,LM75_REG_CONF); // Send register address
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
//	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
//	I2C_Send7bitAddress(I2C_PORT,M_MOD_TEMP_CUBEMX_LM75__ADDR,I2C_Direction_Receiver); // Send slave address for READ
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
//	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
//	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
//	value = I2C_ReceiveData(I2C_PORT);
//
//	return value;
//}
//
//// Write value to LM75 configuration register  (8 bit)
//void LM75_WriteConf(uint8_t value) {
//	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledgment
//	I2C_GenerateSTART(I2C_PORT,ENABLE);
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
//	I2C_Send7bitAddress(I2C_PORT,M_MOD_TEMP_CUBEMX_LM75__ADDR,I2C_Direction_Transmitter); // Send slave address
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
//	I2C_SendData(I2C_PORT,LM75_REG_CONF); // Send register address
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
//	I2C_SendData(I2C_PORT,value);
//	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
//	I2C_GenerateSTOP(I2C_PORT,ENABLE);
//}
//
//// Set LM75 shutdown mode
//// newstate:
////    ENABLE = put LM75 into powerdown mode
////    DISABLE = wake up LM75
//void LM75_Shutdown(FunctionalState newstate) {
//	uint8_t value;
//
//	value = LM75_ReadConf();
//	LM75_WriteConf(newstate == ENABLE ? value | 0x01 : value & 0xFE);
//}
//
//// Read temperature readings from LM75 in decimal format
//// IIIF where:
////   III - integer part
////   F   - fractional part
//// e.g. 355 means 35.5C
//int16_t LM75_Temperature(void) {
//
//	uint16_t raw;
//	int16_t temp;
//
//	raw = LM75_ReadReg(LM75_REG_TEMP) >> 7;
//	if (raw & 0x0100) {
//		// Negative temperature
//		temp = -10 * (((~(uint8_t)(raw & 0xFE) + 1) & 0x7F) >> 1) - (raw & 0x01) * 5;
//	} else {
//		// Positive temperature
//		temp = ((raw & 0xFE) >> 1) * 10 + (raw & 0x01) * 5;
//	}
//
//	return temp;
//}
