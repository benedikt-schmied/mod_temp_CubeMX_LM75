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

#define M_MOD_TEMP_CUBEMX_LM75_IS_GOTO(_goto) \
    ( \
        (M_MOD_TEMP_CUBEMX_LM75__GOTO_SLEEP == _goto) || \
        (M_MOD_TEMP_CUBEMX_LM75__GOTO_RUN   == _goto) \
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

/**
 * @brief internal 'write a register' function
 */
static int mod_temp_CubeMX_LM75__write_reg(enum mod_temp_CubeMX_LM75_reg_selector _sel, union mod_temp_CubeMX_LM75__reg_attr *_reg);


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

/**
 * mod_temp_CubeMX_LM75__get_temperatue
 */
int mod_temp_CubeMX_LM75__define_mode(unsigned _goto)
{
    /* automatic variables */
    int ret;
    union mod_temp_CubeMX_LM75__reg_attr reg;

    /* executable statements */
    if (!M_MOD_TEMP_CUBEMX_LM75_IS_GOTO(_goto)) {
        return -1;
    }

    /*
     *  we probably have to read from the register first and mask all other
     *  values
     */

    /* switch - case - statement */
    switch (_goto) {
    case M_MOD_TEMP_CUBEMX_LM75__GOTO_SLEEP:
        reg._ui8 = 0;
        break;
    case M_MOD_TEMP_CUBEMX_LM75__GOTO_RUN:
        reg._ui8 = 0;
        break;
    default:
        break;
    } /* end of switch - case - statement */

    /*  */
    ret = mod_temp_CubeMX_LM75__write_reg(mod_temp_CubeMX_LM75_reg_temperature, &reg);
    if (0 == ret) {
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


/**
 * @brief mod_temp_CubeMX_LM75__write_reg
 */
static int mod_temp_CubeMX_LM75__write_reg(enum mod_temp_CubeMX_LM75_reg_selector _sel, union mod_temp_CubeMX_LM75__reg_attr *_reg)
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
    HAL_I2C_Master_Transmit(&hi2c1, M_MOD_TEMP_CUBEMX_LM75__ADDR, (void *)&temp_reg_cfg[_sel].data, temp_reg_cfg[_sel].width, 1000);

    /* copy the currently written value */
    memcpy((void *)&temp_reg_cfg[_sel].data, _reg, temp_reg_cfg[_sel].width);
    return 0;
}

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
