/***************************************************************************//**
 * mod_temp_CubeMX_LM75_cfg.h
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

#ifndef __MOD_TEMP_CUBEMX_LM75_CFG__H__
#define __MOD_TEMP_CUBEMX_LM75_CFG__H__

////////////////////////////////////////////////////////////////////////////////
/// Includes
////////////////////////////////////////////////////////////////////////////////

/* c - runtime */
#include <stdint.h>
#include <stddef.h>

/* system */

/* own libs */

/* module (this project) */


#ifdef __cplusplus
#extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////

/* I2C to use for communications with LM75 */

#define I2C_PORT         I2C1
#define I2C_SCL_PIN      GPIO_PIN_8     // PB6
#define I2C_SDA_PIN      GPIO_PIN_9    // PB7
#define I2C_GPIO_PORT    GPIOB
#define I2C_CLOCK        RCC_APB1Periph_I2C1


/* LM75 defines */

#define M_MOD_TEMP_CUBEMX_LM75__ADDR    0x90

////////////////////////////////////////////////////////////////////////////////
/// Type definitions, structures and unions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// (global) function declaration
////////////////////////////////////////////////////////////////////////////////


#ifdef __cplusplus
}
#endif

#endif /* __MOD_TEMP_CUBEMX_LM75_CFG__H__ */
