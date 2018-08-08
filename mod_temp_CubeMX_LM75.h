/***************************************************************************//**
 * mod_temp_CubeMX_LM75.h
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

#ifndef __MOD_TEMP_CUBEMX_LM75_H__
#define __MOD_TEMP_CUBEMX_LM75_H__

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


#ifdef __cplusplus
#extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// Type definitions, structures and unions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// (global) function declaration
////////////////////////////////////////////////////////////////////////////////


int dev_temperature_LM75__init(uint32_t _i2c_clk_speed);


#ifdef __cplusplus
}
#endif

#endif /* __MOD_TEMP_CUBEMX_LM75_H__ */
