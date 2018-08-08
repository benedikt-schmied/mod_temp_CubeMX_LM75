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

/* own libs */

/* module (this project) */


#ifdef __cplusplus
#extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////

/**
 * macro, which help to choose the desired mode
 */
#define M_MOD_TEMP_CUBEMX_LM75__GOTO_SLEEP      (0U)
#define M_MOD_TEMP_CUBEMX_LM75__GOTO_RUN        (M_MOD_TEMP_CUBEMX_LM75__GOTO_SLEEP + 1U)

////////////////////////////////////////////////////////////////////////////////
/// Type definitions, structures and unions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// (global) function declaration
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief mod_temp_CubeMX_LM75__init
 *
 * @param   _i2c_clk_speed      I2C clock speed
 *
 * @return 0 if successful, > 0 if error
 *          * '-1', if parameter is invalid
 */
int mod_temp_CubeMX_LM75__init(uint32_t _i2c_clk_speed);


/**
 * @brief read the current temperature
 *
 * @remark blocking function
 *
 * @param [out]     *_temp      reference onto variable which will receive
 *                              the temperature
 *
 * @return 0 if successful, > 0 if error
 *          * '-1', if parameter is invalid
 */
int mod_temp_CubeMX_LM75__get_temperatue(int32_t *_temp);

/**
 * mod_temp_CubeMX_LM75__get_temperatue
 *
 * @param   _goto       desired mode
 *                      use the M_MOD_TEMP_CUBEMX_LM75__GOTO_(..) macro
 *
 * @return 0 if successful, > 0 if error
 *         * '-1', if parameter is invalid
 */
int mod_temp_CubeMX_LM75__define_mode(unsigned _goto);

#ifdef __cplusplus
}
#endif

#endif /* __MOD_TEMP_CUBEMX_LM75_H__ */
