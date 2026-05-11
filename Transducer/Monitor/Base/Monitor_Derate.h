#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Monitor_Derate.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/

// static inline ufract16_t Monitor_WarningDerateScale(const Monitor_Config_T * p_config, int32_t input)
// {
//     return linear_map_sat(p_config->Warning.Setpoint, p_config->Fault.Setpoint, FRACT16_MAX, 0, input);
// }