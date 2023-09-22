/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Pin.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Pin.h"

void Pin_Output_Init(Pin_T * p_pin)
{
    HAL_Pin_InitOutput(p_pin->CONFIG.P_HAL_PIN, _Pin_GetHalArg(p_pin));
    HAL_Pin_WriteOutputOff(p_pin->CONFIG.P_HAL_PIN, _Pin_GetHalArg(p_pin));
}

void Pin_Input_Init(Pin_T * p_pin)
{
    HAL_Pin_InitInput(p_pin->CONFIG.P_HAL_PIN, _Pin_GetHalArg(p_pin));
}

void Pin_Deinit(const Pin_T * p_pin)
{
    HAL_Pin_Deinit(p_pin->CONFIG.P_HAL_PIN, _Pin_GetHalArg(p_pin));
}
