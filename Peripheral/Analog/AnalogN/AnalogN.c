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
    @file     AnalogN.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "AnalogN.h"

void AnalogN_Init(const AnalogN_T * p_analogn)
{
    for(uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
    {
        Analog_Init(&p_analogn->CONFIG.P_ANALOGS[iAdc]);
    }
}


