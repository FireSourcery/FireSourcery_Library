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
    @file   Config.h
    @author FireSourcery
    @brief  Analog module preprocessor configuration options and defaults.

*/
/******************************************************************************/
#ifndef CONFIG_ANALOG_H
#define CONFIG_ANALOG_H

// #if     defined(ANALOG_ADC_HW_FIFO_DISABLE)
// #elif   defined(ANALOG_ADC_HW_FIFO_ENABLE)
// #else
//     #define ANALOG_ADC_HW_FIFO_DISABLE
// #endif

#if     defined(ANALOG_ADC_HW_CONTINUOUS_CONVERSION_ENABLE)
#elif   defined(ANALOG_ADC_HW_CONTINUOUS_CONVERSION_DISABLE)
#else
    #define ANALOG_ADC_HW_CONTINUOUS_CONVERSION_DISABLE
#endif

#endif

