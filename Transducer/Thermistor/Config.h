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
    @brief

*/
/******************************************************************************/
#ifndef CONFIG_THERMISTOR_H
#define CONFIG_THERMISTOR_H

#if     defined(CONFIG_THERMISTOR_UNITS_LINEAR)
#elif   defined(CONFIG_THERMISTOR_UNITS_FLOAT)
#elif   defined(CONFIG_THERMISTOR_UNITS_LUT)
#else
    #define CONFIG_THERMISTOR_UNITS_LINEAR
#endif

#endif

