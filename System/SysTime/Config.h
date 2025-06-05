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
#ifndef CONFIG_SYSTIME_H
#define CONFIG_SYSTIME_H

#if defined(CPU_FREQ)
#else
    #error "SysTime - Undefined CPU_FREQ"
#endif

#ifdef CONFIG_SYSTIME_SYSTICK
#endif

// #ifdef CONFIG_SYSTIME_SYSTICK_PRIORITY
// #else
//     #define CONFIG_SYSTIME_SYSTICK_PRIORITY 0x80U /* Priority 8 */
// #endif

#endif

