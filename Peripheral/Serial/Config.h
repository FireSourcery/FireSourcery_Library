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
#ifndef CONFIG_SERIAL_H
#define CONFIG_SERIAL_H

#if       defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
#elif     defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
#elif     defined(CONFIG_SERIAL_SINGLE_THREADED)
#else
    #define CONFIG_SERIAL_SINGLE_THREADED
#endif

#if     defined(CONFIG_SERIAL_HW_FIFO_DISABLE)
#elif   defined(CONFIG_SERIAL_HW_FIFO_ENABLE)
#else
    #define CONFIG_SERIAL_HW_FIFO_DISABLE
#endif

#endif
