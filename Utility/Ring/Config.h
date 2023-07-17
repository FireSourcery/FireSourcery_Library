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
    @version V0
*/
/******************************************************************************/
#ifndef CONFIG_RING_H
#define CONFIG_RING_H

#if     defined(CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED)    /* Power 2 length only. Integer overflow wrap only */
#elif   defined(CONFIG_RING_LENGTH_POW2_INDEX_WRAPPED)      /* Power 2 length only. Does not need integer overflow wrap */
#elif   defined(CONFIG_RING_LENGTH_ANY)
#else
    #define CONFIG_RING_LENGTH_ANY
#endif

#if     defined(CONFIG_RING_MULTITHREADED_ENABLE)
#elif   defined(CONFIG_RING_MULTITHREADED_DISABLE) || defined(CONFIG_RING_SINGLE_THREADED)
#else
    #define CONFIG_RING_MULTITHREADED_DISABLE
    #define CONFIG_RING_SINGLE_THREADED
#endif

#endif




