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

#if     defined(CONFIG_RING_LENGTH_POW2)    /* Power 2 length only. */
    #define CONFIG_RING_POW2_MASK
#elif   defined(CONFIG_RING_POW2_MASK)      /* Power 2 length only. Mask on access */
#elif   defined(CONFIG_RING_POW2_WRAP)      /* Power 2 length only. Mask and update index */
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
#else
    #define CONFIG_RING_LENGTH_COMPARE
#endif

#if     defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
#elif   defined(CONFIG_RING_LOCAL_CRITICAL_DISABLE) /* Disable Critical at Ring Buffer 'class' level */
#else
    #define CONFIG_RING_LOCAL_CRITICAL_DISABLE
#endif

#endif




