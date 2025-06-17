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
    @brief     Encoder module preprocessor configuration options and defaults.

*/
/******************************************************************************/
#ifndef CONFIG_ENCODER_H
#define CONFIG_ENCODER_H

/*
    Compile time define if chip supports decoder/counter.
*/
#if     defined(CONFIG_ENCODER_HW_DECODER)
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
#else
    // #define CONFIG_ENCODER_HW_DECODER
#endif

/* Compile time define for all encoder instances if A Lead B is increment, additional configure available at runtime */
#if     defined(CONFIG_ENCODER_HW_DECODER_A_LEAD_B_INCREMENT)
#elif   defined(CONFIG_ENCODER_HW_DECODER_A_LEAD_B_DECREMENT)
#else
    #define CONFIG_ENCODER_HW_DECODER_A_LEAD_B_INCREMENT
#endif

/* Emulated and Decoder Quadrature Capture. Enables toggle during runtime */
#if     defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE) /* Emulated and Decoder */
#elif   defined(CONFIG_ENCODER_QUADRATURE_MODE_DISABLE)
#else
    #define CONFIG_ENCODER_QUADRATURE_MODE_ENABLE
#endif

/* Adjust timer freq at runtime */
#ifdef CONFIG_ENCODER_DYNAMIC_TIMER
#else
#endif

#endif
