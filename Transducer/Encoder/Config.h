/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@file 	Config.h
	@author FireSourcery
	@brief 	Encoder module preprocessor configuration options and defaults.
	@version V0
*/
/******************************************************************************/
#ifndef CONFIG_ENCODER_H
#define CONFIG_ENCODER_H

/*
	HW_TIMER_ONLY
	HW_TIMER_COUNTER
	HW_DECODER
	Compile time define if chip supports quadrature decoder capture. Enables toggle during runtime
*/
#if 	defined(CONFIG_ENCODER_HW_DECODER)
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
#else
	// #define CONFIG_ENCODER_HW_DECODER
#endif

/* Compile time define for all encoder instances if A Lead B is increment, additional configure available at runtime */
#if 	defined(CONFIG_ENCODER_HW_DECODER_A_LEAD_B_INCREMENT)
#elif 	defined(CONFIG_ENCODER_HW_DECODER_A_LEAD_B_DECREMENT)
#else
	#define CONFIG_ENCODER_HW_DECODER_A_LEAD_B_INCREMENT
#endif

/*  Capture DeltaT Mode */
#if 	defined(CONFIG_ENCODER_HW_TIMER_COUNTER_MAX)
#else
		#define CONFIG_ENCODER_HW_TIMER_COUNTER_MAX 0xFFFFU
#endif

#if 	defined(CONFIG_ENCODER_ANGLE_DEGREES_BITS)
#else
	#define CONFIG_ENCODER_ANGLE_DEGREES_BITS 16U
#endif

#endif
