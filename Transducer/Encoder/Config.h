/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
	@author FireSoucery
	@brief 	Encoder module preprocessor configuration options and defaults.
	@version V0
*/
/******************************************************************************/
#ifndef CONFIG_ENCODER_H
#define CONFIG_ENCODER_H

/*
 * Compile time define if chip supports quadrature capture. Enables toggle during runtime
 */
#ifdef CONFIG_ENCODER_HW_QUADRATURE_CAPABLE

#elif defined(CONFIG_ENCODER_HW_QUADRATURE_DISABLED)

#else
	#define CONFIG_ENCODER_HW_QUADRATURE_CAPABLE
#endif

/*
 * Compile time define for all encoder instances if A Lead B is increment
 */
#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT

#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)

#else
	#define CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
#endif


#ifdef CONFIG_ENCODER_ANGLE_RESOLUTION_BITS

#else
	#define CONFIG_ENCODER_ANGLE_RESOLUTION_BITS 16U
#endif



#endif
