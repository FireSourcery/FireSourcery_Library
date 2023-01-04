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
	@file 	.h
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef CONFIG_PID_H
#define CONFIG_PID_H

#if 	defined(CONFIG_PID_K_INT)
#elif 	defined(CONFIG_PID_K_FLOAT)
#else
	#define CONFIG_PID_K_INT
#endif

#if 	defined(CONFIG_PID_DIVIDE_SHIFT)
#elif 	defined(CONFIG_PID_DIVIDE_NUMERICAL)
#else
	#define CONFIG_PID_DIVIDE_SHIFT
#endif


#endif
