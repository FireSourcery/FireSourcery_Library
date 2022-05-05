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
	@brief  Critical module preprocessor configuration options and defaults
	@version V0
*/
/******************************************************************************/
#ifndef CONFIG_CRITICAL_H
#define CONFIG_CRITICAL_H

#if 	defined(CONFIG_CRITICAL_MCU_ARM) 
#elif 	defined(CONFIG_CRITICAL_USER_DEFINED) 
#elif 	defined(CONFIG_CRITICAL_DISABLED) 
#else
	#define CONFIG_CRITICAL_DISABLED
#endif

#endif
