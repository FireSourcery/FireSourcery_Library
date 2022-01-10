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
	@brief 	State machine module preprocessor configuration options and defaults.
	@version V0
*/
/******************************************************************************/
#ifndef CONFIG_STATE_MACHINE_H
#define CONFIG_STATE_MACHINE_H

#ifdef CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED

#elif defined(CONFIG_STATE_MACHINE_CRITICAL_USER_DEFINED)
/*
 * User provide functions
 * void Critical_Enter(void);
 * void Critical_Exit(void);
 */
#elif defined(CONFIG_STATE_MACHINE_MULTITHREADED_DISABLED)
/*
 * Default configuration
 */
#else
	#define CONFIG_STATE_MACHINE_MULTITHREADED_DISABLED
#endif

#endif
