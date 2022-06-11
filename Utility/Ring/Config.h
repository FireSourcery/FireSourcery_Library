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
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef CONFIG_RING_H
#define CONFIG_RING_H

#if 	defined(CONFIG_RING_CRITICAL_LIBRARY_DEFINED)
#elif 	defined(CONFIG_RING_CRITICAL_EXTERN_DEFINED)
#elif 	defined(CONFIG_RING_SINGLE_THREADED)
#else
	#define CONFIG_RING_SINGLE_THREADED
#endif

#if 	defined(CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED) 	/* Power 2 length only, interger overflow wrap only */
#elif  	defined(CONFIG_RING_LENGTH_POW2_INDEX_WRAPPED)		/* Power 2 length only. Does not need interger overflow wrap */
#elif  	defined(CONFIG_RING_LENGTH_ANY)
#else
	#define CONFIG_RING_LENGTH_ANY
#endif

#endif




