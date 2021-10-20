/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	Config.h
    @author FireSoucery
    @brief	Hall module
    @version V0
*/
/**************************************************************************/
#ifndef CONFIG_HALL_H
#define CONFIG_HALL_H

/*
	Match in HAL
 */
#if (CONFIG_HALL_HW_SENSOR_ORDER_CBA)

#elif (CONFIG_HALL_HW_SENSOR_ORDER_ABC)

#else
	#define CONFIG_HALL_HW_SENSOR_ORDER_CBA
#endif


/*
	Module Options
 */
#if  	defined(CONFIG_HALL_COMMUTATION_TABLE_SECTOR_ID)

#elif 	defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)

#else
	#define CONFIG_HALL_COMMUTATION_TABLE_SECTOR_ID
#endif


#if  	defined(CONFIG_HALL_HAL_PIN)

#elif 	defined(CONFIG_HALL_HAL_HALL)

#else
	#define CONFIG_HALL_HAL_HALL
#endif

#endif
