/**************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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
    @file 	Default.h
    @author FireSoucery
    @brief  Motor default fallback parameters
    @version V0
*/
/**************************************************************************/
#ifndef DEFAULT_MOTOR_H
#define DEFAULT_MOTOR_H

/*
 * Default value. If not load from flash. Run time adjustable
 */



#define DEFAULT_CONTROL_FREQ_HZ 20000
#define DEFAULT_ENCODER_DISTANCE_PER_COUNT 1
#define DEFAULT_ENCODER_COUNTS_PER_REVOLUTION 8192
#define DEFAULT_POLE_PAIRS 7


#define DEFAULT_FOC_OPEN_LOOP_VQ 3276 /* 10 percent */

#define DEFAULT_FOC_KP_FACTOR 	148 //qangle16, milliseconds
#define DEFAULT_FOC_KP_DIVISOR 	1

#define DEFAULT_FOC_KI_FACTOR 	1 //qangle16, milliseconds
#define DEFAULT_FOC_KI_DIVISOR 	4

#endif
