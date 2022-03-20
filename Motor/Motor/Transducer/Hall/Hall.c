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
    @file 	Hall.c
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Hall.h"
#include "Config.h"
#include "Peripheral/Pin/Pin.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*

 */
void Hall_Init(Hall_T * p_hall)
{
	Pin_Input_Init(&p_hall->CONFIG.PIN_A);
	Pin_Input_Init(&p_hall->CONFIG.PIN_B);
	Pin_Input_Init(&p_hall->CONFIG.PIN_C);
	p_hall->Direction = HALL_DIRECTION_CCW;
	p_hall->SensorsRef.State = 0U;

	if (p_hall->CONFIG.P_PARAMS_NVM != 0U)
	{
		memcpy(&p_hall->SensorsTable[0U], &p_hall->CONFIG.P_PARAMS_NVM->SensorsTable[0U], sizeof(Hall_Params_T));
	}
	else
	{
		Hall_MapSensorsTableDefault(p_hall);
	}
}
//
//void Hall_LoadDefault(Hall_T * p_hall)
//{
//	if (p_hall->CONFIG.P_PARAMS_DEFAULT != 0U)
//	{
//		memcpy(&p_hall->SensorsTable[0U], &p_hall->CONFIG.P_PARAMS_DEFAULT->SensorsTable[0U], sizeof(Hall_Params_T));
//	}
//	else
//	{
//		Hall_MapSensorsTableDefault(p_hall);
//	}
//}

void Hall_MapSensorsTable
(
	Hall_T * p_hall,
	uint8_t sensorsA,
	uint8_t sensorsInvC,
	uint8_t sensorsB,
	uint8_t sensorsInvA,
	uint8_t sensorsC,
	uint8_t sensorsInvB
)
{
	p_hall->SensorsTable[sensorsA] 		= HALL_VIRTUAL_SENSORS_A;
	p_hall->SensorsTable[sensorsInvC] 	= HALL_VIRTUAL_SENSORS_INV_C;
	p_hall->SensorsTable[sensorsB] 		= HALL_VIRTUAL_SENSORS_B;
	p_hall->SensorsTable[sensorsInvA] 	= HALL_VIRTUAL_SENSORS_INV_A;
	p_hall->SensorsTable[sensorsC] 		= HALL_VIRTUAL_SENSORS_C;
	p_hall->SensorsTable[sensorsInvB] 	= HALL_VIRTUAL_SENSORS_INV_B;
	p_hall->SensorsTable[0U] = 0U;
	p_hall->SensorsTable[7U] = 7U;
}

void Hall_MapSensorsTableDefault(Hall_T * p_hall)
{
	Hall_MapSensorsTable
	(
		p_hall,
		HALL_VIRTUAL_SENSORS_A,
		HALL_VIRTUAL_SENSORS_INV_C,
		HALL_VIRTUAL_SENSORS_B,
		HALL_VIRTUAL_SENSORS_INV_A,
		HALL_VIRTUAL_SENSORS_C,
		HALL_VIRTUAL_SENSORS_INV_B
	);
}

/*
 * For case where sensors are aligned to motor phase, 0 degree offset.
 * i.e Motor Phase A measure in between a hall state and not on a transition boundary.
 */
void Hall_CalibratePhaseA(Hall_T * p_hall)
{
	p_hall->SensorsTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_A;
}

void Hall_CalibratePhaseInvC(Hall_T * p_hall)
{
	p_hall->SensorsTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_INV_C;
}

void Hall_CalibratePhaseB(Hall_T * p_hall)
{
	p_hall->SensorsTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_B;
}

void Hall_CalibratePhaseInvA(Hall_T * p_hall)
{
	p_hall->SensorsTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_INV_A;
}

void Hall_CalibratePhaseC(Hall_T * p_hall)
{
	p_hall->SensorsTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_C;
}

void Hall_CalibratePhaseInvB(Hall_T * p_hall)
{
	p_hall->SensorsTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_INV_B;
}

