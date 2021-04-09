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
    @file 	Hall.h
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#ifndef HALL_H
#define HALL_H

#include "Config.h"

#include "HAL.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	HALL_DIRECTION_CW,
	HALL_DIRECTION_CCW,
} Hall_Direction_T;

/*
 * Sector reflector sector voltage is to be applied (next step from active sensor)
 */
#if defined(CONFIG_HALL_COMMUTATION_TABLE_SECTOR_ID)
typedef enum
{
	HALL_PHASE_DISABLE = 0,
	HALL_PHASE_AC = 1,
	HALL_PHASE_BC = 2,
	HALL_PHASE_BA = 3,
	HALL_PHASE_CA = 4,
	HALL_PHASE_CB = 5,
	HALL_PHASE_AB = 6,
//	HALL_PHASE_DISABLE = 7,

	HALL_SECTOR_0 = 0,
	HALL_SECTOR_1 = 1,
	HALL_SECTOR_2 = 2,
	HALL_SECTOR_3 = 3,
	HALL_SECTOR_4 = 4,
	HALL_SECTOR_5 = 5,
	HALL_SECTOR_6 = 6,
	HALL_SECTOR_7 = 7,
} Hall_Phase_T;
#elif 	defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
//typedef void * Hall_Phase_T;
typedef void (* Hall_Phase_T)(void * p_userData);
#endif

//typedef struct
//{
//	union {
//		struct {
//			uint8_t A : 1;
//			uint8_t B : 1;
//			uint8_t C : 1;
//			uint8_t Resv3 : 1;
//			uint8_t Resv4 : 1;
//			uint8_t Resv5 : 1;
//			uint8_t Resv6 : 1;
//			uint8_t Resv7 : 1;
//        } Bits;
//		uint8_t Byte;
//	};
//} HALL_Sensor_T;

typedef struct 
{
	/*
	 * user provides function to return hall sensors in 0bxxxxxcba format, user accounts for endianess
	 */
	const HAL_Hall_T * p_HAL_Hall;
	Hall_Phase_T CommuntationTable[8];
	volatile Hall_Direction_T Direction;
	volatile uint8_t SensorsSaved;	/* Save last read */
	volatile Hall_Phase_T PhaseSaved; /* Save last read */

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
	void * p_UserData;
#endif
}
Hall_T;

static inline uint8_t InverseHall(uint8_t hall) {return (~hall)&0x07;}

/*
 * ISR version
 */
static inline uint8_t Hall_CapturePhase_IO(Hall_T * p_hall)
{
	if (p_hall->Direction == HALL_DIRECTION_CCW)
	{
		p_hall->PhaseSaved = p_hall->CommuntationTable[InverseHall(HAL_Hall_ReadSensors(p_hall->p_HAL_Hall))];
	}
	else
	{
		p_hall->PhaseSaved = p_hall->CommuntationTable[HAL_Hall_ReadSensors(p_hall->p_HAL_Hall)];
	}

	return p_hall->PhaseSaved;
}

/*
 * Polling
 */
static inline bool Hall_PollPhase_IO(Hall_T * p_hall)
{
	bool isNewPhase;

	if (p_hall->SensorsSaved != HAL_Hall_ReadSensors(p_hall->p_HAL_Hall))
	{
		p_hall->SensorsSaved = HAL_Hall_ReadSensors(p_hall->p_HAL_Hall);
		Hall_CapturePhase_IO(p_hall);
		isNewPhase = true;
	}
	else
	{
		isNewPhase = false;
	}

	return (isNewPhase);
}

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
	static inline void Hall_Commutate(Hall_T * p_hall)
	{
		if (p_hall->Direction == HALL_DIRECTION_CCW)
		{
			p_hall->CommuntationTable[InverseHall(HAL_Hall_ReadSensors(p_hall->p_HAL_Hall))]();
		}
		else
		{
			p_hall->CommuntationTable[HAL_Hall_ReadSensors(p_hall->p_HAL_Hall)]();
		}
	}

	static inline bool Hall_PollCommutate(Hall_T * p_hall)
	{

		bool isNewPhase;
		//	XOR bits to div 6 for
		if (p_hall->SensorsSaved != HAL_Hall_ReadSensors(p_hall->p_HAL_Hall))
		{
			p_hall->SensorsSaved = HAL_Hall_ReadSensors(p_hall->p_HAL_Hall);
			Hall_Commutate(p_hall);
			isNewPhase = true;
		}
		else
		{
			isNewPhase = false;
		}

		return (isNewPhase);
	}
#endif

static inline void Hall_SetDirection(Hall_T * p_hall, Hall_Direction_T dir)
{
	p_hall->Direction = dir;
}

static inline void Hall_ToggleDirection(Hall_T * p_hall)
{
	p_hall->Direction = ~p_hall->Direction;
}

static inline Hall_Direction_T Hall_GetDirection(Hall_T * p_hall)
{
	return (p_hall->Direction);
}

/*
	Can use 0 for phase and sensor ids, and calibrate after
 */
void Hall_Init
(
	Hall_T * p_hall,
	const HAL_Hall_T * p_hal_Hall,
	Hall_Phase_T phaseAC,
	Hall_Phase_T phaseBC,
	Hall_Phase_T phaseBA,
	Hall_Phase_T phaseCA,
	Hall_Phase_T phaseCB,
	Hall_Phase_T phaseAB,
	uint8_t sensorPhaseAC,
	uint8_t sensorPhaseBC,
	uint8_t sensorPhaseBA,
	uint8_t sensorPhaseCA,
	uint8_t sensorPhaseCB,
	uint8_t sensorPhaseAB,
	Hall_Phase_T fault000,
	Hall_Phase_T fault111
);

void Hall_MapCommuntationTable
(
	Hall_T * p_hall,
	Hall_Phase_T phaseAC,
	Hall_Phase_T phaseBC,
	Hall_Phase_T phaseBA,
	Hall_Phase_T phaseCA,
	Hall_Phase_T phaseCB,
	Hall_Phase_T phaseAB,
	uint8_t sensorPhaseAC,
	uint8_t sensorPhaseBC,
	uint8_t sensorPhaseBA,
	uint8_t sensorPhaseCA,
	uint8_t sensorPhaseCB,
	uint8_t sensorPhaseAB
);

void Hall_MapCommuntationTable_Default
(
	Hall_T * p_hall,
	Hall_Phase_T phaseAC,
	Hall_Phase_T phaseBC,
	Hall_Phase_T phaseBA,
	Hall_Phase_T phaseCA,
	Hall_Phase_T phaseCB,
	Hall_Phase_T phaseAB
);

void Hall_MapCommuntationTableFaultStates
(
	Hall_T * p_hall,
	Hall_Phase_T fault000,
	Hall_Phase_T fault111
);

/*
 * calibrate phase to be next active phase when hall state is detected
 * non blocking implement outside module
 */
void Hall_CalibrateCommuntationTable_Blocking
(
	Hall_T * p_hall,

//	void (* phaseAC)(void * userData),
//	void (* phaseBC)(void * userData),
//	void (* phaseBA)(void * userData),
//	void (* phaseCA)(void * userData),
//	void (* phaseCB)(void * userData),
//	void (* phaseAB)(void * userData),

	Hall_Phase_T phaseAC,
	Hall_Phase_T phaseBC,
	Hall_Phase_T phaseBA,
	Hall_Phase_T phaseCA,
	Hall_Phase_T phaseCB,
	Hall_Phase_T phaseAB,

	void (* activatePwmPhaseABC)(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
	uint16_t pwm,
	void (*activatePwmStatePhaseABC)(bool enA, bool enB, bool enC),

	void (*delay)(uint32_t),
	uint32_t delayTime
);

#endif
