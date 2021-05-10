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
 * Compile-time sensor order from HAL
 */
#if defined(CONFIG_HALL_SENSORS_ORDER_CBA)
	#define	HALL_SENSORS_A (0b001)
	#define	HALL_SENSORS_B (0b010)
	#define	HALL_SENSORS_C (0b100)
	#define	HALL_SENSORS_NOT_A (0b110)
	#define	HALL_SENSORS_NOT_B (0b101)
	#define	HALL_SENSORS_NOT_C (0b011)

	typedef struct
	{
		union {
			struct {
				uint8_t A : 1;
				uint8_t B : 1;
				uint8_t C : 1;
				uint8_t Resv3 : 1;
				uint8_t Resv4 : 1;
				uint8_t Resv5 : 1;
				uint8_t Resv6 : 1;
				uint8_t Resv7 : 1;
			} Bits;
			uint8_t Byte;
		};
	} Hall_Sensors_T;
#elif (CONFIG_HALL_HALL_SENSORS_ORDER_ABC)
	#define	HALL_SENSORS_A (0b100)
	#define	HALL_SENSORS_B (0b010)
	#define	HALL_SENSORS_C (0b001)
	#define	HALL_SENSORS_NOT_A (0b011)
	#define	HALL_SENSORS_NOT_B (0b101)
	#define	HALL_SENSORS_NOT_C (0b110)

	typedef struct
	{
		union {
			struct {
				uint8_t C : 1;
				uint8_t B : 1;
				uint8_t A : 1;
				uint8_t Resv3 : 1;
				uint8_t Resv4 : 1;
				uint8_t Resv5 : 1;
				uint8_t Resv6 : 1;
				uint8_t Resv7 : 1;
			} Bits;
			uint8_t Byte;
		};
	} Hall_Sensors_T;
#endif


/*

 */
#if defined(CONFIG_HALL_COMMUTATION_TABLE_SECTOR_ID)
typedef enum
{
	/* commutation direction */
	HALL_PHASE_DISABLE = 0,
	HALL_PHASE_AC = 1,
	HALL_PHASE_BC = 2,
	HALL_PHASE_BA = 3,
	HALL_PHASE_CA = 4,
	HALL_PHASE_CB = 5,
	HALL_PHASE_AB = 6,

	/* commutation direction */
	HALL_SECTOR_0 = 0,
	HALL_SECTOR_1 = 1,
	HALL_SECTOR_2 = 2,
	HALL_SECTOR_3 = 3,
	HALL_SECTOR_4 = 4,
	HALL_SECTOR_5 = 5,
	HALL_SECTOR_6 = 6,
	HALL_SECTOR_7 = 7,

	/* rotator position via boundary from CCW and CW*/
	/*
	 * Calibrated for CCW
	 */
	HALL_ANGLE_CCW_30 = 3, 		//HALL_PHASE_BA,
	HALL_ANGLE_CCW_90 = 4, 		//HALL_PHASE_CA,
	HALL_ANGLE_CCW_150 = 5, 	//HALL_PHASE_CB,
	HALL_ANGLE_CCW_210 = 6, 	//HALL_PHASE_AB,
	HALL_ANGLE_CCW_270 = 1, 	//HALL_PHASE_AC,
	HALL_ANGLE_CCW_330 = 2, 	//HALL_PHASE_BC,

	HALL_ANGLE_CW_30 = 5, 	//HALL_PHASE_CB,
	HALL_ANGLE_CW_90 = 6, 	//HALL_PHASE_AB,
	HALL_ANGLE_CW_150 = 1, 	//HALL_PHASE_AC,
	HALL_ANGLE_CW_210 = 2, 	//HALL_PHASE_BC,
	HALL_ANGLE_CW_270 = 3, 	//HALL_PHASE_BA,
	HALL_ANGLE_CW_330 = 4, 	//HALL_PHASE_CA,
} Hall_Id_T;
#elif 	defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
//typedef void * Hall_Id_T;
typedef void (* Hall_Id_T)(void * p_userData);
#endif



typedef struct 
{
#ifdef CONFIG_HALL_HAL_PIN
	const HAL_Pin_T * p_HAL_PinA;
	const HAL_Pin_T * p_HAL_PinB;
	const HAL_Pin_T * p_HAL_PinC;
#elif defined(CONFIG_HALL_HAL_HALL)
	/*
	 * user provides function to return hall sensors in format defined by CONFIG_HALL_SENSOR_ORDER_CBA or CONFIG_HALL_SENSOR_ORDER_ABC
	 * user accounts for endianess
	 */
	const HAL_Hall_T * p_HAL_Hall;
#endif

	Hall_Id_T CommuntationTable[8];
	volatile Hall_Direction_T Direction;
	volatile Hall_Sensors_T SensorsSaved;	/* Save last read */
	volatile Hall_Id_T IdSaved; /* Save last read */

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
	void * p_UserData;
#endif
}
Hall_T;

static inline uint8_t InverseHall(uint8_t hall)
{
	return (~hall) & 0x07;
}

static inline Hall_Sensors_T Hall_ReadSensors(const Hall_T * p_hall)
{
	Hall_Sensors_T sensors;

#ifdef CONFIG_HALL_HAL_HALL
	sensors.Byte = HAL_Hall_ReadSensors(p_hall->p_HAL_Hall);
#elif defined(CONFIG_HALL_HAL_PIN)
	sensors.Bits.A = HAL_Pin_ReadState(p_hall->p_HAL_PinA);
	sensors.Bits.B = HAL_Pin_ReadState(p_hall->p_HAL_PinB);
	sensors.Bits.C = HAL_Pin_ReadState(p_hall->p_HAL_PinC);
#endif

	return sensors;
}

/*
 * ISR version
 */
static inline uint8_t Hall_CaptureSector_IO(Hall_T * p_hall)
{
	/*
	 * Calibrated for CCW
	 */
	if (p_hall->Direction == HALL_DIRECTION_CW)
	{
		p_hall->IdSaved = p_hall->CommuntationTable[InverseHall(Hall_ReadSensors(p_hall).Byte)];
	}
	else
	{
		p_hall->IdSaved = p_hall->CommuntationTable[Hall_ReadSensors(p_hall).Byte];
	}

	return p_hall->IdSaved;
}

/*
 * Polling
 */
static inline bool Hall_PollSector_IO(Hall_T * p_hall)
{
	bool isNewPhase;

	if (p_hall->SensorsSaved.Byte != Hall_ReadSensors(p_hall).Byte)
	{
		p_hall->SensorsSaved = Hall_ReadSensors(p_hall);
		Hall_CaptureSector_IO(p_hall);
		isNewPhase = true;
	}
	else
	{
		isNewPhase = false;
	}

	return (isNewPhase);
}

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
	static inline void Hall_Commutate_IO(Hall_T * p_hall)
	{
		if (p_hall->Direction == HALL_DIRECTION_CCW)
		{
			p_hall->CommuntationTable[InverseHall(Hall_ReadSensors(p_hall))](p_hall->p_UserData);
		}
		else
		{
			p_hall->CommuntationTable[Hall_ReadSensors(p_hall)](p_hall->p_UserData);
		}
	}

	static inline bool Hall_PollCommutate_IO(Hall_T * p_hall)
	{
		bool isNewPhase;
		//	XOR bits to div 6 for
		if (p_hall->SensorsSaved != Hall_ReadSensors(p_hall))
		{
			p_hall->SensorsSaved = Hall_ReadSensors(p_hall);
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

static inline Hall_Id_T Hall_GetSectorId(Hall_T * p_hall)
{
	return p_hall->IdSaved;
}

static inline uint16_t Hall_GetRotorAngle_Degrees16(Hall_T * p_hall)
{
	uint16_t degrees;

	if (p_hall->Direction == HALL_DIRECTION_CCW)
	{
		switch (p_hall->IdSaved)
		{
			case HALL_ANGLE_CCW_270: degrees = 270; break;
			case HALL_ANGLE_CCW_330: degrees = 330; break;
			case HALL_ANGLE_CCW_30:	degrees = 30; break;
			case HALL_ANGLE_CCW_90:	degrees = 90; break;
			case HALL_ANGLE_CCW_150: degrees = 150; break;
			case HALL_ANGLE_CCW_210: degrees = 210; break;
			default: break;
		}
	}
	else
	{
		switch (p_hall->IdSaved)
		{
			case HALL_ANGLE_CW_30:	degrees = 30; break;
			case HALL_ANGLE_CW_90:	degrees = 90; break;
			case HALL_ANGLE_CW_150:	degrees = 150; break;
			case HALL_ANGLE_CW_210:	degrees = 210; break;
			case HALL_ANGLE_CW_270:	degrees = 270; break;
			case HALL_ANGLE_CW_330:	degrees = 330; break;
			default: break;
		}
	}

	return degrees;
}

/*
	Can pass 0 for phase and sensor ids, and calibrate after
 */
void Hall_Init
(
	Hall_T * p_hall,
	const HAL_Hall_T * p_hal_Hall,
	Hall_Id_T phaseAC,
	Hall_Id_T phaseBC,
	Hall_Id_T phaseBA,
	Hall_Id_T phaseCA,
	Hall_Id_T phaseCB,
	Hall_Id_T phaseAB,
	uint8_t sensorIndexPhaseAC,
	uint8_t sensorIndexPhaseBC,
	uint8_t sensorIndexPhaseBA,
	uint8_t sensorIndexPhaseCA,
	uint8_t sensorIndexPhaseCB,
	uint8_t sensorIndexPhaseAB,
	Hall_Id_T fault000,
	Hall_Id_T fault111
);

void Hall_MapCommuntationTable
(
	Hall_T * p_hall,
	Hall_Id_T phaseAC,
	Hall_Id_T phaseBC,
	Hall_Id_T phaseBA,
	Hall_Id_T phaseCA,
	Hall_Id_T phaseCB,
	Hall_Id_T phaseAB,
	uint8_t sensorIndexPhaseAC,
	uint8_t sensorIndexPhaseBC,
	uint8_t sensorIndexPhaseBA,
	uint8_t sensorIndexPhaseCA,
	uint8_t sensorIndexPhaseCB,
	uint8_t sensorIndexPhaseAB
);

void Hall_MapCommuntationTable_Default
(
	Hall_T * p_hall
);

void Hall_MapCommuntationTableFaultStates
(
	Hall_T * p_hall,
	Hall_Id_T fault000,
	Hall_Id_T fault111
);

/*
 * calibrate phase to be next active phase when hall state is detected
 * non blocking implement outside module
 */
void Hall_CalibrateCommuntationTable_Blocking
(
	Hall_T * p_hall,
	Hall_Id_T phaseAC,
	Hall_Id_T phaseBC,
	Hall_Id_T phaseBA,
	Hall_Id_T phaseCA,
	Hall_Id_T phaseCB,
	Hall_Id_T phaseAB,
	void (* activatePwmValuePhaseABC)(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
	uint16_t pwm,
	void (*activatePwmStatePhaseABC)(bool enA, bool enB, bool enC),
	void (*delay)(uint32_t),
	uint32_t delayTime
);

#endif
