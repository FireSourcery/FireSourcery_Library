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
	#define	HALL_SENSORS_INV_A (0b110)
	#define	HALL_SENSORS_INV_B (0b101)
	#define	HALL_SENSORS_INV_C (0b011)
#elif (CONFIG_HALL_HALL_SENSORS_ORDER_ABC)
	#define	HALL_SENSORS_A (0b100)
	#define	HALL_SENSORS_B (0b010)
	#define	HALL_SENSORS_C (0b001)
	#define	HALL_SENSORS_NOT_A (0b011)
	#define	HALL_SENSORS_NOT_B (0b101)
	#define	HALL_SENSORS_NOT_C (0b110)
#endif

//Virtual hall sensor state
#define	HALL_VIRTUAL_SENSORS_A (0b001)
#define	HALL_VIRTUAL_SENSORS_B (0b010)
#define	HALL_VIRTUAL_SENSORS_C (0b100)
#define	HALL_VIRTUAL_SENSORS_INV_A (0b110)
#define	HALL_VIRTUAL_SENSORS_INV_B (0b101)
#define	HALL_VIRTUAL_SENSORS_INV_C (0b011)

//typedef union
//{
//	struct
//	{
//		uint8_t A :1;
//		uint8_t B :1;
//		uint8_t C :1;
//		uint8_t Resv3 :1;
//		uint8_t Resv4 :1;
//		uint8_t Resv5 :1;
//		uint8_t Resv6 :1;
//		uint8_t Resv7 :1;
//	};
//	uint8_t State; //State
//} Hall_Sensors_T;

//static const Hall_Sensors_T HALL_VIRTUAL_SENSORS_A = {.A = 1U, .B = 0U, .C = 0U};
//static const Hall_Sensors_T HALL_VIRTUAL_SENSORS_B = {.A = 0U, .B = 1U, .C = 0U};
//static const Hall_Sensors_T HALL_VIRTUAL_SENSORS_C = {.A = 0U, .B = 0U, .C = 1U};
//static const Hall_Sensors_T HALL_VIRTUAL_SENSORS_INV_A = {.A = 0U, .B = 1U, .C = 1U};
//static const Hall_Sensors_T HALL_VIRTUAL_SENSORS_INV_B = {.A = 1U, .B = 0U, .C = 1U};
//static const Hall_Sensors_T HALL_VIRTUAL_SENSORS_INV_C = {.A = 1U, .B = 1U, .C = 0U};


/*
hall commutation angle
Applied Voltage
 */
#if defined(CONFIG_HALL_COMMUTATION_TABLE_SECTOR_ID)
typedef enum
{

	HALL_SECTOR_0 = 0,
	HALL_SECTOR_1 = 1,
	HALL_SECTOR_2 = 2,
	HALL_SECTOR_3 = 3,
	HALL_SECTOR_4 = 4,
	HALL_SECTOR_5 = 5,
	HALL_SECTOR_6 = 6,
	HALL_SECTOR_7 = 7,

	HALL_PHASE_DISABLE = 0,
	HALL_PHASE_AC = 1,
	HALL_PHASE_BC = 2,  //sensor A
	HALL_PHASE_BA = 3,	//sensor inv C
	HALL_PHASE_CA = 4,
	HALL_PHASE_CB = 5,
	HALL_PHASE_AB = 6,
	HALL_PHASE_7 = 7,

	HALL_ANGLE_30 = 1,
	HALL_ANGLE_90 = 2,
	HALL_ANGLE_150 = 3,
	HALL_ANGLE_210 = 4,
	HALL_ANGLE_270 = 5,
	HALL_ANGLE_330 = 6,

	/*
	 * Rotor Angle
	 */
	/* rotator position Id via boundary from CCW and CW*/
//	HALL_ANGLE_CCW_30 = 3, 		//HALL_PHASE_BA,
//	HALL_ANGLE_CCW_90 = 4, 		//HALL_PHASE_CA,
//	HALL_ANGLE_CCW_150 = 5, 	//HALL_PHASE_CB,
//	HALL_ANGLE_CCW_210 = 6, 	//HALL_PHASE_AB,
//	HALL_ANGLE_CCW_270 = 1, 	//HALL_PHASE_AC,
//	HALL_ANGLE_CCW_330 = 2, 	//HALL_PHASE_BC,
//
//	HALL_ANGLE_CW_30 = 5, 	//HALL_PHASE_CB,
//	HALL_ANGLE_CW_90 = 6, 	//HALL_PHASE_AB,
//	HALL_ANGLE_CW_150 = 1, 	//HALL_PHASE_AC,
//	HALL_ANGLE_CW_210 = 2, 	//HALL_PHASE_BC,
//	HALL_ANGLE_CW_270 = 3, 	//HALL_PHASE_BA,
//	HALL_ANGLE_CW_330 = 4, 	//HALL_PHASE_CA,
} Hall_CommutationId_T;

/*
 * hall position angle
 * Rotor Angle
 */
typedef enum
{
	HALL_ANGLE_330_30 	= HALL_VIRTUAL_SENSORS_A,
	HALL_ANGLE_30_90 	= HALL_VIRTUAL_SENSORS_INV_C,
	HALL_ANGLE_90_150 	= HALL_VIRTUAL_SENSORS_B,
	HALL_ANGLE_150_210 	= HALL_VIRTUAL_SENSORS_INV_A,
	HALL_ANGLE_210_270	= HALL_VIRTUAL_SENSORS_C,
	HALL_ANGLE_270_330 	= HALL_VIRTUAL_SENSORS_INV_B,

	/* rotator position Id via boundary from CCW and CW*/
	HALL_ANGLE_CCW_30 	= HALL_ANGLE_30_90,
	HALL_ANGLE_CCW_90 	= HALL_ANGLE_90_150,
	HALL_ANGLE_CCW_150 	= HALL_ANGLE_150_210,
	HALL_ANGLE_CCW_210 	= HALL_ANGLE_210_270,
	HALL_ANGLE_CCW_270 	= HALL_ANGLE_270_330,
	HALL_ANGLE_CCW_330 	= HALL_ANGLE_330_30,

	HALL_ANGLE_CW_30 	= HALL_ANGLE_330_30,
	HALL_ANGLE_CW_90 	= HALL_ANGLE_30_90,
	HALL_ANGLE_CW_150 	= HALL_ANGLE_90_150,
	HALL_ANGLE_CW_210 	= HALL_ANGLE_150_210,
	HALL_ANGLE_CW_270 	= HALL_ANGLE_210_270,
	HALL_ANGLE_CW_330 	= HALL_ANGLE_270_330,
} Hall_PositionId_T;

#elif 	defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
//typedef void * Hall_CommutationId_T;
typedef void (* Hall_CommutationId_T)(void * p_userData);
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

	/*
	 * includes user mapped sensor virtualization map and const commutationId map
	 */
	Hall_CommutationId_T CommuntationTable[8U]; //edit n
	Hall_PositionId_T VirtualSensorTable[8U]; //  angle id table

	volatile Hall_Direction_T Direction;
	volatile uint8_t 				SensorsSaved;	/* Save last read */
	volatile Hall_PositionId_T 		VirtualSensorSaved;
	volatile Hall_CommutationId_T 	CommutationIdSaved; /* Save last read */

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
	void * p_UserData;
#endif
}
Hall_T;

static inline uint8_t InverseHall(uint8_t hall)
{
	return (~hall) & 0x07;
}

//read physical sensors
static inline volatile uint8_t Hall_ReadSensors(const Hall_T * p_hall)
{
	uint8_t sensors;

#ifdef CONFIG_HALL_HAL_HALL

	/*
	 * HAL function returns in XXXXXABC or XXXXXCBA based on CONFIG_HALL_SENSORS_ORDER_
	 */
	sensors = HAL_Hall_ReadSensors(p_hall->p_HAL_Hall);
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
static inline void Hall_CaptureSensors_IO(Hall_T * p_hall)
{
	uint8_t hall = Hall_ReadSensors(p_hall);
	p_hall->SensorsSaved = hall;
	p_hall->VirtualSensorSaved = p_hall->VirtualSensorTable[p_hall->SensorsSaved];

	/*
	 * Calibrated for CCW
	 */
//	if (p_hall->Direction == HALL_DIRECTION_CW)
//	{
//		p_hall->CommutationIdSaved = p_hall->CommuntationTable[InverseHall(hall)];
//	}
//	else
//	{
//		p_hall->CommutationIdSaved = p_hall->CommuntationTable[hall];
//	}

//	return (p_hall->CommutationIdSaved);
}

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
	static inline void Hall_Commutate_IO(Hall_T * p_hall)
	{
		if (p_hall->Direction == HALL_DIRECTION_CW)
		{
			p_hall->CommuntationTable[InverseHall(Hall_ReadSensors(p_hall))](p_hall->p_UserData);
		}
		else
		{
			p_hall->CommuntationTable[Hall_ReadSensors(p_hall)](p_hall->p_UserData);
		}
	}
#endif

/*
 * Polling
 * hall edge, angle boundary
 */
static inline bool Hall_PollEdge_IO(Hall_T * p_hall)
{
	bool isEdge;

	uint8_t hall = Hall_ReadSensors(p_hall);

	if (hall != p_hall->SensorsSaved)
	{
		p_hall->SensorsSaved = hall;
		p_hall->VirtualSensorSaved = p_hall->VirtualSensorTable[p_hall->SensorsSaved];
#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
		Hall_Commutate_IO(p_hall);
#endif

		isEdge = true;
	}
	else
	{
		isEdge = false;
	}

	return (isEdge);
}

//temp
static inline bool Hall_GetCycle(Hall_T * p_hall)
{
	bool isCycle;

	if ((Hall_ReadSensors(p_hall) & 0x01 == true) && (p_hall->SensorsSaved & 0x01 == false))
	{
		isCycle = true;
	}
	else
	{
		isCycle = false;
	}

	return (isCycle);
}

static inline Hall_CommutationId_T Hall_GetCommutationId(Hall_T * p_hall)
{
//	const Hall_CommutationId_T VIRTUAL_ID_COMMUTATION_TABLE[8] =
//	{
//		 [HALL_VIRTUAL_SENSORS_A] = HALL_SECTOR_2,
//	};
	//		p_hall->CommutationIdSaved = p_hall->VIRTUAL_ID_COMMUTATION_TABLE[ p_hall->VirtualSensorsSaved ];

	if (p_hall->Direction == HALL_DIRECTION_CW)
	{
		p_hall->CommutationIdSaved = p_hall->CommuntationTable[InverseHall(p_hall->SensorsSaved)];
	}
	else
	{
		p_hall->CommutationIdSaved = p_hall->CommuntationTable[p_hall->SensorsSaved];
	}

	return (p_hall->CommutationIdSaved);
}

//static inline Hall_CommutationId_T Hall_ReadCommutationId(Hall_T * p_hall)
//{
//	if (p_hall->Direction == HALL_DIRECTION_CW)
//	{
//		p_hall->CommutationIdSaved = p_hall->CommuntationTable[InverseHall(Hall_ReadSensors(p_hall))];
//	}
//	else
//	{
//		p_hall->CommutationIdSaved = p_hall->CommuntationTable[Hall_ReadSensors(p_hall)];
//	}
//
//	return (p_hall->CommutationIdSaved);
//}

static inline Hall_PositionId_T Hall_ConvertToVirtualSensor(Hall_T * p_hall, uint8_t physicalSensor)
{
	//need user maped tabled in either case
	return p_hall->VirtualSensorTable[physicalSensor];
}

static inline uint16_t Hall_GetRotorAngle_Degrees16(Hall_T * p_hall)
{
	uint16_t degrees;
	const uint16_t DEGREES_TABLE[] =
	{
		[HALL_ANGLE_CCW_30] 	= 5461U,
		[HALL_ANGLE_CCW_90] 	= 16384U,
		[HALL_ANGLE_CCW_150] 	= 27306U,
		[HALL_ANGLE_CCW_210] 	= 38229U,
		[HALL_ANGLE_CCW_270] 	= 49152U,
		[HALL_ANGLE_CCW_330] 	= 60074U,
	};

	if (p_hall->Direction == HALL_DIRECTION_CW)
	{
		degrees = DEGREES_TABLE[InverseHall(p_hall->VirtualSensorSaved)];
	}
	else
	{
		degrees = DEGREES_TABLE[p_hall->VirtualSensorSaved];
	}

//	switch (tempVirtualSensorSaved)
//	{
//		case HALL_ANGLE_CCW_30:	degrees = 5461U; break;
//		case HALL_ANGLE_CCW_90:	degrees = 16384U; break;
//		case HALL_ANGLE_CCW_150: degrees = 27306U; break;
//		case HALL_ANGLE_CCW_210: degrees = 38229U; break;
//		case HALL_ANGLE_CCW_270: degrees = 49152U; break;
//		case HALL_ANGLE_CCW_330: degrees = 60074U; break;
//		default: break;
//	}

	// convert back from commutation id	// commutation plus or minus 90
//	degrees = DEGREES_TABLE[p_hall->CommutationIdSaved];
//
//	if (p_hall->Direction == HALL_DIRECTION_CW)
//	{
//		degrees += 16384U;
//	}

	return degrees;
}

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
	Can pass 0 for phase and sensor ids, and calibrate after
 */
void Hall_Init
(
	Hall_T * p_hall,
	const HAL_Hall_T * p_hal_Hall,
	Hall_CommutationId_T phaseAC,
	Hall_CommutationId_T phaseBC,
	Hall_CommutationId_T phaseBA,
	Hall_CommutationId_T phaseCA,
	Hall_CommutationId_T phaseCB,
	Hall_CommutationId_T phaseAB,
	uint8_t sensorIndexPhaseAC,
	uint8_t sensorIndexPhaseBC,
	uint8_t sensorIndexPhaseBA,
	uint8_t sensorIndexPhaseCA,
	uint8_t sensorIndexPhaseCB,
	uint8_t sensorIndexPhaseAB,
	Hall_CommutationId_T fault000,
	Hall_CommutationId_T fault111
);

void Hall_MapCommuntationTable
(
	Hall_T * p_hall,
	Hall_CommutationId_T phaseAC,
	Hall_CommutationId_T phaseBC,
	Hall_CommutationId_T phaseBA,
	Hall_CommutationId_T phaseCA,
	Hall_CommutationId_T phaseCB,
	Hall_CommutationId_T phaseAB,
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
	Hall_CommutationId_T fault000,
	Hall_CommutationId_T fault111
);

/*
 * calibrate phase to be next active phase when hall state is detected
 * non blocking implement outside module
 */
void Hall_CalibrateCommuntationTable_Blocking
(
	Hall_T * p_hall,
	Hall_CommutationId_T phaseAC,
	Hall_CommutationId_T phaseBC,
	Hall_CommutationId_T phaseBA,
	Hall_CommutationId_T phaseCA,
	Hall_CommutationId_T phaseCB,
	Hall_CommutationId_T phaseAB,
	void (* activatePwmValuePhaseABC)(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
	uint16_t pwm,
	void (*activatePwmStatePhaseABC)(bool enA, bool enB, bool enC),
	void (*delay)(uint32_t),
	uint32_t delayTime
);

#endif
