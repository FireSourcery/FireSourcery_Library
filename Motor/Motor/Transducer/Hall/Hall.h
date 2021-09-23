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
#include "HAL_Hall.h"

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
//#if defined(CONFIG_HALL_SENSORS_ORDER_CBA)
//	#define	HALL_SENSORS_A (0b001)
//	#define	HALL_SENSORS_B (0b010)
//	#define	HALL_SENSORS_C (0b100)
//	#define	HALL_SENSORS_INV_A (0b110)
//	#define	HALL_SENSORS_INV_B (0b101)
//	#define	HALL_SENSORS_INV_C (0b011)
//#elif (CONFIG_HALL_HALL_SENSORS_ORDER_ABC)
//	#define	HALL_SENSORS_A (0b100)
//	#define	HALL_SENSORS_B (0b010)
//	#define	HALL_SENSORS_C (0b001)
//	#define	HALL_SENSORS_NOT_A (0b011)
//	#define	HALL_SENSORS_NOT_B (0b101)
//	#define	HALL_SENSORS_NOT_C (0b110)
//#endif

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

	HALL_COMMUTATION_ANGLE_30 = 1,
	HALL_COMMUTATION_ANGLE_90 = 2,
	HALL_COMMUTATION_ANGLE_150 = 3,
	HALL_COMMUTATION_ANGLE_210 = 4,
	HALL_COMMUTATION_ANGLE_270 = 5,
	HALL_COMMUTATION_ANGLE_330 = 6,

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
} Hall_CommutationPhase_T;

#elif 	defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
//typedef void * Hall_CommutationId_T;
typedef void (* Hall_CommutationPhase_T)(void * p_userData);
#endif

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
} Hall_RotorAngle_T;

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
	 *
	 * CommutationTable - can be const at cost of 1 more level of dereference
	 * AngleTable - convert back from commutation id, commutation plus or minus 90 //need user maped tabled in either case
	 */
	Hall_CommutationPhase_T 	CommuntationTable[8U];
	Hall_RotorAngle_T 			RotorAngleTable[8U];

	volatile Hall_Direction_T 	Direction;
	volatile uint8_t 			SensorsSaved;	/* Save last read */
//	volatile Hall_Angle_T 		VirtualSensorSaved;
//	volatile Hall_CommutationPhase_T 	CommutationIdSaved; /* Save last read */

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
	void * p_UserData;
#endif
}
Hall_T;


/*
 * +180 degrees
 */
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
//	p_hall->VirtualSensorSaved = p_hall->AngleTable[p_hall->SensorsSaved];

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

	static inline bool Hall_PollCommutation_IO(Hall_T * p_hall)
	{
		bool isEdge;

		uint8_t hall = Hall_ReadSensors(p_hall);

		if (hall != p_hall->SensorsSaved)
		{
//			p_hall->SensorsSaved = hall;
			Hall_Commutate_IO(p_hall);

			isEdge = true;
		}
		else
		{
			isEdge = false;
		}

		return (isEdge);
	}

#endif

/*
 * Polling
 * hall edge, angle boundary
 */
static inline bool Hall_PollSensorsEdge_IO(Hall_T * p_hall)
{
	bool isEdge;

	uint8_t hall = Hall_ReadSensors(p_hall);

	if (hall != p_hall->SensorsSaved)
	{
		p_hall->SensorsSaved = hall;
//		p_hall->VirtualSensorSaved = p_hall->VirtualSensorTable[p_hall->SensorsSaved];
		isEdge = true;
	}
	else
	{
		isEdge = false;
	}

	return (isEdge);
}


//static inline bool Hall_PollCaptureSensors_IO(Hall_T * p_hall)
//{
//	bool isEdge = Hall_PollSensorsEdge_IO(p_hall);
//
//	if (isEdge)
//	{
//		Hall_CaptureSensors_IO(p_hall);
//	}
//
//	return (isEdge);
//}

//temp
static inline bool Hall_PollSensorA(Hall_T * p_hall)
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

static inline Hall_CommutationPhase_T Hall_ConvertCommutation(Hall_T * p_hall, uint8_t hallSensors)
{
//	const uint16_t COMMUTATION_TABLE[] =  //using virtual angle
//	{
//		[HALL_ANGLE_CCW_30] 	= HALL_SECTOR_2,
//		[HALL_ANGLE_CCW_90] 	= HALL_SECTOR_3,
//		[HALL_ANGLE_CCW_150] 	= HALL_SECTOR_4,
//		[HALL_ANGLE_CCW_210] 	= HALL_SECTOR_5,
//		[HALL_ANGLE_CCW_270] 	= HALL_SECTOR_6,
//		[HALL_ANGLE_CCW_330] 	= HALL_SECTOR_1,
//	};

	volatile Hall_CommutationPhase_T id;

	if (p_hall->Direction == HALL_DIRECTION_CW)
	{
		id = p_hall->CommuntationTable[InverseHall(hallSensors)];
	}
	else
	{
		id = p_hall->CommuntationTable[hallSensors];
	}

	return (id);
}

static inline Hall_CommutationPhase_T Hall_GetCommutation(Hall_T * p_hall)
{
	Hall_CommutationPhase_T id = Hall_ConvertCommutation(p_hall, p_hall->SensorsSaved);
	return (id);
}

static inline Hall_CommutationPhase_T Hall_ReadSensorsCommutation(Hall_T * p_hall)
{
	Hall_CommutationPhase_T id = Hall_ConvertCommutation(p_hall, Hall_ReadSensors(p_hall));
	return (id);
}

static inline Hall_RotorAngle_T Hall_ConvertRotorAngle(Hall_T * p_hall, uint8_t physicalSensors)
{
	return p_hall->RotorAngleTable[physicalSensors];
}

static inline uint16_t Hall_GetRotorAngle(Hall_T * p_hall)
{
	return p_hall->RotorAngleTable[p_hall->SensorsSaved];
//	return p_hall->VirtualSensorSaved;
}

static inline uint16_t Hall_ConvertRotorAngle_Degrees16(Hall_T * p_hall, uint8_t physicalSensors)
{
	const uint16_t DEGREES_TABLE[] =
	{
		[HALL_ANGLE_CCW_30] 	= 5461U,
		[HALL_ANGLE_CCW_90] 	= 16384U,
		[HALL_ANGLE_CCW_150] 	= 27306U,
		[HALL_ANGLE_CCW_210] 	= 38229U,
		[HALL_ANGLE_CCW_270] 	= 49152U,
		[HALL_ANGLE_CCW_330] 	= 60074U,
	};

	uint16_t degrees;

	Hall_RotorAngle_T angle = p_hall->RotorAngleTable[physicalSensors];

	if (p_hall->Direction == HALL_DIRECTION_CW)
	{
		degrees = DEGREES_TABLE[angle] + 10922U; //+ 60 degrees
	}
	else
	{
		degrees = DEGREES_TABLE[angle];
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
//	degrees = DEGREES_TABLE[p_hall->CommutationIdSaved]; DEGREES_TABLE[Hall_GetCommutation(p_hall->SensorsSaved)];
//
//	if (p_hall->Direction == HALL_DIRECTION_CW)
//	{
//		degrees += 16384U;
//	}

	return degrees;
}

static inline uint16_t Hall_GetRotorAngle_Degrees16(Hall_T * p_hall)
{
	Hall_ConvertRotorAngle_Degrees16(p_hall, p_hall->SensorsSaved);
}

static inline Hall_Direction_T Hall_GetDirection(Hall_T * p_hall)
{
	return (p_hall->Direction);
}

//Sets commutation direction
static inline void Hall_SetDirection(Hall_T * p_hall, Hall_Direction_T dir)
{
	p_hall->Direction = dir;
}

static inline void Hall_ToggleDirection(Hall_T * p_hall)
{
	p_hall->Direction = ~p_hall->Direction;
}


/*
	Can pass 0 for phase and sensor ids, and calibrate after
 */
void Hall_Init
(
	Hall_T * p_hall,
	const HAL_Hall_T * p_hal_Hall,
	Hall_CommutationPhase_T phaseAC,
	Hall_CommutationPhase_T phaseBC,
	Hall_CommutationPhase_T phaseBA,
	Hall_CommutationPhase_T phaseCA,
	Hall_CommutationPhase_T phaseCB,
	Hall_CommutationPhase_T phaseAB,
	uint8_t sensorIndexPhaseAC,
	uint8_t sensorIndexPhaseBC,
	uint8_t sensorIndexPhaseBA,
	uint8_t sensorIndexPhaseCA,
	uint8_t sensorIndexPhaseCB,
	uint8_t sensorIndexPhaseAB,
	Hall_CommutationPhase_T fault000,
	Hall_CommutationPhase_T fault111
);

void Hall_MapCommuntationTable
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T phaseAC,
	Hall_CommutationPhase_T phaseBC,
	Hall_CommutationPhase_T phaseBA,
	Hall_CommutationPhase_T phaseCA,
	Hall_CommutationPhase_T phaseCB,
	Hall_CommutationPhase_T phaseAB,
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
	Hall_CommutationPhase_T fault000,
	Hall_CommutationPhase_T fault111
);

/*
 * calibrate phase to be next active phase when hall state is detected
 * non blocking implement outside module
 */
void Hall_CalibrateCommuntationTable_Blocking
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T phaseAC,
	Hall_CommutationPhase_T phaseBC,
	Hall_CommutationPhase_T phaseBA,
	Hall_CommutationPhase_T phaseCA,
	Hall_CommutationPhase_T phaseCB,
	Hall_CommutationPhase_T phaseAB,
	void (* activatePwmValuePhaseABC)(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
	uint16_t pwm,
	void (*activatePwmStatePhaseABC)(bool enA, bool enB, bool enC),
	void (*delay)(uint32_t),
	uint32_t delayTime
);

#endif
