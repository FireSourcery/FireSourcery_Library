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
    @file 	Hall.h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef HALL_H
#define HALL_H

#include "Config.h"
#include "Peripheral/Pin/Pin.h"

#include <stdbool.h>
#include <stdint.h>

typedef union
{
	struct
	{
		uint8_t A :1;
		uint8_t B :1;
		uint8_t C :1;
		uint8_t Resv3 :1;
		uint8_t Resv4 :1;
		uint8_t Resv5 :1;
		uint8_t Resv6 :1;
		uint8_t Resv7 :1;
	};
	uint8_t State;
}
Hall_Sensors_T;

#ifdef CONFIG_HALL_SENSORS_ID_PINS_STATE
#define	HALL_VIRTUAL_SENSORS_A 		(0b001U)
#define	HALL_VIRTUAL_SENSORS_B 		(0b010U)
#define	HALL_VIRTUAL_SENSORS_C 		(0b100U)
#define	HALL_VIRTUAL_SENSORS_INV_A 	(0b110U)
#define	HALL_VIRTUAL_SENSORS_INV_B 	(0b101U)
#define	HALL_VIRTUAL_SENSORS_INV_C 	(0b011U)
#else
//#ifdef CONFIG_HALL_SENSORS_ID_BASE_COMMUTATION_SECTOR
//where  sector 1 => commutation phase AC
#define	HALL_VIRTUAL_SENSORS_A 		(2U)
#define	HALL_VIRTUAL_SENSORS_INV_C 	(3U)
#define	HALL_VIRTUAL_SENSORS_B 		(4U)
#define	HALL_VIRTUAL_SENSORS_INV_A 	(5U)
#define	HALL_VIRTUAL_SENSORS_C 		(6U)
#define	HALL_VIRTUAL_SENSORS_INV_B 	(1U)

//#ifdef CONFIG_HALL_SENSORS_ID_BASE_ROTOR_SECTOR
//where sector 1 => rotor angle 0 [330, 30] degrees
//add 1 if need match commutation
//#define	HALL_VIRTUAL_SENSORS_A 		(1U)
//#define	HALL_VIRTUAL_SENSORS_INV_C 	(2U)
//#define	HALL_VIRTUAL_SENSORS_B 		(3U)
//#define	HALL_VIRTUAL_SENSORS_INV_A 	(4U)
//#define	HALL_VIRTUAL_SENSORS_C 		(5U)
//#define	HALL_VIRTUAL_SENSORS_INV_B 	(6U)
#endif

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
typedef void (* Hall_CommutationPhase_T)(void * p_userData);
#endif

/*
 * Hall sensor id. id base value reflects 3 bit sensor state, or sequential id
 * if both sequential id, and sensor state is needed, another table must be used
 */
typedef enum
{
	/*
	 * Rotor Angle
	 */
	HALL_ANGLE_30_90 	= HALL_VIRTUAL_SENSORS_INV_C,
	HALL_ANGLE_90_150 	= HALL_VIRTUAL_SENSORS_B,
	HALL_ANGLE_150_210 	= HALL_VIRTUAL_SENSORS_INV_A,
	HALL_ANGLE_210_270	= HALL_VIRTUAL_SENSORS_C,
	HALL_ANGLE_270_330 	= HALL_VIRTUAL_SENSORS_INV_B,
	HALL_ANGLE_330_30 	= HALL_VIRTUAL_SENSORS_A,

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

	/*
 	 	Commutation angle
		CCW direction, 90 degree
	 */
	HALL_COMMUTATION_PHASE_0 = 0U,
	HALL_COMMUTATION_PHASE_AC = HALL_ANGLE_CCW_270,
	HALL_COMMUTATION_PHASE_BC = HALL_ANGLE_CCW_330,
	HALL_COMMUTATION_PHASE_BA = HALL_ANGLE_CCW_30,
	HALL_COMMUTATION_PHASE_CA = HALL_ANGLE_CCW_90,
	HALL_COMMUTATION_PHASE_CB = HALL_ANGLE_CCW_150,
	HALL_COMMUTATION_PHASE_AB = HALL_ANGLE_CCW_210,
	HALL_COMMUTATION_PHASE_7 = 7U,

	HALL_COMMUTATION_ANGLE_30 = HALL_COMMUTATION_PHASE_AC,
	HALL_COMMUTATION_ANGLE_90 = HALL_COMMUTATION_PHASE_BC,
	HALL_COMMUTATION_ANGLE_150 = HALL_COMMUTATION_PHASE_BA,
	HALL_COMMUTATION_ANGLE_210 = HALL_COMMUTATION_PHASE_CA,
	HALL_COMMUTATION_ANGLE_270 = HALL_COMMUTATION_PHASE_CB,
	HALL_COMMUTATION_ANGLE_330 = HALL_COMMUTATION_PHASE_AB,

	//	HALL_SECTOR_0 = 0U,
	//	HALL_SECTOR_1 = HALL_ANGLE_330_30,
	//	HALL_SECTOR_2 = HALL_ANGLE_30_90,
	//	HALL_SECTOR_3 = HALL_ANGLE_90_150,
	//	HALL_SECTOR_4 = HALL_ANGLE_150_210,
	//	HALL_SECTOR_5 = HALL_ANGLE_210_270,
	//	HALL_SECTOR_6 = HALL_ANGLE_270_330,
	//	HALL_SECTOR_7 = 7U,
}
Hall_SensorsId_T;

#define	HALL_SENSORS_TABLE_LENGTH 	(8U)

typedef enum
{
	HALL_DIRECTION_CW 	= 0U,
	HALL_DIRECTION_CCW 	= 1U,
}
Hall_Direction_T;

typedef struct
{
	const Pin_T PIN_A;
	const Pin_T PIN_B;
	const Pin_T PIN_C;

//	const Hall_SensorsId_T * const P_SENSOR_TABLE; //NV Memory copy
}
Hall_Config_T;

typedef struct 
{
	const Hall_Config_T CONFIG;

	Hall_SensorsId_T SensorsTable[HALL_SENSORS_TABLE_LENGTH];
	Hall_Direction_T Direction;
	Hall_Sensors_T SensorsRef; 		/* Save last read */
#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
	Hall_CommutationPhase_T 	CommuntationTable[8U];
	void * p_CommutationContext;
#endif
}
Hall_T;

#define HALL_CONFIG(p_PinA_Hal, PinAId, p_PinB_Hal, PinBId, p_PinC_Hal, PinCId)	\
{																\
	.CONFIG = 													\
	{															\
		.PIN_A = PIN_CONFIG(p_PinA_Hal, PinAId),				\
		.PIN_B = PIN_CONFIG(p_PinB_Hal, PinBId),				\
		.PIN_C = PIN_CONFIG(p_PinC_Hal, PinCId),				\
	}															\
}
//		.P_SENSOR_TABLE = p_SensorTableFlash

/*
 * +180 degrees
 */
static inline uint8_t InverseHall(uint8_t hall) {return (~hall & 0x07U);}

static inline uint8_t Hall_ReadSensors(const Hall_T * p_hall)
{
	Hall_Sensors_T sensors = {.State = 0U};

	sensors.A = Pin_Input_Read(&p_hall->CONFIG.PIN_A);
	sensors.B = Pin_Input_Read(&p_hall->CONFIG.PIN_B);
	sensors.C = Pin_Input_Read(&p_hall->CONFIG.PIN_C);

	return sensors.State ;
}

/*
 * ISR version
 */
static inline void Hall_CaptureSensors(Hall_T * p_hall)
{
	p_hall->SensorsRef.State  = Hall_ReadSensors(p_hall);
}

//static inline bool Hall_PollSensorsEdge(Hall_T * p_hall)
//{
//	return (Hall_ReadSensors(p_hall) != p_hall->SensorsRef) ? true : false;
//}

/*
 * return true on single phase edge. i.e 6x per hall cycle
 * hall edge, angle boundary
 */
static inline bool Hall_PollCaptureSensors(Hall_T * p_hall)
{
	uint8_t sensorsNew = Hall_ReadSensors(p_hall);
	bool isEdge = (sensorsNew != p_hall->SensorsRef.State);

	if (isEdge)
	{
		p_hall->SensorsRef.State = sensorsNew;
	}

	return (isEdge);
}

/*
 * return true 1x per hall cycle
 */
static inline bool Hall_PollSensorA(Hall_T * p_hall)
{
	return ((((Hall_Sensors_T)Hall_ReadSensors(p_hall)).A == true) && ((p_hall->SensorsRef.A) == false));
}

static inline Hall_SensorsId_T Hall_ConvertToSensorsId(Hall_T * p_hall, uint8_t physicalSensors)
{
	return p_hall->SensorsTable[physicalSensors];
}

static inline Hall_SensorsId_T Hall_ConvertToCommutationId(Hall_T * p_hall, uint8_t physicalSensors)
{
	return (p_hall->Direction == HALL_DIRECTION_CW) ? p_hall->SensorsTable[InverseHall(physicalSensors)] : p_hall->SensorsTable[physicalSensors];
}

static inline uint16_t Hall_ConvertToRotorAngle_Degrees16(Hall_T * p_hall, uint8_t physicalSensors)
{
	static const uint16_t DEGREES_TABLE[] =
	{
		[HALL_ANGLE_CCW_30] 	= 5461U,
		[HALL_ANGLE_CCW_90] 	= 16384U,
		[HALL_ANGLE_CCW_150] 	= 27306U,
		[HALL_ANGLE_CCW_210] 	= 38229U,
		[HALL_ANGLE_CCW_270] 	= 49152U,
		[HALL_ANGLE_CCW_330] 	= 60074U,
	};

	Hall_SensorsId_T angle 	= p_hall->SensorsTable[physicalSensors];

	return ((p_hall->Direction == HALL_DIRECTION_CW) ? DEGREES_TABLE[angle] + 10922U : DEGREES_TABLE[angle]); /* CW = CCW + 60 degrees */
}

static inline Hall_SensorsId_T Hall_GetSensorsId(Hall_T * p_hall)
{
	return Hall_ConvertToSensorsId(p_hall, p_hall->SensorsRef.State);
}

static inline Hall_SensorsId_T Hall_GetCommutationId(Hall_T * p_hall)
{
	return Hall_ConvertToCommutationId(p_hall, p_hall->SensorsRef.State);
}

/*
 * returns based on Direction
 */
static inline uint16_t Hall_GetRotorAngle_Degrees16(Hall_T * p_hall)
{
	return Hall_ConvertToRotorAngle_Degrees16(p_hall, p_hall->SensorsRef.State);
}

static inline uint16_t Hall_GetRotorAngleMid_Degrees16(Hall_T * p_hall)
{
//	return  DEGREES_TABLE[angle] + 65536U / 12U); /*   CCW + 30 degrees */
}

static inline Hall_Direction_T Hall_GetDirection(Hall_T * p_hall)
{
	return (p_hall->Direction);
}

/*
 * Sets commutation direction
 */
static inline void Hall_SetDirection(Hall_T * p_hall, Hall_Direction_T dir)
{
	p_hall->Direction = dir;
}

static inline void Hall_ToggleDirection(Hall_T * p_hall)
{
	p_hall->Direction = ~p_hall->Direction;
}

/*

 */
extern void Hall_Init(Hall_T * p_hall);
extern void Hall_MapSensorsTable(Hall_T * p_hall, uint8_t sensorsA, uint8_t sensorsInvC, uint8_t sensorsB, uint8_t sensorsInvA, uint8_t sensorsC, uint8_t sensorsInvB);
extern void Hall_MapSensorsTable_Default(Hall_T * p_hall);

#endif

