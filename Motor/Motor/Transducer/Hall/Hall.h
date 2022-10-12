/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@author FireSourcery
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

#if defined(CONFIG_HALL_ID_BASE_PINS_STATE)
	/*
		Where ID => 0bCBA
	*/
	#define	HALL_VIRTUAL_SENSORS_A 		(0b001U)
	#define	HALL_VIRTUAL_SENSORS_B 		(0b010U)
	#define	HALL_VIRTUAL_SENSORS_C 		(0b100U)
	#define	HALL_VIRTUAL_SENSORS_INV_A 	(0b110U)
	#define	HALL_VIRTUAL_SENSORS_INV_B 	(0b101U)
	#define	HALL_VIRTUAL_SENSORS_INV_C 	(0b011U)
#elif defined(CONFIG_HALL_ID_BASE_ROTOR_SECTOR)
	/*
		Where ID 001 => rotor angle 0 [330, 30] degrees
		1 previous to commutation
	*/
	#define	HALL_VIRTUAL_SENSORS_A 		(1U)
	#define	HALL_VIRTUAL_SENSORS_INV_C 	(2U)
	#define	HALL_VIRTUAL_SENSORS_B 		(3U)
	#define	HALL_VIRTUAL_SENSORS_INV_A 	(4U)
	#define	HALL_VIRTUAL_SENSORS_C 		(5U)
	#define	HALL_VIRTUAL_SENSORS_INV_B 	(6U)
#elif defined(CONFIG_HALL_ID_BASE_COMMUTATION_SECTOR)
	/*
		Where ID 001 => commutation phase AC
	*/
	#define	HALL_VIRTUAL_SENSORS_INV_B 	(1U)
	#define	HALL_VIRTUAL_SENSORS_A 		(2U)
	#define	HALL_VIRTUAL_SENSORS_INV_C 	(3U)
	#define	HALL_VIRTUAL_SENSORS_B 		(4U)
	#define	HALL_VIRTUAL_SENSORS_INV_A 	(5U)
	#define	HALL_VIRTUAL_SENSORS_C 		(6U)
#endif

#define	HALL_SENSORS_TABLE_LENGTH 	(8U)

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
typedef void (*Hall_CommutationPhase_T)(void * p_userData);
#endif

typedef union Hall_Sensors_Tag
{
	struct
	{
		uint8_t A : 1U;
		uint8_t B : 1U;
		uint8_t C : 1U;
		uint8_t Resv3 : 1U;
		uint8_t Resv4 : 1U;
		uint8_t Resv5 : 1U;
		uint8_t Resv6 : 1U;
		uint8_t Resv7 : 1U;
	};
	uint8_t State;
}
Hall_Sensors_T;

/*
	Hall sensor ID. ID base value reflects 3 bit sensor state, or sequential ID
	if both sequential ID, and sensor state is needed, another table must be used
*/
typedef enum Hall_Id_Tag
{
	/*
		Rotor Angle
	*/
	HALL_ANGLE_30_90 = HALL_VIRTUAL_SENSORS_INV_C,
	HALL_ANGLE_90_150 = HALL_VIRTUAL_SENSORS_B,
	HALL_ANGLE_150_210 = HALL_VIRTUAL_SENSORS_INV_A,
	HALL_ANGLE_210_270 = HALL_VIRTUAL_SENSORS_C,
	HALL_ANGLE_270_330 = HALL_VIRTUAL_SENSORS_INV_B,
	HALL_ANGLE_330_30 = HALL_VIRTUAL_SENSORS_A,
	HALL_ANGLE_ERROR_0 = 0U,
	HALL_ANGLE_ERROR_7 = 7U,

	/* Rotor Angle Id via boundary from CCW and CW */
	HALL_ANGLE_CCW_30 = HALL_ANGLE_30_90,
	HALL_ANGLE_CCW_90 = HALL_ANGLE_90_150,
	HALL_ANGLE_CCW_150 = HALL_ANGLE_150_210,
	HALL_ANGLE_CCW_210 = HALL_ANGLE_210_270,
	HALL_ANGLE_CCW_270 = HALL_ANGLE_270_330,
	HALL_ANGLE_CCW_330 = HALL_ANGLE_330_30,

	HALL_ANGLE_CW_30 = HALL_ANGLE_330_30,
	HALL_ANGLE_CW_90 = HALL_ANGLE_30_90,
	HALL_ANGLE_CW_150 = HALL_ANGLE_90_150,
	HALL_ANGLE_CW_210 = HALL_ANGLE_150_210,
	HALL_ANGLE_CW_270 = HALL_ANGLE_210_270,
	HALL_ANGLE_CW_330 = HALL_ANGLE_270_330,

	/*
		Commutation angle
		CCW direction, 90 degree
	*/
	HALL_COMMUTATION_PHASE_AC = HALL_VIRTUAL_SENSORS_INV_B,
	HALL_COMMUTATION_PHASE_BC = HALL_VIRTUAL_SENSORS_A,
	HALL_COMMUTATION_PHASE_BA = HALL_VIRTUAL_SENSORS_INV_C,
	HALL_COMMUTATION_PHASE_CA = HALL_VIRTUAL_SENSORS_B,
	HALL_COMMUTATION_PHASE_CB = HALL_VIRTUAL_SENSORS_INV_A,
	HALL_COMMUTATION_PHASE_AB = HALL_VIRTUAL_SENSORS_C,
	HALL_COMMUTATION_PHASE_ERROR_0 = 0U,
	HALL_COMMUTATION_PHASE_ERROR_7 = 7U,

	HALL_COMMUTATION_ANGLE_30 = HALL_COMMUTATION_PHASE_AC,
	HALL_COMMUTATION_ANGLE_90 = HALL_COMMUTATION_PHASE_BC,
	HALL_COMMUTATION_ANGLE_150 = HALL_COMMUTATION_PHASE_BA,
	HALL_COMMUTATION_ANGLE_210 = HALL_COMMUTATION_PHASE_CA,
	HALL_COMMUTATION_ANGLE_270 = HALL_COMMUTATION_PHASE_CB,
	HALL_COMMUTATION_ANGLE_330 = HALL_COMMUTATION_PHASE_AB,

	/*
		Sector number
	*/
	HALL_SECTOR_0 = 0U,
	HALL_SECTOR_1 = HALL_VIRTUAL_SENSORS_A,
	HALL_SECTOR_2 = HALL_VIRTUAL_SENSORS_INV_C,
	HALL_SECTOR_3 = HALL_VIRTUAL_SENSORS_B,
	HALL_SECTOR_4 = HALL_VIRTUAL_SENSORS_INV_A,
	HALL_SECTOR_5 = HALL_VIRTUAL_SENSORS_C,
	HALL_SECTOR_6 = HALL_VIRTUAL_SENSORS_INV_B,
	HALL_SECTOR_7 = 7U,
}
Hall_Id_T;

typedef enum Hall_Direction_Tag
{
	HALL_DIRECTION_CW = 0U,
	HALL_DIRECTION_CCW = 1U,
}
Hall_Direction_T;

typedef struct __attribute__((aligned(4U))) Hall_Params_Tag
{
	Hall_Id_T SensorsTable[HALL_SENSORS_TABLE_LENGTH];
}
Hall_Params_T;

typedef const struct Hall_Config_Tag
{
	const Hall_Params_T * const P_PARAMS_NVM;
}
Hall_Config_T;

typedef struct Hall_Tag
{
	const Hall_Config_T CONFIG;
	Pin_T PinA;
	Pin_T PinB;
	Pin_T PinC;
	Hall_Params_T Params;
	Hall_Direction_T Direction;
	Hall_Sensors_T SensorsRef; 		/* Save last read */
#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
	Hall_CommutationPhase_T 	CommuntationTable[8U];
	void * p_CommutationContext;
#endif
}
Hall_T;

#define HALL_INIT(p_PinAHal, PinAId, p_PinBHal, PinBId, p_PinCHal, PinCId, p_Params)	\
{																\
	.CONFIG = 													\
	{															\
		.P_PARAMS_NVM = p_Params, 								\
	},															\
	.PinA = PIN_INIT(p_PinAHal, PinAId),						\
	.PinB = PIN_INIT(p_PinBHal, PinBId),						\
	.PinC = PIN_INIT(p_PinCHal, PinCId),						\
}

/* +180 degrees */
static inline uint8_t InverseHall(uint8_t hall) { return (~hall & 0x07U); }

static inline Hall_Sensors_T Hall_ReadSensors(const Hall_T * p_hall)
{
	Hall_Sensors_T sensors = {.State = 0U};

	sensors.A = Pin_Input_ReadPhysical(&p_hall->PinA);
	sensors.B = Pin_Input_ReadPhysical(&p_hall->PinB);
	sensors.C = Pin_Input_ReadPhysical(&p_hall->PinC);

	return sensors;
}

/*

*/
static inline void Hall_CaptureSensors_ISR(Hall_T * p_hall)
{
	p_hall->SensorsRef.State = Hall_ReadSensors(p_hall).State;
}

/*
	Capture sensor on hall edge, angle boundary
	return true on every phase edge. i.e 6x per hall cycle
*/
static inline bool Hall_PollCaptureSensors(Hall_T * p_hall)
{
	uint8_t sensorsNew = Hall_ReadSensors(p_hall).State;
	bool isEdge = (sensorsNew != p_hall->SensorsRef.State);

	if(isEdge)
	{
		p_hall->SensorsRef.State = sensorsNew;
	}

	return (isEdge);
}

/*
	Next capture is edge
*/
static inline void Hall_ResetCapture(Hall_T * p_hall)
{
	p_hall->SensorsRef.State = 0U;
}

/*
	return true once per hall cycle
*/
static inline bool Hall_PollEdgeA(Hall_T * p_hall)
{
	return ((Hall_ReadSensors(p_hall).A == true) && ((p_hall->SensorsRef.A) == false));
}

static inline Hall_Id_T Hall_ConvertToRotorId(Hall_T * p_hall, uint8_t physicalSensors)
{
	return p_hall->Params.SensorsTable[physicalSensors];
}

static inline Hall_Id_T Hall_ConvertToCommutationId(Hall_T * p_hall, uint8_t physicalSensors)
{
	return (p_hall->Direction == HALL_DIRECTION_CW) ? p_hall->Params.SensorsTable[InverseHall(physicalSensors)] : p_hall->Params.SensorsTable[physicalSensors]; /* Offset 180 */
}

static inline uint16_t Hall_ConvertToAngle_Degrees16(Hall_T * p_hall, uint8_t physicalSensors)
{
	static const uint16_t DEGREES_TABLE[8U] =
	{
		[HALL_ANGLE_30_90] 		= 10922U,
		[HALL_ANGLE_90_150] 	= 21845U,
		[HALL_ANGLE_150_210] 	= 32768U,
		[HALL_ANGLE_210_270] 	= 43690U,
		[HALL_ANGLE_270_330] 	= 54613U,
		[HALL_ANGLE_330_30] 	= 0U,
	};

	Hall_Id_T angle = p_hall->Params.SensorsTable[physicalSensors];

	return DEGREES_TABLE[angle];
}

static inline uint16_t Hall_ConvertToRotorAngle_Degrees16(Hall_T * p_hall, uint8_t physicalSensors)
{
	uint16_t angle16 = Hall_ConvertToAngle_Degrees16(p_hall, physicalSensors);

	return ((p_hall->Direction == HALL_DIRECTION_CW) ? angle16 + 5461U : angle16 - 5461U); /* CW = CCW + 60 degrees */
}

static inline Hall_Id_T Hall_GetRotorId(Hall_T * p_hall)
{
	return Hall_ConvertToRotorId(p_hall, p_hall->SensorsRef.State);
}

/* returns based on Direction */
static inline Hall_Id_T Hall_GetCommutationId(Hall_T * p_hall)
{
	return Hall_ConvertToCommutationId(p_hall, p_hall->SensorsRef.State);
}

/* returns based on Direction */
static inline uint16_t Hall_GetRotorAngle_Degrees16(Hall_T * p_hall)
{
	return Hall_ConvertToRotorAngle_Degrees16(p_hall, p_hall->SensorsRef.State);
}

static inline Hall_Direction_T Hall_GetDirection(Hall_T * p_hall)
{
	return (p_hall->Direction);
}

/* Sets direction => commutation, angle degrees16 conversion */
static inline void Hall_SetDirection(Hall_T * p_hall, Hall_Direction_T dir)
{
	p_hall->Direction = dir;
}

static inline void Hall_ToggleDirection(Hall_T * p_hall)
{
	p_hall->Direction = ~p_hall->Direction;
}

static inline Hall_Sensors_T Hall_GetSensors(Hall_T * p_hall) { return p_hall->SensorsRef; }
static inline uint8_t Hall_GetSensorsId(Hall_T * p_hall) { return p_hall->SensorsRef.State; }
static inline bool Hall_GetSensorA(Hall_T * p_hall) { return p_hall->SensorsRef.A; }
static inline bool Hall_GetSensorB(Hall_T * p_hall) { return p_hall->SensorsRef.B; }
static inline bool Hall_GetSensorC(Hall_T * p_hall) { return p_hall->SensorsRef.C; }

/*

*/
extern void Hall_Init(Hall_T * p_hall);
extern void Hall_SetSensorsTable(Hall_T * p_hall, uint8_t sensorsA, uint8_t sensorsInvC, uint8_t sensorsB, uint8_t sensorsInvA, uint8_t sensorsC, uint8_t sensorsInvB);
extern void Hall_CalibratePhaseA(Hall_T * p_hall);
extern void Hall_CalibratePhaseInvC(Hall_T * p_hall);
extern void Hall_CalibratePhaseB(Hall_T * p_hall);
extern void Hall_CalibratePhaseInvA(Hall_T * p_hall);
extern void Hall_CalibratePhaseC(Hall_T * p_hall);
extern void Hall_CalibratePhaseInvB(Hall_T * p_hall);

#endif
