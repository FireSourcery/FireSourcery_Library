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
    @file 	Hall.c
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#include "Hall.h"
#include "HAL_Hall.h"

#include "Config.h"

#include <stdbool.h>
#include <stdint.h>

/*
	Can use 0 for phase and sensor ids, and calibrate after
 */
void Hall_Init //InitCommutationTable
(
	Hall_T * p_hall,
	const HAL_Hall_T * p_hal_Hall,
	Hall_CommutationPhase_T phaseAC,
	Hall_CommutationPhase_T phaseBC,
	Hall_CommutationPhase_T phaseBA,
	Hall_CommutationPhase_T phaseCA,
	Hall_CommutationPhase_T phaseCB,
	Hall_CommutationPhase_T phaseAB,
	uint8_t sensorInvBPhaseAC,
	uint8_t sensorAPhaseBC,
	uint8_t sensorInvCPhaseBA,
	uint8_t sensorBPhaseCA,
	uint8_t sensorInvAPhaseCB,
	uint8_t sensorCPhaseAB,
	Hall_CommutationPhase_T fault000,
	Hall_CommutationPhase_T fault111
)

{
	p_hall->p_HAL_Hall = p_hal_Hall;
	p_hall->Direction = HALL_DIRECTION_CCW;
	p_hall->SensorsSaved  = 0U;
//	p_hall->VirtualSensorSaved = 0U;
//	p_hall->CommutationIdSaved = 0U;

	Hall_MapCommuntationTable
	(
		p_hall,
		phaseAC, phaseBC, phaseBA, phaseCA, phaseCB, phaseAB,
		sensorInvBPhaseAC, sensorAPhaseBC, sensorInvCPhaseBA, sensorBPhaseCA, sensorInvAPhaseCB, sensorCPhaseAB
	);

	p_hall->RotorAngleTable[0U] = 0U;
	p_hall->RotorAngleTable[7U] = 7U;

	Hall_MapCommuntationTableFaultStates(p_hall, fault000, fault111);

	HAL_Hall_Init(p_hal_Hall);
}

void Hall_Init_Default(Hall_T * p_hall, const HAL_Hall_T * p_hal_Hall)
{
	p_hall->p_HAL_Hall = p_hal_Hall;
	p_hall->Direction = HALL_DIRECTION_CCW;
	p_hall->SensorsSaved = 0U;

//	Hall_MapCommuntationTable_Default(p_hall);
//	Hall_MapCommuntationTableFaultStates(p_hall, 0U, 7U);

	HAL_Hall_Init(p_hal_Hall);
}

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
)
{
	p_hall->CommuntationTable[sensorIndexPhaseAC] = phaseAC;
	p_hall->CommuntationTable[sensorIndexPhaseBC] = phaseBC;
	p_hall->CommuntationTable[sensorIndexPhaseBA] = phaseBA;
	p_hall->CommuntationTable[sensorIndexPhaseCA] = phaseCA;
	p_hall->CommuntationTable[sensorIndexPhaseCB] = phaseCB;
	p_hall->CommuntationTable[sensorIndexPhaseAB] = phaseAB;

	p_hall->RotorAngleTable[sensorIndexPhaseAC] = HALL_VIRTUAL_SENSORS_INV_B;
	p_hall->RotorAngleTable[sensorIndexPhaseBC] = HALL_VIRTUAL_SENSORS_A;
	p_hall->RotorAngleTable[sensorIndexPhaseBA] = HALL_VIRTUAL_SENSORS_INV_C;
	p_hall->RotorAngleTable[sensorIndexPhaseCA] = HALL_VIRTUAL_SENSORS_B;
	p_hall->RotorAngleTable[sensorIndexPhaseCB] = HALL_VIRTUAL_SENSORS_INV_A;
	p_hall->RotorAngleTable[sensorIndexPhaseAB] = HALL_VIRTUAL_SENSORS_C;
}


#if defined(CONFIG_HALL_COMMUTATION_TABLE_SECTOR_ID)
void Hall_MapCommuntationTable_Default
(
	Hall_T * p_hall
)
{
//	/*
//	 * Calibrating for CW
//	 */
//	Hall_MapCommuntationTable
//	(
//		p_hall,
//		HALL_PHASE_AC,
//		HALL_PHASE_BC,
//		HALL_PHASE_BA,
//		HALL_PHASE_CA,
//		HALL_PHASE_CB,
//		HALL_PHASE_AB,
//		HALL_SENSORS_B,
//		HALL_SENSORS_NOT_A,
//		HALL_SENSORS_C,
//		HALL_SENSORS_NOT_B,
//		HALL_SENSORS_A,
//		HALL_SENSORS_NOT_C
//	);

	/*
	 * Calibrating for CCW
	 */
	Hall_MapCommuntationTable
	(
		p_hall,
		HALL_PHASE_AC,
		HALL_PHASE_BC,
		HALL_PHASE_BA,
		HALL_PHASE_CA,
		HALL_PHASE_CB,
		HALL_PHASE_AB,
		HALL_VIRTUAL_SENSORS_INV_B,
		HALL_VIRTUAL_SENSORS_A,
		HALL_VIRTUAL_SENSORS_INV_C,
		HALL_VIRTUAL_SENSORS_B,
		HALL_VIRTUAL_SENSORS_INV_A,
		HALL_VIRTUAL_SENSORS_C
	);
}
#elif defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
/*
 * In cases where Hall_CommutationId_T is a function pointer
 */
void Hall_MapCommuntationTable_PhaseDefault
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T phaseAC,
	Hall_CommutationPhase_T phaseBC,
	Hall_CommutationPhase_T phaseBA,
	Hall_CommutationPhase_T phaseCA,
	Hall_CommutationPhase_T phaseCB,
	Hall_CommutationPhase_T phaseAB
)
{
//	/*
//	 * Calibrating for CW
//	 */
//	Hall_MapCommuntationTable
//	(
//		p_hall,
//		phaseAC,
//		phaseBC,
//		phaseBA,
//		phaseCA,
//		phaseCB,
//		phaseAB,
//		HALL_SENSORS_B,
//		HALL_SENSORS_NOT_A,
//		HALL_SENSORS_C,
//		HALL_SENSORS_NOT_B,
//		HALL_SENSORS_A,
//		HALL_SENSORS_NOT_C
//	);

	/*
	 * Calibrating for CCW
	 */
	Hall_MapCommuntationTable
	(
		p_hall,
		phaseAC,
		phaseBC,
		phaseBA,
		phaseCA,
		phaseCB,
		phaseAB,
		HALL_SENSORS_NOT_B,
		HALL_SENSORS_A,
		HALL_SENSORS_NOT_C,
		HALL_SENSORS_B,
		HALL_SENSORS_NOT_A,
		HALL_SENSORS_C
	);
}
#endif

void Hall_MapCommuntationTableFaultStates
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T fault000,
	Hall_CommutationPhase_T fault111
)
{
	p_hall->CommuntationTable[0] = fault000;
	p_hall->CommuntationTable[7] = fault111;
}

//Non blocking calibration
void Hall_CalibrateSensorAPhaseBC
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T phaseBC
)
{
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseBC;
	p_hall->RotorAngleTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_A;
//	p_hall->RotorAngleTable[HALL_VIRTUAL_SENSOR_A] = Hall_ReadSensors(p_hall);
}

void Hall_CalibrateSensorInvCPhaseBA
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T phaseBA
)
{
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseBA;
	p_hall->RotorAngleTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_INV_C;
}

void Hall_CalibrateSensorBPhaseCA
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T phaseCA
)
{
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseCA;
	p_hall->RotorAngleTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_B;
}

void Hall_CalibrateSensorInvAPhaseCB
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T phaseCB
)
{
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseCB;
	p_hall->RotorAngleTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_INV_A;
}

void Hall_CalibrateSensorCPhaseAB
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T phaseAB
)
{
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseAB;
	p_hall->RotorAngleTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_C;
}

void Hall_CalibrateSensorInvBPhaseAC
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T phaseAC
)
{
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseAC;
	p_hall->RotorAngleTable[Hall_ReadSensors(p_hall)] = HALL_VIRTUAL_SENSORS_INV_B;
}

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

	void (*activatePwmValuePhaseABC)(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
	uint16_t pwm,
	void (*activatePwmStatePhaseABC)(bool enA, bool enB, bool enC),

	void (*delay)(uint32_t time),
	uint32_t delayTime
)
{
	if(activatePwmStatePhaseABC) {activatePwmStatePhaseABC(1,1,1);}

	/*
	 * Calibrating for CCW, while rotating CCW, if ABC are connected as intended
	 */
	activatePwmValuePhaseABC(pwm, 0, 0);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseBC;
//	if (returnIndexBC) *returnIndexBC = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(pwm, pwm, 0);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseBA;
//	if (returnIndexBA) *returnIndexBA = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(0, pwm, 0);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseCA;
//	if (returnIndexCA) *returnIndexCA = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(0, pwm, pwm);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseCB;
//	if (returnIndexCB) *returnIndexCB = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(0, 0, pwm);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseAB;
//	if (returnIndexAB) *returnIndexAB = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(pwm, 0, pwm);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseAC;
//	if (returnIndexAC) *returnIndexAC = Hall_ReadSensors(p_hall->p_HAL_Hall);

	if (activatePwmStatePhaseABC){activatePwmStatePhaseABC(0,0,0);}
}

//void Hall_CalibrateCommuntationTable_NonBlocking
//(
//	Hall_T * p_hall,
//
//	Hall_CommutationId_T phaseAC,
//	Hall_CommutationId_T phaseBC,
//	Hall_CommutationId_T phaseBA,
//	Hall_CommutationId_T phaseCA,
//	Hall_CommutationId_T phaseCB,
//	Hall_CommutationId_T phaseAB,
//
//	void (*activatePwmValuePhaseABC)(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
//	uint16_t pwm,
//	void (*activatePwmStatePhaseABC)(bool enA, bool enB, bool enC)
//)
//{
//	static uint8_t state = 0; //limits calibration to 1 at a time;
//
//	switch (state)
//	{
//	case 0:
//		if (activatePwmStatePhaseABC){activatePwmStatePhaseABC(1, 1, 1);}
//		activatePwmValuePhaseABC(pwm, 0, 0);
//		p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseBC;
//		state++;
//		break;
//
//	case 1:
//		activatePwmValuePhaseABC(pwm, pwm, 0);
//		p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseBA;
//		state++;
//		break;
//
//	case 2:
//		activatePwmValuePhaseABC(0, pwm, 0);
//		p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseCA;
//		state++;
//		break;
//
//	case 3:
//		activatePwmValuePhaseABC(0, pwm, pwm);
//		p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseCB;
//		state++;
//		break;
//
//	case 4:
//		activatePwmValuePhaseABC(0, 0, pwm);
//		p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseAB;
//		state++;
//		break;
//
//	case 5:
//		activatePwmValuePhaseABC(pwm, 0, pwm);
//		p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseAC;
//		if (activatePwmStatePhaseABC){activatePwmStatePhaseABC(0, 0, 0);}
//		state = 0;
//		break;
//
//	default:
//		break;
//
//	}
//}
