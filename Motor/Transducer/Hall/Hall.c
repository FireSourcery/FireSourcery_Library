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
#include "HAL.h"

#include "Config.h"

#include <stdbool.h>
#include <stdint.h>

/*
	Can use 0 for phase and sensor ids, and calibrate after
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
)
{
	p_hall->p_HAL_Hall = p_hal_Hall;
	p_hall->Direction = HALL_DIRECTION_CW;
	p_hall->SensorsSaved.Byte = 0;
	p_hall->IdSaved = 0;

	Hall_MapCommuntationTable
	(
		p_hall,
		phaseAC, phaseBC, phaseBA, phaseCA, phaseCB, phaseAB,
		sensorIndexPhaseAC, sensorIndexPhaseBC, sensorIndexPhaseBA, sensorIndexPhaseCA, sensorIndexPhaseCB, sensorIndexPhaseAB
	);

	Hall_MapCommuntationTableFaultStates(p_hall, fault000, fault111);
}

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
)
{
	p_hall->CommuntationTable[sensorIndexPhaseAC] = phaseAC;
	p_hall->CommuntationTable[sensorIndexPhaseBC] = phaseBC;
	p_hall->CommuntationTable[sensorIndexPhaseBA] = phaseBA;
	p_hall->CommuntationTable[sensorIndexPhaseCA] = phaseCA;
	p_hall->CommuntationTable[sensorIndexPhaseCB] = phaseCB;
	p_hall->CommuntationTable[sensorIndexPhaseAB] = phaseAB;
}

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
		HALL_SENSORS_NOT_B,
		HALL_SENSORS_A,
		HALL_SENSORS_NOT_C,
		HALL_SENSORS_B,
		HALL_SENSORS_NOT_A,
		HALL_SENSORS_C
	);
}

/*
 * In cases where Hall_Id_T is a function pointer
 */
void Hall_MapCommuntationTable_PhaseDefault
(
	Hall_T * p_hall,
	Hall_Id_T phaseAC,
	Hall_Id_T phaseBC,
	Hall_Id_T phaseBA,
	Hall_Id_T phaseCA,
	Hall_Id_T phaseCB,
	Hall_Id_T phaseAB
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


void Hall_MapCommuntationTableFaultStates
(
	Hall_T * p_hall,
	Hall_Id_T fault000,
	Hall_Id_T fault111
)
{
	p_hall->CommuntationTable[0] = fault000;
	p_hall->CommuntationTable[7] = fault111;
}

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

	void (*activatePwmValuePhaseABC)(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
	uint16_t pwm,
	void (*activatePwmStatePhaseABC)(bool enA, bool enB, bool enC),

	void (*delay)(uint32_t time),
	uint32_t delayTime
)
{
	if(activatePwmStatePhaseABC) activatePwmStatePhaseABC(1,1,1);

	/*
	 * Calibrating for CCW, while rotating CCW, if ABC are connected as intended
	 */
	activatePwmValuePhaseABC(pwm, 0, 0);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall).Byte] = phaseBC;
//	if (returnIndexBC) *returnIndexBC = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(pwm, pwm, 0);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall).Byte] = phaseBA;
//	if (returnIndexBA) *returnIndexBA = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(0, pwm, 0);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall).Byte] = phaseCA;
//	if (returnIndexCA) *returnIndexCA = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(0, pwm, pwm);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall).Byte] = phaseCB;
//	if (returnIndexCB) *returnIndexCB = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(0, 0, pwm);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall).Byte] = phaseAB;
//	if (returnIndexAB) *returnIndexAB = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(pwm, 0, pwm);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall).Byte] = phaseAC;
//	if (returnIndexAC) *returnIndexAC = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmStatePhaseABC(0,0,0);
}
