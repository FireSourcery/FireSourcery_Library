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
	Hall_Phase_T phaseAC,
	Hall_Phase_T phaseBC,
	Hall_Phase_T phaseBA,
	Hall_Phase_T phaseCA,
	Hall_Phase_T phaseCB,
	Hall_Phase_T phaseAB,
	uint8_t sensorIndexPhaseAC,
	uint8_t sensorIndexPhaseBC,
	uint8_t sensorIndexPhaseBA,
	uint8_t sensorIndexPhaseCA,
	uint8_t sensorIndexPhaseCB,
	uint8_t sensorIndexPhaseAB,
	Hall_Phase_T fault000,
	Hall_Phase_T fault111
)
{
	p_hall->p_HAL_Hall = p_hal_Hall;
	p_hall->Direction = HALL_DIRECTION_CW;
	p_hall->SensorsSaved = 0;
	p_hall->PhaseSaved = 0;

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
	Hall_Phase_T phaseAC,
	Hall_Phase_T phaseBC,
	Hall_Phase_T phaseBA,
	Hall_Phase_T phaseCA,
	Hall_Phase_T phaseCB,
	Hall_Phase_T phaseAB,
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
	Hall_T * p_hall,
	Hall_Phase_T phaseAC,
	Hall_Phase_T phaseBC,
	Hall_Phase_T phaseBA,
	Hall_Phase_T phaseCA,
	Hall_Phase_T phaseCB,
	Hall_Phase_T phaseAB
)
{
	//#if (HALL_SENSOR_ORDER_CBA)
	//	typedef enum
	//	{
	//		//2, 6, 4, 5, 1, 3,
	//		PHASE_DISABLE = 0,
	//		PHASE_AB = 0b011, //3
	//		PHASE_AC = 0b010, //2
	//		PHASE_BC = 0b110, //6
	//		PHASE_BA = 0b100, //4
	//		PHASE_CA = 0b101, //5
	//		PHASE_CB = 0b001, //1
	//	} PHASE_ID_T;
	//#elif (HALL_SENSOR_ORDER_ABC)
	//	typedef enum
	//	{
	//		PHASE_DISABLE = 0,
	//		PHASE_AB = 0b110, //6
	//		PHASE_AC = 0b010, //2
	//		PHASE_BC = 0b011, //3
	//		PHASE_BA = 0b001, //1
	//		PHASE_CA = 0b101, //5
	//		PHASE_CB = 0b100, //4
	//	} PHASE_ID_T;
	//#endif

	Hall_MapCommuntationTable
	(
		p_hall,
		phaseAC, phaseBC, phaseBA, phaseCA, phaseCB, phaseAB,
		2, 3, 1, 5, 4, 6
	);
}

void Hall_MapCommuntationTableFaultStates
(
	Hall_T * p_hall,
	Hall_Phase_T fault000,
	Hall_Phase_T fault111
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

	Hall_Phase_T phaseAC,
	Hall_Phase_T phaseBC,
	Hall_Phase_T phaseBA,
	Hall_Phase_T phaseCA,
	Hall_Phase_T phaseCB,
	Hall_Phase_T phaseAB,

	void (*activatePwmValuePhaseABC)(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
	uint16_t pwm,
	void (*activatePwmStatePhaseABC)(bool enA, bool enB, bool enC),

	void (*delay)(uint32_t time),
	uint32_t delayTime
)
{
	if(activatePwmStatePhaseABC) activatePwmStatePhaseABC(1,1,1);

	activatePwmValuePhaseABC(pwm, 0, 0);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall->p_HAL_Hall)] 	= phaseBC;
//	if (returnIndexBC) *returnIndexBC = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(pwm, pwm, 0);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall->p_HAL_Hall)] 	= phaseBA;
//	if (returnIndexBA) *returnIndexBA = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(0, pwm, 0);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall->p_HAL_Hall)] 	= phaseCA;
//	if (returnIndexCA) *returnIndexCA = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(0, pwm, pwm);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall->p_HAL_Hall)] 	= phaseCB;
//	if (returnIndexCB) *returnIndexCB = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(0, 0, pwm);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall->p_HAL_Hall)] 	= phaseAB;
//	if (returnIndexAB) *returnIndexAB = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmValuePhaseABC(pwm, 0, pwm);
	delay(delayTime);
	p_hall->CommuntationTable[Hall_ReadSensors(p_hall->p_HAL_Hall)] 	= phaseAC;
//	if (returnIndexAC) *returnIndexAC = Hall_ReadSensors(p_hall->p_HAL_Hall);

	activatePwmStatePhaseABC(0,0,0);
}
