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
	@file 	.h
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Motor_Analog.h"
#include "Motor_FOC.h"
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
#include "Motor_SixStep.h"
#endif

/******************************************************************************/
/*!
	@brief  Callback functions must be mapped
*/
/******************************************************************************/
/******************************************************************************/
/*!
	@brief  Vabc
*/
/******************************************************************************/
static inline void CaptureVPeak(Motor_T * p_motor, uint16_t adcu) { if(adcu > p_motor->VBemfPeakTemp_Adcu) { p_motor->VBemfPeakTemp_Adcu = adcu; } }

void Motor_Analog_CaptureVa(Motor_T * p_motor)
{
	Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureVa, 0U/* Motor_SixStep_CaptureBemfA */);
	CaptureVPeak(p_motor, p_motor->AnalogResults.Va_Adcu);
}

void Motor_Analog_CaptureVb(Motor_T * p_motor)
{
	Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureVb, 0U/* Motor_SixStep_CaptureBemfB */);
	// CaptureVPeak(p_motor, p_motor->AnalogResults.Vb_Adcu);
}

void Motor_Analog_CaptureVc(Motor_T * p_motor)
{
	Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureVc, 0U/* Motor_SixStep_CaptureBemfC */);
	// CaptureVPeak(p_motor, p_motor->AnalogResults.Vc_Adcu);
}

/******************************************************************************/
/*!
	@brief  Iabc
*/
/******************************************************************************/
static inline void CaptureIZeroToPeak(Motor_T * p_motor, uint16_t adcuZero, uint16_t adcu)
{
	int16_t zeroToPeak = (int16_t)adcu - (int16_t)adcuZero;
	uint16_t zeroToPeakAbs = (zeroToPeak > 0) ? zeroToPeak : (0 - zeroToPeak);
	if(zeroToPeakAbs > p_motor->IPhasePeakTemp_Adcu) { p_motor->IPhasePeakTemp_Adcu = zeroToPeakAbs; }
}

void Motor_Analog_CaptureIa(Motor_T * p_motor)
{
	Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureIa, 0U/* Motor_SixStep_CaptureIa */);
	CaptureIZeroToPeak(p_motor, p_motor->Parameters.IaZeroRef_Adcu, p_motor->AnalogResults.Ia_Adcu);
}

void Motor_Analog_CaptureIb(Motor_T * p_motor)
{
	Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureIb, 0U/* Motor_SixStep_CaptureIb */);
	// CaptureIZeroToPeak(p_motor, p_motor->Parameters.IbZeroRef_Adcu, p_motor->AnalogResults.Ib_Adcu);
}

void Motor_Analog_CaptureIc(Motor_T * p_motor)
{
	Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureIc, 0U/* Motor_SixStep_CaptureIc */);
	// CaptureIZeroToPeak(p_motor, p_motor->Parameters.IcZeroRef_Adcu, p_motor->AnalogResults.Ic_Adcu);
	Debug_LED();
}


/******************************************************************************/
/*!
	@brief
*/
/******************************************************************************/

//static inline void Motor_Analog_Proc(Motor_T * p_motor)
//{
//	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_PWM);
//
//	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
//	{
//		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
//		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
//	}
////	switch(p_motor->AnalogCmd)
////	{
////		case FOC_I_ABC :
////			break;
////
////		case FOC_VBEMF :
////			break;
////
////		default :
////			break;
////	}
//
//	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_PWM);
//}