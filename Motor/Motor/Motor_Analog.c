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
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP) { Motor_SixStep_CaptureBemfA(p_motor); }
#endif
	CaptureVPeak(p_motor, p_motor->AnalogResults.Va_Adcu);
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
	// p_motor->DebugTimeABC[0] = SysTime_GetMicros() - p_motor->MicrosRef;
#endif
}

void Motor_Analog_CaptureVb(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP) { Motor_SixStep_CaptureBemfB(p_motor); }
#endif
	CaptureVPeak(p_motor, p_motor->AnalogResults.Vb_Adcu);
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
	// p_motor->DebugTimeABC[1] = SysTime_GetMicros() - p_motor->MicrosRef;
#endif
}

void Motor_Analog_CaptureVc(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP) { Motor_SixStep_CaptureBemfC(p_motor); }
#endif
	CaptureVPeak(p_motor, p_motor->AnalogResults.Vc_Adcu);
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
	p_motor->DebugTimeABC[2] = SysTime_GetMicros() - p_motor->MicrosRef;
#endif
}

/******************************************************************************/
/*!
	@brief  Iabc
*/
/******************************************************************************/
static inline void CaptureIZeroToPeak(Motor_T * p_motor, uint16_t adcuZero, uint16_t adcu)
{
	int16_t signedAdcu = adcu - adcuZero;
	uint16_t zeroToPeak = (signedAdcu > 0) ? signedAdcu : 0 - signedAdcu;
	if(zeroToPeak > p_motor->IPhasePeakTemp_Adcu) { p_motor->IPhasePeakTemp_Adcu = zeroToPeak; }
}

void Motor_Analog_CaptureIa(Motor_T * p_motor)
{
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_CaptureIa(p_motor); }
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	else /* (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP) */
	{ Motor_SixStep_CaptureIa(p_motor); }
#endif
	CaptureIZeroToPeak(p_motor, p_motor->Parameters.IaZeroRef_Adcu, p_motor->AnalogResults.Ia_Adcu);
}

void Motor_Analog_CaptureIb(Motor_T * p_motor)
{
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_CaptureIb(p_motor); }
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	else /* (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP) */
	{ Motor_SixStep_CaptureIb(p_motor); }
#endif
	CaptureIZeroToPeak(p_motor, p_motor->Parameters.IbZeroRef_Adcu, p_motor->AnalogResults.Ib_Adcu);
}

void Motor_Analog_CaptureIc(Motor_T * p_motor)
{
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_CaptureIc(p_motor); }
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	else /* (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP) */
	{ Motor_SixStep_CaptureIc(p_motor); }
#endif
	CaptureIZeroToPeak(p_motor, p_motor->Parameters.IcZeroRef_Adcu, p_motor->AnalogResults.Ic_Adcu);
}


/******************************************************************************/
/*!
	@brief
*/
/******************************************************************************/

//void Motor_CapturePwmOnFlag(Motor_T * p_motor)
//{
//	p_motor->IsPwmOn = true;
//}

/*
 * Sets options only
 */
 //const Analog_Options_T MOTOR_ANALOG_OPTIONS_PWM_ON =
 //{
 //	.OPTIONS = {.IsValid = 1U, .HwTriggerConversion = 1U, },
 //	.ON_OPTIONS = SetPwmOnFlag,
 //};
 //
 //const Analog_Options_T MOTOR_ANALOG_OPTIONS_RESTORE =
 //{
 //	.OPTIONS = {.IsValid = 1U, .HwTriggerConversion = 0U, },
 //};

static inline void EnqueueVabc(Motor_T * p_motor)
{
	//no current sense during pwm float, check bemf
#if  defined(CONFIG_MOTOR_V_SENSORS_ADC) && !defined(CONFIG_MOTOR_V_SENSORS_ISOLATED)
	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
#else
	(void)p_motor;
#endif
}

//static inline void Motor_Analog_Proc(Motor_T * p_motor)
//{
//	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_PWM);
//
//	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
//	{
//		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
//		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
//	}
//
////	switch(p_motor->AnalogCmd)
////	{
////		case FOC_I_ABC :
////			AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
////			AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
////#if defined(CONFIG_MOTOR_I_SENSORS_ABC) && !defined(CONFIG_MOTOR_I_SENSORS_AB)
////			AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
////#endif
////			break;
////
////		case FOC_BEMF :
////#if !defined(CONFIG_MOTOR_V_SENSORS_ISOLATED) &&defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
////			AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
////			AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
////			AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
////#endif
////			break;
////
////		default :
////			break;
////	}
//
//	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_PWM);
//}