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
    @file 	Motor_Calibrate_.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CALIBRATE_H
#define MOTOR_CALIBRATE_H

#include "Motor.h"
#include "Motor_FOC.h"

/******************************************************************************/
/*
	Calibration State Functions - Mapped to StateMachine, Nonblocking
*/
/******************************************************************************/
static inline void Motor_Calibrate_StartHall(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Parameters.AlignTime_Cycles);
}

//120 degree, hall aligned with phase
static inline bool Motor_Calibrate_Hall(Motor_T * p_motor)
{
	const uint16_t duty = p_motor->Parameters.AlignVPwm_Frac16;
	bool isComplete = false;

	if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
	{
		switch (p_motor->CalibrationStateIndex)
		{
		case 0U:
			Hall_StartCalibrate(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, duty, 0U, 0U);
			p_motor->CalibrationStateIndex = 1U;
			break;

		case 1U:
			Hall_CalibratePhaseA(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, duty, duty, 0U);
			p_motor->CalibrationStateIndex = 2U;
			break;

		case 2U:
			Hall_CalibratePhaseInvC(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, 0U, duty, 0U);
			p_motor->CalibrationStateIndex = 3U;
			break;

		case 3U:
			Hall_CalibratePhaseB(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, 0U, duty, duty);
			p_motor->CalibrationStateIndex = 4U;
			break;

		case 4U:
			Hall_CalibratePhaseInvA(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, 0U, 0U, duty);
			p_motor->CalibrationStateIndex = 5U;
			break;

		case 5U:
			Hall_CalibratePhaseC(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, duty, 0U, duty);
			p_motor->CalibrationStateIndex = 6U;
			break;

		case 6U:
			Hall_CalibratePhaseInvB(&p_motor->Hall);
			Phase_Float(&p_motor->Phase);
			isComplete = true;
			break;

		default:
			break;
		}
	}

	return isComplete;
}

static inline void Motor_Calibrate_StartEncoder(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Parameters.AlignTime_Cycles);
}

static inline bool Motor_Calibrate_Encoder(Motor_T * p_motor)
{
	bool isComplete = false;

	if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
	{
//		switch (p_motor->CalibrationStateIndex)
//		{
		// Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVPwm_Frac16, 0U, 0U); /* Align Phase A 10% pwm */
//			case 0U:
//				Encoder_CalibrateQuadratureReference(&p_motor->Encoder);
//				Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVPwm_Frac16, p_motor->Parameters.AlignVPwm_Frac16, 0U);
//				p_motor->CalibrationStateIndex = 1U;
//				break;
//
//			case 1U:
//				Encoder_CalibrateQuadraturePositive(&p_motor->Encoder);
//				Phase_Float(&p_motor->Phase);
//				p_motor->CalibrationStateIndex = 0U;
//				isComplete = true;
//				break;
//
//			default:
//				break;
//		}
	}

	return isComplete;
}

/*
	Calibrate Current ADC
*/
static inline void Motor_Calibrate_StartAdc(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, GLOBAL_MOTOR.CONTROL_FREQ);

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_ActivateOutput(p_motor);
	}
	else if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP)
	{
		Phase_Ground(&p_motor->Phase);
	}
	Filter_InitAvg(&p_motor->FilterA);
	Filter_InitAvg(&p_motor->FilterB);
	Filter_InitAvg(&p_motor->FilterC);
	p_motor->AnalogResults.Ia_Adcu = 0U;
	p_motor->AnalogResults.Ib_Adcu = 0U;
	p_motor->AnalogResults.Ic_Adcu = 0U;
}

static inline bool Motor_Calibrate_Adc(Motor_T *p_motor)
{
	bool isComplete = Timer_Periodic_Poll(&p_motor->ControlTimer);

	if (isComplete == true)
	{
		p_motor->Parameters.IaZeroRef_Adcu = Filter_Avg(&p_motor->FilterA, p_motor->AnalogResults.Ia_Adcu);
		p_motor->Parameters.IbZeroRef_Adcu = Filter_Avg(&p_motor->FilterB, p_motor->AnalogResults.Ib_Adcu);
		p_motor->Parameters.IcZeroRef_Adcu = Filter_Avg(&p_motor->FilterC, p_motor->AnalogResults.Ic_Adcu);
		Motor_ResetUnitsIabc(p_motor);
		Phase_Float(&p_motor->Phase);
	}
	else
	{
		/* 4x sample time */
		if((p_motor->ControlTimerBase & 0b11UL) == 0UL)
		{
			if(p_motor->AnalogResults.Ia_Adcu != 0U) { Filter_Avg(&p_motor->FilterA, p_motor->AnalogResults.Ia_Adcu); }
			if(p_motor->AnalogResults.Ib_Adcu != 0U) { Filter_Avg(&p_motor->FilterB, p_motor->AnalogResults.Ib_Adcu); }
			if(p_motor->AnalogResults.Ic_Adcu != 0U) { Filter_Avg(&p_motor->FilterC, p_motor->AnalogResults.Ic_Adcu); }
		}
		else
		{
			AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
			AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
			AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
		#if defined(CONFIG_MOTOR_I_SENSORS_ABC)
			AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
		#endif
			AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
		}
	}

	return isComplete;
}


#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
static inline void Motor_Calibrate_StartSinCos(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Parameters.AlignTime_ControlCycles);
}

static inline bool Motor_Calibrate_SinCos(Motor_T * p_motor)
{
	bool isComplete = false;

	if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
	{
		switch (p_motor->CalibrationStateIndex)
		{
			case 0U:
				Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVPwm_Frac16, 0U, 0U);
				p_motor->CalibrationStateIndex = 2U;
				/* wait 1s */
				break;

			// case 1U:
			// 	//can repeat adc and filter results, or skip state use check in sensor routine
			// 	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
			// 	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
			// 	p_motor->CalibrationStateIndex = 2U;
			// 	/* wait 50us, 1s */
			// 	break;

			case 2U:
				SinCos_CalibrateA(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
				Phase_ActivateDuty(&p_motor->Phase, 0U, p_motor->Parameters.AlignVPwm_Frac16, 0U);
				p_motor->CalibrationStateIndex = 4U;
				/* wait 1s */
				break;

			// case 3U:
			// 	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
			// 	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
			// 	p_motor->CalibrationStateIndex = 4U;
			// 	break;

			case 4U:
				SinCos_CalibrateB(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
//				Phase_ActivateDuty(&p_motor->Phase, 0U, 0U, p_motor->Parameters.AlignVPwm_Frac16);
				p_motor->CalibrationStateIndex = 5U;
				// isComplete = true;
				Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVPwm_Frac16, 0U, 0U);
				break;

			case 5U:
				p_motor->SinCos.DebugAPostMech = SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
				p_motor->SinCos.DebugAPostElec = SinCos_GetElectricalAngle(&p_motor->SinCos);
				Phase_ActivateDuty(&p_motor->Phase, 0U, p_motor->Parameters.AlignVPwm_Frac16, 0U);
				p_motor->CalibrationStateIndex = 6U;
				break;

			case 6U:
				p_motor->SinCos.DebugBPostMech = SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
				p_motor->SinCos.DebugBPostElec = SinCos_GetElectricalAngle(&p_motor->SinCos);
				p_motor->CalibrationStateIndex = 0U;
				isComplete = true;
				break;
			default: break;
		}
	}

	return isComplete;
}
#endif

/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
