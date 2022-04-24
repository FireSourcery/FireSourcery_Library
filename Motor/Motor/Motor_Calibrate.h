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
    @file 	Motor_Calibrate_.h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CALIBRATE_H
#define MOTOR_CALIBRATE_H

#include "Motor.h"

/******************************************************************************/
/*
	Calibration State Functions - Mapped to StateMachine, Nonblocking
 */
/******************************************************************************/
static inline void Motor_Calibrate_StartSinCos(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Parameters.AlignTime_ControlCycles);
}

static inline bool Motor_Calibrate_ProcSinCos(Motor_T * p_motor)
{
	bool isComplete = false;

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		switch (p_motor->CalibrationStateStep)
		{
			case 0U:
				Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16, 0U, 0U); /* Align Phase A 10% pwm */
				p_motor->CalibrationStateStep = 1U;
				/* wait 1s */
				break;

			case 1U:
				//can repeat adc and filter results
				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
				p_motor->CalibrationStateStep = 2U;
				/* wait 50us, 1s */
				break;

			case 2U:
				p_motor->Debug[9U] = SinCos_CalcAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_ADCU, p_motor->AnalogResults.Cos_ADCU);
				SinCos_CalibrateA(&p_motor->SinCos, p_motor->AnalogResults.Sin_ADCU, p_motor->AnalogResults.Cos_ADCU);
				p_motor->Debug[0U] = SinCos_CalcAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_ADCU, p_motor->AnalogResults.Cos_ADCU);
				p_motor->Debug[3U] = p_motor->AnalogResults.Sin_ADCU;
				p_motor->Debug[4U] = p_motor->AnalogResults.Cos_ADCU;
				Phase_ActivateDuty(&p_motor->Phase, 0U, p_motor->Parameters.AlignVoltage_Frac16, 0U);
				p_motor->CalibrationStateStep = 3U;
				/* wait 1s */
				break;

			case 3U:
				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
				p_motor->CalibrationStateStep = 4U;
				break;

			case 4U:
				SinCos_CalibrateB(&p_motor->SinCos, p_motor->AnalogResults.Sin_ADCU, p_motor->AnalogResults.Cos_ADCU);
				p_motor->Debug[1U] = SinCos_CalcAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_ADCU, p_motor->AnalogResults.Cos_ADCU);
				p_motor->Debug[5U] = p_motor->AnalogResults.Sin_ADCU;
				p_motor->Debug[6U] = p_motor->AnalogResults.Cos_ADCU;
				Phase_ActivateDuty(&p_motor->Phase, 0U, 0U, p_motor->Parameters.AlignVoltage_Frac16);
				p_motor->CalibrationStateStep = 5U;
				break;

			case 5U:
				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
				p_motor->CalibrationStateStep = 6U;
				break;

			case 6U:
				p_motor->Debug[2U] = SinCos_CalcAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_ADCU, p_motor->AnalogResults.Cos_ADCU);
				p_motor->Debug[7U] = p_motor->AnalogResults.Sin_ADCU;
				p_motor->Debug[8U] = p_motor->AnalogResults.Cos_ADCU;
				p_motor->CalibrationStateStep = 0U;
				isComplete = true;
				break;
		}
	}

	return isComplete;
}

static inline void Motor_Calibrate_StartHall(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Parameters.AlignTime_ControlCycles);
}

//120 degree, hall aligned with phase
static inline bool Motor_Calibrate_Hall(Motor_T * p_motor)
{
	const uint16_t duty = p_motor->Parameters.AlignVoltage_Frac16;
	bool isComplete = false;

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		switch (p_motor->CalibrationStateStep)
		{
		case 0U:
			Phase_ActivateDuty(&p_motor->Phase, duty, 0U, 0U);
			p_motor->CalibrationStateStep = 1U;
			break;

		case 1U:
			Hall_CalibratePhaseA(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, duty, duty, 0U);
			p_motor->CalibrationStateStep = 2U;
			break;

		case 2U:
			Hall_CalibratePhaseInvC(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, 0U, duty, 0U);
			p_motor->CalibrationStateStep = 3U;
			break;

		case 3U:
			Hall_CalibratePhaseB(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, 0U, duty, duty);
			p_motor->CalibrationStateStep = 4U;
			break;

		case 4U:
			Hall_CalibratePhaseInvA(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, 0U, 0U, duty);
			p_motor->CalibrationStateStep = 5U;
			break;

		case 5U:
			Hall_CalibratePhaseC(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, duty, 0U, duty);
			p_motor->CalibrationStateStep = 6U;
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
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Parameters.AlignTime_ControlCycles);
	Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16, 0U, 0U); /* Align Phase A 10% pwm */
}

static inline bool Motor_Calibrate_Encoder(Motor_T * p_motor)
{
	bool isComplete = false;

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
//		switch (p_motor->CalibrationStateStep)
//		{
//			case 0U:
//				Encoder_DeltaD_CalibrateQuadratureReference(&p_motor->Encoder);
//				Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16, p_motor->Parameters.AlignVoltage_Frac16, 0U);
//				p_motor->CalibrationStateStep = 1U;
//				break;
//
//			case 1U:
//				Encoder_DeltaD_CalibrateQuadraturePositive(&p_motor->Encoder);
//				Phase_Float(&p_motor->Phase);
//				p_motor->CalibrationStateStep = 0U;
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
 * Calibrate Current ADC
 */
static inline void Motor_Calibrate_StartAdc(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U); // Motor.Parameters.AdcCalibrationTime

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		FOC_Zero(&p_motor->Foc);
		Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
	}
	else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
	{
		Phase_Ground(&p_motor->Phase);
	}

	p_motor->AnalogResults.Ia_ADCU = 2048U;
	p_motor->AnalogResults.Ib_ADCU = 2048U;
	p_motor->AnalogResults.Ic_ADCU = 2048U;

	Filter_InitAvg(&p_motor->FilterA);
	Filter_InitAvg(&p_motor->FilterB);
	Filter_InitAvg(&p_motor->FilterC);
}

static inline bool Motor_Calibrate_Adc(Motor_T *p_motor)
{
	bool isComplete = Timer_Poll(&p_motor->ControlTimer);
	if (isComplete == true)
	{
		p_motor->Parameters.IaRefZero_ADCU = Filter_Avg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
		p_motor->Parameters.IbRefZero_ADCU = Filter_Avg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
		p_motor->Parameters.IcRefZero_ADCU = Filter_Avg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);
		p_motor->Parameters.IaRefMax_ADCU = p_motor->Parameters.IaRefZero_ADCU + p_motor->Parameters.IRefZeroToPeak_ADCU;
		p_motor->Parameters.IbRefMax_ADCU = p_motor->Parameters.IbRefZero_ADCU + p_motor->Parameters.IRefZeroToPeak_ADCU;
		p_motor->Parameters.IcRefMax_ADCU = p_motor->Parameters.IcRefZero_ADCU + p_motor->Parameters.IRefZeroToPeak_ADCU;
		Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_ADCU, p_motor->Parameters.IaRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
		Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_ADCU, p_motor->Parameters.IbRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
		Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_ADCU, p_motor->Parameters.IcRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
		Phase_Float(&p_motor->Phase);
	}
	else
	{
		Filter_Avg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
		Filter_Avg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
		Filter_Avg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
	}

	return isComplete;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
