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

#ifndef CONFIG_HALL_DEBUG
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
#else
	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		p_motor->Debug[p_motor->CalibrationStateStep] = Hall_ReadSensors(&p_motor->Hall); //PhaseA starts at 1

		switch (p_motor->CalibrationStateStep)
		{
			case 0U :
				Phase_Polar_ActivateA(&p_motor->Phase, duty);
				break;

			case 1U :
				Hall_CalibratePhaseA(&p_motor->Hall);
				Phase_Polar_ActivateAC(&p_motor->Phase, duty);
				break;

			case 2U :
				Phase_Polar_ActivateInvC(&p_motor->Phase, duty);
				break;

			case 3U :
				Hall_CalibratePhaseInvC(&p_motor->Hall);
				Phase_Polar_ActivateBC(&p_motor->Phase, duty);
				break;

			case 4U :
				Phase_Polar_ActivateB(&p_motor->Phase, duty);
				break;

			case 5U :
				Hall_CalibratePhaseB(&p_motor->Hall);
				Phase_Polar_ActivateBA(&p_motor->Phase, duty);
				break;

			case 6U :
				Phase_Polar_ActivateInvA(&p_motor->Phase, duty);
				break;

			case 7U :
				Hall_CalibratePhaseInvA(&p_motor->Hall);
				Phase_Polar_ActivateCA(&p_motor->Phase, duty);
				break;

			case 8U :
				Phase_Polar_ActivateC(&p_motor->Phase, duty);
				break;

			case 9U :
				Hall_CalibratePhaseC(&p_motor->Hall);
				Phase_Polar_ActivateCB(&p_motor->Phase, duty);
				break;

			case 10U :
				Phase_Polar_ActivateInvB(&p_motor->Phase, duty);
				break;

			case 11U :
				Hall_CalibratePhaseInvB(&p_motor->Hall);
				Phase_Polar_ActivateAB(&p_motor->Phase, duty);
				break;

			case 12U :
				Phase_Float(&p_motor->Phase);
				isComplete = true;
				break;

			default :
				break;
		}

		p_motor->CalibrationStateStep++;
	}
#endif

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
//	FOC_SetZero(&p_motor->Foc);
//	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output

	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);

	p_motor->AnalogResults.Ia_ADCU = 2048U;
	p_motor->AnalogResults.Ib_ADCU = 2048U;
	p_motor->AnalogResults.Ic_ADCU = 2048U;

	Filter_MovAvg_InitN(&p_motor->FilterA, 2048U, 10U);
	Filter_MovAvg_InitN(&p_motor->FilterB, 2048U, 10U);
	Filter_MovAvg_InitN(&p_motor->FilterC, 2048U, 10U);
}

static inline bool Motor_Calibrate_Adc(Motor_T *p_motor)
{
	bool isComplete = Timer_Poll(&p_motor->ControlTimer);
	if (isComplete == true)
	{
//		Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_ADCU, p_motor->Parameters.IaRefZero_ADCU + p_motor->Parameters.IRefPeak_ADCU, p_motor->Parameters.IRefMax_Amp);
//		Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_ADCU, p_motor->Parameters.IbRefZero_ADCU + p_motor->Parameters.IRefPeak_ADCU, p_motor->Parameters.IRefMax_Amp);
//		Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_ADCU, p_motor->Parameters.IcRefZero_ADCU + p_motor->Parameters.IRefPeak_ADCU, p_motor->Parameters.IRefMax_Amp);
		Phase_Float(&p_motor->Phase);
//		save params
	}
	else
	{
		p_motor->Parameters.IaRefZero_ADCU = Filter_MovAvg_N(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
		p_motor->Parameters.IbRefZero_ADCU = Filter_MovAvg_N(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
		p_motor->Parameters.IcRefZero_ADCU = Filter_MovAvg_N(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);

		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
	}

	return isComplete;
}

/*
 * Calibrate Current ADC
 */
//static inline bool Motor_SixStep_CalibrateAdc(Motor_T *p_motor)
//{
////	bool isComplete = false;
//
////	 Timer_Poll(&p_motor->ControlTimer);
//
////	switch(p_motor->CalibrationSubstateStep)
////	{
////		case 0U :
////			Filter_MovAvg_InitN(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU, 1000U);
////			Filter_MovAvg_InitN(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU, 1000U);
////			Filter_MovAvg_InitN(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU, 1000U);
////			p_motor->CalibrationSubstateStep = 1U;
////			break;
////
////		case 1U:
////			if (Timer_Poll(&p_motor->ControlTimer) == false)
////			{
////				p_motor->Parameters.IaZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
////				p_motor->Parameters.IbZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
////				p_motor->Parameters.IcZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);
////
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
////			}
////			else
////			{
////				p_motor->CalibrationSubstateStep = 2U;
////			}
////			break;
////
////		case 2U:
////
////			if (Timer_Poll(&p_motor->ControlTimer) == true)
////			{
////				Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
////				Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
////				Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
////				p_motor->CalibrationSubstateStep = 4U;
////			}
////			else
////			{
////				p_motor->Parameters.IaZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
////				p_motor->Parameters.IbZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
////				p_motor->Parameters.IcZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);
////
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
////			}
////
////			//get bemf offset
//////		case 3U:
//////			Filter_MovAvg_InitN(&p_motor->FilterA, p_motor->AnalogResults.Va_ADCU, 1000U);
//////			Filter_MovAvg_InitN(&p_motor->FilterB, p_motor->AnalogResults.Vb_ADCU, 1000U);
//////			Filter_MovAvg_InitN(&p_motor->FilterC, p_motor->AnalogResults.Vc_ADCU, 1000U);
////
////		case 4U:
////
////			isComplete = true;
////
////
////		default :
////			break;
////	}
//
//	bool isComplete = Timer_Poll(&p_motor->ControlTimer);
//
//	if (isComplete == true)
//	{
//		Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
//		Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
//		Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
//		Phase_Float(&p_motor->Phase);
////		save params
//	}
//	else
//	{
//		p_motor->Parameters.IaZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
//		p_motor->Parameters.IbZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
//		p_motor->Parameters.IcZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);
//
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
//	}
//
//	return isComplete;
//}
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
