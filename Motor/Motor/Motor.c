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
    @file 	Motor.c
    @author FireSoucery
    @brief  Motor module conventional function definitions.
    @version V0
*/
/******************************************************************************/
#include "Motor.h"
#include "Config.h"
#include "MotorAnalog.h"

#include "Transducer/Phase/Phase.h"
#include "Transducer/Hall/Hall.h"
//#include "Transducer/BEMF/BEMF.h"
#include "Math/FOC.h"

#include "Transducer/Encoder/Encoder_Motor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Timer/Timer.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

void Motor_Init(Motor_T * p_motor)
{
	if (p_motor->CONFIG.P_PARAMS_NVM != 0U)
	{
		memcpy(&p_motor->Parameters, p_motor->CONFIG.P_PARAMS_NVM, sizeof(Motor_Params_T));
	}

	StateMachine_Init(&p_motor->StateMachine);
	Motor_InitReboot(p_motor);
}


//void Motor_LoadDefault(Motor_T * p_motor)
//{
//	if (p_motor->CONFIG.P_PARAMS_DEFAULT != 0U)
//	{
//		memcpy(&p_motor->Parameters, p_motor->CONFIG.P_PARAMS_DEFAULT, sizeof(Motor_Params_T));
//	}
//
//	Hall_LoadDefault(&p_motor->Hall);
////	Encoder_LoadDefault(&p_motor->Encoder);
////	PID_LoadDefault(&p_motor->PidSpeed);
////	PID_LoadDefault(&p_motor->PidIq);
////	PID_LoadDefault(&p_motor->PidId);
////	PID_LoadDefault(&p_motor->PidIBus);
////	Thermistor_LoadDefault(&p_motor->Thermistor);
//
//	Motor_InitReboot(p_motor);
//}



//todo state machine init-state run
void Motor_InitReboot(Motor_T * p_motor)
{
	/*
	 * HW Wrappers Init
	 */
	Phase_Init(&p_motor->Phase);
	Phase_Polar_ActivateMode(&p_motor->Phase, p_motor->Parameters.PhasePwmMode);

	if ((p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP) || (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL))
	{
		/*
		 * all sixstep modes and hall foc mode use CaptureTime
		 */
		Encoder_Motor_InitCaptureTime(&p_motor->Encoder);
	}
	else
	{
		Encoder_Motor_InitCaptureCount(&p_motor->Encoder);
		AnalogN_EnqueueConversionOptions(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_OPTION_RESTORE);
	}

	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		Hall_Init(&p_motor->Hall);
	}

	/*
	 * SW Structs
	 */
	FOC_Init(&p_motor->Foc);
//	BEMF_Init(&p_motor->Bemf);

	PID_Init(&p_motor->PidSpeed);

	PID_Init(&p_motor->PidIq);
	PID_Init(&p_motor->PidId);
	PID_Init(&p_motor->PidIBus);

	Timer_InitPeriodic(&p_motor->ControlTimer, 	1U);
	Timer_InitPeriodic(&p_motor->MillisTimer, 	1U);
	Timer_InitPeriodic(&p_motor->SpeedTimer, 	1U);

	/*
	 * Initial runtime config settings
	 */
	/*
	 * Run calibration later, default zero to middle adc
	 */
	//scales 4095 to physical units. alternatively use opamp equation
#ifdef CONFIG_MOTOR_CURRENT_SAMPLE_INVERT
	Linear_ADC_Init_Inverted(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_ADCU, p_motor->Parameters.IaRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init_Inverted(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_ADCU, p_motor->Parameters.IbRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init_Inverted(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_ADCU, p_motor->Parameters.IcRefMax_ADCU, p_motor->Parameters.IRefMax_Amp);
#elif defined(CONFIG_MOTOR_CURRENT_SAMPLE_NONINVERT)
	Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_ADCU, p_motor->Parameters.IaRefMax_ADCU,  p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_ADCU, p_motor->Parameters.IbRefMax_ADCU,  p_motor->Parameters.IRefMax_Amp);
	Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_ADCU, p_motor->Parameters.IcRefMax_ADCU,  p_motor->Parameters.IRefMax_Amp);
#endif

	//Linear_Init(&(p_Motor->VFMap), vPerRPM, 1, vOffset); //f(freq) = voltage

	/*
	 * Index max is 13,107
	 */
	Linear_Ramp_InitMillis(&p_motor->Ramp, 10U, 20000U, 0U, 1000U); /* final value is overwritten, slope is persistent */
	p_motor->RampCmd = 0;
	p_motor->RampIndex = 0;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP)
	{

	}
	else
	{
		//can start at 0 speed in foc mode for continuous angle displacements
		Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 2000U, 20000U, 0U, 300U);
	}
	p_motor->OpenLoopRampIndex = 0U;

	Motor_SetDirectionForward(p_motor);
	p_motor->UserDirection = p_motor->Direction;
//	p_motor->Direction 				= MOTOR_DIRECTION_CCW;

	p_motor->Speed_RPM 				= 0U;
	p_motor->VPwm 					= 0U;
	p_motor->ControlTimerBase 		= 0U;
}

// void Motor_SetControlMode(Motor_T * p_motor, Motor_ControlMode_T mode)
//{
//	switch(mode)
//	{
//		case MOTOR_CONTROL_MODE_OPEN_LOOP:
//			p_motor->ControlMode.OpenLoop = 1U;
//			p_motor->ControlMode.Current = 0U;
//			break;
//
//		case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
//			p_motor->ControlMode.Speed = 0U;
//			p_motor->ControlMode.Current = 0U;
//			break;
//
//	//	MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ: break;
//
//		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
//			p_motor->ControlMode.Speed = 1U;
//			p_motor->ControlMode.Current = 0U;
//			break;
//
//		case MOTOR_CONTROL_MODE_CONSTANT_CURRENT:
//			p_motor->ControlMode.Speed = 0U;
//			p_motor->ControlMode.Current = 1U;
//			break;
//
//		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT:
//			p_motor->ControlMode.Speed = 1U;
//			p_motor->ControlMode.Current = 1U;
//			break;
//	}
//}


/******************************************************************************/
/*
 *	Align State
 */
/******************************************************************************/
Motor_AlignMode_T Motor_GetAlignMode(Motor_T *p_motor)
{
	Motor_AlignMode_T alignMode;

	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		alignMode = MOTOR_ALIGN_MODE_DISABLE;
	}
	else
	{
		//		if useHFI alignMode= MOTOR_ALIGN_MODE_HFI;
		//		else
		alignMode = MOTOR_ALIGN_MODE_ALIGN;
	}

	return alignMode;
}

void Motor_StartAlign(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Parameters.AlignTime_ControlCycles);
	Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16, 0U, 0U);
	Phase_ActivateSwitchABC(&p_motor->Phase);
}

bool Motor_ProcAlign(Motor_T * p_motor)
{
	bool status = Timer_Poll(&p_motor->ControlTimer);

	if(status == true)
	{
		p_motor->ElectricalAngle = 0U;
		Encoder_Reset(&p_motor->Encoder); //zero angularD
//		Motor_Float(&p_motor->Foc);
	}

	return status;
}

/******************************************************************************/
/*
 * 	Calibration State Functions
 * 	@{
 */
/******************************************************************************/
/*
 * Nonblocking Calibration State Functions
 */
void Motor_SetCalibrationStateAdc(Motor_T * p_motor)		{p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_ADC;}
void Motor_SetCalibrationStateHall(Motor_T * p_motor)		{p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_HALL;}
void Motor_SetCalibrationStateEncoder(Motor_T * p_motor)	{p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_ENCODER;}

static void StartMotorCalibrateCommon(Motor_T * p_motor)
{
	p_motor->ControlTimerBase = 0U;
	Phase_Ground(&p_motor->Phase); //activates abc
	p_motor->CalibrationSubstateStep = 0U;
}

/*
 * Calibrate Current ADC
 */
void Motor_StartCalibrateAdc(Motor_T * p_motor)
{
	StartMotorCalibrateCommon(p_motor);
	Timer_StartPeriod(&p_motor->ControlTimer, 100000U); // Motor.Parameters.AdcCalibrationTime
//	FOC_SetZero(&p_motor->Foc);
//	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output

//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VA);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VB);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VC);

	p_motor->AnalogResults.Ia_ADCU = 2048U;
	p_motor->AnalogResults.Ib_ADCU = 2048U;
	p_motor->AnalogResults.Ic_ADCU = 2048U;

	Filter_MovAvg_InitN(&p_motor->FilterA, 2048U, 40U);
	Filter_MovAvg_InitN(&p_motor->FilterB, 2048U, 40U);
	Filter_MovAvg_InitN(&p_motor->FilterC, 2048U, 40U);
}

bool Motor_CalibrateAdc(Motor_T *p_motor)
{
	bool isComplete = Timer_Poll(&p_motor->ControlTimer);

	if (isComplete == true)
	{
		Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_ADCU, 4095U, p_motor->Parameters.IRefMax_Amp);
		Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_ADCU, 4095U, p_motor->Parameters.IRefMax_Amp);
		Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_ADCU, 4095U, p_motor->Parameters.IRefMax_Amp);
		Phase_Float(&p_motor->Phase);
//		save params
	}
	else
	{
		p_motor->Parameters.IaRefZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
		p_motor->Parameters.IbRefZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
		p_motor->Parameters.IcRefZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);

		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
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
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
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
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
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
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VA);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VB);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VC);
//	}
//
//	return isComplete;
//}


void Motor_StartCalibrateEncoder(Motor_T * p_motor)
{
	StartMotorCalibrateCommon(p_motor);
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U);
	Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16, 0U, 0U); /* Align Phase A 10% pwm */

}

bool Motor_CalibrateEncoder(Motor_T * p_motor)
{
	bool isComplete = false;

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		switch (p_motor->CalibrationSubstateStep)
		{
			case 0U:
				Encoder_DeltaD_CalibrateQuadratureReference(&p_motor->Encoder);

				Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16, p_motor->Parameters.AlignVoltage_Frac16, 0U);
				p_motor->CalibrationSubstateStep = 1U;
				break;

			case 1U:
				Encoder_DeltaD_CalibrateQuadraturePositive(&p_motor->Encoder);
				Phase_Float(&p_motor->Phase);
				p_motor->CalibrationSubstateStep = 0;
				isComplete = true;
				break;

			default:
				break;
		}
	}

	return isComplete;
}

void Motor_StartCalibrateHall(Motor_T * p_motor)
{
	StartMotorCalibrateCommon(p_motor);
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U); //Parameter.HallCalibrationTime
}

//120 degree hall aligned with phase
bool Motor_CalibrateHall(Motor_T * p_motor)
{
	const uint16_t duty = p_motor->Parameters.AlignVoltage_Frac16;
	bool isComplete = false;

//	if (Timer_Poll(&p_motor->ControlTimer) == true)
//	{
//		switch (p_motor->CalibrationSubstateStep)
//		{
//		case 0U:
//			Phase_ActivateDuty(&p_motor->Phase, duty, 0U, 0U);
//			p_motor->CalibrationSubstateStep = 1U;
//			break;
//
//		case 1U:
//			Hall_CalibratePhaseA(&p_motor->Hall);
//			Phase_ActivateDuty(&p_motor->Phase, duty, duty, 0U);
//			p_motor->CalibrationSubstateStep = 2U;
//			break;
//
//		case 2U:
//			Hall_CalibratePhaseInvC(&p_motor->Hall);
//			Phase_ActivateDuty(&p_motor->Phase, 0U, duty, 0);
//			p_motor->CalibrationSubstateStep = 3U;
//			break;
//
//		case 3U:
//			Hall_CalibratePhaseB(&p_motor->Hall);
//			Phase_ActivateDuty(&p_motor->Phase, 0U, duty, duty);
//			p_motor->CalibrationSubstateStep = 4U;
//			break;
//
//		case 4U:
//			Hall_CalibratePhaseInvA(&p_motor->Hall);
//			Phase_ActivateDuty(&p_motor->Phase, 0U, 0U, duty);
//			p_motor->CalibrationSubstateStep = 5U;
//			break;
//
//		case 5U:
//			Hall_CalibratePhaseC(&p_motor->Hall);
//			Phase_ActivateDuty(&p_motor->Phase, duty, 0U, duty);
//			p_motor->CalibrationSubstateStep = 6U;
//			break;
//
//		case 6U:
//			Hall_CalibratePhaseInvB(&p_motor->Hall);
//			Phase_Float(&p_motor->Phase);
//			isComplete = true;
//			break;
//
//		default:
//			break;
//		}
//	}

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		p_motor->HallDebug[p_motor->CalibrationSubstateStep] = Hall_ReadSensors(&p_motor->Hall); //PhaseA starts at 1

		switch (p_motor->CalibrationSubstateStep)
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

		p_motor->CalibrationSubstateStep++;
	}




	return isComplete;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

