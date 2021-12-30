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
#include "Transducer/BEMF/BEMF.h"
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
	if (p_motor->CONFIG.P_PARAMETERS != 0U)
	{
		memcpy(&p_motor->Parameters, p_motor->CONFIG.P_PARAMETERS, sizeof(p_motor->Parameters));
	}

	StateMachine_Init(&p_motor->StateMachine);
	Motor_InitReboot(p_motor);
}

//todo state machine init-state run
void Motor_InitReboot(Motor_T * p_motor)
{
	/*
	 * HW Wrappers Init
	 */
	Phase_Init(&p_motor->Phase);
	Phase_Polar_ActivateMode(&p_motor->Phase, p_motor->Parameters.PhasePwmMode);

	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP || p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		/*
		 * all sixstep modes and hall foc mode use CaptureTime
		 * use PolePairs * 6 for count per commutation or PolePairs for count per erotation
		 */
		Encoder_Motor_InitCaptureTime(&p_motor->Encoder, p_motor->Parameters.PolePairs * 6U, p_motor->Parameters.EncoderDistancePerCount, p_motor->Parameters.PolePairs);
		Hall_Init(&p_motor->Hall);
	}
	else if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_ENCODER)
	{
		Encoder_Motor_InitCaptureCount(&p_motor->Encoder, p_motor->Parameters.EncoderCountsPerRevolution, p_motor->Parameters.EncoderDistancePerCount, p_motor->Parameters.PolePairs);
		Encoder_SetQuadratureMode(&p_motor->Encoder, p_motor->Parameters.EncoderIsQuadratureModeEnabled);
		Encoder_SetQuadratureDirectionCalibration(&p_motor->Encoder, p_motor->Parameters.EncoderIsALeadBPositive);
		FOC_Init(&p_motor->Foc);
	}
	else
	{
		//FOC sensorless
	}

	/*
	 * SW Structs
	 */
	BEMF_Init(&p_motor->Bemf);
//	p_motor->p_BemfConversionActive = &p_motor->CONFIG.CONVERSION_BEMF_A;


	/*
	 * Timer only mode, no function attached
	 */
	Timer_InitPeriodic(&p_motor->ControlTimer, 	1U);
	Timer_InitPeriodic(&p_motor->MillisTimer, 	1U);
//	Timer_InitPeriodic(&p_motor->SecondsTimer, 	1000U);

	/*
	 * Initial runtime config settings
	 */


	/*
	 * Run calibration later, default zero to middle adc
	 */
	Linear_ADC_Init(&p_motor->UnitIa, 2048U, 4095U, p_motor->Parameters.Imax_Amp); //scales 4095 to physical units. alternatively use opamp equation
	Linear_ADC_Init(&p_motor->UnitIb, 2048U, 4095U, p_motor->Parameters.Imax_Amp);
	Linear_ADC_Init(&p_motor->UnitIc, 2048U, 4095U, p_motor->Parameters.Imax_Amp);

	//Linear_Init(&(p_Motor->VFMap), vPerRPM, 1, vOffset); //f(freq) = voltage

	p_motor->Direction 				= MOTOR_DIRECTION_CCW;
//	p_motor->DirectionInput 		= MOTOR_DIRECTION_CCW;
	p_motor->Speed_RPM 				= 0U;
	p_motor->VPwm 					= 0U;
	p_motor->ControlTimerBase 		= 0U;

 	p_motor->SignalBufferBemfA.AdcFlags 		= 0U;
//	p_motor->SignalBufferBemfARepeat.AdcFlags 	= 0U;
	p_motor->SignalBufferBemfB.AdcFlags 		= 0U;
	p_motor->SignalBufferBemfC.AdcFlags 		= 0U;
	p_motor->SignalBufferRemainder.AdcFlags 	= 0U;
	p_motor->SignalBufferFocIabc.AdcFlags 		= 0U;
	p_motor->SignalBufferFocRemainder.AdcFlags 	= 0U;
	p_motor->SignalBufferIdle.AdcFlags 			= 0U;

}

/******************************************************************************/
/*
 *	Analog Capture
 */
/******************************************************************************/
void Motor_CaptureIMotorA(Motor_T * p_motor)
{
	//Filter here if needed
	p_motor->IMotor_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIa,  p_motor->AnalogResults.Ia_ADCU);
//	p_motor->IBus_ADCU  Filter_MovAvg(&p_motor->FilterIa, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IA]);
}

void Motor_CaptureIMotorB(Motor_T * p_motor)
{
	p_motor->IMotor_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIb, p_motor->AnalogResults.Ib_ADCU);
}

void Motor_CaptureIMotorC(Motor_T * p_motor)
{
	p_motor->IMotor_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIc, p_motor->AnalogResults.Ic_ADCU);
}

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
	uint32_t alignVoltage = (65536U/10U/4U); // + (p_motor->UserCmd / 2U); //= (65536U/10U/4U) + (p_motor->VPwm / 2U);
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U); //Parameter.AlignTime
	Phase_Ground(&p_motor->Phase);
	Phase_ActuateDuty(&p_motor->Phase, alignVoltage, 0, 0);
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
	Phase_Ground(&p_motor->Phase);
}

void Motor_StartCalibrateAdc(Motor_T * p_motor)
{
	StartMotorCalibrateCommon(p_motor);
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U);//Motor.Parameters.AdcCalibrationTime
}

bool Motor_CalibrateAdc(Motor_T *p_motor)
{
	bool isComplete;
	//todo movavg repeat
//	p_motor->ZeroIa = filter_movavgn(p_motor->ZeroIa, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_IA], p_motor->CurrentFilterCoeffcientN);
//	p_motor->ZeroIb = filter_movavgn(p_motor->ZeroIb, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_IB], p_motor->CurrentFilterCoeffcientN);
//	p_motor->ZeroIc = filter_movavgn(p_motor->ZeroIc, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_IC], p_motor->CurrentFilterCoeffcientN);

//	p_motor->ZeroIa = Filter_MovAvg(&p_motor->FilterIa, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_IA]);
//	p_motor->ZeroIb = Filter_MovAvg(&p_motor->FilterIb, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_IB]);
//	p_motor->ZeroIc = Filter_MovAvg(&p_motor->FilterIc, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_IC]);

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		Linear_ADC_Init(&p_motor->UnitIa, p_motor->AnalogResults.Ia_ADCU, 4095U, p_motor->Parameters.Imax_Amp); //temp 120amp
		Linear_ADC_Init(&p_motor->UnitIb, p_motor->AnalogResults.Ib_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
		Linear_ADC_Init(&p_motor->UnitIc, p_motor->AnalogResults.Ic_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
//		Linear_ADC_Init(&p_motor->UnitIc, Filter_MovAvg_GetResult(&p_motor->FilterIc), 4095U, p_motor->Parameters.Imax_Amp);

		Phase_Float(&p_motor->Phase);
		isComplete = true;
	}
	else
	{
		isComplete = false;
	}

	return isComplete;
}


void Motor_StartCalibrateEncoder(Motor_T * p_motor)
{
	StartMotorCalibrateCommon(p_motor);
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U);
	Phase_ActuateDuty(&p_motor->Phase, 6553U/4U, 0, 0); /* Align Phase A 10% pwm */
}

bool Motor_CalibrateEncoder(Motor_T * p_motor)
{
	static uint8_t state = 0; //limits calibration to 1 at a time;
	const uint16_t duty = 65536/10/4;
	bool isComplete = false;

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		switch (state)
		{
			case 0U:
	//			Encoder_DeltaD_CalibrateQuadratureReference(&p_motor->Encoder);
				Phase_ActuateDuty(&p_motor->Phase, 0U, duty, 0U);
				state++;
				break;

			case 1U:
	//			Encoder_DeltaD_CalibrateQuadratureDirectionPositive(&p_motor->Encoder);
				Phase_Float(&p_motor->Phase);
				state = 0;
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
	static uint8_t state = 0U; //limits calibration to 1 at a time; todo reuse hall sate
	const uint16_t duty = 65536U/10U/4U;
	bool isComplete = false;

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		switch (state)
		{
		case 0U:
			Phase_ActuateDuty(&p_motor->Phase, duty, 0U, 0U);
			state++;
			break;

		case 1U:
			Hall_CalibratePhaseA(&p_motor->Hall);
			Phase_ActuateDuty(&p_motor->Phase, duty, duty, 0U);
			state++;
			break;

		case 2U:
			Hall_CalibratePhaseInvC(&p_motor->Hall);
			Phase_ActuateDuty(&p_motor->Phase, 0U, duty, 0);
			state++;
			break;

		case 3U:
			Hall_CalibratePhaseB(&p_motor->Hall);
			Phase_ActuateDuty(&p_motor->Phase, 0U, duty, duty);
			state++;
			break;

		case 4U:
			Hall_CalibratePhaseInvA(&p_motor->Hall);
			Phase_ActuateDuty(&p_motor->Phase, 0U, 0U, duty);
			state++;
			break;

		case 5U:
			Hall_CalibratePhaseC(&p_motor->Hall);
			Phase_ActuateDuty(&p_motor->Phase, duty, 0U, duty);
			state++;
			break;

		case 6U:
			Hall_CalibratePhaseInvB(&p_motor->Hall);
			Phase_Float(&p_motor->Phase);
			state = 0U;
			isComplete = true;
			break;

		default:
			break;
		}
	}

	return isComplete;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*
 * Control Variable Functions
 * @{
 */
/******************************************************************************/
//void Motor_SetUserCmd(Motor_T * p_motor, uint16_t userCommand)
//{
//	p_motor->UserCmdPrev = p_motor->UserCmd;
//	p_motor->UserCmd = userCommand;
//}
/*
 * Input Ramp
 */
void Motor_ProcRamp(Motor_T * p_motor) // input voltage/speed cmd
{
	if (p_motor->RampCmd != Linear_Ramp_GetFinal(&p_motor->Ramp))
	{
		p_motor->RampCmd = Linear_Ramp_CalcTargetIncIndex(&p_motor->Ramp, &p_motor->RampIndex, 1U);
	}
}

void Motor_ResetRampCmd(Motor_T * p_motor) // input voltage/speed cmd
{
	p_motor->RampCmd = 0U;
}

/*
 * userCmd is acceleration
 */
void Motor_SetRampUp(Motor_T * p_motor, uint16_t userCmd)
{
//	if (userCmd != p_motor->RampCmd)
	{
		Linear_Ramp_Init_Millis(&p_motor->Ramp, 20000U, p_motor->RampCmd, userCmd, 10U);
		p_motor->RampIndex = 0;
	}
//	Linear_Ramp_Init_Acceleration(&p_motor->Ramp, 20000U, p_motor->RampCmd, userCmd, userCmd); //ramp up only
}

/*
 * userCmd is deceleration
 */
void Motor_SetRampDown(Motor_T * p_motor, uint16_t userCmd)
{
//	if (0U != p_motor->RampCmd)
	{
		//Decel by Brake per 1s
		Linear_Ramp_Init_Acceleration(&p_motor->Ramp, 20000U, p_motor->RampCmd, 0U, -(int32_t)userCmd);
		p_motor->RampIndex = 0;
	}
}

void Motor_SetRamp(Motor_T * p_motor, int32_t accelerationSigned)
{

}

/*
 * Speed PID Feedback Loop
 */
void Motor_ProcSpeedFeedbackLoop(Motor_T * p_motor)
{
//	p_motor->Speed_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
	//	//p_motor->Speed_RPM = Filter_MovAvg(p_motor->Speed_RPM, coef, coef);
	//	PID_Proc(&p_motor->PidSpeed, p_motor->Speed_RPM, setpoint);
}

void Motor_PollSpeedFeedbackLoop(Motor_T * p_motor)
{
	if (Timer_Poll(&p_motor->MillisTimer) == true)
	{
		Motor_ProcSpeedFeedbackLoop(p_motor);
	}
}

//split motor state version and controvar verions
void Motor_ProcControlVariable(Motor_T * p_motor)
{
	uint32_t debug;

//	if (p_motor->Parameters.ControlMode  == MOTOR_CONTROL_MODE_OPEN_LOOP)
//	{
//		p_motor->VPwm =  65536U/20U;
//
////		p_motor->VPwm =  (p_motor->UserCmd / 2U);
////
////		// bound to 2.5 to 10 percent
////		if (p_motor->VPwm < (65536U/10U/4U))
////		{
////			p_motor->VPwm = (65536U/10U/4U);
////		}
////		else if (p_motor->VPwm > 65536U/10U)
////		{
////			p_motor->VPwm = (65536U/10U);
////		}
//
//	}
//	else
//	{
		switch (p_motor->Parameters.ControlMode)
		{
	//	case MOTOR_CONTROL_MODE_OPEN_LOOP:
	//		p_motor->VPwm = p_motor->UserCmd / 4U;
	//		break;

		case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
			if (p_motor->IMotor_Frac16 > (58982U))
			{
				p_motor->VPwm = p_motor->VPwm - ((p_motor->IMotor_Frac16 - 58982U) * p_motor->VPwm >> 16U);
				//proc constant current pid, set integral term to vpwm.
			}
			else
			{
				p_motor->VPwm = p_motor->RampCmd;
			}
			break;

		case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ:
			//p_motor->VPwm = p_motor->RampCmd * p_motor->Speed_RPM * p_motor->VRpmGain;
			break;

		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
			Motor_PollSpeedFeedbackLoop(p_motor);
			break;

		case MOTOR_CONTROL_MODE_CONSTANT_CURRENT:

			break;

		default:
			break;
		}
//	}
}

//static inline void Motor_SetBrakeRegenOptimal(Motor_T * p_motor)
//{
//	p_motor->VPwm = Linear_Voltage_CalcUnsignedFraction16(&p_motor->UnitVabc, GetVBemf(&p_motor)) / 2U;
//}
//
//static inline void Motor_SetBrakeRegenProportional(Motor_T * p_motor, uint16_t intensity)
//{
//	p_motor->VPwm = Linear_Voltage_CalcUnsignedFraction16(&p_motor->UnitVabc, GetVBemf(&p_motor)) * (65536U - intensity) >> 16U;
//
//	//braking 0% -> pwm 100% of back emf;
//	//braking 10% -> pwm 90% of back emf;
//	//braking 50% -> pwm 50% of back emf;
//	//braking 90% -> pwm 10% of back emf;
//}
//
//static inline void Motor_SetBrakeRegenScalar(Motor_T * p_motor, uint16_t intensity)
//{
//	p_motor->VPwm = Linear_Voltage_CalcUnsignedFraction16(&p_motor->UnitVabc, GetVBemf(&p_motor)) * (32768U - ((intensity + 32768U) / p_motor->Parameters.BrakeCoeffcient)) >> 16U;
//
//	// e.g
//	// RegenCoeffcientInput = 4, RegenCoeffcient -> fract16(1,4)
//
//	//braking 0% -> pwm 62.5% of back emf;
//	//braking 10% -> pwm 60% of back emf;
//	//braking 50% -> pwm 50% of back emf;
//	//braking 90% -> pwm 40% of back emf;
//	//braking 100% -> pwm 37.5% of back emf;
//}


//static inline void Motor_ProcControlVarBrake(Motor_T * p_motor)
//{
//	switch (p_motor->Parameters.BrakeMode)
//	{
//	case MOTOR_BRAKE_MODE_SCALAR:
//		Motor_ProcRamp(p_motor);
//		break;
//	case MOTOR_BRAKE_MODE_REGEN_OPTIMAL:
//		Motor_SixStep_SetBrakeRegenOptimal(p_motor);
//		break;
//	case MOTOR_BRAKE_MODE_REGEN_PROPRTIONAL:
//		Motor_SixStep_SetBrakeRegenProportional(p_motor, p_motor->UserCmd);
//		break;
//	case MOTOR_BRAKE_MODE_REGEN_SCALAR:
//		Motor_SixStep_SetBrakeRegenScalar(p_motor, p_motor->UserCmd);
//		break;
//	default:
//		break;
//	}
//
//}


/******************************************************************************/
/*! @} */
/******************************************************************************/
