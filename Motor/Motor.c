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
    @file 	Motor.c
    @author FireSoucery
    @brief  Motor module conventional function definitions.
    @version V0
*/
/**************************************************************************/
#include "Motor.h"

#include "Config.h"
//#include "Default.h"

#include "MotorController/MotorFlash.h"
#include "MotorStateMachine.h"


#include "System/StateMachine/StateMachine.h"
#include "System/Thread/Thread.h"

#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"
#include "Transducer/Phase/Phase.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear.h"



/*
 * General Init
 */
void Motor_Init(Motor_T * p_motor, const Motor_Init_T * p_motorInit)
{
	p_motor->p_Init = p_motorInit;

	//if first time boot, use serial com instead

	MotorStateMachine_Init(p_motor);
	Motor_InitReboot(p_motor);
}

//state machine init-state run
void Motor_InitReboot(Motor_T * p_motor)
{
	const Motor_Init_T * p_motorInit = p_motor->p_Init; //Load Compile time consts


	/*
	 * Motor Utility Instance Init, 1 for all motor
	 */

	//call from outside since there is only 1
//	MotorFlash_Init
//	(
//		&p_motorInit->HAL_FLASH
//	);


	MotorFlash_LoadParameterAll(p_motor);

	//	MotorShell_Init();
	//	MotorUser_Init();

	/*
	 * HW Wrappers Init
	 */
	Phase_Init
	(
		&p_motor->Phase,
		&p_motorInit->HAL_PHASE,
		p_motorInit->PHASE_PWM_PERIOD
	);


	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP || p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		//all sixstep modes and hall foc mode use CaptureTime
		Encoder_Motor_InitCaptureTime
		(
			&p_motor->Encoder,
			&p_motorInit->HAL_ENCODER,
			p_motorInit->HALL_ENCODER_TIMER_COUNTER_MAX,
			p_motorInit->HALL_ENCODER_TIMER_COUNTER_FREQ,
			p_motorInit->MOTOR_PWM_FREQ,
//			16U, //always 16 bits
			p_motor->Parameters.PolePairs,
			p_motor->Parameters.PolePairs*6U,  // use PolePairs * 6 for count per commutation or PolePairs for per erotation
			p_motor->Parameters.EncoderDistancePerCount
		);

		Encoder_DeltaT_InitExtendedTimer
		(
			&p_motor->Encoder,
			&p_motor->MillisTimerBase,
			1000U,
			1000U
		);

		Hall_Init
		(
			&p_motor->Hall,
			&p_motorInit->HAL_HALL,
			MOTOR_SECTOR_ID_1,
			MOTOR_SECTOR_ID_2,
			MOTOR_SECTOR_ID_3,
			MOTOR_SECTOR_ID_4,
			MOTOR_SECTOR_ID_5,
			MOTOR_SECTOR_ID_6,
			p_motor->Parameters.HallVirtualSensorInvBMap,
			p_motor->Parameters.HallVirtualSensorAMap,
			p_motor->Parameters.HallVirtualSensorInvCMap,
			p_motor->Parameters.HallVirtualSensorBMap,
			p_motor->Parameters.HallVirtualSensorInvAMap,
			p_motor->Parameters.HallVirtualSensorCMap,
			MOTOR_SECTOR_ERROR_000,
			MOTOR_SECTOR_ERROR_111
		);
	}
	else if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_ENCODER)
	{
		Encoder_Motor_InitCaptureCount
		(
			&p_motor->Encoder,
			&p_motorInit->HAL_ENCODER,
			p_motorInit->MOTOR_PWM_FREQ,
//			16U,
			p_motor->Parameters.PolePairs,
			p_motor->Parameters.EncoderCountsPerRevolution,
			p_motor->Parameters.EncoderDistancePerCount
		);

		Encoder_SetQuadratureMode(&p_motor->Encoder, p_motor->Parameters.EncoderIsQuadratureModeEnabled);
		Encoder_SetQuadratureDirectionCalibration(&p_motor->Encoder, p_motor->Parameters.EncoderIsALeadBPositive);
	}
	else //FOC open loop
	{
		//unused for now
		Encoder_Motor_InitCaptureCount
		(
			&p_motor->Encoder,
			&p_motorInit->HAL_ENCODER,
			p_motorInit->MOTOR_PWM_FREQ,
//			16U,
			p_motor->Parameters.PolePairs,
			p_motor->Parameters.PolePairs, //p_motor->Parameters.EncoderCountsPerRevolution,
			p_motor->Parameters.EncoderDistancePerCount
		 );
	}

	BEMF_Init
	(
		&p_motor->Bemf,
		&p_motor->ControlTimerBase,
		&p_motor-> AnalogChannelResults[MOTOR_ANALOG_CHANNEL_VA], //change todo
		&p_motor-> AnalogChannelResults[MOTOR_ANALOG_CHANNEL_VB],
		&p_motor-> AnalogChannelResults[MOTOR_ANALOG_CHANNEL_VC],
		&p_motor-> AnalogChannelResults[MOTOR_ANALOG_CHANNEL_VBUS],
		p_motor->Parameters.BemfSampleMode
	);

	Debounce_Init(&p_motor->PinBrake, 		&p_motorInit->HAL_PIN_BRAKE, 		&p_motor->MillisTimerBase, 5U);	//5millis
	Debounce_Init(&p_motor->PinThrottle,	&p_motorInit->HAL_PIN_THROTTLE, 	&p_motor->MillisTimerBase, 5U);	//5millis
	Debounce_Init(&p_motor->PinForward, 	&p_motorInit->HAL_PIN_FORWARD, 		&p_motor->MillisTimerBase, 5U);	//5millis
	Debounce_Init(&p_motor->PinReverse, 	&p_motorInit->HAL_PIN_REVERSE, 		&p_motor->MillisTimerBase, 5U);	//5millis

	//	if analog module algo supports matching channel to adc virtualization layer
	//	Analog_Init
	//	(
	//	);

	FOC_Init(&p_motor->Foc);

	Thread_InitThreadPeriodic_Period //Timer only mode
	(
		&p_motor->ControlTimerThread,
		&p_motor->ControlTimerBase,
		p_motorInit->MOTOR_PWM_FREQ,
		1U,
		0U,
		0U
	);

	Thread_InitThreadPeriodic_Period //Timer only mode
	(
		&p_motor->MillisTimerThread,
		&p_motor->MillisTimerBase,
		1000U,
		1U,
		0U,
		0U
	);

	Thread_InitThreadPeriodic_Period //Timer only mode
	(
		&p_motor->SecondsTimerThread,
		&p_motor->MillisTimerBase,
		1000U,
		1000U,
		0U,
		0U
	);


	//initial runtime config settings
	Phase_Polar_ActivateMode(&p_motor->Phase, p_motor->Parameters.PhasePwmMode);

	//uncalibrated default
	Linear_ADC_Init(&p_motor->UnitThrottle, 0U, 4095U, 100U);
	Linear_ADC_Init(&p_motor->UnitBrake, 0U, 4095U, 100U);

	/*
	 * Run calibration later, default zero to middle adc
	 */
	Linear_ADC_Init(&p_motor->UnitIa, 2048U, 4095U, 120U); //temp 120amp
	Linear_ADC_Init(&p_motor->UnitIb, 2048U, 4095U, 120U);
	Linear_ADC_Init(&p_motor->UnitIc, 2048U, 4095U, 120U);


	Linear_Voltage_Init(&p_motor->UnitVBus, p_motorInit->LINEAR_V_BUS_R1, p_motorInit->LINEAR_V_BUS_R2, p_motorInit->LINEAR_V_ADC_VREF, p_motorInit->LINEAR_V_ADC_BITS, p_motor->Parameters.VBusRef);
	Linear_Voltage_Init(&p_motor->UnitVabc, p_motorInit->LINEAR_V_ABC_R1, p_motorInit->LINEAR_V_ABC_R2, p_motorInit->LINEAR_V_ADC_VREF, p_motorInit->LINEAR_V_ADC_BITS, p_motor->Parameters.VBusRef);

	//Linear_Init(&(p_Motor->VFMap), vPerRPM, 1, vOffset); //f(freq) = voltage

	p_motor->Direction 		= MOTOR_DIRECTION_CCW;
	p_motor->DirectionInput = MOTOR_DIRECTION_CCW;
	p_motor->SpeedFeedback_RPM = 0U;
	p_motor->VPwm = 0U;

}


void Motor_SetZero(Motor_T * p_motor)
{
	p_motor->SpeedFeedback_RPM = 0U;
	p_motor->VPwm = 0;
}


void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	p_motor->Direction = direction;


	//sets for 6 step
//	if (direction = MOTOR_DIRECTION_CW)
//	{
//		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW);
//	}
//	else
//	{
//		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW);
//	}


//	switch (direction)
//	{
//
//	case MOTOR_DIRECTION_CW:
//		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW);
//		//	 BEMF_SetDirection(&p_motor->Bemf, direction);
//		//	 FOC_SetDirection(&p_motor);
//		break;
//
//	case MOTOR_DIRECTION_CCW:
//		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW);
//		//	 BEMF_SetDirection(&p_motor->Bemf, direction);
//		//	 FOC_SetDirection(&p_motor);
//		break;
//
////	case	MOTOR_DIRECTION_NEUTRAL:
////		break;
//
//	default:
//		break;
//	}


}







//Move to control variable
//per motor
 void Motor_SetUserCmd(Motor_T * p_motor, uint16_t userCommand)
{
	p_motor->UserCmdPrev = p_motor->UserCmd;
	p_motor->UserCmd = userCommand;
}

// int32_t Motor_GetRampIncIndex(Motor_T * p_motor, uint32_t * p_index)
//{
//	return Linear_Ramp_CalcTarget_IncIndex(&p_motor->Ramp, p_index, 1U);
//}

 void Motor_ProcRamp(Motor_T * p_motor) // input voltage/speed cmd
{
	p_motor->RampCmd = Linear_Ramp_CalcTarget_IncIndex(&p_motor->Ramp, &p_motor->RampIndex, 1U);
}


//proc ramp update ~millis
 void Motor_SetRampAccelerate(Motor_T * p_motor, uint16_t acceration)
{

	if (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE || p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
	{

	}
	else if (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT)
	{

	}
	else //p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE
	{
		//Ramp to throttle over 1s
//		if (p_motor->UserCmd > p_motor->UserCmdPrev)
		{
			//(int32_t)p_motor->UserCmd - (int32_t)p_motor->UserCmdPrev
			Linear_Ramp_InitAcceleration(&p_motor->Ramp, 20000U, p_motor->VPwm, p_motor->UserCmd, p_motor->UserCmd);
		}

		//todo match  bemf
	}
	p_motor->RampIndex = 0;
}

 void Motor_SetRampDecelerate(Motor_T * p_motor, uint16_t deceleration)
{
	if (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE || p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
	{

	}
	else if (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT)
	{

	}
	else
	{
		//Decel by Brake per 1s
		//todo match  bemf/current
		Linear_Ramp_InitAcceleration(&p_motor->Ramp, 20000U, p_motor->VPwm, 0, -(int32_t)p_motor->UserCmd);

	}
	p_motor->RampIndex = 0;
}

// void Motor_SetRamp(Motor_T * p_motor)
//{
//	if (p_motor->InputSwitchBrake == true)
//	{
//		Motor_SetRampDecel(p_motor);
//	}
//	else
//	{
//		Motor_SetRampAccel(p_motor);
//	}
//}

 /*
  * Speed PID Feedback Loop
  *
  * Inputs by pointer:
  * 	p_motor->Speed_RPM
  * 	p_motor->SpeedReq_RPM
  *
  * Output by pointer:
  * 	FOC Mode - sets iqReq, Vcmd,
  * 	SixStep -
  *
  * generalize to scalar feedback variable e.g. temperature
  *
  */
 void Motor_ProcSpeedFeedback(Motor_T * p_motor)
 {
 	p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
 //	//p_motor->SpeedFeedback_RPM = Filter_MovAvg(p_motor->Speed_RPM, coef, coef);
 //	PID_Proc(&p_motor->PidSpeed, p_motor->SpeedFeedback_RPM, setpoint);
 }

 void Motor_PollSpeedFeedback(Motor_T * p_motor)
 {
 	if (Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
 	{
 		 Motor_ProcSpeedFeedback(p_motor);
 	}
 }



uint32_t Motor_CalcControlVariable(Motor_T * p_motor, Motor_ControlMode_T liveControlVarMode, uint32_t var) // input voltage/speed cmd
{
	switch (liveControlVarMode)
	{
	case MOTOR_CONTROL_MODE_OPEN_LOOP:
		p_motor->VPwm = var/4U;
		break;

	case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
		p_motor->VPwm = var;
		break;

	case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ:
		//p_motor->VPwm = p_motor->RampCmd * p_motor->Speed_RPM * p_motor->VRpmGain;
		break;

	case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
		if (Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
		{
			 Motor_ProcSpeedFeedback(p_motor); //get from PID
		}

		break;

	case MOTOR_CONTROL_MODE_CONSTANT_CURRENT:
		p_motor->VPwm = p_motor->VPwm * 58982U/p_motor->PhaseCurrentFiltered_Frac16;
		//proc constant current pid
		break;

	default:
		break;
	}

}





// void Motor_SetControlVariable(Motor_T * p_motor, mode, cmd) // input voltage/speed cmd

 void Motor_ProcControlVariable(Motor_T * p_motor, Motor_ControlMode_T liveControlVarMode, uint32_t var) // input voltage/speed cmd
{
	switch (liveControlVarMode)
	{
	case MOTOR_CONTROL_MODE_OPEN_LOOP:
		p_motor->VPwm = p_motor->UserCmd/4U;
		break;

	case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
		p_motor->VPwm = p_motor->RampCmd;
		break;

	case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ:
		//p_motor->VPwm = p_motor->RampCmd * p_motor->Speed_RPM * p_motor->VRpmGain;
		break;

	case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
//		if (Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
//		{
//			 Motor_ProcSpeedFeedback(p_motor); //get from PID
//		}
		break;

	case MOTOR_CONTROL_MODE_CONSTANT_CURRENT:

		break;

	default:
		break;
	}

}

 void Motor_ProcControlVariable_Thread(Motor_T * p_motor)
{
	Motor_ControlMode_T liveControlVarMode;

	if (p_motor->PhaseCurrentFiltered_Frac16 > 58982U)
	{
		p_motor->VPwm = p_motor->VPwm * 58982U/p_motor->PhaseCurrentFiltered_Frac16;
				//proc constant current pid
		liveControlVarMode = MOTOR_CONTROL_MODE_CONSTANT_CURRENT;
	}
	else
	{
		liveControlVarMode = p_motor->Parameters.ControlMode;
	}

	Motor_ProcControlVariable(p_motor, liveControlVarMode, p_motor->RampCmd);

	//< 90%
//	if(p_motor->PhaseCurrentFiltered_Frac16 > 58982U)
//	{
//		p_motor->VPwm = p_motor->VPwm * 58982U/p_motor->PhaseCurrentFiltered_Frac16;
//		//proc constant current pid
//	}
//	else
//	{
//		Motor_ProcControlVariable(p_motor);
//	}
}









void Motor_StartCalibrateEncoder(Motor_T * p_motor)
{
	p_motor->ControlTimerBase = 0U;
	Thread_SetTimerPeriod(&p_motor->ControlTimerThread, 20000U);

	Phase_ActuateDutyCycle(&p_motor->Phase, 6553U/4U, 0, 0); /* Align Phase A 10% pwm */
	Phase_ActuateState(&p_motor->Phase, true, true, true);
}

void Motor_CalibrateEncoder(Motor_T * p_motor)
{
	static uint8_t state = 0; //limits calibration to 1 at a time;

	const uint16_t duty = 65536/10/4;

	bool isComplete = false;

	if (Thread_PollTimerCompletePeriodic(&p_motor->ControlTimerThread) == true)
	{
		switch (state)
		{
		case 0U:
			Encoder_DeltaD_CalibrateQuadratureReference(&p_motor->Encoder);

			Phase_ActuateState(&p_motor->Phase, true, true, true);
			Phase_ActuateDutyCycle(&p_motor->Phase, 0U, duty, 0U);
			state++;
			break;

		case 1U:
			Encoder_DeltaD_CalibrateQuadratureDirectionPositive(&p_motor->Encoder);
			Phase_ActuateState(&p_motor->Phase, false, false, false);
			state = 0;
			isComplete = true;
			break;

		default:
			break;

		}
	}

	return isComplete;
}


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

//void Motor_ActivateAlign(Motor_T * p_motor)
//{
//	Phase_ActuateDutyCycle(&p_motor->Phase, 6553U/4U, 0, 0); /* Align Phase A 10% pwm */
//	Phase_ActuateState(&p_motor->Phase, 1, 1, 1);
//}

void Motor_StartAlign(Motor_T * p_motor)
{
	p_motor->ControlTimerBase = 0U;
	Thread_SetTimerPeriod(&p_motor->ControlTimerThread, 20000U); //Parameter.AlignTime
//	Motor_ActivateAlign(p_motor);
	//set to usercmd  6553U/4U p_motor->UserCmd
	Phase_ActuateDutyCycle(&p_motor->Phase, 6553U/4U, 0, 0); /* Align Phase A 10% pwm */
	Phase_ActuateState(&p_motor->Phase, 1, 1, 1);
}

bool Motor_ProcAlign(Motor_T * p_motor)
{
	bool status;
	if (Thread_PollTimerComplete(&p_motor->ControlTimerThread) == true)
	{
		p_motor->ElectricalAngle= 0U;
		Encoder_Reset(&p_motor->Encoder);
//		Motor_Float(&p_motor->Foc);
		status = true;
	}
	else
	{
		status = false;
	}
	return status;
}


void Motor_StartCalibrateHall(Motor_T * p_motor)
{
	p_motor->ControlTimerBase = 0U;
	Thread_SetTimerPeriod(&p_motor->ControlTimerThread, 20000U); //Parameter.HallCalibrationTime
}

//120 degree hall aligned with phase
bool Motor_CalibrateHall(Motor_T * p_motor)
{
	static uint8_t state = 0; //limits calibration to 1 at a time;

	const uint16_t duty = 65536 / 10/4;

	bool isComplete = false;

	if (Thread_PollTimerCompletePeriodic(&p_motor->ControlTimerThread) == true)
	{
		switch (state)
		{
		case 0U:
			Phase_ActuateState(&p_motor->Phase, true, true, true);

			Phase_ActuateDutyCycle(&p_motor->Phase, duty, 0U, 0U);
			state++;
			break;

		case 1U:
			Hall_CalibrateSensorAPhaseBC(&p_motor->Hall, MOTOR_SECTOR_ID_2);

			Phase_ActuateDutyCycle(&p_motor->Phase, duty, duty, 0U);
			state++;
			break;

		case 2U:
			Hall_CalibrateSensorInvCPhaseBA(&p_motor->Hall, MOTOR_SECTOR_ID_3);

			Phase_ActuateDutyCycle(&p_motor->Phase, 0U, duty, 0);
			state++;
			break;

		case 3U:
			Hall_CalibrateSensorBPhaseCA(&p_motor->Hall, MOTOR_SECTOR_ID_4);

			Phase_ActuateDutyCycle(&p_motor->Phase, 0U, duty, duty);
			state++;
			break;

		case 4U:
			Hall_CalibrateSensorInvAPhaseCB(&p_motor->Hall, MOTOR_SECTOR_ID_5);

			Phase_ActuateDutyCycle(&p_motor->Phase, 0U, 0U, duty);
			state++;
			break;

		case 5U:
			Hall_CalibrateSensorCPhaseAB(&p_motor->Hall, MOTOR_SECTOR_ID_6);

			Phase_ActuateDutyCycle(&p_motor->Phase, duty, 0U, duty);
			state++;
			break;

		case 6U:
			Hall_CalibrateSensorInvBPhaseAC(&p_motor->Hall, MOTOR_SECTOR_ID_1);

			Phase_ActuateState(&p_motor->Phase, false, false, false);
			state = 0;
			isComplete = true;
			break;

		default:
			break;

		}
	}

	return isComplete;
}



//void Motor_OnBlock(Motor_T * p_motor)
//{
//
//}
