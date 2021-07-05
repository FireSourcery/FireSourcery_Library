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
    @file 	Motor_FOC.h
    @author FireSoucery
    @brief  Motor FOC submodule. FOC control functions.

    		defined as inline for StateMachine wrapper functions
    @version V0
*/
/**************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "Motor.h"
#include "Config.h"

#include "Math/FOC.h"

#include "Transducer/Encoder/Encoder_DeltaT.h"
#include "Transducer/Encoder/Encoder_DeltaD.h"
#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"

#include "Transducer/Hall/Hall.h"
#include "Transducer/Phase/Phase.h"

#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Ramp.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
	@addtogroup
	@{
*/
/******************************************************************************/
static inline void Motor_FOC_ActivateAngle(Motor_T * p_motor) //input pwm and angle
{
	//FOC_SetVd(&p_motor->Foc, p_motor->FieldWeakening);
	if (p_motor->Direction == MOTOR_DIRECTION_CW) //negative angle
	{
		FOC_SetVq(&p_motor->Foc, 0 - (qfrac16_t)(p_motor->VPwm >> 1U));
	}
	else
	{
		FOC_SetVq(&p_motor->Foc, (qfrac16_t)(p_motor->VPwm >> 1U));
	}

	FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);

	FOC_ProcTheta(&p_motor->Foc);
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);

//	FOC_ProcDirectSvpwm(&p_motor->Foc);

	Phase_SetDutyCyle(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
	Phase_Actuate(&p_motor->Phase);
}



/*
	StateMachine calls each PWM, ~20kHz
	shared function -> less states
 */
static inline void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
	uint32_t electricalDelta;

	static volatile uint32_t debug[1000];
	static volatile uint32_t debugCounter = 0;

	if ((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT))
	{
//		Analog_ActivateConversion(&p_motor->Analog, &FOC_ANALOG_CONVERSION_ANGLE_CONTROL);
		//Proc angle on ADC Return
	}
	else
	{
		switch (p_motor->Parameters.SensorMode)
		{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			/* OpenLoop
			 * Blind input angle, constant voltage
			 *
			 * 	p_foc->Theta = integral of speed req
			 */
			/* integrate speed to angle */
//			p_motor->SpeedOpenLoop_RPM = Linear_Ramp_CalcTarget_IncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, 1U);
//			p_motor->ElectricalAngle += Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(&p_motor->Encoder, p_motor->OpenLoop_RPM);

			break;

		case MOTOR_SENSOR_MODE_BEMF:

			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			Encoder_DeltaD_Capture_IO(&p_motor->Encoder);
			p_motor->ElectricalAngle = (qangle16_t)Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);
			//recalibrate when phase a current cross 0
//			if(debugCounter < 1000)
//			{
//				debug[debugCounter] = p_motor->ElectricalAngle;
//				debugCounter++;
//			}
//			else
//			{
//				debugCounter = 0;
//			}


			break;

		case MOTOR_SENSOR_MODE_HALL:
			if (Hall_PollSensorsEdge_IO(&p_motor->Hall)) //update speed on every edge
			{
				Encoder_DeltaT_Capture_IO(&p_motor->Encoder);
				Encoder_DeltaT_CaptureExtension_IO(&p_motor->Encoder); //feed stop poll
				p_motor->HallAngle = (qangle16_t)Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
				p_motor->ElectricalAngle = p_motor->HallAngle;
				p_motor->InterpolatedAngleIndex = 0U;
			}
			else
			{
				p_motor->InterpolatedAngleIndex++;
				electricalDelta = Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder, p_motor->InterpolatedAngleIndex);
				electricalDelta = ((int32_t)p_motor->ElectricalDeltaPrev + (int32_t)electricalDelta)/2;
				p_motor->ElectricalDeltaPrev = Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder, p_motor->InterpolatedAngleIndex);

//
//				if (electricalDelta > 65536/5)
//				{
//					electricalDelta = 65536/6;
//				}

				if (electricalDelta > 65536/6)
				{
					electricalDelta = 65536/6;
				}

				p_motor->ElectricalAngle = p_motor->HallAngle + electricalDelta;

			}
			break;

		default:
			break;
		}



		Motor_ProcRamp(p_motor);
//		Motor_ProcControlVariable(p_motor); //input usercmd rampcmd, set vpwm

		Motor_FOC_ActivateAngle(p_motor); 	//input vpwm, theta
	}
}

static inline void Motor_FOC_ResumeAngleControl(Motor_T * p_motor)
{

}

/*
 * prep run state. shared to reduce number of states
 */
static inline void Motor_FOC_StartAngleControl(Motor_T * p_motor)
{
	if(Motor_GetSpeed(p_motor) == 0)
	{
		// from stop only
//		Encoder_Reset(&p_motor->Encoder);
		FOC_SetZero(&p_motor->Foc);
		Phase_SetDutyCyle(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output
		Phase_SetState(&p_motor->Phase, true, true, true);

		//if hall Encoder_StartDeltaT(&p_motor->Encoder);
	}

	switch (p_motor->Parameters.SensorMode)
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
//		p_motor->PositionFeedback = MOTOR_POSITION_FEEDBACK_OPEN_LOOP;

		//openloop speed ramp
		//should voltage ramp or alway be proportional to throttle for heavy loads
		//can start at 0 speed in foc mode for continuous angle displacements
		Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 20000U, Motor_GetSpeed(p_motor), Motor_GetSpeed(p_motor) + 100U, 1000U);
		p_motor->OpenLoopRampIndex = 0U;
		break;

	case MOTOR_SENSOR_MODE_BEMF:
		Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 20000U, Motor_GetSpeed(p_motor), Motor_GetSpeed(p_motor) + 100U, 1000U);
		p_motor->OpenLoopRampIndex = 0U;
		break;

	case MOTOR_SENSOR_MODE_ENCODER:

		break;

	case MOTOR_SENSOR_MODE_HALL:
//		p_motor->PositionFeedback = MOTOR_POSITION_FEEDBACK_HALL;

		//or reset hall for next edge
		Hall_CaptureSensors_IO(&p_motor->Hall);

		p_motor->HallAngle = (qangle16_t)Hall_GetRotorAngle_Degrees16(&p_motor->Hall) ; // + 65536/12 compensate for boarder to middle
		p_motor->ElectricalAngle = p_motor->HallAngle;
		p_motor->InterpolatedAngleIndex = 0U;

		//encodder zero prior
		if (Motor_GetSpeed(p_motor) == 0U)
		{
			Encoder_DeltaT_SetInitial(&p_motor->Encoder, 20U);
		}

//		Encoder_CaptureDeltaT_IO(&p_motor->Encoder);
//		Encoder_CaptureExtendedDeltaT_IO(&p_motor->Encoder); //feed stop poll

		break;

	default:
		break;
	}

//	switch (p_motor->ControlMode)
//	{
//	case MOTOR_CONTROL_MODE_OPEN_LOOP:
//		//		//Linear_Ramp_Init(&p_motor->Ramp, freq, start, end, accerleration);
//		Linear_Ramp_InitMillis(&p_motor->Ramp, 0U, 10U, 1000U, 20000U); //must start at sufficent speed for fixed angle displacements
//		p_motor->CommutationPeriodCmd = 0U;
//		break;
//
//	case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
//		break;
//
//	case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
//		break;
//
//	default:
//		break;
//
//	}
}

/*
 * Current PID Feedback Loop
 *
 *	Input     :
 *	motor->IdReq
 *	motor->IqReq
 *	p_foc->Id
 *	p_foc->Iq
 *
 *	output    :
 *	p_foc->Vd
 *	p_foc->Vq
 */
/*
 * ADC conversion complete call
 */
static inline void Motor_FOC_ProcCurrentFeedback(Motor_T * p_motor)
{
	Encoder_DeltaD_Capture_IO(&p_motor->Encoder);

	p_motor->ElectricalAngle = Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);

//	if(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
//	{
//		Motor_PollSpeedFeedback(p_motor);
//	}
//	/* else (p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) */

	p_motor->RampCmd = Linear_Ramp_CalcTarget_IncIndex(&p_motor->Ramp, &p_motor->RampIndex, 1U);
//	Motor_ProcControlVariable(p_motor); //inpus rampcmd, set vpwm


	//	Motor_FOC_ProcCurrentLoop(p_motor);
	FOC_ProcClarkePark(&p_motor->Foc);
	//	PID_Proc_ISR(&PIDd);
	//	PID_Proc_ISR(&PIDq);

	// p_motor->VCmd =  get from PID

	Motor_FOC_ActivateAngle(p_motor);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*!
	@addtogroup
	FOC ADC conversion
	@{
*/
/******************************************************************************/
/*
	Convert current from ADCU to QFrac
 */
static inline void Motor_FOC_ProcIa_IO(Motor_T * p_motor)
{
	//Filter here if needed
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIa, *p_motor->p_Init->P_IA_ADCU);
	FOC_SetIa(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_ProcIb_IO(Motor_T * p_motor)
{
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIb, *p_motor->p_Init->P_IB_ADCU);
	FOC_SetIb(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_ProcIc_IO(Motor_T * p_motor)
{
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIc, *p_motor->p_Init->P_IC_ADCU);
	FOC_SetIc(&p_motor->Foc, i_temp);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

/*
 * Calibrate Current ADC
 */
static inline void  Motor_FOC_StartCalibrateAdc(Motor_T * p_motor)
{
	Thread_SetTimerPeriod(&p_motor->ControlTimerThread, 20000U);//Motor.Parameters.AdcCalibrationTime
	FOC_SetZero(&p_motor->Foc);
	Phase_SetDutyCyle_15(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output
	Phase_SetState(&p_motor->Phase, true, true, true);

//	Linear_ADC_Init(&p_motor->UnitIa, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IA], 4095U, 120U); //temp 120amp
//	Linear_ADC_Init(&p_motor->UnitIb, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IB], 4095U, 120U);
//	Linear_ADC_Init(&p_motor->UnitIc, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IC], 4095U, 120U);
}

static inline bool Motor_FOC_CalibrateAdc(Motor_T *p_motor)
{
	return Motor_CalibrateAdc(p_motor);
}

#endif
//static inline void Motor_FOC_Zero(Motor_T * p_motor)
//{
//
//}


