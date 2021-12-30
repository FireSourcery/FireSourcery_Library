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
    @file 	Motor_FOC.h
    @author FireSoucery
    @brief  Motor FOC submodule. FOC control functions.

    		defined as inline for StateMachine wrapper functions
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "Motor.h"
#include "Config.h"

#include "Math/FOC.h"

#include "Peripheral/Analog/AnalogN/AnalogN.h"

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
/*
 * ElectricalAngle, VPwm => Phase Duty
 */
static inline void ActivateMotorFocAngle(Motor_T * p_motor)
{
	FOC_SetVector(&p_motor->Foc, p_motor->ElectricalAngle);

	//FOC_SetVd(&p_motor->Foc, p_motor->FieldWeakening);
	if (p_motor->Direction == MOTOR_DIRECTION_CW)
	{
		FOC_SetVq(&p_motor->Foc, 0 - (qfrac16_t)(p_motor->VPwm >> 1U));
	}
	else
	{
		FOC_SetVq(&p_motor->Foc, (qfrac16_t)(p_motor->VPwm >> 1U));
	}

	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);

	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
}

static inline void ProcMotorFocAngleControl(Motor_T * p_motor)
{
	uint32_t electricalDelta;

//	static volatile uint32_t debug[1000];
//	static volatile uint32_t debugCounter = 0;

//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_OBSERVE_IABC); observe only
	switch (p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			/* OpenLoop
			 * Blind input angle, constant voltage
			 *
			 * 	p_foc->Theta = integral of speed req
			 */
			/* integrate speed to angle */
			p_motor->OpenLoopSpeed_RPM = Linear_Ramp_CalcTargetIncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, 1U);
			p_motor->ElectricalAngle += Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
			break;

		case MOTOR_SENSOR_MODE_BEMF:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			/*
			 * Encoder_Motor_GetElectricalTheta return [0, 65535] maps directly to negative portions of qangle16_t
			 */
			Encoder_DeltaD_Capture(&p_motor->Encoder);
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
			if(Hall_PollCaptureSensors(&p_motor->Hall) == true)
			{
				Encoder_DeltaT_Capture(&p_motor->Encoder);
				Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);
				p_motor->HallAngle = (qangle16_t)Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
				p_motor->InterpolatedAngleIndex = 0U;
			}

			electricalDelta = Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder, p_motor->InterpolatedAngleIndex);

			if (electricalDelta > 65536U/6U)
			{
				electricalDelta = 65536U/6U;
			}
//			else
//			{
//				electricalDelta = ((int32_t)p_motor->ElectricalDeltaPrev + (int32_t)electricalDelta)/2U;
//			}
//
//			p_motor->ElectricalDeltaPrev = electricalDelta;

			//todo check index

			p_motor->ElectricalAngle = p_motor->HallAngle + electricalDelta;
			p_motor->InterpolatedAngleIndex++;
			break;

		default:
			break;
	}


}



/*
	StateMachine calls each PWM, ~20kHz
	shared function -> less states
 */
static inline void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
	if (((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)) == false)
	{
		/* Voltage Control mode - Proc angle immediately */
		ProcMotorFocAngleControl(p_motor);

		Motor_ProcControlVariable(p_motor); //input usercmd rampcmd, set vpwm
		ActivateMotorFocAngle(p_motor); 	//input vpwm, theta
	}

	/* Current Control Mode - Proc angle on ADC return */
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_FOC_IABC);
}



static inline void Motor_FOC_ResumeAngleControl(Motor_T * p_motor)
{
	switch (p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			//from stop only
			//can start at 0 speed in foc mode for continuous angle displacements
			Linear_Ramp_Init_Millis(&p_motor->OpenLoopRamp, 20000U, 0U, 200U, 2000U);
			p_motor->OpenLoopRampIndex = 0U;
			break;

		case MOTOR_SENSOR_MODE_BEMF:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:

			break;

		case MOTOR_SENSOR_MODE_HALL:
			//or reset hall for next edge
			Hall_ResetCapture(&p_motor->Hall);
			break;

		default:
			break;
	}

	switch (p_motor->Parameters.ControlMode)
	{
//	case MOTOR_CONTROL_MODE_OPEN_LOOP:
//		//		//Linear_Ramp_Init(&p_motor->Ramp, freq, start, end, accerleration);
//		Linear_Ramp_InitMillis(&p_motor->Ramp, 0U, 10U, 1000U, 20000U);
//		p_motor->CommutationPeriodCmd = 0U;
//		break;

	case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
		break;

	case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
		break;

	default:
		break;

	}
}

/*
 *	from stop only
 * prep run state. shared to reduce number of states
 */
static inline void Motor_FOC_StartAngleControl(Motor_T * p_motor)
{
	Phase_Ground(&p_motor->Phase); //activates abc
	FOC_SetZero(&p_motor->Foc);
	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output
	Encoder_Reset(&p_motor->Encoder); //zero angle? speed?

	Motor_FOC_ResumeAngleControl(p_motor);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/



/******************************************************************************/
/*! @} */
/******************************************************************************/
/*
 * Calibrate Current ADC
 */
extern void Motor_FOC_StartCalibrateAdc(Motor_T * p_motor);
extern bool Motor_FOC_CalibrateAdc(Motor_T *p_motor);

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
extern void Motor_FOC_CaptureIa(Motor_T *p_motor);
extern void Motor_FOC_CaptureIb(Motor_T *p_motor);
extern void Motor_FOC_CaptureIc(Motor_T *p_motor);

/*
 * ADC conversion complete call
 */
extern void Motor_FOC_ProcCurrentFeedback(Motor_T * p_motor);

#endif



