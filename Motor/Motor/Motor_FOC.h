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
#include "Math/Filter/Filter_MovAvg.h"
#include "Math/Filter/Filter.h"

#include "Math/PID/PID.h"

#include <stdint.h>
#include <stdbool.h>

#include "Utility/Debug/Debug.h"


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
 * Map to Motor Analog Conversions
 * 	Convert current from ADCU to QFrac
 */
static inline void Motor_FOC_CaptureIa(Motor_T *p_motor)
{
	qfrac16_t i_temp = (Linear_ADC_CalcFractionSigned16(&p_motor->UnitIa, p_motor->AnalogResults.Ia_ADCU) + FOC_GetIa(&p_motor->Foc)) / 2;
	FOC_SetIa(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_CaptureIb(Motor_T *p_motor)
{
	qfrac16_t i_temp = (Linear_ADC_CalcFractionSigned16(&p_motor->UnitIb, p_motor->AnalogResults.Ib_ADCU) + FOC_GetIb(&p_motor->Foc)) / 2;
	FOC_SetIb(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_CaptureIc(Motor_T *p_motor)
{
	qfrac16_t i_temp = (Linear_ADC_CalcFractionSigned16(&p_motor->UnitIc, p_motor->AnalogResults.Ic_ADCU) + FOC_GetIc(&p_motor->Foc)) / 2;
	FOC_SetIc(&p_motor->Foc, i_temp);
}
/******************************************************************************/
/*!

*/
/******************************************************************************/


/******************************************************************************/
/*!
	@addtogroup
	@{
*/
/******************************************************************************/
static inline void ProcMotorFocPositionFeedback(Motor_T * p_motor)
{
	uint32_t electricalDelta;

//	static volatile uint32_t debug[1000];
//	static volatile uint32_t debugCounter = 0;

	bool captureSpeed = Motor_PollSpeedFeedback(p_motor);

	switch (p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			/* OpenLoop
			 * Blind input angle, constant voltage
			 *
			 * 	p_foc->Theta = integral of speed req
			 */
			/* integrate speed to angle */
			p_motor->OpenLoopSpeed_RPM = Linear_Ramp_Proc(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex);
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

			if (captureSpeed == true)
			{
				if (Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true)
				{
					p_motor->Speed_RPM = 0U;
					p_motor->SpeedFeedback_Frac16 = 0U;
				}
			}
			break;

		default:
			break;
	}
}

/*
 * ElectricalAngle, VPwm => Phase Duty
 */
static inline void ActivateMotorFocAngle(Motor_T * p_motor)
{
	FOC_SetVector(&p_motor->Foc, p_motor->ElectricalAngle);
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
}

static inline bool PollMotorFocIqOverLimit(Motor_T * p_motor)
{
	if (FOC_GetIq(&p_motor->Foc) > p_motor->Parameters.IqLimit)
	{
//		p_motor->VPwm = p_motor->VPwm - ((p_motor->IBus_Frac16 - 58982U) * p_motor->VPwm >> 16U)
		if(p_motor->IOverLimitFlag == false)
		{
			p_motor->IOverLimitFlag = true;
			PID_SetIntegral(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc) * 9 / 10);
		}

		FOC_SetVq(&p_motor->Foc, PID_Calc(&p_motor->PidIq, p_motor->Parameters.IqLimit, FOC_GetIq(&p_motor->Foc)));
	}

	return p_motor->IOverLimitFlag;
}


static inline void ProcMotorFocControlFeedback(Motor_T * p_motor)
{
	qfrac16_t qReq;
//	qfrac16_t vqReq;
//	qfrac16_t vdReq;

	//todo change to flags
	if((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE))
	{
		qReq = p_motor->SpeedControl >> 1U;
	}
	else /* else (p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT / VOLTAGE) */
	{
		qReq = p_motor->RampCmd >> 1U;
	}

	if (p_motor->Direction == MOTOR_DIRECTION_CW)
	{
		qReq = 0 - qReq;
	}

	if 	(
			(
				(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE) ||
				(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE) ||
				(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_OPEN_LOOP)
			) == true
		)
	{
		/* Voltage Control mode - use current feedback for over current only */
		if (PollMotorFocIqOverLimit(p_motor) == false)
		{
			FOC_SetVq(&p_motor->Foc, qReq);
			FOC_SetVd(&p_motor->Foc, 0); //p_motor->FieldWeakening
		}
	}
	else if (((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)) == true)
	{
		/* Current Control Mode - Proc angle using prev ADC return from prev Pwm */
//		vqReq = PID_Calc(&p_motor->PidIq, qReq, FOC_GetIq(&p_motor->Foc));
//		vdReq = PID_Calc(&p_motor->PidId, 0, FOC_GetId(&p_motor->Foc));
		FOC_SetVq(&p_motor->Foc, PID_Calc(&p_motor->PidIq, qReq, FOC_GetIq(&p_motor->Foc)));
		FOC_SetVd(&p_motor->Foc, PID_Calc(&p_motor->PidId, 0, FOC_GetId(&p_motor->Foc)));
	}

	ActivateMotorFocAngle(p_motor);
}

/******************************************************************************/
/*!
*/
/******************************************************************************/


/******************************************************************************/
/*!
*/
/******************************************************************************/


/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline void Motor_FOC_ProcAngleObserve(Motor_T * p_motor)
{
	AnalogN_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
	AnalogN_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);

	FOC_ProcClarkePark(&p_motor->Foc); //using prev adc reading
	ProcMotorFocPositionFeedback(p_motor);
}

static inline void Motor_FOC_StartAngleObserve(Motor_T * p_motor)
{
	p_motor->IOverLimitFlag = false;
}

/*
	StateMachine calls each PWM, ~20kHz
	shared function -> less states
 */
static inline void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
	Debug_CaptureElapsed(7);
	Motor_FOC_ProcAngleObserve(p_motor);
	ProcMotorFocControlFeedback(p_motor);
	Debug_CaptureElapsed(8);
}

static inline void Motor_FOC_ResumeAngleControl(Motor_T * p_motor)
{
	if((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE))
	{
		Motor_ResumeSpeedFeedback(p_motor);
	}

	if (((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)) == true)
	{
		PID_SetIntegral(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
	}

	switch (p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:

			break;

		case MOTOR_SENSOR_MODE_BEMF:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:

			break;

		case MOTOR_SENSOR_MODE_HALL:
			Hall_ResetCapture(&p_motor->Hall);
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
	Motor_FOC_ResumeAngleControl(p_motor);

	Phase_Ground(&p_motor->Phase); //activates abc
	FOC_SetZero(&p_motor->Foc);
	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output

	Encoder_Reset(&p_motor->Encoder); //zero angle speed //reset before Encoder_DeltaT_SetInitial

	switch (p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			//from stop only
			//can start at 0 speed in foc mode for continuous angle displacements
			Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 20000U, 0U, 300U, 2000U);
			p_motor->OpenLoopRampIndex = 0U;
			break;

		case MOTOR_SENSOR_MODE_BEMF:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:

			break;

		case MOTOR_SENSOR_MODE_HALL:
			Encoder_DeltaT_SetInitial(&p_motor->Encoder, 10U);
			break;

		default:
			break;
	}


}

/******************************************************************************/
/*! @} */
/******************************************************************************/


#endif



