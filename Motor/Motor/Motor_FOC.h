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

			FOC mode State Machine mapping
    		defined as inline for StateMachine wrapper functions
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "Motor.h"
#include "Config.h"

#include "Math/FOC.h"
#include "Transducer/Hall/Hall.h"
#include "Transducer/Phase/Phase.h"

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#include "Transducer/Encoder/Encoder_DeltaT.h"
#include "Transducer/Encoder/Encoder_DeltaD.h"
#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"

#include "Math/PID/PID.h"
#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Ramp.h"
#include "Math/Filter/Filter_MovAvg.h"
#include "Math/Filter/Filter.h"

#include "System/SysTime/SysTime.h"

#include <stdint.h>
#include <stdbool.h>


/******************************************************************************/
/*!
	Map to Motor Analog Conversions
	Convert current from ADCU to QFrac
*/
/******************************************************************************/
static inline void Motor_FOC_CaptureIa(Motor_T *p_motor)
{
	qfrac16_t i_temp = ((int32_t)Linear_ADC_CalcFractionSigned16(&p_motor->UnitIa, p_motor->AnalogResults.Ia_ADCU) + (int32_t)FOC_GetIa(&p_motor->Foc)) / 2;
	FOC_SetIa(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_CaptureIb(Motor_T *p_motor)
{
	qfrac16_t i_temp = ((int32_t)Linear_ADC_CalcFractionSigned16(&p_motor->UnitIb, p_motor->AnalogResults.Ib_ADCU) + (int32_t)FOC_GetIb(&p_motor->Foc)) / 2;
	FOC_SetIb(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_CaptureIc(Motor_T *p_motor)
{
	qfrac16_t i_temp = ((int32_t)Linear_ADC_CalcFractionSigned16(&p_motor->UnitIc, p_motor->AnalogResults.Ic_ADCU) + (int32_t)FOC_GetIc(&p_motor->Foc)) / 2;
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
	bool captureSpeed = Timer_Poll(&p_motor->SpeedTimer);

	switch (p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			/* OpenLoop
			 * Blind input angle, constant voltage
			 *
			 * 	p_foc->Theta = integral of speed req
			 */
			/* integrate speed to angle */
			p_motor->OpenLoopSpeed_RPM = Linear_Ramp_ProcOutput(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, p_motor->OpenLoopSpeed_RPM);
			p_motor->ElectricalAngle += Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
 			break;

		case MOTOR_SENSOR_MODE_SENSORLESS:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			if (captureSpeed == true)
			{
				Encoder_DeltaD_Capture(&p_motor->Encoder); /* Capture position and speed */
			}
			else
			{
				Encoder_DeltaD_CaptureAngle(&p_motor->Encoder); /* Capture position only */
			}
			/* Encoder_Motor_GetElectricalTheta return [0, 65535] maps directly to negative portions of qangle16_t */
			p_motor->ElectricalAngle = (qangle16_t)Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);
			break;

		case MOTOR_SENSOR_MODE_HALL:
			if(Hall_PollCaptureSensors(&p_motor->Hall) == true)
			{
				Encoder_DeltaT_Capture(&p_motor->Encoder);
				Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);
				p_motor->HallAngle = (qangle16_t)Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
				p_motor->InterpolatedAngleIndex = 0U;
			}
			else
			{
				if (captureSpeed == true) /* Use as indicator for once per millis */
				{
					if (Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true)
					{
						p_motor->Speed_RPM = 0U;
						p_motor->Speed_Frac16 = 0U;
					}
				}
			}

			electricalDelta = Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder, p_motor->InterpolatedAngleIndex);
			if (electricalDelta > 65536U/6U) {electricalDelta = 65536U/6U;}

			if (p_motor->Direction == MOTOR_DIRECTION_CCW)
			{
				p_motor->ElectricalAngle = p_motor->HallAngle + electricalDelta;
			}
			else
			{
				p_motor->ElectricalAngle = p_motor->HallAngle - electricalDelta;
			}

			p_motor->InterpolatedAngleIndex++;
			break;

		default:
			break;
	}

	if (captureSpeed == true)
	{
		Motor_CaptureSpeed(p_motor);
		Motor_ProcSpeedFeedback(p_motor);
	}

	FOC_SetVector(&p_motor->Foc, p_motor->ElectricalAngle);
}


static void ProcMotorFocVoltageMode(Motor_T * p_motor, qfrac16_t vqReq, qfrac16_t vdReq)
{
	qfrac16_t vqReqLimit;

	if (p_motor->Direction == MOTOR_DIRECTION_CCW)
	{
		//match pid output state on overlimit for faster response

		//	if (FOC_GetIq(&p_motor->Foc) > p_motor->Parameters.IqLimit)
		//	{
		//		if(p_motor->IOverLimitFlag == false)
		//		{
		//			p_motor->IOverLimitFlag = true;
		//			PID_SetIntegral(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
		//		}
		//	}
		//	else //alternatively remain set until throttle decrease
		//	{
		//		p_motor->IOverLimitFlag = false;
		//	}

		vqReqLimit = PID_Calc(&p_motor->PidIq, p_motor->Parameters.IqLimit, FOC_GetIq(&p_motor->Foc));
		(vqReq > vqReqLimit) ? FOC_SetVq(&p_motor->Foc, vqReqLimit) : FOC_SetVq(&p_motor->Foc, vqReq);
	}
	else
	{
		vqReqLimit = PID_Calc(&p_motor->PidIq, (int32_t)0 - (int32_t)p_motor->Parameters.IqLimit, FOC_GetIq(&p_motor->Foc));
		(vqReq < vqReqLimit) ? FOC_SetVq(&p_motor->Foc, vqReqLimit) : FOC_SetVq(&p_motor->Foc, vqReq);
	}

	FOC_SetVd(&p_motor->Foc, 0);
}

static  void ProcMotorFocCurrentFeedbackLoop(Motor_T * p_motor, qfrac16_t iqReq, qfrac16_t idReq)
{
	qfrac16_t vqReq;
	qfrac16_t iqReqNew;

	if(iqReq > p_motor->Parameters.IqLimit)
	{
		iqReqNew = p_motor->Parameters.IqLimit;
	}
	else if(iqReq < ((int32_t)0 - (int32_t)p_motor->Parameters.IqLimit))
	{
		iqReqNew = ((int32_t)0 - (int32_t)p_motor->Parameters.IqLimit);
	}
	else
	{
		iqReqNew = iqReq;
	}

	if (p_motor->Direction == MOTOR_DIRECTION_CCW)
	{
		vqReq = PID_Calc(&p_motor->PidIq, iqReqNew, FOC_GetIq(&p_motor->Foc));
		(vqReq > 0) ? FOC_SetVq(&p_motor->Foc, vqReq) : FOC_SetVq(&p_motor->Foc, 0); //no plugging, when req opposite current
	}
	else
	{
		vqReq = PID_Calc(&p_motor->PidIq, iqReqNew, FOC_GetIq(&p_motor->Foc));
		(vqReq < 0) ? FOC_SetVq(&p_motor->Foc, vqReq) : FOC_SetVq(&p_motor->Foc, 0);
	}

	FOC_SetVd(&p_motor->Foc, PID_Calc(&p_motor->PidId, idReq, FOC_GetId(&p_motor->Foc)));
}


static inline qfrac16_t GetMotorFocRampCmd(Motor_T * p_motor)
{
	qfrac16_t qReq = (p_motor->RampCmd >> 1U);
	return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? qReq : 0 - qReq;
}

//output SpeedControl is IqReq or VqReq  =>  RampCmd - Speed always positive values direction independent
static inline qfrac16_t GetMotorFocSpeedControl(Motor_T * p_motor)
{
	qfrac16_t qReq = (p_motor->SpeedControl >> 1U);
	return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? qReq : 0 - qReq;
}

static inline qfrac16_t GetMotorFocSpeedFeedback(Motor_T * p_motor)
{
	qfrac16_t qReq = (p_motor->Speed_Frac16 >> 1U);
	return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? qReq : 0 - qReq;
}

/*
 * Sign indicates absolute direction, positive is virtual CCW.
 * B and Beta are virtual CCW of A/Alpha.
 * Iq sign is relative to rotor direction, NOT Vq direction.
 *
 * CCW +Vq +Iq => Forward Motoring Q1
 * CCW +Vq -Iq => Forward Regen Q4
 * CCW -Vq -Iq => Forward Plugging
 *
 * CW -Vq -Iq => Reverse Motoring Q2
 * CW -Vq +Iq => Reverse Regen Q3
 * CW +Vq +Iq => Reverse Plugging
 *
 *  open loop mode, must be matched? Hall mode physical to virtual conversion will compensate.
 *
 */
static inline void ProcMotorFocControlFeedback(Motor_T * p_motor)
{
	if(p_motor->Brake == true)
	{
		//req opposite iq, vq approach towards 0 but not past 0, no plugging
		ProcMotorFocCurrentFeedbackLoop(p_motor, 0 - GetMotorFocRampCmd(p_motor), 0);
	}
	else
	{
		switch(p_motor->Parameters.ControlMode)
		{
			case MOTOR_CONTROL_MODE_OPEN_LOOP:
				ProcMotorFocVoltageMode(p_motor, GetMotorFocRampCmd(p_motor), 0);
				break;

			case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE: /* Voltage Control mode - use current feedback for over current only */
				ProcMotorFocVoltageMode(p_motor, GetMotorFocRampCmd(p_motor), 0);
				break;

			case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
				ProcMotorFocVoltageMode(p_motor, GetMotorFocSpeedControl(p_motor), 0);
				break;

			case MOTOR_CONTROL_MODE_CONSTANT_CURRENT: 	/* Current Control Mode - Proc angle using ADC return */
				ProcMotorFocCurrentFeedbackLoop(p_motor, GetMotorFocRampCmd(p_motor), 0);
				break;

			case MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT:
				ProcMotorFocCurrentFeedbackLoop(p_motor, GetMotorFocSpeedControl(p_motor), 0);
				break;

			default: break;
		}
	}
}

/*
 * ElectricalAngle, VPwm => Phase Duty
 */
static inline void ActivateMotorFocAngle(Motor_T * p_motor)
{
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc); //todo check output for small vq and sine cos
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
}

	///external call todo combine
	static inline void Motor_FOC_ActivateAngle(Motor_T * p_motor, qangle16_t angle, uint16_t vq)
	{
	//	p_motor->ElectricalAngle = angle;
		FOC_SetVq(&p_motor->Foc, vq);
		FOC_SetVector(&p_motor->Foc, angle);
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
static inline void Motor_FOC_ProcAngleObserve(Motor_T * p_motor)
{
	//no current sense during pwm float
	ProcMotorFocPositionFeedback(p_motor);
}

static inline void Motor_FOC_StartAngleObserve(Motor_T * p_motor)
{
//	p_motor->IOverLimitFlag = false;
}

/*
	StateMachine calls each PWM, ~20kHz
	shared function -> less states
 */
static inline void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
//	p_motor->DebugTime[0] = SysTime_GetMicros() - p_motor->MicrosRef;
	AnalogN_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
	AnalogN_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);

	//samples complete when queue resumes, adc isr priority higher than pwm.

	ProcMotorFocPositionFeedback(p_motor);

//	p_motor->DebugTime[1] = SysTime_GetMicros() - p_motor->MicrosRef;
//	FOC_SetVector(&p_motor->Foc, p_motor->ElectricalAngle);
	FOC_ProcClarkePark(&p_motor->Foc);
//	p_motor->DebugTime[2] = SysTime_GetMicros() - p_motor->MicrosRef;

	ProcMotorFocControlFeedback(p_motor);
	ActivateMotorFocAngle(p_motor);
//	p_motor->DebugTime[3] = SysTime_GetMicros() - p_motor->MicrosRef;
}

static inline void Motor_FOC_SetMatchOutput(Motor_T * p_motor)
{
	//Speed pid always uses directionless positive value
	if(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
	{
		PID_SetIntegral(&p_motor->PidSpeed, 0); // output SpeedControl is IqReq
	}
	else if(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE)
	{
		PID_SetIntegral(&p_motor->PidSpeed, p_motor->Speed_Frac16); 	// output SpeedControl is VqReq
	}

	PID_SetIntegral(&p_motor->PidIq, GetMotorFocSpeedFeedback(p_motor)); //output is VqReq, set proportional to speed to start at 0 torq
	PID_SetIntegral(&p_motor->PidId, 0);
	FOC_SetVq(&p_motor->Foc, GetMotorFocSpeedFeedback(p_motor));
	FOC_SetVd(&p_motor->Foc, 0);
}

/*
 *	from freewheel
 */
static inline void Motor_FOC_ResumeAngleControl(Motor_T * p_motor)
{
	Motor_FOC_SetMatchOutput(p_motor);
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
//	FOC_SetOuputZero(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
	Phase_ActivateSwitchABC(&p_motor->Phase); /* Switches Disabled when entering freewheel State */

//	switch (p_motor->Parameters.SensorMode)
//	{
//		case MOTOR_SENSOR_MODE_OPEN_LOOP:
//			break;
//
//		case MOTOR_SENSOR_MODE_SENSORLESS:
//			break;
//
//		case MOTOR_SENSOR_MODE_ENCODER:
//			break;
//
//		case MOTOR_SENSOR_MODE_HALL:
//			break;
//
//		default:
//			break;
//	}
}

/*
 *	from stop only
 */
static inline void Motor_FOC_StartAngleControl(Motor_T * p_motor)
{
	Encoder_Reset(&p_motor->Encoder); //zero angle speed //reset before Encoder_DeltaT_SetInitial

	Motor_FOC_ResumeAngleControl(p_motor);

	switch (p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			p_motor->OpenLoopRampIndex = 0U;
			break;

		case MOTOR_SENSOR_MODE_SENSORLESS:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			break;

		case MOTOR_SENSOR_MODE_HALL: //move to align for regularity
			Hall_ResetCapture(&p_motor->Hall);
			Encoder_DeltaT_SetInitial(&p_motor->Encoder, 10U);
			break;

		default:
			break;
	}
}

static inline void Motor_FOC_StopAngleControl(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/


#endif



