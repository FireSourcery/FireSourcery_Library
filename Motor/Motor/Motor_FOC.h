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

/*
 * +/- Sign indicates absolute direction, CW/CCW. NOT along or against direction selected.
 * Positive is virtual CCW.
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
 */


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
			p_motor->OpenLoopSpeed_RPM = Linear_Ramp_ProcIndexOutput(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, p_motor->OpenLoopSpeed_RPM);
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

			//reset peak
			break;

		case MOTOR_SENSOR_MODE_HALL:
			if(Hall_PollCaptureSensors(&p_motor->Hall) == true)
			{
				Encoder_DeltaT_Capture(&p_motor->Encoder);
				Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);
				p_motor->HallAngle = (qangle16_t)Hall_GetRotorAngle_Degrees16(&p_motor->Hall);

				if(Hall_GetSensors(&p_motor->Hall).State == 1U)
				{
					p_motor->VBemfPeak_ADCU = p_motor->VBemfPeakTemp_ADCU;
					p_motor->VBemfPeakTemp_ADCU = 0U;
				}

				p_motor->InterpolatedAngleIndex = 0U;
			}
			else
			{
				if (captureSpeed == true) /* Use as indicator for once per millis */
				{
					Motor_PollDeltaTStop(p_motor);
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

		if (p_motor->ControlModeFlags.Speed == 1U)
		{
			//output SpeedControl is IqReq or VqReq  =>  RampCmd - Speed always positive values direction independent
			//cannot request negative current on speed decrease
			p_motor->SpeedControl = PID_Calc(&p_motor->PidSpeed, p_motor->RampCmd, p_motor->Speed_Frac16) >> 1U;
			if (p_motor->Direction == MOTOR_DIRECTION_CW) {p_motor->SpeedControl = 0 - p_motor->SpeedControl;};
		}
	}

	FOC_SetVector(&p_motor->Foc, p_motor->ElectricalAngle);
}

static void ProcMotorFocVoltageMode(Motor_T * p_motor, qfrac16_t vqReq, qfrac16_t vdReq)
{
	qfrac16_t vqReqLimit;

	if (p_motor->Direction == MOTOR_DIRECTION_CCW)
	{
		//match pid output state on overlimit for faster response

		if (FOC_GetIq(&p_motor->Foc) > p_motor->Parameters.IqLimit)
		{
			if(p_motor->WarningFlags.IOverLimit == false)
			{
				p_motor->WarningFlags.IOverLimit = true;
				PID_SetIntegral(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
			}
		}
		else //alternatively remain set until throttle decrease
		{
			p_motor->WarningFlags.IOverLimit = false;
		}

		vqReqLimit = PID_Calc(&p_motor->PidIq, p_motor->Parameters.IqLimit, FOC_GetIq(&p_motor->Foc));
		(vqReq > vqReqLimit) ? FOC_SetVq(&p_motor->Foc, vqReqLimit) : FOC_SetVq(&p_motor->Foc, vqReq);
	}
	else
	{
		if (FOC_GetIq(&p_motor->Foc) < (int32_t)0 - (int32_t)p_motor->Parameters.IqLimit)
		{
			if(p_motor->WarningFlags.IOverLimit == false)
			{
				p_motor->WarningFlags.IOverLimit = true;
				PID_SetIntegral(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
			}
		}
		else //alternatively remain set until throttle decrease
		{
			p_motor->WarningFlags.IOverLimit = false;
		}

		vqReqLimit = PID_Calc(&p_motor->PidIq, (int32_t)0 - (int32_t)p_motor->Parameters.IqLimit, FOC_GetIq(&p_motor->Foc));
		(vqReq < vqReqLimit) ? FOC_SetVq(&p_motor->Foc, vqReqLimit) : FOC_SetVq(&p_motor->Foc, vqReq);
	}

	FOC_SetVd(&p_motor->Foc, 0);
}

/*
 * does not allow plugging
 */
static void ProcMotorFocCurrentFeedbackLoop(Motor_T * p_motor, qfrac16_t iqReq, qfrac16_t idReq)
{
	qfrac16_t vqReq;
	qfrac16_t iqReqLimited;

	qfrac16_t iqFeedback;
	qfrac16_t idFeedback;

	if(iqReq > p_motor->Parameters.IqLimit)
	{
		iqReqLimited = p_motor->Parameters.IqLimit;
	}
	else if(iqReq < ((int32_t)0 - (int32_t)p_motor->Parameters.IqLimit))
	{
		iqReqLimited = ((int32_t)0 - (int32_t)p_motor->Parameters.IqLimit);
	}
	else
	{
		iqReqLimited = iqReq;
	}

	iqFeedback = FOC_GetIq(&p_motor->Foc);
	idFeedback = FOC_GetId(&p_motor->Foc);

	vqReq = PID_Calc(&p_motor->PidIq, iqReqLimited, iqFeedback);

	if (p_motor->Direction == MOTOR_DIRECTION_CCW)
	{
		if (vqReq < 0) {vqReq = 0;} //no plugging, when req opposite current
	}
	else
	{
		if (vqReq > 0) {vqReq = 0;}
	}

	FOC_SetVq(&p_motor->Foc, vqReq);
	FOC_SetVd(&p_motor->Foc, PID_Calc(&p_motor->PidId, idReq, idFeedback));
}

static inline void ProcMotorFocControlFeedback(Motor_T * p_motor)
{
	qfrac16_t qReq;

	qReq = (p_motor->ControlModeFlags.Speed == 1U) ? p_motor->SpeedControl : p_motor->RampCmd;

	if(p_motor->ControlModeFlags.Current == 1U)
	{
		ProcMotorFocCurrentFeedbackLoop(p_motor, qReq, 0);
	}
	else
	{
//		if (p_motor->ControlModeFlags.VFreqScalar) {qReq = qReq  * p_motor->Speed_RPM* VFreqScalarFactor/  VFreqScalarDivisor;}
		ProcMotorFocVoltageMode(p_motor, qReq, 0);
	}
}

/*
 * ElectricalAngle, VPwm => Phase Duty
 */
static inline void ActivateMotorFocAngle(Motor_T * p_motor)
{
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
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
	//no current sense during pwm float, check bemf
	AnalogN_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VA);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VB);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VC);
	AnalogN_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);

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

//static inline void Motor_FOC_ResetOutput(Motor_T * p_motor)
//{
//	PID_SetIntegral(&p_motor->PidIq, 0);
//	PID_SetIntegral(&p_motor->PidId, 0);
////	FOC_SetVq(&p_motor->Foc, 0);
////	FOC_SetVd(&p_motor->Foc, 0);
//}

static inline void Motor_FOC_SetMatchOutput(Motor_T * p_motor, int32_t iq, int32_t vq)
{
	int32_t speedMatch;

	if (p_motor->ControlModeFlags.Speed == 1U)
	{
		/* Speed PID always uses directionless positive value [0:65535] */
		if(p_motor->ControlModeFlags.Current == 1U)
		{
			speedMatch = iq; //brake directly to throttle discontinuity go directly to 0
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) //does not support reverse current during speed decrease
			{
				if (speedMatch < 0)	{speedMatch = 0;}
			}
			else
			{
				if (speedMatch > 0)	{speedMatch = 0;}
			}
		}
		else
		{
			speedMatch = vq;
			if (speedMatch < 0)	{speedMatch = 0 - speedMatch;}
		}

		Motor_ResumeSpeedOutput(p_motor, speedMatch << 1U);
	}
	else /* (p_motor->ControlModeFlags.Speed == 0U) */
	{
		if (p_motor->ControlModeFlags.Current == 1U) 	{Motor_ResumeRampOutput(p_motor, iq);}
		else 											{Motor_ResumeRampOutput(p_motor, vq);}
	}

	PID_SetIntegral(&p_motor->PidIq, vq);
	PID_SetIntegral(&p_motor->PidId, 0);
}

/*
 * From Run State, match to Vq
 */
static inline void Motor_FOC_SetMatchOutputKnown(Motor_T * p_motor)
{
	/* output is prev VqReq, set to start at 0 change in torque */
	/* Iq PID output differs between voltage Iq limit mode and Iq control mode. still need to match */
	Motor_FOC_SetMatchOutput(p_motor, FOC_GetIq(&p_motor->Foc), FOC_GetVq(&p_motor->Foc));
//	Motor_FOC_SetMatchOutput(p_motor, FOC_GetIq(&p_motor->Foc), 0);
}

/*
 * From FreeWheel State, match to speed/bemf
 */
static inline void Motor_FOC_SetMatchOutputUnknown(Motor_T * p_motor)
{
	qfrac16_t vqReq;

	// match to bemf
	//todo check bemf capture available.
	/*
	 * Captured VBemfPeak always positive
	 */
//	vqReq = Linear_ADC_CalcFractionUnsigned16(&p_motor->CONFIG.UNIT_V_ABC, p_motor->VBemfPeak_ADCU) >> 1U;

	// match to speed
	// use larger SpeedRefVoltage_RPM for smaller vqReq to ensure never resume to higher speed
	 vqReq = ((int32_t)p_motor->Speed_RPM * (int32_t)32767 / (int32_t)p_motor->Parameters.SpeedRefVoltage_RPM);

	 //vspeedref greater or overflow

	if (p_motor->Direction == MOTOR_DIRECTION_CW) {vqReq = 0 - vqReq;}

	Motor_FOC_SetMatchOutput(p_motor, 0, vqReq);
}


/*
 *	from freewheel
 */
static inline void Motor_FOC_ResumeAngleControl(Motor_T * p_motor)
{
	Motor_FOC_SetMatchOutputUnknown(p_motor);
	ActivateMotorFocAngle(p_motor);
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

	PID_SetIntegral(&p_motor->PidIq, 0);
	PID_SetIntegral(&p_motor->PidId, 0);
//	FOC_SetVq(&p_motor->Foc, 0);
//	FOC_SetVd(&p_motor->Foc, 0);

	FOC_SetOutputZero(&p_motor->Foc);
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
	Phase_ActivateSwitchABC(&p_motor->Phase);

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

/******************************************************************************/
/*! @} */
/******************************************************************************/

/*
 * Call from user must also set Vector Sine/Cosine, not set during position read
 */
//static inline void Motor_FOC_ActivateAngle(Motor_T * p_motor, qangle16_t angle, qfrac16_t vq, qfrac16_t vd)
//{
////	p_motor->ElectricalAngle = angle;
//	FOC_SetVq(&p_motor->Foc, vq);
//	FOC_SetVd(&p_motor->Foc, vd);
//	FOC_SetVector(&p_motor->Foc, angle); //angle -90
//	ActivateMotorFocAngle(p_motor);
//}

#endif

