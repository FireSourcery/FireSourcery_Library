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
	bool captureSpeed = Timer_Poll(&p_motor->SpeedTimer);
	int32_t speedControl;
	uint32_t electricalDelta;

	switch (p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			/*
			 * OpenLoop
			 * Blind input angle, constant voltage
			 *
			 * p_foc->Theta = integral of speed req
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
				Motor_CaptureEncoderSpeed(p_motor);
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

			if (captureSpeed == true)
			{
				Motor_CaptureEncoderSpeed(p_motor);
			}

			electricalDelta = Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder, p_motor->InterpolatedAngleIndex); //todo add boundary, use ref index
			if (electricalDelta > 65536U/6U) {electricalDelta = 65536U/6U;}

			if (p_motor->Direction == MOTOR_DIRECTION_CW) {electricalDelta = 0 - electricalDelta;};
			p_motor->ElectricalAngle = p_motor->HallAngle + electricalDelta;
			p_motor->InterpolatedAngleIndex++;
			break;

		case MOTOR_SENSOR_MODE_SIN_COS:
			p_motor->ElectricalAngle = SinCos_CalcAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_ADCU, p_motor->AnalogResults.Cos_ADCU);
			if (captureSpeed == true)
			{
				Motor_CaptureSpeed(p_motor);
			}
			break;

		default:
			break;
	}

	if (captureSpeed == true)
	{
		Motor_CaptureSpeed(p_motor);
		if (p_motor->ControlModeFlags.Speed == 1U)
		{
			/*
			 * 	input	RampCmd, Speed, always positive values direction independent
			 * 	output 	SpeedControl is signed IqReq or VqReq
			 */
			speedControl = PID_Calc(&p_motor->PidSpeed, p_motor->RampCmd, p_motor->Speed_Frac16) >> 1U;

			if (p_motor->Direction == MOTOR_DIRECTION_CW) {speedControl = 0 - speedControl;};

			if(p_motor->ControlModeFlags.Current == 0U) //speed control is Vq
			{
				if (p_motor->Direction == MOTOR_DIRECTION_CCW)
				{
					if (speedControl < 0) {speedControl = 0;} //no plugging
				}
				else
				{
					if (speedControl > 0) {speedControl = 0;}
				}
			}

			p_motor->SpeedControl = speedControl;
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
	qfrac16_t iqReqNew;

	//todo throttle limit, brake limit
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

	vqReq = PID_Calc(&p_motor->PidIq, iqReqNew, FOC_GetIq(&p_motor->Foc));

	if (p_motor->Direction == MOTOR_DIRECTION_CCW)
	{
		if (vqReq < 0) {vqReq = 0;} //no plugging, when req opposite current
	}
	else
	{
		if (vqReq > 0) {vqReq = 0;}
	}

	FOC_SetVq(&p_motor->Foc, vqReq);
	FOC_SetVd(&p_motor->Foc, PID_Calc(&p_motor->PidId, idReq, FOC_GetId(&p_motor->Foc)));
}

static inline void ProcMotorFocControlFeedback(Motor_T * p_motor)
{
	qfrac16_t qReq;

	qReq = (p_motor->ControlModeFlags.Speed == 1U) ? p_motor->SpeedControl : p_motor->RampCmd;

	if(p_motor->ControlModeFlags.Current == 1U)		/* Current Control Mode - Proc angle using last adc measured*/
	{
		ProcMotorFocCurrentFeedbackLoop(p_motor, qReq, 0);
	}
	else  		/* Voltage Control mode - use current feedback for over current only */
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
	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
	{
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
	}
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
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
	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
	{
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
	}
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
	AnalogN_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);

	//samples complete when queue resumes, adc isr priority higher than pwm.
	ProcMotorFocPositionFeedback(p_motor);

//	p_motor->DebugTime[1] = SysTime_GetMicros() - p_motor->MicrosRef;
	FOC_ProcClarkePark(&p_motor->Foc);
//	p_motor->DebugTime[2] = SysTime_GetMicros() - p_motor->MicrosRef;

	if(p_motor->ControlModeFlags.Hold == 0U)
	{
		ProcMotorFocControlFeedback(p_motor);
		ActivateMotorFocAngle(p_motor);
	}
//	p_motor->DebugTime[3] = SysTime_GetMicros() - p_motor->MicrosRef;
}


static inline void Motor_FOC_SetMatchOutput(Motor_T * p_motor, int32_t iq, int32_t vq, int32_t vd)
{
	int32_t control = (p_motor->ControlModeFlags.Current == 1U) ? iq : vq;

	if (p_motor->ControlModeFlags.Speed == 1U)
	{
		Motor_ResumeSpeedOutput(p_motor, control * 2U);  /* Speed PID, SpeedControl, is signed [-65535:65535] */
	}
	else /* (p_motor->ControlModeFlags.Speed == 0U) */
	{
		Motor_ResumeRampOutput(p_motor, control);
	}

	PID_SetIntegral(&p_motor->PidIq, vq);
	PID_SetIntegral(&p_motor->PidId, vd);
}

/*
 * From Run State, match to Vq
 */
static inline void Motor_FOC_SetMatchOutputKnown(Motor_T * p_motor)
{
	/* output is prev VqReq, set to start at 0 change in torque */
	/* Iq PID output differs between voltage Iq limit mode and Iq control mode. still need to match */
	Motor_FOC_SetMatchOutput(p_motor, FOC_GetIq(&p_motor->Foc), FOC_GetVq(&p_motor->Foc), FOC_GetVd(&p_motor->Foc));
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

	 //todo saturate vq if speed ref is speed is greater than speedref

	if (p_motor->Direction == MOTOR_DIRECTION_CW) {vqReq = 0 - vqReq;}

	Motor_FOC_SetMatchOutput(p_motor, 0, vqReq, 0);
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
 *	from stop
 */
static inline void Motor_FOC_StartAngleControl(Motor_T * p_motor)
{
	Encoder_Reset(&p_motor->Encoder); //zero angle speed //reset before Encoder_DeltaT_SetInitial
	FOC_Reset(&p_motor->Foc);
	Motor_FOC_SetMatchOutputKnown(p_motor);
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

