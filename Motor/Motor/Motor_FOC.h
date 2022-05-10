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
	+/- Sign indicates absolute direction, CW/CCW. NOT along or against direction selected.
	Positive is virtual CCW.
	B and Beta are virtual CCW of A/Alpha.
	Iq sign is relative to rotor direction, NOT Vq direction.

	CCW +Vq +Iq => Forward Motoring Q1
	CCW +Vq -Iq => Forward Regen Q4
	CCW -Vq -Iq => Forward Plugging

	CW -Vq -Iq => Reverse Motoring Q2
	CW -Vq +Iq => Reverse Regen Q3
	CW +Vq +Iq => Reverse Plugging
*/
#include "System/SysTime/SysTime.h"

/******************************************************************************/
/*!
	Map to Motor Analog Conversions
	Convert current from ADCU to QFrac
*/
/******************************************************************************/
static inline void Motor_FOC_CaptureIa(Motor_T * p_motor)
{
	qfrac16_t i_temp = ((int32_t)Linear_ADC_CalcFractionSigned16(&p_motor->UnitIa, p_motor->AnalogResults.Ia_Adcu) + (int32_t)FOC_GetIa(&p_motor->Foc)) / 2;
	FOC_SetIa(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_CaptureIb(Motor_T * p_motor)
{
	qfrac16_t i_temp = ((int32_t)Linear_ADC_CalcFractionSigned16(&p_motor->UnitIb, p_motor->AnalogResults.Ib_Adcu) + (int32_t)FOC_GetIb(&p_motor->Foc)) / 2;
	FOC_SetIb(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_CaptureIc(Motor_T * p_motor)
{
	qfrac16_t i_temp = ((int32_t)Linear_ADC_CalcFractionSigned16(&p_motor->UnitIc, p_motor->AnalogResults.Ic_Adcu) + (int32_t)FOC_GetIc(&p_motor->Foc)) / 2;
	FOC_SetIc(&p_motor->Foc, i_temp);

	// p_motor->DebugTime[9] = SysTime_GetMicros() - p_motor->MicrosRef;
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

static inline int32_t _Motor_FOC_CaptureAngleSpeed(Motor_T * p_motor, qangle16_t speedAngle)
{
	int32_t speedDelta = speedAngle - p_motor->SpeedAngle; /* loops if no overflow past 1 full cycle */
	int32_t speedFeedback_Frac16 = (p_motor->SpeedFeedback_Frac16 + Linear_Speed_CalcAngleRpmFrac16(&p_motor->UnitAngleRpm, speedDelta)) / 2;
		// if(p_motor->Direction == MOTOR_DIRECTION_CW) { speedDelta = 0 - speedDelta; } //alraedy signed?
	p_motor->SpeedAngle = speedAngle; /* mechanical angle */
}

static inline void _Motor_FOC_ProcPositionFeedback(Motor_T * p_motor)
{
	bool procSpeed = Timer_Poll(&p_motor->SpeedTimer);
	uint32_t electricalDelta;
	uint32_t electricalAngle;
	int32_t speedFeedback_Frac16;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			/*
				OpenLoop
				Blind input angle, constant voltage

				p_foc->Theta = integral of speed req
			*/
			/* integrate speed to angle */
			p_motor->OpenLoopSpeed_RPM = Linear_Ramp_ProcIndexOutput(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, p_motor->OpenLoopSpeed_RPM);
			electricalAngle = p_motor->ElectricalAngle + Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
			break;

		case MOTOR_SENSOR_MODE_SENSORLESS:
			electricalAngle = 0U;
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			if(procSpeed == true)
			{
				Encoder_DeltaD_Capture(&p_motor->Encoder); /* Capture position and speed */
				speedFeedback_Frac16 = Encoder_DeltaD_GetUnitSpeed(&p_motor->Encoder);
				if(p_motor->Direction == MOTOR_DIRECTION_CW) { speedFeedback_Frac16 = 0 - speedFeedback_Frac16; };
			}
			else
			{
				Encoder_DeltaD_CaptureAngle(&p_motor->Encoder); /* Capture position only */
			}
			/* Encoder_Motor_GetElectricalTheta return [0, 65535] maps directly to negative portions of qangle16_t */
			electricalAngle = (qangle16_t)Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);

			//reset peakbemf
			break;

		case MOTOR_SENSOR_MODE_HALL:
			if(Hall_PollCaptureSensors(&p_motor->Hall) == true)
			{
				Encoder_DeltaT_Capture(&p_motor->Encoder);
				Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);
				p_motor->HallAngle = (qangle16_t)Hall_GetRotorAngle_Degrees16(&p_motor->Hall);

				if(Hall_GetSensorsId(&p_motor->Hall) == 1U)
				{
					p_motor->VBemfPeak_Adcu = p_motor->VBemfPeakTemp_Adcu;
					p_motor->VBemfPeakTemp_Adcu = 0U;
				}

				electricalAngle = p_motor->HallAngle;
				p_motor->InterpolatedAngleIndex = 1U;
			}
			else
			{
				electricalDelta = Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder, p_motor->InterpolatedAngleIndex);
				if(electricalDelta > 65536U / 6U) { electricalDelta = 65536U / 6U; }
				if(p_motor->Direction == MOTOR_DIRECTION_CW) { electricalDelta = 0 - electricalDelta; };

				electricalAngle = p_motor->HallAngle + electricalDelta;
				p_motor->InterpolatedAngleIndex++;

				if(procSpeed == true) { Motor_PollDeltaTStop(p_motor); } /* Use as indicator for once per millis */
			}

			if(procSpeed == true)
			{
				speedFeedback_Frac16 = Encoder_DeltaT_GetUnitSpeed(&p_motor->Encoder);
				if(p_motor->Direction == MOTOR_DIRECTION_CW) { speedFeedback_Frac16 = 0 - speedFeedback_Frac16; };
				speedFeedback_Frac16 = (speedFeedback_Frac16 + p_motor->SpeedFeedback_Frac16) / 2;

				// speedFeedback_Frac16 = _Motor_FOC_CaptureAngleSpeed(p_motor, electricalAngle);
			}

			break;

		case MOTOR_SENSOR_MODE_SIN_COS:
			SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
			electricalAngle = SinCos_GetElectricalAngle(&p_motor->SinCos);
			if(procSpeed == true) { speedFeedback_Frac16 = _Motor_FOC_CaptureAngleSpeed(p_motor, SinCos_GetMechanicalAngle(&p_motor->SinCos)); }

			//todo observer
			break;

		default:
			electricalAngle = 0U;
			break;
	}

	if(procSpeed == true)
	{
		/*
			SpeedControl update 1000Hz, Ramp input 1000Hz, RampCmd output 20000Hz, alternatively use RampIndex += 20?
			input	RampCmd[-32767:32767], (speedFeedback_Frac16 / 2)[-32767:32767]
			output 	SpeedControl[-32767:32767] => IqReq or VqReq
		*/
		if(p_motor->FeedbackModeFlags.Speed == 1U) { p_motor->SpeedControl = PID_Calc(&p_motor->PidSpeed, p_motor->RampCmd, speedFeedback_Frac16 / 2); };
		p_motor->SpeedFeedback_Frac16 = speedFeedback_Frac16;
	}

	FOC_SetVector(&p_motor->Foc, electricalAngle);
	p_motor->ElectricalAngle = (qangle16_t)electricalAngle; /* Save for UI output */
}


static inline void _Motor_FOC_ProcVoltageMode(Motor_T * p_motor, qfrac16_t vqReq)
{
	bool isOverLimit = (p_motor->Direction == MOTOR_DIRECTION_CCW) ?
		(FOC_GetIq(&p_motor->Foc) > p_motor->ILimitVoltageMode) :
		(FOC_GetIq(&p_motor->Foc) < p_motor->ILimitVoltageMode);

	qfrac16_t vqReqOut;

	if((isOverLimit == true) && (p_motor->RunStateFlags.ILimitVoltageModeActive == false))
	{
		p_motor->RunStateFlags.ILimitVoltageModeActive = true;
		PID_SetOutputState(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
	}
	else //alternatively remain set until throttle decrease
	{
		p_motor->RunStateFlags.ILimitVoltageModeActive = false;
	}

	vqReqOut = (p_motor->RunStateFlags.ILimitVoltageModeActive == true) ?
		PID_Calc(&p_motor->PidIq, p_motor->ILimitVoltageMode, FOC_GetIq(&p_motor->Foc)) :
		vqReq;

	// if(p_motor->Direction == MOTOR_DIRECTION_CCW)
	// {
	// // 	if(vqReq < vqReqOut) { vqReqOut = vqReq; }
	// // 	// vqReqLimit = (vqReq > vqReqLimit) ? FOC_SetVq(&p_motor->Foc, vqReqLimit) : FOC_SetVq(&p_motor->Foc, vqReq);
	// // }
	// {
	// else
	// {
	// // 	if(vqReq > vqReqOut) { vqReqOut = vqReq; }
	// // 	// (vqReq < vqReqLimit) ? FOC_SetVq(&p_motor->Foc, vqReqLimit) : FOC_SetVq(&p_motor->Foc, vqReq);
	// }

	FOC_SetVq(&p_motor->Foc, vqReqOut);
	FOC_SetVd(&p_motor->Foc, 0);
}

static inline void _Motor_FOC_ProcFeedbackLoop(Motor_T * p_motor)
{
	qfrac16_t userOutput = (p_motor->FeedbackModeFlags.Speed == 1U) ? p_motor->SpeedControl : p_motor->RampCmd;

	if(p_motor->FeedbackModeFlags.Current == 1U) /* Current Control mode - proc using last adc measure */
	{
		FOC_SetVq(&p_motor->Foc, PID_Calc(&p_motor->PidIq, userOutput, 	FOC_GetIq(&p_motor->Foc)));
		FOC_SetVd(&p_motor->Foc, PID_Calc(&p_motor->PidId, 0U, 			FOC_GetId(&p_motor->Foc)));
	}
	else /* Voltage Control mode - use current feedback for over current only */
	{
		/*
			input	RampCmd[0:65535], (speedFeedback_Frac16 / 2)[-32767:32767]
			output 	[-32767:32767] => VqReq
		*/
		if(p_motor->FeedbackModeFlags.VFreqScalar == 1U) { userOutput = p_motor->RampCmd * (p_motor->SpeedFeedback_Frac16 / 2) / 65536; } //seletable SpeedControl?
		_Motor_FOC_ProcVoltageMode(p_motor, userOutput);
		// if(p_motor->RunStateFlags.ILimitVoltageModeActive == true) { userOutput = PID_Calc(&p_motor->PidIq, p_motor->ILimitVoltageMode_Frac16, FOC_GetIq(&p_motor->Foc)); }
		// FOC_SetVq(&p_motor->Foc, vqReqOut);
		// FOC_SetVd(&p_motor->Foc, 0);
	}
}

static inline void _Motor_FOC_ActivateAngle(Motor_T * p_motor)
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
//	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
//	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
//	{
//		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
//		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
//	}
//#if !defined(CONFIG_MOTOR_V_SENSORS_ISOLATED) &&defined(CONFIG_MOTOR_V_SENSORS_ADC)
////	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
////	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
////	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
//#endif
//	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);

	_Motor_FOC_ProcPositionFeedback(p_motor);
}

//static inline void Motor_FOC_StartAngleObserve(Motor_T * p_motor)
//{
////	p_motor->IOverLimitFlag = false;
//}

/*
	StateMachine calls each PWM, ~20kHz
	shared function -> less states
 */
static inline void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{

	AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
	//	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
	//	{
	//		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
	//		AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
	//	}
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
#if defined(CONFIG_MOTOR_I_SENSORS_ABC) && !defined(CONFIG_MOTOR_I_SENSORS_AB)
	AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
#endif
	AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);

	/* ~10us */
	// p_motor->DebugTime[1] = SysTime_GetMicros() - p_motor->MicrosRef;

	//samples complete when queue resumes, adc isr priority higher than pwm.
	_Motor_FOC_ProcPositionFeedback(p_motor);

	/* Ic complete ~29 us */

	/* ~29 us */
	// p_motor->DebugTime[2] = SysTime_GetMicros() - p_motor->MicrosRef;

#ifdef CONFIG_MOTOR_I_SENSORS_AB
	FOC_ProcClarkePark_AB(&p_motor->Foc);
#elif defined(CONFIG_MOTOR_I_SENSORS_ABC)
	FOC_ProcClarkePark(&p_motor->Foc);
#endif

	/* ~30us */
	// p_motor->DebugTime[3] = SysTime_GetMicros() - p_motor->MicrosRef;

	// if(p_motor->RunStateFlags.Hold == 0U)
	{
		_Motor_FOC_ProcFeedbackLoop(p_motor);
		_Motor_FOC_ActivateAngle(p_motor);
	}

	/* ~37us */
	// p_motor->DebugTime[4] = SysTime_GetMicros() - p_motor->MicrosRef;

}


static inline void Motor_FOC_SetMatchOutput(Motor_T * p_motor, int32_t iq, int32_t vq, int32_t vd)
{
	int32_t speedControl = (p_motor->FeedbackModeFlags.Current == 1U) ? iq : vq;

	if(p_motor->FeedbackModeFlags.VFreqScalar != 1U)
	{
		(p_motor->FeedbackModeFlags.Speed == 1U) ?
			Motor_SetSpeedOutput(p_motor, speedControl) :
			Motor_SetRampOutput(p_motor, speedControl);
	}
	else
	{
		Motor_SetRampOutput(p_motor, 65536); //start from scalar of 1, output speed
	}

	PID_SetIntegral(&p_motor->PidIq, vq);
	PID_SetIntegral(&p_motor->PidId, vd);
}


/*
	from freewheel
*/
static inline void Motor_FOC_ResumeAngleControl(Motor_T * p_motor)
{
	int32_t vqReq;

	/*
		From FreeWheel State, match to speed/bemf
	*/
	 /*
		Match to Bemf //todo check bemf capture available.
		Captured VBemfPeak always positive
	*/
	  //	vqReq = Linear_ADC_CalcFractionUnsigned16(&p_motor->CONFIG.UNIT_V_ABC, p_motor->VBemfPeak_Adcu) >> 1U;

	/*
		Match to Speed
		User sets larger SpeedRefVBemf_Rpm for smaller vqReq to ensure never resume to higher speed
		vqReq = Speed_RPM * 32768 / SpeedRefVBemf_Rpm

		SpeedFeedback_Frac16 = Speed_RPM * 65536 /  SpeedRefMax_Rpm, todo account for SpeedRefVBemf_Rpm
	*/
	// vqReq = p_motor->SpeedFeedback_Frac16 * p_motor->Parameters.SpeedRefMax_Rpm / p_motor->Parameters.SpeedRefVBemf_Rpm / 2U;
	// if(vqReq > 32767) { vqReq = 32767; }
	// else if(vqReq < -32767) { vqReq = -32767; }

	Motor_FOC_SetMatchOutput(p_motor, 0, p_motor->SpeedFeedback_Frac16 / 2, 0);

	_Motor_FOC_ActivateAngle(p_motor);
	Phase_ActivateSwitchABC(&p_motor->Phase); /* Switches Disabled when entering freewheel State */
}

/*
	from stop
*/
static inline void Motor_FOC_StartAngleControl(Motor_T * p_motor)
{
	FOC_Zero(&p_motor->Foc);
	Motor_FOC_SetMatchOutput(p_motor, FOC_GetIq(&p_motor->Foc), FOC_GetVq(&p_motor->Foc), FOC_GetVd(&p_motor->Foc));
	Phase_ActivateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
	Phase_ActivateSwitchABC(&p_motor->Phase);

	//move to align for regularity?
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			p_motor->OpenLoopRampIndex = 0U;
			break;

		case MOTOR_SENSOR_MODE_SENSORLESS:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			Encoder_Zero(&p_motor->Encoder);
			break;

		case MOTOR_SENSOR_MODE_HALL:
			Encoder_Zero(&p_motor->Encoder); //zero angle speed //reset before Encoder_DeltaT_SetInitial
			Encoder_DeltaT_SetInitial(&p_motor->Encoder, 10U); /* Set first capture DeltaT = 0xffff */
			Hall_ResetCapture(&p_motor->Hall);
			break;

		case MOTOR_SENSOR_MODE_SIN_COS:
			break;

		default:
			break;
	}
}




/******************************************************************************/
/*! @} */
/******************************************************************************/
static inline uint16_t Motor_FOC_GetIMagnitude_Frac16(Motor_T * p_motor)
{
	return FOC_GetIMagnitude(&p_motor->Foc) * 2;
}

/*
 * Call from user must also set Vector Sine/Cosine, not set during position read
 */
 //static inline void Motor_FOC_ActivateAngle(Motor_T * p_motor, qangle16_t angle, qfrac16_t vq, qfrac16_t vd)
 //{
 ////	p_motor->ElectricalAngle = angle;
 //	FOC_SetVq(&p_motor->Foc, vq);
 //	FOC_SetVd(&p_motor->Foc, vd);
 //	FOC_SetVector(&p_motor->Foc, angle); //angle -90
 //	_Motor_FOC_ActivateAngle(p_motor);
 //}

extern void Motor_FOC_SetOutputLimitsCcw(Motor_T * p_motor);
extern void Motor_FOC_SetOutputLimitsCw(Motor_T * p_motor);
extern void Motor_FOC_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_FOC_SetDirectionCw(Motor_T * p_motor);
extern void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);

#endif
