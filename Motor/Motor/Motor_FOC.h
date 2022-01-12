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

			if (captureSpeed == true)
			{
				if (Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true)
				{
					p_motor->Speed_RPM = 0U;
				}
			}
			break;

		default:
			break;
	}


}
/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline qfrac16_t GetMotorFocQReq(Motor_T * p_motor)
{
	qfrac16_t qReq;

	if((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE))
	{
//		Motor_PollSpeedFeedbackLoop(p_motor);
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

	return qReq;
}

/*
 * ElectricalAngle, VPwm => Phase Duty
 */
static inline void ActivateMotorFocAngle(Motor_T * p_motor, qfrac16_t vqReq, qfrac16_t vdReq)
{
	FOC_SetVq(&p_motor->Foc, vqReq);
	FOC_SetVd(&p_motor->Foc, vdReq);

	FOC_SetVector(&p_motor->Foc, p_motor->ElectricalAngle);
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
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
 * Current feedback angle control
 * ADC conversion complete call
 */
static inline void Motor_FOC_ProcCurrentFeedback(Motor_T * p_motor)
{
	qfrac16_t vqReq;
	qfrac16_t vdReq;
	qfrac16_t iqReq;
	qfrac16_t idReq;

//	ProcMotorFocPositionFeedback(p_motor);

	iqReq = GetMotorFocQReq(p_motor);
	idReq = 0; //p_motor->FieldWeakening

	vqReq = PID_Calc(&p_motor->PidIq, iqReq, FOC_GetIq(&p_motor->Foc));
	vdReq = PID_Calc(&p_motor->PidId, idReq, FOC_GetId(&p_motor->Foc));

	ActivateMotorFocAngle(p_motor, vqReq, vdReq);
}

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
	//Filter here
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIa, p_motor->AnalogResults.Ia_ADCU);
	FOC_SetIa(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_CaptureIb(Motor_T *p_motor)
{
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIb, p_motor->AnalogResults.Ib_ADCU);
	FOC_SetIb(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_CaptureIc(Motor_T *p_motor)
{
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIc, p_motor->AnalogResults.Ic_ADCU);
	FOC_SetIc(&p_motor->Foc, i_temp);

	//todo oncomplete signal flag
	FOC_ProcClarkePark(&p_motor->Foc);

	if (((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)) == true)
	{
		Motor_FOC_ProcCurrentFeedback(p_motor);
	}
}

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
	StateMachine calls each PWM, ~20kHz
	shared function -> less states
 */
static inline void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
	qfrac16_t vqReq;
	qfrac16_t vdReq;

	uint16_t magIq;
	/* Current Control Mode - Proc angle on ADC return */
	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
	ProcMotorFocPositionFeedback(p_motor);

	if (((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE)) == true)
	{
		/* Voltage Control mode - Proc angle immediately */
//		ProcMotorFocPositionFeedback(p_motor);

		if(FOC_GetIq(&p_motor->Foc) < 0)
		{
			magIq = 0 - FOC_GetIq(&p_motor->Foc);
		}
		else
		{
			magIq = FOC_GetIq(&p_motor->Foc);
		}

		if (magIq > 32768 * 9 / 10)
		{
			vqReq = PID_Calc(&p_motor->PidIq, 32768 * 9 / 10, magIq);
		}
		else
		{
			PID_Calc(&p_motor->PidIq, 32768 * 9 / 10, magIq);
			vqReq = GetMotorFocQReq(p_motor);
		}

		vdReq = 0; //p_motor->FieldWeakening
		Debug_CaptureElapsed(7);
		ActivateMotorFocAngle(p_motor, vqReq, vdReq);
		Debug_CaptureElapsed(8);
	}
}

static inline void Motor_FOC_ResumeAngleControl(Motor_T * p_motor)
{
	PID_Reset(&p_motor->PidSpeed);
	PID_Reset(&p_motor->PidIq);
	PID_Reset(&p_motor->PidId);

	Phase_ActivateSwitchABC(&p_motor->Phase);

	switch (p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			//from stop only
			//can start at 0 speed in foc mode for continuous angle displacements
			Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 20000U, 0U, 200U, 2000U);
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
				//Linear_Ramp_Init(&p_motor->Ramp, freq, start, end, accerleration);
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
	Encoder_Reset(&p_motor->Encoder); //zero angle speed

	Timer_StartPeriod(&p_motor->MillisTimer, 1U);

	PID_Reset(&p_motor->PidSpeed);
	PID_Reset(&p_motor->PidIq);
	PID_Reset(&p_motor->PidId);

//	Phase_Ground(&p_motor->Phase); //activates abc
	FOC_SetZero(&p_motor->Foc);
	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output

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
//extern void Motor_FOC_StartCalibrateAdc(Motor_T * p_motor);
//extern bool Motor_FOC_CalibrateAdc(Motor_T *p_motor);

/*
 * Calibrate Current ADC
 */
static inline void Motor_FOC_StartCalibrateAdc(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, 100000U); // Motor.Parameters.AdcCalibrationTime
	Phase_Ground(&p_motor->Phase); //activates abc
	FOC_SetZero(&p_motor->Foc);
	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output

//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VA);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VB);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VC);

	p_motor->AnalogResults.Ia_ADCU = 2000U;
	p_motor->AnalogResults.Ia_ADCU = 2000U;
	p_motor->AnalogResults.Ia_ADCU = 2000U;

	Filter_MovAvg_InitN(&p_motor->FilterA, 2000U, 100U);
	Filter_MovAvg_InitN(&p_motor->FilterB, 2000U, 100U);
	Filter_MovAvg_InitN(&p_motor->FilterC, 2000U, 32U);
}

static inline bool Motor_FOC_CalibrateAdc(Motor_T *p_motor)
{
	bool isComplete = Timer_Poll(&p_motor->ControlTimer);

	if (isComplete == true)
	{
		Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
		Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
		Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
		Phase_Float(&p_motor->Phase);
//		save params
	}
	else
	{
		p_motor->Parameters.IaZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
		p_motor->Parameters.IbZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
		p_motor->Parameters.IcZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);

		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
	}

	return isComplete;
}

#endif



