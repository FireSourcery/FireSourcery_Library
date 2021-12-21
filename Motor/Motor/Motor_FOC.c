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
    @version V0
*/
/**************************************************************************/

#include "Motor_FOC.h"
#include "Motor.h"

#include "Config.h"
//#include "Default.h"

//#include "Transducer/Encoder/Encoder_Motor.h"
//#include "Transducer/Encoder/Encoder.h"

#include "Transducer/Phase/Phase.h"

#include "Utility/Timer/Timer.h"

//#include "Math/Q/QFrac16.h"
#include "Math/Linear/Linear_ADC.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Map to Motor Analog Conversions
 */
void Motor_FOC_CaptureIa(Motor_T *p_motor)
{
	//Filter here if needed
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIa, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_IA]);
	FOC_SetIa(&p_motor->Foc, i_temp);
}

void Motor_FOC_CaptureIb(Motor_T *p_motor)
{
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIb, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_IB]);
	FOC_SetIb(&p_motor->Foc, i_temp);
}

void Motor_FOC_CaptureIc(Motor_T *p_motor)
{
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIc, p_motor->AnalogResults[MOTOR_ANALOG_CHANNEL_IC]);
	FOC_SetIc(&p_motor->Foc, i_temp);
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
void Motor_FOC_ProcCurrentFeedback(Motor_T * p_motor)
{
	switch (p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_BEMF:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			Encoder_DeltaD_Capture(&p_motor->Encoder);
			p_motor->ElectricalAngle = Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);
			break;

		case MOTOR_SENSOR_MODE_HALL:
			break;
	}
//
//	if(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
//	{
//		Motor_PollSpeedFeedback(p_motor);
//	}
//	/* else (p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) */
//
//	//	Motor_FOC_ProcCurrentLoop(p_motor);
//	FOC_ProcClarkePark(&p_motor->Foc);
//	//	PID_Proc (&PIDd);
//	//	PID_Proc (&PIDq);
//
//	// p_motor->VCmd =  get from PID
//
	Motor_ProcRamp(p_motor);
	Motor_ProcControlVariable(p_motor); //input usercmd rampcmd, set vpwm
	_Motor_FOC_ActivateAngle(p_motor); 	//input vpwm, theta
}


/*
 * Maps to state machine, move to Motor.c?
 * Calibrate Current ADC
 */
void Motor_FOC_StartCalibrateAdc(Motor_T * p_motor)
{
	Timer_SetPeriod(&p_motor->ControlTimer, 20000U); // Motor.Parameters.AdcCalibrationTime
	Phase_Ground(&p_motor->Phase); //activates abc
	FOC_SetZero(&p_motor->Foc);
	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output
}

bool Motor_FOC_CalibrateAdc(Motor_T *p_motor)
{
	return Motor_CalibrateAdc(p_motor);
}
