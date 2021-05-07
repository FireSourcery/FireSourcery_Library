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
#include "Default.h"
#include "Config.h"

#include "Math/FOC.h"

#include "Transducer/Encoder/Encoder_IO.h"
#include "Transducer/Encoder/Encoder_Motor.h"
//#include "Transducer/Encoder/Encoder.h"

#include <stdint.h>
#include <stdbool.h>


/******************************************************************************/
/*!
	@addtogroup FocAlignGroup
	FOC Align functions
	@{
*/
/******************************************************************************/
static inline void Motor_FOC_Zero(Motor_T * p_motor)
{
	Encoder_Zero(&p_motor->Encoder);
	FOC_SetZero(&p_motor->Foc);
}

/*
	FOC read returns angle
 */
//static inline qfrac16_t Motor_FOC_ReadSensor(Motor_T *p_motor)
//{
//	switch (p_motor->SensorMode)
//	{
//	case MOTOR_SENSOR_MODE_ENCODER:
//
//		break;
//
//	case MOTOR_SENSOR_MODE_BEMF:
//
//		break;
//
//	case MOTOR_SENSOR_MODE_HALL:
//
//		break;
//
//	default:
//		break;
//	}
//}


/******************************************************************************/
/*! @} */
/******************************************************************************/

/*
 * prep run state. shared to reduce number of states
 */
static inline void Motor_FOC_StartAngleControl(Motor_T * p_motor)
{
	Motor_FOC_Zero(p_motor);
	Phase_SetDutyCyle_15(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output
	Phase_SetState(&p_motor->Phase,  true,  true,  true);

	p_motor->VCmd = 0;

	switch (p_motor->ControlMode)
	{
	case MOTOR_CONTROL_MODE_OPEN_LOOP:
		//		//Linear_Ramp_Init(&p_motor->Ramp, pollingFreq, setpoint);
		//		//Linear_Ramp_Init(&p_motor->Ramp, freq, start, end, accerleration);
		p_motor->Speed_RPM = 60;

		break;
	case  MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
		p_motor->Speed_RPM = 0;

//		case CONTROL_MODE_VOLTAGE:
//
//		case CONTROL_MODE_SPEED_VOLTAGE:
//
//		case CONTROL_MODE_OPEN_LOOP:
//			//Linear_Ramp_Init(&p_motor->Ramp, pollingFreq, setpoint);
//			//Linear_Ramp_Init(&p_motor->Ramp, freq, start, end, accerleration);
//			break;

	default:
		break;
	}

	/*
	 * Common
	 */
	//p_motor->RampIndex = 0;

	//align already set theta to zero
}

/*
	StateMachine calls each PWM, ~20kHz
	shared function -> less states
 */
extern Analog_Conversion_T FOC_ANALOG_CONVERSION_ANGLE_CONTROL;

static inline void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
	//check local fault

	if ((p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) || (p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT))
	{
		Analog_ActivateConversion(&p_motor->Analog, &FOC_ANALOG_CONVERSION_ANGLE_CONTROL);
	}
	else
	{
		if (p_motor->ControlMode == MOTOR_CONTROL_MODE_OPEN_LOOP)
		{
			/* integrate speed to angle */
			//	speed_RPM = Linear_Ramp_ConvertIndex(&p_motor->Ramp, p_motor->RampIndex);
			//	p_motor->Speed_RPM = Linear_Ramp_ProcIndex(&p_motor->Ramp, &p_motor->RampIndex);
			p_motor->ElectricalAngle += Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(&p_motor->Encoder, p_motor->Speed_RPM);
			p_motor->VCmd = DEFAULT_FOC_OPEN_LOOP_VQ/2; //todo prop to throttle
		}
		else /* Position Feedback */
		{
			switch (p_motor->SensorMode)
			{
			case MOTOR_SENSOR_MODE_OPEN_LOOP:
				break;

			case MOTOR_SENSOR_MODE_ENCODER:
				Encoder_CaptureDeltaD_IO(&p_motor->Encoder);
				//	p_motor->Speed_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder); //Always calc speed?
				p_motor->ElectricalAngle = Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);  //need to save theta?
				break;

			case MOTOR_SENSOR_MODE_HALL:
				Encoder_PollDeltaT_IO(&p_motor->Encoder); //true for 1 hall edge
				//	p_motor->Speed_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder); //Always calc speed?

				if (Hall_PollSector_IO(&p_motor->Hall))
				{
					p_motor->ElectricalAngle = Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
				}
				else
				{
					p_motor->ElectricalAngle += Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder, p_motor->InterpolatedAngleIndex);
					p_motor->InterpolatedAngleIndex++;
				}
				break;

			case MOTOR_SENSOR_MODE_BEMF:

				break;

			default:
				break;
			}

			if (p_motor->ControlMode == MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ)
			{
				//p_motor->VCmd = p_motor->Speed_RPM * p_motor->VRpmGain;
			}
			else /* (p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE) || (p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE) */
			{
				if (p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE)
				{
					Motor_PollSpeedLoop(p_motor);
					p_motor->VCmd = 00000; //get from PID

				}
				else /*(p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE) */
				{
					p_motor->VCmd = DEFAULT_FOC_OPEN_LOOP_VQ/2; //todo prop to throttle
				}
			}
		}

		//FOC_SetVd(&p_motor->Foc, 000000);
		FOC_SetVq(&p_motor->Foc, p_motor->VCmd);
		FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);
		FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
		Phase_SetDutyCyle_15(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
		Phase_Actuate(&p_motor->Phase);
	}
}

/*
 * Current PID Feedback Loop
 *
 *	Input by  pointer:
 *	motor->IdReq
 *	motor->IqReq
 *	p_foc->Id
 *	p_foc->Iq
 *
 *	output by pointer:
 *	p_foc->Vd
 *	p_foc->Vq
 */
static inline void Motor_FOC_ProcCurrentLoop(Motor_T * p_motor)
{
	FOC_ProcClarkePark(&p_motor->Foc);
//	PID_Proc_ISR(&PIDd);
//	PID_Proc_ISR(&PIDq);
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
}


/*
 * ADC conversion complete call
 */
static inline void Motor_FOC_ProcCurrentControl(Motor_T * p_motor)
{
	Encoder_CaptureDeltaD_IO(&p_motor->Encoder);
//	p_motor->Speed_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder); //Always calc speed?
	p_motor->ElectricalAngle = Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);  //need to save theta?
	FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);

	if(p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
	{
		Motor_PollSpeedLoop(p_motor);
	} /* else (p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) */

	//	Motor_FOC_ProcCurrentLoop(p_motor);
	FOC_ProcClarkePark(&p_motor->Foc);
	//	PID_Proc_ISR(&PIDd);
	//	PID_Proc_ISR(&PIDq);
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_SetDutyCyle_15(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
	Phase_Actuate(&p_motor->Phase); //Phase_Actuate_ABC(&p_motor->Phase, a,b,c);
}

#endif
