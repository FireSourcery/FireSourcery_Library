/**************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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

#include "Math/FOC.h"

#include "Transducer/Encoder/Encoder_IO.h"

#include "Transducer/Encoder/Encoder_Motor.h"
//#include "Transducer/Encoder/Encoder.h"

#include "Config.h"

#include <stdbool.h>

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
//	//qfract16_limitvector_xy(&p_foc->Vd, &p_foc->Vq, p_foc->vectorMax, sqrt13);
}

/******************************************************************************/
/*!
	@addtogroup FocAlignGroup
	FOC Align functions
	@{
*/
/******************************************************************************/

/*!
	Called by StateMachine wrapper
 */
static inline void Motor_FOC_ActivateAlign(Motor_T * p_motor)
{
	FOC_SetAlign(&p_motor->Foc, p_motor->Parameters.FocAlignVd); /* 5-10 percent*/
	Phase_Actuate(&p_motor->Phase);

	//should this be in state wrapper?
	p_motor->Timer_ControlFreq = CONFIG_MOTOR_CONTROL_FREQ; /* set timer for 1 second */

}

static inline void Motor_FOC_SetZeros(Motor_T * p_motor)
{
	Encoder_Reset(&p_motor->Encoder);
	FOC_SetZero(&p_motor->Foc);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

/*
 * prep run state. shared to reduce number of states
 */
static inline void Motor_FOC_SetRunMode(Motor_T * p_motor)
{
//	switch (p_motor->SensorMode)
//	{
//	case MOTOR_SENSOR_MODE_ENCODER:
//		break;
//
//	case MOTOR_SENSOR_MODE_BEMF:
//		break;
//
//	case MOTOR_SENSOR_MODE_OPEN_LOOP:
//		break;
//
//	default:
//		break;
//	}
//
//	switch (p_motor->FeedbackMode)
//	{
//	case CONTROL_MODE_SPEED:
//
//		break;
//
//	case CONTROL_MODE_VOLTAGE:
//
//	case CONTROL_MODE_SPEED_VOLTAGE:
//
//	case CONTROL_MODE_OPEN_LOOP:
//		//Linear_Ramp_Init(&p_motor->Ramp, pollingFreq, setpoint);
//		//Linear_Ramp_Init(&p_motor->Ramp, freq, start, end, accerleration);
//		break;
//
//
//	default:
//		break;
//	}

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

static inline void Motor_FOC_ActivateAngleControl(Motor_T * p_motor)
{
	//check local fault

	if ((p_motor->FocMode == MOTOR_FOC_MODE_CONSTANT_CURRENT) || (p_motor->FocMode == MOTOR_FOC_MODE_CONSTANT_SPEED_CURRENT))
	{
		Analog_ActivateConversion(&p_motor->Analog, &FOC_ANALOG_CONVERSION_ANGLE_CONTROL);
	}
	else
	{
		if (p_motor->FocMode == MOTOR_FOC_MODE_OPENLOOP)
		{
			/* integrate speed to angle */
			//	uint16_t speedRPM = Linear_Ramp_ConvertIndex(&p_motor->Ramp, p_motor->RampIndex);
			//	p_motor->Speed_RPM = Linear_Ramp_ProcIndex(&p_motor->Ramp, &p_motor->RampIndex);
			p_motor->ElectricalAngle += Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(&p_motor->Encoder, p_motor->Speed_RPM);
			//FOC_SetVd(&p_motor->Foc, 0);
			FOC_SetVq(&p_motor->Foc, p_motor->Parameters.FocOpenLoopVq);
		}
		else /* Position Feedback */
		{
			Encoder_CaptureDeltaD_IO(&p_motor->Encoder);

			p_motor->Speed_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder); //Always calc speed?
			p_motor->ElectricalAngle = Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);  //need to save theta?

			FOC_SetVector(&p_motor->Foc, p_motor->ElectricalAngle);

			if (p_motor->FocMode == MOTOR_FOC_MODE_SCALAR_VOLTAGE_FREQ)
			{
				//	FOC_SetVq(p_motor->Foc, p_motor->Speed_RPM * p_motor->VRpmGain);
			}
			else /* (p_motor->FocMode == MOTOR_FOC_MODE_CONSTANT_SPEED_VOLTAGE) || (p_motor->FocMode == MOTOR_FOC_MODE_CONSTANT_VOLTAGE) */
			{
				if (p_motor->FocMode == MOTOR_FOC_MODE_CONSTANT_SPEED_VOLTAGE)
				{
					Motor_PollSpeedLoop(p_motor); /* SpeedLoop sets Vq by pointer */
				}
				else /*(p_motor->FocMode == MOTOR_FOC_MODE_CONSTANT_VOLTAGE) */
				{
					//FOC_SetVd(p_motor->FOC, 0);
					FOC_SetVq(&p_motor->Foc, p_motor->VCmd);
				}
			}
		}

		FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
		Phase_Actuate(&p_motor->Phase);	//Phase_Actuate_ABC(&p_motor->Phase, a,b,c);
	}
}


/*
 * ADC conversion complete call
 */
static inline void Motor_FOC_ActivateCurrentFeedback(Motor_T * p_motor)
{
	if(p_motor->FocMode == MOTOR_FOC_MODE_CONSTANT_SPEED_CURRENT)
	{
		p_motor->Speed_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
		Motor_PollSpeedLoop(p_motor);

	} /* else (p_motor->FocMode == MOTOR_FOC_MODE_CONSTANT_CURRENT) */

	FOC_ProcClarkePark(&p_motor->Foc);
	Motor_FOC_ProcCurrentLoop(p_motor);
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_Actuate(&p_motor->Phase); //Phase_Actuate_ABC(&p_motor->Phase, a,b,c);
}





#endif

