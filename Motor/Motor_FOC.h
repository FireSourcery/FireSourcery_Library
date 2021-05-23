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
#include "Transducer/Encoder/Encoder.h"

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

/******************************************************************************/
/*! @} */
/******************************************************************************/

static inline void Motor_FOC_ActivateAngle(Motor_T * p_motor)
{
	//FOC_SetVd(&p_motor->Foc, 000000);
	FOC_SetVq(&p_motor->Foc, p_motor->VCtrl);
	FOC_SetTheta(&p_motor->Foc, p_motor->ElectricalAngle);
	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
	Phase_SetDutyCyle_15(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc));
	Phase_Actuate(&p_motor->Phase);
}

/*
 * prep run state. shared to reduce number of states
 */
static inline void Motor_FOC_SetAngleControl(Motor_T * p_motor)
{
	Encoder_Zero(&p_motor->Encoder);
	FOC_SetZero(&p_motor->Foc);
	Phase_SetDutyCyle_15(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output
	Phase_SetState(&p_motor->Phase, true, true, true);

	p_motor->RampIndex = 0U;
	p_motor->VCtrl = 0U;
	p_motor->SpeedReq_RPM = 0U;

	switch (p_motor->SensorMode)
	{
	case MOTOR_SENSOR_MODE_OPEN_LOOP:
		//		//Linear_Ramp_Init(&p_motor->Ramp, freq, start, end, accerleration);
		Linear_Ramp_InitMillis(&p_motor->Ramp, 0U, 10U, 1000U, 20000U); //can start at 0 for speed for continuous angle displacements
		p_motor->CommutationPeriodCmd = 0U;
		break;

	case MOTOR_SENSOR_MODE_ENCODER:

		break;

	case MOTOR_SENSOR_MODE_BEMF:

		break;

	case MOTOR_SENSOR_MODE_HALL:

		break;

	default:
		break;
	}

//	switch (p_motor->ControlMode)
//	{
//	case MOTOR_CONTROL_MODE_OPEN_LOOP:
//		//		//Linear_Ramp_Init(&p_motor->Ramp, freq, start, end, accerleration);
//		Linear_Ramp_InitMillis(&p_motor->Ramp, 0U, 10U, 1000U, 20000U); //must start at sufficent speed for fixed angle displacements
//		p_motor->CommutationPeriodCmd = 0U;
//		break;
//
//	case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
//		break;
//
//	case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
//		break;
//
//	default:
//		break;
//
//	}

}


/*
	StateMachine calls each PWM, ~20kHz
	shared function -> less states
 */
//extern Analog_Conversion_T FOC_ANALOG_CONVERSION_ANGLE_CONTROL;

static inline void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
	//check local fault
	p_motor->VReq = Linear_ADC_CalcUnsignedFraction16(&p_motor->UnitThrottle, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_THROTTLE]);

	if ((p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) || (p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT))
	{
//		Analog_ActivateConversion(&p_motor->Analog, &FOC_ANALOG_CONVERSION_ANGLE_CONTROL);
		//Proc on ADC Return
	}
	else
	{
		switch (p_motor->SensorMode)
		{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			/* OpenLoop
			 * Blind input angle, constant voltage
			 *
			 * 	p_foc->Theta = integral of speed req
			 */
			/* integrate speed to angle */
			p_motor->SpeedReq_RPM = Linear_Ramp_CalcTarget_IncIndex(&p_motor->Ramp, &p_motor->RampIndex, 1U);
			p_motor->ElectricalAngle += Encoder_Motor_ConvertMechanicalRpmToElectricalDelta(&p_motor->Encoder, p_motor->SpeedReq_RPM);
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			Encoder_CaptureDeltaD_IO(&p_motor->Encoder);
//			p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder); //Always calc speed?
			p_motor->ElectricalAngle = Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);  //need to save theta?
			break;

		case MOTOR_SENSOR_MODE_HALL:
			if (Hall_PollSector_IO(&p_motor->Hall))
			{
				Encoder_PollDeltaT_IO(&p_motor->Encoder);  //check calibrate for commutation and not hall cycle
//				p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder); //Always calc speed?

				p_motor->ElectricalAngle = Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
				p_motor->InterpolatedAngleIndex = 0U;
			}
			else
			{
				p_motor->InterpolatedAngleIndex++;
				p_motor->ElectricalAngle += Encoder_Motor_InterpolateElectricalDelta(&p_motor->Encoder, p_motor->InterpolatedAngleIndex);
			}
			break;

		case MOTOR_SENSOR_MODE_BEMF:
//			p_motor->ElectricalAngle = est
			break;

		default:
			break;
		}

		switch (p_motor->ControlMode)
		{
		case MOTOR_CONTROL_MODE_OPEN_LOOP:
			//			p_motor->VCmd = DEFAULT_FOC_OPEN_LOOP_VQ/2; //todo prop to throttle
			p_motor->VCtrl = p_motor->VReq / 4U;
			break;

		case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ:
			//p_motor->VCmd = p_motor->SpeedFeedback_RPM * p_motor->VRpmGain;
			break;

		case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
			/*
			 * Constant Voltage,
			 * VoltagePostion,	scalar closed loop,
			 *
			 * 	p_foc->Theta = Encoder_GetElectricalDelta;
			 * 	p_foc->Vq = p_motor->VCmd
			 * 	p_foc->Vd = 0
			 *
			 */
			p_motor->VCtrl = p_motor->VReq;
			break;

		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
			/*
			 * Constant Speed,
			 *
			 * PID closed loop,
			 *
			 */

			Motor_PollSpeedLoop(p_motor);
			// p_motor->VCmd =  get from PID
			break;

		default:
			break;

		}

		Motor_FOC_ActivateAngle(p_motor);
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
//static inline void Motor_FOC_ProcCurrentLoop(Motor_T * p_motor)
//{
//	FOC_ProcClarkePark(&p_motor->Foc);
////	PID_Proc_ISR(&PIDd);
////	PID_Proc_ISR(&PIDq);
//	FOC_ProcInvParkInvClarkeSvpwm(&p_motor->Foc);
//}


/*
 * ADC conversion complete call
 */
static inline void Motor_FOC_ProcCurrentControl(Motor_T * p_motor)
{
	Encoder_CaptureDeltaD_IO(&p_motor->Encoder);

	p_motor->ElectricalAngle = Encoder_Motor_GetElectricalTheta(&p_motor->Encoder);

	if(p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
	{
//		p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
		Motor_PollSpeedLoop(p_motor);
	} /* else (p_motor->ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) */

	//	Motor_FOC_ProcCurrentLoop(p_motor);
	FOC_ProcClarkePark(&p_motor->Foc);
	//	PID_Proc_ISR(&PIDd);
	//	PID_Proc_ISR(&PIDq);

	// p_motor->VCmd =  get from PID

	Motor_FOC_ActivateAngle(p_motor);
}




/*
 * Calibrate Current ADC
 */
static inline void  Motor_FOC_StartCalibrateAdc(Motor_T * p_motor)
{
	//p_motor->StateTimerCounter = Memory_Read(p_motor->Memory, MEMORY_MAP_CURRENT_CALIBRATION_TIME);
	Thread_SetTimer(&p_motor->ControlTimerThread, 1000U);
	FOC_SetZero(&p_motor->Foc);
	Phase_SetDutyCyle_15(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output
	Phase_SetState(&p_motor->Phase, true, true, true);

//	Linear_ADC_Init(&p_motor->UnitIa, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IA], 4095U, 120U); //temp 120amp
//	Linear_ADC_Init(&p_motor->UnitIb, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IB], 4095U, 120U);
//	Linear_ADC_Init(&p_motor->UnitIc, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IC], 4095U, 120U);
}


static inline bool Motor_FOC_CalibrateAdc(Motor_T *p_motor)
{
//	State_Input_T returnInput;
	bool isComplete;
//	p_motor->ZeroIa = filter_movavgn(p_motor->ZeroIa, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IA], p_motor->CurrentFilterCoeffcientN);
//	p_motor->ZeroIb = filter_movavgn(p_motor->ZeroIb, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IB], p_motor->CurrentFilterCoeffcientN);
//	p_motor->ZeroIc = filter_movavgn(p_motor->ZeroIc, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IC], p_motor->CurrentFilterCoeffcientN);

//	p_motor->ZeroIa = Filter_MovAvg(&p_motor->FilterIa, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IA]);
//	p_motor->ZeroIb = Filter_MovAvg(&p_motor->FilterIb, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IB]);
//	p_motor->ZeroIc = Filter_MovAvg(&p_motor->FilterIc, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IC]);

	if (Thread_PollTimer(&p_motor->ControlTimerThread) == true)
	{
		Linear_ADC_Init(&p_motor->UnitIa, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IA], 4095U, 120U); //temp 120amp
		Linear_ADC_Init(&p_motor->UnitIb, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IB], 4095U, 120U);
		Linear_ADC_Init(&p_motor->UnitIc, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IC], 4095U, 120U);
//		Linear_ADC_Init(&p_motor->UnitIc, Filter_MovAvg_GetResult(&p_motor->FilterIc), 4095U, 120U);
		isComplete = true;
//		//State_Transition(&(p_motor->StateMachine), MOTOR_CALIBRATION_COMPLETE);
//		returnInput = MOTOR_INPUT_CALIBRATION_COMPLETE;
	}
	else
	{
		isComplete = false;
	}
	return isComplete;
}


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
static inline void Motor_FOC_ConvertIa(Motor_T * p_motor)
{
	//Filter here if needed
	qfrac16_t i_temp = Linear_ADC_CalcSignedFraction16(&p_motor->UnitIa, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IA]);
	FOC_SetIa(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_ConvertIb(Motor_T * p_motor)
{
	qfrac16_t i_temp = Linear_ADC_CalcSignedFraction16(&p_motor->UnitIb, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IB]);
	FOC_SetIb(&p_motor->Foc, i_temp);
}

static inline void Motor_FOC_ConvertIc(Motor_T * p_motor)
{
	qfrac16_t i_temp = Linear_ADC_CalcSignedFraction16(&p_motor->UnitIc, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IC]);
	FOC_SetIc(&p_motor->Foc, i_temp);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
//static inline void Motor_FOC_Zero(Motor_T * p_motor)
//{
//
//}

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
