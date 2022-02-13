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
	@file  	Encoder_DeltaD.h
	@author FireSourcery
	@brief 	Capture Delta D Mode, Polling time is fixed
	@version V0
 */
/******************************************************************************/
#ifndef ENCODER_DELTA_D_H
#define ENCODER_DELTA_D_H

#include "Encoder.h"
#include "HAL_Encoder.h"

#include <stdint.h>
#include <stdbool.h>

/*!
	@brief 	Capture DeltaD, per fixed changed in time, via timer periodic interrupt
			Looping Angle Capture

 	 	 	TimerCounter should loop for correct angular position
 	 	 	TimerCounterMax == EncoderRes -1
 */
static inline void Encoder_DeltaD_CapturePhase(Encoder_T * p_encoder)
{
	_Encoder_CaptureDelta(p_encoder, &p_encoder->DeltaD, p_encoder->Params.CountsPerRevolution - 1U);
	p_encoder->AngularD = p_encoder->TimerCounterSaved;
	p_encoder->TotalD += p_encoder->DeltaD;
	p_encoder->TotalT += 1;
}


// HW Quadrature if counter ticks downs
// todo overflow both directions, flag in isr
static inline void Encoder_DeltaD_CaptureQuadrature(Encoder_T * p_encoder)
{
	uint32_t counterValue = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
	bool isIncrement;
	bool isCounterIncrementDirectionPositive;



	/*
	 * Unsigned DeltaD capture
	 */
	if (HAL_Encoder_ReadTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER))
	{
		if(HAL_Encoder_ReadQuadratureCounterOverflowIncrement(p_encoder->CONFIG.P_HAL_ENCODER))
		{
			p_encoder->DeltaD = p_encoder->Params.CountsPerRevolution - p_encoder->TimerCounterSaved + counterValue;
			isIncrement = true;
		}
		else if (HAL_Encoder_ReadQuadratureCounterOverflowDecrement(p_encoder->CONFIG.P_HAL_ENCODER)) //counter counts down, deltaD is negative
		{
			p_encoder->DeltaD = p_encoder->Params.CountsPerRevolution - counterValue + p_encoder->TimerCounterSaved;
			isIncrement = false;
		}

		HAL_Encoder_ClearTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER);
	}
	else
	{
		if (counterValue > p_encoder->TimerCounterSaved)
		{
			p_encoder->DeltaD = counterValue - p_encoder->TimerCounterSaved;
			isIncrement = true;
		}
		else //counter counts down, deltaD is negative
		{
			p_encoder->DeltaD = p_encoder->TimerCounterSaved - counterValue;
			isIncrement = false;
		}

		//signed capture
//		p_encoder->DeltaD = counterValue - p_encoder->TimerCounterSaved;
	}

	p_encoder->TimerCounterSaved = counterValue;
	p_encoder->AngularD = counterValue;
	p_encoder->TotalT += 1U;


#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
	isCounterIncrementDirectionPositive = p_encoder->Params.IsALeadBPositive;
#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
	isCounterIncrementDirectionPositive = !p_encoder->Params.IsALeadBPositive;
#endif

	if ((isCounterIncrementDirectionPositive) && (isIncrement == true))
//	if ((p_encoder->Params.DirectionCalibration == ENCODER_DIRECTION_DIRECT) && (isIncrement == true)) //Positive DeltaD is positive direction
	{
		p_encoder->TotalD += p_encoder->DeltaD;
	}
	else
	{
		p_encoder->TotalD -= p_encoder->DeltaD;	//  deltaD is negative
	}

}

static inline uint32_t Encoder_DeltaD_CaptureAngle(Encoder_T * p_encoder)
{
	return p_encoder->AngularD = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);;
}

static inline void Encoder_DeltaD_Capture(Encoder_T * p_encoder)
{
#ifdef CONFIG_ENCODER_HW_QUADRATURE_CAPABLE
	if (p_encoder->Params.IsQuadratureCaptureEnabled == true)
	{
		Encoder_DeltaD_CaptureQuadrature(p_encoder);
	}
	else
#endif
	{
		Encoder_DeltaD_CapturePhase(p_encoder);
	}
}

static inline uint32_t Encoder_DeltaD_GetDeltaAngle(Encoder_T * p_encoder)
{
	return Encoder_ConvertCounterDToAngle(p_encoder, p_encoder->DeltaD);
}

static inline uint32_t Encoder_DeltaD_GetAngle(Encoder_T * p_encoder)
{
	return Encoder_ConvertCounterDToAngle(p_encoder, HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER));
}



//static inline void Encoder_DeltaD_ReadQuadratureDirection(Encoder_T * p_encoder)
//{//#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
//	//	isCounterIncrementDirectionPositive = p_encoder->IsALeadBDirectionPositive;
//	//#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
//	//	isCounterIncrementDirectionPositive = !p_encoder->IsALeadBDirectionPositive;
//	//#endif
//	return HAL_Encoder_ReadQuadratureCounterDirection(p_encoder->CONFIG.P_HAL_ENCODER);
//}




/******************************************************************************/
/*!
	Unit Conversions -
	Only for variable DeltaD (DeltaT is fixed, == 1).
	Meaningless for DeltaT, variable DeltaD (DeltaT is fixed, == 1).
 */
/******************************************************************************/
static inline uint32_t Encoder_DeltaD_Get(Encoder_T * p_encoder)		{return p_encoder->DeltaD;}
//static inline uint32_t Encoder_DeltaD_Get_Units(Encoder_T * p_encoder)	{return p_encoder->DeltaD * p_encoder->UnitLinearD;}
//
//static inline uint32_t Encoder_DeltaD_GetSpeed(Encoder_T * p_encoder)
//{
////	return Encoder_DeltaD_CalcSpeed(p_encoder, p_encoder->DeltaD, 1U);
//}

/*!
	Capture DeltaD  only-
	CaptureDeltaD, DeltaT == 1: DeltaD count on fixed time sample.
	CaptureDeltaT, DeltaD == 1: <= 1, (Number of fixed DeltaD samples, before a DeltaT increment)
*/
static inline uint32_t Encoder_DeltaD_ConvertFromSpeed(Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
	return speed_UnitsPerSecond / p_encoder->UnitLinearSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertToSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitLinearSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertFromSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
	return speed_UnitsPerMinute * 60U / p_encoder->UnitLinearSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertToSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitLinearSpeed * 60U;
}


/*!
	@return DeltaD is angle in raw timer counter ticks.
	CaptureDeltaD only, Fixed DeltaT: DeltaD count on fixed time sample.
 */
static inline uint32_t Encoder_DeltaD_ConvertFromAngularSpeed(Encoder_T * p_encoder, uint32_t angularSpeed_UserDegreesPerSecond)
{
	return angularSpeed_UserDegreesPerSecond / p_encoder->UnitAngularSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertToAngularSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitAngularSpeed;
}

/*
 * Todo fix. functions use polling_SAMPLE_FREQ, polling freq conversion only
 *
 * unitsAngularSpeed => DELTA_D_SAMPLE_FREQ 1000Hz
 * delta angle for speed position integration => p_encoder->CONFIG.SAMPLE_FREQ  2000Hz
 */
static inline uint32_t Encoder_DeltaD_ConvertFromRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	//todo use share anglular speed
//	return (rpm << (CONFIG_ENCODER_ANGLE_DEGREES_BITS)) / (60 * p_encoder->UnitAngularSpeed);
	return rpm * p_encoder->Params.CountsPerRevolution / (p_encoder->CONFIG.SAMPLE_FREQ * 60U);
}

static inline uint32_t Encoder_DeltaD_ConvertToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
//	return Encoder_ConvertDeltaDToAngularSpeed(p_encoder, deltaD_Ticks * 60U) >> CONFIG_ENCODER_ANGLE_DEGREES_BITS;  overflow?
	return (deltaD_Ticks * p_encoder->CONFIG.SAMPLE_FREQ * 60U) / p_encoder->Params.CountsPerRevolution;
}

/*
 */
static inline uint32_t Encoder_DeltaD_ConvertRotationalSpeedToDeltaAngle_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	if (rpm < (UINT32_MAX >> CONFIG_ENCODER_ANGLE_DEGREES_BITS))
	{
		return (rpm << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / (60U * p_encoder->CONFIG.SAMPLE_FREQ);
	}
	else
	{
		return (1U << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / 60U * rpm / p_encoder->CONFIG.SAMPLE_FREQ;
	}
}

static inline uint32_t Encoder_DeltaD_ConvertDeltaAngleToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t angle_UserDegrees)
{
	return (angle_UserDegrees * p_encoder->CONFIG.SAMPLE_FREQ >> CONFIG_ENCODER_ANGLE_DEGREES_BITS) * 60U;
}

#endif
