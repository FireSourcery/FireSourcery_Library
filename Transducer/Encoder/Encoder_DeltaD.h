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
	_Encoder_CaptureDelta(p_encoder, &p_encoder->DeltaD, p_encoder->EncoderResolution - 1U);
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
			p_encoder->DeltaD = p_encoder->EncoderResolution - p_encoder->TimerCounterSaved + counterValue;
			isIncrement = true;
		}
		else if (HAL_Encoder_ReadQuadratureCounterOverflowDecrement(p_encoder->CONFIG.P_HAL_ENCODER)) //counter counts down, deltaD is negative
		{
			p_encoder->DeltaD = p_encoder->EncoderResolution - counterValue + p_encoder->TimerCounterSaved;
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
	}

	p_encoder->TimerCounterSaved = counterValue;
	p_encoder->AngularD = counterValue;
	p_encoder->TotalT += 1U;

	//signed capture
//#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
//	isCounterIncrementDirectionPositive = p_encoder->IsALeadBDirectionPositive;
//#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
//	isCounterIncrementDirectionPositive = !p_encoder->IsALeadBDirectionPositive;
//#endif
//
//	if (isCounterIncrementDirectionPositive) //Positive DeltaD is positive direction
//	{
////		p_encoder->TotalD += p_encoder->DeltaD;
//		//direction = or use deltaD sign?
//	}
//	else
//	{
////		p_encoder->TotalD -= p_encoder->DeltaD;
//	}

}

static inline void Encoder_DeltaD_Capture(Encoder_T * p_encoder)
{
#ifdef CONFIG_ENCODER_HW_QUADRATURE_CAPABLE
	if (p_encoder->IsQuadratureCaptureEnabled == true)
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
 * unit converions
	Capture DeltaD Mode Functions -
	Only for variable DeltaD (DeltaT is fixed, == 1).
	Meaningless for DeltaT, variable DeltaD (DeltaT is fixed, == 1).
 */
/******************************************************************************/
static inline uint32_t Encoder_DeltaD_Get(Encoder_T * p_encoder)		{return p_encoder->DeltaD;}
static inline uint32_t Encoder_DeltaD_Get_Units(Encoder_T * p_encoder)	{return p_encoder->DeltaD * p_encoder->UnitD;}


//	static inline uint32_t Encoder_GetSpeed_FixedDeltaD(Encoder_T * p_encoder) //DeltaD is fixed, i.e 1
//	{
//		uint32_t spd;
//
//		if (p_encoder->DeltaT == 0)
//		{
//			spd = 0;
//		}
//		else
//		{
//			/*
//			 * Case of Capture DeltaT with large UnitT_Freq
//			 */
//	//		if ([UnitD * UnitT_Freq] > UINT32_MAX) //determine in init
//	//		{
//	//			spd = UnitD * ( UnitT_Freq/ p_encoder->DeltaT);
//	//		}
//	//		else
//	//		{
//	//			spd =  UnitD * UnitT_Freq / p_encoder->DeltaT;
//	//		}
//			spd = p_encoder->UnitSpeed / p_encoder->DeltaT;
//		}
//
//		return spd;
//	}

/*!
	Capture DeltaD  only-
	CaptureDeltaD, DeltaT == 1: DeltaD count on fixed time sample.
	CaptureDeltaT, DeltaD == 1: <= 1, (Number of fixed DeltaD samples, before a DeltaT increment)
*/
static inline uint32_t Encoder_DeltaD_ConvertFromSpeed(Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
	return speed_UnitsPerSecond / p_encoder->UnitSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertToSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertFromSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
	return speed_UnitsPerMinute * 60U / p_encoder->UnitSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertToSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitSpeed * 60U;
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

static inline uint32_t Encoder_DeltaD_ConvertFromRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	//todo use share anglular speed
//	return (rpm << (CONFIG_ENCODER_ANGLE_DEGREES_BITS)) / (60 * p_encoder->UnitAngularSpeed);
	return rpm * p_encoder->EncoderResolution / (p_encoder->UnitT_Freq * 60U);
}

static inline uint32_t Encoder_DeltaD_ConvertToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
//	return Encoder_ConvertDeltaDToAngularSpeed(p_encoder, deltaD_Ticks * 60U) >> CONFIG_ENCODER_ANGLE_DEGREES_BITS;  overflow?
	return (deltaD_Ticks * p_encoder->UnitT_Freq * 60U) / p_encoder->EncoderResolution;
}


#endif
