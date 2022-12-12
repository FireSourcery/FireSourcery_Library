/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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

/*
	e.g. Capture DeltaD, Fixed DeltaT:
	UnitT_Freq = 20000							=> Period = 50 uS
	UnitD = 0x10000/CountsPerRevolution = 8 	=> AngleRes = 0.000122 (of 1), 8 Angle16
	CounterMax = 0xFFFF							=> 8 revolutions max (if no counter reset per rev)

	DeltaD = 0xFFFF => Speed = 20000*(8)*0xFFFF	= 10485600000 angle16pers, 	159997.558594 rps, 9599853.51564 RPM
	DeltaD = 1 		=> Speed = 20000*(8) 	 	= 160000 angle16pers,		2.44140625 rps, 146.484375 RPM => max error

	1RPM: DeltaD = (1/60)/(20000*(1/8192)) = (8*8192/60)/(20000*8) = 0.00682666666, Angle16 = 1092.26666667
	RPM = 100, 		RPS = 1.6667, 	DeltaD = 0.68267, Angle16Real = 109226.66667 => DeltaD = 0, 	Angle16 = 0, 		error 109226.66667, 1.66666666672 rps
	RPM = 1000, 	RPS = 16.667, 	DeltaD = 6.82667, Angle16Real = 1092266.6667 => DeltaD = 6, 	Angle16 = 960000, 	error 132266.666, 2.01822916718 rps
	RPM = 10000, 	RPS = 166.67, 	DeltaD = 68.2667, Angle16Real = 10922666.667 => DeltaD = 68, 	Angle16 = 10880000, error 42666.66, .65104167175 rps

	e.g. 2:
	UnitT_Freq = 1000							=> Period = 1ms
	UnitD(0x10000/CountsPerRevolution) = 8 		=> AngleRes 0.000122 (of 1), 8 Angle16
	CounterMax = 0xFFFF								=> Overflow on 524,280 angle16, completion of rev 8

	DeltaD = 0xFFFF => Speed = 1000*(8)*0xFFFF	= 524280000 angle16pers, 7999.87792969 rps, 479992.675781 RPM
	DeltaD = 1 		=> Speed = 1000*(8) 	 	= 8000 angle16pers,		 0.1220703125 rps, 7.32421875 RPM => max error

	1RPM: DeltaD = (0x10000/60)/(1000*8) = 0.13653333333, Angle16 = 1092.26666667
	RPM = 100, 		RPS = 1.6667, Angle16Real = 109226.66667, DeltaD = 13.6533 => DeltaD = 13, 		Angle16 = 104000,  	error 5226.667, 0.07975260925 rps
	RPM = 1000, 	RPS = 16.667, Angle16Real = 1092266.6667, DeltaD = 136.533 => DeltaD = 136, 	Angle16 = 1088000, 	error 4266.667, 0.06510417175 rps
	RPM = 10000, 	RPS = 166.67, Angle16Real = 10922666.667, DeltaD = 1365.33 => DeltaD = 1365, 	Angle16 = 10920000, error 2666.667, 0.04069010925 rps
*/

/*!
	CONFIG_ENCODER_HW_EMULATED set AngularD in ISR
*/
// static inline uint32_t _Encoder_DeltaD_GetAngularD(Encoder_T * p_encoder)
// {
// #if 	defined(CONFIG_ENCODER_HW_DECODER)
// 	return HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
// #elif 	defined(CONFIG_ENCODER_HW_EMULATED)
// 	return p_encoder->AngularD;
// #endif
// }

/* for common interface via angularD */
// static inline uint32_t Encoder_DeltaD_CaptureAngularD(Encoder_T * p_encoder)
// {
// #if 	defined(CONFIG_ENCODER_HW_DECODER)
// 	p_encoder->AngularD = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
// #elif 	defined(CONFIG_ENCODER_HW_EMULATED) /* Capture in ISR */
// #endif
// 	return p_encoder->AngularD;
// }

/******************************************************************************/
/*!
	@brief 	Capture DeltaD, per fixed changed in time, D_SPEED_FREQ
*/
/******************************************************************************/
static inline void Encoder_DeltaD_Capture(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	/* For Common interface functions */
	p_encoder->AngularD = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
	/* Emulated Capture in ISR */
#endif

	// if (p_encoder->Params.IsQuadratureCaptureEnabled == true)
	// {

	// }
	// else
	{
		p_encoder->DeltaD = _Encoder_CaptureDelta(p_encoder, p_encoder->Params.CountsPerRevolution - 1U, p_encoder->AngularD);
	}
	// integral/average functions
	// p_encoder->TotalD += p_encoder->DeltaD;
	// p_encoder->TotalT += 1U;
}

#if defined(CONFIG_ENCODER_HW_DECODER) && defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
// // HW Quadrature if counter ticks downs
// static inline void _Encoder_DeltaD_CaptureQuadrature(Encoder_T * p_encoder)
// {
// 	uint32_t counterValue = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
// 	bool isIncrement;
// 	// bool isCounterIncrementDirectionPositive;

// 	/*
// 		Unsigned DeltaD capture
// 	*/
// 	if (HAL_Encoder_ReadTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER) == true)
// 	{
// 		if(HAL_Encoder_ReadDecoderCounterOverflowIncrement(p_encoder->CONFIG.P_HAL_ENCODER) == true)
// 		{
// 			p_encoder->DeltaD = p_encoder->Params.CountsPerRevolution - p_encoder->TimerCounterSaved + counterValue;
// 			isIncrement = true;
// 		}
// 		else if (HAL_Encoder_ReadDecoderCounterOverflowDecrement(p_encoder->CONFIG.P_HAL_ENCODER) == true) //counter counts down, deltaD is negative
// 		{
// 			p_encoder->DeltaD = p_encoder->Params.CountsPerRevolution - counterValue + p_encoder->TimerCounterSaved;
// 			isIncrement = false;
// 		}

// 		HAL_Encoder_ClearTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER);
// 	}
// 	else
// 	{
// 		if (counterValue > p_encoder->TimerCounterSaved)
// 		{
// 			p_encoder->DeltaD = counterValue - p_encoder->TimerCounterSaved;
// 			isIncrement = true;
// 		}
// 		else //counter counts down, deltaD is negative
// 		{
// 			p_encoder->DeltaD = p_encoder->TimerCounterSaved - counterValue;
// 			isIncrement = false;
// 		}

// 		//signed capture
// //		p_encoder->DeltaD = counterValue - p_encoder->TimerCounterSaved;
// 	}

// 	p_encoder->TimerCounterSaved = counterValue;
// 	p_encoder->AngularD = counterValue;
// 	p_encoder->TotalT += 1U;

	//static inline void Encoder_DeltaD_ReadQuadratureDirection(Encoder_T * p_encoder)
	//{//#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
	//	//	isCounterIncrementDirectionPositive = p_encoder->IsALeadBDirectionPositive;
	//	//#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
	//	//	isCounterIncrementDirectionPositive = !p_encoder->IsALeadBDirectionPositive;
	//	//#endif
	//	return HAL_Encoder_ReadDecoderCounterDirection(p_encoder->CONFIG.P_HAL_ENCODER);
	//}

// #ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
// 	isCounterIncrementDirectionPositive = p_encoder->Params.IsALeadBPositive;
// #elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
// 	isCounterIncrementDirectionPositive = !p_encoder->Params.IsALeadBPositive;
// #endif

// 	if ((isCounterIncrementDirectionPositive) && (isIncrement == true))
// //	if ((p_encoder->Params.DirectionCalibration == ENCODER_DIRECTION_DIRECT) && (isIncrement == true)) //Positive DeltaD is positive direction
// 	{
// 		p_encoder->TotalD += p_encoder->DeltaD;
// 	}
// 	else
// 	{
// 		p_encoder->TotalD -= p_encoder->DeltaD;	//  deltaD is negative
// 	}
// }
#endif


/******************************************************************************/
/*!
	@brief 	Get and unit conversion.
			Convert To - shared with encoder common.
			Convert From - unique to mode
*/
/******************************************************************************/
/******************************************************************************/
/*!
	Angle
*/
/******************************************************************************/
// static inline uint32_t Encoder_DeltaD_GetAngle(Encoder_T * p_encoder)
// {
// 	return Encoder_ConvertCounterDToAngle(p_encoder, _Encoder_DeltaD_GetAngularD(p_encoder));
// }

static inline uint32_t Encoder_DeltaD_GetDelta(Encoder_T * p_encoder) 			{ return p_encoder->DeltaD; }
static inline uint32_t Encoder_DeltaD_GetDeltaAngle(Encoder_T * p_encoder) 		{ return Encoder_ConvertCounterDToAngle(p_encoder, p_encoder->DeltaD); }
static inline uint32_t Encoder_DeltaD_GetDeltaDistance(Encoder_T * p_encoder) 	{ return Encoder_ConvertCounterDToUnits(p_encoder, p_encoder->DeltaD); }

/******************************************************************************/
/*!
	Get Speed - Variable DeltaD (DeltaT is fixed, == 1).
	Fixed DeltaT: DeltaD count on fixed time sample.
*/
/******************************************************************************/
/******************************************************************************/
/*!
	Angular
*/
/******************************************************************************/
/*
	e.g.
	CountsPerRevolution = 8192

	SampleFreq = 20000
	UnitAngularSpeed = 160,000 => DeltaD overflow ~= 26,843
	10k RPM => DeltaD = 10000 / 60 * 8192 / 20000 = 68

	SampleFreq = 1000
	UnitAngularSpeed = 8,000 =>  DeltaD overflow ~= 536,870
	10k RPM => DeltaD = 10000 / 60 * 8192 / 1000 = 1,365
*/
static inline uint32_t Encoder_DeltaD_GetAngularSpeed(Encoder_T * p_encoder)
{
	return _Encoder_CalcAngularSpeed(p_encoder, p_encoder->DeltaD, 1U);
}

/*!
	@return DeltaD in counter ticks.
*/
static inline uint32_t Encoder_DeltaD_ConvertFromAngularSpeed(Encoder_T * p_encoder, uint32_t angularSpeed_UserDegreesPerSecond)
{
	return angularSpeed_UserDegreesPerSecond / p_encoder->UnitAngularSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertToAngularSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return _Encoder_CalcAngularSpeed(p_encoder, deltaD_Ticks, 1U);
}

static inline uint32_t Encoder_DeltaD_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	return _Encoder_CalcRotationalSpeed_Shift(p_encoder, p_encoder->DeltaD * 60U, 1U);
}

/*
	Speed to DeltaD conversion
	1 Division for inverse conversion. Alternatively, (rpm * CountsPerRevolution) / (UnitT_Freq * 60U)
*/
static inline uint32_t Encoder_DeltaD_ConvertFromRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	// return (rpm << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / (p_encoder->UnitAngularSpeed * 60U);
	return (rpm * p_encoder->Params.CountsPerRevolution) / (p_encoder->UnitT_Freq * 60U);
}

static inline uint32_t Encoder_DeltaD_ConvertToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return _Encoder_CalcRotationalSpeed_Shift(p_encoder, deltaD_Ticks * 60U, 1U);
}

/******************************************************************************/
/*!
	Scalar
*/
/******************************************************************************/
static inline uint32_t Encoder_DeltaD_GetScalarSpeed(Encoder_T * p_encoder)
{
	return Encoder_CalcScalarSpeed(p_encoder, p_encoder->DeltaD, 1U);
}

/******************************************************************************/
/*!
	Linear
*/
/******************************************************************************/
/*!
	Overflow caution: Max DeltaD = UINT32_MAX / UnitLinearSpeed
*/
static inline uint32_t Encoder_DeltaD_GetLinearSpeed(Encoder_T * p_encoder)
{
	return Encoder_CalcLinearSpeed(p_encoder, p_encoder->DeltaD, 1U);
}

static inline uint32_t Encoder_DeltaD_ConvertFromLinearSpeed(Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
	return speed_UnitsPerSecond / p_encoder->UnitLinearSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertFromLinearSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
	return speed_UnitsPerMinute * 60U / p_encoder->UnitLinearSpeed;
}

static inline uint32_t Encoder_DeltaD_ConvertToLinearSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return Encoder_CalcLinearSpeed(p_encoder, deltaD_Ticks, 1U);
}

static inline uint32_t Encoder_DeltaD_ConvertToLinearSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return Encoder_CalcLinearSpeed(p_encoder, deltaD_Ticks * 60U, 1U);
}

static inline uint32_t Encoder_DeltaD_GetGroundSpeed_Mph(Encoder_T * p_encoder)
{
	return Encoder_CalcGroundSpeed_Mph(p_encoder, p_encoder->DeltaD, 1U);
}

static inline uint32_t Encoder_DeltaD_GetGroundSpeed_Kmh(Encoder_T * p_encoder)
{
	return Encoder_CalcGroundSpeed_Kmh(p_encoder, p_encoder->DeltaD, 1U);
}

/******************************************************************************/
/*!
	@brief 	Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void Encoder_DeltaD_Init(Encoder_T * p_encoder);
extern void Encoder_DeltaD_SetInitial(Encoder_T * p_encoder);
extern void Encoder_DeltaD_CalibrateQuadratureReference(Encoder_T * p_encoder);
extern void Encoder_DeltaD_CalibrateQuadraturePositive(Encoder_T * p_encoder);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
