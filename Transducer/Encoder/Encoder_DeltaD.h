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

/******************************************************************************/
/*!
	@brief 	Capture DeltaD, per fixed changed in time, SPEED_SAMPLE_FREQ
			Preliminary for all speed functions
*/
/******************************************************************************/
static inline void Encoder_DeltaD_Capture(Encoder_T * p_encoder)
{
#if defined(CONFIG_ENCODER_HW_DECODER)
	/* For common interface functions. Emulated Capture in ISR */
	uint16_t counterD;
	counterD = HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
	p_encoder->DeltaD = _Encoder_CaptureDelta(p_encoder, p_encoder->Params.CountsPerRevolution - 1U, counterD);
	// p_encoder->CounterD += p_encoder->DeltaD;
	p_encoder->Angle32 = counterD * p_encoder->UnitAngularD;
#else
	p_encoder->DeltaD = p_encoder->CounterD;
	p_encoder->CounterD = 0;
#endif
}

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
static inline uint32_t Encoder_DeltaD_GetDelta(Encoder_T * p_encoder) 			{ return p_encoder->DeltaD; }
static inline uint32_t Encoder_DeltaD_GetDeltaAngle(Encoder_T * p_encoder) 		{ return Encoder_ConvertCounterDToAngle(p_encoder, p_encoder->DeltaD); }
static inline uint32_t Encoder_DeltaD_GetDeltaDistance(Encoder_T * p_encoder) 	{ return Encoder_ConvertCounterDToDistance(p_encoder, p_encoder->DeltaD); }

/******************************************************************************/
/*!
	Compiler optimize conversion
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
	1 Division for inverse conversion. Alternatively,
	(rpm << ENCODER_ANGLE16) / (p_encoder->UnitAngularSpeed * 60U)
*/
static inline uint32_t Encoder_DeltaD_ConvertFromRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
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