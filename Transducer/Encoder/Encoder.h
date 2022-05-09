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
	@file  	Encoder.h
	@author FireSourcery
	@brief 	Determines Speed, Position, Acceleration, using Speed == DeltaD/DeltaT*UnitSpeed.
			A variable slope (rate of change), with respect to time, determined by hardware capture of a variable Delta.

			Supports both:
				Capture DeltaD: variable DeltaD, DeltaT is fixed, == 1.
				Capture DeltaT: variable DeltaT, DeltaD is fixed, == 1.

			Encoder defined in Counts Per Revolution and Distance Per Count

			Encoder common file contains unit conversions

	@version V0
*/
/******************************************************************************/
#ifndef ENCODER_H
#define ENCODER_H

#include "HAL_Encoder.h"
#include "Config.h"
#include "Peripheral/Pin/Pin.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct __attribute__((aligned(4U))) Encoder_Params_Tag
{
	uint16_t CountsPerRevolution; 		/* Derive Angular Speed. Max for counting AngularD, CaptureDeltaT mode need 2nd TimerCounterMax */
	uint16_t DistancePerCount;			/* Derive Linear Speed. */
	uint16_t SpeedRef_Rpm; 				/* Derive Frac16 Speed. */
	bool IsQuadratureCaptureEnabled; 	/* Quadrature Mode - Calibrate for encoder install direction */
	bool IsALeadBPositive; 				/* User runtime calibration, combine with compile time defined QUADRATURE_A_LEAD_B_INCREMENT */
	uint16_t ExtendedTimerDeltaTStop;	/* ExtendedTimer time read as deltaT stopped, default as 1s or .5rpm */

//	uint32_t InterpolateAngleLimit;
	uint8_t MotorPolePairs; 			/*! Motor sub type, Convert between electrical speed and mechanical speed. Motor Hall Mode */
											/* Overallocation of variable space to allow runtime polymorphism */
}
Encoder_Params_T;

typedef const struct Encoder_Config_Tag
{
	HAL_Encoder_T * const P_HAL_ENCODER; 	/*!< DeltaT and DeltaD timer counter */
	const uint32_t POLLING_FREQ;			/*!<  Polling D reference Freq, CaptureDeltaT mode need 2nd freq interpolation calculations */
	const uint32_t DELTA_T_TIMER_FREQ;
	const uint32_t DELTA_D_SAMPLE_FREQ; 	/* Speed sample, different from polling */
	const volatile uint32_t * const P_EXTENDED_TIMER;	/* Polling DeltaT stop */
	const uint32_t EXTENDED_TIMER_FREQ;
	const Encoder_Params_T * P_PARAMS;
}
Encoder_Config_T;

typedef struct Encoder_Tag
{
	const Encoder_Config_T CONFIG;
	Encoder_Params_T Params;

	Pin_T PhaseA;
	Pin_T PhaseB;

	/*
		Runtime Variables
	*/
	uint32_t TimerCounterSaved; /*!< First time/count sample used to calculate Delta */
	uint32_t ExtendedTimerSaved;

	uint32_t DeltaT; 	/*!< Captured TimerCounter time interval counts between 2 distance events. Units in raw timer ticks */
	int16_t DeltaD; 	/*!< Captured TimerCounter distance interval counts between 2 points in time. Units in raw timer ticks */
	uint16_t AngularD; 	/*!< Looping TotalD at EncoderResolution. TotalAngularD */

	uint32_t TotalT;	 /*!< TimerCounter ticks, persistent through Delta captures */
	int32_t TotalD;

	uint32_t SpeedSaved; /*!< Most recently calculated speed, used for acceleration calc */
	uint32_t DeltaSpeed; /*!< Save for acceleration calc */

	bool PulseReferenceSaved; /*!< PhaseA state. For polling capture of DeltaT */

	/*
		Unit conversion. derived on init from Nv Params
	*/
	/*
		Linear

		User Input:
		unitLinearDistance			Linear unit conversion factor. Set to UnitD. Encoder [Distance Per Revolution]/[Pulses Per Revolution]. Enforce user input whole number value.
		unitLinearSpeed				Linear Speed conversion factor. Set to UnitSpeed.

		DeltaTime[s]												= DeltaT[TimerTicks] * UnitT[s] = DeltaT[TimerTicks] / UnitT_Freq[Hz]
		DeltaDistance[Units]										= DeltaD[CounterTicks] * UnitD[Units]
		Speed[Units/s] == DeltaDistance[Units] / DeltaTime[s] 		= DeltaD[CounterTicks] * UnitD[Units] * UnitT_Freq[Hz] / DeltaT[TimerTicks]
		UnitSpeed													= UnitD[Units] * UnitT_Freq[Hz]
		LinearDistance[Units]	= DeltaD[EncoderTicks] * DistancePerRevolution[Units/1] / CountsPerRevolution[EncoderTicks/1] = DeltaD[EncoderTicks] * UnitD[Units]
		LinearSpeed[Units/s] 	= LinearDistance[Units]/DeltaTime[s] = DeltaD[Ticks] * UnitD[Units] * UnitT_Freq[Hz] / DeltaT[Ticks]
	*/
	/*
		Rotational/Angular

		User Input:
		DEGREES_BITS			Angle unit conversion. Degrees per revolution in Bits. e.g. 16 for 65535 degrees cycle
		CountsPerRevolution 	Angle unit conversion. Encoder Pulses Per Revolution

		Revolutions[N]												= DeltaD[EncoderTicks] / CountsPerRevolution[EncoderTicks/1]
		RotationalSpeed[N/s] == Revolutions[N] / DeltaTime[s] 		= DeltaD[EncoderTicks] * UnitT_Freq[Hz] / CountsPerRevolution[EncoderTicks/1] / DeltaT[TimerTicks]

		DeltaAngle[DegreesNBits]		= DeltaD[EncoderTicks] * DegreesPerRevolution[DegreesNBits/1] / CountsPerRevolution[EncoderTicks/1]
		AngularSpeed[DegreesNBits/s]	= DeltaD[EncoderTicks] * DegreesPerRevolution[DegreesNBits/1] * UnitT_Freq[Hz] / CountsPerRevolution[EncoderTicks/1] / DeltaT[TimerTicks]
		UnitAngle[DegreesNBits]			= DegreesPerRevolution[DegreesNBits/1]/CountsPerRevolution[EncoderTicks/1]

		Angle Fast Divide:
		Angle = D * UnitAngle_Factor >> UnitAngle_DivisorShift
	*/
	uint32_t UnitT_Freq;					/*!< T unit(seconds) conversion factor. TimerCounter freq, using Capture DeltaT (DeltaD is 1). Polling DeltaD freq, using Capture DeltaD (DeltaT is 1). */
	uint32_t UnitLinearD;					/*!< Linear D unit conversion factor. Units per TimerCounter tick, using Capture DeltaD (DeltaT is 1). Units per DeltaT capture, using Capture DeltaT (DeltaD is 1).*/
	uint32_t UnitLinearSpeed;				/*!< [UnitD * UnitT_Freq] => Speed = DeltaD * UnitSpeed / DeltaT */
	uint32_t UnitAngularD_Factor; 			/*!< [0xFFFFFFFFU/CountsPerRevolution + 1] => DeltaAngle = DeltaD * UnitAngle_Factor >> [32 - DEGREES_BITS] */
	uint32_t UnitAngularSpeed; 				/*!< [(1 << DEGREES_BITS) * UnitT_Freq / CountsPerRevolution] => AngularSpeed = DeltaD * UnitAngularSpeed / DeltaT
													e.g. UnitAngularSpeed == 160,000 { DEGREES_BITS = 16, UnitT_Freq = 20000, CountsPerRevolution = 8912 } */
	uint32_t UnitInterpolateAngle; 			/*!< [UnitT_Freq << DEGREES_BITS / POLLING_FREQ / CountsPerRevolution] */
//	uint32_t UnitInterpolateD_Factor;		/*!< [UnitD * UnitT_Freq] => D = index * DeltaD * UnitInterpolateD_Factor / POLLING_FREQ */

	uint32_t UnitRefSpeed;
}
Encoder_T;

#define ENCODER_CONFIG(p_Hal_Encoder, p_PinA_Hal, PinAId, p_PinB_Hal, PinBId, PollingFreq, DeltaDSampleFreq, DeltaTTimerFreq, p_ExtendedTimer, ExtendedTimerFreq, p_Params)	\
{																		\
	.CONFIG = 															\
	{																	\
		.P_HAL_ENCODER 			= p_Hal_Encoder,						\
		.POLLING_FREQ 			= PollingFreq,		 					\
		.DELTA_D_SAMPLE_FREQ 	= DeltaDSampleFreq, 					\
		.DELTA_T_TIMER_FREQ 	= DeltaTTimerFreq,						\
		.P_EXTENDED_TIMER 		= p_ExtendedTimer,						\
		.EXTENDED_TIMER_FREQ 	= ExtendedTimerFreq,					\
		.P_PARAMS 				= p_Params,								\
	},																	\
	.PhaseA = PIN_CONFIG(p_PinA_Hal, PinAId),							\
	.PhaseB = PIN_CONFIG(p_PinB_Hal, PinBId),							\
}

/******************************************************************************/
/*!
	ISRs
*/
/*! @{ */
/******************************************************************************/
//static inline void Encoder_TimerCounterOverflow_ISR(Encoder_T * p_encoder)
//{
//	HAL_Encoder_ClearTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER);
//	//set overflow direction?
//}
//
///*
// * Index Pin
// */
//static inline void Encoder_OnIndex_ISR(Encoder_T * p_encoder)
//{
//	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
//	p_encoder->AngularD = 0U;
//}
/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*!
	@addtogroup	CaptureDelta
	@brief	Capture DeltaT or DeltaD between 2 samples.
			Either DeltaT or DeltaD is selected to be fixed.
			Used for all speed calculations.

			Filtering Delta is responsibility of caller.
*/
/*! @{ */
/******************************************************************************/
/*!
	@brief Protected CaptureDelta Helper

	capture increasing only
*/
static inline void _Encoder_CaptureDelta(Encoder_T * p_encoder, uint32_t * p_delta, uint32_t timerCounterMax)
{
	uint32_t timerCounterValue = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);

	if(timerCounterValue < p_encoder->TimerCounterSaved) /* TimerCounter overflow */
	{
		*p_delta = timerCounterMax - p_encoder->TimerCounterSaved + timerCounterValue + 1U;
	}
	else /* normal case */
	{
		*p_delta = timerCounterValue - p_encoder->TimerCounterSaved;
	}

	p_encoder->TimerCounterSaved = timerCounterValue;
}


/******************************************************************************/
/*!
	Inline Setters
*/
/*! @{ */
/******************************************************************************/
//static inline void Encoder_ResetDeltas(Encoder_T * p_encoder)
//{
//	p_encoder->DeltaD = 1;
//	p_encoder->DeltaT = 1;
//}
//
//static inline void Encoder_SetZeroTotals(Encoder_T * p_encoder)
//{
//	p_encoder->TotalD = 0;
//	p_encoder->TotalT = 0;
//}
//
//static inline void Encoder_SetZeroTimerCounter(Encoder_T * p_encoder)
//{
//	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
//	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
//}
//
//static inline void Encoder_SetZeroSpeed(Encoder_T * p_encoder)
//{
//	p_encoder->DeltaD = 1;
//	p_encoder->DeltaT = 1;
//
//	p_encoder->SpeedSaved = 0U;
//	p_encoder->DeltaSpeed = 0U;
//
//	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);; /* reset for angularD */
//	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER);
//
//	p_encoder->ExtendedDeltaTimerSaved = *p_encoder->p_ExtendedDeltaTimer;
//}
/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
	@addtogroup	Delta
	@brief 	 Captured Delta conversions.

	todo untested
*/
/*! @{ */
/******************************************************************************/
/*!
	@brief Private Micros Helper.
	Determine micros with max precision, when timerTicks * 1000000 overflows

	micros = 1000000 * (timerTicks / timerFreq);
	micros = timerTicks * (1000000 / timerFreq);
	micros = 1000000 / (timerFreq / timerTicks);
	micros = timerTicks / (timerFreq / 1000000);
*/
//static inline uint32_t _Encoder_MicrosHelper(uint32_t timerTicks, uint32_t timerFreq)
//{
//	uint32_t micros;
//
//	/* overflows if DeltaT > 4294 */
//	if (timerTicks > (UINT32_MAX / 1000000))
//	{
//		// todo optimize
//		/* divide largest by smallest */
//		if 		((timerTicks > 1000000) && (1000000 > timerFreq))
//		{
//			micros = 1000000 * (timerTicks / timerFreq);
//		}
//		else if ((1000000 > timerTicks) && (timerTicks > timerFreq))
//		{
//			micros = timerTicks * (1000000 / timerFreq);
//		}
//		else if((timerTicks > timerFreq) && (timerFreq > 1000000))
//		{
////			micros = 1000000 * (timerTicks / timerFreq);
//			micros = timerTicks / (timerFreq / 1000000);
//		}
//		else if ((1000000 > timerFreq) && (timerFreq > timerTicks))
//		{
////			micros = timerTicks * (1000000 / timerFreq);
//			micros = 1000000 / (timerFreq / timerTicks);
//		}
//		else if ((timerFreq > timerTicks) && (timerTicks > 1000000))
//		{
//			micros = timerTicks / (timerFreq / 1000000);
//		}
//		else if ((timerFreq > 1000000) && (1000000 > timerTicks))
//		{
//			micros = 1000000 / (timerFreq / timerTicks);
//		}
//	}
//	else
//	{
//		micros = timerTicks * 1000000 / timerFreq;
//	}
//
//	return micros;
//}



/******************************************************************************/
/*!
	TotalD TotalT Functions - Both modes
*/
/******************************************************************************/
static inline uint32_t Encoder_ConvertCounterDToUnits(Encoder_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks * p_encoder->UnitLinearD; }
static inline uint32_t Encoder_ConvertUnitsToCounterD(Encoder_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks / p_encoder->UnitLinearD; }

/*!
	Same as integral D
*/
static inline uint32_t Encoder_GetTotalD_Units(Encoder_T * p_encoder) { return p_encoder->TotalD * p_encoder->UnitLinearD; }

//static inline uint32_t Encoder_GetTotalTFreq(Encoder_T * p_encoder)		{return p_encoder->UnitT_Freq / p_encoder->TotalT;}
static inline uint32_t Encoder_GetTotalT_Millis(Encoder_T * p_encoder) { return p_encoder->TotalT * 1000U / p_encoder->UnitT_Freq; }
//static inline uint32_t Encoder_GetTotalT_Micros(Encoder_T * p_encoder)	{return _Encoder_MicrosHelper(p_encoder->TotalT,  p_encoder->UnitT_Freq);}

/******************************************************************************/
/*!
	@brief Angle Functions
*/
/*! @{ */
/******************************************************************************/
static inline uint32_t Encoder_ConvertCounterDToAngle(Encoder_T * p_encoder, uint32_t counterD_Ticks)
{
	uint32_t angle;

	/*
	 * Overflow if counterD > CountsPerRevolution
	 *
	 * CountsPerRevolution * p_encoder->UnitAngularD_Factor == UINT32_MAX
	 */
	if(counterD_Ticks > p_encoder->Params.CountsPerRevolution)
	{
		angle = ((counterD_Ticks << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / p_encoder->Params.CountsPerRevolution);
	}
	else
	{
		angle = ((counterD_Ticks * p_encoder->UnitAngularD_Factor) >> (32U - CONFIG_ENCODER_ANGLE_DEGREES_BITS));
	}

	return angle;
}

static inline uint32_t Encoder_ConvertAngleToCounterD(Encoder_T * p_encoder, uint16_t angle_UserDegrees)
{
	return (angle_UserDegrees << (32U - CONFIG_ENCODER_ANGLE_DEGREES_BITS)) / p_encoder->UnitAngularD_Factor;
}

static inline uint32_t Encoder_GetAngle(Encoder_T * p_encoder)
{
	return Encoder_ConvertCounterDToAngle(p_encoder, p_encoder->AngularD);
}

/*
 * Integral angle
 */
static inline uint32_t Encoder_GetTotalRevolutions(Encoder_T * p_encoder)
{
	return p_encoder->TotalD / p_encoder->Params.CountsPerRevolution;
}

static inline uint32_t Encoder_GetTotalAngle(Encoder_T * p_encoder)
{
	return Encoder_ConvertCounterDToAngle(p_encoder, p_encoder->TotalD);
}

static inline void Encoder_ResetTotalAngle(Encoder_T * p_encoder)
{
	p_encoder->TotalD = 0U;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*!
	@brief Linear Speed Functions
*/
/*! @{ */
/******************************************************************************/
/*
*	e.g. CaptureDeltaT, Fixed DeltaD:
*	UnitT_Freq = 312500					=> Period = 3.2 uS
*	CounterMax = 0xFFFF							=> Overflow 209,712 us, 209 ms
*	DeltaDUnits(CountsPerRevolution) = 1/8 (8) 	=> AngleRes 0.125 (of 1), 8,169.5 Angle16
*
*  DeltaT = 0xFFFF => Speed = 312500*(1/8)/0xFFFF	= 0.5976 rps, 35.76 RPM
*	DeltaT = 1 		=> Speed = 312500*(1/8) 		= 39062.5 rps, 2343750 RPM
*	DeltaT = 100	=> Speed = 312500*(1/8)/100		= 390.625 rps, 23437.50 RPM // accurate within 1% below
*
*  1RPM: DeltaT = (312500/8)/(1/60) = 2,343,750
*  RPM = 100, 		RPS = 1.6667, 	Angle16Real = 109225 	=> ISR = 13.3333 Hz, 75ms,   => DeltaT = 23437, 	Angle16 = 109227, error 2,
*	RPM = 1000, 	RPS = 16.667, 	Angle16Real = 1092250 	=> ISR = 133.333 Hz, 7.5ms,  => DeltaT = 2343, 		Angle16 = 1092599, error 349, 0.0053 rps
*	RPM = 10000, 	RPS = 166.67,	Angle16Real = 10922500 	=> ISR = 1333.33 Hz, .75ms,  => DeltaT = 234, 		Angle16 = 10940004, error 17504, .2670 rps
*
*	e.g. 2:
*	UnitT_Freq = 2,000,000						=> Period = .5 uS
*	CounterMax = 0xFFFF								=> Overflow 32677.5us, 32.6775 ms
*	CounterMax = 0xFFFFFFFF							=> Overflow 35.7913941 minutes
*	DeltaDUnits(CountsPerRevolution) = 1/2048(2048) 	=> AngleRes 0.000488 (of 1), 32 Angle16
*
*  DeltaT = 0xFFFF => Speed = 2,000,000*(1/2048)/0xFFFF	= 0.01490 rps, 0.89408331426 RPM
*	DeltaT = 1 		=> Speed = 2,000,000*(1/2048) 			= 976.5625 rps, 58593.75 RPM
*	DeltaT = 10		=> Speed = 2,000,000*(1/2048)/10 		= 97.65625 rps, 5859.375 RPM => accurate within 10% below
*
*  1RPM: DeltaT = (2,000,000/2048)/(1/60) = 58593.75, angle 1092.26666667
*  RPM = 100, 		RPS = 1.6667, Angle16Real = 109226.66667 => ISR = 3413.3333 Hz, 292.96875us, 	=> DeltaT = 585, 	Angle16 = 109400, error 175
*	RPM = 1000, 	RPS = 16.667, Angle16Real = 1092266.6667 => ISR = 34133.333 Hz, 29.29687us, 	=> DeltaT = 58, 	Angle16 = 1103431, error 11181,	0.1706 rps
*	RPM = 10000, 	RPS = 166.67, Angle16Real = 10922666.667 => ISR = 341333.33 Hz, 2.92968us,		=> DeltaT = 5, 		Angle16 = 12799804, error 1877304, 28.645 rps
*
*	PPR = 8192 => 136.5333 ISR/s/RPM => 10000 rpm => 1,365,333 ISR/s
*	2MHz Period = .5 uS => accurate within 1% when speed < 146.484375 RPM, within 10% when 1464.84375
*	20MHz Period = .05 uS => accurate within 1% when speed < 1464.84375 RPM, within 10% when 14648.4375
*
*	20000Hz/(10000RPM/60) => 120 PPR. Less than 120 PPR, use Fixed DeltaD
*/

/*
*	e.g. CaptureDeltaD, Fixed DeltaT:
*	UnitT_Freq = 20000							=> Period = 50 uS
*	UnitD(0x10000/CountsPerRevolution) = 8 		=> AngleRes 0.000122 (of 1), 8 Angle16
*	CounterMax = 0xFFFF								=> Overflow on 524,280 angle16, completion of rev 8
*
*  DeltaD = 0xFFFF => Speed = 20000*(8)*0xFFFF	= 10485600000 angle16pers, 	159997.558594 rps, 9599853.51564 RPM
*	DeltaD = 1 		=> Speed = 20000*(8) 	 	= 160000 angle16pers,		2.44140625 rps, 146.484375 RPM => max error
*
*  1RPM: DeltaD = (1/60)/(20000*(1/8192)) = (8*8192/60)/(20000*8) = 0.00682666666, Angle16 = 1092.26666667
*  RPM = 100, 		RPS = 1.6667, 	DeltaD = 0.68267, Angle16Real = 109226.66667 => DeltaD = 0, 	Angle16 = 0, 		error 109226.66667, 1.66666666672 rps
*	RPM = 1000, 	RPS = 16.667, 	DeltaD = 6.82667, Angle16Real = 1092266.6667 => DeltaD = 6, 	Angle16 = 960000, 	error 132266.666, 2.01822916718 rps
*	RPM = 10000, 	RPS = 166.67, 	DeltaD = 68.2667, Angle16Real = 10922666.667 => DeltaD = 68, 	Angle16 = 10880000, error 42666.66, .65104167175 rps
*
*	e.g. 2:
*	UnitT_Freq = 1000							=> Period = 1ms
*	UnitD(0x10000/CountsPerRevolution) = 8 		=> AngleRes 0.000122 (of 1), 8 Angle16
*	CounterMax = 0xFFFF								=> Overflow on 524,280 angle16, completion of rev 8
*
*  DeltaD = 0xFFFF => Speed = 1000*(8)*0xFFFF	= 524280000 angle16pers, 7999.87792969 rps, 479992.675781 RPM
*	DeltaD = 1 		=> Speed = 1000*(8) 	 	= 8000 angle16pers,		 0.1220703125 rps, 7.32421875 RPM => max error
*
*	1RPM: DeltaD = (0x10000/60)/(1000*8) = 0.13653333333, Angle16 = 1092.26666667
*  RPM = 100, 		RPS = 1.6667, Angle16Real = 109226.66667, DeltaD = 13.6533 => DeltaD = 13, 		Angle16 = 104000,  	error 5226.667, 0.07975260925 rps
*	RPM = 1000, 	RPS = 16.667, Angle16Real = 1092266.6667, DeltaD = 136.533 => DeltaD = 136, 	Angle16 = 1088000, 	error 4266.667, 0.06510417175 rps
*	RPM = 10000, 	RPS = 166.67, Angle16Real = 10922666.667, DeltaD = 1365.33 => DeltaD = 1365, 	Angle16 = 10920000, error 2666.667, 0.04069010925 rps
*/




/*!
	@brief Linear speed
	units in units/s
*/
static inline uint32_t Encoder_GetSpeed(Encoder_T * p_encoder)
{
	/*
		For case of Capture DeltaD, DeltaT == 1
		possible 32bit overflow
		Max deltaD = UINT32_MAX / (unitDeltaD * unitDeltaT_Freq)
		deltaD ~14,000, for 300,000 (unitDeltaD * unitDeltaT_Freq)
	*/
	return p_encoder->DeltaD * p_encoder->UnitLinearSpeed / p_encoder->DeltaT; /* p_encoder->DeltaD * [UnitD * UnitT_Freq] / p_encoder->DeltaT; */

}



static inline uint32_t Encoder_GetSpeed_UnitsPerMinute(Encoder_T * p_encoder)
{
	return p_encoder->DeltaD * p_encoder->UnitLinearSpeed * 60U / p_encoder->DeltaT;
}

/*!
	Also save speed for acceleration calculation
*/
static inline uint32_t Encoder_CaptureSpeed(Encoder_T * p_encoder)
{
	uint32_t newSpeed = p_encoder->DeltaD * p_encoder->UnitLinearSpeed / p_encoder->DeltaT;
	p_encoder->DeltaSpeed = newSpeed - p_encoder->SpeedSaved;
	p_encoder->SpeedSaved = newSpeed;
	return newSpeed;
}

/*
	Units/S/S
*/
 //static inline uint32_t Encoder_GetAcceleration(Encoder_T * p_encoder)
 //{
 //	return p_encoder->DeltaSpeed * p_encoder->UnitT_Freq / p_encoder->DeltaT;
 //}
 //static inline uint32_t Encoder_GetLinearDeltaDistance(Encoder_T * p_encoder)	{return Encoder_GetDeltaD_Units(p_encoder);}
static inline uint32_t Encoder_GetLinearTotalDistance(Encoder_T * p_encoder) { return Encoder_GetTotalD_Units(p_encoder); }
static inline uint32_t Encoder_GetLinearSpeed(Encoder_T * p_encoder) { return Encoder_GetSpeed(p_encoder); }

/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*!
	Angular Speed Functions

	DegreesUser Per Seconds
*/
/******************************************************************************/
static inline uint32_t CalcEncoderAngularSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitAngularSpeed / deltaT_Ticks;
}

static inline uint32_t CalcEncoderRotationalSpeed_RPS(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	/*
		DeltaT Mode at least 1 division
	*/
	uint32_t speed;

	if(deltaD_Ticks > UINT32_MAX / p_encoder->UnitT_Freq)
	{
		speed = (deltaD_Ticks * (p_encoder->UnitT_Freq / p_encoder->Params.CountsPerRevolution)) / deltaT_Ticks;
	}
	else
	{
		speed = (deltaD_Ticks * p_encoder->UnitT_Freq) / (p_encoder->Params.CountsPerRevolution * deltaT_Ticks);
	}

	return speed;
}




/*
	Compiler may optimize passing const 1 in place of either delta
*/
static inline uint32_t Encoder_CalcAngularSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	uint32_t speed;

	/*
	 * p_encoder->UnitAngularSpeed set to 0U if overflow: UnitAngularSpeed*deltaD_Ticks will allow full rotation minimal
	 */
	if((p_encoder->UnitAngularSpeed == 0U) || (deltaD_Ticks > UINT32_MAX / p_encoder->UnitAngularSpeed)) //todo optimize
	{
		/* When UnitAngularSpeed is too high. Large timer freq (> 16 bits), angle res, small countsPerRevolution */
		if(deltaD_Ticks > UINT32_MAX / p_encoder->UnitT_Freq)
		{
			speed = ((deltaD_Ticks * (p_encoder->UnitT_Freq / p_encoder->Params.CountsPerRevolution)) << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / deltaT_Ticks;
		}
		else
		{
			speed = ((deltaD_Ticks * p_encoder->UnitT_Freq / p_encoder->Params.CountsPerRevolution) << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / deltaT_Ticks;
		}
	}
	else
	{
		speed = CalcEncoderAngularSpeed(p_encoder, deltaD_Ticks, deltaT_Ticks);
	}

	return speed;
}


static inline uint32_t Encoder_GetAngularSpeed(Encoder_T * p_encoder)
{
	return Encoder_CalcAngularSpeed(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
}

static inline uint32_t Encoder_GetAngularSpeed_RadS(Encoder_T * p_encoder)
{
	return Encoder_CalcAngularSpeed(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT) * 628 / 100 / 65536;
}

static inline uint32_t Encoder_CaptureAngularSpeedAverage(Encoder_T * p_encoder)
{
	//	uint32_t totalD = (p_encoder->TotalD < 0) ? 0 - p_encoder->TotalD : p_encoder->TotalD;
	//	uint32_t speed = Encoder_CalcAngularSpeed(p_encoder, totalD, p_encoder->TotalT);
	//	p_encoder->TotalD = 0U;
	//	p_encoder->TotalT = 0U;
	//	return speed;
}

/*!
	Revolution per Second
*/

 /*
	Averages Speeds may call
*/
static inline uint32_t Encoder_CalcRotationalSpeed_RPS(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	uint32_t speed;

	if((p_encoder->UnitAngularSpeed == 0U) || (deltaD_Ticks > UINT32_MAX / p_encoder->UnitAngularSpeed))
	{
		speed = CalcEncoderRotationalSpeed_RPS(p_encoder, deltaD_Ticks, deltaT_Ticks);
	}
	else
	{
		speed = CalcEncoderAngularSpeed(p_encoder, deltaD_Ticks, deltaT_Ticks) >> CONFIG_ENCODER_ANGLE_DEGREES_BITS;
	}

	return speed;
}

static inline uint32_t Encoder_CalcRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	/*
		DeltaT Avg calc will use (deltaD_Ticks > UINT32_MAX / p_encoder->UnitAngularSpeed), regardless of 60 factor
	*/
	return  Encoder_CalcRotationalSpeed_RPS(p_encoder, deltaD_Ticks * 60U, deltaT_Ticks);
}

/*
	using captured delta, will allow 1 full rotation angular speed without overflow
*/
static inline uint32_t Encoder_GetRotationalSpeed_RPS(Encoder_T * p_encoder)
{
	/*
		Angle is already highest possible rotation resolution
	*/
	return Encoder_GetAngularSpeed(p_encoder) >> CONFIG_ENCODER_ANGLE_DEGREES_BITS;
}

/*

*/
static inline uint32_t Encoder_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	return Encoder_GetAngularSpeed(p_encoder) * 60U >> CONFIG_ENCODER_ANGLE_DEGREES_BITS;  //Overflow : DeltaT mode UnitT_Freq 625000, deltaT_Ticks < 10. => 60000rpm
}



/*
	direct to user max rpm
	(Encoder_CalcAngularSpeed(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT) * 60U >> (16U - CONFIG_ENCODER_ANGLE_DEGREES_BITS)) / p_encoder->Params.SpeedRef_Rpm; // if angle bits < 16
*/
static inline uint32_t Encoder_CalcUnitSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitRefSpeed / deltaT_Ticks;
}

// static inline uint32_t Encoder_GetUnitSpeed(Encoder_T * p_encoder)
// {
// 	return Encoder_CalcUnitSpeed(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
// }

/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
	Integrate Speed to position

	Independent from Delta Captures, move
*/
/******************************************************************************/
static inline uint32_t Encoder_ConvertRotationalSpeedToAngle_RPM(Encoder_T * p_encoder, uint32_t rpm, uint32_t sampleFreq)
{
	uint32_t angle;

	if(rpm < (UINT32_MAX >> CONFIG_ENCODER_ANGLE_DEGREES_BITS))
	{
		angle = (rpm << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / (60U * sampleFreq);
	}
	else
	{
		angle = (1U << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / 60U * rpm / sampleFreq;
	}

	return angle;
}

static inline uint32_t Encoder_ConvertAngleToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t angle_UserDegrees, uint32_t sampleFreq)
{
	return (angle_UserDegrees * sampleFreq >> CONFIG_ENCODER_ANGLE_DEGREES_BITS) * 60U;
}

/*
 * delta angle for speed position integration => p_encoder->CONFIG.POLLING_FREQ  20000Hz
 */
static inline uint32_t Encoder_ConvertRotationalSpeedToControlAngle_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	return Encoder_ConvertRotationalSpeedToAngle_RPM(p_encoder, rpm, p_encoder->CONFIG.POLLING_FREQ);
}

static inline uint32_t Encoder_ConvertControlAngleToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t angle_UserDegrees)
{
	return Encoder_ConvertAngleToRotationalSpeed_RPM(p_encoder, angle_UserDegrees, p_encoder->CONFIG.POLLING_FREQ);
}

static inline uint32_t Encoder_ConvertRotationalSpeedToSampleAngle_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	return Encoder_ConvertRotationalSpeedToAngle_RPM(p_encoder, rpm, p_encoder->CONFIG.DELTA_D_SAMPLE_FREQ);
}

static inline uint32_t Encoder_ConvertSampleAngleToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t angle_UserDegrees)
{
	return Encoder_ConvertAngleToRotationalSpeed_RPM(p_encoder, angle_UserDegrees, p_encoder->CONFIG.DELTA_D_SAMPLE_FREQ);
}


/******************************************************************************/
/*!
	@brief 	Extern Functions
*/
/*! @{ */
/******************************************************************************/
extern void Encoder_SetSpeedRef(Encoder_T * p_encoder, uint16_t speedRef);
extern void Encoder_Zero(Encoder_T * p_encoder);
extern void Encoder_DeltaT_SetInitial(Encoder_T * p_encoder, uint16_t initialRpm);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
