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
	@file  	Encoder.h
	@author FireSourcery
	@brief 	Encoder Speed, Position, Acceleration
			Supports both:
				Capture DeltaD: variable DeltaD, DeltaT is fixed == 1. DeltaD count on fixed time sample.
				Capture DeltaT: variable DeltaT, DeltaD is fixed == 1.
			Encoder common file contains unit conversions.
			No invocation of hardware in this file.
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

typedef struct __attribute__((aligned(2U))) Encoder_Params_Tag
{
	uint16_t CountsPerRevolution; 		/* Derive Angular Units. Max for counting AngularD, CaptureDeltaT mode need 2nd TimerCounterMax */
	uint16_t DistancePerRevolution;		/* Derive Linear Units. */
	uint16_t SurfaceDiameter;			/* Derive Linear Units. */
	uint16_t GearRatio_Factor;			/* Derive Linear Units. Surface:Encoder Ratio */
	uint16_t GearRatio_Divisor;			/* Derive Linear Units. */
	uint16_t ScalarSpeedRef_Rpm; 		/* Derive Frac16 Units. */
#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
	bool IsQuadratureCaptureEnabled; 	/* Quadrature Mode - enable hardware/emulated quadrature speed capture */
	bool IsALeadBPositive; 				/* User runtime calibration for encoder install direction, combine with compile time defined QUADRATURE_A_LEAD_B_INCREMENT */
#endif
	uint16_t ExtendedTimerDeltaTStop;	/* ExtendedTimer time read as deltaT stopped, default as 1s or .5rpm */

	//	uint32_t InterpolateAngleLimit;
	/* Overallocation of variable space to allow runtime polymorphism */
	uint8_t MotorPolePairs; 			/* Motor subtype, Convert between electrical speed and mechanical speed. Motor Hall Mode */
}
Encoder_Params_T;

typedef const struct Encoder_Config_Tag
{
	//todo separate HAL_Encoder_Timer_T, HAL_Encoder_Counter_T for DT Mode
	HAL_Encoder_T * const P_HAL_ENCODER; 	/*!< DeltaT and DeltaD TimerCounter - DeltaT, DeltaD Emulated Hw use a Timer. DeltaD Decoder Use as Counter. */
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	HAL_Encoder_Counter_T * const P_HAL_ENCODER_COUNTER;
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	/* Emulated D Mode */
	// HAL_Encoder_Timer_T * const P_HAL_ENCODER;
	HAL_Encoder_Phase_T * const P_HAL_ENCODER_A;
	HAL_Encoder_Phase_T * const P_HAL_ENCODER_B;
	HAL_Encoder_Phase_T * const P_HAL_ENCODER_Z;
	const uint32_t PHASE_A_ID;
	const uint32_t PHASE_B_ID;
	const uint32_t PHASE_Z_ID;
#endif
	const uint32_t POLLING_FREQ;	/*!< DeltaD - Polling D Freq. DeltaT - Interpolation Freq. */
	const uint32_t D_SPEED_FREQ; 	/*!< DeltaD Mode Speed Sample Freq */
	const uint32_t T_TIMER_FREQ;	/*!< DeltaT Mode Timer Freq */
	const volatile uint32_t * const P_EXTENDED_TIMER;	/* Polling DeltaT stop */
	const uint32_t EXTENDED_TIMER_FREQ;
	const Encoder_Params_T * P_PARAMS;
}
Encoder_Config_T;

typedef struct Encoder_Tag
{
	const Encoder_Config_T CONFIG;
	Encoder_Params_T Params;

	/* For emulated quadrature mode, sample other phase */
#if (defined(CONFIG_ENCODER_HW_EMULATED) && defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)) || defined(CONFIG_ENCODER_POLLING_CAPTURE_ENABLE)
	Pin_T PinA;
	Pin_T PinB;
#endif

	/*
		Runtime Variables
	*/
	volatile uint32_t DeltaT; 		/*!< Captured TimerCounter time interval counts between 2 distance events. Units in raw timer ticks */
#if  defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE) || defined(CONFIG_ENCODER_QUADRATURE_MODE_DECODER_ONLY)
	volatile int16_t DeltaD; 		/*!< Captured TimerCounter distance interval counts between 2 points in time. Units in raw timer ticks */
#elif defined(CONFIG_ENCODER_QUADRATURE_MODE_DISABLED)
	volatile uint16_t DeltaD; 		/*!< Captured TimerCounter distance interval counts between 2 points in time. Units in raw timer ticks */
#endif
	volatile uint16_t AngularD; 	/*!< Looping CounterD at CountsPerRevolution */

	uint32_t TimerCounterSaved; 		/*!< First time/count sample used to calculate Delta */
	uint32_t ExtendedTimerSaved;
	uint32_t ExtendedTimerConversion;	/* Extended Timer to Short Timer */

	/* Experimental */
	// uint32_t TotalT; 		/*!< TimerCounter ticks, persistent through Delta captures */
	// int32_t TotalD; 				/* Integral Capture */
	// uint32_t SpeedSaved; 	/*!< Most recently calculated speed, used for acceleration calc */
	// uint32_t DeltaSpeed; 	/*!< Save for acceleration calc */
#if defined(CONFIG_ENCODER_POLLING_CAPTURE_ENABLE)
	bool PulseReferenceSaved; /*!< PhaseA state. For polling capture of DeltaT */
#endif

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
		DEGREES_BITS				Angle unit conversion. Degrees per revolution in Bits. e.g. 16 for 65535 degrees cycle
			(UnitAngularD_ShiftDivisor == 32 - DEGREES_BITS)
		CountsPerRevolution 		Angle unit conversion. Encoder Resolution

		Revolutions[N]												= DeltaD[EncoderTicks] / CountsPerRevolution[EncoderTicks/1]
		RotationalSpeed[N/s] == Revolutions[N] / DeltaTime[s] 		= DeltaD[EncoderTicks] * UnitT_Freq[Hz] / CountsPerRevolution[EncoderTicks/1] / DeltaT[TimerTicks]

		DeltaAngle[DegreesNBits]				= DeltaD[EncoderTicks] * DegreesPerRevolution[DegreesNBits/1] / CountsPerRevolution[EncoderTicks/1]
		AngularSpeed[DegreesNBits/s]			= DeltaD[EncoderTicks] * DegreesPerRevolution[DegreesNBits/1] * UnitT_Freq[Hz] / CountsPerRevolution[EncoderTicks/1] / DeltaT[TimerTicks]
		UnitAngle[DegreesNBits/EncoderTick]		= DegreesPerRevolution[DegreesNBits/1]/CountsPerRevolution[EncoderTicks/1]
		UnitAngularSpeed[DegreesNBits/s]
	*/
	uint32_t UnitT_Freq;					/*!< T unit(seconds) conversion factor. TimerCounter freq, using Capture DeltaT (DeltaD is 1). Polling DeltaD freq, using Capture DeltaD (DeltaT is 1). */
	uint32_t UnitLinearD;					/*!< Linear D unit conversion factor. Units per TimerCounter tick, using Capture DeltaD (DeltaT is 1). Units per DeltaT capture, using Capture DeltaT (DeltaD is 1).*/
	uint32_t UnitLinearSpeed;				/*!< [UnitD * UnitT_Freq] 										=> Speed = DeltaD * UnitSpeed / DeltaT */
	uint32_t UnitAngularD_Factor; 			/*!< [0xFFFFFFFFU/CountsPerRevolution + 1] 						=> Angle = DeltaD * UnitAngle_Factor >> (32 - DEGREES_BITS) */
	uint32_t UnitAngularSpeed; 				/*!< [(1 << DEGREES_BITS) * UnitT_Freq / CountsPerRevolution] 	=> AngularSpeed = DeltaD * UnitAngularSpeed / DeltaT */
	uint32_t UnitInterpolateAngle; 			/*!< [UnitT_Freq << DEGREES_BITS / POLLING_FREQ / CountsPerRevolution] */
	uint32_t UnitScalarSpeed;				/*!< Percentage Speed of ScalarSpeedRef_Rpm, given max speed, as Fraction16 */
	//	uint32_t UnitInterpolateDistance_Factor;		/*!< [UnitD * UnitT_Freq] => D = index * DeltaD * UnitInterpolateD_Factor / POLLING_FREQ */
}
Encoder_T;


/*
	T_TIMER_FREQ <
	T_TIMER_FREQ * 60 < UINT32_MAX for RPM calc
	T_TIMER_FREQ * 60 * PolePairs < UINT32_MAX for Motor RPM calc

	(1 / POLLING_FREQ) < (0xFFFF / T_TIMER_FREQ), (for 16-bit timer)
	For 1000Hz (1ms) D_SPEED_FREQ and ExtendedTimer, T_TIMER_FREQ must be < 65MHz for 16-bit Timer
*/
#define ENCODER_INIT_HW_DECODER(p_Hal_Encoder, PollingFreq, SpeedSampleFreq, p_ExtendedTimer, ExtendedTimerFreq, p_Params)	\
{														\
	.CONFIG = 											\
	{													\
		.P_HAL_ENCODER 			= p_Hal_Encoder,		\
		.POLLING_FREQ 			= PollingFreq,			\
		.D_SPEED_FREQ 			= SpeedSampleFreq, 		\
		.P_EXTENDED_TIMER 		= p_ExtendedTimer,		\
		.EXTENDED_TIMER_FREQ 	= ExtendedTimerFreq,	\
		.P_PARAMS 				= p_Params,				\
	},													\
}

#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
#define _ENCODER_INIT_HW_EMULATED_QUADRATURE(p_PinA_Hal, PinAId, p_PinB_Hal, PinBId)	\
	{														\
		.PhaseA = PIN_INIT(p_PinA_Hal, PinAId),				\
		.PhaseB = PIN_INIT(p_PinB_Hal, PinBId),				\
	}
#else
#define _ENCODER_INIT_HW_EMULATED_QUADRATURE(p_PinA_Hal, PinAId, p_PinB_Hal, PinBId)
#endif

#define ENCODER_INIT_HW_EMULATED(p_Timer_Hal, p_PhaseA_Hal, PhaseAId, p_PhaseB_Hal, PhaseBId, p_PhaseZ_Hal, PhaseZId, PollingFreq, TimerFreq, SpeedSampleFreq, p_ExtendedTimer, ExtendedTimerFreq, p_Params)	\
{														\
	.CONFIG = 											\
	{													\
		.P_HAL_ENCODER 			= p_Timer_Hal,			\
		.P_HAL_ENCODER_A 		= p_PhaseA_Hal,			\
		.P_HAL_ENCODER_B 		= p_PhaseB_Hal,			\
		.P_HAL_ENCODER_Z 		= p_PhaseZ_Hal,			\
 		.PHASE_A_ID 			= PhaseAId,				\
		.PHASE_B_ID 			= PhaseBId,				\
		.PHASE_Z_ID 			= PhaseZId,				\
		.POLLING_FREQ 			= PollingFreq,			\
		.T_TIMER_FREQ 			= TimerFreq, 			\
		.D_SPEED_FREQ 			= SpeedSampleFreq, 		\
		.P_EXTENDED_TIMER 		= p_ExtendedTimer,		\
		.EXTENDED_TIMER_FREQ 	= ExtendedTimerFreq,	\
		.P_PARAMS 				= p_Params,				\
	},													\
}

/******************************************************************************/
/*!
	@brief Common Capture Functions
*/
/*! @{ */
/******************************************************************************/
/*
	DeltaT and Emulated DeltaD
*/
static inline void _Encoder_CaptureAngularD_Increasing(Encoder_T * p_encoder)
{
	p_encoder->AngularD = (p_encoder->AngularD < p_encoder->Params.CountsPerRevolution - 1U) ? p_encoder->AngularD + 1U : 0U;
}

static inline void _Encoder_CaptureAngularD(Encoder_T * p_encoder)
{
	// if(p_encoder->Params.IsQuadratureCaptureEnabled == true)
	// {
	// 	//todo
	// }
	// else
	{
		_Encoder_CaptureAngularD_Increasing(p_encoder);
	}
}

/*!
	@brief 	Capture Increasing DeltaT or DeltaD between 2 samples. Used speed calculations.
			Filtering Delta handle by caller.
*/
static inline uint32_t _Encoder_CaptureDelta(Encoder_T * p_encoder, uint32_t timerCounterMax, uint32_t timerCounterValue)
{
	uint32_t delta = (timerCounterValue < p_encoder->TimerCounterSaved) ?
		(timerCounterMax - p_encoder->TimerCounterSaved + timerCounterValue + 1U) :	/* TimerCounter overflow */
		(timerCounterValue - p_encoder->TimerCounterSaved); 						/* Normal case */
	p_encoder->TimerCounterSaved = timerCounterValue;
	return delta;
}

/*!
	Acceleration
*/
// static inline uint32_t Encoder_CaptureSpeed(Encoder_T * p_encoder)
// {
// 	uint32_t newSpeed = p_encoder->DeltaD * p_encoder->UnitLinearSpeed / p_encoder->DeltaT;
// 	p_encoder->DeltaSpeed = newSpeed - p_encoder->SpeedSaved;
// 	p_encoder->SpeedSaved = newSpeed;
// 	return newSpeed;
// }

/******************************************************************************/
/*!
	@brief 	 CounterD conversions.
*/
/*! @{ */
/******************************************************************************/
/******************************************************************************/
/*!
	Angle
	Base unit in CONFIG_ENCODER_ANGLE_DEGREES_BITS
*/
/******************************************************************************/
static inline uint32_t Encoder_ConvertCounterDToAngle(Encoder_T * p_encoder, uint32_t counterD_Ticks)
{
	uint32_t angle;

	/*
		Overflow if counterD > CountsPerRevolution
		CountsPerRevolution * UnitAngularD_Factor == UINT32_MAX
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

/******************************************************************************/
/*!
	Linear Distance
*/
/******************************************************************************/
static inline uint32_t Encoder_ConvertCounterDToUnits(Encoder_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks * p_encoder->UnitLinearD; }
static inline uint32_t Encoder_ConvertUnitsToCounterD(Encoder_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks / p_encoder->UnitLinearD; }
// static inline uint32_t Encoder_GetTotalDistance(Encoder_T * p_encoder) { return Encoder_ConvertCounterDToUnits(p_encoder, p_encoder->TotalD); }
/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
	@brief Speed Conversions. GetSpeed functions use DeltaD/T Modes
	Compiler may optimize passing const 1 in place of either delta
*/
/*! @{ */
/******************************************************************************/
/******************************************************************************/
/*!
	Angular Speed Functions
	Base unit in CONFIG_ENCODER_ANGLE_DEGREES_BITS Per Second
*/
/******************************************************************************/
/*

*/
static inline uint32_t _Encoder_CalcAngularSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitAngularSpeed / deltaT_Ticks;
}

/*
	DeltaT Mode process at least 1 division, use over conversion via UnitAngularSpeed, calculation when << 16 is too large
*/
static inline uint32_t _Encoder_CalcRotationalSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	uint32_t speed;

	// if(deltaD_Ticks > UINT32_MAX / p_encoder->UnitT_Freq)
	// {
	// 	speed = (deltaD_Ticks * (p_encoder->UnitT_Freq / p_encoder->Params.CountsPerRevolution)) / deltaT_Ticks;
	// }
	// else
	{
		speed = (deltaD_Ticks * p_encoder->UnitT_Freq) / (p_encoder->Params.CountsPerRevolution * deltaT_Ticks);
	}

	return speed;
}

/*
	Using Capture DeltaD, will allow at least 1 full rotation between polling without overflow
	AngularSpeed is already rotational speed shifted to highest possible resolution
*/
static inline uint32_t _Encoder_CalcRotationalSpeed_Shift(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return _Encoder_CalcAngularSpeed(p_encoder, deltaD_Ticks, deltaT_Ticks) >> CONFIG_ENCODER_ANGLE_DEGREES_BITS;
}

static inline uint32_t Encoder_CalcAngularSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	uint32_t speed;

	/*
		p_encoder->UnitAngularSpeed set to 0U if overflow:
		DeltaD mode bound during init, UnitAngularSpeed*deltaD_Ticks will allow full rotation minimal
	*/
	if(p_encoder->UnitAngularSpeed == 0U)
	{
		speed = _Encoder_CalcRotationalSpeed(p_encoder, deltaD_Ticks, deltaT_Ticks) << CONFIG_ENCODER_ANGLE_DEGREES_BITS;
	}
	else
	{
		speed = _Encoder_CalcAngularSpeed(p_encoder, deltaD_Ticks, deltaT_Ticks);
	}

	return speed;
}

static inline uint32_t Encoder_CalcAngularSpeed_RadS(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return Encoder_CalcAngularSpeed(p_encoder, deltaD_Ticks, deltaT_Ticks) * 628 / 100 / 65536;
}

/*!
	Revolution per Second
*/
static inline uint32_t Encoder_CalcRotationalSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	uint32_t speed;

	if(p_encoder->UnitAngularSpeed == 0U)
	{
		speed = _Encoder_CalcRotationalSpeed(p_encoder, deltaD_Ticks, deltaT_Ticks);
	}
	else
	{

		speed = _Encoder_CalcRotationalSpeed_Shift(p_encoder, deltaD_Ticks, deltaT_Ticks);
	}

	return speed;
}

/*
	Overflow : DeltaT mode UnitT_Freq 625000, deltaT_Ticks < 10. => 60000rpm
*/
static inline uint32_t Encoder_CalcRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	/*
		DeltaT Avg calc will use non shifted, regardless of 60 factor
	*/
	return  Encoder_CalcRotationalSpeed(p_encoder, deltaD_Ticks * 60U, deltaT_Ticks);
}

/******************************************************************************/
/*!
	Scalar Speed Functions
	RotationalSpeed * 60 / ScalarSpeedRef_Rpm

	Unsigned Frac16 65556 <=> 1
	Can over saturate 1
*/
/******************************************************************************/
static inline uint32_t Encoder_CalcScalarSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitScalarSpeed / deltaT_Ticks;
}

/******************************************************************************/
/*!
	Linear Speed Functions
*/
/******************************************************************************/
/*!
	@brief Linear speed in units in userUnits/S
	DeltaD * [UnitD * UnitT_Freq] / DeltaT;
*/
static inline uint32_t Encoder_CalcLinearSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitLinearSpeed / deltaT_Ticks;
}

/*
	Only When base units in mm, as set via SetGroundRatio function.
*/
static inline uint32_t Encoder_CalcGroundSpeed_Mph(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitLinearSpeed * 60U * 60U / (deltaT_Ticks * 1609344U);
}

static inline uint32_t Encoder_CalcGroundSpeed_Kmh(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitLinearSpeed * 60U * 60U / (deltaT_Ticks * 1000000U);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
	Speed position conversions
	Independent from Delta Captures
	Use for integrate Speed to position
*/
/******************************************************************************/
/*
	Overflow caution:
*/
static inline uint32_t Encoder_ConvertRotationalSpeedToAngle_RPM(uint32_t rpm, uint32_t sampleFreq)
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

static inline uint32_t Encoder_ConvertAngleToRotationalSpeed_RPM(uint32_t angle_UserDegrees, uint32_t sampleFreq)
{
	return (angle_UserDegrees * sampleFreq >> CONFIG_ENCODER_ANGLE_DEGREES_BITS) * 60U;
}

/*
	D/T Common
	delta angle for speed position integration => POLLING_FREQ e.g. 20000Hz
*/
static inline uint32_t Encoder_ConvertRotationalSpeedToControlAngle_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	return Encoder_ConvertRotationalSpeedToAngle_RPM(rpm, p_encoder->CONFIG.POLLING_FREQ);
}

static inline uint32_t Encoder_ConvertControlAngleToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t angle_UserDegrees)
{
	return Encoder_ConvertAngleToRotationalSpeed_RPM(angle_UserDegrees, p_encoder->CONFIG.POLLING_FREQ);
}

/*
	D only
	Avg Speed sampling
*/
static inline uint32_t Encoder_ConvertRotationalSpeedToSampleAngle_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	return Encoder_ConvertRotationalSpeedToAngle_RPM(rpm, p_encoder->CONFIG.D_SPEED_FREQ);
}

static inline uint32_t Encoder_ConvertSampleAngleToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t angle_UserDegrees)
{
	return Encoder_ConvertAngleToRotationalSpeed_RPM(angle_UserDegrees, p_encoder->CONFIG.D_SPEED_FREQ);
}

/******************************************************************************/
/*
	Shared Get Functions
	Speed Functions- Use D/T mode for compiler optimization
	todo alternatively, freq mode change
*/
/******************************************************************************/
// may change HW_DECODER capture is exception to this interface
static inline uint32_t Encoder_GetAngularD(Encoder_T * p_encoder) 	{ return p_encoder->AngularD; }
static inline uint32_t Encoder_GetAngle(Encoder_T * p_encoder) 		{ return Encoder_ConvertCounterDToAngle(p_encoder, p_encoder->AngularD); }

static inline uint32_t Encoder_GetAngularSpeed(Encoder_T * p_encoder)
{
	return Encoder_CalcAngularSpeed(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
}

static inline uint32_t Encoder_GetAngularSpeed_RadS(Encoder_T * p_encoder)
{
	return Encoder_CalcAngularSpeed_RadS(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
}

static inline uint32_t Encoder_GetRotationalSpeed_RPS(Encoder_T * p_encoder)
{
	return Encoder_CalcRotationalSpeed(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
}

static inline uint32_t Encoder_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	return Encoder_CalcRotationalSpeed_RPM(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
}

static inline uint32_t Encoder_GetFrac16Speed(Encoder_T * p_encoder)
{
	return Encoder_CalcScalarSpeed(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
}

static inline uint32_t Encoder_GetLinearSpeed(Encoder_T * p_encoder)
{
	return Encoder_CalcLinearSpeed(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
}

static inline uint32_t Encoder_GetGroundSpeed_Mph(Encoder_T * p_encoder)
{
	return Encoder_CalcGroundSpeed_Mph(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
}

static inline uint32_t Encoder_GetGroundSpeed_Kmh(Encoder_T * p_encoder)
{
	return Encoder_CalcGroundSpeed_Kmh(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
}

/*
	Units/S/S
*/
//static inline uint32_t Encoder_GetAcceleration(Encoder_T * p_encoder)
//{
//	return p_encoder->DeltaSpeed * p_encoder->UnitT_Freq / p_encoder->DeltaT;
//}

/******************************************************************************/
/*!
	@brief 	Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void _Encoder_ResetUnitsAngular(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsLinear(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsScalarSpeed(Encoder_T * p_encoder);
// extern void _Encoder_ResetTimerFreq(Encoder_T * p_encoder);

extern void Encoder_SetCountsPerRevolution(Encoder_T * p_encoder, uint16_t countsPerRevolution);
extern void Encoder_SetDistancePerRevolution(Encoder_T * p_encoder, uint16_t distancePerCount);
extern void Encoder_SetScalarSpeedRef(Encoder_T * p_encoder, uint16_t speedRef);
extern void Encoder_Motor_SetSurfaceRatio(Encoder_T * p_encoder, uint32_t surfaceDiameter, uint32_t gearRatio_Factor, uint32_t gearRatio_Divisor);
extern void Encoder_Motor_SetGroundRatio_US(Encoder_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
extern void Encoder_Motor_SetGroundRatio_Metric(Encoder_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
#if 	defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE) || defined(CONFIG_ENCODER_QUADRATURE_MODE_DECODER_ONLY)
extern void Encoder_SetQuadratureMode(Encoder_T * p_encoder, bool isEnabled);
extern void Encoder_SetQuadratureDirectionCalibration(Encoder_T * p_encoder, bool isALeadBPositive);
#endif
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
