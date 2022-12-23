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

#ifndef ENCODER_TIMER_MAX
#define ENCODER_TIMER_MAX (0xFFFFU)
#endif

#ifndef ENCODER_ANGLE16
#define ENCODER_ANGLE16 (16U)
#endif

#define _ENCODER_TABLE_ERROR (2U)
#define _ENCODER_TABLE_LENGTH (16U)
extern const int8_t _ENCODER_TABLE[_ENCODER_TABLE_LENGTH];
extern const int8_t _ENCODER_TABLE_PHASE_A[_ENCODER_TABLE_LENGTH];
typedef union Encoder_Phases_Tag
{
	struct
	{
		uint8_t B : 1U;
		uint8_t A : 1U;
		uint8_t PrevB : 1U;
		uint8_t PrevA : 1U;
		uint8_t Resv4 : 1U;
		uint8_t Resv5 : 1U;
		uint8_t Resv6 : 1U;
		uint8_t Resv7 : 1U;
	};
	uint8_t State;
}
Encoder_Phases_T;

typedef struct __attribute__((aligned(2U))) Encoder_Params_Tag
{
	uint16_t CountsPerRevolution; 		/* Derive Angular Units. Max for counting AngularD, CaptureDeltaT mode need 2nd TimerCounterMax */
	uint16_t DistancePerRevolution;		/* DistancePerRevolution_Factor Derive Linear Units. */
	uint16_t SurfaceDiameter;			/* Derive Linear Units. */
	uint16_t GearRatio_Factor;			/* Derive Linear Units. Surface:Encoder Ratio */
	uint16_t GearRatio_Divisor;			/* Derive Linear Units. */
	uint16_t ScalarSpeedRef_Rpm; 		/* Derive Frac16 Units. */
	uint16_t ExtendedTimerDeltaTStop;	/* ExtendedTimer time read as deltaT stopped, default as 1s or .5rpm */
	uint32_t InterpolateAngleScalar;
#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
	bool IsQuadratureCaptureEnabled; 	/* Quadrature Mode - enable hardware/emulated quadrature speed capture */
	bool IsALeadBPositive; 				/* User runtime calibration for encoder install direction, combine with compile time defined QUADRATURE_A_LEAD_B_INCREMENT */
#endif
}
Encoder_Params_T;
typedef const struct Encoder_Config_Tag
{
	HAL_Encoder_Timer_T * const P_HAL_ENCODER_TIMER; 	/*!< DeltaT Timer. */
	const uint32_t TIMER_FREQ;	/*!< DeltaT Timer Freq */
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	HAL_Encoder_Counter_T * const P_HAL_ENCODER_COUNTER; /*!< ModeD Counter */
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	HAL_Encoder_Phase_T * const P_HAL_ENCODER_A; const uint32_t PHASE_A_ID;
	HAL_Encoder_Phase_T * const P_HAL_ENCODER_B; const uint32_t PHASE_B_ID;
	HAL_Encoder_Phase_T * const P_HAL_ENCODER_Z; const uint32_t PHASE_Z_ID;
#endif
	const uint32_t POLLING_FREQ;		/*!< DeltaT Interpolation Freq. */
	const uint32_t SAMPLE_FREQ; 		/*!< Speed DeltaD DT Sample Freq. */
	const Encoder_Params_T * P_PARAMS;

	const volatile uint32_t * const P_EXTENDED_TIMER;	/* Polling DeltaT stop */
	const uint32_t EXTENDED_TIMER_FREQ;
}
Encoder_Config_T;

typedef struct Encoder_Tag
{
	const Encoder_Config_T CONFIG;
	Encoder_Params_T Params;
#if (defined(CONFIG_ENCODER_HW_EMULATED) && defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE))
	Pin_T PinA;
	Pin_T PinB;
#endif
	/*
		Runtime Variables
	*/
#if (defined(CONFIG_ENCODER_HW_EMULATED) && defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE))
	volatile Encoder_Phases_T Phases; /* Save Prev State */
#endif
	volatile int32_t CounterD;
	volatile uint32_t Angle32;
	volatile int32_t DeltaD; 		/*!< Captured TimerCounter distance interval counts between 2 points in time. Units in raw timer ticks */
	volatile uint32_t DeltaT; 		/*!< Captured TimerCounter time interval counts between 2 distance events. Units in raw timer ticks */
	volatile uint32_t DeltaTh; 		/*!< ModeDT */
	volatile int32_t FreqD; 		/*!< ModeDT */
	volatile uint32_t InterpolationIndex;

	volatile uint32_t ErrorCount;
	volatile uint32_t IndexCount;
	volatile int32_t IndexCounterD;
	volatile int32_t IndexCounterDOffset;
	uint32_t TimerCounterPrev; 		/*!< First time/count sample used to calculate Delta */
	uint32_t ExtendedTimerPrev;
	uint32_t ExtendedTimerConversion;	/* Extended Timer to Short Timer */
	/* Experimental */
	// uint32_t TotalT; 		/*!< TimerCounter ticks, persistent through Delta captures */
	// int32_t TotalD; 			/* Integral Capture */
	// uint32_t SpeedSaved; 	/*!< Most recently calculated speed, used for acceleration calc */
	// uint32_t DeltaSpeed; 	/*!< Save for acceleration calc */
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
	uint32_t UnitT_Freq;					/*!< Common Unit Reset? T unit(seconds) conversion factor. TimerCounter freq, using Capture DeltaT (DeltaD is 1). Polling DeltaD freq, using Capture DeltaD (DeltaT is 1). */
	uint32_t UnitLinearD;					/*!< Linear D unit conversion factor. Units per TimerCounter tick, using Capture DeltaD (DeltaT is 1). Units per DeltaT capture, using Capture DeltaT (DeltaD is 1).*/
	uint32_t UnitLinearSpeed;				/*!< [UnitD * UnitT_Freq] 						=> Speed = DeltaD * UnitSpeed / DeltaT */
	uint32_t UnitAngularD; 					/*!< [0xFFFFFFFFU/CountsPerRevolution + 1] 		=> Angle = CounterD * UnitAngle_Factor >> (32 - DEGREES_BITS) */
	uint32_t UnitAngularSpeed; 				/*!< [(1 << DEGREES_BITS) * UnitT_Freq / CountsPerRevolution] 	=> AngularSpeed = DeltaD * UnitAngularSpeed / DeltaT */
	uint32_t UnitInterpolateAngle; 			/*!< [UnitT_Freq << DEGREES_BITS / POLLING_FREQ / CountsPerRevolution] */
	uint32_t InterpolateAngleLimit;

	uint32_t UnitScalarSpeed;				/*!< Percentage Speed of ScalarSpeedRef_Rpm, given max speed, as Fraction16 */
	uint32_t UnitScalarSpeed2;				/*!<   */
	uint32_t DeltaTUnitScalarSpeed;				/*!<   */
	uint32_t ModeDTUnitScalarSpeed;				/*!<   */
	uint32_t DeltaDUnitScalarSpeed;				/*!<   */
	//	uint32_t UnitInterpolateDistance_Factor;		/*!< [UnitD * UnitT_Freq] => D = index * DeltaD * UnitInterpolateD_Factor / POLLING_FREQ */
}
Encoder_T;


/*
	TIMER_FREQ time, 16-bit Timer, SAMPLE_FREQ 1000Hz (1ms)
	0xFFFF/50[Mhz]*1000 = 1.31[ms]
	0xFFFF/65.536[Mhz]*1000 = 1[ms]

	Mode DeltaT,
	Min Speed for Interpolation,

	UnitAngularSpeed Shift Divide
	TIMER_FREQ < UINT32_MAX * CountsPerRevolution / (1 << ENCODER_ANGLE16) ~= 1Mhz-4Mhz for Unsigned Speed
	TIMER_FREQ * 60 < UINT32_MAX for RPM calc
*/

#define ENCODER_INIT_HW_DECODER(p_Hal_Encoder, PollingFreq, SpeedSampleFreq, p_ExtendedTimer, ExtendedTimerFreq, p_Params)	\
{														\
	.CONFIG = 											\
	{													\
		.P_HAL_ENCODER_COUNTER 	= p_Hal_Encoder,		\
		.POLLING_FREQ 			= PollingFreq,			\
		.SAMPLE_FREQ 			= SpeedSampleFreq, 		\
		.P_EXTENDED_TIMER 		= p_ExtendedTimer,		\
		.EXTENDED_TIMER_FREQ 	= ExtendedTimerFreq,	\
		.P_PARAMS 				= p_Params,				\
	},													\
}

#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
#define _ENCODER_INIT_HW_EMULATED_QUADRATURE(p_PinA_Hal, PinAId, p_PinB_Hal, PinBId)	\
		.PinA = PIN_INIT(p_PinA_Hal, PinAId),											\
		.PinB = PIN_INIT(p_PinB_Hal, PinBId),
#else
#define _ENCODER_INIT_HW_EMULATED_QUADRATURE(p_PinA_Hal, PinAId, p_PinB_Hal, PinBId)
#endif

//todo aff hw counter
#define ENCODER_INIT(p_PhaseA_Hal, PhaseAId, p_PinA_Hal, PinAId, p_PhaseB_Hal, PhaseBId, p_PinB_Hal, PinBId, p_PhaseZ_Hal, PhaseZId, p_Timer_Hal, TimerFreq, PollingFreq, SpeedSampleFreq, p_ExtendedTimer, ExtendedTimerFreq, p_Params )	\
{																										\
	.CONFIG = 																							\
	{																									\
		.P_HAL_ENCODER_A 		= p_PhaseA_Hal,			.PHASE_A_ID 			= PhaseAId,				\
		.P_HAL_ENCODER_B 		= p_PhaseB_Hal,			.PHASE_B_ID 			= PhaseBId,				\
		.P_HAL_ENCODER_Z 		= p_PhaseZ_Hal,			.PHASE_Z_ID 			= PhaseZId,				\
		.P_HAL_ENCODER_TIMER 	= p_Timer_Hal,			.TIMER_FREQ 			= TimerFreq,			\
		.POLLING_FREQ 			= PollingFreq,			.SAMPLE_FREQ 			= SpeedSampleFreq, 		\
		.P_EXTENDED_TIMER 		= p_ExtendedTimer,		.EXTENDED_TIMER_FREQ 	= ExtendedTimerFreq,	\
		.P_PARAMS 				= p_Params,																\
	},																									\
	_ENCODER_INIT_HW_EMULATED_QUADRATURE(p_PinA_Hal, PinAId, p_PinB_Hal, PinBId)						\
}

/******************************************************************************/
/*!
	@brief 	SW Capture Functions - Emulated ModeD, ModeDT
*/
/******************************************************************************/
/******************************************************************************/
/*
	Quadrature, Signed Direction
*/
/******************************************************************************/
static inline uint8_t _Encoder_CapturePhasesState(Encoder_T * p_encoder)
{
	p_encoder->Phases.PrevA = p_encoder->Phases.A;
	p_encoder->Phases.PrevB = p_encoder->Phases.B;
	p_encoder->Phases.A = Pin_Input_ReadPhysical(&p_encoder->PinA);
	p_encoder->Phases.B = Pin_Input_ReadPhysical(&p_encoder->PinB);
	return p_encoder->Phases.State;
}

static inline void _Encoder_CaptureCount(Encoder_T * p_encoder, int8_t count)
{
	if(count == _ENCODER_TABLE_ERROR) { p_encoder->ErrorCount++; }
	else
	{
		p_encoder->CounterD += count;
		p_encoder->Angle32 += ((int32_t)count * (int32_t)p_encoder->UnitAngularD);
	}
}

static inline void _Encoder_CaptureCounterD_Quadrature(Encoder_T * p_encoder)
{
	_Encoder_CaptureCount(p_encoder, _ENCODER_TABLE[_Encoder_CapturePhasesState(p_encoder)]);
}

static inline void _Encoder_CaptureCounterD_QuadraturePhaseA(Encoder_T * p_encoder)
{
	_Encoder_CaptureCount(p_encoder, _ENCODER_TABLE_PHASE_A[_Encoder_CapturePhasesState(p_encoder)]);
}

static inline void _Encoder_CaptureCounterD_QuadraturePhaseALeadingEdge(Encoder_T * p_encoder)
{
	int8_t count = (Pin_Input_ReadPhysical(&p_encoder->PinB) == false) ? 1 : -1;
	p_encoder->CounterD += count;
	p_encoder->Angle32 += ((int32_t)count * p_encoder->UnitAngularD);
}

/******************************************************************************/
/*
	Single Phase, Unsigned Direction
*/
/******************************************************************************/
static inline void _Encoder_CaptureCounterD_Inc(Encoder_T * p_encoder)
{
	p_encoder->CounterD++;
	p_encoder->Angle32 += p_encoder->UnitAngularD;
}

/******************************************************************************/
/*
	Quadrature On/Off Switch
*/
/******************************************************************************/
static inline void _Encoder_CaptureCounterD(Encoder_T * p_encoder)
{
#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
	if(p_encoder->Params.IsQuadratureCaptureEnabled == true) { _Encoder_CaptureCounterD_Quadrature(p_encoder); }
	else
#endif
	{ _Encoder_CaptureCounterD_Inc(p_encoder); }
}


/*!
	@brief 	Capture Increasing DeltaT or DeltaD between 2 samples. Used speed calculations.
			Filtering Delta handle by caller.
*/
static inline uint32_t _Encoder_CaptureDelta(Encoder_T * p_encoder, uint32_t timerCounterMax, uint32_t timerCounterValue)
{
	uint32_t delta = (timerCounterValue < p_encoder->TimerCounterPrev) ?
		(timerCounterMax - p_encoder->TimerCounterPrev + timerCounterValue + 1U) :	/* TimerCounter overflow */
		(timerCounterValue - p_encoder->TimerCounterPrev); 						/* Normal case */
	p_encoder->TimerCounterPrev = timerCounterValue;
	return delta;
}

static inline void _Encoder_ZeroAngle(Encoder_T * p_encoder)
{
	p_encoder->CounterD = 0U;
	p_encoder->Angle32 = 0U;
}

static inline uint32_t _Encoder_GetAngle32(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	return HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER) * (int32_t)p_encoder->UnitAngularD;
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	return p_encoder->Angle32;
#endif
}

/* adjust for capture   */
static inline int32_t _Encoder_GetDirectionValue(Encoder_T * p_encoder, int32_t value)
{
	return (p_encoder->Params.IsALeadBPositive == true) ? value : 0 - value;
}

static inline uint16_t Encoder_GetAngle(Encoder_T * p_encoder)
{
	uint16_t angle = _Encoder_GetAngle32(p_encoder) >> 16U;
	return (p_encoder->Params.IsALeadBPositive == true) ? angle : 0 - angle;
}


/******************************************************************************/
/*!
	@brief 	 CounterD conversions. For DeltaD
*/
/*! @{ */
/******************************************************************************/
/******************************************************************************/
/*!
	Angle
	Base unit in ENCODER_ANGLE16
*/
/******************************************************************************/
static inline uint32_t Encoder_ConvertCounterDToAngle(Encoder_T * p_encoder, uint32_t counterD_Ticks)
{
	/* Overflow: counterD > CountsPerRevolution, UnitAngularD == UINT32_MAX / CountsPerRevolution */
	return (counterD_Ticks > p_encoder->Params.CountsPerRevolution) ?
		((counterD_Ticks << ENCODER_ANGLE16) / p_encoder->Params.CountsPerRevolution) :
		((counterD_Ticks * p_encoder->UnitAngularD) >> (32U - ENCODER_ANGLE16));
}

static inline uint32_t Encoder_ConvertAngleToCounterD(Encoder_T * p_encoder, uint16_t angle_UserDegrees)
{
	return (angle_UserDegrees << (32U - ENCODER_ANGLE16)) / p_encoder->UnitAngularD;
}

static inline uint32_t Encoder_GetCounterD(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	return HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	return p_encoder->CounterD;
#endif
}

/******************************************************************************/
/*!
	Linear Distance
*/
/******************************************************************************/
static inline uint32_t Encoder_ConvertCounterDToDistance(Encoder_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks * p_encoder->UnitLinearD; }
static inline uint32_t Encoder_ConvertDistanceToCounterD(Encoder_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks / p_encoder->UnitLinearD; }
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
	Base unit in ENCODER_ANGLE16 Per Second
*/
/******************************************************************************/
/*

*/
static inline uint32_t _Encoder_CalcAngularSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitAngularSpeed / deltaT_Ticks;
}

static inline int32_t _Encoder_CalcAngularVelocity(Encoder_T * p_encoder, int32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitAngularSpeed / deltaT_Ticks;
}

/*!
	Revolution per Second
*/
/*
	DeltaT Mode process at least 1 division, use over conversion via UnitAngularSpeed, calculation when << 16 is too large
*/
static inline uint32_t _Encoder_CalcRotationalSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return (deltaD_Ticks * p_encoder->UnitT_Freq) / (p_encoder->Params.CountsPerRevolution * deltaT_Ticks);
}

/*
	Using Capture DeltaD, will allow at least 1 full rotation between polling without overflow
	AngularSpeed is already rotational speed shifted to highest possible resolution
*/
static inline uint32_t _Encoder_CalcRotationalSpeed_Shift(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return _Encoder_CalcAngularSpeed(p_encoder, deltaD_Ticks, deltaT_Ticks) >> ENCODER_ANGLE16;
}

static inline int32_t _Encoder_CalcRotationalVelocity_Shift(Encoder_T * p_encoder, int32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return _Encoder_CalcAngularVelocity(p_encoder, deltaD_Ticks, deltaT_Ticks) >> ENCODER_ANGLE16;
}

/******************************************************************************/
/*!
	Scalar Speed Functions
	RotationalSpeed_RPS * 65556 * 60 / ScalarSpeedRef_Rpm

	Unsigned Frac16 65556 <=> 1
	Can over saturate 1
*/
/******************************************************************************/
static inline uint32_t Encoder_CalcScalarSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitScalarSpeed / deltaT_Ticks;
}

static inline int32_t Encoder_CalcScalarVelocity(Encoder_T * p_encoder, int32_t deltaD_Ticks, uint32_t deltaT_Ticks)
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
	@brief 	Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void _Encoder_ResetUnitsAngular(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsInterpolateAngle(Encoder_T * p_encoder);
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
