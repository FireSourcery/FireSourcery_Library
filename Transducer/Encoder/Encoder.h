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
	@brief 	Encoder Speed, Position
			Encoder common file contains unit conversions.
	@version V0
*/
/******************************************************************************/
#ifndef ENCODER_H
#define ENCODER_H

#include "HAL_Encoder.h"
#include "Config.h"
#include "Peripheral/Pin/Pin.h"

#include "Math/Q/Q.h"
#include "Math/math_general.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef ENCODER_TIMER_MAX
#define ENCODER_TIMER_MAX (0xFFFFU)
#endif

#ifndef ENCODER_ANGLE16
#define ENCODER_ANGLE16 (16U)
#endif

#define ENCODER_ANGLE_DEGREES 	((uint32_t)1UL << ENCODER_ANGLE16)
#define ENCODER_ANGLE_SHIFT 	(32U - ENCODER_ANGLE16)

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


typedef enum Encoder_Align_Tag
{
	ENCODER_ALIGN_NO,
	ENCODER_ALIGN_YES,
	ENCODER_ALIGN_ABSOLUTE,
}
Encoder_Align_T;

typedef struct __attribute__((aligned(2U))) Encoder_Params_Tag
{
	uint16_t CountsPerRevolution; 		/* Derive Angular Units. Max for counting AngularD, CaptureDeltaT mode need 2nd TimerCounterMax */
	uint16_t ScalarSpeedRef_Rpm; 		/* Derive Frac16 Units. */
	uint16_t SurfaceDiameter;			/* Derive Linear Units. */
	uint16_t GearRatio_Factor;			/* Derive Linear Units. Surface:Encoder Ratio */
	uint16_t GearRatio_Divisor;			/* Derive Linear Units. */
	/* DistancePerRevolution_Factor, DistancePerRevolution_Factor_Divider */
	uint16_t ExtendedDeltaTStop;		/* ExtendedTimer time read as deltaT stopped, default as 1s or .5rpm */
	uint32_t InterpolateAngleScalar;	/* Sets UnitInterpolateAngle Scalar and InterpolateAngleLimit */
#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
	bool IsQuadratureCaptureEnabled; 	/* Quadrature Mode - enable hardware/emulated quadrature speed capture */
	bool IsALeadBPositive; 				/* User runtime calibration for encoder install direction, combine with compile time defined QUADRATURE_A_LEAD_B_INCREMENT */
#endif
}
Encoder_Params_T;
typedef const struct Encoder_Config_Tag
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	HAL_Encoder_Counter_T * const P_HAL_ENCODER_COUNTER; /*!< Pulse Counter */
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	HAL_Encoder_Phase_T * const P_HAL_ENCODER_A; const uint32_t PHASE_A_ID;
	HAL_Encoder_Phase_T * const P_HAL_ENCODER_B; const uint32_t PHASE_B_ID;
	HAL_Encoder_Phase_T * const P_HAL_ENCODER_Z; const uint32_t PHASE_Z_ID;
#endif
	HAL_Encoder_Timer_T * const P_HAL_ENCODER_TIMER; 	/*!< DeltaT Timer. */
	const uint32_t TIMER_FREQ;							/*!< DeltaT Timer Freq */
	const volatile uint32_t * const P_EXTENDED_TIMER;	/* Polling DeltaT stop */
	const uint32_t EXTENDED_TIMER_FREQ;
	const uint32_t POLLING_FREQ;		/*!< DeltaT Interpolation Freq. */
	const uint32_t SAMPLE_FREQ; 		/*!< DeltaD Speed Sample Freq. */
	const Encoder_Params_T * P_PARAMS;
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
		Compute-Time Variables
	*/
#if (defined(CONFIG_ENCODER_HW_EMULATED) && defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE))
	Encoder_Phases_T Phases; /* Save Prev State */
#endif
	int32_t CounterD;
	uint32_t Angle32;
	int32_t DeltaD; 		/*!< Counter distance counts between 2 samples. Units in raw counter ticks */
	uint32_t DeltaT; 		/*!< Timer time counts between 2 Encoder pulse counts. Units in raw timer ticks */
	uint32_t DeltaTh; 		/*!< ModeDT */
	int32_t FreqD; 			/*!< EncoderPulseFreq ModeDT */
	uint32_t InterpolateAngleIndex; /*!< ModeDT */
	int32_t DirectionD; 	/*!< DeltaD sign, when DeltaD == 0 */
	uint32_t ErrorCount;
	uint32_t IndexCount;
	// int32_t IndexCounterDOffset;
	uint32_t ExtendedTimerPrev;
	uint32_t ExtendedTimerConversion;	/* Extended Timer to Short Timer */
	bool IsSinglePhasePositive;
	Encoder_Align_T Align;
	int32_t AbsoluteOffset;

	/* Experimental */
	int32_t TotalD; 			/* Integral Capture */
	// uint32_t TotalT; 		/*!< Extended? Timer ticks, persistent through DeltaT captures */
	// uint32_t SpeedPrev; 		/*!< Most recently calculated speed, use for acceleration */
	// uint32_t DeltaSpeed; 	/*!< Save for acceleration calc */

	/*
		Unit conversion. derived on init from Nv Params
	*/
	uint32_t UnitT_Freq;					/*!< Common Units propagating set depending on mode. T unit(seconds) conversion factor. */
	uint32_t UnitAngularD; 					/*!< [0xFFFFFFFFU/CountsPerRevolution + 1] 		=> Angle = CounterD * UnitAngle_Factor >> (32 - DEGREES_BITS) */
	uint32_t UnitLinearD;					/*!< Linear D unit conversion factor. Units per TimerCounter tick, using Capture DeltaD (DeltaT is 1). Units per DeltaT capture, using Capture DeltaT (DeltaD is 1).*/
	uint32_t UnitInterpolateAngle; 			/*!< [UnitT_Freq << DEGREES_BITS / POLLING_FREQ / CountsPerRevolution] */
	uint32_t InterpolateAngleLimit;
	uint32_t UnitAngularSpeed; 				/*!< [(1 << DEGREES_BITS) * UnitT_Freq / CountsPerRevolution] 	=> AngularSpeed = DeltaD * UnitAngularSpeed / DeltaT */
	uint32_t UnitSurfaceSpeed;				/*!< [UnitD * UnitT_Freq] 						=> Speed = DeltaD * UnitSpeed / DeltaT */
	uint32_t UnitScalarSpeed;				/*!< Percentage Speed of ScalarSpeedRef_Rpm, given max speed, as Fraction16 */
	uint8_t UnitSurfaceSpeedShift;			/* Shifts applicable to ModeD/DT */
	uint8_t UnitAngularSpeedShift;
	uint8_t UnitScalarSpeedShift;
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
#if defined(CONFIG_ENCODER_HW_EMULATED)
#define _ENCODER_INIT_HW_PHASES(p_Counter_Hal, p_PhaseA_Hal, PhaseAId, p_PhaseB_Hal, PhaseBId, p_PhaseZ_Hal, PhaseZId)		\
		.P_HAL_ENCODER_A = p_PhaseA_Hal, .PHASE_A_ID = PhaseAId, 	\
		.P_HAL_ENCODER_B = p_PhaseB_Hal, .PHASE_B_ID = PhaseBId, 	\
		.P_HAL_ENCODER_Z = p_PhaseZ_Hal, .PHASE_Z_ID = PhaseZId,
#define _ENCODER_INIT_HW_PINS(p_PinA_Hal, PinAId, p_PinB_Hal, PinBId) 	\
		.PinA = PIN_INIT(p_PinA_Hal, PinAId), 							\
		.PinB = PIN_INIT(p_PinB_Hal, PinBId),
#else
#define _ENCODER_INIT_HW_PHASES(p_Counter_Hal, p_PhaseA_Hal, PhaseAId, p_PhaseB_Hal, PhaseBId, p_PhaseZ_Hal, PhaseZId)		\
		.P_HAL_ENCODER_COUNTER 	= p_Counter_Hal, \
#define _ENCODER_INIT_HW_PINS(p_PinA_Hal, PinAId, p_PinB_Hal, PinBId)
#endif

#define ENCODER_INIT(p_Counter_Hal, p_PhaseA_Hal, PhaseAId, p_PinA_Hal, PinAId, p_PhaseB_Hal, PhaseBId, p_PinB_Hal, PinBId, p_PhaseZ_Hal, PhaseZId, p_Timer_Hal, TimerFreq, PollingFreq, SpeedSampleFreq, p_ExtendedTimer, ExtendedTimerFreq, p_Params )	\
{																														\
	.CONFIG = 																											\
	{																													\
		_ENCODER_INIT_HW_PHASES(p_Counter_Hal, p_PhaseA_Hal, PhaseAId, p_PhaseB_Hal, PhaseBId, p_PhaseZ_Hal, PhaseZId) 	\
		.P_HAL_ENCODER_TIMER 	= p_Timer_Hal,			.TIMER_FREQ 			= TimerFreq,							\
		.P_EXTENDED_TIMER 		= p_ExtendedTimer,		.EXTENDED_TIMER_FREQ 	= ExtendedTimerFreq,					\
		.POLLING_FREQ 			= PollingFreq,			.SAMPLE_FREQ 			= SpeedSampleFreq, 						\
		.P_PARAMS 				= p_Params,																				\
	},																													\
	_ENCODER_INIT_HW_PINS(p_PinA_Hal, PinAId, p_PinB_Hal, PinBId)														\
}

/******************************************************************************/
/*!
	@brief  Function Templates
*/
/******************************************************************************/
typedef void(*Encoder_CaptureModeFunction_T)(Encoder_T * p_encoder);

static inline void Encoder_ProcCaptureModeFunction(Encoder_T * p_encoder, Encoder_CaptureModeFunction_T quadratureFunction, Encoder_CaptureModeFunction_T singlePhaseFunction)
{
#if 	defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
	if(p_encoder->Params.IsQuadratureCaptureEnabled == true) 	{ quadratureFunction(p_encoder); }
	else 														{ singlePhaseFunction(p_encoder); }
#else
	(void)quadratureFunction; singlePhaseFunction(p_encoder);
#endif
}

typedef int32_t(*Encoder_CaptureModeFunction_Value_T)(Encoder_T * p_encoder);

static inline int32_t Encoder_ProcCaptureModeFunction_Value(Encoder_T * p_encoder, Encoder_CaptureModeFunction_Value_T quadratureFunction, Encoder_CaptureModeFunction_Value_T singlePhaseFunction)
{
#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
	return (p_encoder->Params.IsQuadratureCaptureEnabled == true) ? quadratureFunction(p_encoder) : singlePhaseFunction(p_encoder);
#else
	(void)quadratureFunction; return singlePhaseFunction(p_encoder);
#endif
}

/******************************************************************************/
/*!
	@brief
*/
/******************************************************************************/
static inline void _Encoder_ZeroPulseCount(Encoder_T * p_encoder)
{
	p_encoder->CounterD = 0U;
	p_encoder->IndexCount = 0U;
}

/* Convert signed capture to user reference */
static inline int32_t Encoder_GetDirection_Quadrature(Encoder_T * p_encoder) { return (p_encoder->Params.IsALeadBPositive == true) ? 1 : -1; }
/* set by user */
static inline int32_t Encoder_GetDirection_SinglePhase(Encoder_T * p_encoder) { return (p_encoder->IsSinglePhasePositive == true) ? 1 : -1; }

static inline int32_t Encoder_GetDirection(Encoder_T * p_encoder)
{
	return Encoder_ProcCaptureModeFunction_Value(p_encoder, Encoder_GetDirection_Quadrature, Encoder_GetDirection_SinglePhase);
}

static inline int32_t Encoder_GetDirectionValue(Encoder_T * p_encoder, int32_t value)
{
	return Encoder_GetDirection(p_encoder) * value;
}

static inline uint32_t _Encoder_GetAngle32(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	return HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER) * (int32_t)p_encoder->UnitAngularD;
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	return p_encoder->Angle32;
#endif
}

static inline uint16_t _Encoder_GetAngle(Encoder_T * p_encoder)
{
	return _Encoder_GetAngle32(p_encoder) >> 16U;
}

static inline uint16_t Encoder_GetAngle_SinglePhase(Encoder_T * p_encoder)
{
	uint16_t angle = _Encoder_GetAngle(p_encoder);
	return (p_encoder->IsSinglePhasePositive == true) ? angle : 0 - angle;
}

static inline uint16_t Encoder_GetAngle_Quadrature(Encoder_T * p_encoder)
{
	uint16_t angle = _Encoder_GetAngle(p_encoder);
	return (p_encoder->Params.IsALeadBPositive == true) ? angle : 0 - angle;
}

static inline uint16_t Encoder_GetAngle(Encoder_T * p_encoder)
{
	return Encoder_GetDirection(p_encoder) * _Encoder_GetAngle(p_encoder);
}

static inline bool Encoder_GetIsAligned(Encoder_T * p_encoder)
{
	return (p_encoder->Align == ENCODER_ALIGN_ABSOLUTE) || (p_encoder->Align == ENCODER_ALIGN_YES);
}

/******************************************************************************/
/*!
	@brief 	 CounterD conversions.
*/
/******************************************************************************/
/*!
	Angle - Base unit in ENCODER_ANGLE16
*/
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

static inline int32_t Encoder_GetCounterD(Encoder_T * p_encoder)
{
#if 	defined(CONFIG_ENCODER_HW_DECODER)
	return HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
#elif 	defined(CONFIG_ENCODER_HW_EMULATED)
	return p_encoder->CounterD;
#endif
}

/*!
	Linear Distance
*/
static inline uint32_t Encoder_ConvertCounterDToDistance(Encoder_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks * p_encoder->UnitLinearD; }
static inline uint32_t Encoder_ConvertDistanceToCounterD(Encoder_T * p_encoder, uint32_t counterD_Ticks) { return counterD_Ticks / p_encoder->UnitLinearD; }

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
	Only When base units in mm, as set via SetGroundRatio function.
*/
static inline uint32_t Encoder_CalcGroundSpeed_Mph(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitSurfaceSpeed * 60U * 60U / (deltaT_Ticks * 1609344U);
}

static inline uint32_t Encoder_CalcGroundSpeed_Kmh(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitSurfaceSpeed * 60U * 60U / (deltaT_Ticks * 1000000U);
}

/******************************************************************************/
/*!
	@brief 	Extern Declarations
*/
/*! @{ */
/******************************************************************************/
extern void Encoder_InitInterrupts_Quadrature(Encoder_T * p_encoder);
extern void Encoder_InitInterrupts_ABC(Encoder_T * p_encoder);

extern void Encoder_SetSinglePhaseDirection(Encoder_T * p_encoder, bool isPositive);
extern void Encoder_CalibrateAlignZero(Encoder_T * p_encoder);
extern void Encoder_CalibrateAlignValidate(Encoder_T * p_encoder);

#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
extern void Encoder_SetQuadratureMode(Encoder_T * p_encoder, bool isEnabled);
extern void Encoder_SetQuadratureDirectionCalibration(Encoder_T * p_encoder, bool isALeadBPositive);
extern void Encoder_CalibrateQuadratureReference(Encoder_T * p_encoder);
extern void Encoder_CalibrateQuadraturePositive(Encoder_T * p_encoder);
#endif

extern void _Encoder_ResetUnits(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsAngle(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsInterpolateAngle(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsAngularSpeed(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsLinearSpeed(Encoder_T * p_encoder);
extern void _Encoder_ResetUnitsScalarSpeed(Encoder_T * p_encoder);
extern void Encoder_SetCountsPerRevolution(Encoder_T * p_encoder, uint16_t countsPerRevolution);
extern void Encoder_SetDistancePerRevolution(Encoder_T * p_encoder, uint16_t distancePerCount);
extern void Encoder_SetScalarSpeedRef(Encoder_T * p_encoder, uint16_t speedRef);
extern void Encoder_SetSurfaceRatio(Encoder_T * p_encoder, uint32_t surfaceDiameter, uint32_t gearRatio_Factor, uint32_t gearRatio_Divisor);
extern void Encoder_SetGroundRatio_US(Encoder_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
extern void Encoder_SetGroundRatio_Metric(Encoder_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
