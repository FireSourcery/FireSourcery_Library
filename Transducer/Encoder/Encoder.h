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

/*
 * Linear
 *
 * DeltaTime[s]												= DeltaT[TimerTicks] * UnitT[s] = DeltaT[TimerTicks] / UnitT_Freq[Hz]
 * DeltaDistance[Units]										= DeltaD[CounterTicks] * UnitD[Units]
 * Speed[Units/s] == DeltaDistance[Units] / DeltaTime[s] 	= DeltaD[CounterTicks] * UnitD[Units] * UnitT_Freq[Hz] / DeltaT[TimerTicks]
 * UnitSpeed												= UnitD[Units] * UnitT_Freq[Hz]
 */
/*
 * LinearDistance[Units]	= DeltaD[EncoderTicks] * DistancePerRevolution[Units/1] / CountsPerRevolution[EncoderTicks/1] = DeltaD[EncoderTicks] * UnitD[Units]
 * LinearSpeed[Units/s] 	= LinearDistance[Units]/DeltaTime[s] = DeltaD[Ticks] * UnitD[Units] * UnitT_Freq[Hz] / DeltaT[Ticks]
 */
/*
 * Rotational/Angular
 *
 * User Input:
 * unitAngle_Bits			Angle unit conversion. Degrees per revolution in Bits. e.g. 16 for 65535 degrees cycle
 * unitAngle_SensorResolution 	Angle unit conversion. Encoder Pulses Per Revolution
 *
 * unitLinearDistance			Linear unit conversion factor. Set to UnitD. Encoder [Distance Per Revolution]/[Pulses Per Revolution]. Enforce user input whole number value.
 * unitLinearSpeed				Linear Speed conversion factor. Set to UnitSpeed.
 */
/*
 * Revolutions[N]											= DeltaD[EncoderTicks] / CountsPerRevolution[EncoderTicks/1]
 * RotationalSpeed[N/s]	== Revolutions[N] / DeltaTime[s] 	= DeltaD[EncoderTicks] * UnitT_Freq[Hz] / CountsPerRevolution[EncoderTicks/1] / DeltaT[TimerTicks]
 *
 * DeltaAngle[DegreesNBits]			= DeltaD[EncoderTicks] * DegreesPerRevolution[DegreesNBits/1] / CountsPerRevolution[EncoderTicks/1]
 * AngularSpeed[DegreesNBits/s] 	= DeltaD[EncoderTicks] * DegreesPerRevolution[DegreesNBits/1] * UnitT_Freq[Hz] / CountsPerRevolution[EncoderTicks/1] / DeltaT[TimerTicks]
 * UnitAngle[DegreesNBits]			= DegreesPerRevolution[DegreesNBits/1]/CountsPerRevolution[EncoderTicks/1]
 *
 * Angle Fast Divide:
 *  Angle = D * UnitAngle_Factor >> UnitAngle_DivisorShift
 *
 */

typedef enum
{
	ENCODER_DIRECTION_DIRECT,		//DeltaD mode increasing is positive. DeltaT Mode ALeadB is positive
	ENCODER_DIRECTION_REVERSE,
}
Encoder_DirectionCalibration_T;

typedef struct
{
	uint32_t CountsPerRevolution;
	uint32_t DistancePerCount;
	/* Quadrature Mode - Calibrate for encoder install direction */

	//DeltaD Mode also check CONFIG_ENCODER_HW_QUADRATURE_CAPABLE //chip peripheral capture ticks up and down
	bool IsQuadratureCaptureEnabled;
	bool IsALeadBPositive;
	//derived calibration,
//	 * isALeadBIncrement - Match to HAL/unused for deltaT Mode
//	 * isALeadBPositive - User runtime calibrate
//	Encoder_DirectionCalibration_T DirectionCalibration;

//	uint32_t ExtendedTimerDeltaTOverflow; 	//short timer overflow time in long timer counts
	uint32_t ExtendedTimerDeltaTStop;		//ExtendedTimer time read at deltaT stopped

	/* Motor Hall Mode */
	uint8_t MotorPolePairs; /*! Motor sub type, Convert between electrical speed and mechanical speed */
}
Encoder_Params_T;

typedef const struct
{
	/* HW Config */
	HAL_Encoder_T * const P_HAL_ENCODER; /*!< DeltaT and DeltaD timer counter */
	//#ifdef phaseAB use pin or use encoder hal
	const Pin_T PIN_PHASE_A;
	const Pin_T PIN_PHASE_B;
	const uint32_t POLLING_FREQ;		/*!< InterpolationFreq, Polling D reference Freq, CaptureDeltaT mode need 2nd freq interpolation calculations */
	const uint32_t DELTA_T_TIMER_FREQ;


	const uint32_t DELTA_D_CAPTURE_FREQ;		/*!< InterpolationFreq, Polling D reference Freq, CaptureDeltaT mode need 2nd freq interpolation calculations */


	/* Polling DeltaT stop */
	const volatile uint32_t * const P_EXTENDED_TIMER;
	const uint32_t EXTENDED_TIMER_FREQ;

	const Encoder_Params_T * P_PARAMS;
}
Encoder_Config_T;

/*
	Overallocation of variable space to allow runtime polymorphism
 */
typedef struct
{
	const Encoder_Config_T CONFIG;
	Encoder_Params_T Params;

//	/*
//	 * Non Volatile Parameters
//	 */
//	uint32_t EncoderResolution;				/*!< Max for looping AngularD, CaptureDeltaT mode need 2nd TimerCounterMax */
//
//	/* Quadrature Mode - Calibrate for encoder install direction */
//#ifdef CONFIG_ENCODER_HW_QUADRATURE_CAPABLE //chip peripheral capture ticks up and down
//	bool IsQuadratureCaptureEnabled;
//	bool IsALeadBDirectionPositive; //is ALeadB positive angle //combined //user set //HAL must be set so ALeadB is Increasing
////	bool IsALeadBCounterIncrement; // determined by HAL
////	bool IsCounterIncrementDirectionPositive; //derived calibration
//#endif
//
//	/* Motor Encoder */
//	uint8_t PolePairs; /*! Convert between electrical speed and mechanical speed */
//	//    uint32_t UnitEletricalAngle_Factor;
//	//    uint32_t UnitEletricalAngle_DivisorShift;
//	//    uint32_t UnitEletricalSpeed;
//
//	/* Poll DeltaT stopped */
//	uint32_t ExtendedDeltaTimerThreshold; //short timer overflow time in long timer counts
//	uint32_t ExtendedDeltaTimerEffectiveStopTime;

	/*
	 * Runtime Variables
	 */
	uint32_t TimerCounterSaved; /*!< First time/count sample used to calculate Delta */
	uint32_t ExtendedTimerSaved;

	uint32_t DeltaT; 	/*!< Captured TimerCounter time interval counts between 2 distance events. Units in raw timer ticks */
	uint32_t DeltaD; 	/*!< Captured TimerCounter distance interval counts between 2 points in time. Units in raw timer ticks */
	uint32_t AngularD; 	/*!< Looping TotalD at EncoderResolution. TotalAngularD */

	uint32_t TotalT; /*!< TimerCounter ticks, persistent through Delta captures*/
	int32_t TotalD;

	uint32_t SpeedSaved; /*!< Most recently calculated speed, used for acceleration calc */
	uint32_t DeltaSpeed; /*!< Save for acceleration calc */

	bool PulseReferenceSaved; /*!< PhaseA state. For polling capture of DeltaT */

    /*
     * Unit conversion.
     * derived on init from Nv Paramss
     */
	uint32_t UnitT_Freq;					/*!< T unit(seconds) conversion factor. TimerCounter freq, using Capture DeltaT (DeltaD is 1). Polling DeltaD freq, using Capture DeltaD (DeltaT is 1). */
	uint32_t UnitLinearD;							/*!< Linear D unit conversion factor. Units per TimerCounter tick, using Capture DeltaD (DeltaT is 1). Units per DeltaT capture, using Capture DeltaT (DeltaD is 1).*/
	uint32_t UnitLinearSpeed;						/*!< [UnitD * UnitT_Freq] => Speed = DeltaD * UnitSpeed / DeltaT */

	uint32_t UnitAngularD_Factor; 			/*!< [0xFFFFFFFFU/unitAngle_SensorResolution + 1] => DeltaAngle = DeltaD * UnitAngle_Factor >> UnitAngle_DivisorShift */
//	uint32_t UnitAngularD_DivisorShift;		/*!< [32 - unitAngle_Bits], 32 - CONFIG_ENCODER_ANGLE_DEGREES_BITS */
	uint32_t UnitAngularSpeed; 				/*!< [(1 << unitAngle_Bits) * UnitT_Freq / unitAngle_SensorResolution] => AngularSpeed = DeltaD * UnitAngularSpeed / DeltaT
												e.g. UnitAngularSpeed == 160,000 { unitAngle_Bits = 16, UnitT_Freq = 20000, unitAngle_SensorResolution = 8912 } */

	uint32_t UnitInterpolateAngle; 			/*!< [UnitT_Freq << angleDataBits / interpolationFreq / encoderCountsPerRevolution] */
	//	uint32_t UnitInterpolateD_Factor;	/*!< [UnitD * UnitT_Freq] => D = index * DeltaD * UnitInterpolateD_Factor / interpolationFreq  */

	/* Motor Encoder */
	//    uint32_t UnitEletricalAngle_Factor;
	//    uint32_t UnitEletricalAngle_DivisorShift;
	//    uint32_t UnitEletricalSpeed;

}
Encoder_T;

#define ENCODER_CONFIG(p_Hal_Encoder, PollingFreq, DeltaDCapture, DeltaTTimerFreq, p_PinA_Hal, PinAId, p_PinB_Hal, PinBId, p_ExtendedTimer, ExtendedTimerFreq, p_Params)	\
{																		\
	.CONFIG = 															\
	{																	\
		.P_HAL_ENCODER 			= p_Hal_Encoder,						\
		.POLLING_FREQ 			= PollingFreq,							\
		.DELTA_D_CAPTURE_FREQ 	= DeltaDCapture, 						\
		.DELTA_T_TIMER_FREQ 	= DeltaTTimerFreq,						\
		.PIN_PHASE_A 			= PIN_CONFIG(p_PinA_Hal, PinAId),		\
		.PIN_PHASE_B 			= PIN_CONFIG(p_PinB_Hal, PinBId),		\
		.P_EXTENDED_TIMER 		= p_ExtendedTimer,						\
		.EXTENDED_TIMER_FREQ 	= ExtendedTimerFreq,					\
		.P_PARAMS 				= p_Params,								\
	}																	\
}
/******************************************************************************/
/*!
	ISRs
 */
/*! @{ */
/******************************************************************************/
static inline void Encoder_ProcTimerCounterOverflow_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER);
	//set overflow direction?
}

/*
 * Index Pin
 */
static inline void Encoder_ProcOnIndex_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
	p_encoder->AngularD = 0U;
}
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

	if (timerCounterValue < p_encoder->TimerCounterSaved) /* TimerCounter overflow */
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
static inline uint32_t _Encoder_MicrosHelper(uint32_t timerTicks, uint32_t timerFreq)
{
	uint32_t micros;

	/* overflows if DeltaT > 4294 */
	if (timerTicks > (UINT32_MAX / 1000000))
	{
		// todo optimize
		/* divide largest by smallest */
		if 		((timerTicks > 1000000) && (1000000 > timerFreq))
		{
			micros = 1000000 * (timerTicks / timerFreq);
		}
		else if ((1000000 > timerTicks) && (timerTicks > timerFreq))
		{
			micros = timerTicks * (1000000 / timerFreq);
		}
		else if((timerTicks > timerFreq) && (timerFreq > 1000000))
		{
//			micros = 1000000 * (timerTicks / timerFreq);
			micros = timerTicks / (timerFreq / 1000000);
		}
		else if ((1000000 > timerFreq) && (timerFreq > timerTicks))
		{
//			micros = timerTicks * (1000000 / timerFreq);
			micros = 1000000 / (timerFreq / timerTicks);
		}
		else if ((timerFreq > timerTicks) && (timerTicks > 1000000))
		{
			micros = timerTicks / (timerFreq / 1000000);
		}
		else if ((timerFreq > 1000000) && (1000000 > timerTicks))
		{
			micros = 1000000 / (timerFreq / timerTicks);
		}
	}
	else
	{
		micros = timerTicks * 1000000 / timerFreq;
	}

	return micros;
}



/******************************************************************************/
/*!
	TotalD TotalT Functions - Both modes
 */
/******************************************************************************/
static inline uint32_t Encoder_ConvertCounterDToUnits(Encoder_T * p_encoder, uint32_t counterD_Ticks)	{return counterD_Ticks * p_encoder->UnitLinearD;}
static inline uint32_t Encoder_ConvertUnitsToCounterD(Encoder_T * p_encoder, uint32_t counterD_Ticks)	{return counterD_Ticks / p_encoder->UnitLinearD;}

/******************************************************************************/
/*!
	@brief Angle Functions
 */
/*! @{ */
/******************************************************************************/
static inline uint32_t Encoder_ConvertCounterDToAngle(Encoder_T * p_encoder, uint32_t counterD_Ticks)
{
	uint32_t angle;

	/* Overflow if counterD > unitAngle_SensorResolution  */
	if(counterD_Ticks > p_encoder->Params.CountsPerRevolution) //(counterD_Ticks * p_encoder->UnitAngularD_Factor) > UINT32_MAX
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

static inline uint32_t Encoder_GetRevolutions(Encoder_T * p_encoder)
{
	return p_encoder->TotalD / p_encoder->Params.CountsPerRevolution;
}

/*!
	Same as integral D
 */
static inline uint32_t Encoder_GetTotalD_Units(Encoder_T * p_encoder)	{return p_encoder->TotalD * p_encoder->UnitLinearD;}

static inline uint32_t Encoder_GetTotalTFreq(Encoder_T * p_encoder)		{return p_encoder->UnitT_Freq / p_encoder->TotalT;}
static inline uint32_t Encoder_GetTotalT_Millis(Encoder_T * p_encoder)	{return p_encoder->TotalT * 1000U / p_encoder->UnitT_Freq;}
static inline uint32_t Encoder_GetTotalT_Micros(Encoder_T * p_encoder)	{return _Encoder_MicrosHelper(p_encoder->TotalT,  p_encoder->UnitT_Freq);}

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
	p_encoder->DeltaSpeed 	= newSpeed - p_encoder->SpeedSaved;
	p_encoder->SpeedSaved 	= newSpeed;
	return newSpeed;
}

/*
 * Units/S/S
 */
//static inline uint32_t Encoder_GetAcceleration(Encoder_T * p_encoder)
//{
//	return p_encoder->DeltaSpeed * p_encoder->UnitT_Freq / p_encoder->DeltaT;
//}
//static inline uint32_t Encoder_GetLinearDeltaDistance(Encoder_T * p_encoder)	{return Encoder_GetDeltaD_Units(p_encoder);}
static inline uint32_t Encoder_GetLinearTotalDistance(Encoder_T * p_encoder)	{return Encoder_GetTotalD_Units(p_encoder);}
static inline uint32_t Encoder_GetLinearSpeed(Encoder_T * p_encoder)			{return Encoder_GetSpeed(p_encoder);}

/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*!
	Angular Speed Functions
 */
/******************************************************************************/
/*
 * Can compiler optimize away passing const 1?
 */
static inline uint32_t Encoder_CalcAngularSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
{
	uint32_t speed;

	if ((deltaD_Ticks > UINT32_MAX / p_encoder->UnitAngularSpeed) || (p_encoder->UnitAngularSpeed == 0U)) //p_encoder->UnitAngularSpeed set to 0U if overflow
	{
		/*
		 When UnitAngularSpeed is too high. Large timer freq (> 16 bits), angle res, small countsPerRevolution
		 */
		 speed = ((deltaD_Ticks * p_encoder->UnitT_Freq / p_encoder->Params.CountsPerRevolution) << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / deltaT_Ticks;
	}
	else
	{
		speed = deltaD_Ticks * p_encoder->UnitAngularSpeed / deltaT_Ticks;
	}

	return speed;
}


static inline uint32_t Encoder_GetAngularSpeed(Encoder_T * p_encoder)
{
	return Encoder_CalcAngularSpeed(p_encoder, p_encoder->DeltaD, p_encoder->DeltaT);
}

static inline uint32_t Encoder_CaptureAvgAngularSpeed(Encoder_T * p_encoder)
{
	uint32_t totalD = (p_encoder->TotalD < 0) ? 0 - p_encoder->TotalD : p_encoder->TotalD;
	uint32_t speed = Encoder_CalcAngularSpeed(p_encoder, totalD, p_encoder->TotalT);
	p_encoder->TotalD = 0U;
	p_encoder->TotalT = 0U;
	return speed;
}


/*!
	Revolution per Second
	//	return (p_encoder->DeltaD * p_encoder->UnitT_Freq) / (p_encoder->Params.CountsPerRevolution * p_encoder->DeltaT);
 */
static inline uint32_t Encoder_GetRotationalSpeed_RPS(Encoder_T * p_encoder)
{
	/*
	 * Angle is already highest possible rotation resolution
	 */
	return Encoder_GetAngularSpeed(p_encoder) >> CONFIG_ENCODER_ANGLE_DEGREES_BITS;
}

//	return (p_encoder->DeltaD * p_encoder->UnitT_Freq * 60U) / (p_encoder->Params.CountsPerRevolution * p_encoder->DeltaT);
static inline uint32_t Encoder_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	//checkoverflow
	return Encoder_GetAngularSpeed(p_encoder) * 60U >> CONFIG_ENCODER_ANGLE_DEGREES_BITS;
}

static inline uint32_t Encoder_GetAvgRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	return  ((p_encoder->TotalD * p_encoder->UnitT_Freq / p_encoder->Params.CountsPerRevolution) ) * 60U / p_encoder->TotalT;

}


/*

 */
static inline uint32_t Encoder_ConvertRotationalSpeedToDeltaAngle_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	if (rpm < (UINT32_MAX >> CONFIG_ENCODER_ANGLE_DEGREES_BITS))
	{
		return (rpm << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / (60U * p_encoder->UnitT_Freq);
	}
	else
	{
		return (1U << CONFIG_ENCODER_ANGLE_DEGREES_BITS) / 60U * rpm / p_encoder->UnitT_Freq;
	}
}

static inline uint32_t Encoder_ConvertDeltaAngleToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t angle_UserDegrees)
{
	return (angle_UserDegrees * p_encoder->UnitT_Freq >> CONFIG_ENCODER_ANGLE_DEGREES_BITS) * 60U;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/
/******************************************************************************/
/*!
	@brief 	Angular Interpolation Estimated Functions
 */
/*! @{ */
/******************************************************************************/

/*
	Polls per encoder count, when polls per encoder count > 1
	only when pollingFreq > Encoder Counts Freq, i.e. 0 encoder counts per poll

	DeltaD Mode, applicable to low speeds
	8192 CPR Encoder, 20Khz Polling Freq => 146 RPM
	encoder count per Poll > 1, use Encoder_ConvertRotationalSpeedToDeltaAngle_RPM
 */
static inline uint32_t Encoder_ConvertRotationalSpeedToInterpolationFreq_RPM(Encoder_T * p_encoder, uint16_t mechRpm)
{
	return p_encoder->CONFIG.POLLING_FREQ * 60U / (p_encoder->Params.CountsPerRevolution * mechRpm);
}

static inline uint32_t Encoder_ConvertInterpolationFreqToRotationalSpeed_RPM(Encoder_T * p_encoder, uint16_t controlPeriods)
{
	return p_encoder->CONFIG.POLLING_FREQ * 60U / (p_encoder->Params.CountsPerRevolution * controlPeriods);
}
/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
	@brief  LinearInterpolation Estimated Functions
 */
/*! @{ */
/******************************************************************************/
///*!
//	@brief CaptureDeltaD Mode: Estimate D using captured DeltaD sample. Assuming constant speed.
//	@param domain unitless
// */
//static inline uint32_t Encoder_InterpolateDistance_Slope(Encoder_T * p_encoder, uint32_t index, uint32_t domain)
//{
//	return index * Encoder_GetDeltaD_Units(p_encoder) / domain;
//}
//
///*!
//	@brief CaptureDeltaT Mode: Estimate DeltaD using captured DeltaT sample. Assuming constant speed.
//
//	Delta Peroid > Control Peroid
//	time domain - if domain is with respect to time =>
//	@param index range [0:interpolationFreq/DeltaTFreq]. interpolationFreq/DeltaT_Freq == [interpolationFreq/(UnitT_Freq/DeltaT)] == [interpolationFreq * DeltaT / UnitT_Freq]
// */
//static inline uint32_t Encoder_InterpolateDistance(Encoder_T * p_encoder, uint32_t index)
//{
////	return index * p_encoder->UnitInterpolateD / p_encoder->DeltaT; /* index * [UnitD * UnitT_Freq / interpolationFreq] / DeltaT */
//	return index * Encoder_GetSpeed(p_encoder) / p_encoder->CONFIG.POLLING_FREQ; 	//index * 1 * [UnitD * UnitT_Freq] / p_encoder->DeltaT / interpolationFreq;
//}

/*!
	Interpolate Delta Angle
 */
//static inline uint32_t Encoder_InterpolateAngle_IndexDomain(Encoder_T * p_encoder, uint32_t index, uint32_t domain)
//{
//	return index * Encoder_GetDeltaAngle(p_encoder) / domain;
//}

//static inline uint32_t Encoder_InterpolateSpeed_Slope(Encoder_T * p_encoder, uint32_t index, uint32_t domain)
//{
//	return index * Encoder_GetSpeed(p_encoder) / domain;
//}

//static inline uint32_t Encoder_InterpolateSpeed(Encoder_T * p_encoder, uint32_t index, uint32_t interpolationFreq)
//{
////	return index * Encoder_GetSpeed(p_encoder) * p_encoder->UnitInterpolatedD / p_encoder->DeltaT;
//
////	index * Encoder_GetAcceleration(p_encoder) / interpolationFreq;
//}
/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
	@brief 	Extern Functions
 */
/*! @{ */
/******************************************************************************/


extern void Encoder_Reset(Encoder_T * p_encoder);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
