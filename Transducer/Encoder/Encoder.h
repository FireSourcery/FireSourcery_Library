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
				Capture DeltaD, variable DeltaD (DeltaT is fixed, == 1).
				Capture DeltaT, variable DeltaT (DeltaD is fixed, == 1).

			Encoder defined in Counts Per Revolution and Distance Per Count

	@version V0
 */
/******************************************************************************/
#ifndef ENCODER_H
#define ENCODER_H

#include "HAL.h"
#include "Config.h"

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
 * LinearDistance[Units]	= DeltaD[EncoderTicks] * DistancePerRevolution[Units/1] / PulsePerRevolution[EncoderTicks/1] = DeltaD[EncoderTicks] * UnitD[Units]
 * LinearSpeed[Units/s] 	= LinearDistance[Units]/DeltaTime[s] = DeltaD[Ticks] * UnitD[Units] * UnitT_Freq[Hz] / DeltaT[Ticks]
 */
/*
 * Rotational/Angular
 *
 * User Input:
 * unitAngle_DataBits			Angle unit conversion. Degrees per revolution in Bits. e.g. 16 for 65535 degrees cycle
 * unitAngle_SensorResolution 	Angle unit conversion. Encoder Pulses Per Revolution
 *
 * unitLinearDistance			Linear unit conversion factor. Set to UnitD. Encoder [Distance Per Revolution]/[Pulses Per Revolution]. Enforce user input whole number value.
 * unitLinearSpeed				Linear Speed conversion factor. Set to UnitSpeed.
 */
/*
 * Revolutions[N]											= DeltaD[EncoderTicks] / PulsePerRevolution[EncoderTicks/1]
 * RotationalSpeed[N/s]	== Revolutions[N] / DeltaTime[s] 	= DeltaD[EncoderTicks] * UnitT_Freq[Hz] / PulsePerRevolution[EncoderTicks/1] / DeltaT[TimerTicks]
 *
 * DeltaAngle[DegreesNBits]			= DeltaD[EncoderTicks] * DegreesPerRevolution[DegreesNBits/1] / PulsesPerRevolution[EncoderTicks/1]
 * AngularSpeed[DegreesNBits/s] 	= DeltaD[EncoderTicks] * DegreesPerRevolution[DegreesNBits/1] * UnitT_Freq[Hz] / PulsesPerRevolution[EncoderTicks/1] / DeltaT[TimerTicks]
 * UnitAngle[DegreesNBits]			= DegreesPerRevolution[DegreesNBits/1]/PulsesPerRevolution[EncoderTicks/1]
 */

/*
	Overallocation of variable space to allow runtime polymorphism
 */
typedef struct
{
	/* HW Config */
	const HAL_Encoder_T * p_HAL_Encoder;	/*!< DeltaT and DeltaD timer counter */

	//	const HAL_TimerCounter_T * p_TimerCounter;	/*!< Delta timer counter */
	//	const HAL_Pin_T * p_PinPhaseA;
	//	const HAL_Pin_T * p_PinPhaseB;

    uint32_t TimerCounterMax; 				/*!< TimerCounter overflow loop */
	uint32_t PollingFreq;					/*!< Polling D reference Freq, CaptureDeltaT mode + interpolation calculations */ 		//	uint32_t UnitInterpolateD;	 /*!< [UnitD * UnitT_Freq / interpolationFreq] */
	uint32_t EncoderResolution;				/*!< Max for Capture looping AngleD, CaptureT mode need 2nd TimerCounterMax */

    /* SW Config, unit conversion. todo move to linear math */
	uint32_t UnitT_Freq;					/*!< T unit(seconds) conversion factor. TimerCounter freq, using Capture DeltaT (DeltaD is 1). Polling DeltaD freq, using Capture DeltaD (DeltaT is 1). */
	uint32_t UnitD;							/*!< Linear D unit conversion factor. Units per TimerCounter tick, using Capture DeltaD (DeltaT is 1). Units per DeltaT capture, using Capture DeltaT (DeltaD is 1).*/
	uint32_t UnitSpeed;						/*!< [UnitD * UnitT_Freq] => Speed = DeltaD * UnitSpeed / DeltaT */

	uint32_t UnitAngularD_Factor; 			/*!< [0xFFFFFFFFU/unitAngle_SensorResolution + 1] => DeltaAngle = DeltaD * UnitAngle_Factor >> UnitAngle_DivisorShift */
	uint32_t UnitAngularD_DivisorShift;		/*!< [32 - unitAngle_DataBits] */
	uint32_t UnitAngularSpeed; 				/*!< [(1 << unitAngle_DataBits) * UnitT_Freq / unitAngle_SensorResolution] => AngularSpeed = DeltaD * UnitAngularSpeed / DeltaT
												e.g. UnitAngularSpeed == 160,000 { unitAngle_DataBits = 16, UnitT_Freq = 20000, unitAngle_SensorResolution = 8912 } */

	uint32_t UnitInterpolateAngle;



	/* Runtime Vars */
	volatile uint32_t TimerCounterSaved;	/*!< First time/count sample used to calculate Delta */
    volatile uint32_t DeltaT;				/*!< Captured TimerCounter time interval counts between 2 distance events. Units in raw timer ticks */
	volatile uint32_t DeltaD;				/*!< Captured TimerCounter distance interval counts between 2 points in time. Units in raw timer ticks */
	volatile uint32_t AngularD;				/*!< Looping TotalD at EncoderResolution. TotalAngularD */

	volatile uint32_t TotalT;				/*!< TimerCounter ticks, persistent through Delta captures*/
	volatile int32_t TotalD;
//	volatile uint32_t UserT;				/*!< TimerCounter ticks, persistent until user reset*/
//	volatile uint32_t UserD;

	volatile uint32_t SpeedSaved; 			/*!< Most recently calculated speed, used for acceleration calc */
	volatile uint32_t DeltaSpeed;			/*!< Save for acceleration calc */
	volatile bool PulseReferenceSaved;		/*!< PhaseA state. For polling capture of DeltaT */

	/*
	 * Motor Encoder
	 */
	uint8_t PolePairs; /*! Convert between electrical speed and mechanical speed */
	//    uint32_t UnitEletricalAngle_Factor;
	//    uint32_t UnitEletricalAngle_DivisorShift;
	//    uint32_t UnitEletricalSpeed;

	/*
	 * SW Quadrature Encoder - Signed Speed
	 */
	bool IsALeadBPositive; //calibrate phase b for positive negative speed
	//	bool DirectionSaved;
	//	volatile int32_t DeltaCountSigned;

	//for long deltas
	volatile const uint32_t * p_ExtendedDeltaTimer; /*!< Poll DeltaT stopped */
	uint32_t ExtendedDeltaTimerFreq;
	uint32_t ExtendedDeltaTimerConversion; //short timer overflow time in long timer counts
	uint32_t ExtendedDeltaTimerEffectiveStopTime;
	volatile uint32_t ExtendedDeltaTimerSaved;
}
Encoder_T;

/******************************************************************************/
/*!
	@addtogroup	Delta
	@brief 	 Captured Delta calculations and conversions.

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
static inline uint32_t MicrosHelper(uint32_t timerTicks, uint32_t timerFreq)
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
	Capture DeltaT Mode Functions -
	Only for variable DeltaT (DeltaD is fixed, == 1).
	Meaningless for variable DeltaD (DeltaT is fixed, == 1).
 */
/******************************************************************************/
/*!
	@brief DeltaT period. unit in raw timer ticks.
 */
static inline uint32_t Encoder_GetDeltaT(Encoder_T * p_encoder)
{
	return p_encoder->DeltaT;
}

/*!
	@brief DeltaT freq.	unit in Hz
 */
static inline uint32_t Encoder_GetDeltaT_Freq(Encoder_T * p_encoder)
{
	uint32_t freqHz;

	if (p_encoder->DeltaT == 0)	{ freqHz = 0; }
	else 						{ freqHz = p_encoder->UnitT_Freq / p_encoder->DeltaT; }

	return freqHz;
}

/*!
	@brief DeltaT period. unit in milliseconds
 */
static inline uint32_t Encoder_GetDeltaT_Millis(Encoder_T * p_encoder)
{
	return p_encoder->DeltaT * 1000 / p_encoder->UnitT_Freq;
}

/*!
	@brief DeltaT period. unit in microseconds
 */
static inline uint32_t Encoder_GetDeltaT_Micros(Encoder_T * p_encoder)
{
	return MicrosHelper(p_encoder->DeltaT,  p_encoder->UnitT_Freq);
}

/*!
	@brief DeltaT freq.	unit in cycles per minute
 */
static inline uint32_t Encoder_GetDeltaT_FreqCPM(Encoder_T * p_encoder)
{
	uint32_t cpm;

	if (p_encoder->DeltaT == 0)	{ cpm = 0; }
	else						{ cpm = p_encoder->UnitT_Freq * 60 / p_encoder->DeltaT; }

	return cpm;
}

/******************************************************************************/
/*!
	Capture DeltaD Mode Functions -
	Only for variable DeltaD (DeltaT is fixed, == 1).
	Meaningless for DeltaT, variable DeltaD (DeltaT is fixed, == 1).
 */
/******************************************************************************/
/*!
	unit in ticks unit in raw timer ticks.
 */
static inline uint32_t Encoder_GetDeltaD(Encoder_T * p_encoder)
{
	return p_encoder->DeltaD;
}

/*!
	@brief DeltaT freq.	unit in cycles per minute
 */
static inline uint32_t Encoder_GetDeltaD_Units(Encoder_T * p_encoder)
{
	return p_encoder->DeltaD * p_encoder->UnitD;
}

/******************************************************************************/
/*!
	TotalD TotalT Functions - Both modes
 */
/******************************************************************************/
/*!
	Same as integral D
 */
static inline uint32_t Encoder_GetTotalD_Units(Encoder_T * p_encoder)
{
	return p_encoder->TotalD * p_encoder->UnitD;
}

/*!

 */
static inline uint32_t Encoder_GetTotalT_Freq(Encoder_T * p_encoder)
{
	return p_encoder->UnitT_Freq / p_encoder->TotalT;
}

/*!

 */
static inline uint32_t Encoder_GetTotalT_Millis(Encoder_T * p_encoder)
{
	return p_encoder->TotalT * 1000 / p_encoder->UnitT_Freq;
}

/*!

 */
static inline uint32_t Encoder_GetTotalT_Micros(Encoder_T * p_encoder)
{
	return MicrosHelper(p_encoder->TotalT,  p_encoder->UnitT_Freq);
}

/******************************************************************************/
/*!
	Argument Conversion
 */
/******************************************************************************/
static inline uint32_t Encoder_ConvertDeltaTToFreq(Encoder_T * p_encoder, uint32_t deltaT_Ticks)
{
	uint32_t deltaT_FreqHz;

	if (deltaT_Ticks == 0)	{ deltaT_FreqHz = 0; }
	else 					{ deltaT_FreqHz = p_encoder->UnitT_Freq / deltaT_Ticks; }

	return deltaT_FreqHz;
}

static inline uint32_t Encoder_ConvertFreqToDeltaT(Encoder_T * p_encoder, uint32_t deltaT_FreqHz)
{
	uint32_t deltaT_Ticks;

	if (deltaT_FreqHz == 0)	{ deltaT_Ticks = 0; }
	else 					{ deltaT_Ticks = p_encoder->UnitT_Freq / deltaT_FreqHz; }

	return deltaT_Ticks;
}

//static inline uint32_t Encoder_ConvertDeltaTToTime_(Encoder_T * p_encoder)
//{
//
//}

static inline uint32_t Encoder_ConvertDeltaDToUnits(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitD;
}

static inline uint32_t Encoder_ConvertUnitsToDeltaD(Encoder_T * p_encoder, uint32_t deltaD_Units)
{
	return deltaD_Units / p_encoder->UnitD;
}

/******************************************************************************/
/*!
	Inline Setters
 */
/*! @{ */
/******************************************************************************/
static inline void Encoder_ZeroDeltas(Encoder_T * p_encoder)
{
	p_encoder->DeltaD = 1;
	p_encoder->DeltaT = 1;
}

static inline void Encoder_ZeroTotals(Encoder_T * p_encoder)
{
	p_encoder->TotalD = 0;
	p_encoder->TotalT = 0;
}

static inline void Encoder_ZeroTimerCounter(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->p_HAL_Encoder, 0U);
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);
}

static inline void Encoder_Zero(Encoder_T * p_encoder)
{
	p_encoder->DeltaD = 1U;
	p_encoder->DeltaT = 1U;
	p_encoder->TotalD = 0U;
	p_encoder->TotalT = 0U;
	p_encoder->AngularD = 0U;
//	p_encoder->UserD 	= 0;
//	p_encoder->UserT 	= 0;

	p_encoder->SpeedSaved = 0U;
	p_encoder->DeltaSpeed = 0U;

	HAL_Encoder_WriteTimerCounter(p_encoder->p_HAL_Encoder, 0U);; /* reset for angularD */
	p_encoder->TimerCounterSaved = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);

	p_encoder->ExtendedDeltaTimerSaved = *p_encoder->p_ExtendedDeltaTimer;
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
 *	DeltaDUnits(PulsePerRevolution) = 1/8 (8) 	=> AngleRes 0.125 (of 1), 8,169.5 Angle16
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
 *	DeltaDUnits(PulsePerRevolution) = 1/2048(2048) 	=> AngleRes 0.000488 (of 1), 32 Angle16
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
 *	UnitD(0x10000/PulsePerRevolution) = 8 		=> AngleRes 0.000122 (of 1), 8 Angle16
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
 *	UnitD(0x10000/PulsePerRevolution) = 8 		=> AngleRes 0.000122 (of 1), 8 Angle16
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
	uint32_t spd;

//	if (p_encoder->DeltaT == 0)
//	{
//		spd = 0;
//		spd = p_encoder->DeltaD * p_encoder->UnitSpeed;
//	}
//	else
//	{
		/*
		 For case of Capture DeltaD, DeltaT == 1
		 possible 32bit overflow
		 Max deltaD will be UINT32_MAX / (unitDeltaD * unitDeltaT_Freq)
		 deltaD ~14,000, for 300,000 (unitDeltaD * unitDeltaT_Freq)
		 */
		spd = p_encoder->DeltaD * p_encoder->UnitSpeed / p_encoder->DeltaT; /* p_encoder->DeltaD * [UnitD * UnitT_Freq] / p_encoder->DeltaT; */
//	}

	return spd;
}
	//	/*!
	//		Save steps
	//	 */
	//	static inline uint32_t Encoder_GetEncoder_FixedDeltaD(Encoder_T * p_encoder) //DeltaD is fixed, i.e 1
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

	//static inline uint32_t Encoder_GetEncoder_FixedDeltaT(Encoder_T * p_encoder)
	//{
	//	uint32_t spd;
	//
	//	/*
	//	 * For case of CaptureDeltaD(), DeltaT == 1: constraint on unitDeltaD, and deltaD
	//	 * Max deltaD will be UINT32_MAX / (unitDeltaD * unitDeltaT_Freq)
	//	 * deltaD ~14,000, for 300,000 (unitDeltaD * unitDeltaT_Freq)
	//	 */
	//	spd = p_encoder->DeltaD * p_encoder->UnitSpeed;
	//
	//	return spd;
	//}

static inline uint32_t Encoder_GetSpeed_UnitsPerMinute(Encoder_T * p_encoder)
{
	uint32_t spd;

//	if (p_encoder->DeltaT == 0)
//	{
//		spd = 0;
//	}
//	else
//	{
		spd = p_encoder->DeltaD * p_encoder->UnitSpeed * 60 / p_encoder->DeltaT;
//	}

	return spd;
}

/*!
	Also save speed for acceleration calculation
 */
static inline uint32_t Encoder_CalcSpeed(Encoder_T * p_encoder)
{
	uint32_t newSpeed;

//	if (p_encoder->DeltaT == 0)
//	{
//		p_encoder->SpeedSaved = 0;
//	}
//	else
//	{
		newSpeed 				= p_encoder->DeltaD * p_encoder->UnitSpeed / p_encoder->DeltaT;
		p_encoder->DeltaSpeed 	= newSpeed - p_encoder->SpeedSaved;
		p_encoder->SpeedSaved 	= newSpeed;
//		speedFactor = p_encoder->DeltaD * p_encoder->UnitSpeed // for perminute conversions
//	}

	return newSpeed;
}

static inline uint32_t Encoder_GetAcceleration(Encoder_T * p_encoder)
{
	return p_encoder->DeltaSpeed * p_encoder->UnitT_Freq / p_encoder->DeltaT;
}

/*!
	returns DeltaT -
	Fixed DeltaD: DeltaT timer ticks for given speed
	Fixed DeltaT: Number of fixed DeltaT samples, before a deltaD increment(should be less than one, if using this mode)

	@param speed
	@param unitsPerSecond
	@return
 */
static inline uint32_t Encoder_ConvertSpeedToDeltaT(Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
	uint32_t deltaT_Ticks;

	if (speed_UnitsPerSecond == 0)
	{
		deltaT_Ticks = p_encoder->UnitSpeed;
	}
	else
	{
		deltaT_Ticks = p_encoder->UnitSpeed / speed_UnitsPerSecond;
	}

	return deltaT_Ticks;
}

static inline uint32_t Encoder_ConvertDeltaTToSpeed(Encoder_T * p_encoder, uint32_t deltaT_ticks)
{
	return Encoder_ConvertSpeedToDeltaT(p_encoder, deltaT_ticks); /* Same division */
}

static inline uint32_t Encoder_ConvertSpeedToDeltaT_UnitsPerMinute(Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
	uint32_t deltaT_Ticks;

	if (speed_UnitsPerMinute == 0)
	{
		deltaT_Ticks = 0;
	}
	else
	{
		deltaT_Ticks = p_encoder->UnitSpeed * 60 / speed_UnitsPerMinute;
	}

	return deltaT_Ticks;
}

static inline uint32_t Encoder_ConvertDeltaTToSpeed_UnitsPerMinute(Encoder_T * p_encoder, uint32_t deltaT_Ticks)
{
	return Encoder_ConvertSpeedToDeltaT_UnitsPerMinute(p_encoder, deltaT_Ticks); /* Same division */
}

/*!
	returns DeltaD -
	Fixed DeltaD: number of deltaD before a DeltaT increment (should be less than one, if using this mode)
	Fixed DeltaT: DeltaD count on fixed time sample.
*/
static inline uint32_t Encoder_ConvertSpeedToDeltaD(Encoder_T * p_encoder, uint32_t speed_UnitsPerSecond)
{
	return speed_UnitsPerSecond / p_encoder->UnitSpeed;
}

static inline uint32_t Encoder_ConvertDeltaDToSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitSpeed;
}

static inline uint32_t Encoder_ConvertSpeedToDeltaD_UnitsPerMinute(Encoder_T * p_encoder, uint32_t speed_UnitsPerMinute)
{
	return speed_UnitsPerMinute * 60 / p_encoder->UnitSpeed;
}

static inline uint32_t Encoder_ConvertDeltaDToEncoder_UnitsPerMinute(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitSpeed * 60;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*!
	@brief Linear alias functions
 */
/*! @{ */
/******************************************************************************/
static inline uint32_t Encoder_GetLinearDeltaDistance(Encoder_T * p_encoder)
{
	return Encoder_GetDeltaD_Units(p_encoder);
}

static inline uint32_t Encoder_GetLinearTotalDistance(Encoder_T * p_encoder)
{
	return Encoder_GetTotalD_Units(p_encoder);
}

static inline uint32_t Encoder_GetLinearSpeed(Encoder_T * p_encoder)
{
	return Encoder_GetSpeed(p_encoder);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*!
	@brief Angle Functions
 */
/*! @{ */
/******************************************************************************/

/******************************************************************************/
/*!
	Capture DeltaD Mode Functions -
	Only for variable DeltaD (DeltaT is fixed, == 1).
	Meaningless for DeltaT, variable DeltaD (DeltaT is fixed, == 1).
 */
/******************************************************************************/
static inline uint32_t Encoder_GetDeltaAngle(Encoder_T * p_encoder)
{
	return  p_encoder->DeltaD * p_encoder->UnitAngularD_Factor >> p_encoder->UnitAngularD_DivisorShift;
}

/******************************************************************************/
/*!
	TotalD - Both modes
 */
/******************************************************************************/
static inline uint32_t Encoder_GetTotalAngle(Encoder_T * p_encoder)
{
	/* overflow if TotalD > unitAngle_SensorResolution, should maintain correct absolute position */
	return  p_encoder->AngularD * p_encoder->UnitAngularD_Factor >> p_encoder->UnitAngularD_DivisorShift;
//	return ((p_encoder->AngleD << unitAngle_DataBits) / unitAngle_SensorResolution);
}

static inline uint32_t Encoder_GetTotalRevolutions(Encoder_T * p_encoder)
{
	return p_encoder->TotalD * (p_encoder->UnitAngularD_Factor >> p_encoder->UnitAngularD_DivisorShift) >> (32 - p_encoder->UnitAngularD_DivisorShift);
}

/******************************************************************************/
/*!
	Angular Speed Functions
 */
/******************************************************************************/
static inline uint32_t Encoder_GetAngularSpeed(Encoder_T * p_encoder)
{

//	if(p_encoder->DeltaT == 0)
//	{
//
//	}

//	if (p_encoder->UnitT_Freq > 65535U)
	if (p_encoder->DeltaD > UINT32_MAX / p_encoder->UnitAngularSpeed)
	{
		/*
			When UnitAngularSpeed is too high. Large timer freq (> 16 bits), angle res, small countsPerRevolution
		*/
		//	UnitRotationalSpeed = (p_encoder->UnitT_Freq) / (p_encoder->EncoderResolution)
		return (p_encoder->DeltaD * p_encoder->UnitT_Freq) / (p_encoder->EncoderResolution * p_encoder->DeltaT) << (32U - p_encoder->UnitAngularD_DivisorShift);
	}
	else
	{
		return  p_encoder->DeltaD * p_encoder->UnitAngularSpeed / p_encoder->DeltaT;
	}
}

/*!
	Revolution per Second
 */
static inline uint32_t Encoder_GetRotationalSpeed_RPS(Encoder_T * p_encoder)
{
	return Encoder_GetAngularSpeed(p_encoder) >> (32U - p_encoder->UnitAngularD_DivisorShift);
}

static inline uint32_t Encoder_GetRotationalSpeed_RPM(Encoder_T * p_encoder)
{
	return Encoder_GetAngularSpeed(p_encoder) * 60 >> (32U - p_encoder->UnitAngularD_DivisorShift);

}

/*!
	@return DeltaD is angle in raw timer counter ticks.
	CaptureDeltaT, Fixed DeltaD: number of deltaD before a DeltaT increment (should be less than one, if using this mode)
	CaptureDeltaD, Fixed DeltaT: DeltaD count on fixed time sample.
 */
static inline uint32_t Encoder_ConvertRotationalSpeedToDeltaD_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	return (rpm << (32U - p_encoder->UnitAngularD_DivisorShift)) / 60 / p_encoder->UnitAngularSpeed;
}

static inline uint32_t Encoder_ConvertDeltaDToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return (deltaD_Ticks * p_encoder->UnitAngularSpeed * 60) >> (32U - p_encoder->UnitAngularD_DivisorShift);
}

static inline uint32_t Encoder_ConvertAngularSpeedToDeltaD(Encoder_T * p_encoder, uint32_t angularSpeed_DegreesNBitsPerSecond)
{
	return angularSpeed_DegreesNBitsPerSecond * p_encoder->DeltaT / p_encoder->UnitAngularSpeed;
}

static inline uint32_t Encoder_ConvertDeltaDToAngularSpeed(Encoder_T * p_encoder, uint32_t deltaD_Ticks)
{
	return deltaD_Ticks * p_encoder->UnitAngularSpeed / p_encoder->DeltaT;
}

/*!
	@return DeltaT
	CaptureDeltaT, Fixed DeltaD: DeltaT timer ticks for given speed
	CaptureDeltaD, Fixed DeltaT: Number of fixed DeltaT samples, before a deltaD increment(should be less than one, if using this mode)
 */
static inline uint32_t Encoder_ConvertRotationalSpeedToDeltaT_RPM(Encoder_T * p_encoder, uint32_t rpm)
{
	return p_encoder->UnitAngularSpeed * 60 / rpm >> (32U - p_encoder->UnitAngularD_DivisorShift);
}

static inline uint32_t Encoder_ConvertDeltaTToRotationalSpeed_RPM(Encoder_T * p_encoder, uint32_t deltaT_ticks)
{
	return p_encoder->UnitAngularSpeed * 60 / deltaT_ticks >> (32U - p_encoder->UnitAngularD_DivisorShift);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/



/******************************************************************************/
/*!
	@brief 	Interpolation Estimated Functions
 */
/*! @{ */
/******************************************************************************/
/*!
	@brief CaptureDeltaD Mode: Estimate D using captured DeltaD sample. Assuming constant speed.
	@param domain unitless
 */
static inline uint32_t Encoder_InterpolateD_Slope(Encoder_T * p_encoder, uint32_t index, uint32_t domain)
{
	return index * Encoder_GetDeltaD_Units(p_encoder) / domain;
}

/*!
	@brief CaptureDeltaT Mode: Estimate D using captured DeltaT sample. Assuming constant speed.

	Delta Peroid > Control Peroid
	time domain - if domain is with respect to time =>
	@param index range [0:interpolationFreq/DeltaT_Freq]. interpolationFreq/DeltaT_Freq == [interpolationFreq/(UnitT_Freq/DeltaT)] == [interpolationFreq * DeltaT / UnitT_Freq]
 */
static inline uint32_t Encoder_InterpolateD(Encoder_T * p_encoder, uint32_t index)
{
//	return index * p_encoder->UnitInterpolateD / p_encoder->DeltaT; /* index * [UnitD * UnitT_Freq / interpolationFreq] / DeltaT */
	return index * Encoder_GetSpeed(p_encoder) / p_encoder->PollingFreq; 	//index * 1 * [UnitD * UnitT_Freq] / p_encoder->DeltaT / interpolationFreq;
}

//static inline uint32_t Encoder_InterpolateSpeed(Encoder_T * p_encoder, uint32_t index, uint32_t domain)
//{
//	return index * Encoder_GetSpeed(p_encoder) / domain;
//}

//static inline uint32_t Encoder_InterpolateSpeed_TimeDomain(Encoder_T * p_encoder, uint32_t index, uint32_t interpolationFreq)
//{
////	return index * Encoder_GetSpeed(p_encoder) * p_encoder->UnitInterpolatedD / p_encoder->DeltaT;
//
////	index * Encoder_GetAcceleration(p_encoder) / interpolationFreq;
//}
//
//static inline uint32_t Encoder_IntegrateSpeed(Encoder_T * p_encoder, uint32_t speed)
//{
////	 positionRaw += Encoder_GetSpeed(p_encoder);
//		positionUnits = Encoder_ConvertDeltaDToUnits(positionRaw)
//}

/******************************************************************************/
/*!
	@brief 	Angular Interpolation Estimated Functions
 */
/*! @{ */
/******************************************************************************/
/*!

 */
static inline uint32_t Encoder_InterpolateAngle_Slope(Encoder_T * p_encoder, uint32_t index, uint32_t domain)
{
	return index * Encoder_GetDeltaAngle(p_encoder) / domain;
}

/*!
	@brief 	  CaptureDeltaT mode
	polling freq index
 */
static inline uint32_t Encoder_InterpolateAngle(Encoder_T * p_encoder, uint32_t pollingIndex)
{
	return pollingIndex * p_encoder->UnitInterpolateAngle / p_encoder->DeltaT; 		//	AngleControlIndex * [AngleSize[0x10000] * (UnitDeltaT_Freq / PollingFreq)] / DeltaT;
//	return index * Encoder_GetAngularSpeed(p_encoder) / p_encoder->PollingFreq; 	//[DeltaD * (UnitT_Freq * 0x10000 *  PolePairs /  PulsePerRevolution) /  DeltaT] / PollingFreq;
}

static inline uint32_t Encoder_InterpolateAngle_IncIndex(Encoder_T * p_encoder, uint32_t * p_pollingIndex)
{
	uint32_t angle = Encoder_InterpolateAngle(p_encoder, *p_pollingIndex);
	*p_pollingIndex++;
	return angle;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
	@brief 	Extern Functions
 */
/*! @{ */
/******************************************************************************/
extern void Encoder_Init
(
	Encoder_T * p_encoder,
	HAL_Encoder_T * p_encoderTimerCounter,
	uint32_t timerCounterMax,
	uint32_t unitT_Freq,					/* UnitT_Freq */
	uint32_t pollingFreq,					/* PollingFreq */
	uint32_t encoderDistancePerCount,		/* UnitLinearD */
	uint32_t encoderCountsPerRevolution,	/* UnitAngularD_Factor = [0xFFFFFFFFU/encoderCountsPerRevolution + 1] */
	uint8_t angleDataBits					/* UnitAngularD_DivisorShift = [32 - unitAngle_DataBits] */
);

extern void Encoder_Zero(Encoder_T * p_encoder);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
