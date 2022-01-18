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
    @file 	BEMF.h
    @author FireSoucery
    @brief 	Zcd

    @version V0
*/
/******************************************************************************/
#ifndef BEMF_H
#define BEMF_H

#include "Config.h"

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	BEMF_SAMPLE_MODE_PWM_ON,
	BEMF_SAMPLE_MODE_PWM_OFF,
	BEMF_SAMPLE_MODE_PWM_ON_OFF,
	BEMF_SAMPLE_MODE_PWM_BIPOLAR,
	BEMF_SAMPLE_MODE_INTEGRAL,
}
BEMF_SampleMode_T; //pwm sample mode

typedef enum
{
	BEMF_CYCLE_MODE_PASSIVE, 		// no blank time
	BEMF_CYCLE_MODE_STARTUP, 		//half zcd zero
	BEMF_CYCLE_MODE_COMMUTATION,  // phase voltage active
//	BEMF_MODE_DRIVEN, //module hw timer driven
}
BEMF_CycleMode_T; //phase active/inactive sample mode

//typedef enum
//{
//	BEMF_ZCD_STATUS_FALSE,
//	BEMF_ZCD_STATUS_TRUE,
//	BEMF_ZCD_STATUS_MISSED,
//}
//BEMF_ZcdStatus_T;

//typedef enum
//{
//	BEMF_PHASE_A,
//	BEMF_PHASE_B,
//	BEMF_PHASE_C,
//}
//BEMF_PhaseId_T;

//typedef struct
//{
//	BEMF_SampleMode_T SampleMode;
//	uint16_t ZeroCrossingThreshold_ADCU;		/// sensitivity offset
//}
//BEMF_Params_T;

typedef const struct
{
//	const BEMF_Params_T * const P_PARAMS;

	volatile const uint32_t * const P_TIMER; //sw timer, 59hr overflow using 20khz timer
	//	module may need to handle unit conversion if phase bemf and vbus use different resistor values
}
BEMF_Config_T;

typedef struct
{
	const BEMF_Config_T CONFIG;

	BEMF_SampleMode_T SampleMode;
	uint16_t ZeroCrossingThreshold_ADCU;		/// sensitivity offset

	//uint16_t BlankTimeScalar;
	//uint16_t UsePWMOnThreshold;			/// for mixed sample mode, use PWM On ZCD when PWM On cycle is above this time.
	//uin16t_t DiodeDrop; 					//	if no complementary pwm vd/2

	//	BEMF_Direction_T Direction;
	BEMF_CycleMode_T CycleMode;					/// substate
	uint32_t TimeReferenceZero;			/// reference start time, commutation start in active mode
	uint32_t TimeBlankPeriod;			/// Blank time after commutation. aka "demagnetization time".

//	const AnalogN_Conversion_T * p_ConversionPhase; /* Set once per commutation */

	//phase sample - user set from outside module, track prev for zcd
	uint16_t VPos_ADCU;
	uint16_t VPhase_ADCU;				/// Phase-to-ground voltage. Phase voltage PWM On: VPhase == (3/2)VBEMF + VPos/2
	uint16_t VPhasePrev_ADCU;			///
	uint32_t TimeVPhase;				/// sample time stamp, from TimerReferenceZero
	uint32_t TimeVPhasePrev;			/// or use constant periodic sample time

	uint16_t VPhasePwmOff_ADCU;
	uint16_t VPhasePrevPwmOff_ADCU;

//	uint16_t VPhaseOnOffDiff_ADCU;

	//if save ZCD processed values
//	volatile int16_t Emf_SignedADCU;				/// VPhase-to-neutral + 1/2 VBemf, ofset for ZCD , PWM On: (3/2)VBemf
//	volatile int16_t EmfPrev_SignedADCU;			///

	//ZCD runtime parameters
	bool IsZeroCrossingDetectionComplete;
	bool IsBemfRising;

	//ZCD results
	uint32_t TimeZeroCrossingDetect; 		/// Time between commutation and zero crossing, 30 degrees.
	uint32_t TimeZeroCrossingPeriod; 		/// Time between 2 zero crossings, 60 degrees.
	uint32_t ZeroCrossingCounter; 		///  consecutive zc
	uint32_t ZeroCrossingMissCounter; 		///  consecutive zc
	uint16_t VZeroCrossing_ADCU;

	//commutation timer
	volatile uint32_t TimePhasePeriod;
	uint16_t PhaseAdvanceTime;			/// Commutation delay reduction. phase advance for high freq when voltage leads current. proportional to inductance.

//reliable
//	  bool IsReliable;
//	  uint32_t IsReliableCounter;
}
BEMF_T;

#define BEMF_CONFIG(p_Timer)	\
{														\
	.CONFIG = 											\
	{													\
		.P_TIMER = p_Timer,								\
	}													\
}


extern volatile uint16_t BemfDebug[300];
extern volatile uint16_t BemfDebugIndex;

/*
	Set from outside, once per commutation
 */
static inline void BEMF_MapPhaseA(BEMF_T * p_bemf){	}//p_bemf->p_ConversionPhase = &p_bemf->CONFIG.CONVERSION_PHASE_A;}
static inline void BEMF_MapPhaseB(BEMF_T * p_bemf){	}//p_bemf->p_ConversionPhase = &p_bemf->CONFIG.CONVERSION_PHASE_B;}
static inline void BEMF_MapPhaseC(BEMF_T * p_bemf){	}//p_bemf->p_ConversionPhase = &p_bemf->CONFIG.CONVERSION_PHASE_C;}

static inline void BEMF_MapRising(BEMF_T * p_bemf){p_bemf->IsBemfRising = true;}
static inline void BEMF_MapFalling(BEMF_T * p_bemf){p_bemf->IsBemfRising = false;}

static inline void BEMF_MapCwPhaseAC(BEMF_T * p_bemf){BEMF_MapPhaseB(p_bemf); p_bemf->IsBemfRising = false;}
static inline void BEMF_MapCwPhaseBC(BEMF_T * p_bemf){BEMF_MapPhaseA(p_bemf); p_bemf->IsBemfRising = true;}
static inline void BEMF_MapCwPhaseBA(BEMF_T * p_bemf){BEMF_MapPhaseC(p_bemf); p_bemf->IsBemfRising = false;}
static inline void BEMF_MapCwPhaseCA(BEMF_T * p_bemf){BEMF_MapPhaseB(p_bemf); p_bemf->IsBemfRising = true;}
static inline void BEMF_MapCwPhaseCB(BEMF_T * p_bemf){BEMF_MapPhaseA(p_bemf); p_bemf->IsBemfRising = false;}
static inline void BEMF_MapCwPhaseAB(BEMF_T * p_bemf){BEMF_MapPhaseC(p_bemf); p_bemf->IsBemfRising = true;}

static inline void BEMF_MapCcwPhaseAC(BEMF_T * p_bemf){BEMF_MapPhaseB(p_bemf); p_bemf->IsBemfRising = true;}
static inline void BEMF_MapCcwPhaseBC(BEMF_T * p_bemf){BEMF_MapPhaseA(p_bemf); p_bemf->IsBemfRising = false;}
static inline void BEMF_MapCcwPhaseBA(BEMF_T * p_bemf){BEMF_MapPhaseC(p_bemf); p_bemf->IsBemfRising = true;}
static inline void BEMF_MapCcwPhaseCA(BEMF_T * p_bemf){BEMF_MapPhaseB(p_bemf); p_bemf->IsBemfRising = false;}
static inline void BEMF_MapCcwPhaseCB(BEMF_T * p_bemf){BEMF_MapPhaseA(p_bemf); p_bemf->IsBemfRising = true;}
static inline void BEMF_MapCcwPhaseAB(BEMF_T * p_bemf){BEMF_MapPhaseC(p_bemf); p_bemf->IsBemfRising = false;}

/* Matched to motor Id */
//typedef enum
//{
//	BEMF_PHASE_AC = 1U, /* CCW Map B Rising */
//	BEMF_PHASE_BC = 2U,
//	BEMF_PHASE_BA = 3U,
//	BEMF_PHASE_CA = 4U,
//	BEMF_PHASE_CB = 5U,
//	BEMF_PHASE_AB = 6U,
//
//	BEMF_CCW_B_RISING = BEMF_PHASE_AC,
//} BEMF_Phase_T;

//static inline void Bemf_MapVPhase(BEMF_T * p_bemf, BEMF_Phase_T phase)
//{

//}


/*
 * Call on Commutation
 * Capture Reference
 */
static inline void BEMF_CapturePhaseReference(BEMF_T * p_bemf)
{
	if (p_bemf->IsZeroCrossingDetectionComplete == false) //commutation occurred before a zcd
	{
//		if (p_bemf->ZeroCrossingMissCounter > 5U)
//		{
//			p_bemf->ZeroCrossingCounter = 0U; //also set zero crossing miss
//			p_bemf->ZeroCrossingMissCounter = 0U;
//		}
//		else
//		{
//			p_bemf->ZeroCrossingMissCounter++;
//		}
		p_bemf->TimeZeroCrossingDetect = 0U;
		p_bemf->TimeZeroCrossingPeriod = 0U;
	}
	else
	{
		p_bemf->IsZeroCrossingDetectionComplete = false;
	}

	p_bemf->TimePhasePeriod 	= (p_bemf->TimePhasePeriod + (*p_bemf->CONFIG.P_TIMER - p_bemf->TimeReferenceZero)) / 2U; //back up time, will be overwritten upon zcd
	p_bemf->TimeReferenceZero 	= *p_bemf->CONFIG.P_TIMER;

	p_bemf->TimeBlankPeriod = p_bemf->TimePhasePeriod / 8U; //default case 25% of commutation time, 50% of zct. set 0 for passive mode?

	p_bemf->TimeVPhasePrev 		= 0U;
	p_bemf->TimeVPhase	 		= 0U;
//	p_bemf->VPhase_ADCU 		= 0U;
//	p_bemf->VPhasePrev_ADCU  	= 0U;

	BemfDebugIndex = 0;

	// module determine next phase from inside
//	BEMF_MapSector();

	//hw timer version
	//HAL_BEMF_StartTimerInterrupt(p_bemf->p_HAL_Bemf, p_bemf->TimeZeroCrossingPeriod);
}

/*
 * On every Pwm Period
 */
static inline bool BEMF_CheckBlankTime(BEMF_T * p_bemf)
{
	return ((*p_bemf->CONFIG.P_TIMER - p_bemf->TimeReferenceZero > p_bemf->TimeBlankPeriod) || (p_bemf->CycleMode == BEMF_CYCLE_MODE_PASSIVE));
}

//static inline void BEMF_PollStartAnalog(BEMF_T * p_bemf)
//{
//	if(BEMF_CheckBlankTime(p_bemf) == true)
//	{
//		AnalogN_EnqueueFrontConversion(p_bemf->CONFIG.P_ANALOG_N, p_bemf->p_ConversionPhase);
//	}
//}

#include "Utility/Debug/Debug.h"
/*
	ZCD Bemf Capture
	Captures 2 points
	Calc signed valued for ZCD

	p_VPhaseObserve_ADCU pre mapped from outside, can be called directly if ZCD is not needed

	BEMF_SAMPLE_MODE_PWM_ON -> need sufficient duty cycle to determine, 3/2 * bemf + vPos/2
	BEMF_SAMPLE_MODE_PWM_OFF -> Must subtract noise or use threshold. no 3/2 factor in this case
 */
static inline void Bemf_CaptureVPhase(BEMF_T * p_bemf, uint32_t vPhaseObserve_ADCU)
{
	//TimeEmf Delta should be 50us when using 20khz pwm
	p_bemf->TimeVPhasePrev 	= p_bemf->TimeVPhase;
	p_bemf->TimeVPhase 		= *p_bemf->CONFIG.P_TIMER - p_bemf->TimeReferenceZero;

	p_bemf->VPhasePrev_ADCU 	= p_bemf->VPhase_ADCU;
	p_bemf->VPhase_ADCU 		= vPhaseObserve_ADCU;
//	p_bemf->VPhase_ADCU 		= (vPhaseObserve_ADCU + p_bemf->VPhase_ADCU) / 2U;

//	if (p_bemf->IsBemfRising)
//	{
//		if(BemfDebugIndex < 300) //try only capture on bemf risisng
//		{
//			BemfDebug[BemfDebugIndex] 	=  p_bemf->VPhase_ADCU;
//			BemfDebugIndex++;
//		}
//	}

}

static inline void Bemf_CaptureVPhasePwmOff(BEMF_T * p_bemf, uint32_t vPhaseObserve_ADCU)
{
	p_bemf->VPhasePrevPwmOff_ADCU 	= p_bemf->VPhasePwmOff_ADCU;
	p_bemf->VPhasePwmOff_ADCU 		= vPhaseObserve_ADCU;
//	p_bemf->VPhase_ADCU 		= (vPhaseObserve_ADCU + p_bemf->VPhase_ADCU) / 2U;
}

/*
 * For case ADC start does not check blank time
 */
static inline void Bemf_PollCaptureVPhase(BEMF_T * p_bemf, uint32_t vPhaseObserve_ADCU)
{
	if(BEMF_CheckBlankTime(p_bemf) == true)
	{
		Bemf_CaptureVPhase(p_bemf, vPhaseObserve_ADCU);
	}
}

static inline void Bemf_CaptureVPos(BEMF_T * p_bemf, uint32_t vPos_ADCU)
{
 	p_bemf->VPos_ADCU 	= vPos_ADCU;
}


/*
	Set from outside:  IsBemfRising, VBemf TimeVBemf
	software compare, todo hw comparator

	Proc last capured sample
 */
static inline bool ProcZeroCrossingDetection(BEMF_T * p_bemf)
{
	bool isZcd = false;

	int16_t vEmf;
	int16_t vEmfPrev;
	uint32_t timeZeroCrossing; /* new zc time */
	int32_t vZero;
//	bool isBemfRising;

	if (p_bemf->CycleMode != BEMF_CYCLE_MODE_PASSIVE)
	{
		switch(p_bemf->SampleMode)
		{
			case BEMF_SAMPLE_MODE_PWM_BIPOLAR :
				vZero = ((int32_t)p_bemf->VPos_ADCU / 2U) + (int32_t)p_bemf->ZeroCrossingThreshold_ADCU;
				break;
//		case BEMF_SAMPLE_MODE_PWM_MIXED: 			break;
			case BEMF_SAMPLE_MODE_PWM_ON :
				vZero = ((int32_t)p_bemf->VPos_ADCU / 2U) + (int32_t)p_bemf->ZeroCrossingThreshold_ADCU;
				break;
			case BEMF_SAMPLE_MODE_PWM_OFF :
				vZero = (int32_t)p_bemf->ZeroCrossingThreshold_ADCU;
				break;
			default :
				vZero = (int32_t)p_bemf->ZeroCrossingThreshold_ADCU;
				break;
		}

		if (p_bemf->CycleMode  == BEMF_CYCLE_MODE_STARTUP)
		{
//			vZero = vZero/2U;
		}
	}
	else
	{
		vZero = (int32_t)p_bemf->ZeroCrossingThreshold_ADCU;
	}
	
	//(p_bemf->Direction == CCW)
	if(p_bemf->IsBemfRising == false) // invert falling bemf to use same rising zcd routine
	{
		vEmf 		= -(int16_t)p_bemf->VPhase_ADCU;
		vEmfPrev 	= -(int16_t)p_bemf->VPhasePrev_ADCU;
	}
	else
	{
		vEmf 		= p_bemf->VPhase_ADCU;
		vEmfPrev 	= p_bemf->VPhasePrev_ADCU;
	}

	if((vEmf - vZero) >= 0) // is zero crossing, calc time of zcd
	{
		// find midpoint to account for error of (TimeBemf - TimeZeroCrossing) past zc time
		if ((vEmf - vEmfPrev) > (vEmf - vZero)) // zcd falls within 2 samples
		{
			timeZeroCrossing = p_bemf->TimeVPhase - (((p_bemf->TimeVPhase - p_bemf->TimeVPhasePrev) * (vEmf - vZero) / (vEmf - vEmfPrev))); //estimate using slope
//			p_bemf->ZeroCrossingCounter++;
		}
		else //missed zero crossing, both samples past vZero
		{
			timeZeroCrossing = p_bemf->TimeVPhasePrev - ((p_bemf->TimeVPhase - p_bemf->TimeVPhasePrev) / 2U);
//			p_bemf->ZeroCrossingCounter = 0U;
			//or use miss counter
		}

//		p_bemf->TimeZeroCrossingDetect = (timeZeroCrossing + p_bemf->TimeZeroCrossingDetect) / 2U;
//		p_bemf->TimeZeroCrossingPeriod = timeZeroCrossing + p_bemf->TimeZeroCrossing;
		p_bemf->TimeZeroCrossingDetect = timeZeroCrossing;
		// average zc with last zc
		p_bemf->TimeZeroCrossingPeriod = (p_bemf->TimeZeroCrossingPeriod + (p_bemf->TimeZeroCrossingDetect + timeZeroCrossing)) / 2U;

		p_bemf->VZeroCrossing_ADCU = vEmf;

		isZcd = true;
	}
	

	return isZcd;
}


//poll last sample, must skip prev sample
static inline bool BEMF_ProcZeroCrossingDetection(BEMF_T * p_bemf)
{
	bool isZcd = false;
	
	if((p_bemf->IsZeroCrossingDetectionComplete == false) && (p_bemf->TimeVPhasePrev > 0U)) //at least 2 samples
	{
		if (ProcZeroCrossingDetection(p_bemf) == true)
		{
			p_bemf->IsZeroCrossingDetectionComplete = true;

			p_bemf->ZeroCrossingCounter++;

			//stop here to compare hall captured vs bemf sample difference
//			p_bemf->TimePhasePeriod = (p_bemf->TimePhasePeriod + p_bemf->TimeZeroCrossingPeriod) / 2U; //avergae with estimate

//			p_bemf->TimePhasePeriod = p_bemf->TimeZeroCrossingPeriod - p_bemf->PhaseAdvanceTime;

			//HAL_BEMF_StartTimerInterrupt(p_bemf->p_HAL_Bemf, p_bemf->TimePhasePeriod);
	
			isZcd = true;
		}
	}

	return isZcd;
}

//check estimated time from Zcd
static inline bool BEMF_CheckPhasePeriod(BEMF_T * p_bemf)
{
	return (*p_bemf->CONFIG.P_TIMER - p_bemf->TimeReferenceZero > p_bemf->TimePhasePeriod);
}

static inline uint32_t BEMF_GetZeroCrossingCounter(BEMF_T * p_bemf)
{
	return p_bemf->ZeroCrossingCounter;
}

static inline bool BEMF_GetIsZeroCrossingDetectionComplete(BEMF_T * p_bemf)
{
	return p_bemf->IsZeroCrossingDetectionComplete;
}


static inline bool BEMF_GetIsReliable(BEMF_T * p_bemf)
{
	return (p_bemf->ZeroCrossingCounter > 10U);
}

//static inline bool BEMF_ProcStartUpReliable(BEMF_T * p_bemf, uint32_t measuredTime)
//{
//	if (measuredTime >> 4U == p_bemf->TimePhasePeriod >> 4U) //within 15 ticks or 750us at 20khz
//	{
//
//	}
//	else
//	{
//		p_bemf->ZeroCrossingCounter--; //negate count
//	}
//
//	return p_bemf->ZeroCrossingCounter;
//}


//static inline BEMF_PhaseId_T BEMF_GetPhaseId(BEMF_T * p_bemf) {return p_bemf->PhaseObserveId;}
//static inline uint16_t BEMF_GetTimeZeroCrossingPeriod(BEMF_T * p_bemf){return p_bemf->TimeZeroCrossingPeriod;}
//static inline uint16_t BEMF_GetTimeZeroCrossingDetect(BEMF_T * p_bemf){return p_bemf->TimeZeroCrossingDetect;}
//static inline uint16_t BEMF_GetTimePhasePeriod(BEMF_T * p_bemf){return p_bemf->TimePhasePeriod;}

/*
	adcu, convert to phase-to-neutral
 */
static inline int16_t ConvertVPhaseToVBemf(BEMF_T * p_bemf, uint32_t emf)
{
	uint16_t bemf;

	if (p_bemf->CycleMode == BEMF_CYCLE_MODE_COMMUTATION)
	{
		switch(p_bemf->SampleMode)
		{
//			case BEMF_SAMPLE_MODE_PWM_BIPOLAR: 	bemf = emf;				break; //todo
			case BEMF_SAMPLE_MODE_PWM_ON:  		bemf = emf * 2U / 3U - p_bemf->VPos_ADCU / 2U ; 	break;
			case BEMF_SAMPLE_MODE_PWM_OFF: 		bemf = emf;				break;
			default:	break;
		}
	}
	else if (p_bemf->CycleMode == BEMF_CYCLE_MODE_PASSIVE)
	{
		bemf = emf;
	}

	return bemf;
}

/*
 * Peak Bemf as it would be without active phase
 */
static inline int16_t BEMF_GetVBemfPeak_ADCU(BEMF_T * p_bemf)
{
//	Vbemf_peak ~= 2* Vbemf_ZC
	return ConvertVPhaseToVBemf(p_bemf, p_bemf->VZeroCrossing_ADCU*2U);
}



#endif


