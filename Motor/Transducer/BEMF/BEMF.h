/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	BEMF.h
    @author FireSoucery
    @brief 	Bemf monitor and reporting algorithms

	Implement sensorless 6-step, 2-phase active, algorithm.

    @version V0
*/
/**************************************************************************/
#ifndef BEMF_H
#define BEMF_H

//#include "HAL.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_BEMF_ADC_8
	typedef uint8_t bemf_t;
#elif defined(CONFIG_BEMF_ADC_16)
	typedef uint16_t bemf_t;
#endif

typedef enum
{
	BEMF_SAMPLE_MODE_PWM_ON,
	BEMF_SAMPLE_MODE_PWM_OFF,
	BEMF_SAMPLE_MODE_PWM_MIXED,
	BEMF_SAMPLE_MODE_PWM_BIPOLAR,
	BEMF_SAMPLE_MODE_INTEGRAL,
}
BEMF_SampleMode_T; //pwm sample mode

typedef enum
{
	BEMF_MODE_PASSIVE, 		// no blank time
	BEMF_MODE_COMMUTATION,  // phase voltage active
//	BEMF_MODE_DRIVEN, //module hw timer driven
}
BEMF_Mode_T; //phase active/inactive sample mode

//typedef enum
//{
//	BEMF_CYCLE_PERIOD_SIXTH,
//	BEMF_CYCLE_PERIOD_FULL, 	//full e rotation
//}
//BEMF_CyclePeriod_T;

//typedef enum
//{
//	BEMF_ZCD_STATUS_FALSE,
//	BEMF_ZCD_STATUS_TRUE,
//	BEMF_ZCD_STATUS_MISSED,
//}
//BEMF_ZcdStatus_T;

typedef enum
{
	BEMF_PHASE_A,
	BEMF_PHASE_B,
	BEMF_PHASE_C,
}
BEMF_PhaseId_T;



typedef struct
{
//	const HAL_BEMF_T * p_HAL_Bemf; //hw timer version
//	uin32_t TimerFreq;

	volatile const uint32_t * p_Timer; //sw timer, 59hr overflow using 20khz timer

	volatile const bemf_t * p_VPhaseA_ADCU;		/// User app provides pointer to ADC results. only valid during its off phase.
	volatile const bemf_t * p_VPhaseB_ADCU;
	volatile const bemf_t * p_VPhaseC_ADCU;
	volatile const bemf_t * p_VBus_ADCU;
	//module may need to handle unit conversion if phase bemf and vbus use different resistor values

	BEMF_SampleMode_T SampleMode;
	uint16_t ZeroCrossingThreshold_ADCU;		/// sensitivity offset

	//uint16_t BlankTimeScalar;
	//uint16_t UsePWMOnThreshold;			/// for mixed sample mode, use PWM On ZCD when PWM On cycle is above this time.
	//uin16t_t DiodeDrop; 					//if no complementary pwm vd/2

	volatile BEMF_Mode_T Mode;						/// UseSensorlessCommutation //substate

	//mode module determines off phase, map from inside or outside
	volatile const bemf_t * volatile p_VPhaseObserve_ADCU;	///observe phase. Phase-to-ground voltage,  Variable pointer to ADC results. Phase voltage PWM On: VPhase == (3/2)VBEMF + VBus/2
//	volatile BEMF_PhaseId_T PhaseObserveId;

	volatile uint32_t TimerReferenceZero;			/// reference start time, commutation start in active mode
	
	//bemf sample parameters
	volatile uint32_t TimeBlankPeriod;				/// Blank time after commutation. aka "demagnetization time".

	//ZCD sample data
//	volatile int16_t Emf_SignedADCU;				/// VPhase-to-neutral, ofset for ZCD , PWM On: (3/2)VBEMF
//	volatile int16_t EmfPrev_SignedADCU;			///

	//phase sample, track prev for zcd
	volatile uint16_t Emf_ADCU;					/// VPhase-to-neutral, PWM On: (3/2)VBEMF
	volatile uint16_t EmfPrev_ADCU;				///
	volatile uint32_t TimeEmf;					/// sample time stamp, from TimerReferenceZero
	volatile uint32_t TimeEmfPrev;				/// or use constant periodic sample time

	//sample for peak
	volatile uint16_t EmfPeak_ADCU;
	volatile uint16_t EmfPeakTemp_ADCU;

	//ZCD runtime parameters
	volatile bool IsZeroCrossingDetectionComplete;
	volatile bool IsBemfRising;

	//ZCD results
	volatile uint32_t TimeZeroCrossingDetect; 		/// Time between commutation and zero crossing, 30 degrees.
	volatile uint32_t TimeZeroCrossingPeriod; 		/// Time between 2 zero crossings, 60 degrees.
	volatile uint32_t ZeroCrossingCounter; 		///  consecutive zc

	//commutation timer
	volatile uint32_t TimeCyclePeriod;
	volatile uint16_t PhaseAdvanceTime;			/// Commutation delay reduction. phase advance for high freq when voltage leads current. proportional to inductance.

	//reliable
//	volatile bool IsReliable;
//	volatile uint32_t IsReliableCounter;
}
BEMF_T;

//call on Commutation
static inline void BEMF_StartCycle_IO(BEMF_T * p_bemf)
{
	//sw timer version
//	*p_bemf->p_Timer = 0U; //if writable timer, overflow precaution
	p_bemf->TimeCyclePeriod = *p_bemf->p_Timer - p_bemf->TimerReferenceZero; //back up time, will be overwritten upon zcd
	p_bemf->TimerReferenceZero = *p_bemf->p_Timer;

	p_bemf->TimeBlankPeriod = p_bemf->TimeCyclePeriod / 4U; //default case 25% of commutation time, 50% of zct. set 0 for passive mode?

//	if (p_bemf->IsZeroCrossingDetectionComplete == false) //commutation occurred before a zcd
//	{
////		p_bemf->ZeroCrossingCounter = 0U;
//	}
//	else
//	{
		p_bemf->IsZeroCrossingDetectionComplete = false;
//	}

	if(p_bemf->IsBemfRising) //must be after mapping phase, if new phase is rising
	{
		p_bemf->EmfPeak_ADCU 		= p_bemf->EmfPeakTemp_ADCU * 2U; // time 2 if captures 1/6 phase
		p_bemf->EmfPeakTemp_ADCU 	= 0U;
	}
	//todo capture peak

	p_bemf->TimeEmfPrev 	= 0; //if polling zcd seprate from sample, skip zcd proc occurrence prior to capture sample
	p_bemf->TimeEmf	 		= 0;
	//	p_bemf->Emf_ADCU 		= 0; //zcd proc occurs once prior to capture sample

	// module determine next phase from inside
//	BEMF_MapSector_IO();

	//hw timer version
	//HAL_BEMF_StartTimerInterrupt(p_bemf->p_HAL_Bemf, p_bemf->TimeZeroCrossingPeriod);
	//#ifdef CONFIG_BEMF_COMMUTATION_FUNCTION
	//	p_bemf->Commutation(p_bemf->p_Data);
	//#endif
}


static inline bool BEMF_PollCycleTimer(BEMF_T * p_bemf)
{
	bool isBoundary;

	if(*p_bemf->p_Timer - p_bemf->TimerReferenceZero > p_bemf->TimeCyclePeriod - p_bemf->PhaseAdvanceTime)
	{
//		p_bemf->TimeCyclePeriod = *p_bemf->p_Timer - p_bemf->TimerReferenceZero;
//		p_bemf->TimeCyclePeriod = p_bemf->TimeZeroCrossingPeriod;
		isBoundary = true;
	}
	else
	{
		isBoundary = false;
	}

	return isBoundary;
}

static inline bool BEMF_PollCycle_IO(BEMF_T * p_bemf)
{
	bool isBoundary;

	if (BEMF_PollCycleTimer(p_bemf))
	{
		BEMF_StartCycle_IO(p_bemf);
		isBoundary = true;
	}
	else
	{
		isBoundary = false;
	}

	return isBoundary;
}

//for known period, hall mode
//static inline void BEMF_CaptureCyclePeriod_IO(BEMF_T * p_bemf)
//{
//	p_bemf->TimeCyclePeriod = *p_bemf->p_Timer - p_bemf->TimerReferenceZero;
//}

/*
	ZCD Bemf Capture
	Captures 2 points
	Calc signed valued for ZCD

	p_VPhaseObserve_ADCU pre mapped from outside, can be called directly if ZCD is not needed

	BEMF_SAMPLE_MODE_PWM_ON -> need sufficient duty cycle to determine, 3/2 * bemf
	BEMF_SAMPLE_MODE_PWM_OFF -> Must subtract noise or use threshold. no 3/2 factor in this case

 */
static void CaptureEmf(BEMF_T * p_bemf, uint32_t vPhaseObserve)
{
	p_bemf->EmfPrev_ADCU = p_bemf->Emf_ADCU;
	p_bemf->Emf_ADCU = vPhaseObserve;

	if (vPhaseObserve > p_bemf->EmfPeakTemp_ADCU)
	{
		p_bemf->EmfPeakTemp_ADCU = vPhaseObserve;
	}
}

static bool PollTimeEmf(BEMF_T * p_bemf)
{
	uint32_t timeNew = *p_bemf->p_Timer - p_bemf->TimerReferenceZero;
	bool isCapture;

	if ((timeNew > p_bemf->TimeBlankPeriod) || (p_bemf->Mode == BEMF_MODE_PASSIVE))
	{
		p_bemf->TimeEmfPrev 			= p_bemf->TimeEmf; //check why prev is greater than curent
		p_bemf->TimeEmf 				= timeNew;
		isCapture = true;
	}
	else
	{
		isCapture = false;
	}

	return isCapture;
}


/*
	Set from outside mode
 */

//call on adc
//capture bemf with time stamp
//can capture bemf sample only at end of adc and proc later
//static inline void BEMF_CaptureVPhaseObserve_IO(BEMF_T * p_bemf){CaptureEmfSample(p_bemf, *p_bemf->p_VPhaseObserve_ADCU);}

static inline void BEMF_PollTimeCaptureVPhaseObserve_IO(BEMF_T * p_bemf)
{
	if(PollTimeEmf(p_bemf))
	{
		CaptureEmf(p_bemf, *p_bemf->p_VPhaseObserve_ADCU);
	}
}

/*
	Set from outside
 */
static inline void BEMF_MapPhaseA(BEMF_T * p_bemf) {p_bemf->p_VPhaseObserve_ADCU = p_bemf->p_VPhaseA_ADCU; }//p_bemf->PhaseObserveId = BEMF_PHASE_A;}
static inline void BEMF_MapPhaseB(BEMF_T * p_bemf) {p_bemf->p_VPhaseObserve_ADCU = p_bemf->p_VPhaseB_ADCU; }//p_bemf->PhaseObserveId = BEMF_PHASE_B;}
static inline void BEMF_MapPhaseC(BEMF_T * p_bemf) {p_bemf->p_VPhaseObserve_ADCU = p_bemf->p_VPhaseC_ADCU; }//p_bemf->PhaseObserveId = BEMF_PHASE_C;}

static inline void BEMF_MapCwPhaseAC_IO(BEMF_T * p_bemf){BEMF_MapPhaseB(p_bemf); p_bemf->IsBemfRising = false;}
static inline void BEMF_MapCwPhaseBC_IO(BEMF_T * p_bemf){BEMF_MapPhaseA(p_bemf); p_bemf->IsBemfRising = true;}
static inline void BEMF_MapCwPhaseBA_IO(BEMF_T * p_bemf){BEMF_MapPhaseC(p_bemf); p_bemf->IsBemfRising = false;}
static inline void BEMF_MapCwPhaseCA_IO(BEMF_T * p_bemf){BEMF_MapPhaseB(p_bemf); p_bemf->IsBemfRising = true;}
static inline void BEMF_MapCwPhaseCB_IO(BEMF_T * p_bemf){BEMF_MapPhaseA(p_bemf); p_bemf->IsBemfRising = false;}
static inline void BEMF_MapCwPhaseAB_IO(BEMF_T * p_bemf){BEMF_MapPhaseC(p_bemf); p_bemf->IsBemfRising = true;}

static inline void BEMF_MapCcwPhaseAC_IO(BEMF_T * p_bemf){BEMF_MapPhaseB(p_bemf); p_bemf->IsBemfRising = true;}
static inline void BEMF_MapCcwPhaseBC_IO(BEMF_T * p_bemf){BEMF_MapPhaseA(p_bemf); p_bemf->IsBemfRising = false;}
static inline void BEMF_MapCcwPhaseBA_IO(BEMF_T * p_bemf){BEMF_MapPhaseC(p_bemf); p_bemf->IsBemfRising = true;}
static inline void BEMF_MapCcwPhaseCA_IO(BEMF_T * p_bemf){BEMF_MapPhaseB(p_bemf); p_bemf->IsBemfRising = false;}
static inline void BEMF_MapCcwPhaseCB_IO(BEMF_T * p_bemf){BEMF_MapPhaseA(p_bemf); p_bemf->IsBemfRising = true;}
static inline void BEMF_MapCcwPhaseAB_IO(BEMF_T * p_bemf){BEMF_MapPhaseC(p_bemf); p_bemf->IsBemfRising = false;}

/*
 * Module determines sector Id via rising/falling detection
 */
//typedef enum
//{
//	HALL_SECTOR_0 = 0,
//	HALL_SECTOR_1 = 1,
//	HALL_SECTOR_2 = 2,
//	HALL_SECTOR_3 = 3,
//	HALL_SECTOR_4 = 4,
//	HALL_SECTOR_5 = 5,
//	HALL_SECTOR_6 = 6,
//	HALL_SECTOR_7 = 7,
//
//	HALL_PHASE_DISABLE = 0,
//	HALL_PHASE_AC = 1,
//	HALL_PHASE_BC = 2,  //sensor A
//	HALL_PHASE_BA = 3,	//sensor inv C
//	HALL_PHASE_CA = 4,
//	HALL_PHASE_CB = 5,
//	HALL_PHASE_AB = 6,
//	HALL_PHASE_7 = 7,
//
//	HALL_COMMUTATION_ANGLE_30 = 1,
//	HALL_COMMUTATION_ANGLE_90 = 2,
//	HALL_COMMUTATION_ANGLE_150 = 3,
//	HALL_COMMUTATION_ANGLE_210 = 4,
//	HALL_COMMUTATION_ANGLE_270 = 5,
//	HALL_COMMUTATION_ANGLE_330 = 6,
//
//	/*
//	 * Rotor Angle
//	 */
//	/* rotator position Id via boundary from CCW and CW*/
////	HALL_ANGLE_CCW_30 = 3, 		//HALL_PHASE_BA,
////	HALL_ANGLE_CCW_90 = 4, 		//HALL_PHASE_CA,
////	HALL_ANGLE_CCW_150 = 5, 	//HALL_PHASE_CB,
////	HALL_ANGLE_CCW_210 = 6, 	//HALL_PHASE_AB,
////	HALL_ANGLE_CCW_270 = 1, 	//HALL_PHASE_AC,
////	HALL_ANGLE_CCW_330 = 2, 	//HALL_PHASE_BC,
////
////	HALL_ANGLE_CW_30 = 5, 	//HALL_PHASE_CB,
////	HALL_ANGLE_CW_90 = 6, 	//HALL_PHASE_AB,
////	HALL_ANGLE_CW_150 = 1, 	//HALL_PHASE_AC,
////	HALL_ANGLE_CW_210 = 2, 	//HALL_PHASE_BC,
////	HALL_ANGLE_CW_270 = 3, 	//HALL_PHASE_BA,
////	HALL_ANGLE_CW_330 = 4, 	//HALL_PHASE_CA,
//} Hall_CommutationPhase_T;
//static inline void BEMF_MapSector(BEMF_T * p_bemf, )
//{
//
//
//}



/*
 * if adc conversion tracks phase
 */
//static inline void BEMF_CaptureVPhaseA_IO(BEMF_T * p_bemf){CaptureEmfSample(p_bemf, *p_bemf->p_VPhaseA_ADCU);}
//static inline void BEMF_CaptureVPhaseB_IO(BEMF_T * p_bemf){CaptureEmfSample(p_bemf, *p_bemf->p_VPhaseB_ADCU);}
//static inline void BEMF_CaptureVPhaseC_IO(BEMF_T * p_bemf){CaptureEmfSample(p_bemf, *p_bemf->p_VPhaseC_ADCU);}

static inline void BEMF_PollTimeCaptureVPhaseA_IO(BEMF_T * p_bemf)
{
	if(PollTimeEmf(p_bemf))
	{
		CaptureEmf(p_bemf, *p_bemf->p_VPhaseA_ADCU);
	}
}

static inline void BEMF_PollTimeCaptureVPhaseB_IO(BEMF_T * p_bemf)
{
	if(PollTimeEmf(p_bemf))
	{
		CaptureEmf(p_bemf, *p_bemf->p_VPhaseB_ADCU);
	}
}

static inline void BEMF_PollTimeCaptureVPhaseC_IO(BEMF_T * p_bemf)
{
	if(PollTimeEmf(p_bemf))
	{
		CaptureEmf(p_bemf, *p_bemf->p_VPhaseC_ADCU);
	}
}

static inline void BEMF_MapBemfRising_IO(BEMF_T * p_bemf){p_bemf->IsBemfRising = true;}
static inline void BEMF_MapBemfFalling_IO(BEMF_T * p_bemf){p_bemf->IsBemfRising = false;}


/*
	Set from outside:  IsBemfRising, VBemf TimeVBemf
	software compare, todo hw comparator

	Proc last capured sample
 */
static inline bool ProcZeroCrossingDetection(BEMF_T * p_bemf)
{
	int16_t vEmf;
	int16_t vEmfPrev;
	uint32_t timeZeroCrossing; /* new zc time */
	bool isZcd = false;
	
	int16_t vZero;

	if (p_bemf->Mode != BEMF_MODE_PASSIVE)
	{
		switch(p_bemf->SampleMode)
		{
//		case BEMF_SAMPLE_MODE_PWM_BIPOLAR: 	vZero = (int32_t)(*p_bemf->p_VPhaseObserve_ADCU) - ((int32_t)(*p_bemf->p_VBus_ADCU) / 2); break;
//		case BEMF_SAMPLE_MODE_PWM_MIXED: 			break;
		case BEMF_SAMPLE_MODE_PWM_ON: 		vZero = ((int32_t)(*p_bemf->p_VBus_ADCU) / 2) + (int32_t)p_bemf->ZeroCrossingThreshold_ADCU; break;
		case BEMF_SAMPLE_MODE_PWM_OFF: 		vZero =  (int32_t)p_bemf->ZeroCrossingThreshold_ADCU; break;
		default:	break;
		}
	}
	else
	{
		vZero = (int32_t)p_bemf->ZeroCrossingThreshold_ADCU;
	}
	
	if(!p_bemf->IsBemfRising) // invert falling bemf to use same rising zcd routine
	{
		vEmf 		= -(int16_t)p_bemf->Emf_ADCU;
		vEmfPrev 	= -(int16_t)p_bemf->EmfPrev_ADCU;
	}
	else
	{
		vEmf 		= p_bemf->Emf_ADCU;
		vEmfPrev 	= p_bemf->EmfPrev_ADCU;
	}

	if((vEmf - vZero) >= 0) // is zero crossing, calc time of zc
	{
		// find midpoint to account for error of (TimeBemf - TimeZeroCrossing) past zc time
		if ((vEmf - vEmfPrev) > (vEmf - vZero)) // zcd falls within 2 samples
		{
			timeZeroCrossing = p_bemf->TimeEmf - (((p_bemf->TimeEmf - p_bemf->TimeEmfPrev) * (vEmf - vZero) / (vEmf - vEmfPrev))); //estimate using slope

//			if (p_bemf->ZeroCrossingCounter < UINT32_MAX)
//			{
				p_bemf->ZeroCrossingCounter++;
//			}
		}
		else //missed zero crossing, both samples past vZero
		{
			timeZeroCrossing = p_bemf->TimeEmfPrev - ((p_bemf->TimeEmf - p_bemf->TimeEmfPrev) / 2U);
			p_bemf->ZeroCrossingCounter = 0u;
		}

		//			p_bemf->TimeZeroCrossingPeriod = timeZeroCrossing + p_bemf->TimeZeroCrossing;
		//			p_bemf->TimeZeroCrossingDetect = timeZeroCrossing;
		// average zc with last zc
		//		p_bemf->TimeZeroCrossingPeriod = (p_bemf->TimeZeroCrossingPeriod + (p_bemf->TimeZeroCrossingDetect + timeZeroCrossing)) / 2U;
		//		p_bemf->TimeZeroCrossingPeriod = (p_bemf->TimeCyclePeriod + (p_bemf->TimeZeroCrossingDetect * 2U)) / 2U;

		p_bemf->TimeZeroCrossingPeriod = (p_bemf->TimeZeroCrossingDetect + timeZeroCrossing);
		p_bemf->TimeZeroCrossingDetect = (p_bemf->TimeZeroCrossingPeriod) / 2U;

		isZcd = true;
	}
	

	return isZcd;
}


//poll last sample, must skip prev sample
static inline bool BEMF_PollZeroCrossingDetection(BEMF_T * p_bemf)
{
	bool isZcd = false;
	
	if((p_bemf->IsZeroCrossingDetectionComplete == false) && (p_bemf->TimeEmfPrev > 0U)) //at least 1 sample set
	{
		if (ProcZeroCrossingDetection(p_bemf) == true)
		{
			p_bemf->IsZeroCrossingDetectionComplete = true;
			
			if( p_bemf->ZeroCrossingCounter > 10U)
			{
				p_bemf->TimeCyclePeriod = p_bemf->TimeZeroCrossingPeriod;
			}

			//if  BEMF_CYCLE_PERIOD_FULL,  p_bemf->TimeCyclePeriod = p_bemf->TimeZeroCrossingPeriod*6
			//HAL_BEMF_StartTimerInterrupt(p_bemf->p_HAL_Bemf, p_bemf->TimeNextCommutation);

			//emf peak is ~ 2* emf at acd
	
			isZcd = true;
		}
	}

	return isZcd;
}

static inline uint32_t BEMF_GetZeroCrossingCounter(BEMF_T * p_bemf)
{
	return p_bemf->ZeroCrossingCounter;
}

//static inline BEMF_PhaseId_T BEMF_GetPhaseId(BEMF_T * p_bemf) {return p_bemf->PhaseObserveId;}
static inline uint16_t BEMF_GetTimeZeroCrossingPeriod(BEMF_T * p_bemf){return p_bemf->TimeZeroCrossingPeriod;}
static inline uint16_t BEMF_GetTimeZeroCrossingDetect(BEMF_T * p_bemf){return p_bemf->TimeZeroCrossingDetect;}
static inline uint16_t BEMF_GetTimeCyclePeriod(BEMF_T * p_bemf){return p_bemf->TimeCyclePeriod;}

/*

 */
static inline int16_t ConvertVPhaseToVBemf(BEMF_T * p_bemf, int16_t emf)
{
	uint16_t bemf;

	if (p_bemf->Mode == BEMF_MODE_COMMUTATION)
	{
		switch(p_bemf->SampleMode)
		{
//			case BEMF_SAMPLE_MODE_PWM_BIPOLAR: 	bemf = emf;				break; //todo
			case BEMF_SAMPLE_MODE_PWM_ON:  		bemf = emf * 2U / 3U; 	break;
			case BEMF_SAMPLE_MODE_PWM_OFF: 		bemf = emf;				break;
			default:	break;
		}
	}
	else
	{
		bemf = emf;
	}

	return bemf;
}

static inline int16_t BEMF_GetVBemfPeak_ADCU(BEMF_T * p_bemf)
{
	return ConvertVPhaseToVBemf(p_bemf, p_bemf->EmfPeak_ADCU);
}

static inline int16_t BEMF_GetVPhasePeak_ADCU(BEMF_T * p_bemf)
{
	return p_bemf->EmfPeak_ADCU;
}

#endif

//static inline bool BEMF_ProcStartUpReliable(BEMF_T * p_bemf, uint32_t measuredTime)
//{
//
//	if (measuredTime >> 4U == p_bemf->TimeCyclePeriod >> 4U) //within 15 ticks or 750us at 20khz
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

