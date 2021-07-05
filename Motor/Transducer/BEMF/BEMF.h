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

	BEMF_SampleMode_T SampleMode;
	uint16_t ZeroCrossingThreshold_ADCU;		/// sensitivity offset
//	uint16_t ZeroCrossingThresholdSampleZero_ADCU;		/// set value other than zero

	//uint16_t BlankTimeScalar;
	//uint16_t UsePWMOnThreshold;			/// for mixed sample mode, use PWM On ZCD when PWM On cycle is above this time.
	//uin16t_t DiodeDrop; 					//if no complementary pwm vd/2

	volatile BEMF_Mode_T Mode;						/// UseSensorlessCommutation //substate

	//mode module determines off phase, map from inside or outside
	volatile const bemf_t * volatile p_VPhaseObserve_ADCU;	///observe phase. Phase-to-ground voltage,  Variable pointer to ADC results. Phase voltage PWM On: VPhase == (3/2)VBEMF + VBus/2
//	volatile BEMF_PhaseId_T PhaseObserveId;

//	volatile uint16_t VPhaseObserve_ADCU; //capture buffer
//	volatile uint16_t VPhaseObservePeak_ADCU;

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
	volatile uint32_t TimeCyclePeriod; 		///  time till next, polymorphic
	volatile uint16_t PhaseAdvanceTime;			/// Commutation delay reduction. phase advance for high freq when voltage leads current. proportional to inductance.




	//reliable
//	volatile bool IsReliable;
//	volatile uint32_t IsReliableCounter;
}
BEMF_T;




//call on Commutation
static inline void BEMF_StartCycle_IO(BEMF_T * p_bemf)
{
	static uint32_t debug = 0;

	if (p_bemf->IsZeroCrossingDetectionComplete == false) //commutation occurred before a zcd
	{
//		p_bemf->IsReliable = false;
		p_bemf->ZeroCrossingCounter = 0U;
		debug++;
	}
	else
	{
		debug = 0;
		p_bemf->IsZeroCrossingDetectionComplete = false;
	}

	p_bemf->TimeBlankPeriod = p_bemf->TimeZeroCrossingDetect / 2; //default case 25% of commutation time, 50% of zct. set 0 for passive mode?

	p_bemf->EmfPeak_ADCU = p_bemf->EmfPeakTemp_ADCU;
	p_bemf->EmfPeakTemp_ADCU = 0U;

	// module determine next phase from inside
//	BEMF_MapSector_IO();

	//sw timer version
//	*p_bemf->p_Timer = 0U; //if writable timer, overflow precaution
	p_bemf->TimerReferenceZero = *p_bemf->p_Timer;

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
		p_bemf->TimeCyclePeriod = p_bemf->TimeZeroCrossingPeriod; //back up time, will be overwritten upon zcd
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


/*
	ZCD Bemf Capture
	Captures 2 points
	Calc signed valued for ZCD

	p_VPhaseObserve_ADCU pre mapped from outside, can be called directly if ZCD is not needed

	BEMF_SAMPLE_MODE_PWM_ON -> need sufficient duty cycle to determine, 3/2 * bemf
	BEMF_SAMPLE_MODE_PWM_OFF -> Must subtract noise or use threshold. no 3/2 factor in this case

 */
static void CaptureEmf(BEMF_T * p_bemf, int32_t vPhaseObserve)
{
	p_bemf->EmfPrev_ADCU = p_bemf->Emf_ADCU;
	p_bemf->Emf_ADCU = vPhaseObserve;

	if (vPhaseObserve > p_bemf->EmfPeakTemp_ADCU)
	{
		p_bemf->EmfPeakTemp_ADCU = vPhaseObserve;
	}

//	if (p_bemf->Mode != BEMF_MODE_PASSIVE)
//	{
//		switch(p_bemf->SampleMode)
//		{
////		case BEMF_SAMPLE_MODE_PWM_BIPOLAR: 	p_bemf->Emf_SignedADCU = (int32_t)(*p_bemf->p_VPhaseObserve_ADCU) - ((int32_t)(*p_bemf->p_VBus_ADCU) / 2); break;
////		case BEMF_SAMPLE_MODE_PWM_MIXED: 			break;
//		case BEMF_SAMPLE_MODE_PWM_ON: 		p_bemf->Emf_SignedADCU = vPhaseObserve - ((int32_t)(*p_bemf->p_VBus_ADCU) / 2); break;
////		case BEMF_SAMPLE_MODE_PWM_OFF: 		p_bemf->Emf_SignedADCU = (int32_t)(*p_bemf->p_VPhaseObserve_ADCU) - (int32_t)p_bemf->ZeroCrossingThreshold_ADCU; break;
//		default:	break;
//		}
//	}
//	else
//	{
//		p_bemf->Emf_SignedADCU = vPhaseObserve;
//	}


}

static bool PollTimeEmf(BEMF_T * p_bemf)
{
	uint32_t timeNew = *p_bemf->p_Timer - p_bemf->TimerReferenceZero;
	bool isCapture;

	if ((timeNew > p_bemf->TimeBlankPeriod) || (p_bemf->Mode == BEMF_MODE_PASSIVE))
	{
		p_bemf->TimeEmfPrev 			= p_bemf->TimeEmf;
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
//static inline void BEMF_MapSector(BEMF_T * p_bemf)
//{

//	Direction = IsBemfRising

//	switch (p_bemf->NextSector)
//	{
//	case MOTOR_SECTOR_ID_0:
//		break;
//
//	case MOTOR_SECTOR_ID_1: //Phase AC
//		if ( p_bemf->IsBemfRising == true) 				{p_bemf->NextSector = MOTOR_SECTOR_ID_2; BEMF_MapCcwPhaseAC_IO(p_bemf);}
//		else											{p_bemf->NextSector = MOTOR_SECTOR_ID_6; BEMF_MapCwPhaseAC_IO(p_bemf);}
//
//		break;
//	case MOTOR_SECTOR_ID_2:
//		if (p_bemf->Direction == MOTOR_DIRECTION_CCW) 	{p_bemf->NextSector = MOTOR_SECTOR_ID_3; BEMF_MapCcwPhaseBC_IO(&p_bemf->Bemf);}
//		else											{p_bemf->NextSector = MOTOR_SECTOR_ID_1; BEMF_MapCwPhaseBC_IO(&p_bemf->Bemf);}
//		break;
//	case MOTOR_SECTOR_ID_3:
//		if (p_bemf->Direction == MOTOR_DIRECTION_CCW) 	{p_bemf->NextSector = MOTOR_SECTOR_ID_4; BEMF_MapCcwPhaseBA_IO(&p_bemf->Bemf);}
//		else											{p_bemf->NextSector = MOTOR_SECTOR_ID_2; BEMF_MapCwPhaseBA_IO(&p_bemf->Bemf);}
//		break;
//	case MOTOR_SECTOR_ID_4:
//		if (p_bemf->Direction == MOTOR_DIRECTION_CCW) 	{p_bemf->NextSector = MOTOR_SECTOR_ID_5; BEMF_MapCcwPhaseCA_IO(&p_bemf->Bemf);}
//		else											{p_bemf->NextSector = MOTOR_SECTOR_ID_3; BEMF_MapCwPhaseCA_IO(&p_bemf->Bemf);}
//		break;
//	case MOTOR_SECTOR_ID_5:
//		if (p_bemf->Direction == MOTOR_DIRECTION_CCW) 	{p_bemf->NextSector = MOTOR_SECTOR_ID_6; BEMF_MapCcwPhaseCA_IO(&p_bemf->Bemf);}
//		else											{p_bemf->NextSector = MOTOR_SECTOR_ID_4; BEMF_MapCwPhaseCA_IO(&p_bemf->Bemf);}
//		break;
//	case MOTOR_SECTOR_ID_6:
//		if (p_bemf->Direction == MOTOR_DIRECTION_CCW) 	{p_bemf->NextSector = MOTOR_SECTOR_ID_1; BEMF_MapCcwPhaseAB_IO(&p_bemf->Bemf);}
//		else											{p_bemf->NextSector = MOTOR_SECTOR_ID_5; BEMF_MapCwPhaseAB_IO(&p_bemf->Bemf);}
//		break;
//	case MOTOR_SECTOR_ID_7:
//		//set error
//		break;
//	default:
//		break;
//	}
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

 

//static inline void BEMF_CapturePeak_IO(BEMF_T * p_bemf)
//{
////module control observe sector, adc samples all
//}

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
//		case BEMF_SAMPLE_MODE_PWM_OFF: 		vZero =  (int32_t)p_bemf->ZeroCrossingThreshold_ADCU; break;
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
		}
		else //missed zero crossing, both samples past vZero
		{
			timeZeroCrossing = p_bemf->TimeEmfPrev - ((p_bemf->TimeEmf - p_bemf->TimeEmfPrev) / 2U);
		}

		//			p_bemf->TimeZeroCrossingPeriod = timeZeroCrossing + p_bemf->TimeZeroCrossing;
		//			p_bemf->TimeZeroCrossingDetect = timeZeroCrossing;
		// average zc with last zc
//		p_bemf->TimeZeroCrossingPeriod = (p_bemf->TimeZeroCrossingPeriod + (p_bemf->TimeZeroCrossingDetect + timeZeroCrossing)) / 2U;
		p_bemf->TimeZeroCrossingDetect = (p_bemf->TimeZeroCrossingDetect + timeZeroCrossing) / 2U;
		p_bemf->TimeZeroCrossingPeriod = (p_bemf->TimeZeroCrossingPeriod + (p_bemf->TimeZeroCrossingDetect * 2U)) / 2U; //increase contribution of current timeZeroCrossing

		isZcd = true;
	}
	

	return isZcd;
}


//poll last sample
static inline bool BEMF_PollZeroCrossingDetection(BEMF_T * p_bemf)
{
	bool isZcd = false;
	
	if(p_bemf->IsZeroCrossingDetectionComplete == false)
	{
		if (ProcZeroCrossingDetection(p_bemf))
		{
			p_bemf->IsZeroCrossingDetectionComplete = true;
			
			//if  BEMF_CYCLE_PERIOD_FULL,  p_bemf->TimeCyclePeriod = p_bemf->TimeZeroCrossingPeriod*6
			p_bemf->TimeCyclePeriod = p_bemf->TimeZeroCrossingPeriod; //updated period
			//HAL_BEMF_StartTimerInterrupt(p_bemf->p_HAL_Bemf, p_bemf->TimeNextCommutation);
	
			if (p_bemf->ZeroCrossingCounter < UINT32_MAX)
			{
				p_bemf->ZeroCrossingCounter++;
			}
	 
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
//static inline uint16_t BEMF_GetTimeAngle60(BEMF_T * p_bemf){return p_bemf->TimeZeroCrossingPeriod - p_bemf->PhaseAdvanceTime;}
//static inline uint16_t BEMF_GetTimeAngle30(BEMF_T * p_bemf){return p_bemf->TimeZeroCrossingDetect - p_bemf->PhaseAdvanceTime;}


/*

 */
static inline int16_t ConvertVPhaseToVBemf(BEMF_T * p_bemf, int16_t emf)
{
	uint16_t bemf;

	if (p_bemf->Mode == BEMF_MODE_COMMUTATION)
	{
		switch(p_bemf->SampleMode)
		{
			case BEMF_SAMPLE_MODE_PWM_BIPOLAR: 	bemf = emf;				break; //todo
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
	return ConvertVPhaseToVBemf(p_bemf, p_bemf->Emf_ADCU);
}

static inline int16_t BEMF_GetVPhasePeak_ADCU(BEMF_T * p_bemf)
{
	return p_bemf->Emf_ADCU;
}


//
//static inline int16_t BEMF_GetVBemfPhaseToNeutral(BEMF_T * p_bemf)
//{
//	return p_bemf->Emf_SignedADCU;
//}
//


//phase to ground //incorrect during demag time
//static inline uint16_t BEMF_GetVBemfPhaseToGround(BEMF_T * p_bemf)
//{
//	return *p_bemf->p_VPhaseObserve_ADCU;
//}

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

