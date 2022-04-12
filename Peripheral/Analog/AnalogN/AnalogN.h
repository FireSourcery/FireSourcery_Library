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
    @file 	.h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef ANALOG_N_H
#define ANALOG_N_H

#include "../Analog/Analog.h"
#include "../Analog/Config.h"

#include "Utility/Queue/Queue.h"

#include <stdint.h>
#include <stdbool.h>

typedef union
{
	struct
	{
		uint32_t Adc0 : 1U;
		uint32_t Adc1 : 1U;
		uint32_t Adc2 : 1U;
		uint32_t Adc3 : 1U;
		uint32_t Adc4 : 1U;
		uint32_t Adc5 : 1U;
		uint32_t Adc6 : 1U;
		uint32_t Adc7 : 1U;
		//isPeakFound
	};

	uint32_t AdcFlags;
}
AnalogN_AdcFlags_T;

typedef const struct
{
	Analog_Conversion_T CONVERSION;
	Analog_T * P_ANALOG;
}
AnalogN_Conversion_T;

#define CONFIG_ANALOG_N_CONVERSION(p_Virtual, PinId, p_Results, p_CallbackContext, p_AnalogIHost) \
{															\
	.CONVERSION =											\
	{														\
		.P_VIRTUAL 				= p_Virtual,				\
		.PIN 					= PinId,					\
		.P_RESULTS_BUFFER 		= p_Results,				\
		.P_CALLBACK_CONTEXT 	= p_CallbackContext,		\
	},														\
	.P_ANALOG 					= p_AnalogIHost,			\
}

typedef const struct
{
	Analog_T * const P_ANALOGS;
	uint8_t ANALOG_COUNT;
}
AnalogN_Config_T;

typedef struct
{
	const AnalogN_Config_T CONFIG;
}
AnalogN_T;

/*!
	@brief	Capture ADC results, when conversion is complete.
			Run in corresponding ADC ISR

	ADC ISR should be higher priority than thread calling Analog_Activate()
 */
static inline void AnalogN_CaptureResults_ISR(AnalogN_T * p_analogn, uint8_t analogId)
{
	Analog_T * p_analogI = &p_analogn->CONFIG.P_ANALOGS[analogId];
//	AnalogN_Conversion_T * p_activeConversion = p_analogI->p_ActiveConversion; /* casting void pointer */
//	Queue_PeekFront(&p_analogI->ConversionQueue, &p_activeConversion); //read an address

//	const Analog_ConversionAdc_T * p_adcConversion 	= &p_activeConversion->P_ADC_CONVERSIONS[analogId];
//	const Analog_ConversionVirtualMap_T * p_map 			= &p_activeConversion->MAP;
//	AnalogN_AdcFlags_T * p_signalComplete 			= p_activeConversion->P_SIGNAL_BUFFER;
////	Analog_ConversionOptions_T options 		 		= p_map->P_VIRTUAL_CONVERSION->OPTIONS;
//	Analog_OnComplete_T onConversionComplete  		= p_map->P_VIRTUAL_CONVERSION->ON_COMPLETE;
//	void * p_onConversionCompleteContext 			= p_map->P_ON_COMPLETE_CONTEXT;

//	volatile static uint32_t debug = 0;

//	if((_Analog_GetIsActive(p_analogI) == true)) //debug only
//	{


	if (_Analog_CaptureResults(p_analogI) == true) //all channels complete
	{
		//Do not run if is capture local peak and not peak found
//		if((((options.CaptureLocalPeak == true) && (p_analogI->IsLocalPeakFound == false)) == false))
//		{
//			p_signalComplete->AdcFlags &= ~(1UL << analogId); //debug only
//
//			if (onConversionComplete != 0U)
//			{
//				// only run oncomplete if conversion has completed on all adcs
//	//			p_signalComplete->AdcFlags &= ~(1UL << analogId);
//				if (p_signalComplete->AdcFlags == 0U)
//				{
//					onConversionComplete(p_onConversionCompleteContext);
//				}
//			}
//		}

		//todo check repeat function
//		if((options.ContinuousConversion == true) && (((options.CaptureLocalPeak == true) && (p_analogI->IsLocalPeakFound == true)) == false))
//		{
//#if !defined(CONFIG_ANALOG_ADC_HW_FIFO_ENABLE)
//			if(p_activeConversion->MAP.P_VIRTUAL_CONVERSION->CHANNEL_COUNT > 1U)
//			{
//				_Analog_ActivateConversion(p_analogI, p_adcConversion, p_map);
//			}
//			else
//			{
//				p_analogI->ActiveConversionIndex = 0U; //auto reactivate, still need to reset index
//			}
//#elif defined(CONFIG_ANALOG_SW_CONTINUOUS_CONVERSION)
//			_Analog_ActivateConversion(p_analogI, p_adcConversion, p_map);
//#endif
//		}
//		else //local peak found
//		{
//
//			if(_Analog_DequeueConversion(p_analogI) == true)
//			{
//				/* p_analogI->p_ActiveConversion updated */
//				p_activeConversion = p_analogI->p_ActiveConversion;
//				_Analog_ActivateConversion(p_analogI, &p_activeConversion->P_ADC_CONVERSIONS[analogId], &p_activeConversion->MAP);
//			}
//			else
//			{
////			Analog_Dectivate(p_analogI);
//			}
//		}
	}
//
//	}
//	else
//	{
//		debug++;
//	}
}

/*
 * Options AnalogN options on all adcs
 */
//static inline bool AnalogN_EnqueueConversionOptions (AnalogN_T * p_analogn, AnalogN_AdcFlags_T activeAdcs)
//{
//	bool isSuccess;
//	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
//	{
//		Analog_EnqueueConversion(&p_analogn->CONFIG.P_ANALOGS[iAdc], &p_conversion->CONVERSION);
//	}
//}

/*
 */
//static inline bool AnalogN_ActivateConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
//{
////	EnqueueConversionCommon(p_analogn, p_conversion, ANALOG_ACTIVATE_MODE_OVERWRITE);
//	Analog_ActivateConversion(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
//}

static inline bool AnalogN_EnqueueConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
//	EnqueueConversionCommon(p_analogn, p_conversion, ANALOG_ACTIVATE_MODE_ENQUEUE_BACK);
	return Analog_EnqueueConversion(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
}

//static inline bool AnalogN_EnqueueFrontConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
//{
////	EnqueueConversionCommon(p_analogn, p_conversion, ANALOG_ACTIVATE_MODE_ENQUEUE_FRONT);
//	Analog_EnqueueFrontConversion(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
//}

static inline bool AnalogN_EnqueueConversionOptions(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
	return Analog_EnqueueConversionOptions(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
}


static inline bool AnalogN_EnqueueConversionOptions_Group(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
	return Analog_EnqueueConversionOptions_Group(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
}

static inline bool AnalogN_EnqueueConversion_Group(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
	return Analog_EnqueueConversion_Group(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
}

/*
 *  AnalogN_AdcFlags_T activeAdcs:
 * Single threaded N adc - use
 * Multi Threaded N adc - not use
 * Single threaded 1 adc - use for sync, N function preserve same interface
 * Multi Threaded 1 adc - not use
 */
static inline void AnalogN_PauseQueue(AnalogN_T * p_analogn, AnalogN_AdcFlags_T activeAdcs)
{
#if  (defined(CONFIG_ANALOG_MULTITHREADED) || defined(CONFIG_ANALOG_CRITICAL_USE_GLOBAL))
	Critical_Enter();
#elif (defined(CONFIG_ANALOG_SINGLE_THREADED))
	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		if (((1U << iAdc) & activeAdcs) != 0U)
		{
			_Analog_EnterCritical(&p_analogn->CONFIG.P_ANALOGS[iAdc]);
		}
	}
#endif
}

static inline void AnalogN_ResumeQueue(AnalogN_T * p_analogn, AnalogN_AdcFlags_T activeAdcs)
{
#if  (defined(CONFIG_ANALOG_MULTITHREADED) || defined(CONFIG_ANALOG_CRITICAL_USE_GLOBAL))
	Critical_Exit();
#elif (defined(CONFIG_ANALOG_SINGLE_THREADED))
	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		if (((1U << iAdc) & activeAdcs) != 0U)
		{
			_Analog_ExitCritical(&p_analogn->CONFIG.P_ANALOGS[iAdc]);
		}
	}
#endif
}

//extern void AnalogN_Init(AnalogN_T * p_analogn);
//extern void AnalogN_EnqueueConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion);
//extern void AnalogN_EnqueueFrontConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion);
//extern void AnalogN_PollDequeueConversion(AnalogN_T * p_analogn);

#endif


