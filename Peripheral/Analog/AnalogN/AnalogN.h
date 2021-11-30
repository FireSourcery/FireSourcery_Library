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

/*
 * Compile time determine which analogs are used
 * Per conversion instance, pass to activate functions
 * pass map by pointer
 * //conversion map has analog->channellist	//for each analog
 * //thsi way conversion dont need to provide id
 */
typedef const struct
{
	const Analog_ConversionMap_T MAP; //change to none pointer?
	const Analog_ConversionAdc_T * const P_ADC_CONVERSIONS; /* 2D array*/
	AnalogN_AdcFlags_T * const P_SIGNAL_BUFFER;			//at least buffer needs to be implemented
	const AnalogN_AdcFlags_T ANALOGS_ACTIVE; 			//also on complete state, results map mask
}
AnalogN_Conversion_T;

//const Analog_ConversionVirtual_T * const P_VIRTUAL_CONVERSION;
//analog_adcresult_t * const P_RESULTS_BUFFER;	 /*!< Persistent ADC results buffer, virtual channel index.  */
//void * P_ON_COMPLETE_CONTEXT; //todo isPeakSignalBuffer associate with conversion. for consistion behavior from app. if associate with Analog_T channels on different analoy will continue to run untill the first peak is found for all converision channels on that adc

#define ANALOG_N_CONVERSION_CONFIG(p_VirtualConversion, p_Results, p_OnCompleteContext, p_AdcConversions, p_Signal, SignalComplete) \
{														\
	.MAP =												\
	{													\
		.P_VIRTUAL_CONVERSION 	= p_VirtualConversion,	\
		.P_RESULTS_BUFFER 		= p_Results ,			\
		.P_ON_COMPLETE_CONTEXT 	= p_OnCompleteContext,	\
	},													\
	.P_ADC_CONVERSIONS 			= p_AdcConversions		\
	.P_SIGNAL_BUFFER 			= p_Signal				\
	.ANALOGS_ACTIVE.AdcFlags 	= SignalComplete		\
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
	AnalogN_Conversion_T * p_activeConversion = p_analogI->p_ActiveConversion; /* casting void pointer */
//	Queue_PeekFront(&p_analogI->ConversionQueue, &p_activeConversion); //read an address

	const Analog_ConversionAdc_T * p_adcConversion 	= &p_activeConversion->P_ADC_CONVERSIONS[analogId];
	const Analog_ConversionMap_T * p_map 			= &p_activeConversion->MAP;
	AnalogN_AdcFlags_T * p_signalComplete 			= p_activeConversion->P_SIGNAL_BUFFER;
	Analog_ConversionOptions_T options 		 		= p_map->P_VIRTUAL_CONVERSION->OPTIONS;
	Analog_OnComplete_T onConversionComplete  		= p_map->P_VIRTUAL_CONVERSION->ON_COMPLETE;
	void * p_onConversionCompleteContext 			= p_map->P_ON_COMPLETE_CONTEXT;


	if (_Analog_CaptureResults(p_analogI, p_adcConversion, p_map) == true) //all channels complete
	{
		//Do not run if is capture local peak and not peak found
//		if((((options.CaptureLocalPeak == true) && (p_analogI->IsLocalPeakFound == false)) == false))
		{
			p_signalComplete->AdcFlags &= ~(1UL << analogId); //debug only

			if (onConversionComplete != 0U)
			{
				// only run oncomplete if conversion has completed on all adcs
	//			p_signalComplete->AdcFlags &= ~(1UL << analogId);
				if (p_signalComplete->AdcFlags == 0U)
				{
					onConversionComplete(p_onConversionCompleteContext);
				}
			}
		}

		//todo check repeat function
		if((options.ContinuousConversion == true) && (((options.CaptureLocalPeak == true) && (p_analogI->IsLocalPeakFound == true)) == false))
		{
#if !defined(CONFIG_ANALOG_ADC_HW_BUFFER)
			if(p_activeConversion->MAP.P_VIRTUAL_CONVERSION->CHANNEL_COUNT > 1U)
			{
				_Analog_ActivateConversion(p_analogI, p_adcConversion, p_map);
			}
			else
			{
				p_analogI->ActiveConversionIndex = 0U; //auto reactivate, still need to reset index
			}
#elif defined(CONFIG_ANALOG_SW_CONTINUOUS_CONVERSION)
			_Analog_ActivateConversion(p_analogI, p_adcConversion, p_map);
#endif
		}
		else
		{
			//local peak found
			if(_Analog_DequeueConversion(p_analogI) == true)
			{
				/* p_analogI->p_ActiveConversion updated */
				p_activeConversion = p_analogI->p_ActiveConversion;
				_Analog_ActivateConversion(p_analogI, &p_activeConversion->P_ADC_CONVERSIONS[analogId], &p_activeConversion->MAP);
			}
//			else
//			{
////				Analog_Dectivate(p_analogI);
//			}
		}
	}
}

extern void AnalogN_Init(AnalogN_T * p_analogn);
extern void AnalogN_EnqueueConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion);
extern void AnalogN_EnqueueFrontConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion);
extern void AnalogN_PollDequeueConversion(AnalogN_T * p_analogn);

#endif


