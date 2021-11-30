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
	@file 	Analog.h
	@author FireSoucery
	@brief 	ADC wrapper module. Implements run time configurable settings.
	@version V0
*/
/******************************************************************************/
#ifndef ANALOG_H
#define ANALOG_H

#include "HAL_Analog.h"
#include "Config.h"

#include "Utility/Queue/Queue.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_ANALOG_CRITICAL_LIBRARY_ENABLE
	#include "System/Critical/Critical.h"
#elif defined(CONFIG_ANALOG_CRITICAL_DISABLE)

#endif

/*
 * Software side data storage
 */
#ifdef CONFIG_ANALOG_ADC_RESULT_UINT8
	typedef uint8_t analog_adcresult_t;
#elif defined(CONFIG_ANALOG_ADC_RESULT_UINT16)
	typedef uint16_t analog_adcresult_t;
#endif

/*
 * ADC Pin Channel - May or may not need 32 bits
 */
#ifdef CONFIG_ANALOG_ADC_PIN_UINT8
	typedef uint8_t adcpin_t;
#elif defined(CONFIG_ANALOG_ADC_PIN_UINT16)
	typedef uint16_t adcpin_t;
#elif defined(CONFIG_ANALOG_ADC_PIN_UINT32)
	typedef uint32_t analog_adcpin_t;
#endif

typedef uint8_t analog_virtual_t;



typedef void (*const Analog_OnComplete_T)(void * p_context);

typedef const struct
{
	// allow multiple conversion to share 1 result buffer,
	// repeat channels to share one result destination
	const analog_virtual_t 		CHANNEL; 		/* index into results buffer */
	const Analog_OnComplete_T 	ON_COMPLETE; 	/* On each channel complete, runs first adc isr, depends on adc hw fifo length */
}
Analog_ConversionVirtualChannel_T;

/*
 * Config options
 */
typedef struct
{
	uint32_t HwTriggerConversion 	:1;	/* 1 Hw Trigger Per Conversion */
	uint32_t ContinuousConversion 	:1;
	uint32_t CaptureLocalPeak 		:1;	/* for now, conversion stops on 1 local peak in channel set, user must also set ContinuousConversion */

//	uint32_t  HwAveraging
//	uint32_t UseHwTriggerChannel 		:1; /* Per Hw buffer complete. Per Channel if both are set*/
//	uint32_t UseInterrupt 				:1;
//	uint32_t UseDma 					:1;

	//uint8_t Priority
}
Analog_ConversionOptions_T;


typedef const struct
{
	const Analog_ConversionVirtualChannel_T * P_CHANNELS; 		/* Virtual Index */
	const uint8_t CHANNEL_COUNT;
	const Analog_OnComplete_T ON_COMPLETE;				/* On conversion group complete */ //for analogN use if all channel results are needed, otherwise channel onomplete will suffice. set to only run on peak if find peak is active
	const Analog_ConversionOptions_T OPTIONS;
}
Analog_ConversionVirtual_T;


typedef const struct
{
	const Analog_ConversionVirtual_T * const P_VIRTUAL_CONVERSION;
	analog_adcresult_t * const P_RESULTS_BUFFER;	 /*!< Persistent ADC results buffer, virtual channel index.  */
	void * P_ON_COMPLETE_CONTEXT; //todo isPeakSignalBuffer associate with conversion. for consistion behavior from app. if associate with Analog_T channels on different analoy will continue to run untill the first peak is found for all converision channels on that adc
}
Analog_ConversionMap_T;

typedef const struct
{
	const analog_adcpin_t PIN;
	uint8_t VIRTUAL_INDEX; /* match to oncomplete functions */
}
Analog_ConversionAdcChannel_T;

/* iterate through adc channels, known at compile time, faster */
typedef const struct
{
	const Analog_ConversionAdcChannel_T * P_CHANNELS;
	uint8_t CHANNEL_COUNT;
}
Analog_ConversionAdc_T;

typedef const struct
{
//	const Analog_ConversionVirtual_T * const P_VIRTUAL_CONVERSION;
//	analog_adcresult_t * const P_RESULTS_BUFFER;	 /*!< Persistent ADC results buffer, virtual channel index.  */
//	void * P_ON_COMPLETE_CONTEXT;
	const Analog_ConversionMap_T MAP;
	const Analog_ConversionAdc_T ADC_CONVERSION;
}
Analog_Conversion_T;

typedef const struct
{
	HAL_ADC_T * P_HAL_ADC; /*!< 1 ADC: pointer to ADC register map base address */
#if defined(CONFIG_ANALOG_ADC_HW_BUFFER)
	const uint8_t ADC_BUFFER_LENGTH;
#endif
}
Analog_Config_T;

/*
 * Analog_T per ADC
 */
typedef struct
{
	const Analog_Config_T CONFIG;
	/*
	 * Channel count needs to enqueue for AnalogN
	 */
	Queue_T ConversionQueue;

	/*
	 * 	Save for on ADC complete.
	 *  Need Active conversion or Enqueue Front would need to shift all items in queue
	 *  also use as adc active flag
	 */
	const void * volatile p_ActiveConversion; 	/*! Queue unit type Selected conversion group in process */
 	volatile uint8_t ActiveConversionIndex; 	/*! Channel index of Selected conversion group */
#ifdef CONFIG_ANALOG_ADC_HW_BUFFER
	uint8_t ActiveAdcChannelCount; 				/*! Hw fifo only. Number of active channels being processed by ADC */
#endif

	bool IsLocalPeakFound;

	volatile uint32_t count;

	//	bool UseConversionOptions; //select use conversion options or adc options

	//common setting for all future conversion
	//Applicable next conversion, multithread must enter critical before changes
//	Analog_ConversionOptions_T Options; 	/* Default config when conversion does not specify */
//	bool WaitForTrigger; //can change to enum to support various trigger sources
//	Analog_TriggerMode_T TriggerMode;
	//if 2 high priority conversion, activate sequentially, do not overwrite the previous
// 	bool OverwriteEnable;
// 	uint8_t ActiveConversionPriority;

	//or us runtime defined
	// 	volatile uint8_t ActiveConversionChannelCount;
//	Queue_T ChannelQueue;

}
Analog_T;

#ifdef CONFIG_ANALOG_ADC_HW_BUFFER
#define ANALOG_CONFIG_ADC_BUFFER_LENGTH(BufferLength) \
	.ADC_BUFFER_LENGTH = BufferLength,
#else
#define ANALOG_CONFIG_ADC_BUFFER_LENGTH(BufferLength)
#endif

/*
* Queue buffer length in units
*/
#define ANALOG_CONFIG(p_HalAdc, HwBufferLength, p_ConversionBuffer, ConversionQueueLength) \
{															\
	.CONFIG =												\
	{														\
		.P_HAL_ADC = p_HalAdc,								\
		ANALOG_CONFIG_ADC_BUFFER_LENGTH(HwBufferLength)		\
	},														\
	.ConversionQueue = QUEUE_CONFIG(p_ConversionBuffer, ConversionQueueLength, sizeof(Analog_Conversion_T *), 0U),	\
}

extern void _Analog_ActivateAdc(const Analog_T * p_analog, const Analog_ConversionAdc_T * p_adcConversion, const Analog_ConversionMap_T * p_adcMap);


/******************************************************************************/
/*!
 *
 */
/******************************************************************************/
static inline void _Analog_EnterCritical(Analog_T * p_analog)
{
#if  (defined(CONFIG_ANALOG_MULTITHREADED) || defined(CONFIG_ANALOG_CRITICAL_USE_GLOBAL))
	/*
	 * Multithreaded calling of Activate.
	 * Must implement Critical_Enter
	 *
	 * Higher priority thread may overwrite Conversion setup data before ADC ISR returns.
	 * e.g. must be implemented if calling from inside interrupts and main.
	 */
	Critical_Enter();
//#if defined(CONFIG_ANALOG_MULTITHREADED_USE_MUTEX)
//	return Critical_AquireMutex(&p_analog->Mutex);
//#endif

#elif (defined(CONFIG_ANALOG_SINGLE_THREADED))
	/*
	 * Single threaded calling of Activate.
	 * Single threaded case, and calling thread is lower priority than ADC ISR, may implement ADC_DisableInterrupt instead of Critical_Enter global disable interrupt
	 *
	 * If calling thread is lower priority than ADC ISR, ADC ISR may occur after Conversion setup data is written by lower priority thread.
	 * In single threaded calling of Activate and calling thread priority is higher than ADC ISR, Activate will run to completion, overwriting the active conversion,
	 * Disable IRQ is not needed, however ADC ISR will still need Critical_Enter
	 */
	Analog_DisableInterrupt(p_analog);
#endif
}

static inline void _Analog_ExitCritical(Analog_T * p_analog)
{
#if (defined(CONFIG_ANALOG_MULTITHREADED) || defined(CONFIG_ANALOG_CRITICAL_USE_GLOBAL))
	Critical_Exit();
#elif  defined(CONFIG_ANALOG_SINGLE_THREADED) 	//interrupts already enabled
#endif
}

/*!
	@brief 	Get Active channel count called before ActivateConversion()
 */
static inline uint8_t SetAnalogActiveAdcChannelCount(Analog_T * p_analog, uint8_t remainingChannelCount)
{
#if defined(CONFIG_ANALOG_ADC_HW_BUFFER)
	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
	return (remainingChannelCount < p_analog->CONFIG.ADC_BUFFER_LENGTH) ? remainingChannelCount : p_analog->CONFIG.ADC_BUFFER_LENGTH;
	p_analog->ActiveAdcChannelCount = remainingChannelCount
#else
	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
	(void) p_analog;
	(void) remainingChannelCount;
	return 1U;
#endif
}

static inline uint8_t GetAnalogActiveAdcChannelCount(const Analog_T * p_analog)
{
#if defined(CONFIG_ANALOG_ADC_HW_BUFFER)
	return p_analog->ActiveAdcChannelCount;
#else
	(void) p_analog;
	return 1U;
#endif
}

//static inline bool GetAnalogIsConversionRepeat(const AnalogN_Conversion_T * p_activeConversion)
//{
//#if !defined(CONFIG_ANALOG_ADC_HW_BUFFER)
//	if(p_activeConversion->MAP.P_VIRTUAL_CONVERSION->CHANNEL_COUNT > 1U)
//	{
//		return true;
//	}
//#elif defined(CONFIG_ANALOG_SW_CONTINOUS_CONVERSION)
//	return true;
//#endif
//}

//static inline void Analog_Deactivate(const Analog_T * p_analog)					{HAL_ADC_Deactivate(p_analog->CONFIG.P_HAL_ADC);}
//static inline void Analog_DisableInterrupt(const Analog_T * p_analog)			{HAL_ADC_DisableInterrupt(p_analog->CONFIG.P_HAL_ADC);}
//static inline void Analog_ClearConversionComplete(Analog_T * p_analog)			{HAL_ADC_ClearConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC);}
//static inline bool Analog_ReadConversionComplete(const Analog_T * p_analog)		{return HAL_ADC_ReadConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC);}
//static inline bool Analog_ReadConversionActive(const Analog_T * p_analog)		{return HAL_ADC_ReadConversionActiveFlag(p_analog->CONFIG.P_HAL_ADC);}


static inline bool _Analog_ReadIsActive(const Analog_T * p_analog)
{
	//case of aborted conversion?
	return (p_analog->p_ActiveConversion != 0U);
//	return (Analog_ReadConversionActive(p_analog) == true) || (Analog_ReadConversionComplete(p_analog) == true);
}



#include "Utility/Debug/Debug.h"

/*!
	 @brief Private capture results subroutine
 */
static inline void _Analog_CaptureAdcResults(Analog_T * p_analog, const Analog_ConversionAdc_T * p_adcConversion, const Analog_ConversionMap_T * p_adcMap)
{
	HAL_ADC_T * p_adc				= p_analog->CONFIG.P_HAL_ADC;
	uint8_t activeChannelCount 		= GetAnalogActiveAdcChannelCount(p_analog);

	const Analog_ConversionAdcChannel_T * p_adcChannels 			= p_adcConversion->P_CHANNELS;
	const Analog_ConversionVirtualChannel_T * p_virtualChannels 	= p_adcMap->P_VIRTUAL_CONVERSION->P_CHANNELS;
	Analog_ConversionOptions_T options 								= p_adcMap->P_VIRTUAL_CONVERSION->OPTIONS;
	analog_adcresult_t * p_resultsBuffer 							= p_adcMap->P_RESULTS_BUFFER;

	analog_adcresult_t result;
	analog_adcpin_t adcPin;
	uint8_t virtualIndex;
	analog_virtual_t virtualChannel;

	/*
	 * Should not need to boundary check on return. Read in the same way it was pushed
	 */


	if (options.CaptureLocalPeak == true)
	{
		if(p_analog->count == 0)
		{
			Debug_CaptureElapsed(5);
		}
		else if(p_analog->count == 1)
		{
			Debug_CaptureElapsed(6);
		}
		else if(p_analog->count == 2)
		{
			Debug_CaptureElapsed(7);
		}
		else if(p_analog->count == 3)
		{
			Debug_CaptureElapsed(8);
		}
		p_analog->count++;

		for (uint8_t iConversionIndex = p_analog->ActiveConversionIndex; iConversionIndex < p_analog->ActiveConversionIndex + activeChannelCount; iConversionIndex++) // 1U literal should optimize away for loop
		{
			adcPin 								= p_adcChannels[iConversionIndex].PIN;
			virtualIndex 						= p_adcChannels[iConversionIndex].VIRTUAL_INDEX;
			virtualChannel 						= p_virtualChannels[virtualIndex].CHANNEL;

			result = HAL_ADC_ReadResult(p_adc, adcPin);

			if (result > p_resultsBuffer[virtualChannel])
			{
				p_resultsBuffer[virtualChannel] = result;
			}
			else
			{
				p_analog->count++;
					Debug_CaptureElapsed(9);
				p_analog->IsLocalPeakFound = true; //try do not stop on peak
			}
		}
	}
	else
	{
		for (uint8_t iConversionIndex = p_analog->ActiveConversionIndex; iConversionIndex < p_analog->ActiveConversionIndex + activeChannelCount; iConversionIndex++) // 1U literal should optimize away for loop
		{
			adcPin 								= p_adcChannels[iConversionIndex].PIN;
			virtualIndex 						= p_adcChannels[iConversionIndex].VIRTUAL_INDEX;
			virtualChannel 						= p_virtualChannels[virtualIndex].CHANNEL;
			p_resultsBuffer[virtualChannel] 	= HAL_ADC_ReadResult(p_adc, adcPin);
		}
	}

}


static inline bool _Analog_CaptureResults(Analog_T * p_analog, const Analog_ConversionAdc_T * p_adcConversion, const Analog_ConversionMap_T * p_adcMap)
{
 	uint8_t completeChannelStartIndex 	= p_analog->ActiveConversionIndex;
	uint8_t completeChannelCount 		= GetAnalogActiveAdcChannelCount(p_analog);
	uint8_t remainingChannelCount;

	const Analog_ConversionAdcChannel_T * p_adcChannels 			= p_adcConversion->P_CHANNELS;
	const Analog_ConversionVirtualChannel_T * p_virtualChannels 	= p_adcMap->P_VIRTUAL_CONVERSION->P_CHANNELS;
	void * p_onCompleteContext 										= p_adcMap->P_ON_COMPLETE_CONTEXT;
	uint8_t virtualIndex;

	static uint32_t debug;

	bool isAllChannelsComplete;

	if((_Analog_ReadIsActive(p_analog) == true)) //debug only
	{
		HAL_ADC_ClearConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC);

		_Analog_CaptureAdcResults(p_analog, p_adcConversion, p_adcMap);
		p_analog->ActiveConversionIndex += completeChannelCount;

		/*
		 * Set up next conversion
		 */
		remainingChannelCount = p_adcConversion->CHANNEL_COUNT - p_analog->ActiveConversionIndex;

		if (remainingChannelCount > 0U)
		{
			SetAnalogActiveAdcChannelCount(p_analog, remainingChannelCount);
			/* Index updated, Start next group of channels in ActiveConversion  */
			_Analog_ActivateAdc(p_analog, p_adcConversion, p_adcMap);
			isAllChannelsComplete = false;
		}
		else
		{
			isAllChannelsComplete = true;
//			Analog_Deactivate(p_analog); do not deactivate for continuous conversion
		}

		/*
		 * _Analog_ProcChannelOnCompletes(p_analog);
		 *  OnComplete functions run after starting next set of channel to pipeline adc run
		 *  if the next conversion completes before OnComplete functions return, ADC ISR should queue, but cannot(should not) interrupt the on going ISR
		 */
		for (uint8_t iConversionIndex = completeChannelStartIndex; iConversionIndex < completeChannelStartIndex + completeChannelCount; iConversionIndex++)
		{
			virtualIndex = p_adcChannels[iConversionIndex].VIRTUAL_INDEX;
			if (p_virtualChannels[virtualIndex].ON_COMPLETE != 0U)
			{
				p_virtualChannels[virtualIndex].ON_COMPLETE(p_onCompleteContext);
			}
		}
	}
	else
	{
		debug++;
	}

	return (isAllChannelsComplete);
}



///*!
//	@brief	Capture ADC results, when conversion is complete.
//			Run in corresponding ADC ISR
//
//	ADC ISR should be higher priority than thread calling Analog_Activate()
// */
//static inline void Analog_CaptureResults_ISR(Analog_T * p_analog)
//{
//	Analog_Conversion_T * p_activeConversion;
//	Queue_PeekFront(&p_analog->ConversionQueue, &p_activeConversion); //read an address
//
//	const Analog_ConversionAdc_T * p_adcConversion 	= &p_activeConversion->ADC_CONVERSION;
//	const Analog_ConversionMap_T * p_adcMap 		= &p_activeConversion->MAP;
//	Analog_ConversionOptions_T options 		 		= p_adcMap->P_VIRTUAL_CONVERSION->OPTIONS;
//
//	Analog_OnComplete_T onConversionComplete  		= p_activeConversion->MAP.P_VIRTUAL_CONVERSION->ON_COMPLETE;
//	void * p_onConversionCompleteContext 			= p_activeConversion->MAP.P_ON_COMPLETE_CONTEXT;
//
//
//	if (_Analog_CaptureResults(p_analog, p_adcConversion, p_adcMap) == true) //all channels complete
//	{
//		if (onConversionComplete != 0U)
//		{
//			onConversionComplete(p_onConversionCompleteContext);
//		}
//
//		if (options.ContinuousConversion == false)
//		{
//			p_activeConversion = _Analog_ProcQueue(p_analog);
//			if(p_activeConversion != 0U)
//			{
//				_Analog_ActivateAdc(p_analog, &p_activeConversion->ADC_CONVERSION, &p_activeConversion->MAP);
//			}
//		}
//		else
//		{
//			#if !defined(CONFIG_ANALOG_ADC_HW_BUFFER)
//			if(p_activeConversion->MAP.P_VIRTUAL_CONVERSION->CHANNEL_COUNT > 1U)
//			{
//
//			}
//
//			#elif defined(CONFIG_ANALOG_SW_CONTINOUS_CONVERSION)
//			p_analog->ActiveConversionIndex = 0U;
//			_Analog_ActivateAdc(p_analog, &p_activeConversion->ADC_CONVERSION, &p_activeConversion->MAP);
//			#endif
//		}
//	}
//}



extern void Analog_Init(Analog_T * p_analog);

//extern void 					Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion);


#endif
