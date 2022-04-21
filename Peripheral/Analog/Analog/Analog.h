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
	typedef volatile uint8_t analog_adcresult_t;
#elif defined(CONFIG_ANALOG_ADC_RESULT_UINT16)
	typedef volatile uint16_t analog_adcresult_t;
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

typedef uint8_t analog_channel_t;

typedef void (* Analog_Callback_T)(void * p_context);

typedef enum
{
	ANALOG_QUEUE_TYPE_CHANNEL,
	ANALOG_QUEUE_TYPE_OPTIONS,
}
Analog_QueueType_T;

typedef const struct
{
	Analog_QueueType_T 			TYPE;
	const analog_channel_t 		CHANNEL; 		/* index into results buffer */
	const Analog_Callback_T 	ON_COMPLETE; 	/* On each channel complete, runs first adc isr, depends on adc hw fifo length */

	void * const P_CALLBACK_CONTEXT;
	volatile analog_adcresult_t * const P_RESULTS_BUFFER;	 /*!< Persistent ADC results buffer, virtual channel index.  */
	const analog_adcpin_t PIN;
}
Analog_Conversion_T;

#define CONFIG_ANALOG_CONVERSION(Channel, OnComplete, p_CallbackContext, p_Results, PinId) \
{															\
	.TYPE 					= ANALOG_QUEUE_TYPE_CHANNEL, 	\
	.CHANNEL 				= Channel,						\
	.ON_COMPLETE 			= OnComplete,					\
	.P_CALLBACK_CONTEXT 	= p_CallbackContext,			\
	.P_RESULTS_BUFFER 		= p_Results,					\
	.PIN 					= PinId,						\
}

/*
 * Config options
 */
typedef struct
{
	uint32_t IsValid				:1U; /* use options */
	uint32_t HwTriggerConversion 	:1U;
//	uint32_t ContinuousConversion 	:1U;
//	uint32_t CaptureLocalPeak 		:1U;	/* for now, conversion stops on 1 local peak in channel set, user must also set ContinuousConversion */

//	uint32_t HwAveraging
//	uint32_t HwTriggerChannel 		:1U; /* Per Hw buffer complete. Per Channel if both are set*/
//	uint32_t Interrupt 				:1U;
//	uint32_t Dma 					:1U;

	//uint8_t Priority
}
Analog_OptionsFlags_T;

typedef const struct
{
	Analog_QueueType_T TYPE;
	const Analog_OptionsFlags_T 	FLAGS;
	const Analog_Callback_T 		ON_OPTIONS;
}
Analog_Options_T;

/*
 * cast to this type to determine which item type first
 */
typedef const struct
{
	union
	{
		const Analog_QueueType_T 	TYPE;
		const Analog_Conversion_T 	CONVERISON;
		const Analog_Options_T 		OPTIONS;
	};
}
Analog_QueueItem_T;

typedef const struct
{
	HAL_ADC_T * const P_HAL_ADC; 	/*!< 1 ADC: pointer to ADC register map base address */
#ifdef CONFIG_ANALOG_ADC_HW_FIFO_ENABLE
	uint8_t ADC_FIFO_LENGTH;
#endif
}
Analog_Config_T;

/*
 * Analog_T per ADC
 */
typedef struct
{
	const Analog_Config_T CONFIG;

	Queue_T ConversionQueue; /* item type (Analog_Conversion_T *) or (Analog_ConversionOptions_T *) */

	/*
	 * 	Active conversion as firsdt conversion or Enqueue Front would need to shift all items in queue
	 *  also use as adc active flag
	 */
//	const Analog_ConversionChannel_T * volatile p_ActiveConversion; 	/*! Queue unit type Selected conversion group in process */
// 	volatile uint8_t ActiveConversionIndex; 	/*! Channel index of Selected conversion group */
//	uint8_t ActiveConversionCount; //share the same options
#ifdef CONFIG_ANALOG_ADC_HW_FIFO_ENABLE
	uint8_t ActiveAdcChannelCount; 				/*! Hw fifo only. Number of active channels being processed by ADC */
#endif

//	bool IsLocalPeakFound;

//	volatile uint32_t debug;
	//common setting for all following conversion
	//Applicable next conversion, multithread must enter critical before changes
//	Analog_ConversionOptions_T Options; 	/* Default config when conversion does not specify */
//	bool WaitForTrigger; //can change to enum to support various trigger sources
//	Analog_TriggerMode_T TriggerMode;
	//if 2 high priority conversion, activate sequentially, do not overwrite the previous
// 	bool OverwriteEnable;
// 	uint8_t ActiveConversionPriority;
}
Analog_T;

//#ifdef CONFIG_ANALOG_ADC_HW_FIFO_ENABLE
//#define ANALOG_CONFIG_ADC_BUFFER_LENGTH(BufferLength) \
//	.ADC_BUFFER_LENGTH = BufferLength,
//#else
//#define ANALOG_CONFIG_ADC_BUFFER_LENGTH(BufferLength)
//#endif

/*
* Queue buffer length in units
*/
#define ANALOG_CONFIG(p_HalAdc, p_ConversionBuffer, ConversionQueueLength) \
{															\
	.CONFIG =												\
	{														\
		.P_HAL_ADC = p_HalAdc,								\
	},														\
	.ConversionQueue = QUEUE_CONFIG(p_ConversionBuffer, ConversionQueueLength, sizeof(Analog_QueueItem_T *), 0U),	\
}

/*!
	 @brief Private capture results subroutine
 */
static inline void _Analog_CaptureAdcResults(Analog_T * p_analog, Analog_Conversion_T * p_activeConversion)	//, const Analog_ConversionAdc_T * p_adcConversion, const Analog_ConversionVirtualMap_T * p_adcMap)
{
//	HAL_ADC_T * p_adc				= p_analog->CONFIG.P_HAL_ADC;
//	uint8_t activeChannelCount 		= GetAnalogActiveAdcChannelCount(p_analog);
//
//	const Analog_ConversionAdcChannel_T * p_adcChannels 			= p_adcConversion->P_CHANNELS;
//	const Analog_ConversionVirtualChannel_T * p_virtualChannels 	= p_adcMap->P_VIRTUAL_CONVERSION->P_CHANNELS;
//	Analog_Options_T options 								= p_adcMap->P_VIRTUAL_CONVERSION->OPTIONS;
//	analog_adcresult_t * p_resultsBuffer 							= p_adcMap->P_RESULTS_BUFFER;
//
//	analog_adcresult_t result;

//	uint8_t virtualIndex;
	analog_channel_t virtualChannel;
	analog_adcpin_t adcPin;

	/*
	 * Should not need to boundary check on return. Read in the same way it was pushed
	 */
//	if (options.CaptureLocalPeak == true)
//	{
//		for (uint8_t iConversionIndex = p_analog->ActiveConversionIndex; iConversionIndex < p_analog->ActiveConversionIndex + activeChannelCount; iConversionIndex++) // 1U literal should optimize away for loop
//		{
//			adcPin 								= p_adcChannels[iConversionIndex].PIN;
//			virtualIndex 						= p_adcChannels[iConversionIndex].VIRTUAL_INDEX;
//			virtualChannel 						= p_virtualChannels[virtualIndex].CHANNEL;
//
//			result = HAL_ADC_ReadResult(p_adc, adcPin);
//
//			if (result > p_resultsBuffer[virtualChannel])
//			{
//				p_resultsBuffer[virtualChannel] = result;
//			}
//			else
//			{
//				p_analog->IsLocalPeakFound = true;
//			}
//		}
//	}
//	else
	{
#ifdef CONFIG_ANALOG_ADC_HW_FIFO_ENABLE
		for (uint8_t iConversionIndex = p_analog->ActiveConversionIndex; iConversionIndex < p_analog->ActiveConversionIndex + activeChannelCount; iConversionIndex++) // 1U literal should optimize away for loop
		{
			adcPin 								= p_adcChannels[iConversionIndex].PIN;
			virtualIndex 						= p_adcChannels[iConversionIndex].VIRTUAL_INDEX;
			virtualChannel 						= p_virtualChannels[virtualIndex].CHANNEL;
			p_resultsBuffer[virtualChannel] 	= HAL_ADC_ReadResult(p_adc, adcPin);
		}
#else
		virtualChannel = p_activeConversion->CHANNEL;
		adcPin = p_activeConversion->PIN;

		p_activeConversion->P_RESULTS_BUFFER[virtualChannel] = HAL_ADC_ReadResult(p_analog->CONFIG.P_HAL_ADC, adcPin);
#endif
	}

}

static inline bool _Analog_CaptureResults(Analog_T * p_analog)//, const Analog_ConversionAdc_T * p_adcConversion, const Analog_ConversionVirtualMap_T * p_adcMap)
{
// 	uint8_t completeChannelStartIndex 	= p_analog->ActiveConversionIndex;
//	uint8_t completeChannelCount 		= GetAnalogActiveAdcChannelCount(p_analog);
	uint8_t remainingChannelCount;
	Analog_Conversion_T * p_completedConversion;
	Analog_Conversion_T * p_nextConversion;

//	const Analog_ConversionAdcChannel_T * p_adcChannels 			= p_adcConversion->P_CHANNELS;
//	const Analog_ConversionVirtualChannel_T * p_virtualChannels 	= p_adcMap->P_VIRTUAL_CONVERSION->P_CHANNELS;
//	void * p_onCompleteContext 										= p_adcMap->P_CALLBACK_CONTEXT;
//	uint8_t virtualIndex;

	bool isAllChannelsComplete;

	HAL_ADC_ClearConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC);


//		_Analog_CaptureAdcResults(p_analog); //, p_adcConversion, p_adcMap);
//		p_analog->ActiveConversionIndex += completeChannelCount;
//
//		/*
//		 * Set up next conversion
//		 */
//		remainingChannelCount = p_analog->ActiveConversionCount - p_analog->ActiveConversionIndex;
//
//		if (remainingChannelCount > 0U)
//		{
////			SetAnalogActiveAdcChannelCount(p_analog, remainingChannelCount);
//			/* Index updated, Start next group of channels in ActiveConversion  */
//			_Analog_ActivateAdc(p_analog);//, p_adcConversion, p_adcMap);
//			isAllChannelsComplete = false;
//		}
//		else
//		{
//			isAllChannelsComplete = true;
////			Analog_Deactivate(p_analog); do not deactivate here in case of continuous conversion
//		}

		/*
		 * _Analog_ProcChannelOnCompletes(p_analog);
		 *  OnComplete functions run after starting next set of channel to pipeline adc run
		 *  if the next conversion completes before OnComplete functions return, ADC ISR should queue, but cannot(should not) interrupt the on going ISR
		 */
//		for (uint8_t iConversionIndex = completeChannelStartIndex; iConversionIndex < completeChannelStartIndex + completeChannelCount; iConversionIndex++)
//		{
////			virtualIndex = p_adcChannels[iConversionIndex].VIRTUAL_INDEX;
////			if (p_virtualChannels[virtualIndex].ON_COMPLETE != 0U)
////			{
////				p_virtualChannels[virtualIndex].ON_COMPLETE(p_onCompleteContext);
////			}
//
//
//		}

		Queue_Dequeue(&p_analog->ConversionQueue, &p_completedConversion);

		//sw support continous use peek
		//		Queue_Peek(&p_analog->ConversionQueue, &p_completedConversion);

		_Analog_CaptureAdcResults(p_analog, p_completedConversion); //, p_adcConversion, p_adcMap);
//		p_analog->ActiveConversionIndex += 1U;

		/*
		 * Set up next conversion
		 */
//		remainingChannelCount = p_analog->ActiveConversionCount - p_analog->ActiveConversionIndex;

//		if (remainingChannelCount > 0U)
//		{
//			/* Index updated, Start next group of channels in ActiveConversion - shared options  */
//			_Analog_ActivateAdc(p_analog);//, p_adcConversion, p_adcMap);
//			isAllChannelsComplete = false;
//		}
//		else
//		{
			/*   Dequeue Next Conversion New Options */
			_Analog_ProcQueue(p_analog);

			isAllChannelsComplete = true;
//		}

		if (p_completedConversion->ON_COMPLETE != 0U)
		{
			p_completedConversion->ON_COMPLETE(p_completedConversion->P_CALLBACK_CONTEXT);
		}

	return (isAllChannelsComplete);
}

/*!
	@brief	Capture ADC results, when conversion is complete.
			Run in corresponding ADC ISR

	ADC ISR should be higher priority than thread calling Analog_Activate()
 */
static inline void Analog_CaptureResults_ISR(Analog_T * p_analog)
{
//	Analog_Conversion_T * p_activeConversion;
//	Queue_PeekFront(&p_analog->ConversionQueue, &p_activeConversion); //read an address

//	const Analog_ConversionAdc_T * p_adcConversion 	= &p_activeConversion->ADC_CONVERSION;
//	const Analog_ConversionMap_T * p_adcMap 		= &p_activeConversion->MAP;
//	Analog_ConversionOptions_T options 		 		= p_adcMap->P_VIRTUAL_CONVERSION->OPTIONS;
//
//	Analog_OnComplete_T onConversionComplete  		= p_activeConversion->MAP.P_VIRTUAL_CONVERSION->ON_COMPLETE;
//	void * p_onConversionCompleteContext 			= p_activeConversion->MAP.P_CALLBACK_CONTEXT;

	if (_Analog_CaptureResults(p_analog) == true) //all channels complete
	{
//		if (onConversionComplete != 0U)
//		{
//			onConversionComplete(p_onConversionCompleteContext);
//		}

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


//#if defined(CONFIG_ANALOG_ADC_HW_FIFO_DISABLE)
//
//#elif defined(CONFIG_ANALOG_SW_CONTINOUS_CONVERSION)
//			p_analog->ActiveConversionIndex = 0U;
//			_Analog_ActivateAdc(p_analog, &p_activeConversion->ADC_CONVERSION, &p_activeConversion->MAP);
//#endif
//		}
	}
}

///*!
//	@brief 	Get Active channel count called before ActivateConversion()
// */
//static inline uint8_t SetAnalogActiveAdcChannelCount(Analog_T * p_analog, uint8_t remainingChannelCount)
//{
//#if defined(CONFIG_ANALOG_ADC_HW_FIFO_ENABLE)
//	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
//	return (remainingChannelCount < p_analog->CONFIG.ADC_BUFFER_LENGTH) ? remainingChannelCount : p_analog->CONFIG.ADC_BUFFER_LENGTH;
//	p_analog->ActiveAdcChannelCount = remainingChannelCount
//#else
//	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
//	(void) p_analog;
//	(void) remainingChannelCount;
//	return 1U;
//#endif
//}
//
//static inline uint8_t GetAnalogActiveAdcChannelCount(const Analog_T * p_analog)
//{
//#if defined(CONFIG_ANALOG_ADC_HW_FIFO_ENABLE)
//	return p_analog->ActiveAdcChannelCount;
//#else
//	(void) p_analog;
//	return 1U;
//#endif
//}

//static inline bool GetAnalogIsConversionRepeat(const AnalogN_Conversion_T * p_activeConversion)
//{
//#if !defined(CONFIG_ANALOG_ADC_HW_FIFO_ENABLE)
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
//static inline bool Analog_ReadIsAdcConversionComplete(const Analog_T * p_analog)		{return HAL_ADC_ReadConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC);}
//static inline bool Analog_ReadIsAdcConversionActive(const Analog_T * p_analog)		{return HAL_ADC_ReadConversionActiveFlag(p_analog->CONFIG.P_HAL_ADC);}
//extern void _Analog_ActivateAdc(const Analog_T * p_analog, Analog_ConversionChannel_T * p_conversion);
//extern void _Analog_ProcQueue(Analog_T * p_analog);

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

#elif (defined(CONFIG_ANALOG_SINGLE_THREADED))
	/*
	 * Single threaded calling of Activate.
	 * Single threaded case, and calling thread is lower priority than ADC ISR, may implement ADC_DisableInterrupt instead of Critical_Enter global disable interrupt
	 * If calling thread is lower priority than ADC ISR, ADC ISR may occur after Conversion setup data is written by lower priority thread.
	 *
	 * In single threaded calling of Activate and calling thread priority is higher than ADC ISR, Activate will run to completion, overwriting the active conversion, Disable adc IRQ is not needed, however ADC ISR will still need Critical_Enter
	 */
	 //  use global critical if disable adc interrupts aborts active conversion
	HAL_ADC_DisableInterrupt(p_analog->CONFIG.P_HAL_ADC);
#endif
}

static inline void _Analog_ExitCritical(Analog_T * p_analog)
{
#if (defined(CONFIG_ANALOG_MULTITHREADED) || defined(CONFIG_ANALOG_CRITICAL_USE_GLOBAL))
	Critical_Exit();
#elif  defined(CONFIG_ANALOG_SINGLE_THREADED)
	HAL_ADC_EnableInterrupt(p_analog->CONFIG.P_HAL_ADC);
#endif
}

//static inline bool Analog_ReadIsActive(const Analog_T * p_analog)		{return ((Analog_ReadIsAdcConversionComplete(p_analog) == true) || (Analog_ReadIsAdcConversionActive(p_analog) == true));}
static inline bool _Analog_ReadIsActive(const Analog_T * p_analog)
{
	//case of aborted conversion?
//	return (p_analog->p_ActiveConversion != 0U);
//	return (p_analog->ActiveConversionCount > 0U);
	return ((HAL_ADC_ReadConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC) == true) || (HAL_ADC_ReadConversionActiveFlag(p_analog->CONFIG.P_HAL_ADC) == true));
}

extern void Analog_Init(Analog_T * p_analog);
extern void Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion);

#endif


//static inline void AnalogN_CaptureResults_ISR(AnalogN_T * p_analogn, uint8_t analogId)
//{
//	Analog_T * p_analogI = &p_analogn->CONFIG.P_ANALOGS[analogId];
////	AnalogN_Conversion_T * p_activeConversion = p_analogI->p_ActiveConversion; /* casting void pointer */
////	Queue_PeekFront(&p_analogI->ConversionQueue, &p_activeConversion); //read an address
//
////	const Analog_ConversionAdc_T * p_adcConversion 	= &p_activeConversion->P_ADC_CONVERSIONS[analogId];
////	const Analog_ConversionVirtualMap_T * p_map 			= &p_activeConversion->MAP;
////	AnalogN_AdcFlags_T * p_signalComplete 			= p_activeConversion->P_SIGNAL_BUFFER;
//////	Analog_Options_T options 		 		= p_map->P_VIRTUAL_CONVERSION->OPTIONS;
////	Analog_OnComplete_T onConversionComplete  		= p_map->P_VIRTUAL_CONVERSION->ON_COMPLETE;
////	void * p_onConversionCompleteContext 			= p_map->P_ON_COMPLETE_CONTEXT;
//
////	volatile static uint32_t debug = 0;
//
////	if((_Analog_GetIsActive(p_analogI) == true)) //debug only
////	{
//
//
//	if (_Analog_CaptureResults(p_analogI) == true) //all channels complete
//	{
//		//Do not run if is capture local peak and not peak found
////		if((((options.CaptureLocalPeak == true) && (p_analogI->IsLocalPeakFound == false)) == false))
////		{
////			p_signalComplete->AdcFlags &= ~(1UL << analogId); //debug only
////
////			if (onConversionComplete != 0U)
////			{
////				// only run oncomplete if conversion has completed on all adcs
////	//			p_signalComplete->AdcFlags &= ~(1UL << analogId);
////				if (p_signalComplete->AdcFlags == 0U)
////				{
////					onConversionComplete(p_onConversionCompleteContext);
////				}
////			}
////		}
//
//		//todo check repeat function
////		if((options.ContinuousConversion == true) && (((options.CaptureLocalPeak == true) && (p_analogI->IsLocalPeakFound == true)) == false))
////		{
////#if !defined(CONFIG_ANALOG_ADC_HW_FIFO_ENABLE)
////			if(p_activeConversion->MAP.P_VIRTUAL_CONVERSION->CHANNEL_COUNT > 1U)
////			{
////				_Analog_ActivateConversion(p_analogI, p_adcConversion, p_map);
////			}
////			else
////			{
////				p_analogI->ActiveConversionIndex = 0U; //auto reactivate, still need to reset index
////			}
////#elif defined(CONFIG_ANALOG_SW_CONTINUOUS_CONVERSION)
////			_Analog_ActivateConversion(p_analogI, p_adcConversion, p_map);
////#endif
////		}
////		else //local peak found
////		{
////
////			if(_Analog_DequeueConversion(p_analogI) == true)
////			{
////				/* p_analogI->p_ActiveConversion updated */
////				p_activeConversion = p_analogI->p_ActiveConversion;
////				_Analog_ActivateConversion(p_analogI, &p_activeConversion->P_ADC_CONVERSIONS[analogId], &p_activeConversion->MAP);
////			}
////			else
////			{
//////			Analog_Dectivate(p_analogI);
////			}
////		}
//	}
////
////	}
////	else
////	{
////		debug++;
////	}
//}
