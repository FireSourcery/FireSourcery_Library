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
    		Persistent setting delegated to outer module with scope of hardware.
	@version V0
*/
/******************************************************************************/
/******************************************************************************/
/*!
	@page misra MISRA-C:2012 violations

	@section [global]
	Violates MISRA 2012 Advisory Rule 19.2, The union keyword should not be used
	Rationale: Allows clearer simpler definition of Channels within struct for cases of single Channel Sample
	ChannelCount used to disambiguate union
 */
/******************************************************************************/
#ifndef ANALOG_H
#define ANALOG_H

#include "HAL_Analog.h"
#include "Config.h"

#include "System/Queue/Queue.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * Software side data storage format
 * User may implement app side buffer size larger than register size
 */
#ifdef CONFIG_ANALOG_ADC_RESULT_DATA_8BIT
	typedef uint8_t analog_adcdata_t;
#elif defined(CONFIG_ANALOG_ADC_RESULT_DATA_16BIT)
	typedef uint16_t analog_adcdata_t;
#endif

/*
 * ADC Pin Channel
 * Pin id up to 32 bits. transient unless memory allocated for pin buffer. is this needed?
 */
#ifdef CONFIG_ANALOG_ADC_PIN_CHANNEL_UINT8
	typedef uint8_t adcpin_t;
#elif defined(CONFIG_ANALOG_ADC_PIN_CHANNEL_UINT16)
	typedef uint16_t adcpin_t;
#elif defined(CONFIG_ANALOG_ADC_PIN_CHANNEL_UINT32)
	typedef uint32_t analog_adcpin_t;
#endif

typedef uint8_t analog_channel_t;

/*
 * Config options passed to ADC
 * uniform for future update
 */
typedef struct
{
	uint32_t UseConfig 					:1; /* config == zero, use default config */ //change check defualt first
	uint32_t UseHwTriggerPerChannel 	:1; /* Per Hw buffer complete. Per Channel if both are set*/
	uint32_t UseHwTriggerPerConversion 	:1;
//	uint32_t UseInterrupt 				:1;
//	uint32_t UseDma 					:1;
//	uint32_t UseContinuousConversion 	:1;
}
Analog_ConversionOptions_T;


/*
 * Virtual Channel
 * Analog Channel
 * ADC Pin
 */
typedef const struct Analog_ConversionChannel_Tag
{
	const uint8_t CHANNEL; //app channel or analog channel
	void (*const ON_COMPLETE)(volatile void * p_userData); /* On each channel complete, runs first adc isr, depends on adc hw fifo length */
}
Analog_ConversionChannel_T;

/*!
	@brief Conversion is group of channels and options.
	processed sequentially (with or without hw fifo)
	conversion share same options and same callback
 */
typedef const struct Analog_Conversion_Tag
{
	const Analog_ConversionChannel_T * P_CHANNELS; /* Virtual Index */
	const uint8_t CHANNEL_COUNT;
	const Analog_ConversionOptions_T OPTIONS;
	void (* const ON_COMPLETE)(volatile void * p_userData); 		/* On conversion group complete */

	//uint8_t Priority
	//uint8_t RepeatCount;
	//const Analog_VirtualChannel_T Channels[];	use/abuse last element array, for ease of definition? should follow return arg pointer or return value pattern?
}
Analog_Conversion_T;

typedef const struct
{
	/* Virtual Channel Config
	 * Channel maps using virtual channel index
	 * User provide:
	 */
	/*
	 const uint32_t ANALOG_CHANNEL_PINS_MAP[] =
	 {
		 [ANALOG_VIRTUAL_CHANNEL_1] 			= ADC_DRV_PIN_X,
		 [ANALOG_VIRTUAL_CHANNEL_2] 			= ADC_DRV_PIN_Y,
		 [ANALOG_VIRTUAL_CHANNEL_3] 			= ADC_DRV_PIN_Z,
		 [ANALOG_VIRTUAL_CHANNEL_4] 			= ADC_DRV_PIN_A,
	 };
	 */
	const analog_adcpin_t * const P_PINS_MAP;				/*!< Channel Pins array, virtual channel index. Translates virtual channels to ADC pin channels */
	volatile analog_adcdata_t * const P_RESULTS_BUFFER;		/*!< Persistent ADC results buffer, virtual channel index.  */
	const uint8_t CHANNEL_COUNT;							/*!< VirtualChannelCount p_VirtualChannelResults and p_VirtualChannelMap length / boundary check */

}
Analog_ChannelMap_T;

typedef const struct
{
	const Analog_ChannelMap_T  CHANNEL_MAP;	//decide convert to pins

	const analog_adcpin_t * const P_PIN_MAP;					/*!< Channel Pins array, virtual channel index. Translates virtual channels to ADC pin channels */
	volatile analog_adcdata_t * const P_RESULTS_BUFFER;		/*!< Persistent ADC results buffer, virtual channel index.  */
	const uint8_t CHANNEL_COUNT;							/*!< VirtualChannelCount p_VirtualChannelResults and p_VirtualChannelMap length / boundary check */


	const analog_channel_t * P_CONVERSION_CHANNEL_MAP;	//decide convert to channel
//	uint8_t CHANNEL_COUNT; //user ensure conversion channels are within mapped limits
	void * P_ON_COMPLETE_DATA;
}
Analog_ConversionContext_T;

typedef struct
{
	const Analog_Conversion_T * volatile p_Conversion;
	const Analog_ConversionContext_T * volatile p_Context;
	//	Analog_ConversionOptions_T Options;
}
Analog_ConversionActive_T; //conversion queue entry


////track channel buffer, or only queue acceptable channels
typedef struct
{

//	const uint8_t ChannelCount;
	const Analog_ConversionChannel_T * p_Channels;
//	analog_adcpin_t Pin;
	const Analog_Conversion_T * volatile p_Conversion;
	const Analog_ConversionContext_T * p_Context;
// can eliminate virtual channel conversion during return isr at the expense of memory space
//	volatile uint8_t * P_ACTIVE_PINS_BUFFER;					/*!< Temp Buffer to store translated ADC pin channels */
//	volatile analog_adcdata_t * const P_CHANNEL_SUM_BUFFER; 	/*!< sum if multiple conversions are required */
}
Analog_ConversionChannelActive_T;

/*
 * ProcVirtualToPinChannels
 */
//static inline bool ProcChannelPinBufferMap(Analog_T * p_analog,  Analog_VirtualChannel_T * p_virtualChannels, uint8_t channelCount)
//{
//	uint8_t iVirtualChannel;
//	for (uint8_t index = 0; index < channelCount; index++)
//	{
//		iVirtualChannel = (uint8_t)(p_virtualChannels[index]);
//
//		if (iVirtualChannel< p_analog->VirtualChannelCount) // check for invalid pin channel??
//		{
//			p_analog->p_PinChannelsBuffer[index] = p_analog->p_VirtualChannelMap[iVirtualChannel];
//			p_analog->p_VirtualChannelResults[iVirtualChannel] = 0;
//			// settings to reset before conversion?
//		}
//	}
//}
//

typedef const struct
{
	HAL_ADC_T * P_HAL_ADC; /*!< 1 ADC: pointer to ADC register map base address */
#if defined(CONFIG_ANALOG_ADC_HW_BUFFER)
	const uint8_t ADC_BUFFER_LENGTH;
#endif
//	Analog_ConversionContext_T CONTEXT_DEFAULT;

	/* Virtual Channel Config
	 * Channel maps using virtual channel index
	 * User provide:
	 */
	/*
	 const uint32_t ANALOG_CHANNEL_PINS_MAP[] =
	 {
		 [ANALOG_VIRTUAL_CHANNEL_1] 			= ADC_DRV_PIN_X,
		 [ANALOG_VIRTUAL_CHANNEL_2] 			= ADC_DRV_PIN_Y,
		 [ANALOG_VIRTUAL_CHANNEL_3] 			= ADC_DRV_PIN_Z,
		 [ANALOG_VIRTUAL_CHANNEL_4] 			= ADC_DRV_PIN_A,
	 };
	 */
	const analog_adcpin_t * const P_CHANNEL_PINS_MAP;		/*!< Channel Pins array, virtual channel index. Translates virtual channels to ADC pin channels */
	volatile analog_adcdata_t * const P_RESULTS_BUFFER;		/*!< Persistent ADC results buffer, virtual channel index.  */
	const uint8_t CHANNEL_COUNT;							/*!< VirtualChannelCount p_VirtualChannelResults and p_VirtualChannelMap length / boundary check */
}
Analog_Config_T;

/*
 * Analog_T per ADC
 */
typedef struct
{
	const Analog_Config_T CONFIG;
//	Queue_T ChannelQueue;
	Queue_T ConversionQueue;

	Analog_ConversionOptions_T OptionsDefault; 	/* Default config when conversion does not specify */

	/* Set for Adc Return */
	// Saved to Queue
	volatile Analog_ConversionActive_T ActiveConversion; 	/*! Selected conversion group in process */

	//channel index and count do not need to enqueue
	volatile uint8_t ActiveChannelIndex; 				/*! Channel index of Selected conversion group */
	volatile uint8_t ActiveChannelIndexOffset;
#if defined(CONFIG_ANALOG_ADC_HW_BUFFER)
	volatile uint8_t ActiveChannelCount; 				/*! Hw fifo only. Number of active channels being processed by ADC */
#endif

	//uint8_t RepeatCounter; count zero to sample repeat
}
Analog_T;

#define ANALOG_QUEUE_UNIT_SIZE (sizeof(Analog_ConversionActive_T))

#define ANALOG_CONFIG_QUEUE ( ) 	\
{									\
	.ConversionQueue.CONFIG.UNIT_SIZE = ANALOG_QUEUE_UNIT_SIZE,	\
}

/*!
	@brief 	Get Active channel count called before ActivateConversion()
 */
static inline uint8_t ProcAnalogActivateChannelCount(Analog_T * p_analog, uint8_t activateChannelCount)
{
#if defined(CONFIG_ANALOG_ADC_HW_BUFFER)
	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
	return (activateChannelCount < p_analog->CONFIG.ADC_BUFFER_LENGTH) ? activateChannelCount : p_analog->CONFIG.ADC_BUFFER_LENGTH;
	p_analog->ActiveChannelCount = activateChannelCount
#else
	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
	(void) p_analog;
	(void) activateChannelCount;
	return 1U;
#endif
}

static inline uint8_t GetAnalogCompleteChannelCount(Analog_T * p_analog)
{
#if defined(CONFIG_ANALOG_ADC_HW_BUFFER)
	return p_analog->ActiveChannelCount;
#else
	(void) p_analog;
	return 1U;
#endif
}

static inline Analog_ConversionOptions_T CalcAnalogActiveOptions(Analog_T * p_analog, Analog_ConversionOptions_T options)
{
	Analog_ConversionOptions_T newOptions;

	newOptions = (options.UseConfig == 1U) ?  options : p_analog->OptionsDefault;

	if (p_analog->ActiveChannelIndex > 0U)
	{
		newOptions.UseHwTriggerPerConversion = 0U;
	}

	return newOptions;
}



/*!
	@brief Fill 1 ADCs
 */
static inline void ActivateAnalogAdc
(
	Analog_T * p_analog,
	const Analog_ConversionChannel_T * p_channels,
	uint8_t activateChannelCount,
	const analog_adcpin_t * p_channelPinsMap,
	const analog_channel_t * p_virtualChannelMap,
	Analog_ConversionOptions_T options
)
{
	uint8_t iChannelIndex = 0U;
	uint8_t virtualChannel; //app channel
	analog_channel_t analogChannel;
	analog_adcpin_t pin;

	if (options.UseHwTriggerPerChannel || options.UseHwTriggerPerConversion)
	{
		HAL_ADC_WriteHwTriggerState(p_analog->CONFIG.P_HAL_ADC, true);
	}
	else
	{
		HAL_ADC_WriteHwTriggerState(p_analog->CONFIG.P_HAL_ADC, false);
	}

//#if defined(CONFIG_ANALOG_ADC_HW_BUFFER) // 1U literal should optimize away for loop
	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
	for (iChannelIndex = 0U; iChannelIndex < activateChannelCount - 1U; iChannelIndex++)
	{
		/*
		 * skip channel greater than ChannelMapLength
		 * use as invalid conversion as indicator for skipping
		 */
		analogChannel = p_channels[iChannelIndex].CHANNEL;
		pin = p_channelPinsMap[analogChannel];

//		if (pin != invalid pinid)
		{
	//		p_analog->p_MapChannelResults[channel] = 0U;
			HAL_ADC_WritePinSelect(p_analog->CONFIG.P_HAL_ADC, pin);
		}
 	}
//#endif

	analogChannel 	= p_channels[iChannelIndex].CHANNEL;
	pin 			= p_channelPinsMap[analogChannel];
	HAL_ADC_WriteLast(p_analog->CONFIG.P_HAL_ADC, pin); /* for shared registers, enable interrupt of last ADC written */
	HAL_ADC_Activate(p_analog->CONFIG.P_HAL_ADC);

	if (p_virtualChannelMap != 0U)
	{
		virtualChannel	 	= p_channels[iChannelIndex].CHANNEL;
		analogChannel 		= p_virtualChannelMap[virtualChannel];

		if(analogChannel < p_analog->CONFIG.CHANNEL_COUNT)
		{
			pin 			= p_channelPinsMap[analogChannel];
			iChannelIndex += 1U;
		}
	}
	else
	{
		virtualChannel 		= p_channels[iChannelIndex].CHANNEL;
		pin 				= p_virtualChannelMap[virtualChannel];
		iChannelIndex += 1U;
	}

	if(iChannelIndex > 0U)
	{
		HAL_ADC_WriteLast(p_analog->CONFIG.P_HAL_ADC, pin); /* for shared registers, enable interrupt of last ADC written */
		HAL_ADC_Activate(p_analog->CONFIG.P_HAL_ADC);
	}
}

static void ActivateAnalogConversionThis(Analog_T * p_analog)
{
	uint8_t activateChannelCount = ProcAnalogActivateChannelCount(p_analog, p_analog->ActiveConversion.p_Conversion->CHANNEL_COUNT - p_analog->ActiveChannelIndex);
	Analog_ConversionOptions_T options = CalcAnalogActiveOptions(p_analog, p_analog->ActiveConversion.p_Conversion->OPTIONS);


	//channelbuffer

	ActivateAnalogAdc
	(
		p_analog,
		&p_analog->ActiveConversion.p_Conversion->P_CHANNELS[p_analog->ActiveChannelIndex],
		activateChannelCount,
		p_analog->ActiveConversion.p_Context->CHANNEL_MAP.P_PINS,
		options
	);
}

static inline void ActivateAnalogConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
	p_analog->ActiveConversion.p_Context = 0U;
	p_analog->ActiveConversion.p_Conversion = p_conversion;
	ActivateAnalogConversionThis(p_analog);
}

static inline void ActivateAnalogConversion_Context(Analog_T * p_analog, const Analog_Conversion_T * p_conversion, const Analog_ConversionContext_T * p_context)
{
	p_analog->ActiveConversion.p_Context = p_context;
	p_analog->ActiveConversion.p_Conversion = p_conversion;
	ActivateAnalogConversionThis(p_analog);
}

/*!
	 @brief Private capture results subroutine
	 @param[in] p_virtualChannels 	offset by p_analog->ActiveChannelIndex
	 Compiler may optimize when arguments are constant literals
 */
static inline void CaptureAnalogAdcResults(const Analog_T * p_analog, volatile analog_adcdata_t * p_channelResults, const Analog_ConversionChannel_T * p_conversionChannels, uint8_t completeChannelCount, const analog_adcpin_t * p_channelPinsMap)
{
	analog_channel_t channel;
	analog_adcpin_t pin;

	/* Read in the same way it was pushed */
	/*
	 * Should not need to boundary check on return
	 * 	if (p_analog->ActiveChannelIndex < p_analog->p_ActiveConversion.VirtualChannelMapLength)
	 */
	for (uint8_t iChannelIndex = 0U; iChannelIndex < completeChannelCount; iChannelIndex++) // 1U literal should optimize away for loop
	{
		channel 	= p_conversionChannels[iChannelIndex].CHANNEL;
		pin 		= p_channelPinsMap[channel];
		p_channelResults[channel] = HAL_ADC_ReadResult(p_analog->CONFIG.P_HAL_ADC, (uint32_t)pin);
 	}
}

static inline void CaptureAnalogConversionResults(const Analog_T * p_analog)
{
	uint8_t completeChannelCount = GetAnalogCompleteChannelCount(p_analog);
	CaptureAnalogAdcResults
	(
		p_analog,
		p_analog->ActiveConversion.p_Context->CHANNEL_MAP.P_RESULTS_BUFFER,
		&p_analog->ActiveConversion.p_Conversion->P_CHANNELS[p_analog->ActiveChannelIndex],
		completeChannelCount,
		p_analog->ActiveConversion.p_Context->CHANNEL_MAP.P_PINS
	);

}

static inline void Analog_Dectivate(const Analog_T * p_analog)				{HAL_ADC_Dectivate(p_analog->CONFIG.P_HAL_ADC);	HAL_ADC_DisableInterrupt(p_analog->CONFIG.P_HAL_ADC);}
static inline void Analog_DisableInterrupt(const Analog_T * p_analog)		{HAL_ADC_DisableInterrupt(p_analog->CONFIG.P_HAL_ADC);}
static inline bool Analog_ReadConversionActive(const Analog_T * p_analog)	{return (p_analog->ActiveConversion.p_Conversion != 0U) || HAL_ADC_ReadConversionActiveFlag(p_analog->CONFIG.P_HAL_ADC);}
static inline bool Analog_ReadConversionComplete(const Analog_T * p_analog)	{return HAL_ADC_ReadConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC);}
static inline void Analog_ClearConversionComplete(Analog_T * p_analog)		{HAL_ADC_ClearConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC);}

/*!
	@brief	Capture ADC results, when conversion is complete.
			Run in corresponding ADC ISR

	ADC ISR should be higher priority than thread calling Analog_Activate()
 */
static inline void Analog_CaptureResults_ISR(Analog_T * p_analog)
{
	const Analog_Conversion_T * p_completeConversion;
	const Analog_ConversionContext_T * p_completeContext;
	uint8_t completeChannelStartIndex;
	uint8_t completeChannelCount;
	uint8_t remainingChannelCount;
	static uint32_t debug;

	if((p_analog->ActiveConversion.p_Conversion != 0U) && (Analog_ReadConversionComplete(p_analog) == true))
	{
		Analog_ClearConversionComplete(p_analog);

		/*
		 * Save current conversion for on complete at the end of routine.
		 */
		p_completeConversion 		= p_analog->ActiveConversion.p_Conversion;
		p_completeContext 			= p_analog->ActiveConversion.p_Context;
		completeChannelStartIndex 	= p_analog->ActiveChannelIndex;
		completeChannelCount 		= GetAnalogCompleteChannelCount(p_analog);

		CaptureAnalogConversionResults(p_analog);
		p_analog->ActiveChannelIndex += completeChannelCount;
		/*
		 * Set up next conversion
		 */
		remainingChannelCount = p_analog->ActiveConversion.p_Conversion->CHANNEL_COUNT - p_analog->ActiveChannelIndex;

		if (remainingChannelCount > 0U)
		{
			/* Continue Conversion: index updated, Start next group of channels in ActiveConversion  */
			ActivateAnalogConversionThis(p_analog);
		}
		else	/* Conversion Complete */
		{
			p_analog->ActiveChannelIndex = 0U;
			if (Queue_Deqeue(&p_analog->ConversionQueue, &p_analog->ActiveConversion) == true)	/* Dequeue Next Conversion: */
			{
				ActivateAnalogConversionThis(p_analog);
			}
			else	/* All Conversions Complete */
			{
				p_analog->ActiveConversion.p_Conversion = 0U;
				Analog_Dectivate(p_analog);
			}
		}

		/*
		 *  OnComplete functions run after starting next set of channel to pipeline adc run
		 *  if the next conversion completes before OnComplete functions return, ADC ISR should queue, but cannot(should not) interrupt the on going ISR
		 */

		for (uint8_t index = completeChannelStartIndex; index < completeChannelStartIndex + completeChannelCount; index++)
		{
			if ((p_completeConversion->P_CHANNELS[index].ON_COMPLETE != 0U))
			{
				p_completeConversion->P_CHANNELS[index].ON_COMPLETE(p_completeContext->P_ON_COMPLETE_DATA);
			}
		}

		if (remainingChannelCount == 0U)
		{
			if (p_completeConversion->ON_COMPLETE != 0U)
			{
				p_completeConversion->ON_COMPLETE(p_completeContext->P_ON_COMPLETE_DATA);
			}
		}
	}
	else
	{
		debug++;
	}
}

/*!
	@brief	Capture ADC results, by polling status register, if ISR is unavailable
 */
static inline void Analog_PollCaptureResults(Analog_T * p_analog)
{
	if (HAL_ADC_ReadConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC))
	{
		Analog_CaptureResults_ISR(p_analog);
	}
}

/*!
	 @brief Read last data captured by Analog_Capture() 	 new data check?
 */
//static inline analog_adcdata_t Analog_GetChannelResult(const Analog_T * p_analog, analog_channel_t channel)					{return (channel < p_analog->CONFIG.CHANNEL_COUNT) ? p_analog->CONFIG.P_CHANNEL_RESULTS_BUFFER[channel] : 0U;}
//static inline volatile analog_adcdata_t * Analog_GetPtrChannelResult(const Analog_T * p_analog, analog_channel_t channel)	{return (channel < p_analog->CONFIG.CHANNEL_COUNT) ? &p_analog->CONFIG.P_CHANNEL_RESULTS_BUFFER[channel] : 0U;}
//static inline void Analog_ResetChannelResult(Analog_T * p_analog, analog_channel_t channel)									{p_analog->CONFIG.P_CHANNEL_RESULTS_BUFFER[channel] = 0U;}
//static inline analog_channel_t Analog_GetActiveChannelId(const Analog_T * p_analog)											{return p_analog->ActiveConversion.p_Conversion->P_CHANNELS[p_analog->ActiveChannelIndex];}
//
//static inline analog_adcdata_t Analog_GetChannelResult_Context(const Analog_T * p_analog, analog_channel_t channel)					{return (channel < p_analog->CONFIG.CHANNEL_COUNT) ? p_analog->CONFIG.P_CHANNEL_RESULTS_BUFFER[channel] : 0U;}
//static inline volatile analog_adcdata_t * Analog_GetPtrChannelResult_Context(const Analog_T * p_analog, analog_channel_t channel)	{return (channel < p_analog->CONFIG.CHANNEL_COUNT) ? &p_analog->CONFIG.P_CHANNEL_RESULTS_BUFFER[channel] : 0U;}
//static inline void Analog_ResetChannelResult_Context(Analog_T * p_analog, analog_channel_t channel)									{p_analog->CONFIG.P_CHANNEL_RESULTS_BUFFER[channel] = 0U;}

//extern void Analog_Init
//(
//	Analog_T * p_analog,
//);

//extern void 					Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion);
//extern adcdata_t 				Analog_ReadChannel(const Analog_T * p_analog, Analog_VirtualChannel_T channel);
//extern volatile adcdata_t * 	Analog_GetPtrChannelResult(const Analog_T * p_analog, Analog_VirtualChannel_T channel);
//extern void 					Analog_ResetChannelResult(Analog_T * p_analog, Analog_VirtualChannel_T channel);

#endif
