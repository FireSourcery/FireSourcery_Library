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

#ifdef CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED
	#include "System/Critical/Critical.h"
#elif defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED)
	extern inline void Critical_Enter(void);
	extern inline void Critical_Exit(void);
#elif defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)

#endif

#include <stdint.h>
#include <stdbool.h>


/*
 * Software side data storage format
 * User may implement app side buffer size larger than register size
 */
#ifdef CONFIG_ANALOG_ADC_RESULT_DATA_8BIT
	typedef uint8_t adcdata_t;
#elif defined(CONFIG_ANALOG_ADC_RESULT_DATA_16BIT)
	typedef uint16_t analog_adcdata_t;
#endif

/*
 * ADC Pin Channel
 * Pin id up to 32 bits. transient unless memory allocated for pin buffer. is this needed?
 */
#ifdef CONFIG_ANALOG_ADC_PIN_CHANNEL_UINT8
	typedef const uint8_t adcpin_t;
#elif defined(CONFIG_ANALOG_ADC_PIN_CHANNEL_UINT16)
	typedef const  uint16_t adcpin_t;
#elif defined(CONFIG_ANALOG_ADC_PIN_CHANNEL_UINT32)
	typedef const uint32_t analog_adcpin_t;
#endif


typedef uint8_t analog_channel_t;

//typedef enum
//{
//	ANALOG_STATUS_OK = 0,
//	ANALOG_STATUS_ERROR_A = 1,
//} Analog_Status_T;
//
//typedef enum
//{
//	ANALOG_REQUEST_REG_CONVERSION_COMPLETE,
//	ANALOG_REQUEST_REG_CONVERSION_ACTIVE,
//} Analog_Request_T;

/*
 * Config options passed to ADC
 */
typedef struct Analog_Config_Tag
{
	uint32_t UseConfig 					:1; /* config == zero, use default config */ //change check defualt first
	uint32_t UseHwTriggerPerChannel 	:1; /* Per Channel if both are set*/
	uint32_t UseHwTriggerPerConversion 	:1;
	uint32_t UseInterrupt 				:1;
	uint32_t UseDma 					:1;
/*
 * bool UseContinuousConversion;
 * uniform for future update
 */
} Analog_ConversionOptions_T;

typedef const struct Analog_ConversionChannel_Tag
{
	const analog_channel_t CHANNEL;
	void (*const ON_COMPLETE)(volatile void * p_userData); /* On each channel complete, runs first adc isr, depends on adc hw fifo length */
//	volatile void *p_OnCompleteUserData;
}  Analog_ConversionChannel_T;

/*!
	@brief Conversion is group of channels and config. "plugin" at runtime

	compile time defined const.
	processed sequentially (with or without hw queue)

	conversion share same config and same callback data
 */
typedef const struct Analog_Conversion_Tag
{
	const Analog_ConversionChannel_T * P_CHANNELS; /* Virtual Index */
	const uint8_t CHANNEL_COUNT;
	const Analog_ConversionOptions_T OPTIONS; //per conversion sample config. should this be pointer to allow run time modification of config, while struct is const,
	void (* const ON_COMPLETE)(volatile void * p_userData); 		/* On conversion group complete */
	//uint8_t Priority
	//uint8_t RepeatCount;
	//const Analog_VirtualChannel_T Channels[];	use/abuse last element array, for ease of definition? should follow return arg pointer or return value pattern?
}  Analog_Conversion_T;

typedef struct
{
//	const Analog_ConversionChannel_T * p_ConversionChannels;
//	const uint8_t ChannelCount;
	Analog_ConversionChannel_T Channel;
	analog_adcpin_t Pin;
	volatile void * p_OnCompleteContext;
}  Analog_ConversionChannelActive_T;

typedef struct
{
	const Analog_Conversion_T * p_Conversion;
	Analog_ConversionOptions_T Options;
	volatile void * p_OnCompleteContext;
}  Analog_ConversionActive_T; //conversion queue entry




typedef const struct
{
	const analog_adcpin_t PIN;
#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_FIXED)
	HAL_ADC_T * const P_HAL_ADC;
#endif
	//	volatile void *p_OnCompleteUserData;
//	volatile analog_adcdata_t * const P_RESULT;
} Analog_ChannelMapEntry_T;


typedef const struct
{
	union
	{
		HAL_ADC_T * P_HAL_ADC;						/*!< 1 ADC: pointer to ADC register map base address */
#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
		HAL_ADC_T (* const (* const PP_HAL_ADCS)); 	/*!< N ADC: pointer to array of pointers to ADC register map base addresses */
#endif
	};

#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	const uint8_t ADC_COUNT;
#endif
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	const uint8_t ADC_BUFFER_LENGTH;
#endif

//#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_FIXED)
//	//	volatile uint8_t * P_ACTIVE_PINS_BUFFER;
//#endif
	volatile analog_adcdata_t * const P_CHANNEL_RESULTS_BUFFER;
//	volatile analog_adcdata_t * const P_CHANNEL_SUM_BUFFER;
	const Analog_ChannelMapEntry_T * P_CHANNEL_MAP;
	const uint8_t CHANNEL_COUNT;

	//channel conversion offset
} Analog_Config_T;

/*
 * Analog_T scales 1 per concurrent conversion group
 * Most general case is N ADC count with M Buffer/Queue/FIFO length
 * Conversion channels must demux to to N ADCs for N ADC mode
 */
typedef struct
{
	const Analog_Config_T CONFIG;

	/*!
	 * MISRA violation.
	 *
	 * Use of unions. Share memory space.
	 *
	 * Rationale: Same as void *, simplify void * casting
	 * N_Adc used to disambiguate union.
	 *
	 * Content will be read in the same form it is written.
	 */
//	union
//	{
//		HAL_ADC_T * p_Adc;					/*!< 1 ADC: pointer to ADC register map base address */
//		HAL_ADC_T (* const (* pp_Adcs)); 	/*!< N ADC: pointer to array of pointers to ADC register map base addresses */
//	};

	/*
	 * N ADC use inheritance vs preprocessor compile time directive vs super class
	 *
	 * preprocessor compile time directive
	 * When active, all N ADC groups, included 1 ADC group, must use N version i.e loops. However there is less code redundancy
	 * When inactive compiler able to optimize away with proper function wrapping
	 *
	 * inheritance
	 * allows 1 adc to remain using single adc functions, n adc use n adc functions.
	 *
	 * super class
	 * is runtime polymorphism needed?
	 */

//#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
//	uint8_t AdcN_Count;  	/*!< Adc N Count */ 	/* count should be per instance define in case of multiple n adc groups with different adc count */
//#endif
//#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
//	uint8_t AdcM_Buffer; /*!< Adc M Hw Buffer Length */ 	/* use for hw queue implementation, run time define, in case buffer are of different sizes? use pointer to array */
//#endif


	/*
	 * Virtual Channel Config
	 */

	/*
	 * Channel maps using virtual channel index
	 *
	 * Multiple ADC can share 1 map. 1 Map per application. Each application requires a unique set of Physical/Virtual pins
	 * Map needs to be index per application. e.g. write index will not work with shared map startChannel(1)
	 *
	 * User provide:
	 */
	/*
	 const uint32_t ANALOG_CHANNEL_MAP[] =
	 {
		 [ANALOG_VIRTUAL_CHANNEL_1] 			= ADC_DRV_PIN_X,
		 [ANALOG_VIRTUAL_CHANNEL_2] 			= ADC_DRV_PIN_Y,
		 [ANALOG_VIRTUAL_CHANNEL_3] 			= ADC_DRV_PIN_Z,
		 [ANALOG_VIRTUAL_CHANNEL_4] 			= ADC_DRV_PIN_A,
	 };
	 */




//	const adcpin_t * p_VirtualChannelMapPins; /*!< Channel Pins array, virtual channel index. Translates virtual channels to ADC pin channels */
//#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_FIXED)
//	const uint8_t * p_VirtualChannelMapAdcs; 	/*! for case of fixed N ADCs Fixed channels */
//#endif
//	volatile adcdata_t * p_VirtualChannelResults; 	/*!< Persistent ADC results buffer, virtual channel index.  */
//	//volatile uint32_t * p_VirtualChannelResultsBuffer;	/*!< sum if multiple conversions are required */
//	uint8_t ChannelCount; /*!< VirtualChannelCount p_VirtualChannelResults and p_VirtualChannelMap length / boundary check */

	Queue_T ChannelQueue;

	Queue_T ConverisionQueue;

	//virtualized n_adc mode must wait for all adcs to finish before dequeue
//	const Analog_Conversion_T * volatile * pp_ConversionQueue;
//	uint8_t ConversionQueueLength;
//	volatile uint8_t ConversionQueueHead;
//	volatile uint8_t ConversionQueueTail;
	// callback data set per conversion

	Analog_Config_T DefaultConfig;	/*   Default config when conversion does not specify */
//	volatile Analog_Config_T ActiveConfig; //not needed unless it is checked on ADC return
	//	* pp_DefaultCallbackData

	/* need to copy channels only if n ADCs none multi muxed */
	const Analog_ConversionChannelActive_T * volatile p_ActiveConversionChannels; 	/*! Selected conversion group in process */
	volatile uint8_t ActiveChannelCount; 	/*! Hw fifo only. Number of active channels being processed by ADC */



	const Analog_ConversionActive_T * volatile p_ActiveConversion; 	/*! Selected conversion group in process */
	volatile uint8_t ActiveConversionCount;
	volatile uint8_t ActiveConversionChannelIndex; 				/*! Channel index of Selected conversion group */

#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	//#ifdef CONFIG_ANALOG_ADC_HW_CHANNELS_N_DEMUX
	volatile uint8_t ActiveChannelCount; 	/*! Number of active channels being processed by ADC */
	//#elif defined(CONFIG_ANALOG_ADC_HW_CHANNELS_FIXED)
//	volatile uint8_t * p_ActiveConversionChannelIndexes; 	 	/*! for case of fixed N ADCs without N demux, determine active channels in each adc */
//	volatile uint8_t ActiveConversionIteration; 				/*! for case of fixed N ADCs without N demux, determine active channels in each adc */
//	volatile uint8_t * p_ActiveChannelCounts; //track channel count or iterate channels, channels * n adc times.
//	uint8_t ActiveChannelMax; 		//store for run time optimization, preprocessor non single conversion case
	//#endif
#endif

	//	bool ActiveConversionComplete; //use status or oncomplete fp?

	//run time set overwrite
//	void (*volatile OnCompleteAdc)(void * userData); /* On each ADC ISR */
//	volatile void * p_OnCompleteUserData;

	//uint8_t RepeatCounter; count zero to sample repeat

	// can eliminate virtual channel conversion during return isr at the expense of memory space
	//	volatile adcpin_t *p_ActivePinChannelsBuffer; 			/*!< Temp Buffer to store translated ADC pin channels */
	//	volatile adcreg_t *p_ActivePinChannelResultsBuffer; 	/*!< Temp ADC results buffer, adc pin channel index*/

	// for more generalized case of round robin with unit size
	//uint8_t AdcHwBufferLengthRW; // RW_Length //read write unit size for "round robin" channels per adc
	//	uint8_t ActiveN;
	//	uint8_t ActiveM; //ActiveChannelCountAdcHwBuffer ; ActiveM
	//	uint8_t ActiveRemainder; // ActiveChannelCountAdcHwBufferRemainder ActiveR_HwRemainder

} Analog_T;




/*!
	@brief Fill 1 ADCs
 */
static inline uint8_t ActivateAdc_Single(Analog_T * p_analog, const Analog_ConversionChannel_T * p_channels, uint8_t activateChannelCount, Analog_ConversionOptions_T options)
{
	uint8_t channelCount = CalcAdcActiveChannelCountMax(p_analog, activateChannelCount);
	analog_channel_t channel;
	uint32_t pin;

	//deactivate if needed
	//perchannel is persw buffer
	if (options.UseHwTriggerPerChannel || options.UseHwTriggerPerConversion)
	{
		HAL_ADC_WriteHwTriggerState(p_analog->CONFIG.P_HAL_ADC, true);
	}
	else
	{
		HAL_ADC_WriteHwTriggerState(p_analog->CONFIG.P_HAL_ADC, false);
	}

	for (uint8_t iChannelIndex = 0U; iChannelIndex < channelCount; iChannelIndex++)
	{
		channel = p_channels[iChannelIndex].CHANNEL;
		pin = p_analog->CONFIG.P_CHANNEL_MAP->PIN[channel];
		/*
		 * iVirtualChannel should be less than p_analog->VirtualChannelMapLength
		 * No need to boundary check if compile time const
		 */
//			p_analog->p_MapChannelResults[iVirtualChannel] = 0U;
		if (iChannelIndex < channelCount - 1U)
		{
			HAL_ADC_WritePinSelect(p_analog->CONFIG.P_HAL_ADC, pin);
		}
		else /* (iChannel + iAdc == activeChannelCount - 1) */
		{
			HAL_ADC_WriteLast(p_analog->CONFIG.P_HAL_ADC, pin); /* enable interrupt of last ADC written */
			HAL_ADC_Activate(p_analog->CONFIG.P_HAL_ADC);
		}
	}

	return channelCount;
}


/*!
	@brief Fill N ADCs "round robin"
	@param[in] pp_adcMaps			read-only
	@param[in] p_virtualChannels 	offset by p_analog->ActiveConversionChannelIndex

	Only when channel can demux to any ADC

	Fill N ADCs "round robin" as follows:

	const uint8_t ANALOG_CONVERSION_CHANNELS[] =
	{
		ANALOG_VIRTUAL_CHANNEL_A, 		<- ADC0 Buffer0
		ANALOG_VIRTUAL_CHANNEL_B, 		<- ADC1 Buffer0
		ANALOG_VIRTUAL_CHANNEL_C, 		<- ADCN Buffer0
		ANALOG_VIRTUAL_CHANNEL_D,	 	<- ADC0 Buffer1
	};

	Compiler may able to optimize when arguments are constant literals
 */
static inline void ActivateAdc_NMultiMuxed
(
	Analog_T * p_analog,
	HAL_ADC_T (* const (* pp_adcMaps)),
	uint8_t nAdc,
	const Analog_VirtualChannel_T * p_virtualChannels,
	uint8_t channelCount,
	Analog_Config_T config
)
{
	Analog_VirtualChannel_T iVirtualChannel;

	for (uint8_t iAdc = 0U; iAdc < nAdc; iAdc++)
	{
		//deactivate if needed
		if (config.UseHwTriggerPerChannel || config.UseHwTriggerPerConversion)
		{
			HAL_ADC_WriteHwTriggerState(pp_adcMaps[iAdc], 1U);
		}
		else
		{
			HAL_ADC_WriteHwTriggerState(pp_adcMaps[iAdc], 0U);
		}
	}

	for (uint8_t iChannel = 0U; iChannel < channelCount; iChannel += nAdc)
	{
		for (uint8_t iAdc = 0U; iAdc < nAdc; iAdc++)
		{
			iVirtualChannel = p_virtualChannels[iChannel + iAdc];
			/*
			 * iVirtualChannel should be less than p_analog->VirtualChannelMapLength
			 * Boundary check if needed
			 * if (iVirtualChannel < p_analog->VirtualChannelMapLength)
			 */
//			p_analog->p_MapChannelResults[iVirtualChannel] = 0U;

			if (iChannel + iAdc < channelCount - 1U)
			{
				HAL_ADC_WritePinSelect(pp_adcMaps[iAdc], (uint32_t) p_analog->p_VirtualChannelMapPins[iVirtualChannel]);
			}
			else /* (iChannel + iAdc == activeChannelCount - 1) */
			{
				HAL_ADC_WriteLast(pp_adcMaps[iAdc], (uint32_t) p_analog->p_VirtualChannelMapPins[iVirtualChannel]); /* enable interrupt of last ADC written */
				HAL_ADC_Activate(pp_adcMaps[iAdc]);
				break; 		/* iChannel += nAdc should also break from outer loop */
			}

		}
	}
}





/*!
	@brief 	Start the selected segment channels
 	 	 	Share by public Analog_ActivateConversion, and Analog_CaptureResults
 */
static inline void ActivateAdcs
(
	Analog_T * p_analog,
	const Analog_VirtualChannel_T * p_virtualChannels,
	uint8_t channelCount,
	Analog_Config_T config
)
{
/* Channel Demux N ADC  */
#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
	(void)channelCount;
	ActivateAdc_NMultiMuxed(p_analog, &p_analog->CONFIG.P_HAL_ADC, 1U, 				p_virtualChannels, 1U, config);
//	ActivateAdc_Single(p_analog, p_virtualChannels, 1U);
#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	/* * Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
	ActivateAdc_NMultiMuxed(p_analog, &p_analog->CONFIG.P_HAL_ADC, 1U, 				p_virtualChannels, channelCount, config);
//	ActivateAdc_Single(p_analog, p_virtualChannels, activateChannelCount);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)
	/* Case N ADC 1 Buffer:  ActiveChannelCount <= N_ADC Count */
	ActivateAdc_NMultiMuxed(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count, 	p_virtualChannels, channelCount, config);
//	ActivateAdc_NFixed()
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
	ActivateAdc_NMultiMuxed(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count, 	p_virtualChannels, channelCount, config);
//	ActivateAdc_NFixed()
#endif
}

/*!
	@brief 	Get Active channel count called before ActivateConversion()

			Share by public Analog_ActivateConversion, and Analog_CaptureResults
 */
static inline uint8_t CalcAdcActiveChannelCountMax
(
	Analog_T * p_analog,
	uint8_t remainingChannelCount
)
{
	uint8_t maxChannelCount;

	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	(void)p_analog;
	(void)remainingChannelCount;
	maxChannelCount = 1U;
#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
	maxChannelCount = p_analog->AdcM_Buffer;
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)
	/* Case N ADC 1 Buffer:  ActiveChannelCount <= N_ADC Count */
	maxChannelCount = p_analog->AdcN_Count;
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
	maxChannelCount = p_analog->AdcN_Count * p_analog->AdcM_Buffer;
#endif

#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	if (remainingChannelCount < maxChannelCount)
	{
		maxChannelCount = remainingChannelCount;
	}
#endif

	return maxChannelCount;
}


static inline Analog_Config_T CalcAdcActiveConfig
(
	Analog_T * p_analog,
	Analog_Config_T conversionConfig,
	bool isFirstActivation
)
{
	Analog_Config_T config;

	if (conversionConfig.UseConfig == 1U)
	{
		config = conversionConfig;
	}
	else
	{
		config = p_analog->DefaultConfig;
	}

	if (isFirstActivation == false)
	{
		config.UseHwTriggerPerConversion = 0;
	}

	return config;
}


/*!
	 @brief Private capture results subroutine
	 	 For when all channels are multiplexed to each ADC

	 @param[in] p_adc			read-only
	 @param[in] p_virtualChannels 	offset by p_analog->ActiveConversionChannelIndex

	 Compiler may optimize when arguments are constant literals
 */
static inline void CaptureAdcResults_NMultiMuxed
(
	const Analog_T * p_analog,
	HAL_ADC_T (* const (* pp_adcMaps)),
	uint8_t nAdc,
	const Analog_VirtualChannel_T * p_virtualChannels,
	uint8_t activeChannelCount
)
{
	Analog_VirtualChannel_T iVirtualChannel;

	/* Read in the same way it was pushed */
	for (uint8_t iChannel = 0U; iChannel < activeChannelCount; iChannel += nAdc)
	{
		/* p_analog->p_ActiveConversion->ChannelCount > 1, on second loop, safe to dereference p_virtualChannels[index>0] */
		for (uint8_t iAdc = 0U; iAdc < nAdc; iAdc++)
		{
			if (iChannel + iAdc < activeChannelCount)
			{
				iVirtualChannel = p_virtualChannels[iChannel + iAdc];
				p_analog->p_VirtualChannelResults[iVirtualChannel] = HAL_ADC_ReadResult(pp_adcMaps[iAdc], (uint32_t) p_analog->p_VirtualChannelMapPins[iVirtualChannel]);
			}
		}
	}
}

static inline void CaptureAdcResults(const Analog_T * p_analog,	const Analog_VirtualChannel_T * p_virtualChannels)
{
#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
	CaptureAdcResults_NMultiMuxed(p_analog, &p_analog->CONFIG.P_HAL_ADC, 1U, p_virtualChannels, 1U);
#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel is in the same buffer */
	CaptureAdcResults_NMultiMuxed(p_analog, &p_analog->CONFIG.P_HAL_ADC, 1U, 	p_virtualChannels, p_analog->ActiveChannelCount);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)
	/* Case N ADC 1 Buffer: ActiveChannelCount <= N_ADC Count */
	CaptureAdcResults_NMultiMuxed(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count, p_virtualChannels, p_analog->ActiveChannelCount);
//	CaptureAdcResults_NFixed()
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
	CaptureAdcResults_NMultiMuxed(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count, p_virtualChannels, p_analog->ActiveChannelCount);
//	CaptureAdcResults_NFixed()
#endif
}

//bool DequeueAnalogConversion(Analog_T * p_analog)
//{
//

//}

/*!
	@brief	Capture ADC results, when conversion is complete.
			Run in corresponding ADC ISR

	ADC ISR should be higher priority than thread calling Analog_Activate()

	CompleteConversion
 */
static inline void Analog_CaptureResults_ISR(Analog_T * p_analog)
{
	const Analog_VirtualChannel_T * p_virtualChannels; //temp buffer argument

	const Analog_Conversion_T *  p_completedConversion;
	uint8_t completedChannelStartIndex;
	uint8_t completedChannelCount;
	uint8_t remainingChannelCount;

	uint8_t newActiveChannelCount;
	Analog_Config_T newConfig;

	static uint32_t debug;

	if((p_analog->p_ActiveConversion != 0U) && (Analog_ReadConversionComplete(p_analog) == true))
	{
		Analog_ClearConversionComplete(p_analog);

		/*
		 * Should not need to boundary check on return
		 * 	if (p_analog->ActiveConversionChannelIndex < p_analog->p_ActiveConversion->VirtualChannelMapLength)
		 */

		/*
		 * ADC ISR should be higher priority than calling thread
		 * If calling thread is higher priority than ADC ISR
		 * 	Critical_Enter();
		 */

		/* Convert from union form to pointer for uniform processing */
//		if(p_analog->p_ActiveConversion->ChannelCount > 1U)
//		{
			p_virtualChannels = &p_analog->p_ActiveConversion->p_VirtualChannels[p_analog->ActiveConversionChannelIndex];
//		}
//		else
//		{
//			p_virtualChannels = &p_analog->p_ActiveConversion->VirtualChannel;
//		}

		CaptureAdcResults(p_analog, p_virtualChannels);

		/*
		 * buffer current conversion for on complete at the end of routine.
		 */
		p_completedConversion = p_analog->p_ActiveConversion;
		completedChannelStartIndex = p_analog->ActiveConversionChannelIndex;
	#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
		completedChannelCount = 1U;
	#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
		completedChannelCount = p_analog->ActiveChannelCount;
	#endif

		/*
		 * Set up next conversion
		 */
		remainingChannelCount = p_analog->p_ActiveConversion->ChannelCount - p_analog->ActiveConversionChannelIndex - completedChannelCount;

		if (remainingChannelCount > 0U)
		{
			/* Continue Conversion: update index, Start next group of channels in selected p_ActiveConversion  */

			p_analog->ActiveConversionChannelIndex = p_analog->ActiveConversionChannelIndex + completedChannelCount;
			p_virtualChannels = &(p_analog->p_ActiveConversion->p_VirtualChannels[p_analog->ActiveConversionChannelIndex]);
			newActiveChannelCount = CalcAdcActiveChannelCountMax(p_analog, remainingChannelCount);
			newConfig = CalcAdcActiveConfig(p_analog, p_analog->p_ActiveConversion->Config, false);
		}
		else
		{
			/* Conversion Complete */
			p_analog->ActiveConversionChannelIndex = 0U;

			if (p_analog->ConversionQueueHead != p_analog->ConversionQueueTail)
			{
				/* Dequeue Conversion:  DequeueAnalogConversion */
				p_analog->p_ActiveConversion = p_analog->pp_ConversionQueue[p_analog->ConversionQueueHead];
				p_analog->ConversionQueueHead = (p_analog->ConversionQueueHead + 1U) % p_analog->ConversionQueueLength;

//				if(p_analog->p_ActiveConversion->ChannelCount > 1U)
//				{
					p_virtualChannels = p_analog->p_ActiveConversion->p_VirtualChannels;
//				}
//				else
//				{
//					p_virtualChannels = &p_analog->p_ActiveConversion->VirtualChannel;
//				}

				newActiveChannelCount = CalcAdcActiveChannelCountMax(p_analog, p_analog->p_ActiveConversion->ChannelCount);
				newConfig = CalcAdcActiveConfig(p_analog, p_analog->p_ActiveConversion->Config, true);
			}
			else
			{
				/* All Conversions Complete */
				p_analog->p_ActiveConversion = 0U;
				Analog_Dectivate(p_analog);
			}
		}

		if (p_analog->p_ActiveConversion != 0U)
		{
			#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
				p_analog->ActiveChannelCount = newActiveChannelCount;
			#endif
			ActivateAdc(p_analog, p_virtualChannels, newActiveChannelCount, newConfig);
		}

		/*	Critical_Exit(); */

		/*
		 * if the next conversion completes before OnComplete functions return, ADC ISR should queue, but cannot(should not) interrupt the on going ISR
		 */
	//	if (p_analog->p_ActiveConversion->OnAdcComplete != 0)
	//	{
	//		p_analog->p_ActiveConversion->OnAdcComplete(p_analog);
	//	}

		/*
		 *  OnComplete functions run after starting next set of channel to pipeline adc run
		 *  main context variables of initial conversion
		 */
		for (uint8_t index = completedChannelStartIndex; index < completedChannelStartIndex + completedChannelCount; index++)
		{
			if ((p_completedConversion->p_OnCompleteChannels != 0U))
			{
				if ((p_completedConversion->p_OnCompleteChannels[index] != 0U))
				{
					p_completedConversion->p_OnCompleteChannels[index](p_completedConversion->p_OnCompleteUserData);
				}
			}
		}

		if (remainingChannelCount == 0U)
		{
			if (p_completedConversion->OnCompleteConversion != 0U)
			{
				p_completedConversion->OnCompleteConversion(p_completedConversion->p_OnCompleteUserData);
			}
		}
	}
	else
	{
		debug++;
	}
}


///*!
//	@brief	Compiler may optimize when arguments are constant literals
// */
//static inline void PollAdcResults(Analog_T * p_analog, 	HAL_ADC_T (* const (* pp_adcMaps)), uint8_t nAdc)
//{
//	bool capture;
//
//	for (uint8_t iAdc = 0U; iAdc < nAdc; iAdc++)
//	{
//		capture = HAL_ADC_ReadConversionCompleteFlag(pp_adcMaps[iAdc]);
//
//		if (capture == false)
//		{
//			break;
//		}
//	}
//
//	if (capture == true)
//	{
//		Analog_CaptureResults(p_analog);
//	}
//}

/*!
	@brief	Capture ADC results, by polling status register, if ISR is unavailable
 */
static inline void Analog_PollCaptureResults(Analog_T * p_analog)
{
	if (HAL_ADC_ReadConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC))
	{
		Analog_CaptureResults(p_analog);
	}
}

/*!
	 @brief Read last data captured by Analog_Capture() 	 new data check?
 */
static inline analog_adcdata_t Analog_GetChannelResult(const Analog_T * p_analog, analog_channel_t channel)					{return (channel < p_analog->CONFIG.CHANNEL_COUNT) ? p_analog->CONFIG.P_CHANNEL_RESULTS_BUFFER[channel] : 0U;}
static inline volatile analog_adcdata_t * Analog_GetPtrChannelResult(const Analog_T * p_analog, analog_channel_t channel)	{return (channel < p_analog->CONFIG.CHANNEL_COUNT) ? &p_analog->CONFIG.P_CHANNEL_RESULTS_BUFFER[channel] : 0U;}
static inline void Analog_ResetChannelResult(Analog_T * p_analog, analog_channel_t channel)									{p_analog->CONFIG.P_CHANNEL_RESULTS_BUFFER[channel] = 0U;}
static inline analog_channel_t Analog_GetActiveChannelId(const Analog_T * p_analog)											{return p_analog->p_ActiveConversion->p_Conversion->P_CHANNELS[p_analog->ActiveConversionChannelIndex];}

static inline void Analog_Dectivate(const Analog_T * p_analog)				{HAL_ADC_Dectivate(p_analog->CONFIG.P_HAL_ADC);	HAL_ADC_DisableInterrupt(p_analog->CONFIG.P_HAL_ADC);}
static inline void Analog_DisableInterrupt(const Analog_T * p_analog)		{HAL_ADC_DisableInterrupt(p_analog->CONFIG.P_HAL_ADC);}
static inline bool Analog_ReadConversionActive(const Analog_T * p_analog)	{return (p_analog->p_ActiveConversion != 0U) || HAL_ADC_ReadConversionActiveFlag(p_analog->CONFIG.P_HAL_ADC);}
static inline bool Analog_ReadConversionComplete(const Analog_T * p_analog)	{return HAL_ADC_ReadConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC);}
static inline void Analog_ClearConversionComplete(Analog_T * p_analog)		{HAL_ADC_ClearConversionCompleteFlag(p_analog->CONFIG.P_HAL_ADC);}

//extern void Analog_Init
//(
//	Analog_T * p_analog,
//);

//extern void 					Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion);
//extern adcdata_t 				Analog_ReadChannel(const Analog_T * p_analog, Analog_VirtualChannel_T channel);
//extern volatile adcdata_t * 	Analog_GetPtrChannelResult(const Analog_T * p_analog, Analog_VirtualChannel_T channel);
//extern void 					Analog_ResetChannelResult(Analog_T * p_analog, Analog_VirtualChannel_T channel);


#endif

//typedef struct Analog_Result_Tag
//{
//	uint16_t NewDataFlag :1;
//	uint16_t DataValue	 :15;
//} Analog_Result_T;


