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

#include "HAL_ADC.h"

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
 * Compile time define size of data size. Should not need to change at run time.
 * transient in function call, can use uint32. is this needed?
 */
#ifdef CONFIG_ANALOG_ADC_RESULT_REGISTER_8BIT
	typedef volatile const uint8_t adcreg_t;
#elif defined(CONFIG_ANALOG_ADC_RESULT_REGISTER_16BIT)
	typedef volatile const uint16_t adcreg_t;
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
	typedef const uint32_t adcpin_t;
#endif



/*
 * Software side data storage format
 * User may implement app side buffer size larger than register size
 */
#ifdef CONFIG_ANALOG_ADC_RESULT_DATA_8BIT
	typedef uint8_t analog_t;
#elif defined(CONFIG_ANALOG_ADC_RESULT_DATA_16BIT)
	typedef uint16_t analog_t;
#endif

/*
 * Virtual Channel
 *
 * Virtual indexes for ADC pin IDs. Scales 1 per application or pin set.
 */
#ifdef CONFIG_ANALOG_VIRUTAL_CHANNEL_ENUM_USER_DEFINED
/*
 * User provide enum typedef
 */
/*
typedef enum
{
	ANALOG_VIRTUAL_CHANNEL_X,
	ANALOG_VIRTUAL_CHANNEL_Y,
	ANALOG_VIRTUAL_CHANNEL_Z,
	ANALOG_VIRTUAL_CHANNEL_A,
	ANALOG_VIRTUAL_CHANNEL_B,
	ANALOG_VIRTUAL_CHANNEL_C,
}
Analog_VirtualChannel_T;
*/
/*
 * Multi applications use overlapping enum values, or use uint8_t
 */
/*
typedef enum
{
	APP1_ANALOG_VIRTUAL_CHANNEL_X = 1,
	APP1_ANALOG_VIRTUAL_CHANNEL_Y = 2,
	APP1_ANALOG_VIRTUAL_CHANNEL_Z = 3,
	APP2_ANALOG_VIRTUAL_CHANNEL_A = 1,
	APP2_ANALOG_VIRTUAL_CHANNEL_B = 2,
	APP2_ANALOG_VIRTUAL_CHANNEL_C = 3,
}
Analog_VirtualChannel_T;
*/
#elif defined(CONFIG_ANALOG_VIRUTAL_CHANNEL_UINT8)
	typedef uint8_t Analog_VirtualChannel_T;
#endif

typedef enum
{
	ANALOG_STATUS_OK = 0,
	ANALOG_STATUS_ERROR_A = 1,
} Analog_Status_T;

typedef enum
{
	ANALOG_REQUEST_REG_CONVERSION_COMPLETE,
	ANALOG_REQUEST_REG_CONVERSION_ACTIVE,
} Analog_Request_T;

/*
 * Config options passed to ADC
 */
typedef struct Analog_Config_Tag
{
	uint32_t UseConfig 					:1; /* config == zero, use default config */
//	uint32_t IsActive 					:1;
	uint32_t UseHwTriggerPerChannel 	:1;
	uint32_t UseHwTriggerPerConversion 	:1;
	uint32_t UseInterrrupt 				:1;
	uint32_t UseDma 					:1;
/*
 * bool UseContinuousConversion;
 * uniform for future update
 */
} Analog_Config_T;

/*!
	@brief Conversion is group of channels and config. "plugin" at runtime

	compile time defined const.
	processed sequentially (with or without hw queue)
 */
typedef const struct Analog_Conversion_Tag
{
	/*!
	 * MISRA violation.
	 *
	 * Use of unions. Share memory space.
	 *
	 * Rationale: allows clearer definition of Channel within struct for cases of single Channel
	 * ChannelCount used to disambiguate union.
	 *
	 * Content will be read in the same form it is written.
	 */
	union
	{
		const 	Analog_VirtualChannel_T * p_VirtualChannels;
				Analog_VirtualChannel_T VirtualChannel;
	};

	uint8_t ChannelCount;
	Analog_Config_T Config; //per conversion sample config. should this be pointer to allow run time modification of config, while struct is const,

	/*
	 * On complete scales per conversion, allows different channel on complete function depending on conversion group
	 */
	void (*(*p_OnCompleteChannels))(volatile void * p_userData); 	/* On each channel complete, runs first adc isr, depends on adc hw fifo length */
	void (*OnCompleteConversion)(volatile void * p_userData); 		/* On conversion group complete */
	volatile void *p_OnCompleteUserData;

	//uint8_t Priority
	//uint8_t RepeatCount;
	//const Analog_VirtualChannel_T Channels[];	use/abuse last element array, for ease of definition? should follow return arg pointer or return value pattern?
}  Analog_Conversion_T;



/*
 * Analog_T scales 1 per concurrent conversion group
 * Most general case is N ADC count with M Buffer/Queue/FIFO length
 * Conversion channels must demux to to N ADCs for N ADC mode
 */
typedef struct
{
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
	union
	{
		HAL_ADC_T * p_Adc;					/*!< 1 ADC: pointer to ADC register map base address */
		HAL_ADC_T (* const (* pp_Adcs)); 	/*!< N ADC: pointer to array of pointers to ADC register map base addresses */
	};

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

#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	uint8_t AdcN_Count;  	/*!< Adc N Count */ 	/* count should be runtime define in case of multiple n adc groups with different adc count */
#endif
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	uint8_t AdcM_Buffer; /*!< Adc M Hw Buffer Length */ 	/* use for hw queue implementation, run time define, in case buffer are of different sizes? */
#endif


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

	const adcpin_t * p_MapChannelPins; /*!< Channel Pins array, virtual channel index. Translates virtual channels to ADC pin channels */
#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_FIXED)
	const uint8_t * p_MapChannelAdcs; 	/*! for case of fixed N ADCs Fixed channels */
#endif

	volatile analog_t * p_MapChannelResults; 	/*!< Persistent ADC results buffer, virtual channel index.  */
	//volatile uint32_t * p_ChannelSumBuffer;	/*!< sum if multiple conversions are required */

	uint8_t ChannelCount; /*!< VirtualChannelCount p_VirtualChannelResults and p_VirtualChannelMap length / boundary check */

	const Analog_Conversion_T * volatile * pp_ConversionQueue;
	uint8_t ConversionQueueLength;
	volatile uint8_t ConversionQueueHead;
	volatile uint8_t ConversionQueueTail;

	Analog_Config_T DefaultConfig;	/*   Default config when conversion does not specify */
//	volatile Analog_Config_T ActiveConfig; //not needed unless it is checked on ADC return

	const Analog_Conversion_T * volatile p_ActiveConversion; 	/*! Selected conversion group in process */
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



	/*
	 * Supports id active channel during on complete
	 */
	//	Analog_VirtualChannel_T ActiveChannel;

	//	bool ActiveConversionComplete; //if entire conversion is complete
	//	void (*OnCompleteAdc)(void * userData); 		/* On each ADC ISR */
	//	volatile void *p_OnCompleteUserData;

	//uint8_t RepeatCounter; count zero to sample repeat

	// can eliminate virtual channel conversion during isr at the expense of memory space
	//	volatile adcpin_t *p_ActivePinChannelsBuffer; 			/*!< Temp Buffer to store translated ADC pin channels */
	//	volatile adcreg_t *p_ActivePinChannelResultsBuffer; 	/*!< Temp ADC results buffer, adc pin channel index*/

	// for more generalized case of round robin with unit size
	//uint8_t AdcHwBufferLengthRW; // RW_Length //read write unit size for "round robin" channels per adc
	//	uint8_t ActiveN;
	//	uint8_t ActiveM; //ActiveChannelCountAdcHwBuffer ; ActiveM
	//	uint8_t ActiveRemainder; // ActiveChannelCountAdcHwBufferRemainder ActiveR_HwRemainder

	/* todo: For multithreaded queuing of samples, must implement software queue:
	 * 	linked list - poor run time efficiency
	 * 	ring buffer - less efficient q in front.
	 * 	2 ring buffer design, 2 level priority queues
	 */
} Analog_T;


extern void Analog_Init
(
	Analog_T * p_analog,
	const void * p_adcMap,
#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	uint8_t nAdc,
#endif
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	uint8_t mHwBufferLength,
#endif
	uint8_t virtualChannelCount,
	const adcpin_t * p_virtualChannelMapPins,
#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_FIXED)
	const uint8_t * p_virtualChannelMapAdcs,
	//	volatile uint8_t * p_activeChannelIndexesBuffer,
#endif
	volatile analog_t * p_virtualChannelMapResultsBuffer,
	const Analog_Conversion_T * volatile * pp_conversionQueue,
	uint8_t conversionQueueLength
);


/*!
	 @brief Read last data captured by Analog_Capture_IO()
 */
static inline analog_t Analog_ReadChannel(const Analog_T * p_analog, Analog_VirtualChannel_T channel)
{
	analog_t result;

	if (channel < p_analog->ChannelCount)
	{
		result = p_analog->p_MapChannelResults[(uint8_t)channel];
	}
	else
	{
		result = 0U;
	}

	return result;
	/* implement new data check
	 * set p_analog->p_VirtualChannelMapResults[(uint8_t)channel].NewDataFlag = false
	 * disable isr if writing completion flag
	 */
}



/*!
	 @brief
 */
static inline void Analog_ResetChannelResult(Analog_T * p_analog, Analog_VirtualChannel_T channel)
{
	p_analog->p_MapChannelResults[channel] = 0U;
}

/*!
	 @brief
 */
static inline void Analog_Dectivate(const Analog_T * p_analog)
{
#if  defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	HAL_ADC_Dectivate(p_analog->p_Adc);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
	{
		HAL_ADC_Dectivate(p_analog->pp_Adcs[iAdc]);
	}
#endif
}

static inline void Analog_DisableInterrupt(const Analog_T * p_analog)
{
#if  defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	HAL_ADC_DisableInterrupt(p_analog->p_Adc);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
	{
		HAL_ADC_DisableInterrupt(p_analog->pp_Adcs[iAdc]);
	}
#endif
}

static inline bool Analog_ReadConversionActive(const Analog_T * p_analog)
{
	bool isActive;

#if  defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	isActive = HAL_ADC_ReadConversionActiveFlag(p_analog->p_Adc);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	isActive = false;
	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
	{
		isActive |= HAL_ADC_ReadConversionActiveFlag(p_analog->pp_Adcs[iAdc]);
	}
#endif

	return isActive;
}

extern void 				Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion);
extern analog_t 			Analog_ReadChannel(const Analog_T * p_analog, Analog_VirtualChannel_T channel);
extern volatile analog_t * 	Analog_GetPtrChannelResult(const Analog_T * p_analog, Analog_VirtualChannel_T channel);
extern void 				Analog_ResetChannelResult(Analog_T * p_analog, Analog_VirtualChannel_T channel);


#endif

/*
 * can use module internal compile time memory allocation if all concurrent apps share 1 channel maps
 */
//#define CONFIG_ANALOG_CHANNEL_COUNT
//extern const adcpin_t		ANALOG_CHANNEL_PINS[CONFIG_ANALOG_CHANNEL_COUNT]; 		/*!< User define */
//extern const uint8_t		ANALOG_CHANNEL_ADCS[CONFIG_ANALOG_CHANNEL_COUNT];
//extern volatile analog_t 	Analog_ChannelResults[CONFIG_ANALOG_CHANNEL_COUNT];		/*!< Measure Result Buffer */
//extern volatile uint32_t 	Analog_ChannelSumBuffer[CONFIG_ANALOG_CHANNEL_COUNT];			/*!< sum if multiple samples are required */


//typedef struct Analog_Result_Tag
//{
//	uint16_t NewDataFlag :1;
//	uint16_t DataValue	 :15;
//} Analog_Result_T;


