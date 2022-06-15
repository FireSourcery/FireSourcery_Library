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

#include "Utility/Ring/Ring.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_ANALOG_CRITICAL_LIBRARY_ENABLE
#include "System/Critical/Critical.h"
#elif defined(CONFIG_ANALOG_CRITICAL_DISABLE)

#endif

/* Software side data storage */
#ifdef CONFIG_ANALOG_ADC_RESULT_UINT8
typedef volatile uint8_t analog_adcresult_t;
#elif defined(CONFIG_ANALOG_ADC_RESULT_UINT16)
typedef volatile uint16_t analog_adcresult_t;
#endif

/* ADC Pin Channel - May or may not need 32 bits */
#ifdef CONFIG_ANALOG_ADC_PIN_UINT8
typedef uint8_t analog_adcpin_t;
#elif defined(CONFIG_ANALOG_ADC_PIN_UINT32)
typedef uint32_t analog_adcpin_t;
#endif

typedef uint8_t analog_channel_t; /* Virtual Channel Index */

typedef void (*Analog_Callback_T)(void * p_context);

typedef enum Analog_QueueType_Tag
{
	ANALOG_QUEUE_TYPE_CHANNEL,
	ANALOG_QUEUE_TYPE_OPTIONS,
}
Analog_QueueType_T;

typedef const struct Analog_Conversion_Tag
{
	/* Defined by module */
	Analog_QueueType_T 			TYPE;
	const analog_channel_t 		CHANNEL; 		/* Index into results buffer */
	const Analog_Callback_T 	ON_COMPLETE; 	/* On channel complete */

	/* Defined by main */
	void * const P_CALLBACK_CONTEXT;
	volatile analog_adcresult_t * const P_RESULTS_BUFFER;	 /*!< Persistent ADC results buffer, virtual channel index.  */
	const analog_adcpin_t PIN;
}
Analog_Conversion_T;

#define ANALOG_CONVERSION_INIT(Channel, OnComplete, p_CallbackContext, p_Results, PinId) 	\
{																							\
	.TYPE 					= ANALOG_QUEUE_TYPE_CHANNEL, 									\
	.CHANNEL 				= Channel,														\
	.ON_COMPLETE 			= OnComplete,													\
	.P_CALLBACK_CONTEXT 	= p_CallbackContext,											\
	.P_RESULTS_BUFFER 		= p_Results,													\
	.PIN 					= PinId,														\
}

typedef struct Analog_OptionsFlags_Tag
{
	uint32_t HwTriggerConversion : 1U;
	//	uint32_t ContinuousConversion 	:1U;
	//	uint32_t CaptureLocalPeak 		:1U; /* for now, conversion stops on 1 local peak in channel set, user must also set ContinuousConversion */
	//	uint32_t HwAveraging
	//	uint32_t HwTriggerChannel 		:1U; /* Per Hw buffer complete. Per Channel if both are set*/
	//	uint32_t Interrupt 				:1U;
	//	uint32_t Dma 					:1U;
	//	uint8_t Priority
}
Analog_OptionsFlags_T;

typedef const struct Analog_Options_Tag
{
	const Analog_QueueType_T TYPE;
	const Analog_OptionsFlags_T FLAGS;
	const Analog_Callback_T ON_OPTIONS; /* On options set */
	void * const P_CALLBACK_CONTEXT;
}
Analog_Options_T;

/*
	Cast to this type to determine which item type first
*/
typedef const union Analog_QueueItem_Tag
{
	const Analog_QueueType_T 	TYPE;
	const Analog_Conversion_T 	CONVERISON;
	const Analog_Options_T 		OPTIONS;
}
Analog_QueueItem_T;

typedef const struct Analog_Config_Tag
{
	HAL_Analog_T * const P_HAL_ANALOG; 	/*!< pointer to ADC register map base address */
}
Analog_Config_T;

/*
	Analog_T per ADC
*/
typedef struct Analog_Tag
{
	const Analog_Config_T CONFIG;
	Ring_T ConversionQueue;	/* Item type (Analog_QueueItem_T *), (Analog_Conversion_T *) or (Analog_Options_T *) */
#ifdef CONFIG_ANALOG_ADC_HW_FIFO_LENGTH
	uint8_t ActiveChannelCount; /*! Hw fifo only. Number of active channels being processed by ADC */
	uint8_t ActiveChannelIndex; /*! Index into active conversion group */
#endif

	/*
		Active conversion as first conversion to support Enqueue Front (without shifting all items in queue).
		Also use as ADC active flag.
	*/
	//	const Analog_QueueItem_T * p_ActiveConversion; 	/*! Queue unit type selected conversion group in process */
	// todo prioity queue?
	//	bool IsLocalPeakFound;
}
Analog_T;

/*
	Queue buffer length in units
*/
#define ANALOG_INIT(p_HalAnalog, p_ConversionBuffer, ConversionQueueLength) 										\
{																													\
	.CONFIG = { .P_HAL_ANALOG = p_HalAnalog, },																		\
	.ConversionQueue = RING_INIT(p_ConversionBuffer, ConversionQueueLength, sizeof(Analog_QueueItem_T *), 0U),		\
}

extern void _Analog_ProcQueue(Analog_T * p_analog);

/******************************************************************************/
/*!
 *
 */
 /******************************************************************************/
static inline void _Analog_EnterCritical(Analog_T * p_analog)
{
#if defined(CONFIG_ANALOG_MULTITHREADED)
	/*
		Multithreaded calling of Activate. Must implement Critical_Enter
		Higher priority thread may overwrite Conversion setup data before ADC ISR returns.
		e.g. must be implemented if calling from inside interrupts and main.
	*/
	(void)p_analog;
	Critical_Enter();
#elif defined(CONFIG_ANALOG_SINGLE_THREADED)
	/*
		Single threaded calling of Activate. Or Threads of same priority level
		Single threaded case, and calling thread is lower priority than ADC ISR, ADC_DisableInterrupt local critical is suffcient over Critical_Enter global disable interrupt
		If calling thread is *lower* priority than ADC ISR, ADC ISR may occur after Conversion setup data is written by lower priority thread.
		If calling thread is *higher* priority than ADC ISR, Activate will run to completion, overwriting the active conversion. ADC ISR need global critcal
		Use global critical if disable ADC interrupts aborts active conversion
	*/
	HAL_Analog_DisableInterrupt(p_analog->CONFIG.P_HAL_ANALOG);
#endif
}

static inline void _Analog_ExitCritical(Analog_T * p_analog)
{
#if defined(CONFIG_ANALOG_MULTITHREADED)
	(void)p_analog;
	Critical_Exit();
#elif  defined(CONFIG_ANALOG_SINGLE_THREADED)
	HAL_Analog_EnableInterrupt(p_analog->CONFIG.P_HAL_ANALOG);
#endif
}

/*!
	@brief Private capture results subroutine
*/
static inline void _Analog_CaptureAdcResults(Analog_T * p_analog, Analog_Conversion_T * p_activeConversion)
{
	p_activeConversion->P_RESULTS_BUFFER[p_activeConversion->CHANNEL] = HAL_Analog_ReadResult(p_analog->CONFIG.P_HAL_ANALOG, p_activeConversion->PIN);
}

static inline void _Analog_CaptureResults(Analog_T * p_analog)
{
	Analog_Conversion_T * p_completedConversion;
	// bool isAllChannelsComplete;

	HAL_Analog_ClearConversionCompleteFlag(p_analog->CONFIG.P_HAL_ANALOG);
	Ring_Dequeue(&p_analog->ConversionQueue, &p_completedConversion);
	_Analog_CaptureAdcResults(p_analog, p_completedConversion);

	_Analog_ProcQueue(p_analog);

	// isAllChannelsComplete = true;

	if(p_completedConversion->ON_COMPLETE != 0U) { p_completedConversion->ON_COMPLETE(p_completedConversion->P_CALLBACK_CONTEXT); }

	// return (isAllChannelsComplete);
}

/*!
	@brief	Capture ADC results, Process Channel OnComplete Functions.
			Run in corresponding ADC ISR

	ADC ISR should be higher priority than thread calling Analog_Activate()
*/
static inline void Analog_OnComplete_ISR(Analog_T * p_analog)
{
	_Analog_CaptureResults(p_analog);
}

/*!
	@brief	Capture ADC results, by polling status register, if ISR is unavailable
*/
static inline void Analog_PollComplete(Analog_T * p_analog)
{
	_Analog_EnterCritical(p_analog);
	if(HAL_Analog_ReadConversionCompleteFlag(p_analog->CONFIG.P_HAL_ANALOG) == true)
	{
		Analog_OnComplete_ISR(p_analog);
	}
	_Analog_ExitCritical(p_analog);
}

/*
	Alternatively, return (p_analog->p_ActiveConversion != 0U);
	//case of aborted conversion?
*/
static inline bool _Analog_ReadIsActive(const Analog_T * p_analog)
{
	return
	(
		(HAL_Analog_ReadConversionActiveFlag(p_analog->CONFIG.P_HAL_ANALOG) == true) ||
		(HAL_Analog_ReadConversionCompleteFlag(p_analog->CONFIG.P_HAL_ANALOG) == true)
	);
}

/*
	Needed to overwrite continuous conversion
*/
static inline void Analog_Deactivate(Analog_T * p_analog)
{
	// _Analog_EnterCritical(p_analog);
//	p_analog->p_ActiveConversion = 0U;
	HAL_Analog_Deactivate(p_analog->CONFIG.P_HAL_ANALOG);
	// _Analog_ExitCritical(p_analog);
}

extern void Analog_Init(Analog_T * p_analog);
extern bool Analog_EnqueueConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion);
extern bool Analog_EnqueueOptions(Analog_T * p_analog, const Analog_Options_T * p_options);
extern void Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion);
extern void Analog_ActivateOptions(Analog_T * p_analog, const Analog_Options_T * p_options);

extern void _Analog_Group_ResumeQueue(Analog_T * p_analog);
extern void Analog_Group_PauseQueue(Analog_T * p_analog);
extern void Analog_Group_ResumeQueue(Analog_T * p_analog);
extern bool Analog_Group_EnqueueConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion);
extern bool Analog_Group_EnqueueOptions(Analog_T * p_analog, const Analog_Options_T * p_options);

#endif
