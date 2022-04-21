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
	};

	uint8_t Flags;
}
AnalogN_AdcFlags_T;

typedef const struct
{
	Analog_Conversion_T CONVERSION;
	Analog_T * P_ANALOG;
}
AnalogN_Conversion_T;

#define CONFIG_ANALOG_N_CONVERSION(Channel, OnComplete, p_CallbackContext, p_Results, PinId, p_AnalogIHost) \
{																\
	.CONVERSION =												\
	{															\
		.TYPE 					= ANALOG_QUEUE_TYPE_CHANNEL, 	\
		.CHANNEL 				= Channel,						\
		.ON_COMPLETE 			= OnComplete,					\
		.P_CALLBACK_CONTEXT 	= p_CallbackContext,			\
		.P_RESULTS_BUFFER 		= p_Results,					\
		.PIN 					= PinId,						\
	},															\
	.P_ANALOG 					= p_AnalogIHost,				\
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


static inline bool AnalogN_ActivateConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
	(void)p_analogn;
	Analog_ActivateConversion(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
}

static inline bool AnalogN_EnqueueConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
	(void)p_analogn;
	return Analog_EnqueueConversion(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
}

//static inline bool AnalogN_EnqueueFrontConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
//{
//	Analog_EnqueueFrontConversion(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
//}

/*
 * Options AnalogN options on all adcs
 */
//static inline bool AnalogN_EnqueueOptions (AnalogN_T * p_analogn, AnalogN_AdcFlags_T activeAdcs)
//{
//	bool isSuccess;
//	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
//	{
//		Analog_EnqueueConversion(&p_analogn->CONFIG.P_ANALOGS[iAdc], &p_conversion->CONVERSION);
//	}
//}
//static inline bool AnalogN_EnqueueOptions(AnalogN_T * p_analogn, const AnalogN_ConversionOptions_T * p_conversion)
//{
//	return Analog_EnqueueConversionOptions(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
//}

//static inline bool AnalogN_EnqueueOptions_Group(AnalogN_T * p_analogn, const AnalogN_ConversionOptions_T * p_conversion)
//{
//	return Analog_EnqueueConversionOptions_Group(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
//}


/******************************************************************************/
/*!
	Group
*/
/******************************************************************************/
/*
 * Multithreaded still need to disable all interrupts
 * AnalogN_AdcFlags_T activeAdcs:
 * Single Threaded N adc - use
 * Multithreaded N adc - not use
 */
static inline void AnalogN_Group_PauseQueue(AnalogN_T * p_analogn, AnalogN_AdcFlags_T activeAdcs)
{
#if  (defined(CONFIG_ANALOG_MULTITHREADED) || defined(CONFIG_ANALOG_CRITICAL_USE_GLOBAL))
	Critical_Enter();
#elif (defined(CONFIG_ANALOG_SINGLE_THREADED))
	for(uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		if(((1U << iAdc) & activeAdcs.Flags) != 0U){_Analog_EnterCritical(&p_analogn->CONFIG.P_ANALOGS[iAdc]);}
	}
#endif
}

static inline void AnalogN_Group_ResumeQueue(AnalogN_T * p_analogn, AnalogN_AdcFlags_T activeAdcs)
{
#if  (defined(CONFIG_ANALOG_MULTITHREADED) || defined(CONFIG_ANALOG_CRITICAL_USE_GLOBAL))
	for(uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		if(((1U << iAdc) & activeAdcs.Flags) != 0U) {_Analog_Group_ResumeQueue(&p_analogn->CONFIG.P_ANALOGS[iAdc]);}
	}
	Critical_Exit();
#elif (defined(CONFIG_ANALOG_SINGLE_THREADED))
	for(uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		if(((1U << iAdc) & activeAdcs.Flags) != 0U) {Analog_ResumeQueue(&p_analogn->CONFIG.P_ANALOGS[iAdc]);}
	}
#endif
}

static inline bool AnalogN_Group_EnqueueConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
	(void)p_analogn;
	return Analog_Group_EnqueueConversion(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
}

extern void AnalogN_Init(AnalogN_T * p_analogn);

#endif


