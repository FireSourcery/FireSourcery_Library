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
	@file 	Analog.c
	@author FireSoucery
	@brief 	Analog module conventional function definitions
	@version V0
*/
/******************************************************************************/
#include "Analog.h"
#include "HAL_Analog.h"

#include "Config.h"

#include "System/Queue/Queue.h"

#ifdef CONFIG_ANALOG_CRITICAL_LIBRARY_DEFINED
	#include "System/Critical/Critical.h"
#elif defined(CONFIG_ANALOG_CRITICAL_USER_DEFINED)
	extern inline void Critical_Enter(void);
	extern inline void Critical_Exit(void);
#elif defined(CONFIG_ANALOG_CRITICAL_DISABLE)

#endif

/******************************************************************************/
/*!
 * Private
 */
/******************************************************************************/
static inline void EnterCritical()
{
#if  (defined(CONFIG_ANALOG_CRITICAL_USER_DEFINED) || defined(CONFIG_ANALOG_CRITICAL_LIBRARY_DEFINED))
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

#elif (defined(CONFIG_ANALOG_CRITICAL_DISABLE) || defined(CONFIG_ANALOG_SINGLE_THREADED))
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

static inline void ExitCritical()
{
#if (defined(CONFIG_ANALOG_CRITICAL_USER_DEFINED) || defined(CONFIG_ANALOG_CRITICAL_LIBRARY_DEFINED)) && !defined(CONFIG_ANALOG_CRITICAL_DISABLE)
	Critical_Exit();
//#elif  || defined(CONFIG_ANALOG_SINGLE_THREADED) 	//interrupts already enabled
#endif
}

void Analog_Init(Analog_T * p_analog)
{
	HAL_Analog_Init(p_analog->CONFIG.P_ADC);
	Analog_Dectivate(p_analog);
//	Queue_Init(&p_analog->ChannelQueue);
	Queue_Init(&p_analog->ConversionQueue);
}

void Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
	EnterCritical();
	p_analog->ActiveChannelIndex = 0U;
	ActivateAnalogConversion(p_analog, p_conversion);
	ExitCritical();
}

/*!
	 @brief Public function to activate ADC.

	 overwrite active conversion
 */
void Analog_ActivateConversion_Context(Analog_T * p_analog, const Analog_Conversion_T * p_conversion, Analog_ConversionContext_T * p_onContext)
{
	EnterCritical();
	p_analog->ActiveChannelIndex = 0U;
	ActivateAnalogConversion_Context(p_analog, p_conversion, p_onContext);
	ExitCritical();
}

/*
	cannot overwrite
 */
bool Analog_EnqueueConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion, Analog_ConversionContext_T * p_onContext)
{
	bool isSuccess;

	Analog_ConversionActive_T conversionActive =
	{
		.p_Conversion = p_conversion,
		.p_OnCompleteContext = p_onContext,
	};

	EnterCritical();

	if (Analog_ReadConversionActive(p_analog) == true)
	{	//if interrupt occurs here, next conversion will not chain
		isSuccess = Queue_Enqueue(&p_analog->ConversionQueue, &conversionActive);
	}
	else
	{
		p_analog->ActiveChannelIndex = 0U;
		ActivateAnalogConversion_Context(p_analog, p_conversion, p_onContext);
		isSuccess = true;
	}

	ExitCritical();

	return isSuccess;
}

/*
	can overwrite last item
 */
bool Analog_EnqueueFrontConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion, Analog_ConversionContext_T * p_onContext)
{
	bool isSuccess = false;

	Analog_ConversionActive_T conversionActive =
	{
		.p_Conversion = p_conversion,
		.p_OnCompleteContext = p_onContext,
	};

	EnterCritical();

	if (Analog_ReadConversionActive(p_analog) == true)
	{
		if (Queue_GetIsFull(&p_analog->ConversionQueue) == true)
		{
			Queue_RemoveBack(&p_analog->ConversionQueue, 1U);
		}

		isSuccess = Queue_PushFront(&p_analog->ConversionQueue, &conversionActive);
	}
	else
	{
		p_analog->ActiveChannelIndex = 0U;
		ActivateAnalogConversion_Context(p_analog, p_conversion, p_onContext);
	}

	ExitCritical();

	return isSuccess;
}

/*
	Dequeue if no conversions are active. Active conversion will automatically dequeue next conversion
 */
bool Analog_DequeueConversion(Analog_T * p_analog)
{
	bool isSuccess = false;

	EnterCritical();

	if (Analog_ReadConversionActive(p_analog) == false)
	{
		if(Queue_Dequeue(&p_analog->ConversionQueue, &p_analog->ActiveConversion) == true)
		{
			isSuccess = true;
			p_analog->ActiveChannelIndex = 0U;
			ActivateAnalogConversionThis(p_analog);
		}
	}

	ExitCritical();

	return isSuccess;
}


