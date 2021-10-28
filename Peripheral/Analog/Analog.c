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

#include "Private.h"
#include "Config.h"


/******************************************************************************/
/*!
 * Private
 */
/******************************************************************************/
static inline bool AquireMutex(Analog_T * p_analog)
{
#if defined(CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED)
	return Critical_AquireMutex(&p_analog->Mutex);
#else
	return true;
#endif
}

static inline void ReleaseMutex(Analog_T * p_analog)
{
#if defined(CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED)
	Critical_ReleaseMutex(&p_analog->Mutex);
#endif
}

static inline void EnterCritical(Analog_T * p_analog)
{
#if  (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED))
	/*
	 * Multithreaded calling of Activate.
	 * Must implement Critical_Enter
	 *
	 * Higher priority thread may overwrite Conversion setup data before ADC ISR returns.
	 * e.g. must be implemented if calling from inside interrupts and main.
	 */
//	Critical_Enter();
	AquireMutex(p_analog);
#elif defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)
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

static inline void ExitCritical(Analog_T * p_analog)
{
#if (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED)) && !defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)
//	Critical_Exit();
	ReleaseMutex(p_analog);
#endif
	//interrupts already enabled
}

void Analog_Init(Analog_T * p_analog)
{
	HAL_Analog_Init(p_analog->CONFIG.P_ADC);
	Analog_Dectivate(p_analog);
	Queue_Init(&p_analog->ChannelQueue);
	Queue_Init(&p_analog->ConverisionQueue);
}

//void Analog_ActivateConversion_simple(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
//{
//
//
//}

/*!
	 @brief Public function to activate ADC.

	 overwrite active conversion
 */
void Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion, void * p_onCompleteContext)
{
//	const Analog_VirtualChannel_T * p_virtualChannels;
	uint8_t activateChannelCount;
//	Analog_Config_T activateConfig;

	if(EnterCritical(p_analog))
	{
		//copy if n adc. no copy, no mixed channels, use conversion, if 1 adc
	//	p_virtualChannels = p_conversion->P_CHANNELS;
	//
	//	activateChannelCount 	= CalcAdcActiveChannelCountMax(p_analog, p_conversion->CHANNEL_COUNT);
	//	activateConfig 			= CalcAdcActiveConfig(p_analog, p_conversion->OPTIONS, true);

		activateChannelCount = ActivateAdc(p_analog, p_conversion->P_CHANNELS, p_conversion->CHANNEL_COUNT, p_conversion->OPTIONS);

		p_analog->p_ActiveConversion->p_Conversion = p_conversion;
		p_analog->p_ActiveConversion->p_OnCompleteContext = p_onCompleteContext;
		p_analog->p_ActiveConversion->Options = p_conversion->OPTIONS;

		p_analog->ActiveConversionChannelIndex = 0U;
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
		p_analog->ActiveChannelCount = activateChannelCount;
#endif

		Critical_Exit();
	}
}

/*
	cannot overwrite
 */
bool Analog_EnqueueConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion, void * p_onCompleteContext)
{
	bool isSuccess;

	Analog_ConversionActive_T conversionActive =
	{
		.p_Conversion = p_conversion,
		.p_OnCompleteContext = p_onCompleteContext,
		.Options = p_conversion->OPTIONS,
	};

	Critical_Enter(); //  if multithreaded

	if (Analog_ReadConversionActive(p_analog) == true)
	{	//if interrupt occurs here, next conversion will not chain
		isSuccess = Queue_Enqueue(&p_analog->ConverisionQueue, &conversionActive);
	}
	else
	{
		Analog_ActivateConversion(p_analog, p_conversion, p_onCompleteContext);
		isSuccess = true;
	}

	Critical_Exit();

	return isSuccess;
}

/*
	can overwrite last item i.e tail of queue
 */
bool Analog_EnqueueFrontConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion, void * p_onCompleteContext)
{
	bool isSuccess;

	Analog_ConversionActive_T conversionActive =
	{
		.p_Conversion = p_conversion,
		.p_OnCompleteContext = p_onCompleteContext,
		.Options = p_conversion->OPTIONS,
	};

	Critical_Enter();

	if (Analog_ReadConversionActive(p_analog) == true)
	{
		if (Queue_GetIsFull(&p_analog->ConverisionQueue) == true)
		{
			Queue_RemoveBack(&p_analog->ConverisionQueue, 1U);
		}

		isSuccess = Queue_Push(&p_analog->ConverisionQueue, &conversionActive);
	}
	else
	{
		Analog_ActivateConversion(p_analog, p_conversion, p_onCompleteContext);
	}

	Critical_Exit();

	return isSuccess;
}

/*
	Dequeue if no conversions are active. Active conversion will automatically dequeue next conversion
 */
bool Analog_DequeueConversion(Analog_T * p_analog)
{
	bool isSuccess;
	Analog_ConversionActive_T conversionActive;

	Critical_Enter();

	if (Analog_ReadConversionActive(p_analog) == false)
	{
		isSuccess = Queue_Denqueue(&p_analog->ConverisionQueue, &conversionActive);

		if(isSuccess == true)
		{
			Analog_ActivateConversion(p_analog, conversionActive.p_Conversion, conversionActive.p_OnCompleteContext);
		}
	}
	else
	{
		isSuccess = false;
	}

	Critical_Exit();

	return isSuccess;
}



//static inline uint32_t ReadRequest(const HAL_ADC_T * p_adc, Analog_Request_T request)
//{
//	uint32_t response;
//
//	switch (request)
//	{
//	case ANALOG_REQUEST_REG_CONVERSION_COMPLETE:
//		response = (uint32_t) HAL_ADC_ReadConversionCompleteFlag(p_adc);
//		break;
//	default:
//		response = 0;
//		break;
//	}
//
//	return response;
//}
//
//static inline void WriteConfig(HAL_ADC_T * p_adc, Analog_Config_T config)
//{
//	p_analog->ActiveConfig = config;
//
//	if (config.UseInterrrupt)
//	{
//		HAL_ADC_EnableInterrupt(p_adc);
//	}
//	else
//	{
//		HAL_ADC_DisableInterrupt(p_adc);
//	}
//}
//
///*!
//	 @brief
// */
//void Analog_WriteConfig(Analog_T * p_analog, Analog_Config_T config)
//{
//#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
//	WriteConfig(p_analog,  p_analog->p_Adc, config);
//#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
//	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
//	{
//		WriteConfig(p_analog->pp_Adcs[iAdc], config);
//	}
//#endif
//}



////is new data available
//bool Analog_IsNewData(Analog_T * p_analog, Analog_VirtualChannel_T channel)
//{
//
//}
//
////read new data
//void Analog_PollChannel(Analog_T * p_analog, Analog_VirtualChannel_T channel)
//{
//	if (Analog_IsNewData(p_analog, channel))
//	{
//		Analog_ReadChannel(p_analog, channel);
//	}
//}
//void Analog_WaitResult(Analog_T * p_analog)
//{
//	//while(!p_analog->ADC_GetCompleteFlag());
//	ADC_GetCompleteFlag(p_analog->p_ADC_RegMap_IO);
//
//	while(!ADC_GetCompleteFlag(p_analog->p_ADC_RegMap_IO[p_analog->ADC_Count_IO - 1]));
//}



/*!
	 @brief Single Channel Conversion
 */
//void Analog_ActivateConversionChannel(Analog_T * p_analog, Analog_VirtualChannel_T p_channel, Analog_Config_T config, void (*onComplete)(volatile void * p_userData), volatile void * p_userData)
//{
//#if  (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED))
//	Critical_Enter();
//#elif defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)
//	Analog_DisableInterrupt(p_analog);
//#endif
//
//	ActivateAdc(p_analog, p_channel, 1U, config);
////	p_analog->p_ActiveConversion = 0U;
////	p_analog->ActiveConversionChannelIndex = 0U;
//
//#if (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED)) && !defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)
//	Critical_Exit();
//#endif
//}

//void Analog_ActivateChannel(Analog_T * p_analog, Analog_VirtualChannel_T p_channel, Analog_Config_T config, void (*onComplete)(volatile void * p_userData), volatile void * p_userData)
//{
//#if  (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED))
//	Critical_Enter();
//#elif defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)
//	Analog_DisableInterrupt(p_analog);
//#endif
//
//	ActivateAdc(p_analog, p_channel, 1U, config);
////	p_analog->p_ActiveConversion = 0U;
////	p_analog->ActiveConversionChannelIndex = 0U;
//
//#if (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED)) && !defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)
//	Critical_Exit();
//#endif
//}
