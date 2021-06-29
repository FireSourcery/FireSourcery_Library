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
#include "HAL_ADC.h"

#include "Private.h"
#include "Config.h"

/*!
	 @brief Initialize struct object
 */
void Analog_Init
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
)
{

#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	p_analog->p_Adc = (HAL_ADC_T *) p_adcMap;
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	if (nAdc > 1U)
	{
		p_analog->pp_Adcs = (HAL_ADC_T (* const (*))) p_adcMap;
	}
	else
	{
		p_analog->p_Adc = (HAL_ADC_T *) p_adcMap;
	}
#endif

#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	p_analog->AdcN_Count 	= nAdc;
#endif

#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	p_analog->AdcM_Buffer 	= mHwBufferLength;
#endif

	p_analog->p_VirtualChannelMapPins 	= p_virtualChannelMapPins;
	p_analog->p_VirtualChannelResults 	= p_virtualChannelMapResultsBuffer;
	p_analog->ChannelCount				= virtualChannelCount;

#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_FIXED)
	p_analog->p_MapChannelAdcs = p_virtualChannelMapAdcs;
#endif

	p_analog->pp_ConversionQueue = pp_conversionQueue;
	p_analog->ConversionQueueLength = conversionQueueLength;
	p_analog->ConversionQueueHead = 0U;
	p_analog->ConversionQueueTail = 0U;

	Analog_Dectivate(p_analog);
}

void Analog_Init_Struct
(
	Analog_T * p_analog,
	Analog_Init_T * p_init
)
{
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	p_analog->p_Adc =  p_init->P_HAL_ADC;
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	if (nAdc > 1U)
	{
		p_analog->pp_Adcs = p_init->P_HAL_ADC;
	}
	else
	{
		p_analog->p_Adc = p_init->P_HAL_ADC;
	}
#endif

#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	p_analog->AdcN_Count 	= p_init->ADC_COUNT;
#endif

#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	p_analog->AdcM_Buffer 	= p_init->ADC_BUFFER_LENGTH;
#endif

	p_analog->p_VirtualChannelMapPins 		= p_init->P_VIRTUAL_CHANNEL_MAP_PINS;
	p_analog->p_VirtualChannelResults 	= p_init->P_VIRTUAL_CHANNEL_MAP_RESULTS_BUFFER;
	p_analog->ChannelCount			= p_init->CHANNEL_COUNT;

#if defined(CONFIG_ANALOG_ADC_HW_N_ADC_FIXED)
	p_analog->p_MapChannelAdcs = P_VIRTUAL_CHANNEL_MAP_ADCS;
#endif

	p_analog->pp_ConversionQueue = p_init->PP_CONVERSION_QUEUE;
	p_analog->ConversionQueueLength = p_init->CONVERSION_QUEUE_LENGTH;
	p_analog->ConversionQueueHead = 0U;
	p_analog->ConversionQueueTail = 0U;
}


/*!
	 @brief Public function to activate ADC.

	 overwrite active conversion
 */
void Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
	const Analog_VirtualChannel_T * p_virtualChannels;
	uint8_t activateChannelCount;
	Analog_Config_T activateConfig;

	/* Convert from union to pointer for uniform processing */
	if (p_conversion->ChannelCount > 1U)
	{
		p_virtualChannels = p_conversion->p_VirtualChannels;
	}
	else
	{
		p_virtualChannels = &p_conversion->VirtualChannel;
	}

	activateChannelCount = CalcAdcActiveChannelCountMax(p_analog, p_conversion->ChannelCount);
	activateConfig = CalcAdcActiveConfig(p_analog, p_conversion->Config, true);


#if  (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED))
	/*
	 * Multithreaded calling of Activate.
	 * Must implement Critical_Enter
	 *
	 * Higher priority thread may overwrite Conversion setup data before ADC ISR returns.
	 * e.g. must be implemented if calling from inside interrupts and main.
	 */
	Critical_Enter();
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


//#ifdef CONFIG_ANALOG_ADC_HW_N_ADC_FIXED
//	for (uint8_t iAdc; iAdc < p_analog->AdcN_Count; iAdc++)
//	{
//		p_analog->p_ActiveConversionChannelIndexes[iAdc] = 0;
//	}
//	p_analog->ActiveM 			= p_analog->ActiveTotal / adcCount;
//	p_analog->ActiveRemainder 	= p_analog->ActiveTotal % adcCount; /* ideally uses the result of the division. */
//#endif

	ActivateAdc(p_analog, p_virtualChannels, activateChannelCount, activateConfig);
	p_analog->p_ActiveConversion = p_conversion;
	p_analog->ActiveConversionChannelIndex = 0U;
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	p_analog->ActiveChannelCount = activateChannelCount;
#endif

#if (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_LIBRARY_DEFINED)) && !defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)
	Critical_Exit();
#endif

}

/*
 * Conversion Queue
 * Dequeue from header
 * Enqueue to tail
 */
// cannot overwrite first item i.e head of queue
bool Analog_EnqueueConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
	uint8_t newTail;
	bool isSucess;

	if (Analog_ReadConversionActive(p_analog) == true)
	{
		Critical_Enter();

		newTail = (p_analog->ConversionQueueTail + 1U) % p_analog->ConversionQueueLength;

		if (newTail != p_analog->ConversionQueueHead)
		{
			p_analog->pp_ConversionQueue[p_analog->ConversionQueueTail] = p_conversion;
			p_analog->ConversionQueueTail = newTail;
			isSucess = true;
		}
		else
		{
			isSucess = false;
		}

		Critical_Exit();
	}
	else
	{
		Analog_ActivateConversion(p_analog, p_conversion);
		isSucess = true;
	}

	return isSucess;
}

/*
 Dequeue if no conversions are active. Active conversion will automatically dequeue next conversion
 */
bool Analog_DequeueConversion(Analog_T * p_analog)
{
	bool isSucess;

	if (Analog_ReadConversionActive(p_analog) == false)
	{
		Critical_Enter(); // still needed if multi threaded activate

		if (p_analog->ConversionQueueHead != p_analog->ConversionQueueTail)
		{
			Analog_ActivateConversion(p_analog, p_analog->pp_ConversionQueue[p_analog->ConversionQueueHead]);
			p_analog->ConversionQueueHead = (p_analog->ConversionQueueHead + 1U) % p_analog->ConversionQueueLength;
			isSucess = true;
		}
		else
		{
			isSucess = false;
		}

		Critical_Exit();
	}
	else
	{
		isSucess = false;
	}

	return isSucess;
}

//can overwrite last item i.e tail of queue
void Analog_EnqueueFrontConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
	if (Analog_ReadConversionActive(p_analog) == true)
	{
		Critical_Enter();

		if (p_analog->ConversionQueueHead < 1U)
		{
			p_analog->ConversionQueueHead = p_analog->ConversionQueueLength - 1;
		}
		else
		{
			p_analog->ConversionQueueHead--;
		}

		p_analog->pp_ConversionQueue[p_analog->ConversionQueueHead] = p_conversion;

		Critical_Exit();
	}
	else
	{
		Analog_ActivateConversion(p_analog, p_conversion);
	}
}





/*!
	 @brief Get pointer to channel result
 */
volatile analog_t * Analog_GetPtrChannelResult(const Analog_T * p_analog, Analog_VirtualChannel_T channel)
{
	volatile analog_t * p_result;

	if (channel < p_analog->ChannelCount)
	{
		p_result = &p_analog->p_VirtualChannelResults[(uint8_t)channel];
	}
	else
	{
		p_result = 0U;
	}

	return p_result;
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

