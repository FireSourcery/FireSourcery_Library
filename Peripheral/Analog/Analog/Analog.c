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

#include "Utility/Queue/Queue.h"
/******************************************************************************/
/*!
	Protected
 */
/******************************************************************************/
void _Analog_ActivateAdc(const Analog_T * p_analog, const Analog_ConversionAdc_T * p_adcConversion, const Analog_ConversionMap_T * p_adcMap)
{
	HAL_ADC_T * p_adc 		= p_analog->CONFIG.P_HAL_ADC;
	uint8_t activeChannelCount 		= GetAnalogActiveAdcChannelCount(p_analog);

	const Analog_ConversionAdcChannel_T * p_adcChannels 	= p_adcConversion->P_CHANNELS;
	Analog_ConversionOptions_T options 						= p_adcMap->P_VIRTUAL_CONVERSION->OPTIONS;

	uint8_t iConversionIndex;
	analog_adcpin_t adcPin;

	((p_analog->ActiveConversionIndex == 0U) && (options.HwTriggerConversion == true)) ?
		HAL_ADC_EnableHwTrigger(p_analog->CONFIG.P_HAL_ADC) :
		HAL_ADC_DisableHwTrigger(p_analog->CONFIG.P_HAL_ADC);

	//#ifdef CONFIG_ANALOG_HW_CONTINOUS_CONVERSION //else use sw support
	(options.ContinuousConversion == true) ?
		HAL_ADC_EnableContinuousConversion(p_analog->CONFIG.P_HAL_ADC) :
		HAL_ADC_DisableContinuousConversion(p_analog->CONFIG.P_HAL_ADC);

	// activateChannelCount => S1U literal should optimize away for loop
	for (iConversionIndex = p_analog->ActiveConversionIndex; iConversionIndex < p_analog->ActiveConversionIndex + activeChannelCount - 1U; iConversionIndex++)
	{
		adcPin = p_adcChannels[iConversionIndex].PIN;
		HAL_ADC_WritePinSelect(p_adc, adcPin);
	}

	adcPin = p_adcChannels[iConversionIndex].PIN;
	HAL_ADC_WriteLast(p_adc, adcPin); /* for shared registers, enable interrupt of last ADC written */
	HAL_ADC_Activate(p_adc);
}

/*
 * New conversion
 */
void _Analog_ActivateConversion(Analog_T * p_analog, const Analog_ConversionAdc_T * p_adcConversion, const Analog_ConversionMap_T * p_adcMap)
{
	analog_adcresult_t * p_resultsBuffer 							= p_adcMap->P_RESULTS_BUFFER;
	const Analog_ConversionVirtualChannel_T * p_virtualChannels 	= p_adcMap->P_VIRTUAL_CONVERSION->P_CHANNELS;
	uint8_t virtualChannelCount 									= p_adcMap->P_VIRTUAL_CONVERSION->CHANNEL_COUNT;
	Analog_ConversionOptions_T options 								= p_adcMap->P_VIRTUAL_CONVERSION->OPTIONS;
	analog_virtual_t virtualChannel;

	if (options.CaptureLocalPeak == true)
	{
		for (uint8_t iVirtualIndex = 0U; iVirtualIndex < virtualChannelCount; iVirtualIndex++)
		{
			virtualChannel 						= p_virtualChannels[iVirtualIndex].CHANNEL;
			p_resultsBuffer[virtualChannel] 	= 0U; //does buffer need to be volatile
		}

		p_analog->IsLocalPeakFound = false;
		p_analog->count = 0;
	}

	p_analog->ActiveConversionIndex = 0U;
	_Analog_ActivateAdc(p_analog, p_adcConversion, p_adcMap);
}

//bool _Analog_EnqueueFrontConversion(Analog_T * p_analog, const void * p_conversion )
//{
//	bool isSuccess;
//
//	if (Queue_GetIsFull(&p_analog->ConversionQueue) == true)
//	{
//		Queue_RemoveBack(&p_analog->ConversionQueue, 1U);
//	}
//	Queue_PushFront(&p_analog->ConversionQueue, &p_conversion);
//
//	return isSuccess;
//}

bool _Analog_EnqueueConversion(Analog_T * p_analog, const void * p_conversion, bool isEnqueueFront)
{
	bool isSuccess;

	if (isEnqueueFront == true)
	{
		if (Queue_GetIsFull(&p_analog->ConversionQueue) == true)
		{
			Queue_RemoveBack(&p_analog->ConversionQueue, 1U);
		}
		Queue_PushFront(&p_analog->ConversionQueue, &p_conversion);
		isSuccess = true;
	}
	else
	{
		isSuccess = Queue_Enqueue(&p_analog->ConversionQueue, &p_conversion);
	}

	return isSuccess;
}

bool _Analog_DequeueConversion(Analog_T * p_analog)
{
	bool isSuccess = Queue_Dequeue(&p_analog->ConversionQueue, &p_analog->p_ActiveConversion);

	/*   Dequeue Next Conversion: */
	if (isSuccess == false)
	{
		p_analog->p_ActiveConversion = 0U;
		HAL_ADC_Deactivate(p_analog->CONFIG.P_HAL_ADC);
	}

	return isSuccess;
}

/******************************************************************************/
/*!
	Public
 */
/******************************************************************************/
void Analog_Init(Analog_T * p_analog)
{
	HAL_ADC_Init(p_analog->CONFIG.P_HAL_ADC);
	HAL_ADC_Deactivate(p_analog->CONFIG.P_HAL_ADC);
	Queue_Init(&p_analog->ConversionQueue);
}

/*!
	 @brief Public function to activate ADC.

	 overwrite active conversion
 */
void Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
	_Analog_EnterCritical(p_analog);
	p_analog->p_ActiveConversion = p_conversion;
	_Analog_ActivateConversion(p_analog, &p_conversion->ADC_CONVERSION, &p_conversion->MAP);
	_Analog_ExitCritical(p_analog);
}

/*
 * Needed to overwrite continuous conversion
 */
void Analog_DeactivateConversion(Analog_T * p_analog)
{
	_Analog_EnterCritical(p_analog);
	p_analog->p_ActiveConversion = 0U;
	HAL_ADC_Deactivate(p_analog->CONFIG.P_HAL_ADC);
	_Analog_ExitCritical(p_analog);
}


/*
	cannot overwrite
 */
bool Analog_EnqueueConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
	bool isSuccess;

	_Analog_EnterCritical(p_analog);  //must use global critical if disable adc interrupts aborts active conversion

	if ((Queue_GetIsEmpty(&p_analog->ConversionQueue) == false) && (_Analog_ReadIsActive(p_analog) == false))
	{
		p_analog->p_ActiveConversion = p_conversion;
		_Analog_ActivateConversion(p_analog, &p_conversion->ADC_CONVERSION, &p_conversion->MAP);
		isSuccess = true;
	}
	else
	{
		isSuccess = Queue_Enqueue(&p_analog->ConversionQueue, &p_conversion);
	}

	_Analog_ExitCritical(p_analog);

	return isSuccess;
}

/*
	can overwrite last item
 */
bool Analog_EnqueueFrontConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
//	bool isSuccess = false; //todo
	_Analog_EnterCritical(p_analog);

	if (_Analog_ReadIsActive(p_analog) == false)
	{
		p_analog->p_ActiveConversion = p_conversion;
		_Analog_ActivateConversion(p_analog, &p_conversion->ADC_CONVERSION, &p_conversion->MAP);
	}
	else
	{
		if (Queue_GetIsFull(&p_analog->ConversionQueue) == true)
		{
			Queue_RemoveBack(&p_analog->ConversionQueue, 1U);
		}
		Queue_PushFront(&p_analog->ConversionQueue, &p_conversion);
	}

	_Analog_ExitCritical(p_analog);

	return true;
}
//
///*
//	Dequeue if no conversions are active. Active conversion will automatically dequeue next conversion
// */
//bool Analog_PollDequeueConversion(Analog_T * p_analog)
//{
//	bool isSuccess = false;
//	const Analog_Conversion_T * p_conversion;
//
//	_Analog_EnterCritical(p_analog);
//	if ((Queue_GetIsEmpty(&p_analog->ConversionQueue) == false) && (_Analog_ReadIsActive(p_analog) == false))
//	{
//		isSuccess = true;
//		Queue_PeekFront(&p_analog->ConversionQueue, &p_conversion);
//		_Analog_ActivateConversion(p_analog, &p_conversion->ADC_CONVERSION, &p_conversion->MAP);
//	}
//	_Analog_ExitCritical(p_analog);
//
//	return isSuccess;
//}
//
//
///*!
//	@brief	Capture ADC results, by polling status register, if ISR is unavailable
// */
//void Analog_PollComplete(Analog_T * p_analog)
//{
//	_Analog_EnterCritical(p_analog);
//	if (Analog_ReadConversionComplete(p_analog))
//	{
//		Analog_OnAdcComplete_ISR(p_analog);
//	}
//	_Analog_ExitCritical(p_analog);
//}

