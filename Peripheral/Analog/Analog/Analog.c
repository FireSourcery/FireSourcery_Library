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
//static inline void WriteAdcChannel(const Analog_T * p_analog, const analog_adcpin_t pin)
//{
//	HAL_ADC_Activate(p_analog->CONFIG.P_HAL_ADC, pin);
//}

//#ifdef CONFIG_ANALOG_ADC_HW_FIFO_ENABLE
//static inline void WriteAdcFifo(const Analog_T * p_analog)
//{
//	Analog_Conversion_T * p_conversion;
//	uint8_t iConversionIndex;
//
//	for (iConversionIndex = p_analog->ActiveConversionIndex; iConversionIndex < p_analog->ActiveConversionIndex + p_analog->ActiveAdcFifoChannelCount - 1U; iConversionIndex++)
//	{
//		Queue_PeekIndex(&p_analog->ConversionQueue, p_conversion, iConversionIndex);
//		HAL_ADC_WritePinSelect(p_analog->CONFIG.P_HAL_ADC, p_conversion->PIN);
//	}
//
//	Queue_PeekIndex(&p_analog->ConversionQueue, p_conversion, iConversionIndex);
//	HAL_ADC_WriteLast(p_analog->CONFIG.P_HAL_ADC, p_conversion->PIN);
//}
//#endif
//
void _Analog_WriteAdcOptions(Analog_T * p_analog, const Analog_Conversion_T * p_conversion) //, const Analog_ConversionAdc_T * p_adcConversion, const Analog_ConversionVirtualMap_T * p_adcMap)
{
//	if(p_conversion->OPTIONS.FLAGS.IsValid == 1U)
//	{
//		if(p_conversion->OPTIONS.FLAGS.HwTriggerConversion == 1U)
//		{
//			HAL_ADC_EnableHwTrigger(p_analog->CONFIG.P_HAL_ADC);
//		}
//		else
//		{
//			HAL_ADC_DisableHwTrigger(p_analog->CONFIG.P_HAL_ADC);
//		}

	if(p_conversion->OPTIONS.IS_VALID == true)
	{
		if(p_conversion->OPTIONS.HW_TRIGGER == true)
		{
			HAL_ADC_EnableHwTrigger(p_analog->CONFIG.P_HAL_ADC);
		}
		else
		{
			HAL_ADC_DisableHwTrigger(p_analog->CONFIG.P_HAL_ADC);
		}


#ifdef CONFIG_ANALOG_HW_CONTINOUS_CONVERSION_ENABLE //else use sw support
		(options.ContinuousConversion == true) ?
			HAL_ADC_EnableContinuousConversion(p_analog->CONFIG.P_HAL_ADC) :
			HAL_ADC_DisableContinuousConversion(p_analog->CONFIG.P_HAL_ADC);
#endif

		if(p_conversion->OPTIONS.ON_OPTIONS != 0U)
		{
			p_conversion->OPTIONS.ON_OPTIONS(p_conversion->P_CALLBACK_CONTEXT);
		}
	}
}

/*
 * New conversion
 */
void _Analog_WriteAdc(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)//, const Analog_ConversionAdc_T * p_adcConversion, const Analog_ConversionVirtualMap_T * p_adcMap)
{
//	analog_adcresult_t * p_resultsBuffer 							= p_adcMap->P_RESULTS_BUFFER;
//	const Analog_ConversionVirtualChannel_T * p_virtualChannels 	= p_adcMap->P_VIRTUAL_CONVERSION->P_CHANNELS;
//	uint8_t virtualChannelCount 									= p_adcMap->P_VIRTUAL_CONVERSION->CHANNEL_COUNT;
//	Analog_ConversionOptions_T options 								= p_adcMap->P_VIRTUAL_CONVERSION->OPTIONS;
//	analog_virtual_t virtualChannel;

//	if (options.CaptureLocalPeak == true)
//	{
//		for (uint8_t iVirtualIndex = 0U; iVirtualIndex < virtualChannelCount; iVirtualIndex++)
//		{
//			virtualChannel 						= p_virtualChannels[iVirtualIndex].CHANNEL;
//			p_resultsBuffer[virtualChannel] 	= 0U; //does buffer need to be volatile
//		}
//
//		p_analog->IsLocalPeakFound = false;
////		p_analog->count = 0;
//	}
//	Analog_ConversionChannel_T * p_queueConversion;
	/*
	 * Get conversions in queue with same options
	 */
//	p_analog->ActiveConversionCount = 1U; //may optimize with channel count queue
//	for (uint8_t iEntry = 1U; iEntry < Queue_GetFullCount(&p_analog->ConversionQueue); iEntry++)
//	{
//		if (Queue_PeekIndex(&p_analog->ConversionQueue, &p_queueConversion, 0U) != 0)
//		{
//			if (p_queueConversion->P_VIRTUAL->OPTIONS ==  p_conversion->P_VIRTUAL->OPTIONS)
//			{
//				p_analog->ActiveConversionCount++;
//			}
//			else
//			{
//				break;
//			}
//		}
//	}

	_Analog_WriteAdcOptions(p_analog, p_conversion);
	/*
	 * Activate and wait for return
	 */
	HAL_ADC_Activate(p_analog->CONFIG.P_HAL_ADC, p_conversion->PIN);
}


/*
 * Dequeue all conversion containing only options, until next channel, starts wait for isr
 */
void _Analog_ProcQueue(Analog_T * p_analog)
{
	Analog_Conversion_T * p_nextConversion;
	bool isEmpty = true;

	for(uint8_t iConversion = 0U; iConversion < Queue_GetFullCount(&p_analog->ConversionQueue) + 1U; iConversion++)
	{
		if (Queue_PeekFront(&p_analog->ConversionQueue, &p_nextConversion) == true)
		{
			if (p_nextConversion->P_VIRTUAL == 0U) //if options only, its done //ensure empty conversion does not call adc isr
			{
				_Analog_WriteAdcOptions(p_analog, p_nextConversion);
				Queue_RemoveFront(&p_analog->ConversionQueue, 1U);
			}
			else
			{
				_Analog_WriteAdc(p_analog, p_nextConversion); //if activate and wait for isr
				isEmpty = false;
				break;
			}
		}
	}

	if (isEmpty == true)
	{
		HAL_ADC_Deactivate(p_analog->CONFIG.P_HAL_ADC);
	}
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
//void Analog_ActivateConversion(Analog_T * p_analog, const Analog_ConversionChannel_T * p_conversion)
//{
//	_Analog_EnterCritical(p_analog);
////	p_analog->p_ActiveConversion = p_conversion;
//	Queue_RemoveFront(&p_analog->ConversionQueue, p_analog->ActiveConversionCount - p_analog->ActiveConversionIndex);
//	_Analog_ActivateConversion(p_analog, p_conversion);//, &p_conversion->ADC_CONVERSION, &p_conversion->MAP);
//	_Analog_ExitCritical(p_analog);
//}

/*
 * Needed to overwrite continuous conversion
 */
void Analog_DeactivateConversion(Analog_T * p_analog)
{
	_Analog_EnterCritical(p_analog);
//	p_analog->p_ActiveConversion = 0U;
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

	if ((Queue_GetIsEmpty(&p_analog->ConversionQueue) == true)) //(_Analog_GetIsActive(p_analog) == false) &&
	{
//		p_analog->p_ActiveConversion = p_conversion;
		Queue_Enqueue(&p_analog->ConversionQueue, &p_conversion);
		_Analog_WriteAdc(p_analog, p_conversion); // &p_conversion->ADC_CONVERSION, &p_conversion->MAP);
		isSuccess = true;
	}
	else
	{
		isSuccess = Queue_Enqueue(&p_analog->ConversionQueue, &p_conversion);
	}

	_Analog_ExitCritical(p_analog);

	return isSuccess;
}

bool Analog_EnqueueConversionOptions(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
	bool isSuccess;

	_Analog_EnterCritical(p_analog);  //must use global critical if disable adc interrupts aborts active conversion

	if ((Queue_GetIsEmpty(&p_analog->ConversionQueue) == true)) //(_Analog_GetIsActive(p_analog) == false) &&
	{
		_Analog_WriteAdcOptions(p_analog, p_conversion);
		HAL_ADC_Deactivate(p_analog->CONFIG.P_HAL_ADC);
		isSuccess = true;
	}
	else
	{
		isSuccess = Queue_Enqueue(&p_analog->ConversionQueue, &p_conversion);
	}

	_Analog_ExitCritical(p_analog);

	return isSuccess;
}


void Analog_PauseQueue(Analog_T * p_analog)
{
	_Analog_EnterCritical(p_analog);
}

void Analog_ResumeQueue(Analog_T * p_analog)
{
	_Analog_ExitCritical(p_analog);
}

bool Analog_EnqueueConversion_Group(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
	bool isSuccess;

	if ((Queue_GetIsEmpty(&p_analog->ConversionQueue) == true))
	{
		Queue_Enqueue(&p_analog->ConversionQueue, &p_conversion);
		_Analog_WriteAdc(p_analog, p_conversion);
		isSuccess = true;
	}
	else
	{
		isSuccess = Queue_Enqueue(&p_analog->ConversionQueue, &p_conversion);
	}

	return isSuccess;
}

bool Analog_EnqueueConversionOptions_Group(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
	bool isSuccess;

	if ((Queue_GetIsEmpty(&p_analog->ConversionQueue) == true))
	{
		_Analog_WriteAdcOptions(p_analog, p_conversion);
		HAL_ADC_Deactivate(p_analog->CONFIG.P_HAL_ADC);
		isSuccess = true;
	}
	else
	{
		isSuccess = Queue_Enqueue(&p_analog->ConversionQueue, &p_conversion);
	}

	return isSuccess;
}



/*
	can overwrite last item
 */
//bool Analog_EnqueueFrontConversion(Analog_T * p_analog, const Analog_ConversionChannel_T * p_conversion)
//{
////	bool isSuccess = false; //todo
//	_Analog_EnterCritical(p_analog);
//
//	if (_Analog_GetIsActive(p_analog) == false)
//	{
////		p_analog->p_ActiveConversion = p_conversion;
//		_Analog_ActivateConversion(p_analog, p_conversion);//, &p_conversion->ADC_CONVERSION, &p_conversion->MAP);
//	}
//	else
//	{
//		if (Queue_GetIsFull(&p_analog->ConversionQueue) == true)
//		{
//			Queue_RemoveBack(&p_analog->ConversionQueue, 1U);
//		}
//		Queue_PushFront(&p_analog->ConversionQueue, &p_conversion);
//		p_analog->ActiveConversionCount = 0U;
//		p_analog->ActiveConversionIndex = 0U;
//		//Active conversion will end and deqeue this conversion.
//		//the following conversion will restore options as the intial conversion
//	}
//
//	_Analog_ExitCritical(p_analog);
//
//	return true;
//}


////
/////*
////	Dequeue if no conversions are active. Active conversion will automatically dequeue next conversion
//// */
//bool Analog_PollDequeueConversion(Analog_T * p_analog)
//{
//	bool isSuccess = false;
//	const Analog_Conversion_T * p_conversion;
//
//	_Analog_EnterCritical(p_analog);
//	if (Queue_PeekFront(&p_analog->ConversionQueue, &p_conversion) == true)
//	{
//		_Analog_ActivateConversion(p_analog, p_conversion);//, &p_conversion->ADC_CONVERSION, &p_conversion->MAP);
//		isSuccess = true;
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

