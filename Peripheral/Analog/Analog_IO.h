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
	@file 	Analog_IO.h
	@author FireSoucery
	@brief 	Analog module functions must be placed into corresponding user app threads
	@version V0
*/
/******************************************************************************/
#ifndef ANALOG_IO_H
#define ANALOG_IO_H

#include "Analog.h"
#include "HAL_ADC.h"
#include "Private.h"

#include <stdint.h>
#include <stdbool.h>

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
	for (uint8_t iChannel = 0; iChannel < activeChannelCount; iChannel += nAdc)
	{
		/* p_analog->p_ActiveConversion->ChannelCount > 1, on second loop, safe to dereference p_virtualChannels[index>0] */
		for (uint8_t iAdc = 0; iAdc < nAdc; iAdc++)
		{
			if (iChannel + iAdc < activeChannelCount)
			{
				iVirtualChannel = p_virtualChannels[iChannel + iAdc];
				p_analog->p_MapChannelResults[iVirtualChannel] = HAL_ADC_ReadResult(pp_adcMaps[iAdc], (uint32_t) p_analog->p_MapChannelPins[iVirtualChannel]);
			}
		}
	}
}


//static inline void CaptureAdcResults_NFixed
//(
//	const Analog_T * p_analog,
//	HAL_ADC_T (* const (* pp_adcMaps)),
//	uint8_t nAdc,
//	const Analog_VirtualChannel_T * p_virtualChannels,
//	uint8_t activeChannelCount
//)
//{
//
//}

static inline void ProcAnalogOnCompleteChannel
(
	const Analog_T * p_analog,
	uint8_t activeChannelStartIndex,
	uint8_t activeChannelCount
)
{
	for (uint8_t index = activeChannelStartIndex; index < activeChannelCount; index++)
	{
		if ((p_analog->p_ActiveConversion->p_OnCompleteChannels != 0U))
		{
			if ((p_analog->p_ActiveConversion->p_OnCompleteChannels[index] != 0U))
			{
				p_analog->p_ActiveConversion->p_OnCompleteChannels[index](p_analog->p_ActiveConversion->p_OnCompleteUserData);
			}
		}
	}
}


//bool DequeueAnalogConversion(Analog_T * p_analog)
//{
//
//	bool isSucess = false;
//
//	Critical_Enter();
//
//
//	Critical_Exit();
//
//	return isSucess;
//}


/*!
	@brief	Capture ADC results, when conversion is complete.
			Run in corresponding ADC ISR

	ADC ISR should be higher priority than thread calling Analog_Activate()
 */
static inline void Analog_CaptureResults_IO(Analog_T * p_analog)
{
	const Analog_VirtualChannel_T * p_virtualChannels;
	uint8_t completedChannelCount;
	uint8_t remainingChannelCount;
//	bool allChannelsComplete;
	const Analog_Conversion_T *  p_newActiveConversion;
	uint8_t newActiveConversionChannelIndex;
	uint8_t newActiveChannelCount;
	Analog_Config_T newConfig;
//	uint8_t onCompleteStartIndex = p_analog->ActiveConversionChannelIndex;


	if((p_analog->p_ActiveConversion != 0U) && (Analog_ReadConversionComplete(p_analog) == true))
	{
	//	Analog_ClearConversionComplete_IO(p_analog); //clears flags corresponding isrs set during activate, todo

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
		if(p_analog->p_ActiveConversion->ChannelCount > 1U)
		{
			p_virtualChannels = &p_analog->p_ActiveConversion->p_VirtualChannels[p_analog->ActiveConversionChannelIndex];
		}
		else
		{
			p_virtualChannels = &p_analog->p_ActiveConversion->VirtualChannel;
		}

	#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
		/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
		CaptureAdcResults_NMultiMuxed(p_analog, &p_analog->p_Adc, 1U, p_virtualChannels, 1U);
	#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
		/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel is in the same buffer */
		CaptureAdcResults_NMultiMuxed(p_analog, &p_analog->p_Adc, 1U, 	p_virtualChannels, p_analog->ActiveChannelCount);
	#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)
		/* Case N ADC 1 Buffer: ActiveChannelCount <= N_ADC Count */
		CaptureAdcResults_NMultiMuxed(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count, p_virtualChannels, p_analog->ActiveChannelCount);
	#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
		/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
		CaptureAdcResults_NMultiMuxed(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count, p_virtualChannels, p_analog->ActiveChannelCount);
	#endif


	#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
		completedChannelCount = 1U;
	#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
		completedChannelCount = p_analog->ActiveChannelCount;
	#endif

		remainingChannelCount = p_analog->p_ActiveConversion->ChannelCount - p_analog->ActiveConversionChannelIndex - completedChannelCount;

		if (remainingChannelCount > 0U)
//		if (p_analog->ActiveConversionChannelIndex + completedChannelCount < p_analog->p_ActiveConversion->ChannelCount)
		{
			/* Continue conversion, update index, Start next group of channels in selected p_ActiveConversion,  */
			p_newActiveConversion = p_analog->p_ActiveConversion; //no changes
			newActiveConversionChannelIndex = p_analog->ActiveConversionChannelIndex + completedChannelCount;
			newActiveChannelCount = CalcAdcActiveChannelCountMax(p_analog, remainingChannelCount);
			newConfig = CalcAdcActiveConfig(p_analog, p_newActiveConversion->Config, false);
			ActivateAdc(p_analog, &(p_newActiveConversion->p_VirtualChannels[newActiveConversionChannelIndex]), newActiveChannelCount, newConfig);
		}
		else
		{
			newActiveConversionChannelIndex = 0U;

			if (p_analog->ConversionQueueHead != p_analog->ConversionQueueTail)
			{
				/* Dequeue conversion, Analog_DequeueConversion */
				p_newActiveConversion = p_analog->pp_ConversionQueue[p_analog->ConversionQueueHead];

				if(p_analog->p_ActiveConversion->ChannelCount > 1U)
				{
					p_virtualChannels = p_newActiveConversion->p_VirtualChannels;
				}
				else
				{
					p_virtualChannels = &p_newActiveConversion->VirtualChannel;
				}

				newActiveChannelCount = CalcAdcActiveChannelCountMax(p_analog, p_newActiveConversion->ChannelCount);
				newConfig = CalcAdcActiveConfig(p_analog, p_newActiveConversion->Config, true);
				ActivateAdc(p_analog, p_virtualChannels, newActiveChannelCount, newConfig);

				p_analog->ConversionQueueHead = (p_analog->ConversionQueueHead + 1U) % p_analog->ConversionQueueLength;
			}
			else
			{
				/* All conversions complete */
				p_newActiveConversion = 0U;
				newActiveChannelCount = 0U;
				//(Analog_Disable(p_analog)
			}
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
	#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
		ProcAnalogOnCompleteChannel(p_analog, p_analog->ActiveConversionChannelIndex, 1U);
	#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
		ProcAnalogOnCompleteChannel(p_analog, onCompleteStartIndex, completedChannelCount)
	#endif

		if (remainingChannelCount <= 0U)
		{
			if (p_analog->p_ActiveConversion->OnCompleteConversion != 0U)
			{
				p_analog->p_ActiveConversion->OnCompleteConversion(p_analog->p_ActiveConversion->p_OnCompleteUserData);
			}
		}

		p_analog->p_ActiveConversion = p_newActiveConversion;
		p_analog->ActiveConversionChannelIndex = newActiveConversionChannelIndex;
	#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
		p_analog->ActiveChannelCount = newActiveChannelCount;
	#endif
	}
}


/*!
	@brief	Compiler may optimize when arguments are constant literals
 */
static inline void PollAdcResults(Analog_T * p_analog, 	HAL_ADC_T (* const (* pp_adcMaps)), uint8_t nAdc)
{
	bool capture;

	for (uint8_t iAdc = 0; iAdc < nAdc; iAdc++)
	{
		capture = HAL_ADC_ReadConversionCompleteFlag(pp_adcMaps[iAdc]);

		if (capture == false)
		{
			break;
		}
	}

	if (capture == true)
	{
		Analog_CaptureResults_IO(p_analog);
	}
}

/*!
	@brief	Capture ADC results, by polling status register, if ISR is unavailable
 */
static inline void Analog_PollResults_IO(Analog_T * p_analog)
{
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	PollAdcResults(p_analog, &p_analog->p_Adc, 1U);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	PollAdcResults(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count);
#endif
}

#endif
