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
#include "HAL_Analog.h"
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

static inline void CaptureAdcResults(const Analog_T * p_analog,	const Analog_VirtualChannel_T * p_virtualChannels)
{
#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
	CaptureAdcResults_NMultiMuxed(p_analog, &p_analog->p_Adc, 1U, p_virtualChannels, 1U);
#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel is in the same buffer */
	CaptureAdcResults_NMultiMuxed(p_analog, &p_analog->p_Adc, 1U, 	p_virtualChannels, p_analog->ActiveChannelCount);
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


/*!
	@brief	Compiler may optimize when arguments are constant literals
 */
static inline void PollAdcResults(Analog_T * p_analog, 	HAL_ADC_T (* const (* pp_adcMaps)), uint8_t nAdc)
{
	bool capture;

	for (uint8_t iAdc = 0U; iAdc < nAdc; iAdc++)
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
