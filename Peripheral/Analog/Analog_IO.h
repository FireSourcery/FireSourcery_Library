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
#include "HAL.h"
#include "Private.h"

#include <stdint.h>
#include <stdbool.h>

/*!
	 @brief Private capture results subroutine

	 @param[in] pp_adcMaps			read-only
	 @param[in] p_virtualChannels 	offset by p_analog->ActiveConversionChannelIndex

	 Compiler may optimize when arguments are constant literals
 */
static inline void CaptureResults_NDemux
(
	const Analog_T * p_analog,
	HAL_ADC_T (* const (* pp_adcMaps)),
	uint8_t nAdc,
	const Analog_VirtualChannel_T * p_virtualChannels,
	uint8_t activeChannelCount
)
{
	Analog_VirtualChannel_T iVirtualChannel;
	uint8_t iHwBuffer = 0;

	/* Read in the same way it was pushed */
	for (uint8_t iChannel = 0; iChannel < activeChannelCount; iChannel += nAdc)
	{
		/* p_analog->p_ActiveConversion->ChannelCount > 1, on second loop, safe to dereference p_virtualChannels[index>0] */
		for (uint8_t iAdc = 0; iAdc < nAdc; iAdc++)
		{
			if (iChannel + iAdc < activeChannelCount)
			{
				iVirtualChannel = p_virtualChannels[iChannel + iAdc];
				p_analog->p_MapChannelResults[iVirtualChannel] = HAL_ADC_ReadResult(pp_adcMaps[iAdc]);
			}
		}
		iHwBuffer++;
	}
}


static inline void CaptureResults_Fixed
(
	const Analog_T * p_analog,
	HAL_ADC_T (* const (* pp_adcMaps)),
	uint8_t nAdc,
	const Analog_VirtualChannel_T * p_virtualChannels,
	uint8_t activeChannelCount
)
{





}

/*!
	@brief	Capture ADC results, when conversion is complete.
			Run in corresponding ADC ISR

	ADC ISR should be higher priority than thread calling Analog_Activate()
 */
static inline void Analog_CaptureResults_IO(Analog_T * p_analog)
{
	const Analog_VirtualChannel_T * p_virtualChannels;
	bool allChannelsComplete = false;
	uint8_t remainingChannelCount;
	uint8_t activateChannelCount;

#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	uint8_t savedActiveConversionChannelIndex = p_analog->ActiveConversionChannelIndex;
	uint8_t savedActiveChannelCount = p_analog->ActiveChannelCount;
#endif

	//ADC_ClearCompleteFlag(p_analog->p_AdcRegisterMap); //if applicable

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
	if(p_analog->p_ActiveConversion->ChannelCount > 1)
	{
		p_virtualChannels = &p_analog->p_ActiveConversion->p_VirtualChannels[p_analog->ActiveConversionChannelIndex];
	}
	else
	{
		p_virtualChannels = &p_analog->p_ActiveConversion->VirtualChannel;
	}

#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
	CaptureResults_NDemux(p_analog, &p_analog->p_Adc, 									1, 					p_virtualChannels, 1);
#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel is in the same buffer */
	CaptureResults_NDemux(p_analog, &p_analog->p_Adc, 									1, 					p_virtualChannels, p_analog->ActiveChannelCount);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)
	/* Case N ADC 1 Buffer: ActiveChannelCount <= N_ADC Count */
	CaptureResults_NDemux(p_analog,  p_analog->pp_Adcs, 	p_analog->AdcN_Count, 	p_virtualChannels, p_analog->ActiveChannelCount);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
	CaptureResults_NDemux(p_analog,  p_analog->pp_Adcs, 	p_analog->AdcN_Count, 	p_virtualChannels, p_analog->ActiveChannelCount);
#endif

	/*
	 * Start next group of channels in selected p_ActiveConversion, same conversion, update index
	 */
#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
	p_analog->ActiveConversionChannelIndex++;
#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	p_analog->ActiveConversionChannelIndex += p_analog->ActiveChannelCount;
#endif

	remainingChannelCount = p_analog->p_ActiveConversion->ChannelCount - p_analog->ActiveConversionChannelIndex;

	if (remainingChannelCount > 0)
	{
		/* p_analog->p_ActiveConversion->ChannelCount > 1, past this point, safe to dereference p_virtualChannels[index>0] */
		p_virtualChannels = &p_analog->p_ActiveConversion->p_VirtualChannels[p_analog->ActiveConversionChannelIndex];

		/* For where p_analog->ActiveAdcChannelCount is always == 1, when using generalized case over single register case -> 1 excess runtime comparison to determine next active channel count*/
		activateChannelCount = CalcActivateChannelCount(p_analog, remainingChannelCount);
		ActivateConversion(p_analog, p_virtualChannels, activateChannelCount);
	}
	else
	{
		allChannelsComplete = true;
		p_analog->ActiveConversionChannelIndex = 0;

#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
		p_analog->ActiveChannelCount = 0;
#endif
	}

	/*	Critical_Exit(); */
	/*
	 * if the next conversion completes before OnComplete functions return, ADC ISR should queue, but cannot(should not) interrupt the on going ISR
	 */

//	if (p_analog->p_ActiveConversion->OnAdcComplete != 0)
//	{
//		p_analog->p_ActiveConversion->OnAdcComplete(p_analog);
//	}


#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
//	for (; savedActiveConversionChannelIndex < savedActiveChannelCount; savedActiveConversionChannelIndex++)
//	{
	while (savedActiveConversionChannelIndex < savedActiveChannelCount)
	{
		if ((p_analog->p_ActiveConversion != 0) && (p_analog->p_ActiveConversion->p_OnCompleteChannels[savedActiveConversionChannelIndex] != 0))
		{
			p_analog->p_ActiveConversion->p_OnCompleteChannels[savedActiveConversionChannelIndex](p_analog->p_OnCompleteUserData);
		}
		savedActiveConversionChannelIndex++;
	}
//	}
#endif

	if (allChannelsComplete)
	{
		if (p_analog->p_ActiveConversion->OnCompleteConversion != 0)
		{
			p_analog->p_ActiveConversion->OnCompleteConversion(p_analog->p_OnCompleteUserData);
		}
	}
}

//static inline void Analog_ActivateNextConversion_IO(Analog_T * p_analog)
//{
//	const Analog_VirtualChannel_T * p_virtualChannels;
//	bool allChannelsComplete = false;
//	uint8_t remainingChannelCount;
//	uint8_t activateChannelCount;
//
//
//}


/*!
	@brief	Compiler may optimize when arguments are constant literals
 */
static inline void PollResults(Analog_T * p_analog, HAL_ADC_T (* const (* pp_adcMaps)), uint8_t nAdc)
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
	PollResults(p_analog, &p_analog->p_Adc , 1);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	PollResults(p_analog,  p_analog->pp_Adcs, 	p_analog->AdcN_Count);
#endif
}

#endif
