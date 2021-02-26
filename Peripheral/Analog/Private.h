/**************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/**************************************************************************/
/**************************************************************************/
/*!
	@file 	Private.h
	@author FireSoucery
	@brief 	Analog module common "private" functions
	@version V0
*/
/**************************************************************************/
#ifndef PRIVATE_ANALOG_H
#define PRIVATE_ANALOG_H

#include "Analog.h"

#include "Config.h"

#ifdef CONFIG_ANALOG_ADC_FUNCTIONS_HAL_DEFINED
	#include "Peripheral/HAL/HAL.h"
#elif defined(CONFIG_ANALOG_ADC_FUNCTIONS_USER_DEFINED)
	#include "Import.h"
#endif


/*!
	@brief Fill N ADCs "round robin"
	@param[in] pp_adcMaps			read-only
	@param[in] p_virtualChannels 	offset by p_analog->ActiveConversionChannelIndex

	Fill N ADCs "round robin" as follows:

	const uint8_t ANALOG_CONVERSION_CHANNELS[] =
	{
		ANALOG_VIRTUAL_CHANNEL_A, 		<- ADC0 Buffer0
		ANALOG_VIRTUAL_CHANNEL_B, 		<- ADC1 Buffer0
		ANALOG_VIRTUAL_CHANNEL_C, 		<- ADCN Buffer0
		ANALOG_VIRTUAL_CHANNEL_D,	 	<- ADC0 Buffer1
	};

	compiler may able to optimize when arguments are constant literals
 */
static inline void ActivateAdc_CompilerOptimize
(
	Analog_T * p_analog,
	volatile void (* const (* pp_adcMaps)),
	uint8_t nAdc,
	const Analog_VirtualChannel_T * p_virtualChannels,
	uint8_t activeChannelCount
)
{
	Analog_VirtualChannel_T iVirtualChannel;
	uint8_t iHwBuffer = 0U;

	for (uint8_t iChannel = 0U; iChannel < activeChannelCount; iChannel += nAdc)
	{
		for (uint8_t iAdc = 0U; iAdc < nAdc; iAdc++)
		{
			iVirtualChannel = p_virtualChannels[iChannel + iAdc];
			/*
			 * iVirtualChannel should be less than p_analog->VirtualChannelMapLength
			 * Boundary check if needed
			 * if (iVirtualChannel < p_analog->VirtualChannelMapLength)
			 */
			p_analog->p_VirtualChannelMapResults[iVirtualChannel] = 0;

			if (iChannel + iAdc < activeChannelCount - 1)
			{
				ADC_WritePinSelect(pp_adcMaps[iAdc], (uint32_t) p_analog->p_VirtualChannelMapPins[iVirtualChannel], iHwBuffer);
			}
			else
			{
				ADC_Activate(pp_adcMaps[iAdc], (uint32_t) p_analog->p_VirtualChannelMapPins[iVirtualChannel], iHwBuffer, p_analog->ActiveConfig);
				break; 		/* iChannel += nAdc should also break from outer loop */
			}
		}
		iHwBuffer++;
	}

}

/*!
	@brief 	Get Active channel count called before ActivateConversion()

			Share by public Analog_ActivateConversion, and Analog_CaptureResults_IO
 */
static inline uint8_t GetActivateChannelCount
(
	Analog_T * p_analog,
	uint8_t targetChannelCount
)
{
	uint8_t activateChannelCount;

#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
	activateChannelCount = 1U;
#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
	activateChannelCount = p_analog->M_HwBuffer;
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)
	/* Case N ADC 1 Buffer:  ActiveChannelCount <= N_ADC Count */
	activateChannelCount = p_analog->N_Adc;
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
	activateChannelCount = p_analog->N_Adc * p_analog->M_HwBuffer;
#endif

#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	if (targetChannelCount < activateChannelCount)
	{
		activateChannelCount = targetChannelCount;
	}
#endif

	return activateChannelCount;
}

/*!
	@brief 	Start the selected segment channels
 	 	 	Share by public Analog_ActivateConversion, and Analog_CaptureResults_IO
 */
static inline void ActivateConversion
(
	Analog_T * p_analog,
	const Analog_VirtualChannel_T * p_virtualChannels,
	uint8_t activateChannelCount
)
{
#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
	ActivateAdc_CompilerOptimize(p_analog, &p_analog->p_AdcRegisterMap, 1U, 				p_virtualChannels, 1U);
#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	/* * Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
	ActivateAdc_CompilerOptimize(p_analog, &p_analog->p_AdcRegisterMap, 1U, 				p_virtualChannels, activateChannelCount);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)
	/* Case N ADC 1 Buffer:  ActiveChannelCount <= N_ADC Count */
	ActivateAdc_CompilerOptimize(p_analog, p_analog->pp_AdcRegisterMaps, p_analog->N_Adc, 	p_virtualChannels, activateChannelCount);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
	ActivateAdc_CompilerOptimize(p_analog, p_analog->pp_AdcRegisterMaps, p_analog->N_Adc, 	p_virtualChannels, activateChannelCount);
#endif

#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	p_analog->ActiveChannelCount = activateChannelCount;
#endif
}

#endif
