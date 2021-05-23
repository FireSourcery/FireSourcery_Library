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
	@file 	Private.h
	@author FireSoucery
	@brief 	Analog module common "private" functions
	@version V0
*/
/******************************************************************************/
#ifndef PRIVATE_ANALOG_H
#define PRIVATE_ANALOG_H

#include "Analog.h"
#include "HAL_ADC.h"
#include "Config.h"

/*!
	@brief Fill N ADCs "round robin"
	@param[in] pp_adcMaps			read-only
	@param[in] p_virtualChannels 	offset by p_analog->ActiveConversionChannelIndex

	Only when channel can demux to any ADC

	Fill N ADCs "round robin" as follows:

	const uint8_t ANALOG_CONVERSION_CHANNELS[] =
	{
		ANALOG_VIRTUAL_CHANNEL_A, 		<- ADC0 Buffer0
		ANALOG_VIRTUAL_CHANNEL_B, 		<- ADC1 Buffer0
		ANALOG_VIRTUAL_CHANNEL_C, 		<- ADCN Buffer0
		ANALOG_VIRTUAL_CHANNEL_D,	 	<- ADC0 Buffer1
	};

	Compiler may able to optimize when arguments are constant literals
 */
static inline void ActivateAdc_NMultiMuxed
(
	Analog_T * p_analog,
	HAL_ADC_T (* const (* pp_adcMaps)),
	uint8_t nAdc,
	const Analog_VirtualChannel_T * p_virtualChannels,
	uint8_t channelCount,
	Analog_Config_T config
)
{
	Analog_VirtualChannel_T iVirtualChannel;

	for (uint8_t iAdc = 0U; iAdc < nAdc; iAdc++)
	{
		//deactivate if needed
		if (config.UseHwTriggerPerChannel || config.UseHwTriggerPerConversion)
		{
			HAL_ADC_WriteHwTriggerState(pp_adcMaps[iAdc], 1U);
		}
		else
		{
			HAL_ADC_WriteHwTriggerState(pp_adcMaps[iAdc], 0U);
		}
	}

	for (uint8_t iChannel = 0U; iChannel < channelCount; iChannel += nAdc)
	{
		for (uint8_t iAdc = 0U; iAdc < nAdc; iAdc++)
		{
			iVirtualChannel = p_virtualChannels[iChannel + iAdc];
			/*
			 * iVirtualChannel should be less than p_analog->VirtualChannelMapLength
			 * Boundary check if needed
			 * if (iVirtualChannel < p_analog->VirtualChannelMapLength)
			 */
//			p_analog->p_MapChannelResults[iVirtualChannel] = 0U;

			if (iChannel + iAdc < channelCount - 1U)
			{
				HAL_ADC_WritePinSelect(pp_adcMaps[iAdc], (uint32_t) p_analog->p_VirtualChannelMapPins[iVirtualChannel]);
			}
			else /* (iChannel + iAdc == activeChannelCount - 1) */
			{
				HAL_ADC_Activate(pp_adcMaps[iAdc], (uint32_t) p_analog->p_VirtualChannelMapPins[iVirtualChannel]); /* enable interrupt of last ADC written */
				break; 		/* iChannel += nAdc should also break from outer loop */
			}
		}
	}
}

/*!
	@brief Fill N ADCs
	@param[in] pp_adcMaps			read-only
	@param[in] p_virtualChannels 	offset by p_analog->ActiveConversionChannelIndex

	Fill N ADCs. When channels are fixed to particular ADC
		//todo algo N adc activation, need virtualchannel to adc map
 */
//static inline void ActivateAdc_NFixed
//(
//	Analog_T * p_analog,
//	HAL_ADC_T (* const (* pp_adcMaps)),
//	uint8_t nAdc,
//	const Analog_VirtualChannel_T * p_virtualChannels,
//	uint8_t channelCount
//)
//{
////	Analog_VirtualChannel_T iVirtualChannel;
////	uint8_t iAdc;
////
////	iVirtualChannel = *p_virtualChannels;
////	iAdc = p_analog->p_MapChannelAdcs[iVirtualChannel];
////
////	for (uint8_t index = 0U; index < channelCount - 1; index++)
////	{
////
////	}
//
//}

/*!
	@brief Fill 1 ADCs
 */
//static inline void ActivateAdc_Single
//(
//	Analog_T * p_analog,
//	const Analog_VirtualChannel_T * p_virtualChannels,
//	uint8_t activateChannelCount
//)
//{
//	ActivateAdc_NMultiMuxed(p_analog, &p_analog->p_Adc, 1U, p_virtualChannels, activateChannelCount);
//}

/*!
	@brief 	Start the selected segment channels
 	 	 	Share by public Analog_ActivateConversion, and Analog_CaptureResults_IO
 */
static inline void ActivateAdc
(
	Analog_T * p_analog,
	const Analog_VirtualChannel_T * p_virtualChannels,
	uint8_t channelCount,
	Analog_Config_T config
)
{
/* Channel Demux N ADC  */
#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
	(void)channelCount;
	ActivateAdc_NMultiMuxed(p_analog, &p_analog->p_Adc, 1U, 				p_virtualChannels, 1U, config);
//	ActivateAdc_Single(p_analog, p_virtualChannels, 1U);
#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	/* * Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
	ActivateAdc_NMultiMuxed(p_analog, &p_analog->p_Adc, 1U, 				p_virtualChannels, channelCount, config);
//	ActivateAdc_Single(p_analog, p_virtualChannels, activateChannelCount);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)
	/* Case N ADC 1 Buffer:  ActiveChannelCount <= N_ADC Count */
	ActivateAdc_NMultiMuxed(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count, 	p_virtualChannels, channelCount, config);
//	ActivateAdc_NFixed()
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
	ActivateAdc_NMultiMuxed(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count, 	p_virtualChannels, channelCount, config);
//	ActivateAdc_NFixed()
#endif
}

/*!
	@brief 	Get Active channel count called before ActivateConversion()

			Share by public Analog_ActivateConversion, and Analog_CaptureResults_IO
 */
static inline uint8_t CalcAdcActiveChannelCountMax
(
	Analog_T * p_analog,
	uint8_t targetChannelCount
)
{
	uint8_t maxChannelCount;

	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	(void)p_analog;
	(void)targetChannelCount;
	maxChannelCount = 1U;
#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
	maxChannelCount = p_analog->AdcM_Buffer;
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)
	/* Case N ADC 1 Buffer:  ActiveChannelCount <= N_ADC Count */
	maxChannelCount = p_analog->AdcN_Count;
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
	maxChannelCount = p_analog->AdcN_Count * p_analog->AdcM_Buffer;
#endif

#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
	if (targetChannelCount < maxChannelCount)
	{
		maxChannelCount = targetChannelCount;
	}
#endif

	return maxChannelCount;
}


static inline Analog_Config_T CalcAdcActiveConfig
(
	Analog_T * p_analog,
	Analog_Config_T conversionConfig,
	bool isFirstActivation
)
{
	Analog_Config_T activateConfig;

	if (conversionConfig.UseConfig == 1U)
	{
		activateConfig = conversionConfig;
	}
	else
	{
		activateConfig = p_analog->DefaultConfig;
	}

	if (isFirstActivation == false)
	{
		activateConfig.UseHwTriggerPerConversion = 0;
	}

	return activateConfig;
}

#endif
