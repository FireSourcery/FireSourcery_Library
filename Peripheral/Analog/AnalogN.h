
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
    @file 	.h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef ANALOG_N_H
#define ANALOG_N_H

typedef uint8_t analogn_channel_t;
typedef uint8_t analogn_index_t;

typedef const struct
{
	analog_channel_t ANALOG_CHANNEL;
	Analog_T * P_ANALOG; //index? 	analogn_index_t ANALOG_INDEX; //index into P_ANALOGS


	//Derive from analog
//	const analog_adcpin_t PIN;
//	HAL_ADC_T * const P_HAL_ADC;
}
AnalogN_ChannelMapEntry_T; //conversion virtual channel map

typedef const struct
{
	AnalogN_ChannelMapEntry_T * P_CHANNEL_MAP;
	uint8_t CHANNEL_COUNT;
//	const analog_adcpin_t * const P_PINS;					/*!< Channel Pins array, virtual channel index. Translates virtual channels to ADC pin channels */
	volatile analog_adcdata_t * const P_RESULTS_BUFFER;		/*!< Persistent ADC results buffer, virtual channel index.  */


} AnalogN_ChannelMap_T;

//* Multiple ADC can share 1 map. 1 Map per application. Each application requires a unique set of Physical/Virtual pins
//* Map needs to be index per application. e.g. write index will not work with shared map startChannel(1)


typedef const struct
{
	Analog_T * P_ANALOGS;
	uint8_t ANALOG_COUNT;

//	const AnalogN_ChannelMap_T * P_CHANNEL_MAPS;
//	const uint8_t CHANNEL_MAP_COUNT;
	AnalogN_ChannelMapEntry_T * P_CHANNEL_MAP;
	uint8_t CHANNEL_COUNT;
}
AnalogN_Config_T;

typedef struct
{
	const AnalogN_Config_T CONFIG;

//	Queue_T ConverisionQueue;

	volatile uint8_t ActiveAnalogCount;
	volatile uint8_t ActiveConversionCount;
}
AnalogN_T;


typedef const struct
{
//	const AnalogN_ChannelMap_T * P_CHANNEL_MAP;
//	void * P_ON_COMPLETE_CONTEXT;
}
AnalogN_ConversionContext_T;





static inline void AnalogN_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{

}

static inline void AnalogN_ActivateConversion_Context(Analog_T * p_analog, const Analog_Conversion_T * p_conversion, void * p_onCompleteContext)
{

}




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
//static inline void ActivateAdc_NMultiMuxed
//(
//	Analog_T * p_analog,
//	HAL_ADC_T (* const (* pp_adcMaps)),
//	uint8_t nAdc,
//	const Analog_VirtualChannel_T * p_virtualChannels,
//	uint8_t channelCount,
//	Analog_Config_T config
//)
//{
//	Analog_VirtualChannel_T iVirtualChannel;
//
//	for (uint8_t iAdc = 0U; iAdc < nAdc; iAdc++)
//	{
//		//deactivate if needed
//		if (config.UseHwTriggerPerChannel || config.UseHwTriggerPerConversion)
//		{
//			HAL_ADC_WriteHwTriggerState(pp_adcMaps[iAdc], 1U);
//		}
//		else
//		{
//			HAL_ADC_WriteHwTriggerState(pp_adcMaps[iAdc], 0U);
//		}
//	}
//
//	for (uint8_t iChannel = 0U; iChannel < channelCount; iChannel += nAdc)
//	{
//		for (uint8_t iAdc = 0U; iAdc < nAdc; iAdc++)
//		{
//			iVirtualChannel = p_virtualChannels[iChannel + iAdc];
//			/*
//			 * iVirtualChannel should be less than p_analog->VirtualChannelMapLength
//			 * Boundary check if needed
//			 * if (iVirtualChannel < p_analog->VirtualChannelMapLength)
//			 */
////			p_analog->p_MapChannelResults[iVirtualChannel] = 0U;
//
//			if (iChannel + iAdc < channelCount - 1U)
//			{
//				HAL_ADC_WritePinSelect(pp_adcMaps[iAdc], (uint32_t) p_analog->p_VirtualChannelMapPins[iVirtualChannel]);
//			}
//			else /* (iChannel + iAdc == activeChannelCount - 1) */
//			{
//				HAL_ADC_WriteLast(pp_adcMaps[iAdc], (uint32_t) p_analog->p_VirtualChannelMapPins[iVirtualChannel]); /* enable interrupt of last ADC written */
//				HAL_ADC_Activate(pp_adcMaps[iAdc]);
//				break; 		/* iChannel += nAdc should also break from outer loop */
//			}
//
//		}
//	}
//}




static inline void ActivateAdcs
(
	Analog_T * p_analog,
	const Analog_VirtualChannel_T * p_virtualChannels,
	uint8_t channelCount,
	Analog_Config_T config
)
{

	/* Case N ADC 1 Buffer:  ActiveChannelCount <= N_ADC Count */
	ActivateAdc_NMultiMuxed(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count, 	p_virtualChannels, channelCount, config);
//	ActivateAdc_NFixed()

	/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
	ActivateAdc_NMultiMuxed(p_analog, p_analog->pp_Adcs, p_analog->AdcN_Count, 	p_virtualChannels, channelCount, config);
//	ActivateAdc_NFixed()
}

/*!
	@brief 	Get Active channel count called before ActivateConversion()

			Share by public Analog_ActivateConversion, and Analog_CaptureResults
 */
//static inline uint8_t CalcAdcActiveChannelCountMax
//(
//	Analog_T * p_analog,
//	uint8_t remainingChannelCount
//)
//{
//	uint8_t maxChannelCount;
//
//	/* Case 1 ADC 1 Buffer: ActiveChannelCount Always == 1 */
//#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
//	(void)p_analog;
//	(void)remainingChannelCount;
//	maxChannelCount = 1U;
//#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
//	/* Case 1 ADC M Buffer: ActiveChannelCount <= M_Buffer Length, all active channel must be in same buffer */
//	maxChannelCount = p_analog->AdcM_Buffer;
//#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)
//	/* Case N ADC 1 Buffer:  ActiveChannelCount <= N_ADC Count */
//	maxChannelCount = p_analog->AdcN_Count;
//#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
//	/* Case N ADC M Buffer: ActiveChannelCount <= N_ADC Count * M_Buffer Length */
//	maxChannelCount = p_analog->AdcN_Count * p_analog->AdcM_Buffer;
//#endif
//
//#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER) && !defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER)
//	if (remainingChannelCount < maxChannelCount)
//	{
//		maxChannelCount = remainingChannelCount;
//	}
//#endif
//
//	return maxChannelCount;
//}

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

//static inline void CaptureResults(const AnalogN_T * p_analogn)
//{
//	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
//	{
//		CaptureAdcResults(&p_analogn->CONFIG.P_ANALOGS[iAdc]);
//	}
//}



/*!
	 @brief
 */
static inline void AnalogN_Dectivate(const AnalogN_T * p_analogn)
{
	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		Analog_Dectivate(&p_analogn->CONFIG.P_ANALOGS[iAdc]);
	}
}

static inline void AnalogN_DisableInterrupt(const AnalogN_T * p_analogn)
{
	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		Analog_DisableInterrupt(&p_analogn->CONFIG.P_ANALOGS[iAdc]);
	}
}

////need critical
////when treating n adc virtualize as a single adc, if 1 conversion active
//static inline bool AnalogN_ReadConversionActive(const AnalogN_T * p_analogn)
//{
//	bool isActive = false;
//
//	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
//	{
//		isActive |= Analog_ReadConversionActive(&p_analogn->CONFIG.P_ANALOGS[iAdc]);
//	}
//
//	return isActive;
//}
//
////need critical
////when treating n adc virtualize as a single adc, if all conversion complete
//static inline bool AnalogN_ReadConversionComplete(const AnalogN_T * p_analogn)
//{
//	bool isComplete = true;
//
//	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
//	{
//		isComplete &= Analog_ReadConversionComplete(&p_analogn->CONFIG.P_ANALOGS[iAdc]);
//	}
//
//	return isComplete;
//}

static inline void AnalogN_ClearConversionComplete(const AnalogN_T * p_analogn)
{
	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		Analog_ClearConversionComplete(&p_analogn->CONFIG.P_ANALOGS[iAdc]);
	}
}


/*!
	@brief
 */
static inline void AnalogN_PollCaptureResults(const AnalogN_T * p_analogn)
{
	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		Analog_PollCaptureResults(&p_analogn->CONFIG.P_ANALOGS[iAdc]);
	}
}

#endif
