/*
 * Virtual Channel
 *
 * Virtual indexes for ADC pin IDs. Scales 1 per application or pin set.
 */
#ifdef CONFIG_ANALOG_VIRUTAL_CHANNEL_ENUM_USER_DEFINED
/*
 * User provide enum typedef
 */
/*
typedef enum
{
	ANALOG_VIRTUAL_CHANNEL_X,
	ANALOG_VIRTUAL_CHANNEL_Y,
	ANALOG_VIRTUAL_CHANNEL_Z,
	ANALOG_VIRTUAL_CHANNEL_A,
	ANALOG_VIRTUAL_CHANNEL_B,
	ANALOG_VIRTUAL_CHANNEL_C,
}
Analog_VirtualChannel_T;
*/
/*
 * Multi applications use overlapping enum values, or use uint8_t
 */
/*
typedef enum
{
	APP1_ANALOG_VIRTUAL_CHANNEL_X = 1,
	APP1_ANALOG_VIRTUAL_CHANNEL_Y = 2,
	APP1_ANALOG_VIRTUAL_CHANNEL_Z = 3,
	APP2_ANALOG_VIRTUAL_CHANNEL_A = 1,
	APP2_ANALOG_VIRTUAL_CHANNEL_B = 2,
	APP2_ANALOG_VIRTUAL_CHANNEL_C = 3,
}
Analog_VirtualChannel_T;
*/
#elif defined(CONFIG_ANALOG_VIRUTAL_CHANNEL_UINT8)
	typedef uint8_t Analog_VirtualChannel_T;
#endif



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
				HAL_ADC_WriteLast(pp_adcMaps[iAdc], (uint32_t) p_analog->p_VirtualChannelMapPins[iVirtualChannel]); /* enable interrupt of last ADC written */
				HAL_ADC_Activate(pp_adcMaps[iAdc]);
				break; 		/* iChannel += nAdc should also break from outer loop */
			}

		}
	}
}


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






//typedef enum
//{
//	ANALOG_STATUS_OK = 0,
//	ANALOG_STATUS_ERROR_A = 1,
//} Analog_Status_T;
//
//typedef enum
//{
//	ANALOG_REQUEST_REG_CONVERSION_COMPLETE,
//	ANALOG_REQUEST_REG_CONVERSION_ACTIVE,
//} Analog_Request_T;


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
//#if  (defined(CONFIG_ANALOG_CRITICAL_USER_DEFINED) || defined(CONFIG_ANALOG_CRITICAL_LIBRARY_DEFINED))
//	EnterCritical();
//#elif defined(CONFIG_ANALOG_CRITICAL_DISABLE)
//	Analog_DisableInterrupt(p_analog);
//#endif
//
//	ActivateAdc(p_analog, p_channel, 1U, config);
////	p_analog->p_ActiveConversion = 0U;
////	p_analog->ActiveConversionChannelIndex = 0U;
//
//#if (defined(CONFIG_ANALOG_CRITICAL_USER_DEFINED) || defined(CONFIG_ANALOG_CRITICAL_LIBRARY_DEFINED)) && !defined(CONFIG_ANALOG_CRITICAL_DISABLE)
//	ExitCritical();
//#endif
//}

//void Analog_ActivateChannel(Analog_T * p_analog, Analog_VirtualChannel_T p_channel, Analog_Config_T config, void (*onComplete)(volatile void * p_userData), volatile void * p_userData)
//{
//#if  (defined(CONFIG_ANALOG_CRITICAL_USER_DEFINED) || defined(CONFIG_ANALOG_CRITICAL_LIBRARY_DEFINED))
//	EnterCritical();
//#elif defined(CONFIG_ANALOG_CRITICAL_DISABLE)
//	Analog_DisableInterrupt(p_analog);
//#endif
//
//	ActivateAdc(p_analog, p_channel, 1U, config);
////	p_analog->p_ActiveConversion = 0U;
////	p_analog->ActiveConversionChannelIndex = 0U;
//
//#if (defined(CONFIG_ANALOG_CRITICAL_USER_DEFINED) || defined(CONFIG_ANALOG_CRITICAL_LIBRARY_DEFINED)) && !defined(CONFIG_ANALOG_CRITICAL_DISABLE)
//	ExitCritical();
//#endif
//}
