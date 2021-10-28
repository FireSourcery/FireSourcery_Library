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

/*!
	@brief Fill N ADCs
	@param[in] pp_adcMaps			read-only
	@param[in] p_virtualChannels 	offset by p_analog->ActiveConversionChannelIndex

	Fill N ADCs. When channels are fixed to particular ADC
		//todo algo N adc activation, need virtualchannel to adc map
 */
static inline void ActivateAdc_NFixed
(

)
{

}

void AnalogN_ActivateConversion (Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
//	Analog_T * p_analog,
////	HAL_ADC_T (* const (* pp_adcMaps)),
////	uint8_t nAdc,
//	const Analog_ConversionChannel_T * p_channels,
//	uint8_t activateChannelCount,
//	Analog_ConversionOptions_T options

	//	uint8_t channelCount = CalcAdcActiveChannelCountMax(p_analog, activateChannelCount);
		analog_channel_t channel;
		uint8_t analogId;

	for (uint8_t iChannelIndex = 0U; iChannelIndex < channelCount; iChannelIndex++)
	{
		channel = p_channels[iChannelIndex].CHANNEL;
		analogId = p_analogN->CONFIG.P_ADC_MAP[channel];
		Queue_Enqueue(&p_analogN.p_Analog[analogId]->ChannelQueue, &p_channels[iChannelIndex]);
	}
}
