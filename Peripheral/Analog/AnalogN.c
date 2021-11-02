

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
