
/*
 * can use module internal compile time memory allocation if all concurrent apps share 1 channel maps
 */
//#define CONFIG_ANALOG_CHANNEL_COUNT
//extern const adcpin_t		ANALOG_CHANNEL_PINS[CONFIG_ANALOG_CHANNEL_COUNT]; 		/*!< User define */
//extern const uint8_t		ANALOG_CHANNEL_ADCS[CONFIG_ANALOG_CHANNEL_COUNT];
//extern volatile adcdata_t 	Analog_ChannelResults[CONFIG_ANALOG_CHANNEL_COUNT];		/*!< Measure Result Buffer */
//extern volatile uint32_t 	Analog_ChannelSumBuffer[CONFIG_ANALOG_CHANNEL_COUNT];			/*!< sum if multiple samples are required */

typedef struct
{
	Analog_T * p_Analog;
	uint8_t AnalogCount;
	Queue_T ConverisionQueue;

} AnalogN_T;




/*!
	 @brief
 */
//static inline void Analog_Dectivate(const Analog_T * p_analog)
//{
//
//	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
//	{
//		HAL_ADC_Dectivate(p_analog->pp_Adcs[iAdc]);
//		HAL_ADC_DisableInterrupt(p_analog->pp_Adcs[iAdc]);
//	}
//
//}
//
//static inline void Analog_DisableInterrupt(const Analog_T * p_analog)
//{
//
//	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
//	{
//		HAL_ADC_DisableInterrupt(p_analog->pp_Adcs[iAdc]);
//	}
//
//}
//
////when treating n adc virtualize as a single adc, if 1 conversion active
//static inline bool Analog_ReadConversionActive(const Analog_T * p_analog)
//{
//	bool isActive = (p_analog->p_ActiveConversion != 0U);
//
//
//	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
//	{
//		isActive |= HAL_ADC_ReadConversionActiveFlag(p_analog->pp_Adcs[iAdc]);
//	}
//
//	return isActive;
//}
//
////when treating n adc virtualize as a single adc, if all conversion complete
//static inline bool Analog_ReadConversionComplete(const Analog_T * p_analog)
//{
//
//	isComplete = false;
//	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
//	{
//		isActive &= HAL_ADC_ReadConversionCompleteFlag(p_analog->pp_Adcs[iAdc]);
//	}
//
//
//	return isComplete;
//}
//
//static inline void Analog_ClearConversionComplete(Analog_T * p_analog)
//{
//
//	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
//	{
//		HAL_ADC_ClearConversionCompleteFlag(p_analog->pp_Adcs[iAdc]);
//	}
//
//}
