/******************************************************************************/
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
/******************************************************************************/
/******************************************************************************/
/*!
	@file 	Analog.c
	@author FireSoucery
	@brief 	Analog module conventional function definitions
	@version V0
*/
/******************************************************************************/
#include "Analog.h"
#include "HAL.h"

#include "Private.h"
#include "Config.h"

/*!
	 @brief Initialize struct object
 */
void Analog_Init
(
	Analog_T * p_analog,
	const void * p_adcMap,
	uint8_t nAdc,
	uint8_t mHwBufferLength,
	uint8_t virtualChannelCount,
	const adcpin_t * p_virtualChannelMapPins,
	const uint8_t * p_virtualChannelMapAdcs,
	volatile analog_t * p_virtualChannelMapResultsBuffer,
//	volatile uint8_t * p_activeAdcChannelIndexesBuffer, /* length of N ADC */
	void * p_onCompleteUserData
)
{
	if (nAdc > 1)
	{
		p_analog->pp_Adcs = (HAL_ADC_T (* const (*))) p_adcMap;
	}
	else
	{
		p_analog->p_Adc = (HAL_ADC_T *) p_adcMap;
	}

	p_analog->AdcN_Count 			= nAdc;
	p_analog->AdcM_LengthBuffer 	= mHwBufferLength;

	p_analog->p_MapChannelPins 		= p_virtualChannelMapPins;
	p_analog->p_MapChannelResults 	= p_virtualChannelMapResultsBuffer;
	p_analog->ChannelCount			= virtualChannelCount;

	p_analog->p_OnCompleteUserData = p_onCompleteUserData;

	p_analog->p_MapChannelAdcs = 0;
}

/*!
	 @brief Public function to activate ADC.
 */
void Analog_ActivateConversion(Analog_T * p_analog, Analog_Conversion_T * p_conversion)
{
	const Analog_VirtualChannel_T * p_virtualChannels;
	uint8_t activateChannelCount;

	/* Convert from union to pointer for uniform processing */
	if (p_conversion->ChannelCount > 1)
	{
		p_virtualChannels = p_conversion->p_VirtualChannels;
	}
	else
	{
		p_virtualChannels = &p_conversion->VirtualChannel;
	}

	activateChannelCount = CalcActivateChannelCount(p_analog, p_conversion->ChannelCount);

	/*
	 * Single threaded calling of Activate.
	 * Single threaded case, and calling thread is lower priority than ADC ISR, may implement ADC_DisableInterrupt instead of Critical_Enter global disable interrupt
	 *
	 * If calling thread is lower priority than ADC ISR, ADC ISR may occur after Conversion setup data is written by lower priority thread.
	 * In single threaded calling of Activate and calling thread priority is higher than ADC ISR, Activate will run to completion, overwriting the active conversion,
	 * Disable IRQ is not needed, however ADC ISR will still need Critical_Enter
	 */
	Analog_DisableInterrupt(p_analog);

	/*
	 * Multithreaded calling of Activate.
	 * Must implement Critical_Enter
	 *
	 * Higher priority thread may overwrite Conversion setup data before ADC ISR returns.
	 * e.g. must be implemented if calling from inside interrupts and main.
	 */
#if  (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_OS_HAL)) && !defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)
	Critical_Enter();
#endif

	// predetermine?
	//	p_analog->ActiveM 			= p_analog->ActiveTotal / adcCount;
	//	p_analog->ActiveRemainder 	= p_analog->ActiveTotal % adcCount; /* ideally uses the result of the division. */

	p_analog->p_ActiveConversion = p_conversion;
	p_analog->ActiveConversionChannelIndex = 0U;

#ifdef CONFIG_ANALOG_CHANNEL_FIXED
	for (uint8_t iAdc; iAdc < p_analog->AdcN_Count; iAdc++)
	{
		p_analog->p_ActiveConversionChannelIndexes[iAdc] = 0;
	}
#endif
	for (uint8_t index = 0U; (index < p_analog->AdcM_LengthBuffer) && (index < p_analog->p_ActiveConversion->ChannelCount); index++) //iChannel + p_analog->ActiveConversionIteration*p_analog->ADC_M_LengthBuffer
	{

	if (p_conversion->Config.UseConfig == 1U)
	{
		p_analog->ActiveConfig = p_conversion->Config;
	}

	ActivateConversion(p_analog, p_virtualChannels, activateChannelCount);

#if (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_OS_HAL)) && !defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)
	Critical_Exit();
#endif
	}
}


/*!
	 @brief Read last data captured by Analog_Capture_IO()
 */
analog_t Analog_ReadChannel(const Analog_T * p_analog, Analog_VirtualChannel_T channel)
{
	analog_t result;

	if (channel < p_analog->ChannelCount)
	{
		result = p_analog->p_MapChannelResults[(uint8_t)channel];
	}
	else
	{
		result = 0U;
	}

	return result;
	/* implement new data check
	 * set p_analog->p_VirtualChannelMapResults[(uint8_t)channel].NewDataFlag = false
	 * disable isr if writing completion flag
	 */
}

/*!
	 @brief Get pointer to channel result
 */
volatile analog_t * Analog_GetPtrChannelResult(const Analog_T * p_analog, Analog_VirtualChannel_T channel)
{
	volatile analog_t * p_result;

	if (channel < p_analog->ChannelCount)
	{
		p_result = &p_analog->p_MapChannelResults[(uint8_t)channel];
	}
	else
	{
		p_result = 0U;
	}

	return p_result;
}

/*!
	 @brief
 */
void Analog_ResetChannelResult(Analog_T * p_analog, Analog_VirtualChannel_T channel)
{
	p_analog->p_MapChannelResults[channel] = 0U;
}

/*!
	 @brief
 */
void Analog_Dectivate(const Analog_T * p_analog)
{
#if  defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	ADC_Dectivate(p_analog->p_Adc);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
	{
		HAL_ADC_Dectivate(p_analog->pp_Adcs[iAdc]);
	}
#endif
}

void Analog_DisableInterrupt(const Analog_T * p_analog)
{
#if  defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	HAL_ADC_DisableInterrupt(p_analog->p_Adc);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
	{
		HAL_ADC_DisableInterrupt(p_analog->pp_Adcs[iAdc]);
	}
#endif
}

static inline uint32_t ReadRequest(const Analog_T * p_analog, const HAL_ADC_T * p_adc, Analog_Request_T request)
{
	uint32_t response;

	switch (request)
	{
	case ANALOG_REQUEST_REG_CONVERSION_COMPLETE:
		response = (uint32_t) HAL_ADC_ReadConversionCompleteFlag(p_adc);
		break;
	default:
		response = 0;
		break;
	}

	return response;
}

static inline void WriteConfig(Analog_T * p_analog, HAL_ADC_T * p_adc, Analog_Config_T config)
{
	p_analog->ActiveConfig = config;

	if (config.UseInterrrupt)
	{
		HAL_ADC_EnableInterrupt(p_adc);
	}
	else
	{
		HAL_ADC_DisableInterrupt(p_adc);
	}
}

/*!
	 @brief
 */
void Analog_WriteConfig(Analog_T * p_analog, Analog_Config_T config)
{
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	WriteConfig(p_analog,  p_analog->p_Adc, config);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	for (uint8_t iAdc = 0U; iAdc < p_analog->AdcN_Count; iAdc++)
	{
		WriteConfig(p_analog, p_analog->pp_Adcs[iAdc], config);
	}
#endif
}
