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
	@file 	Analog.c
	@author FireSoucery
	@brief 	ADC wrapper module. Implements run time configurable settings.
    		Persistent setting delegated to outer module with scope of hardware.
	@version V0
*/
/**************************************************************************/
#include "Analog.h"
#include "Private.h"
#include "Config.h"

/*!
	 @brief Initialize struct object
 */
void Analog_Init
(
	Analog_T * p_analog,
	volatile void const * p_adcMap,
	uint8_t nAdc,
	uint8_t mHwBuffer,
	const adcpin_t * p_virtualChannelMapPinsBuffer,
	volatile analog_t * p_virtualChannelMapResultsBuffer,
	uint8_t virtualChannelCount
)
{
	if (nAdc > 1)
	{
		p_analog->pp_AdcRegisterMaps = (volatile void (* const (*))) p_adcMap;
	}
	{
		p_analog->p_AdcRegisterMap = (volatile void *) p_adcMap;
	}

	p_analog->N_Adc 		= nAdc;
	p_analog->M_HwBuffer 	= mHwBuffer;

	p_analog->p_VirtualChannelMapPins 		= p_virtualChannelMapPinsBuffer;
	p_analog->p_VirtualChannelMapResults 	= p_virtualChannelMapResultsBuffer;
	p_analog->VirtualChannelMapLength		= virtualChannelCount;
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
		p_virtualChannels = &p_conversion->p_VirtualChannels[0];
	}
	else
	{
		p_virtualChannels = &p_conversion->VirtualChannel;
	}

	activateChannelCount = GetActivateChannelCount(p_analog, p_conversion->ChannelCount);

	/*
	 * Single threaded calling of Activate.
	 * Single threaded case, and calling thread is lower priority than ADC ISR, may implement ADC_DisableInterrupt instead of Critical_Enter global disable interrupt
	 *
	 * If calling thread is lower priority than ADC ISR, ADC ISR may occur after Conversion setup data is written by lower priority thread.
	 * In single threaded calling of Activate and calling thread priority is higher than ADC ISR, Activate will run to completion, overwriting the active conversion,
	 * Disable IRQ is not needed, however ADC ISR will still need Critical_Enter
	 */
	ADC_DisableInterrupt(p_analog);

	/*
	 * Multithreaded calling of Activate.
	 * Must implement Critical_Enter
	 *
	 * Higher priority thread may overwrite Conversion setup data before ADC ISR returns.
	 * e.g. must be implemented if calling from inside interrupts and main.
	 */
#if !defined(CONFIG_ANALOG_MULTITHREADED_DISABLE) /* && (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_OS_HAL)) */
	Critical_Enter();
#endif

	// predetermine?
	//	p_analog->ActiveM 			= p_analog->ActiveTotal / adcCount;
	//	p_analog->ActiveRemainder 	= p_analog->ActiveTotal % adcCount; /* ideally uses the result of the division. */

	p_analog->p_ActiveConversion = p_conversion;
	p_analog->ActiveConversionChannelIndex = 0;

	if (p_conversion->Config.UseConfig == 1U)
	{
		p_analog->ActiveConfig = p_conversion->Config;
	}

	ActivateConversion(p_analog, p_virtualChannels, activateChannelCount);

#if !defined(CONFIG_ANALOG_MULTITHREADED_DISABLE) /* && (defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED) || defined(CONFIG_ANALOG_MULTITHREADED_OS_HAL)) */
	Critical_Exit();
#endif
}


/*!
	 @brief Read last data captured by Analog_Capture_IO()
 */
analog_t Analog_ReadChannel(const Analog_T * p_analog, Analog_VirtualChannel_T channel)
{
	analog_t result;

	if (channel < p_analog->VirtualChannelMapLength)
	{
		result = p_analog->p_VirtualChannelMapResults[(uint8_t)channel];
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

	if (channel < p_analog->VirtualChannelMapLength)
	{
		p_result = &p_analog->p_VirtualChannelMapResults[(uint8_t)channel];
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
	p_analog->p_VirtualChannelMapResults[channel] = 0U;
}

/*!
	 @brief
 */
void Analog_Dectivate(const Analog_T * p_analog)
{
#if  defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	ADC_Dectivate(p_analog->p_AdcRegisterMap);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	for (uint8_t iAdc = 0U; iAdc < p_analog->N_Adc; iAdc++)
	{
		ADC_Dectivate(p_analog->pp_AdcRegisterMaps[iAdc]);
	}
#endif
}

/*!
	 @brief
 */
void Analog_WriteConfig(const Analog_T * p_analog, Analog_Config_T config)
{
#if defined(CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)
	ADC_WriteConfig(p_analog->p_AdcRegisterMap, config);
#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER) || defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)
	for (uint8_t iAdc = 0U; iAdc < p_analog->N_Adc; iAdc++)
	{
		ADC_WriteConfig(p_analog->pp_AdcRegisterMaps[iAdc], config);
	}
#endif
}
