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
    @file 	AnalogN.h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#include "AnalogN.h"
#include "../Analog/Analog.h"

#include <stdint.h>
#include <stdbool.h>

void AnalogN_Init(AnalogN_T * p_analogn)
{
	Analog_T * p_analogI;

	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		p_analogI = &p_analogn->CONFIG.P_ANALOGS[iAdc];
		Analog_Init(p_analogI);
	}
}

static volatile uint32_t debug;

typedef enum
{
	ANALOG_ACTIVATE_MODE_ENQUEUE_FRONT,
	ANALOG_ACTIVATE_MODE_ENQUEUE_BACK,
	ANALOG_ACTIVATE_MODE_OVERWRITE,
}
Analog_ActivateMode_T;

//todo when to overwrite continuous conversion
/*
 * Enqeue gurantees condition to start interrupt is there currently no interrupt expected, i.e no conversion active, or interrupt pending
 * Oncomplete ISR gurantees condition set active flag false, or start new conversion
 * These 2 condition should guarantee chaining of queue
 */
static inline void EnqueueConversionCommon(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion, Analog_ActivateMode_T mode)
{
	const Analog_ConversionMap_T * p_map 		= &p_conversion->MAP;
	AnalogN_AdcFlags_T * p_signalBuffer 		= p_conversion->P_SIGNAL_BUFFER;
	const Analog_ConversionAdc_T * p_adcConversion;
	Analog_T * p_analogI;


	//Assume only 1 copy of conversion in queue, additional copies may run oncomplete with partial completion
	//alternatively do not activate if not all adcs completed selected conversion + handle case of aborted conversion
	//change to pointer to signal queue if use multiple copies of same conversion

	if (p_signalBuffer->AdcFlags != 0U)
	{
		debug++;
	}

	p_signalBuffer->AdcFlags = p_conversion->ANALOGS_ACTIVE.AdcFlags;

	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
	{
		p_adcConversion = &p_conversion->P_ADC_CONVERSIONS[iAdc];
		if (p_adcConversion->P_CHANNELS != 0U)
		{
			p_analogI = &p_analogn->CONFIG.P_ANALOGS[iAdc];
			_Analog_EnterCritical(p_analogI);

//			if(mode == ANALOG_ACTIVATE_MODE_OVERWRITE)
//			{
//				p_analogI->p_ActiveConversion = p_conversion;
//				_Analog_ActivateConversion(p_analogI, p_adcConversion, p_map);
//			}
//			else
			{
				if (_Analog_ReadIsActive(p_analogI) == false) //check q empty is not checked at end of conversion ((Queue_GetIsEmpty(&p_analog->ConversionQueue) == false) && (_Analog_ReadIsActive(p_analog) == false))
				{
					p_analogI->p_ActiveConversion = p_conversion;
					_Analog_ActivateConversion(p_analogI, p_adcConversion, p_map);
					debug = 0;
				}
				else
				{
					switch(mode)
					{
						case ANALOG_ACTIVATE_MODE_ENQUEUE_FRONT:
							if (Queue_GetIsFull(&p_analogI->ConversionQueue) == true)
							{
								Queue_RemoveBack(&p_analogI->ConversionQueue, 1U);
							}
							Queue_PushFront(&p_analogI->ConversionQueue, &p_conversion);
							break;
						case ANALOG_ACTIVATE_MODE_ENQUEUE_BACK:
							Queue_Enqueue(&p_analogI->ConversionQueue, &p_conversion);
							break;
						default: break;
					}
					debug++;
				}

				//inclue dequeu if need to account for aborted conversions
			}

			_Analog_ExitCritical(p_analogI);
		}
	}

}


/*
 */
void AnalogN_ActivateConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
	EnqueueConversionCommon(p_analogn, p_conversion, ANALOG_ACTIVATE_MODE_OVERWRITE);
}

void AnalogN_EnqueueConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
	EnqueueConversionCommon(p_analogn, p_conversion, ANALOG_ACTIVATE_MODE_ENQUEUE_BACK);
}

void AnalogN_EnqueueFrontConversion(AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
	EnqueueConversionCommon(p_analogn, p_conversion, ANALOG_ACTIVATE_MODE_ENQUEUE_FRONT);
}

//void AnalogN_PollDequeueConversion(AnalogN_T * p_analogn)
//{
//	Analog_T * p_analogI;
//	const AnalogN_Conversion_T * p_conversion;
//	const Analog_ConversionAdc_T * p_adcConversion;
//	const Analog_ConversionMap_T * p_map;
//
//	for (uint8_t iAdc = 0U; iAdc < p_analogn->CONFIG.ANALOG_COUNT; iAdc++)
//	{
//		p_analogI = &p_analogn->CONFIG.P_ANALOGS[iAdc];
//		_Analog_EnterCritical(p_analogI);
//		if ((Queue_GetIsEmpty(&p_analogI->ConversionQueue) == false) && (_Analog_ReadIsActive(p_analogI) == false))
//		{
//			Queue_PeekFront(&p_analogI->ConversionQueue, &p_conversion);
//			p_adcConversion = &p_conversion->P_ADC_CONVERSIONS[iAdc];
//			p_map = &p_conversion->MAP;
//			_Analog_ActivateConversion(p_analogI, p_adcConversion, p_map);
//		}
//		_Analog_ExitCritical(p_analogI);
//	}
//}


