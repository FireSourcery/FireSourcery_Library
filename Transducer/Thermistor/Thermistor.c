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
	@file  	Thermistor.c
	@author FireSourcery
	@brief
	@version V0
 */
/******************************************************************************/
#include "Thermistor.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

uint32_t Thermistor_Init(Thermistor_T * p_thermistor)
{
	if(p_thermistor->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_thermistor->Params, p_thermistor->CONFIG.P_PARAMS, sizeof(Thermistor_Params_T));
	}
}

Thermistor_ThesholdStatus_T Thermistor_ProcLimit(Thermistor_T * p_thermistor, uint16_t adcu)
{
	if(p_thermistor->Status == THERMISTOR_THRESHOLD_OK)
	{
		if(adcu > p_thermistor->Params.Limit_ADCU)
		{
			p_thermistor->Status = THERMISTOR_ERROR_LIMIT; //crossing shutdown
		}
	}
	else //over threshold or over limit
	{
		if(adcu > p_thermistor->Params.Limit_ADCU)
		{
			p_thermistor->Status = THERMISTOR_ERROR_LIMIT; //crossing shutdown
		}
		else if(adcu > p_thermistor->Params.Threshold_ADCU)
		{
			p_thermistor->Status = THERMISTOR_ERROR_THRESHOLD; //still over threshold
		}
		else
		{
			p_thermistor->Status = THERMISTOR_THRESHOLD_OK;
		}
	}

	return p_thermistor->Status;
}

static inline uint32_t GetRth_PullUp(Thermistor_T * p_thermistor, uint16_t adcu)
{
	/*
	 * Thermistor wired as bottom resistor R2
	 */
	// R2 = VOUT*R1 /(VIN-VOUT)
	// R2 = ((ADC*VREF)/ADC_MAX)*R1 / (VIN-((ADC*VREF)/ADC_MAX))
	// R2 = R1/(VIN*ADC_MAX/(ADC*VREF)-1)
	// R2 =	(R1*ADC*VREF)/(VIN*ADC_MAX - ADC*VREF)

	return ((uint32_t)adcu * (uint32_t)p_thermistor->CONFIG.R_SERIES * (uint32_t)p_thermistor->CONFIG.V_AREF) /
		 ((uint32_t)p_thermistor->CONFIG.V_IN * (uint32_t)p_thermistor->CONFIG.ADC_MAX - (uint32_t)adcu * (uint32_t)p_thermistor->CONFIG.V_AREF);

}

static inline uint32_t GetRth_PullDown(Thermistor_T * p_thermistor, uint16_t adcu)
{
	//	uint32_t rThermistor = p_thermistor->CONFIG.R_SERIES / (((uint32_t)p_thermistor->CONFIG.V_IN * (uint32_t)p_thermistor->CONFIG.ADC_RESOLUTION) / (*p_thermistor->CONFIG.P_ADC_VALUE * p_thermistor->CONFIG.ADC_VREF) - 1U);

}

static inline uint32_t GetRth_SeriesUpParallelDown(Thermistor_T * p_thermistor, uint16_t adcu)
{

}

static inline float Steinhart(Thermistor_T * p_thermistor, uint16_t rThermistor)
{
//	return (1.0F / (log((double)rThermistor / (double)p_thermistor->Params.RNominal) / (double)p_thermistor->Params.BConstant + 1.0F / (double)p_thermistor->Params.TNominal) - 273.15F);
}

static float ConvertToDegC(Thermistor_T * p_thermistor, uint16_t adcu)
{
	uint32_t rThermistor = GetRth_PullUp(p_thermistor, adcu);

	//switch(topology)
//	{
//
//	}

	float tSteinhart = Steinhart(p_thermistor, rThermistor);
	return tSteinhart;
}

static uint16_t ConvertToAdcu_DegC(Thermistor_T * p_thermistor, uint16_t degressC)
{
	uint16_t adcu;
	return adcu;
}


float Thermistor_ConvertToDegC_Float(Thermistor_T * p_thermistor, uint16_t adcu)
{
	return  ConvertToDegC(p_thermistor, adcu);
}


uint32_t Thermistor_ConvertToDegC_Round(Thermistor_T * p_thermistor, uint16_t adcu)
{
	return (uint32_t)ConvertToDegC(p_thermistor, adcu);
}

//uint32_t Thermistor_ConvertToDegC_NDecimal(Thermistor_T * p_thermistor, uint16_t adcu, uint8_t nDecimal)
//{
//	return (uint32_t)(ConvertToDegC(p_thermistor, adcu) * pow(10.0F, nDecimal));
//}

uint32_t Thermistor_ConvertToDegC_Scalar(Thermistor_T * p_thermistor, uint16_t adcu, uint16_t scalar)
{
	return (uint32_t)(ConvertToDegC(p_thermistor, adcu) * (float)scalar);
}

uint32_t Thermistor_ConvertToDegC_Fixed32(Thermistor_T * p_thermistor, uint16_t adcu)
{

}

uint32_t Thermistor_GetHeatLimit_DegC(Thermistor_T * p_thermistor)
{
	return ConvertToDegC(p_thermistor, p_thermistor->Params.Limit_ADCU);
}

uint32_t Thermistor_GetHeatLimit_DegCScalar(Thermistor_T * p_thermistor, uint16_t scalar)
{
	return (uint32_t)(ConvertToDegC(p_thermistor, p_thermistor->Params.Limit_ADCU) * scalar);
}

uint32_t Thermistor_GetHeatThreshold_DegC(Thermistor_T * p_thermistor)
{
	return ConvertToDegC(p_thermistor, p_thermistor->Params.Threshold_ADCU);
}

uint32_t Thermistor_GetHeatThreshold_DegCScalar(Thermistor_T * p_thermistor, uint16_t scalar)
{
	return (uint32_t)(ConvertToDegC(p_thermistor, p_thermistor->Params.Threshold_ADCU) * scalar);
}


uint32_t Thermistor_SetHeatLimit_DegC(Thermistor_T * p_thermistor, uint8_t limit_degreesC)
{
	p_thermistor->Params.Limit_ADCU = ConvertToAdcu_DegC(p_thermistor, limit_degreesC);
}

uint32_t Thermistor_SetHeatThreshold_DegC(Thermistor_T * p_thermistor, uint8_t threshold_degreesC)
{
	p_thermistor->Params.Threshold_ADCU = ConvertToAdcu_DegC(p_thermistor, threshold_degreesC);
}

