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


void Thermistor_Init(Thermistor_T * p_thermistor)
{
	if(p_thermistor->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_thermistor->Params, p_thermistor->CONFIG.P_PARAMS, sizeof(Thermistor_Params_T));
	}

	p_thermistor->Status = THERMISTOR_THRESHOLD_OK;
	p_thermistor->Heat_DegCInt = 0;
}



/*
	Thermistor wired as bottom resistor R2
	R2 = VOUT*R1/(VIN-VOUT)
	R2 = (ADC*VREF/ADC_MAX)*R1 / (VIN-(ADC*VREF/ADC_MAX))
	R2 = R1/(VIN*ADC_MAX/(ADC*VREF)-1)
	R2 = (R1*ADC*VREF)/(VIN*ADC_MAX - ADC*VREF)
 */
static inline uint32_t Rth_PullUp(uint8_t vIn, uint32_t rSeries, uint32_t rParallel, uint8_t adcVRef, uint16_t adcMax, uint16_t adcu)
{
	uint32_t rThermistor = (rSeries * adcVRef * adcu) / (vIn * adcMax - adcVRef * adcu);

	if (rParallel != 0U)
	{

	}

	return rThermistor;
}

static inline uint16_t InvRth_PullUp(uint8_t vIn, uint32_t rSeries, uint32_t rParallel, uint8_t adcVRef, uint16_t adcMax, uint32_t rTh)
{
	uint16_t adcu = (vIn * adcMax * rTh) / (adcVRef * (rSeries + rTh));

	if (rParallel != 0U)
	{

	}

	return adcu;
}

//#define THERMISTOR_STEINHART(B, T0, R0, R_TH) (log(R_TH / R0) / B) + (1.0F / T0)
//#define THERMISTOR_ADCU_PULL_UP(B, T0, R0, R_TH)  THERMISTOR_INV_R_TH_PULLUP( , THERMISTOR_INV_STEINHART(B, T0, R0, 1.0F / (DEG_C + 273.15F)))


/*
 * return 1/T
 */
static inline float Steinhart(double b, double t0, double r0, double rTh)
{
	return (log(rTh / r0) / b) + (1.0F / t0);
}

/*
 * return Rth
 */
static inline float InvSteinhart(double b, double t0, double r0, double heatInv)
{
	return exp((heatInv - 1.0F / t0) * b) * r0;
}

//static inline uint32_t CalcRth_PullUp(Thermistor_T * p_thermistor, uint16_t adcu)
//{
//	uint32_t rThermistor = ((uint32_t)adcu * p_thermistor->CONFIG.R_SERIES * (uint32_t)p_thermistor->CONFIG.ADC_VREF) /
//		 ((uint32_t)p_thermistor->CONFIG.V_IN * (uint32_t)p_thermistor->CONFIG.ADC_MAX - (uint32_t)adcu * (uint32_t)p_thermistor->CONFIG.ADC_VREF);
//
//	if (p_thermistor->CONFIG.R_PARALLEL != 0U)
//	{
//
//	}
//
//	return rThermistor;
//}

//static inline uint32_t CalcRth_PullDown(Thermistor_T * p_thermistor, uint16_t adcu)
//{
//	//	uint32_t rThermistor = p_thermistor->CONFIG.R_SERIES / (((uint32_t)p_thermistor->CONFIG.V_IN * (uint32_t)p_thermistor->CONFIG.ADC_RESOLUTION) / (*p_thermistor->CONFIG.P_ADC_VALUE * p_thermistor->CONFIG.ADC_VREF) - 1U);
//}

//static inline float Steinhart(Thermistor_T * p_thermistor, uint16_t rTh)
//{
//	return (log((double)rTh / (double)p_thermistor->Params.RNominal) / (double)p_thermistor->Params.BConstant + 1.0F / (double)p_thermistor->Params.TNominal);
//}

//static inline float InvSteinhart(Thermistor_T * p_thermistor, uint16_t heatInv)
//{
//	return exp((heatInv - 1.0F / (double)p_thermistor->Params.TNominal) * (double)p_thermistor->Params.BConstant) * (double)p_thermistor->Params.RNominal;
//}



static float ConvertAdcuToDegC(Thermistor_T * p_thermistor, uint16_t adcu)
{
	uint32_t rThermistor = 	Rth_PullUp
							(
								p_thermistor->CONFIG.V_IN,
								p_thermistor->CONFIG.R_SERIES,
								p_thermistor->CONFIG.R_PARALLEL,
								p_thermistor->CONFIG.ADC_VREF,
								p_thermistor->CONFIG.ADC_MAX,
								adcu
							);

	double heatInv = Steinhart(p_thermistor->Params.BConstant, p_thermistor->Params.TNominal, p_thermistor->Params.RNominal, rThermistor);

	return (1.0F / heatInv - 273.15F);
}

static uint16_t ConvertDegCToAdcu(Thermistor_T * p_thermistor, uint16_t degressC)
{
	double heatInv = 1.0F / (degressC + 273.15F);
	uint32_t rTh = InvSteinhart(p_thermistor->Params.BConstant, p_thermistor->Params.TNominal, p_thermistor->Params.RNominal, heatInv);

	return 	InvRth_PullUp
			(
				p_thermistor->CONFIG.V_IN,
				p_thermistor->CONFIG.R_SERIES,
				p_thermistor->CONFIG.R_PARALLEL,
				p_thermistor->CONFIG.ADC_VREF,
				p_thermistor->CONFIG.ADC_MAX,
				rTh
			);
}

float Thermistor_ConvertToDegC_Float(Thermistor_T * p_thermistor, uint16_t adcu)
{
	return ConvertAdcuToDegC(p_thermistor, adcu);
}

int32_t Thermistor_ConvertToDegC_Int(Thermistor_T * p_thermistor, uint16_t adcu, uint16_t scalar)
{
	return (int32_t)(ConvertAdcuToDegC(p_thermistor, adcu) * scalar);
}

//int32_t Thermistor_ConvertToDegC_Fixed32(Thermistor_T * p_thermistor, uint16_t adcu)
//{
//
//}

/*
 * For when conversion processing is less frequent than user output
 */
void Thermistor_CaptureUnitConversion(Thermistor_T * p_thermistor, uint16_t adcu)
{
	if (p_thermistor->Params.IsEnable)
	{
		p_thermistor->Heat_DegCInt = Thermistor_ConvertToDegC_Int(p_thermistor, adcu, p_thermistor->Params.IntScalar);
	}
}

/*
 * Smaller adcu is high heat
 */
Thermistor_ThesholdStatus_T Thermistor_ProcThreshold(Thermistor_T * p_thermistor, uint16_t adcu)
{
	if (p_thermistor->Params.IsEnable)
	{
		if(p_thermistor->Status == THERMISTOR_THRESHOLD_OK)
		{
			/* current state is ok, check limit only */
			if(adcu < p_thermistor->Params.Limit_ADCU)
			{
				p_thermistor->Status = THERMISTOR_ERROR_LIMIT; //crossing shutdown
			}
		}
		else
		{
			/* current state is not ok, check threshold and limit */
			if(adcu < p_thermistor->Params.Limit_ADCU)
			{
				p_thermistor->Status = THERMISTOR_ERROR_LIMIT; //crossing shutdown
			}
			else if(adcu < p_thermistor->Params.Threshold_ADCU)
			{
				p_thermistor->Status = THERMISTOR_ERROR_THRESHOLD; //still over threshold
			}
			else
			{
				p_thermistor->Status = THERMISTOR_THRESHOLD_OK;
			}
		}
	}

	return p_thermistor->Status;
}

void Thermistor_SetHeatLimit_DegC(Thermistor_T * p_thermistor, uint8_t limit_degreesC)
{
	p_thermistor->Params.Limit_ADCU = ConvertDegCToAdcu(p_thermistor, limit_degreesC);
}

void Thermistor_SetHeatThreshold_DegC(Thermistor_T * p_thermistor, uint8_t threshold_degreesC)
{
	p_thermistor->Params.Threshold_ADCU = ConvertDegCToAdcu(p_thermistor, threshold_degreesC);
}

void Thermistor_SetParams_DegC(Thermistor_T * p_thermistor, uint32_t r0, uint32_t t0, uint32_t b, uint8_t threshold_degC, uint8_t limit_degC)
{
	Thermistor_SetHeatLimit_DegC(p_thermistor, limit_degC);
	Thermistor_SetHeatThreshold_DegC(p_thermistor, threshold_degC);
	p_thermistor->Params.RNominal = r0;
//	p_thermistor->Params.TNominal = t0 + 273;
	p_thermistor->Params.BConstant = b;
}


int32_t Thermistor_GetHeatLimit_DegCInt(Thermistor_T * p_thermistor, uint16_t scalar)
{
	return Thermistor_ConvertToDegC_Int(p_thermistor, p_thermistor->Params.Limit_ADCU, scalar);
}

int32_t Thermistor_GetHeatThreshold_DegCInt(Thermistor_T * p_thermistor, uint16_t scalar)
{
	return Thermistor_ConvertToDegC_Int(p_thermistor, p_thermistor->Params.Threshold_ADCU, scalar);
}

//int32_t Thermistor_GetHeatLimit_DegC(Thermistor_T * p_thermistor)
//{
//	return ConvertAdcuToDegC(p_thermistor, p_thermistor->Params.Limit_ADCU);
//}
//int32_t Thermistor_GetHeatThreshold_DegC(Thermistor_T * p_thermistor)
//{
//	return ConvertAdcuToDegC(p_thermistor, p_thermistor->Params.Threshold_ADCU);
//}

//float Thermistor_GetHeatLimit_DegCFloat(Thermistor_T * p_thermistor)
//{
//	return ConvertAdcuToDegC(p_thermistor, p_thermistor->Params.Limit_ADCU);
//}
//
//float Thermistor_GetHeatThreshold_DegCFloat(Thermistor_T * p_thermistor, uint16_t scalar)
//{
//	return ConvertAdcuToDegC(p_thermistor, p_thermistor->Params.Threshold_ADCU);
//}
