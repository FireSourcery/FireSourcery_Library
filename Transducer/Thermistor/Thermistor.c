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
#include <string.h>
#include <math.h>

static uint16_t Thermistor_AdcVRef_Scalar; /* Shared by all instances. */

/*
	Set Vin to same decimal precision as AdcVRef
*/
void Thermistor_InitAdcVRef_Scalar(uint16_t adcVRef_MilliV)
{
	Thermistor_AdcVRef_Scalar = adcVRef_MilliV / 10U;
}

void Thermistor_Init(Thermistor_T * p_therm)
{
	if(p_therm->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_therm->Params, p_therm->CONFIG.P_PARAMS, sizeof(Thermistor_Params_T));
	}

	p_therm->ThresholdStatus = THERMISTOR_STATUS_OK;
	p_therm->Heat_DegC = 0;
}

/*
	Smaller adcu is higher heat
*/
// Thermistor_Status_T Thermistor_PollLimitThesholdMonitor(Thermistor_T * p_therm, uint16_t adcu)
// {
	// if(p_therm->Params.IsEnable)
	// {
	// 		if		(adcu < p_therm->Params.Limit_Adcu) 		{ p_therm->ThresholdStatus = THERMISTOR_LIMIT_SHUTDOWN; } 		/* crossing shutdown */

	// 	if(p_therm->ThresholdStatus == THERMISTOR_STATUS_OK)	/* current state is ok, check limit only */
	// 	{ 
	// 	}
	// 	else	/* current state is not ok, check threshold and limit */
	// 	{

	// 		else if	(adcu < p_therm->Params.Threshold_Adcu) 	{ p_therm->ThresholdStatus = THERMISTOR_LIMIT_THRESHOLD; } 	/* still over threshold  */
	// 		else 												{ p_therm->ThresholdStatus = THERMISTOR_STATUS_OK; }
	// 	}

	// 	if(adcu < p_therm->Params.Limit_Adcu) { p_therm->ThresholdStatus = THERMISTOR_LIMIT_SHUTDOWN; }	 

	// 	else if(p_therm->ThresholdStatus == THERMISTOR_LIMIT_THRESHOLD)	 
	// 	{
	// 		if(adcu > p_therm->Params.Threshold_Adcu) //under therhold 
	// 		{
	// 			p_therm->ThresholdStatus = THERMISTOR_LIMIT_THRESHOLD; 
	// 		} 	/* still over threshold  */

	// 		else { p_therm->ThresholdStatus = THERMISTOR_STATUS_OK; }
	// 	}
	// 	else
	// 	{
	// 			//check  warning
	// 	}

	// 	if(adcu < p_therm->Params.Limit_Adcu) { p_therm->ThresholdStatus = THERMISTOR_LIMIT_SHUTDOWN; }	/* crossing shutdown */
	// 	{		

	// 	}
	// 	else if(p_therm->ThresholdStatus == THERMISTOR_STATUS_OK)	/* current state is ok, check limit only */
	// 	{
	// 		// chck waring
	// 		if	(adcu < p_therm->Params.Threshold_Adcu) 	{ p_therm->ThresholdStatus = THERMISTOR_LIMIT_THRESHOLD; } 	/* still over threshold  */
	// 		else 											{ p_therm->ThresholdStatus = THERMISTOR_STATUS_OK; }
	// 	}
	// 	else	//threhold, check threshold,
	// 	{
	// 		if		(adcu < p_therm->Params.Limit_Adcu) 		{ p_therm->ThresholdStatus = THERMISTOR_LIMIT_SHUTDOWN; } 		/* crossing shutdown */
	// 		else if	(adcu < p_therm->Params.Threshold_Adcu) 	{ p_therm->ThresholdStatus = THERMISTOR_LIMIT_THRESHOLD; } 	/* still over threshold  */
	// 		else 												{ p_therm->ThresholdStatus = THERMISTOR_STATUS_OK; }
	// 	}

	// }

// 	return p_therm->ThresholdStatus;
// }

// Thermistor_Status_T Thermistor_ProcStatus(Thermistor_T * p_therm, uint16_t adcu)
// {
// 	Thermistor_Status_T status = Thermistor_ProcThreshold(p_therm, adcu);

// 	if(p_therm->Params.IsEnable)
// 	{
// 		if((status == THERMISTOR_STATUS_OK) && (adcu < p_therm->Params.Warning_Adcu))
// 		{
// 			status = THERMISTOR_WARNING;
// 		}
// 	}

// 	return status;
// }


Thermistor_Status_T Thermistor_PollMonitor(Thermistor_T * p_therm, uint16_t adcu)
{

}

// Thermistor_Status_T Thermistor_GetWarningStatus(Thermistor_T * p_therm)
// {
// 	// return (adcu < p_therm->Params.Warning_Adcu) ? THERMISTOR_WARNING : THERMISTOR_STATUS_OK;
// }

// Thermistor_Status_T Thermistor_GetStatus(Thermistor_T * p_therm)
// {
// 	// Thermistor_Status_T status = p_therm->ThresholdStatus;

// 	// if((status == THERMISTOR_STATUS_OK) && (adcu < p_therm->Params.Warning_Adcu))
// 	// {
// 	// 	status = THERMISTOR_WARNING;
// 	// }

// 	// return status;

// 	// 	return (p_therm->ThresholdStatus == THERMISTOR_STATUS_OK) : ;
// }

/*
	For when conversion processing is less frequent than user output
*/
void Thermistor_CaptureUnits_DegC(Thermistor_T * p_therm, uint16_t adcu)
{
	if(p_therm->Params.IsEnable)
	{
		p_therm->Heat_DegC = Thermistor_ConvertToDegC_Int(p_therm, adcu, p_therm->Params.CaptureScalar);
	}
}

/*
	Thermistor wired as pull-down resistor R2
	RSeries wired as pull-up resistor R1

	R2 = VOUT*R1/(VIN-VOUT)
	R2 = (ADC*VREF/ADC_MAX)*R1 / (VIN-(ADC*VREF/ADC_MAX))
	R2 = R1/(VIN*ADC_MAX/(ADC*VREF)-1)
	R2 = (R1*ADC*VREF)/(VIN*ADC_MAX - ADC*VREF)
*/
static inline uint32_t r_pulldown_adcu(uint32_t rPullUp, uint16_t vIn, uint16_t adcMax, uint16_t adcVRef, uint16_t adcu)
{
	return ((double)rPullUp * adcVRef * adcu) / (vIn * adcMax - adcVRef * adcu);
}

static inline uint32_t r_pullup_adcu(uint32_t rPullDown, uint16_t vIn, uint16_t adcMax, uint16_t adcVRef, uint16_t adcu)
{
	return rPullDown * (((double)vIn * adcMax) / (adcVRef * adcu) - 1U);
}

static inline uint16_t adcu_r(uint16_t adcMax, uint16_t adcVRef, uint16_t vIn, uint32_t rPullUp, uint32_t rPullDown)
{
	return ((double)vIn * adcMax * rPullDown) / ((double)adcVRef * (rPullUp + rPullDown));
}

static inline uint32_t r_net(uint32_t rParallel1, uint32_t rParallel2)
{
	return ((double)rParallel1 * rParallel2) / (rParallel1 + rParallel2); 	/* 1 / (1/r1 + 1/r2) */
}

static inline uint32_t r_parallel(uint32_t rNet, uint32_t rParallel)
{
	return ((double)rNet * rParallel) / (rNet - rParallel); 				/* 1 / (1/rNet - 1/rParallel) */
}

/*
	1/T = 1/T0 + (1/B)*ln(R/R0)
	return 1/T
*/
static inline float steinhart(double b, double t0, double r0, double rTh)
{
	return (log(rTh / r0) / b) + (1.0F / t0);
}

/*
	return RTh
*/
static inline float invsteinhart(double b, double t0, double r0, double Inv)
{
	return exp((Inv - 1.0F / t0) * b) * r0;
}

static float ConvertAdcuToDegC(Thermistor_T * p_therm, uint16_t adcu)
{
	uint32_t rNet = r_pulldown_adcu(p_therm->CONFIG.R_SERIES/100U, p_therm->Params.VIn_Scalar, ADC_MAX, Thermistor_AdcVRef_Scalar, adcu) * 100U;
	uint32_t rTh = (p_therm->CONFIG.R_PARALLEL != 0U) ? r_parallel(rNet, p_therm->CONFIG.R_PARALLEL) : rNet;
	double Inv = steinhart(p_therm->Params.BConstant, p_therm->Params.TNominal, p_therm->Params.RNominal, rTh);
	return (1.0F / Inv - 273.15F);
}

static uint16_t ConvertDegCToAdcu(Thermistor_T * p_therm, uint16_t degreesC)
{
	double Inv = (double)1.0F / (degreesC + 273.15F);
	uint32_t rTh = invsteinhart(p_therm->Params.BConstant, p_therm->Params.TNominal, p_therm->Params.RNominal, Inv);
	uint32_t rNet = (p_therm->CONFIG.R_PARALLEL != 0U) ? r_net(p_therm->CONFIG.R_PARALLEL, rTh) : rTh;
	return adcu_r(ADC_MAX, Thermistor_AdcVRef_Scalar, p_therm->Params.VIn_Scalar, p_therm->CONFIG.R_SERIES, rNet);
}

float Thermistor_ConvertToDegC_Float(Thermistor_T * p_therm, uint16_t adcu)
{
	return ConvertAdcuToDegC(p_therm, adcu);
}

int32_t Thermistor_ConvertToDegC_Int(Thermistor_T * p_therm, uint16_t adcu, uint16_t scalar)
{
	return (int32_t)(ConvertAdcuToDegC(p_therm, adcu) * scalar);
}

uint16_t Thermistor_ConvertToAdcu_DegC(Thermistor_T * p_therm, uint16_t degreesC)
{
	return ConvertDegCToAdcu(p_therm, degreesC); 
}


/******************************************************************************/
/*!
	Set Params
*/
/******************************************************************************/
void Thermistor_SetRThValues_DegC(Thermistor_T * p_therm, uint32_t r0, uint32_t t0_degC, uint32_t b, uint8_t threshold_degC, uint8_t limit_degC)
{
	p_therm->Params.RNominal = r0;
	p_therm->Params.TNominal = t0_degC + 273;
	p_therm->Params.BConstant = b;
}

void Thermistor_SetVIn_MilliV(Thermistor_T * p_therm, uint32_t vIn_MilliV)
{
	p_therm->Params.VIn_Scalar = vIn_MilliV / 10U;
}

uint32_t Thermistor_GetR0(Thermistor_T * p_therm) { return p_therm->Params.RNominal; }
uint32_t Thermistor_GetT0(Thermistor_T * p_therm) { return p_therm->Params.TNominal; }
uint32_t Thermistor_GetB(Thermistor_T * p_therm) { return p_therm->Params.BConstant; }
uint32_t Thermistor_GetVIn(Thermistor_T * p_therm) { return p_therm->Params.VIn_Scalar; }

static uint16_t ConvertDegCToAdcu_SetUser(Thermistor_T * p_therm, uint8_t degreesC)
{
	uint16_t adcu = ConvertDegCToAdcu(p_therm, degreesC);
	while(((uint16_t)ConvertAdcuToDegC(p_therm, adcu) > degreesC) && (adcu < ADC_MAX))
	{ 
		adcu += 1U; 
	}
	//while(ConvertAdcuToDegC(p_therm, adcu) < degreesC) { adcu -= 1U; }
	return adcu;
}

void Thermistor_SetLimit_DegC(Thermistor_T * p_therm, uint8_t limit_degreesC) 			{ p_therm->Params.Limit_Adcu = ConvertDegCToAdcu_SetUser(p_therm, limit_degreesC); }
void Thermistor_SetThreshold_DegC(Thermistor_T * p_therm, uint8_t threshold_degreesC) 	{ p_therm->Params.Threshold_Adcu = ConvertDegCToAdcu_SetUser(p_therm, threshold_degreesC); }
void Thermistor_SetWarning_DegC(Thermistor_T * p_therm, uint8_t warning_degreesC) 		{ p_therm->Params.Warning_Adcu = ConvertDegCToAdcu_SetUser(p_therm, warning_degreesC); }

int32_t Thermistor_GetLimit_DegC(Thermistor_T * p_therm) 		{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Limit_Adcu); }
int32_t Thermistor_GetThreshold_DegC(Thermistor_T * p_therm) 	{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Threshold_Adcu); }
int32_t Thermistor_GetWarning_DegC(Thermistor_T * p_therm) 		{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Warning_Adcu); }

int32_t Thermistor_GetLimit_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 		{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.Limit_Adcu, scalar); }
int32_t Thermistor_GetThreshold_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 	{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.Threshold_Adcu, scalar); }
int32_t Thermistor_GetWarning_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 		{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.Warning_Adcu, scalar); }

float Thermistor_GetLimit_DegCFloat(Thermistor_T * p_therm) 		{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Limit_Adcu); }
float Thermistor_GetThreshold_DegCFloat(Thermistor_T * p_therm) 	{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Threshold_Adcu); }
float Thermistor_GetWarning_DegCFloat(Thermistor_T * p_therm) 		{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Warning_Adcu); }
