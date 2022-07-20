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

static uint16_t _AdcVRef_Scalar; /* Shared by all instances. */

/*
	Set Vin to same decimal precision as AdcVRef
*/
void Thermistor_InitAdcVRef_Scalar(uint16_t adcVRef_MilliV)
{
	_AdcVRef_Scalar = adcVRef_MilliV / 10U;
}

void Thermistor_Init(Thermistor_T * p_therm)
{
	if(p_therm->CONFIG.P_PARAMS != 0U) { memcpy(&p_therm->Params, p_therm->CONFIG.P_PARAMS, sizeof(Thermistor_Params_T)); }

	p_therm->Status = THERMISTOR_STATUS_OK;
	p_therm->ShutdownThreshold = THERMISTOR_THRESHOLD_OK;
	p_therm->WarningThreshold = THERMISTOR_THRESHOLD_OK;

	if
	(
		(p_therm->Params.Shutdown_Adcu == 0U) || (p_therm->Params.ShutdownThreshold_Adcu == 0U) ||
		(p_therm->Params.Warning_Adcu == 0U) || (p_therm->Params.WarningThreshold_Adcu == 0U)
	)
	{
		p_therm->Params.IsMonitorEnable = false;
	}
}

/******************************************************************************/
/*!
	Limits Monitor
*/
/******************************************************************************/
/*
	Lower adcu is higher heat
*/
Thermistor_ThresholdStatus_T PollThreshold(Thermistor_ThresholdStatus_T status, uint16_t limit, uint16_t threshold, uint16_t adcu)
{
	if(status == THERMISTOR_THRESHOLD_OK)
	{
		if(adcu < limit) { status = THERMISTOR_THRESHOLD_LIMIT; } /* crossing limit */
	}
	else /* if(p_therm->ShutdownThreshold == THERMISTOR_STATUS_SHUTDOWN) || THERMISTOR_SHUTDOWN_THRESHOLD */
	{
		/* in case if heat sample out of sync, repeat check */
		if		(adcu < limit) 			{ status = THERMISTOR_THRESHOLD_LIMIT; } 	/* crossing limit */
		else if	(adcu < threshold) 		{ status = THERMISTOR_THRESHOLD_CONTINUE; } /* still over threshold  */
		else 							{ status = THERMISTOR_THRESHOLD_OK; }
	}

	return status;
}

/*!
	@return
*/
Thermistor_Status_T Thermistor_PollMonitor(Thermistor_T * p_therm, uint16_t captureAdcu)
{
	if(p_therm->Params.IsMonitorEnable == true)
	{
		p_therm->Adcu = (captureAdcu + p_therm->Adcu) / 2U;
		// p_therm->Adcu = captureAdcu;
		p_therm->ShutdownThreshold = PollThreshold(p_therm->ShutdownThreshold, p_therm->Params.Shutdown_Adcu, p_therm->Params.ShutdownThreshold_Adcu, p_therm->Adcu);
		p_therm->Status = (p_therm->ShutdownThreshold == THERMISTOR_THRESHOLD_OK) ? THERMISTOR_STATUS_OK : THERMISTOR_STATUS_SHUTDOWN;

		if(p_therm->Status == THERMISTOR_STATUS_OK)
		{
			p_therm->WarningThreshold = PollThreshold(p_therm->WarningThreshold, p_therm->Params.Warning_Adcu, p_therm->Params.WarningThreshold_Adcu, p_therm->Adcu);
			p_therm->Status = (p_therm->WarningThreshold == THERMISTOR_THRESHOLD_OK) ? THERMISTOR_STATUS_OK : THERMISTOR_STATUS_WARNING;
		}
	}

	return p_therm->Status;
}

/******************************************************************************/
/*!
	Set Params
*/
/******************************************************************************/
void Thermistor_SetNtc(Thermistor_T * p_therm, uint32_t r0, uint32_t t0_degC, uint32_t b)
{
	p_therm->Params.RNominal = r0;
	p_therm->Params.TNominal = t0_degC + 273;
	p_therm->Params.BConstant = b;
}

void Thermistor_SetVInRef_MilliV(Thermistor_T * p_therm, uint32_t vIn_MilliV)
{
	p_therm->Params.VIn_Scalar = vIn_MilliV / 10U;
}


/******************************************************************************/
/*!
	Unit Conversion
*/
/******************************************************************************/
// #ifndef HOST_SIDE_UNIT_CONVERSION

/*
	For when conversion processing is less frequent than user output
*/
// void Thermistor_CaptureUnits_DegC(Thermistor_T * p_therm, uint16_t adcu)
// {
// 		p_therm->Heat_DegC = Thermistor_ConvertToDegC_Int(p_therm, adcu, p_therm->Params.CaptureScalar);
// }

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

/* Convert Resistence [Ohm] to ADCU */
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
static inline float invsteinhart(double b, double t0, double r0, double invKelvin)
{
	return exp((invKelvin - 1.0F / t0) * b) * r0;
}

static float ConvertAdcuToDegC(Thermistor_T * p_therm, uint16_t adcu)
{
	uint32_t rNet = r_pulldown_adcu(p_therm->CONFIG.R_SERIES/100U, p_therm->Params.VIn_Scalar, ADC_MAX, _AdcVRef_Scalar, adcu) * 100U;
	uint32_t rTh = (p_therm->CONFIG.R_PARALLEL != 0U) ? r_parallel(rNet, p_therm->CONFIG.R_PARALLEL) : rNet;
	double invKelvin = steinhart(p_therm->Params.BConstant, p_therm->Params.TNominal, p_therm->Params.RNominal, rTh);
	return (1.0F / invKelvin - 273.15F);
}

static uint16_t ConvertDegCToAdcu(Thermistor_T * p_therm, uint16_t degC)
{
	double invKelvin;
	uint32_t rTh;
	uint32_t rNet;

	switch(p_therm->Params.Type)
	{
		case THERMISTOR_TYPE_NTC:
			invKelvin = (double)1.0F / (degC + 273.15F);
			rTh = invsteinhart(p_therm->Params.BConstant, p_therm->Params.TNominal, p_therm->Params.RNominal, invKelvin);
			rNet = (p_therm->CONFIG.R_PARALLEL != 0U) ? r_net(p_therm->CONFIG.R_PARALLEL, rTh) : rTh;
			break;
		case THERMISTOR_TYPE_LINEAR:
			invKelvin = 0;
			rTh = 0;
			rNet = 0;
			break;
		default:
			invKelvin = 0;
			rTh = 0;
			rNet = 0;
			break;
	}



	return adcu_r(ADC_MAX, _AdcVRef_Scalar, p_therm->Params.VIn_Scalar, p_therm->CONFIG.R_SERIES, rNet);
}

float Thermistor_ConvertToDegC_Float(Thermistor_T * p_therm, uint16_t adcu)
{
	return ConvertAdcuToDegC(p_therm, adcu);
}

int32_t Thermistor_ConvertToDegC_Int(Thermistor_T * p_therm, uint16_t adcu, uint16_t scalar)
{
	return (int32_t)(ConvertAdcuToDegC(p_therm, adcu) * scalar);
}

uint16_t Thermistor_ConvertToAdcu_DegC(Thermistor_T * p_therm, uint16_t degC)
{
	return ConvertDegCToAdcu(p_therm, degC);
}

/******************************************************************************/
/*!
	Set Params
*/
/******************************************************************************/
static uint16_t ConvertDegCToAdcu_SetUser(Thermistor_T * p_therm, uint8_t degC)
{
	uint16_t adcu = ConvertDegCToAdcu(p_therm, degC);
	while(((uint16_t)ConvertAdcuToDegC(p_therm, adcu) > degC) && (adcu < ADC_MAX))
	{
		adcu += 1U;
	}
	return adcu;
}

void Thermistor_SetShutdown_DegC(Thermistor_T * p_therm, uint8_t shutdown_degC, uint8_t shutdownThreshold_degC)
{
	p_therm->Params.Shutdown_Adcu = ConvertDegCToAdcu_SetUser(p_therm, shutdown_degC);
	p_therm->Params.ShutdownThreshold_Adcu = ConvertDegCToAdcu_SetUser(p_therm, shutdownThreshold_degC);
}

void Thermistor_SetWarning_DegC(Thermistor_T * p_therm, uint8_t warning_degC, uint8_t warningThreshold_degC)
{
	p_therm->Params.Warning_Adcu = ConvertDegCToAdcu_SetUser(p_therm, warning_degC);
	p_therm->Params.WarningThreshold_Adcu = ConvertDegCToAdcu_SetUser(p_therm, warningThreshold_degC);
}

void Thermistor_SetLimits_DegC(Thermistor_T * p_therm, uint8_t shutdown, uint8_t shutdownThreshold, uint8_t warning, uint8_t warningThreshold)
{
	Thermistor_SetShutdown_DegC(p_therm, shutdown, shutdownThreshold);
	Thermistor_SetWarning_DegC(p_therm, warning, warningThreshold);
}

int32_t Thermistor_GetShutdown_DegC(Thermistor_T * p_therm) 			{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Shutdown_Adcu); }
int32_t Thermistor_GetShutdownThreshold_DegC(Thermistor_T * p_therm) 	{ return ConvertAdcuToDegC(p_therm, p_therm->Params.ShutdownThreshold_Adcu); }
int32_t Thermistor_GetWarning_DegC(Thermistor_T * p_therm) 				{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Warning_Adcu); }
int32_t Thermistor_GetWarningThreshold_DegC(Thermistor_T * p_therm) 	{ return ConvertAdcuToDegC(p_therm, p_therm->Params.WarningThreshold_Adcu); }

int32_t Thermistor_GetShutdown_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 			{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.Shutdown_Adcu, scalar); }
int32_t Thermistor_GetShutdownThreshold_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 	{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.ShutdownThreshold_Adcu, scalar); }
int32_t Thermistor_GetWarning_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 				{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.Warning_Adcu, scalar); }
int32_t Thermistor_GetWarningThreshold_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 	{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.WarningThreshold_Adcu, scalar); }

float Thermistor_GetShutdown_DegCFloat(Thermistor_T * p_therm) 				{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Shutdown_Adcu); }
float Thermistor_GetShutdownThreshold_DegCFloat(Thermistor_T * p_therm) 	{ return ConvertAdcuToDegC(p_therm, p_therm->Params.ShutdownThreshold_Adcu); }
float Thermistor_GetWarning_DegCFloat(Thermistor_T * p_therm) 				{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Warning_Adcu); }
float Thermistor_GetWarningThreshold_DegCFloat(Thermistor_T * p_therm) 		{ return ConvertAdcuToDegC(p_therm, p_therm->Params.WarningThreshold_Adcu); }
