/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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

static void ResetUnitsLinear(Thermistor_T * p_therm)
{
	Linear_ADC_Init(&p_therm->LinearLimits, p_therm->Params.Shutdown_Adcu, p_therm->Params.Warning_Adcu, 0, 0);
	/*
		Max resolution
		12-bit ADC, 200 physical degrees interval
		.1 to .05 physical degrees resolution.
		x20 for max resolution
	*/
	Linear_Init_Map(&p_therm->LinearUnits, p_therm->Params.LinearUnits0_Adcu, p_therm->Params.LinearUnits1_Adcu, p_therm->Params.LinearUnits0_DegC, p_therm->Params.LinearUnits1_DegC);
}

void Thermistor_Init(Thermistor_T * p_therm)
{
	if(p_therm->CONFIG.P_PARAMS != 0U) { memcpy(&p_therm->Params, p_therm->CONFIG.P_PARAMS, sizeof(Thermistor_Params_T)); }

	ResetUnitsLinear(p_therm);

	if(p_therm->Params.Shutdown_Adcu == 0U) 			{ p_therm->Params.IsMonitorEnable = false; }
	if(p_therm->Params.ShutdownThreshold_Adcu == 0U) 	{ p_therm->Params.IsMonitorEnable = false; }
	if(p_therm->Params.Warning_Adcu == 0U) 				{ p_therm->Params.IsMonitorEnable = false; }
	if(p_therm->Params.WarningThreshold_Adcu == 0U) 	{ p_therm->Params.IsMonitorEnable = false; }

	p_therm->Status = THERMISTOR_STATUS_OK;
	p_therm->ShutdownThreshold = THERMISTOR_THRESHOLD_OK;
	p_therm->WarningThreshold = THERMISTOR_THRESHOLD_OK;
}

/******************************************************************************/
/*!
	Limits Monitor
*/
/******************************************************************************/
/*
	Lower adcu is higher heat
*/
Thermistor_ThresholdStatus_T CheckThreshold(Thermistor_ThresholdStatus_T status, uint16_t limit, uint16_t threshold, uint16_t adcu)
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
		p_therm->ShutdownThreshold = CheckThreshold(p_therm->ShutdownThreshold, p_therm->Params.Shutdown_Adcu, p_therm->Params.ShutdownThreshold_Adcu, p_therm->Adcu);
		p_therm->Status = (p_therm->ShutdownThreshold == THERMISTOR_THRESHOLD_OK) ? THERMISTOR_STATUS_OK : THERMISTOR_STATUS_SHUTDOWN;

		if(p_therm->Status == THERMISTOR_STATUS_OK)
		{
			p_therm->WarningThreshold = CheckThreshold(p_therm->WarningThreshold, p_therm->Params.Warning_Adcu, p_therm->Params.WarningThreshold_Adcu, p_therm->Adcu);
			p_therm->Status = (p_therm->WarningThreshold == THERMISTOR_THRESHOLD_OK) ? THERMISTOR_STATUS_OK : THERMISTOR_STATUS_WARNING;
		}
	}

	return p_therm->Status;
}

/******************************************************************************/
/*!
	Unit Conversion
*/
/******************************************************************************/
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
	return ((uint64_t)rPullUp * adcVRef * adcu) / (vIn * adcMax - adcVRef * adcu);
}

/* Thermistor as pull-up */
static inline uint32_t r_pullup_adcu(uint32_t rPullDown, uint16_t vIn, uint16_t adcMax, uint16_t adcVRef, uint16_t adcu)
{
	return ((uint64_t)rPullDown * vIn * adcMax) / (adcVRef * adcu) - rPullDown;
}

/* Convert Resistance [Ohm] to ADCU */
static inline uint16_t adcu_r(uint16_t adcMax, uint16_t adcVRef, uint16_t vIn, uint32_t rPullUp, uint32_t rPullDown)
{
	return ((uint64_t)vIn * adcMax * rPullDown) / ((uint64_t)adcVRef * (rPullUp + rPullDown));
}

static inline uint32_t r_net(uint32_t rParallel1, uint32_t rParallel2)
{
	return ((uint64_t)rParallel1 * rParallel2) / (rParallel1 + rParallel2); 	/* 1 / (1/r1 + 1/r2) */
}

static inline uint32_t r_parallel(uint32_t rNet, uint32_t rParallel)
{
	return ((uint64_t)rNet * rParallel) / (rNet - rParallel); 				/* 1 / (1/rNet - 1/rParallel) */
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

static float ConvertAdcuToDegC_Ntc(Thermistor_T * p_therm, uint16_t adcu)
{
	uint32_t rNet = r_pulldown_adcu(p_therm->CONFIG.R_SERIES, p_therm->Params.VInRef_MilliV, GLOBAL_ANALOG.ADC_MAX, GLOBAL_ANALOG.ADC_VREF_MILLIV, adcu);
	uint32_t rTh = (p_therm->CONFIG.R_PARALLEL != 0U) ? r_parallel(rNet, p_therm->CONFIG.R_PARALLEL) : rNet;
	double invKelvin = steinhart(p_therm->Params.BConstant, p_therm->Params.TNominal, p_therm->Params.RNominal, rTh);
	return (1.0F / invKelvin - 273.15F);
}

static uint16_t ConvertDegCToAdcu_Ntc(Thermistor_T * p_therm, uint16_t degC)
{
	double invKelvin = (double)1.0F / (degC + 273.15F);
	uint32_t rTh = invsteinhart(p_therm->Params.BConstant, p_therm->Params.TNominal, p_therm->Params.RNominal, invKelvin);
	uint32_t rNet = (p_therm->CONFIG.R_PARALLEL != 0U) ? r_net(p_therm->CONFIG.R_PARALLEL, rTh) : rTh;
	adcu_r(GLOBAL_ANALOG.ADC_MAX, GLOBAL_ANALOG.ADC_VREF_MILLIV, p_therm->Params.VInRef_MilliV, p_therm->CONFIG.R_SERIES, rNet);
}

static int16_t ConvertAdcuToDegC_Linear(Thermistor_T * p_therm, uint16_t adcu) 	{ return Linear_Function(&p_therm->LinearUnits, adcu); }
static uint16_t ConvertDegCToAdcu_Linear(Thermistor_T * p_therm, int16_t degC) 	{ return Linear_InvFunction(&p_therm->LinearUnits, degC); }

static int16_t ConvertAdcuToDegC(Thermistor_T * p_therm, uint16_t adcu)
{
	int16_t degC;
	switch(p_therm->Params.Type)
	{
		case THERMISTOR_TYPE_LINEAR: 	degC = ConvertAdcuToDegC_Linear(p_therm, adcu); break;
#if defined(CONFIG_THERMISTOR_UNITS_NON_LINEAR)
		case THERMISTOR_TYPE_NTC:   	degC = ConvertAdcuToDegC_Ntc(p_therm, adcu); break;
#endif
		default: degC = 0; break;
	}
	return degC;
}

static uint16_t ConvertDegCToAdcu(Thermistor_T * p_therm, uint16_t degC)
{
	uint16_t adcu;
	switch(p_therm->Params.Type)
	{
		case THERMISTOR_TYPE_LINEAR: 	adcu = ConvertDegCToAdcu_Linear(p_therm, degC); break;
#if defined(CONFIG_THERMISTOR_UNITS_NON_LINEAR)
		case THERMISTOR_TYPE_NTC:   	adcu = ConvertDegCToAdcu_Ntc(p_therm, degC); break;
#endif
		default: adcu = 0U; break;
	}
	return adcu;
}

/* Scalar < 20 */
int16_t Thermistor_ConvertToDegC(Thermistor_T * p_therm, uint16_t adcu) { return ConvertAdcuToDegC(p_therm, adcu); }
int32_t Thermistor_ConvertToDegC_Int(Thermistor_T * p_therm, uint16_t adcu, uint8_t scalar) { return ConvertAdcuToDegC(p_therm, adcu * scalar); }
uint16_t Thermistor_ConvertToAdcu_DegC(Thermistor_T * p_therm, uint16_t degC) { return ConvertDegCToAdcu(p_therm, degC); }

static float ConvertAdcuToDegC_Float(Thermistor_T * p_therm, uint16_t adcu)
{
	float degC;
	switch(p_therm->Params.Type)
	{
		case THERMISTOR_TYPE_LINEAR: 	degC = ConvertAdcuToDegC_Linear(p_therm, adcu); break;
#if defined(CONFIG_THERMISTOR_UNITS_NON_LINEAR)
		case THERMISTOR_TYPE_NTC:   	degC = ConvertAdcuToDegC_Ntc(p_therm, adcu); break;
#endif
		default: degC = 0; break;
	}
	return degC;
}

float Thermistor_ConvertToDegC_Float(Thermistor_T * p_therm, uint16_t adcu) { return ConvertAdcuToDegC_Float(p_therm, adcu); }

/* For setting conversion processing to lower frequency than user output */
// void Thermistor_CaptureUnits_DegC(Thermistor_T * p_therm, uint16_t adcu) { p_therm->Heat_DegC = Thermistor_ConvertToDegC_Int(p_therm, adcu, p_therm->Params.CaptureScalar);}

/******************************************************************************/
/*!
	Set Unit Conversion Params Params
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
	p_therm->Params.VInRef_MilliV = vIn_MilliV;
}

/*
	Linear using known adcu calibration values, todo seperate
	Shutdown_Adcu 		=> Shutdown_DegC 	100
	Warning_Adcu 		=> Warning_DegC		70
*/
/* Warning_Adcu must be > 1/2 * ADC_MAX, or ADC_MAX will overflow Linear */
// void Thermistor_SetLinearUnits_DegC(Thermistor_T * p_therm, uint32_t warningDegC, uint32_t shutdownDegC)
// {
// 	p_therm->Params.LinearUnits1_DegC = warningDegC;
// 	p_therm->Params.UnitsShutdown_DegC = shutdownDegC;
// 	ResetUnitsLinear(p_therm);
// }


/******************************************************************************/
/*!
	Set Limits Params
*/
/******************************************************************************/
static uint16_t ConvertDegCToAdcu_Input(Thermistor_T * p_therm, uint8_t degC)
{
	return ConvertDegCToAdcu(p_therm, degC) + ConvertDegCToAdcu(p_therm, 1U);
}

void Thermistor_SetShutdown_DegC(Thermistor_T * p_therm, uint8_t shutdown_degC, uint8_t shutdownThreshold_degC)
{
	p_therm->Params.Shutdown_Adcu = ConvertDegCToAdcu_Input(p_therm, shutdown_degC);
	p_therm->Params.ShutdownThreshold_Adcu = ConvertDegCToAdcu_Input(p_therm, shutdownThreshold_degC);
}

void Thermistor_SetWarning_DegC(Thermistor_T * p_therm, uint8_t warning_degC, uint8_t warningThreshold_degC)
{
	p_therm->Params.Warning_Adcu = ConvertDegCToAdcu_Input(p_therm, warning_degC);
	p_therm->Params.WarningThreshold_Adcu = ConvertDegCToAdcu_Input(p_therm, warningThreshold_degC);
}

void Thermistor_SetLimits_DegC(Thermistor_T * p_therm, uint8_t shutdown, uint8_t shutdownThreshold, uint8_t warning, uint8_t warningThreshold)
{
	Thermistor_SetShutdown_DegC(p_therm, shutdown, shutdownThreshold);
	Thermistor_SetWarning_DegC(p_therm, warning, warningThreshold);
	ResetUnitsLinear(p_therm);
}

int16_t Thermistor_GetShutdown_DegC(Thermistor_T * p_therm) 			{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Shutdown_Adcu); }
int16_t Thermistor_GetShutdownThreshold_DegC(Thermistor_T * p_therm) 	{ return ConvertAdcuToDegC(p_therm, p_therm->Params.ShutdownThreshold_Adcu); }
int16_t Thermistor_GetWarning_DegC(Thermistor_T * p_therm) 				{ return ConvertAdcuToDegC(p_therm, p_therm->Params.Warning_Adcu); }
int16_t Thermistor_GetWarningThreshold_DegC(Thermistor_T * p_therm) 	{ return ConvertAdcuToDegC(p_therm, p_therm->Params.WarningThreshold_Adcu); }

int32_t Thermistor_GetShutdown_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 			{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.Shutdown_Adcu, scalar); }
int32_t Thermistor_GetShutdownThreshold_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 	{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.ShutdownThreshold_Adcu, scalar); }
int32_t Thermistor_GetWarning_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 				{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.Warning_Adcu, scalar); }
int32_t Thermistor_GetWarningThreshold_DegCInt(Thermistor_T * p_therm, uint16_t scalar) 	{ return Thermistor_ConvertToDegC_Int(p_therm, p_therm->Params.WarningThreshold_Adcu, scalar); }

float Thermistor_GetShutdown_DegCFloat(Thermistor_T * p_therm) 				{ return Thermistor_ConvertToDegC_Float(p_therm, p_therm->Params.Shutdown_Adcu); }
float Thermistor_GetShutdownThreshold_DegCFloat(Thermistor_T * p_therm) 	{ return Thermistor_ConvertToDegC_Float(p_therm, p_therm->Params.ShutdownThreshold_Adcu); }
float Thermistor_GetWarning_DegCFloat(Thermistor_T * p_therm) 				{ return Thermistor_ConvertToDegC_Float(p_therm, p_therm->Params.Warning_Adcu); }
float Thermistor_GetWarningThreshold_DegCFloat(Thermistor_T * p_therm) 		{ return Thermistor_ConvertToDegC_Float(p_therm, p_therm->Params.WarningThreshold_Adcu); }
