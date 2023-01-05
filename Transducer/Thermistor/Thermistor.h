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
	@file  	Thermistor.h
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "Config.h"
#include "Peripheral/Analog/Analog/Global_Analog.h"
#include "Math/Linear/Linear_ADC.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum Thermistor_Status_Tag
{
	/* Main Status Return */
	THERMISTOR_STATUS_OK,
	THERMISTOR_STATUS_SHUTDOWN,
	THERMISTOR_STATUS_WARNING,

	// THERMISTOR_SHUTDOWN_RISING_EDGE,
	// THERMISTOR_SHUTDOWN_FALLING_EDGE,
	// THERMISTOR_SHUTDOWN_LIMIT,			/* over limit shutdown */
	// THERMISTOR_SHUTDOWN_THRESHOLD,		/* over limit threshold */

	// THERMISTOR_WARNING_RISING_EDGE,		/* Over Limit edge, returns true once per edge */
	// THERMISTOR_WARNING_FALLING_EDGE,		/* Under Threshold, returns true once per edge */
	// THERMISTOR_WARNING_LIMIT,
	// THERMISTOR_WARNING_THRESHOLD,
}
Thermistor_Status_T;

/* Private module use */
typedef enum Thermistor_ThresholdStatus_Tag
{
	THERMISTOR_THRESHOLD_OK,
	THERMISTOR_THRESHOLD_CONTINUE,
	THERMISTOR_THRESHOLD_LIMIT,
}
Thermistor_ThresholdStatus_T;

/* Calculation */
typedef enum Thermistor_Type_Tag
{
	THERMISTOR_TYPE_LINEAR,
#if defined(CONFIG_THERMISTOR_UNITS_NON_LINEAR)
	THERMISTOR_TYPE_NTC,
	THERMISTOR_TYPE_PTC,
#endif
	THERMISTOR_TYPE_LUT,
}
Thermistor_Type_T;

// typedef struct Thermistor_LimitThreshold_Tag
// {
// 	uint16_t Limit_Adcu;
// 	uint16_t Threshold_Adcu;
// 	Thermistor_Threshold_Status_T Status;
// }
// Thermistor_LimitThreshold_T;

/*
	Set Vin to same decimal precision as ADC_VREF
*/
typedef struct __attribute__((aligned(2U))) Thermistor_Params_Tag
{
	Thermistor_Type_T Type;

	/* NTC Unit Conversion */
	uint16_t VInRef_MilliV;
	uint32_t NtcR0;
	uint16_t NtcT0; /* In Kelvin*/
	uint16_t NtcB;

	/* Linear Unit Conversion */
	uint16_t LinearT0_Adcu;
	uint8_t LinearT0_DegC;
	uint16_t LinearT1_Adcu;
	uint8_t LinearT1_DegC;

	/* Monitor Limits */
	uint16_t Fault_Adcu;
	uint16_t FaultThreshold_Adcu;
	uint16_t Warning_Adcu;
	uint16_t WarningThreshold_Adcu;

	// uint16_t CaptureScalar;
	bool IsMonitorEnable;
}
Thermistor_Params_T;

typedef struct Thermistor_Config_Tag
{
	const uint32_t R_SERIES; 	/* Pull-up */
	const uint32_t R_PARALLEL; 	/* Parallel pull-down if applicable. 0 for Disable */
	//bool IS_CONST;			/* Disable Coefficient set functions */
	const Thermistor_Params_T * P_PARAMS;
}
Thermistor_Config_T;

typedef struct Thermistor_Tag
{
	const Thermistor_Config_T CONFIG;
	Thermistor_Params_T Params;
	Linear_T LinearUnits; /* Simple linear fit. */
	Linear_T LinearLimits; /* return value [Warning_Adcu:Fault_Adcu] as [65535:0], Roughly linear 70-100C */
	Thermistor_ThresholdStatus_T ShutdownThreshold;
	Thermistor_ThresholdStatus_T WarningThreshold; /* Threshold save state info */
	Thermistor_Status_T Status;
	uint16_t Adcu; /* Previous ADC sample */
}
Thermistor_T;

#define _THERMISTOR_INIT_CONFIG(RSeries, RParallel, p_Params) 	\
{																\
	.R_SERIES 		= RSeries,									\
	.R_PARALLEL		= RParallel,								\
	.P_PARAMS		= p_Params,									\
}

#define _THERMISTOR_INIT_PARAMS_NTC(R0, T0_Kelvin, B) 	\
{														\
	.NtcR0 		= R0,								\
	.NtcT0		= T0_Kelvin,						\
	.NtcB		= B,								\
}														\

#define THERMISTOR_INIT(RSeries, RParallel, p_Params) 					\
{																		\
	.CONFIG = _THERMISTOR_INIT_CONFIG(RSeries, RParallel, p_Params), 	\
}

#define THERMISTOR_INIT_WITH_NTC(RSeries, RParallel, p_Params, NtcR0, NtcT0_Kelvin, NtcB) 	\
{																							\
	.CONFIG = _THERMISTOR_INIT_CONFIG(RSeries, RParallel, p_Params),						\
	.Params = _THERMISTOR_INIT_PARAMS_NTC(NtcR0, NtcT0_Kelvin, NtcB),						\
}

/******************************************************************************/
/*
	HeatLimit scalar value - Fraction Inverse to heat
	From Warning_Adcu => 65535
	To Fault_Adcu => 0
*/
/******************************************************************************/
static inline uint16_t Thermistor_GetHeatLimit_FracU16(Thermistor_T * p_therm) 						{ return Linear_ADC_CalcFracU16(&p_therm->LinearLimits, p_therm->Adcu); }
static inline uint16_t Thermistor_ConvertHeatLimit_FracU16(Thermistor_T * p_therm, uint16_t adcu) 	{ return Linear_ADC_CalcFracU16(&p_therm->LinearLimits, adcu); }

/* Monitor */
static inline Thermistor_Status_T Thermistor_GetStatus(Thermistor_T * p_therm) 	{ return (p_therm->Status); }
static inline bool Thermistor_GetIsShutdown(Thermistor_T * p_therm) 			{ return p_therm->Status == THERMISTOR_STATUS_SHUTDOWN; } //||THERMISTOR_SHUTDOWN_RISING_EDGE
static inline bool Thermistor_GetIsWarning(Thermistor_T * p_therm) 				{ return p_therm->Status == THERMISTOR_STATUS_WARNING; }
static inline bool Thermistor_GetIsMonitorEnable(Thermistor_T * p_therm) 		{ return p_therm->Params.IsMonitorEnable; }

static inline void Thermistor_EnableMonitor(Thermistor_T * p_therm) 					{ p_therm->Params.IsMonitorEnable = true; }
static inline void Thermistor_DisableMonitor(Thermistor_T * p_therm) 					{ p_therm->Params.IsMonitorEnable = false; }
static inline void Thermistor_SetMonitorEnable(Thermistor_T * p_therm, bool isEnable) 	{ p_therm->Params.IsMonitorEnable = isEnable; }

static inline uint32_t Thermistor_GetR0(Thermistor_T * p_therm) 		{ return p_therm->Params.NtcR0; }
static inline uint16_t Thermistor_GetT0(Thermistor_T * p_therm) 		{ return p_therm->Params.NtcT0; } /* Degrees Kelvin */
static inline uint16_t Thermistor_GetT0_DegC(Thermistor_T * p_therm) 	{ return p_therm->Params.NtcT0 - 273; }
static inline uint16_t Thermistor_GetB(Thermistor_T * p_therm) 			{ return p_therm->Params.NtcB; }
static inline uint16_t Thermistor_GetVIn(Thermistor_T * p_therm) 		{ return p_therm->Params.VInRef_MilliV; }

static inline uint16_t Thermistor_GetShutdown_Adcu(Thermistor_T * p_therm)			{ return p_therm->Params.Fault_Adcu; }
static inline uint16_t Thermistor_GetShutdownThreshold_Adcu(Thermistor_T * p_therm)	{ return p_therm->Params.FaultThreshold_Adcu; }
static inline uint16_t Thermistor_GetWarning_Adcu(Thermistor_T * p_therm)			{ return p_therm->Params.Warning_Adcu; }
static inline uint16_t Thermistor_GetWarningThreshold_Adcu(Thermistor_T * p_therm)	{ return p_therm->Params.WarningThreshold_Adcu; }

/* Using capture conversion only */
// static inline int32_t Thermistor_GetHeat_DegC(Thermistor_T * p_therm) { return p_therm->Heat_DegC; }

/******************************************************************************/
/*
	Extern
*/
/******************************************************************************/
extern void Thermistor_Init(Thermistor_T * p_therm);

extern Thermistor_Status_T Thermistor_PollMonitor(Thermistor_T * p_therm, uint16_t adcu);
extern void Thermistor_SetNtc(Thermistor_T * p_therm, uint32_t r0, uint32_t t0_degC, uint32_t b);
extern void Thermistor_SetVInRef_MilliV(Thermistor_T * p_therm, uint32_t vIn_MilliV);

extern void Thermistor_CaptureUnits_DegC(Thermistor_T * p_therm, uint16_t adcu);
extern float Thermistor_ConvertToDegC_Float(Thermistor_T * p_therm, uint16_t adcu);
extern int32_t Thermistor_ConvertToDegC_Int(Thermistor_T * p_therm, uint16_t adcu, uint8_t scalar);

extern void Thermistor_SetShutdown_DegC(Thermistor_T * p_therm, uint8_t shutdown_degC, uint8_t shutdownThreshold_degC);
extern void Thermistor_SetWarning_DegC(Thermistor_T * p_therm, uint8_t warning_degC, uint8_t warningThreshold_degC);
extern void Thermistor_SetLimits_DegC(Thermistor_T * p_therm, uint8_t shutdown, uint8_t shutdownThreshold, uint8_t warning, uint8_t warningThreshold);

extern int16_t Thermistor_GetShutdown_DegC(Thermistor_T * p_therm);
extern int16_t Thermistor_GetShutdownThreshold_DegC(Thermistor_T * p_therm);
extern int16_t Thermistor_GetWarning_DegC(Thermistor_T * p_therm);
extern int16_t Thermistor_GetWarningThreshold_DegC(Thermistor_T * p_therm);
extern int32_t Thermistor_GetShutdown_DegCInt(Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetShutdownThreshold_DegCInt(Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetWarning_DegCInt(Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetWarningThreshold_DegCInt(Thermistor_T * p_therm, uint16_t scalar);
extern float Thermistor_GetShutdown_DegCFloat(Thermistor_T * p_therm);
extern float Thermistor_GetShutdownThreshold_DegCFloat(Thermistor_T * p_therm);
extern float Thermistor_GetWarning_DegCFloat(Thermistor_T * p_therm);
extern float Thermistor_GetWarningThreshold_DegCFloat(Thermistor_T * p_therm);

#endif
