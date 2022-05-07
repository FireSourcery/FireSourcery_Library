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
	@file  	Thermistor.h
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "Config.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum Thermistor_Status_Tag
{
	THERMISTOR_STATUS_OK,
	THERMISTOR_LIMIT_SHUTDOWN,		/* over limit shutdown */
	THERMISTOR_LIMIT_THRESHOLD,		/* over limit threshold */
	THERMISTOR_WARNING,
}
Thermistor_Status_T;

/*
	Set Vin to same decimal precision as ADC_VREF
*/
typedef struct __attribute__((aligned(4U))) Thermistor_Params_Tag
{
	uint16_t VIn_Scalar;
	uint32_t RNominal;
	uint32_t TNominal; /* In Kelvin*/
	uint32_t BConstant;
	uint16_t Limit_Adcu;
	uint16_t Threshold_Adcu;
	uint16_t Warning_Adcu;

	uint16_t CaptureScalar;
	bool IsEnable;
}
Thermistor_Params_T;

typedef struct Thermistor_Config_Tag
{
	const Thermistor_Params_T * P_PARAMS;
	uint32_t R_SERIES; 		/* Pull-up */
	uint32_t R_PARALLEL; 	/* Parallel pull-down if applicable */
}
Thermistor_Config_T;

typedef struct Thermistor_Tag
{
	const Thermistor_Config_T CONFIG;
	Thermistor_Params_T Params;

	Thermistor_Status_T ThresholdStatus; /* Threshold save state info */
	Thermistor_Status_T Status; /* Threshold save state info */
	int32_t Heat_DegC;
}
Thermistor_T;

#define THERMISTOR_CONFIG(RSeries, RParallel, p_Params) 	\
{													\
	.CONFIG =										\
	{												\
		.R_SERIES 		= RSeries,					\
		.R_PARALLEL		= RParallel,				\
		.P_PARAMS		= p_Params,					\
	}												\
}

/* Using capture conversion only */
static inline int32_t Thermistor_GetHeat_DegC(Thermistor_T * p_therm) { return p_therm->Heat_DegC; }
static inline bool Thermistor_GetIsEnable(Thermistor_T * p_therm) { return p_therm->Params.IsEnable; } 

static inline bool Thermistor_GetIsStatusLimit(Thermistor_T * p_therm) { return ((p_therm->Status == THERMISTOR_LIMIT_SHUTDOWN) || (p_therm->Status == THERMISTOR_LIMIT_THRESHOLD)); }
static inline bool Thermistor_GetIsStatusWarning(Thermistor_T * p_therm) { return (p_therm->Status == THERMISTOR_WARNING); }
static inline Thermistor_Status_T Thermistor_GetStatus(Thermistor_T * p_therm) { return (p_therm->Status); }


extern void Thermistor_InitAdcVRef_Scalar(uint16_t adcVRef_MilliV);
extern void Thermistor_Init(Thermistor_T * p_therm);

// extern Thermistor_Status_T Thermistor_ProcThreshold(Thermistor_T * p_therm, uint16_t adcu);
extern Thermistor_Status_T Thermistor_PollMonitor(Thermistor_T * p_therm, uint16_t adcu); 

extern void Thermistor_CaptureUnits_DegC(Thermistor_T * p_therm, uint16_t adcu);
extern float Thermistor_ConvertToDegC_Float(Thermistor_T * p_therm, uint16_t adcu);
extern int32_t Thermistor_ConvertToDegC_Int(Thermistor_T * p_therm, uint16_t adcu, uint16_t scalar);
extern void Thermistor_SetRThValues_DegC(Thermistor_T * p_therm, uint32_t r0, uint32_t t0_degC, uint32_t b, uint8_t threshold_degC, uint8_t limit_degC);
extern void Thermistor_SetVIn_MilliV(Thermistor_T * p_therm, uint32_t vIn_MilliV);

extern void Thermistor_SetLimit_DegC(Thermistor_T * p_therm, uint8_t limit_degreesC);
extern void Thermistor_SetThreshold_DegC(Thermistor_T * p_therm, uint8_t threshold_degreesC);
extern void Thermistor_SetWarning_DegC(Thermistor_T * p_therm, uint8_t warning_degreesC);
extern int32_t Thermistor_GetLimit_DegCInt(Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetThreshold_DegCInt(Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetWarning_DegCInt(Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetLimit_DegC(Thermistor_T * p_therm);
extern int32_t Thermistor_GetThreshold_DegC(Thermistor_T * p_therm);
extern int32_t Thermistor_GetWarning_DegC(Thermistor_T * p_therm);
extern float Thermistor_GetLimit_DegCFloat(Thermistor_T * p_therm);
extern float Thermistor_GetThreshold_DegCFloat(Thermistor_T * p_therm);
extern float Thermistor_GetWarning_DegCFloat(Thermistor_T * p_therm);

#endif
