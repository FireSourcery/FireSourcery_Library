/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file    Thermistor.h
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

typedef enum Thermistor_Status
{
    /* Main Status Return */
    THERMISTOR_STATUS_OK,
    THERMISTOR_STATUS_WARNING_THRESHOLD,
    THERMISTOR_STATUS_WARNING,
    THERMISTOR_STATUS_FAULT_THRESHOLD,
    THERMISTOR_STATUS_FAULT,
}
Thermistor_Status_T;

/* Calculation */
typedef enum Thermistor_Type
{
    THERMISTOR_TYPE_LINEAR,
#if defined(CONFIG_THERMISTOR_UNITS_NON_LINEAR)
    THERMISTOR_TYPE_NTC,
    THERMISTOR_TYPE_PTC,
#endif
    THERMISTOR_TYPE_LUT,
}
Thermistor_Type_T;

/*
    Set Vin to same decimal precision as ADC_VREF
*/
typedef struct __attribute__((aligned(2U))) Thermistor_Params
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

typedef struct Thermistor_Config
{
    const uint32_t R_SERIES;    /* Pull-up */
    const uint32_t R_PARALLEL;  /* Parallel pull-down if applicable. 0 for Disable */
    //bool IS_CONST;            /* Disable Coefficient set functions */
    const Thermistor_Params_T * P_PARAMS;
}
Thermistor_Config_T;

typedef struct Thermistor
{
    const Thermistor_Config_T CONFIG;
    Thermistor_Params_T Params;
    Linear_T LinearUnits; /* Simple linear fit. */
    Linear_T LinearLimits; /* return value [Warning_Adcu:Fault_Adcu] as [65535:0], Roughly linear 70-100C */
    Thermistor_Status_T Status;
    uint16_t Adcu; /* Previous ADC sample */
}
Thermistor_T;

#define _THERMISTOR_INIT_CONFIG(RSeries, RParallel, p_Params)   \
{                                                               \
    .R_SERIES       = RSeries,                                  \
    .R_PARALLEL     = RParallel,                                \
    .P_PARAMS       = p_Params,                                 \
}

#define _THERMISTOR_INIT_PARAMS_NTC(R0, T0_Kelvin, B)   \
{                                                       \
    .NtcR0  = R0,                                       \
    .NtcT0  = T0_Kelvin,                                \
    .NtcB   = B,                                        \
}                                                       \

#define THERMISTOR_INIT(RSeries, RParallel, p_Params)                   \
{                                                                       \
    .CONFIG = _THERMISTOR_INIT_CONFIG(RSeries, RParallel, p_Params),    \
}

#define THERMISTOR_INIT_WITH_NTC(RSeries, RParallel, p_Params, NtcR0, NtcT0_Kelvin, NtcB)   \
{                                                                                           \
    .CONFIG = _THERMISTOR_INIT_CONFIG(RSeries, RParallel, p_Params),                        \
    .Params = _THERMISTOR_INIT_PARAMS_NTC(NtcR0, NtcT0_Kelvin, NtcB),                       \
}

/******************************************************************************/
/*
    HeatLimit scalar value - Fraction Inverse to heat
    From Warning_Adcu => 65535
    To Fault_Adcu => 0
*/
/******************************************************************************/
static inline uint16_t Thermistor_ConvertHeatLimit_FracU16(const Thermistor_T * p_therm, uint16_t adcu)   { return Linear_ADC_CalcFracU16(&p_therm->LinearLimits, adcu); }
static inline uint16_t Thermistor_GetHeatLimit_FracU16(const Thermistor_T * p_therm)                      { return Thermistor_ConvertHeatLimit_FracU16(p_therm, p_therm->Adcu); }

/* Monitor */
static inline Thermistor_Status_T Thermistor_GetStatus(const Thermistor_T * p_therm)    { return (p_therm->Status); }
static inline bool Thermistor_GetIsFault(const Thermistor_T * p_therm)                  { return ((p_therm->Status == THERMISTOR_STATUS_FAULT) || (p_therm->Status == THERMISTOR_STATUS_FAULT_THRESHOLD)); }
static inline bool Thermistor_GetIsWarning(const Thermistor_T * p_therm)                { return ((p_therm->Status == THERMISTOR_STATUS_WARNING) || (p_therm->Status == THERMISTOR_STATUS_WARNING_THRESHOLD)); }

static inline void Thermistor_EnableMonitor(Thermistor_T * p_therm)                     { p_therm->Params.IsMonitorEnable = true; } /* todo check values */
static inline void Thermistor_DisableMonitor(Thermistor_T * p_therm)                    { p_therm->Params.IsMonitorEnable = false; }
static inline void Thermistor_SetMonitorEnable(Thermistor_T * p_therm, bool isEnable)   { p_therm->Params.IsMonitorEnable = isEnable; }

static inline bool Thermistor_GetIsMonitorEnable(const Thermistor_T * p_therm)              { return p_therm->Params.IsMonitorEnable; }
static inline uint16_t Thermistor_GetFault_Adcu(const Thermistor_T * p_therm)               { return p_therm->Params.Fault_Adcu; }
static inline uint16_t Thermistor_GetFaultThreshold_Adcu(const Thermistor_T * p_therm)      { return p_therm->Params.FaultThreshold_Adcu; }
static inline uint16_t Thermistor_GetWarning_Adcu(const Thermistor_T * p_therm)             { return p_therm->Params.Warning_Adcu; }
static inline uint16_t Thermistor_GetWarningThreshold_Adcu(const Thermistor_T * p_therm)    { return p_therm->Params.WarningThreshold_Adcu; }

/* Conversion */
static inline Thermistor_Type_T Thermistor_GetType(const Thermistor_T * p_therm)    { return p_therm->Params.Type; }
static inline uint16_t Thermistor_GetLinearT0_Adcu(const Thermistor_T * p_therm)    { return p_therm->Params.LinearT0_Adcu; }
static inline uint16_t Thermistor_GetLinearT0_DegC(const Thermistor_T * p_therm)    { return p_therm->Params.LinearT0_DegC; }
static inline uint16_t Thermistor_GetLinearT1_Adcu(const Thermistor_T * p_therm)    { return p_therm->Params.LinearT1_Adcu; }
static inline uint16_t Thermistor_GetLinearT1_DegC(const Thermistor_T * p_therm)    { return p_therm->Params.LinearT1_DegC; }
static inline uint32_t Thermistor_GetR0(const Thermistor_T * p_therm)               { return p_therm->Params.NtcR0; }
static inline uint16_t Thermistor_GetT0(const Thermistor_T * p_therm)               { return p_therm->Params.NtcT0; } /* Degrees Kelvin */
static inline uint16_t Thermistor_GetT0_DegC(const Thermistor_T * p_therm)          { return p_therm->Params.NtcT0 - 273; }
static inline uint16_t Thermistor_GetB(const Thermistor_T * p_therm)                { return p_therm->Params.NtcB; }
static inline uint16_t Thermistor_GetVIn(const Thermistor_T * p_therm)              { return p_therm->Params.VInRef_MilliV; }

static inline void Thermistor_SetType(Thermistor_T * p_therm, uint16_t value)           { p_therm->Params.Type = value; }
static inline void Thermistor_SetLinearT0_Adcu(Thermistor_T * p_therm, uint16_t value)  { p_therm->Params.LinearT0_Adcu = value; }
static inline void Thermistor_SetLinearT0_DegC(Thermistor_T * p_therm, uint16_t value)  { p_therm->Params.LinearT0_DegC = value; }
static inline void Thermistor_SetLinearT1_Adcu(Thermistor_T * p_therm, uint16_t value)  { p_therm->Params.LinearT1_Adcu = value; }
static inline void Thermistor_SetLinearT1_DegC(Thermistor_T * p_therm, uint16_t value)  { p_therm->Params.LinearT1_DegC = value; }
static inline void Thermistor_SetR0(Thermistor_T * p_therm, uint16_t value)             { p_therm->Params.NtcR0 = value; }
static inline void Thermistor_SetT0(Thermistor_T * p_therm, uint16_t value)             { p_therm->Params.NtcT0 = value; } /* Degrees Kelvin */
static inline void Thermistor_SetT0_DegC(Thermistor_T * p_therm, uint16_t value)        { p_therm->Params.NtcT0 = value; }
static inline void Thermistor_SetB(Thermistor_T * p_therm, uint16_t value)              { p_therm->Params.NtcB = value; }
static inline void Thermistor_SetVIn(Thermistor_T * p_therm, uint16_t value)            { p_therm->Params.VInRef_MilliV = value; }

/* Using async capture conversion only */
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

extern void Thermistor_CaptureUnits_DegC(const Thermistor_T * p_therm, uint16_t adcu);
extern float Thermistor_ConvertToDegC_Float(const Thermistor_T * p_therm, uint16_t adcu);
extern int32_t Thermistor_ConvertToDegC_Int(const Thermistor_T * p_therm, uint16_t adcu, uint8_t scalar);

extern void Thermistor_SetFault_DegC(Thermistor_T * p_therm, uint8_t fault_degC, uint8_t faultThreshold_degC);
extern void Thermistor_SetWarning_DegC(Thermistor_T * p_therm, uint8_t warning_degC, uint8_t warningThreshold_degC);
extern void Thermistor_SetLimits_DegC(Thermistor_T * p_therm, uint8_t fault, uint8_t faultThreshold, uint8_t warning, uint8_t warningThreshold);

extern int16_t Thermistor_GetFault_DegC(const Thermistor_T * p_therm);
extern int16_t Thermistor_GetFaultThreshold_DegC(const Thermistor_T * p_therm);
extern int16_t Thermistor_GetWarning_DegC(const Thermistor_T * p_therm);
extern int16_t Thermistor_GetWarningThreshold_DegC(const Thermistor_T * p_therm);
extern int32_t Thermistor_GetFault_DegCInt(const Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetFaultThreshold_DegCInt(const Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetWarning_DegCInt(const Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetWarningThreshold_DegCInt(const Thermistor_T * p_therm, uint16_t scalar);
extern float Thermistor_GetFault_DegCFloat(const Thermistor_T * p_therm);
extern float Thermistor_GetFaultThreshold_DegCFloat(const Thermistor_T * p_therm);
extern float Thermistor_GetWarning_DegCFloat(const Thermistor_T * p_therm);
extern float Thermistor_GetWarningThreshold_DegCFloat(const Thermistor_T * p_therm);

#endif
