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
    @file   Thermistor.h
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

#if     defined(CONFIG_THERMISTOR_UNITS_LINEAR) || defined(CONFIG_THERMISTOR_UNITS_LUT)
    typedef int16_t thermal_t;
    static const thermal_t ABSOLUTE_ZERO_CELSIUS = -273U;
#elif   defined(CONFIG_THERMISTOR_UNITS_FLOAT)
    static const thermal_t ABSOLUTE_ZERO_CELSIUS = -273.15F;
    typedef float thermal_t;
#endif

/* Monitor Status Return */
typedef enum Thermistor_Status
{
    THERMISTOR_STATUS_OK,
    THERMISTOR_STATUS_WARNING_THRESHOLD,
    THERMISTOR_STATUS_WARNING,
    THERMISTOR_STATUS_FAULT_THRESHOLD,
    THERMISTOR_STATUS_FAULT,
}
Thermistor_Status_T;

/*   */
// typedef enum Thermistor_Type
// {
//     THERMISTOR_TYPE_NTC,
//     THERMISTOR_TYPE_PTC,
// }
// Thermistor_Type_T;

// // optionally place in configurable Params, or CONFIG
// typedef struct Thermistor_Coeffs
// {
//     uint16_t VInRef_MilliV;
//     uint32_t R0;
//     uint16_t T0; /* In Kelvin*/
//     uint16_t B;
// }
// Thermistor_Coeffs_T;

/*
    Set Vin to same decimal precision as ADC_VREF
*/
typedef struct Thermistor_Params
{
    // Thermistor_Type_T Type;

    /* NTC Coffceients Conversion */
    uint32_t R0;
    uint16_t T0;    /* In Kelvin*/
    uint16_t B;     /* In Kelvin*/
    uint16_t VInRef_MilliV; /* Generally the same as VADC */

    /* Back Up Linear Unit Conversion. Bypass R, alternatively derive from DeltaT, DeltaR */
    // uint32_t R1;
    // uint16_t T1;
    uint16_t LinearT0_Adcu;
    uint16_t LinearT1_Adcu;
    uint8_t LinearT0_DegC;
    uint8_t LinearT1_DegC;

    /* Monitor Limits */
    uint16_t FaultTrigger_Adcu;
    uint16_t FaultThreshold_Adcu;
    uint16_t WarningTrigger_Adcu;
    uint16_t WarningThreshold_Adcu;

    bool IsMonitorEnable;
}
Thermistor_Params_T;

typedef struct Thermistor_Config
{
    const uint32_t R_SERIES;    /* Pull-up */
    const uint32_t R_PARALLEL;  /* Parallel pull-down if applicable. 0 for Disable */
    //bool IS_CONST;            /* Disable Coefficient set functions */
    // uint32_t R0;
    // uint16_t T0; /* In Kelvin*/
    // uint16_t B;
    const Thermistor_Params_T * P_PARAMS;
}
Thermistor_Config_T;

typedef struct Thermistor
{
    const Thermistor_Config_T CONFIG;
    Thermistor_Params_T Params;
    Linear_T LinearUnits;       /* Back up linear fit. */
    Linear_T HeatLimit;   /* Linear fit for warning region, return value [WarningTrigger_Adcu:FaultTrigger_Adcu] as [65535:0], Roughly linear 70-100C */
// Thermistor_Type_T Type;
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

#define _THERMISTOR_INIT_PARAMS_B(R0, T0_Kelvin, B)     \
{                                                       \
    .R0  = R0,                                          \
    .T0  = T0_Kelvin,                                   \
    .B   = B,                                           \
}                                                       \

#define THERMISTOR_INIT(RSeries, RParallel, p_Params)                   \
{                                                                       \
    .CONFIG = _THERMISTOR_INIT_CONFIG(RSeries, RParallel, p_Params),    \
}

#define THERMISTOR_INIT_WITH_B(RSeries, RParallel, p_Params, coeffR0, coeffT0_Kelvin, coeffB)       \
{                                                                                                   \
    .CONFIG = _THERMISTOR_INIT_CONFIG(RSeries, RParallel, p_Params),                                \
    .Params = _THERMISTOR_INIT_PARAMS_B(R0, coeffT0_Kelvin, coeffB),                                \
}

/******************************************************************************/
/*
    HeatLimit scalar value - Fraction Inverse to heat
    [WarningTrigger_Adcu:FaultTrigger_Adcu] as [65535:0]
*/
/******************************************************************************/
static inline uint16_t Thermistor_HeatLimitOfAdcu_Scalar16(const Thermistor_T * p_therm, uint16_t adcu) { return Linear_ADC_CalcFracU16(&p_therm->HeatLimit, adcu); }
/* Captured adcu on Monitor */
static inline uint16_t Thermistor_GetHeatLimit_Scalar16(const Thermistor_T * p_therm) { return Thermistor_HeatLimitOfAdcu_Scalar16(p_therm, p_therm->Adcu); }

/******************************************************************************/
/*
    Monitor
*/
/******************************************************************************/
static inline Thermistor_Status_T Thermistor_GetStatus(const Thermistor_T * p_therm)        { return (p_therm->Status); }
static inline bool Thermistor_GetIsFault(const Thermistor_T * p_therm)                      { return ((p_therm->Status == THERMISTOR_STATUS_FAULT) || (p_therm->Status == THERMISTOR_STATUS_FAULT_THRESHOLD)); }
static inline bool Thermistor_GetIsWarning(const Thermistor_T * p_therm)                    { return ((p_therm->Status == THERMISTOR_STATUS_WARNING) || (p_therm->Status == THERMISTOR_STATUS_WARNING_THRESHOLD)); }

/******************************************************************************/
/*
    Parameters
*/
/******************************************************************************/
/******************************************************************************/
/* Monitor */
/******************************************************************************/
static inline bool Thermistor_IsMonitorEnable(const Thermistor_T * p_therm)                 { return p_therm->Params.IsMonitorEnable; }
static inline uint16_t Thermistor_GetFaultTrigger_Adcu(const Thermistor_T * p_therm)        { return p_therm->Params.FaultTrigger_Adcu; }
static inline uint16_t Thermistor_GetFaultThreshold_Adcu(const Thermistor_T * p_therm)      { return p_therm->Params.FaultThreshold_Adcu; }
static inline uint16_t Thermistor_GetWarningTrigger_Adcu(const Thermistor_T * p_therm)      { return p_therm->Params.WarningTrigger_Adcu; }
static inline uint16_t Thermistor_GetWarningThreshold_Adcu(const Thermistor_T * p_therm)    { return p_therm->Params.WarningThreshold_Adcu; }

static inline void Thermistor_EnableMonitor(Thermistor_T * p_therm)                                         { p_therm->Params.IsMonitorEnable = true; }
static inline void Thermistor_DisableMonitor(Thermistor_T * p_therm)                                        { p_therm->Params.IsMonitorEnable = false; }
static inline void Thermistor_SetIsMonitorEnable(Thermistor_T * p_therm, bool isEnable)                     { p_therm->Params.IsMonitorEnable = isEnable; }

/* Caller handle validation */
static inline void Thermistor_SetFaultTrigger_Adcu(Thermistor_T * p_therm, uint16_t fault)                  { p_therm->Params.FaultTrigger_Adcu = fault; }
static inline void Thermistor_SetFaultThreshold_Adcu(Thermistor_T * p_therm, uint16_t faultThreshold)       { p_therm->Params.FaultThreshold_Adcu = faultThreshold; }
static inline void Thermistor_SetWarningTrigger_Adcu(Thermistor_T * p_therm, uint16_t warning)              { p_therm->Params.WarningTrigger_Adcu = warning; }
static inline void Thermistor_SetWarningThreshold_Adcu(Thermistor_T * p_therm, uint16_t warningThreshold)   { p_therm->Params.WarningThreshold_Adcu = warningThreshold; }

/******************************************************************************/
/* Units */
/******************************************************************************/
// static inline Thermistor_Type_T Thermistor_GetType(const Thermistor_T * p_therm)    { return p_therm->Params.Type; }
static inline uint32_t Thermistor_GetR0(const Thermistor_T * p_therm)               { return p_therm->Params.R0; }
static inline uint16_t Thermistor_GetT0(const Thermistor_T * p_therm)               { return p_therm->Params.T0; } /* Degrees Kelvin */
static inline uint16_t Thermistor_GetT0_DegC(const Thermistor_T * p_therm)          { return p_therm->Params.T0 + ABSOLUTE_ZERO_CELSIUS; }
static inline uint16_t Thermistor_GetB(const Thermistor_T * p_therm)                { return p_therm->Params.B; }
static inline uint16_t Thermistor_GetVInRef_MilliV(const Thermistor_T * p_therm)    { return p_therm->Params.VInRef_MilliV; }

// static inline void Thermistor_SetType(Thermistor_T * p_therm, uint16_t value)               { p_therm->Params.Type = value; }
static inline void Thermistor_SetR0(Thermistor_T * p_therm, uint16_t value)                 { p_therm->Params.R0 = value; }
static inline void Thermistor_SetT0(Thermistor_T * p_therm, uint16_t value)                 { p_therm->Params.T0 = value; } /* Degrees Kelvin */
static inline void Thermistor_SetT0_DegC(Thermistor_T * p_therm, uint16_t value)            { p_therm->Params.T0 = value - ABSOLUTE_ZERO_CELSIUS; }
static inline void Thermistor_SetB(Thermistor_T * p_therm, uint16_t value)                  { p_therm->Params.B = value; }
static inline void Thermistor_SetVInRef_MilliV(Thermistor_T * p_therm, uint16_t vIn_MilliV) { p_therm->Params.VInRef_MilliV = vIn_MilliV; }

static inline uint16_t Thermistor_GetLinearT0_Adcu(const Thermistor_T * p_therm)    { return p_therm->Params.LinearT0_Adcu; }
static inline uint16_t Thermistor_GetLinearT1_Adcu(const Thermistor_T * p_therm)    { return p_therm->Params.LinearT1_Adcu; }
static inline uint16_t Thermistor_GetLinearT0_DegC(const Thermistor_T * p_therm)    { return p_therm->Params.LinearT0_DegC; }
static inline uint16_t Thermistor_GetLinearT1_DegC(const Thermistor_T * p_therm)    { return p_therm->Params.LinearT1_DegC; }
static inline void Thermistor_SetLinearT0_Adcu(Thermistor_T * p_therm, uint16_t value)      { p_therm->Params.LinearT0_Adcu = value; }
static inline void Thermistor_SetLinearT1_Adcu(Thermistor_T * p_therm, uint16_t value)      { p_therm->Params.LinearT1_Adcu = value; }
static inline void Thermistor_SetLinearT0_DegC(Thermistor_T * p_therm, uint16_t value)      { p_therm->Params.LinearT0_DegC = value; }
static inline void Thermistor_SetLinearT1_DegC(Thermistor_T * p_therm, uint16_t value)      { p_therm->Params.LinearT1_DegC = value; }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void Thermistor_Init(Thermistor_T * p_therm);

extern Thermistor_Status_T Thermistor_PollMonitor(Thermistor_T * p_therm, uint16_t adcu);

extern thermal_t Thermistor_ConvertToDegC(const Thermistor_T * p_therm, uint16_t adcu);
extern uint16_t Thermistor_ConvertToAdcu_DegC(const Thermistor_T * p_therm, thermal_t degC) ;
#if defined(CONFIG_THERMISTOR_UNITS_LINEAR)
extern int32_t Thermistor_ConvertToDegC_Scalar(const Thermistor_T * p_therm, uint16_t adcu, uint8_t scalar);
#endif

extern void Thermistor_SetFault_DegC(Thermistor_T * p_therm, thermal_t fault_degC, thermal_t faultThreshold_degC);
extern void Thermistor_SetWarning_DegC(Thermistor_T * p_therm, thermal_t warning_degC, thermal_t warningThreshold_degC);
extern void Thermistor_SetLimits_DegC(Thermistor_T * p_therm, thermal_t fault, thermal_t faultThreshold, thermal_t warning, thermal_t warningThreshold);

extern thermal_t Thermistor_GetFault_DegC(const Thermistor_T * p_therm);
extern thermal_t Thermistor_GetFaultThreshold_DegC(const Thermistor_T * p_therm);
extern thermal_t Thermistor_GetWarning_DegC(const Thermistor_T * p_therm);
extern thermal_t Thermistor_GetWarningThreshold_DegC(const Thermistor_T * p_therm);

#if defined(CONFIG_THERMISTOR_UNITS_LINEAR)
extern int32_t Thermistor_GetFault_DegCScalar(const Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetFaultThreshold_DegCScalar(const Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetWarning_DegCScalar(const Thermistor_T * p_therm, uint16_t scalar);
extern int32_t Thermistor_GetWarningThreshold_DegCScalar(const Thermistor_T * p_therm, uint16_t scalar);
#endif

#endif

/* Using async capture conversion, e.g. conversion processing lower frequency than user output */
// static inline int32_t Thermistor_GetHeat_DegC(Thermistor_T * p_therm) { return p_therm->Heat_DegC; }
// static inline void Thermistor_CaptureUnits_DegC(Thermistor_T * p_therm, uint16_t adcu) { p_therm->Heat_DegC = Thermistor_ConvertToDegC_Scalar(p_therm, adcu, p_therm->Params.CaptureScalar);}
