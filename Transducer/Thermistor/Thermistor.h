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
#include "Peripheral/Analog/AnalogReference.h"
#include "Math/Linear/Linear_ADC.h"
#include <stdint.h>
#include <stdbool.h>

#if     defined(CONFIG_THERMISTOR_UNITS_LINEAR) || defined(CONFIG_THERMISTOR_UNITS_LUT)
    typedef int16_t thermal_t;
    static const thermal_t ABSOLUTE_ZERO_CELSIUS = -273;
#elif   defined(CONFIG_THERMISTOR_UNITS_FLOAT)
    typedef float thermal_t;
    static const thermal_t ABSOLUTE_ZERO_CELSIUS = -273.15F;
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

// Thermistor_StatusBits_T;

/*   */
typedef enum Thermistor_Type
{
    THERMISTOR_TYPE_NTC,
    THERMISTOR_TYPE_PTC,
    THERMISTOR_TYPE_LINEAR,
}
Thermistor_Type_T;

// optionally place in configurable Config, or CONFIG
// typedef struct Thermistor_Coefficients
// {
//     uint16_t B;
//     uint32_t R0;
//     uint16_t T0; /* In Kelvin*/
//     uint16_t VInRef_MilliV;
// }
// Thermistor_Coefficients_T;

/*
    Set Vin to same decimal precision as ADC_VREF
*/
typedef struct Thermistor_Config
{
    /* Unit Conversion */
    Thermistor_Type_T Type;
    uint16_t B;     /* In Kelvin */
    uint32_t R0;
    uint16_t T0;    /* In Kelvin */
    uint16_t VInRef_MilliV; /* Generally the same as VADC */
    // Thermistor_Coeffs_T directly wtihout pointer

    /* Back Up Linear Unit Conversion. Derive from DeltaR, DeltaT */
    // uint32_t DeltaR;
    // uint16_t DeltaT;
    // uint16_t Linear_T0;
    // int32_t LinearCoefficient_Fixed32;
        // uint16_t R/T;
        // uint16_t V/T; /* VADC / K */

    /* Monitor Limits */
    uint16_t FaultTrigger_Adcu;
    uint16_t FaultThreshold_Adcu;
    uint16_t WarningTrigger_Adcu;
    uint16_t WarningThreshold_Adcu;

    bool IsMonitorEnable;
}
Thermistor_Config_T;

typedef struct Thermistor_Const
{
    const Thermistor_Config_T * P_CONFIG; /* Optional NvM */
    const uint32_t R_SERIES;    /* Pull-up */
    const uint32_t R_PARALLEL;  /* Parallel pull-down if applicable. 0 for Disable */
    // Thermistor_Coeffs_T * P_FIXED_COEFFICIENTS;
    /* Non detachable. Disable Coefficient set functions */
    // configure as pointer to config ram or const?
    // allocate both, if const load from config
}
Thermistor_Const_T;

typedef struct Thermistor
{
    const Thermistor_Const_T CONST;
    Thermistor_Config_T Config;
    Linear_T LinearUnits;   /* Back up linear fit. */
    Linear_T HeatLimit;     /* Linear fit for warning region, return value [WarningTrigger_Adcu:FaultTrigger_Adcu] as [65535:0], Roughly linear 70-100C */

    /* State for polling compare */
    Thermistor_Status_T Status; /* Status is sufficient for brief State */
    uint16_t Adcu; /* Previous ADC sample */
}
Thermistor_T;

#define _THERMISTOR_INIT_CONST(RSeries, RParallel, p_Config)   \
{                                                        \
    .R_SERIES       = RSeries,                           \
    .R_PARALLEL     = RParallel,                         \
    .P_CONFIG       = p_Config,                          \
}

#define _THERMISTOR_INIT_CONFIG_B(B, R0, T0_Kelvin)     \
{                                                       \
    .R0  = R0,                                          \
    .T0  = T0_Kelvin,                                   \
    .B   = B,                                           \
}                                                       \

// #define _THERMISTOR_INIT_CONFIG_CONST_COEFF(B, R0, T0_Kelvin)     \
// {                                                       \
//     .R_SERIES       = RSeries,                          \
//     .R_PARALLEL     = RParallel,                        \
//     .P_CONFIG       = p_Config,                         \
//     .R0  = R0,                                          \
//     .T0  = T0_Kelvin,                                   \
//     .B   = B,                                           \
// }                                                       \

#define THERMISTOR_INIT(RSeries, RParallel, p_Config)                   \
{                                                                       \
    .CONST = _THERMISTOR_INIT_CONST(RSeries, RParallel, p_Config),      \
}

#define THERMISTOR_INIT_FIXED(RSeries, RParallel, B, R0, T0_Kelvin, p_Config)   \
{                                                                               \
    .CONST = _THERMISTOR_INIT(RSeries, RParallel, p_Config),                    \
    .Config = _THERMISTOR_INIT_CONFIG_B(B, R0, T0_Kelvin),                      \
}

typedef enum Thermistor_ConfigId
{
    THERMISTOR_CONFIG_R_SERIES, // All instance Read-Only
    THERMISTOR_CONFIG_R_PARALLEL, // All instance Read-Only
    THERMISTOR_CONFIG_TYPE,
    THERMISTOR_CONFIG_R0,
    THERMISTOR_CONFIG_T0,
    THERMISTOR_CONFIG_B,
    // THERMISTOR_CONFIG_LINEAR_T0_ADCU,
    // THERMISTOR_CONFIG_LINEAR_T1_ADCU,
    // THERMISTOR_CONFIG_LINEAR_T0_DEG_C,
    // THERMISTOR_CONFIG_LINEAR_T1_DEG_C,
    // THERMISTOR_CONFIG_LINEAR_DELTA_R,
    // THERMISTOR_CONFIG_LINEAR_DELTA_T_ADCU,
    // THERMISTOR_CONFIG_LINEAR_DELTA_T_DEG,
    THERMISTOR_CONFIG_FAULT_TRIGGER_ADCU,
    THERMISTOR_CONFIG_FAULT_THRESHOLD_ADCU,
    THERMISTOR_CONFIG_WARNING_TRIGGER_ADCU,
    THERMISTOR_CONFIG_WARNING_THRESHOLD_ADCU,
    THERMISTOR_CONFIG_IS_MONITOR_ENABLE,
}
Thermistor_ConfigId_T;

/******************************************************************************/
/*
    HeatLimit scalar value - Fraction Inverse to heat
    [WarningTrigger_Adcu:FaultTrigger_Adcu] as [65535:0]
*/
/******************************************************************************/
/* todo update Init */
static inline uint16_t Thermistor_HeatLimitOfAdcu_Percent16(const Thermistor_T * p_therm, uint16_t adcu) { return Linear_ADC_Percent16(&p_therm->HeatLimit, adcu); }
/* Captured adcu on Thermistor_PollMonitor */
static inline uint16_t Thermistor_GetHeatLimit_Percent16(const Thermistor_T * p_therm) { return Thermistor_HeatLimitOfAdcu_Percent16(p_therm, p_therm->Adcu); }

/******************************************************************************/
/*
    Monitor
*/
/******************************************************************************/
static inline Thermistor_Status_T Thermistor_GetStatus(const Thermistor_T * p_therm)    { return (p_therm->Status); }
static inline bool Thermistor_IsFault(const Thermistor_T * p_therm)                     { return ((p_therm->Status == THERMISTOR_STATUS_FAULT) || (p_therm->Status == THERMISTOR_STATUS_FAULT_THRESHOLD)); }
static inline bool Thermistor_IsWarning(const Thermistor_T * p_therm)                   { return ((p_therm->Status == THERMISTOR_STATUS_WARNING) || (p_therm->Status == THERMISTOR_STATUS_WARNING_THRESHOLD)); }

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
static inline bool Thermistor_IsMonitorEnable(const Thermistor_T * p_therm)                 { return p_therm->Config.IsMonitorEnable; }
static inline void Thermistor_EnableMonitor(Thermistor_T * p_therm)                         { p_therm->Config.IsMonitorEnable = true; }
static inline void Thermistor_DisableMonitor(Thermistor_T * p_therm)                        { p_therm->Config.IsMonitorEnable = false; }
static inline void Thermistor_SetIsMonitorEnable(Thermistor_T * p_therm, bool isEnable)     { p_therm->Config.IsMonitorEnable = isEnable; }

static inline uint16_t Thermistor_GetFaultTrigger_Adcu(const Thermistor_T * p_therm)        { return p_therm->Config.FaultTrigger_Adcu; }
static inline uint16_t Thermistor_GetFaultThreshold_Adcu(const Thermistor_T * p_therm)      { return p_therm->Config.FaultThreshold_Adcu; }
static inline uint16_t Thermistor_GetWarningTrigger_Adcu(const Thermistor_T * p_therm)      { return p_therm->Config.WarningTrigger_Adcu; }
static inline uint16_t Thermistor_GetWarningThreshold_Adcu(const Thermistor_T * p_therm)    { return p_therm->Config.WarningThreshold_Adcu; }
/* Caller handle validation */
static inline void Thermistor_SetFaultTrigger_Adcu(Thermistor_T * p_therm, uint16_t fault)                  { p_therm->Config.FaultTrigger_Adcu = fault; }
static inline void Thermistor_SetFaultThreshold_Adcu(Thermistor_T * p_therm, uint16_t faultThreshold)       { p_therm->Config.FaultThreshold_Adcu = faultThreshold; }
static inline void Thermistor_SetWarningTrigger_Adcu(Thermistor_T * p_therm, uint16_t warning)              { p_therm->Config.WarningTrigger_Adcu = warning; }
static inline void Thermistor_SetWarningThreshold_Adcu(Thermistor_T * p_therm, uint16_t warningThreshold)   { p_therm->Config.WarningThreshold_Adcu = warningThreshold; }

/******************************************************************************/
/* Units Calibration */
/******************************************************************************/
static inline uint16_t Thermistor_GetVInRef_MilliV(const Thermistor_T * p_therm)    { return p_therm->Config.VInRef_MilliV; }
static inline Thermistor_Type_T Thermistor_GetType(const Thermistor_T * p_therm)    { return p_therm->Config.Type; }
static inline uint32_t Thermistor_GetR0(const Thermistor_T * p_therm)               { return p_therm->Config.R0; }
static inline uint16_t Thermistor_GetT0(const Thermistor_T * p_therm)               { return p_therm->Config.T0; } /* Degrees Kelvin */
static inline uint16_t Thermistor_GetT0_DegC(const Thermistor_T * p_therm)          { return p_therm->Config.T0 + ABSOLUTE_ZERO_CELSIUS; }
static inline uint16_t Thermistor_GetB(const Thermistor_T * p_therm)                { return p_therm->Config.B; }

static inline void Thermistor_SetVInRef_MilliV(Thermistor_T * p_therm, uint16_t vIn_MilliV) { p_therm->Config.VInRef_MilliV = vIn_MilliV; }
static inline void Thermistor_SetType(Thermistor_T * p_therm, uint16_t value)               { p_therm->Config.Type = value; }

// todo with IsConst?
static inline void Thermistor_SetR0(Thermistor_T * p_therm, uint16_t value)                 { p_therm->Config.R0 = value; }
static inline void Thermistor_SetT0(Thermistor_T * p_therm, uint16_t value)                 { p_therm->Config.T0 = value; } /* Degrees Kelvin */
static inline void Thermistor_SetT0_DegC(Thermistor_T * p_therm, uint16_t value)            { p_therm->Config.T0 = value - ABSOLUTE_ZERO_CELSIUS; }
static inline void Thermistor_SetB(Thermistor_T * p_therm, uint16_t value)                  { p_therm->Config.B = value; }

// static inline uint16_t Thermistor_GetLinearR(const Thermistor_T * p_therm)            { return p_therm->Config.DeltaR; }
// static inline uint16_t Thermistor_GetLinearT(const Thermistor_T * p_therm)            { return p_therm->Config.DeltaT; }
// static inline void Thermistor_SetLinearR(Thermistor_T * p_therm, uint16_t value)      { p_therm->Config.DeltaR = value; }
// static inline void Thermistor_SetLinearT(Thermistor_T * p_therm, uint16_t value)      { p_therm->Config.DeltaT = value; }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void Thermistor_Init(Thermistor_T * p_therm);

extern Thermistor_Status_T Thermistor_PollMonitor(Thermistor_T * p_therm, uint16_t adcu);

extern thermal_t Thermistor_ConvertToDegC(const Thermistor_T * p_therm, uint16_t adcu);
extern uint16_t Thermistor_ConvertToAdcu_DegC(const Thermistor_T * p_therm, thermal_t degC) ;

extern void Thermistor_SetFault_DegC(Thermistor_T * p_therm, thermal_t fault_degC, thermal_t faultThreshold_degC);
extern void Thermistor_SetWarning_DegC(Thermistor_T * p_therm, thermal_t warning_degC, thermal_t warningThreshold_degC);
extern void Thermistor_SetLimits_DegC(Thermistor_T * p_therm, thermal_t fault, thermal_t faultThreshold, thermal_t warning, thermal_t warningThreshold);

extern thermal_t Thermistor_GetFault_DegC(const Thermistor_T * p_therm);
extern thermal_t Thermistor_GetFaultThreshold_DegC(const Thermistor_T * p_therm);
extern thermal_t Thermistor_GetWarning_DegC(const Thermistor_T * p_therm);
extern thermal_t Thermistor_GetWarningThreshold_DegC(const Thermistor_T * p_therm);

extern int32_t Thermistor_ConfigId_Get(const Thermistor_T * p_thermistor, Thermistor_ConfigId_T id);
extern void Thermistor_ConfigId_Set(Thermistor_T * p_thermistor, Thermistor_ConfigId_T id, int32_t value);

#endif

/* Using async capture conversion, e.g. conversion processing lower frequency than user output */
// static inline int32_t Thermistor_GetHeat_DegC(Thermistor_T * p_therm) { return p_therm->Heat_DegC; }
// static inline void Thermistor_CaptureUnits_DegC(Thermistor_T * p_therm, uint16_t adcu) { p_therm->Heat_DegC = Thermistor_ConvertToDegC_Scalar(p_therm, adcu, p_therm->Config.CaptureScalar);}
