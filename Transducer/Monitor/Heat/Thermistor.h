#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Peripheral/Analog/Analog_Reference.h"
#include "Peripheral/Analog/Linear_ADC.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

/******************************************************************************/
/*!

*/
/******************************************************************************/
#if defined(THERMISTOR_UNITS_LINEAR) || defined(THERMISTOR_UNITS_LUT)
    #define THERMISTOR_VALUE_T int16_t
#elif defined(THERMISTOR_UNITS_FLOAT)
    #define THERMISTOR_VALUE_T float
#else
    #define THERMISTOR_VALUE_T int16_t
#endif

typedef THERMISTOR_VALUE_T thermal_t; /* Use thermal_value_t for all logic */

static const thermal_t ABSOLUTE_ZERO_CELSIUS = -273.15F;

/*   */
typedef enum Thermistor_Type
{
    THERMISTOR_TYPE_NTC,
    THERMISTOR_TYPE_NTC_PULL_UP,
    THERMISTOR_TYPE_PTC,
    THERMISTOR_TYPE_PTC_PULL_UP,
    THERMISTOR_TYPE_LINEAR,
    THERMISTOR_TYPE_LUT,
}
Thermistor_Type_T;

// in configurable Config, or FIXED
typedef struct Thermistor_Coeffs
{
    uint16_t B;
    uint32_t R0;
    uint16_t T0; /* In Kelvin. */
    // Thermistor_Type_T Type; /* Unused for now */
    // uint16_t VInRef_MilliV;

    /* Back Up Linear Unit Conversion. */
    int32_t DeltaR;
    int16_t DeltaT;
}
Thermistor_Coeffs_T;

#define THERMISTOR_COEFF_INIT(TypeId, BValue, R0Value, T0_Kelvin) { .B = BValue, .R0 = R0Value, .T0 = T0_Kelvin, }
#define THERMISTOR_COEFF_ALLOC(TypeId, BValue, R0Value, T0_Kelvin) (&(Thermistor_Coeffs_T)THERMISTOR_COEFF_INIT(TypeId, BValue, R0Value, T0_Kelvin))
#define THERMISTOR_COEFF_ALLOC_ZERO() (&(Thermistor_Coeffs_T){0})

/*

*/
typedef const struct Thermistor
{
    /* Board */
    uint32_t R_SERIES;    /* Pull-up */
    uint32_t R_PARALLEL;  /* Parallel pull-down if applicable. 0 for Disable */
    uint16_t V_SERIES_MV; /* If VSeries is different than AdcVRef */

    Thermistor_Coeffs_T * P_COEFFS; /* read access */
    const Thermistor_Coeffs_T * P_CONST_COEFFS; /* Nvm or compile time constant */
}
Thermistor_T;

#define THERMISTOR_V_SERIES_MV_NULL (0U) /* Use AdcVRef */
// #define _THERMISTOR_BOARD_INIT(RSeries, RParallel, VSeries) { .R_SERIES = RSeries, .R_PARALLEL = RParallel, .V_SERIES_MV = VSeries, }

#define THERMISTOR_INIT(RSeries, RParallel, VSeries, p_Coeffs, p_ConstCoeffs) \
    { .R_SERIES = RSeries, .R_PARALLEL = RParallel, .V_SERIES_MV = VSeries, .P_COEFFS = p_Coeffs, .P_CONST_COEFFS = p_ConstCoeffs, }

/*
    Init as Configurable/Detachable
    p_NvmCoeffs maps to P_CONST_COEFFS — for wired instances, the const baseline is NVM-backed.
*/
#define THERMISTOR_WIRED_INIT(RSeries, RParallel, VSeries, p_Coeffs, p_NvmCoeffs) \
    { .R_SERIES = (RSeries), .R_PARALLEL = (RParallel), .V_SERIES_MV = (VSeries), .P_COEFFS = (p_Coeffs), .P_CONST_COEFFS = (p_NvmCoeffs), }

/*
    Init as Fixed to Board
    Define in 2 steps:
        static const Thermistor_Coeffs_T NAME = THERMISTOR_COEFF_FIXED_INIT(...);
        THERMISTOR_FIXED_INIT(..., &NAME)
    Set guarded by Thermistor_IsFixed (pointer-identity sentinel).
*/
#define THERMISTOR_COEFF_FIXED_INIT(BValue, R0Value, T0_Kelvin) { .B = (BValue), .R0 = (R0Value), .T0 = (T0_Kelvin), }
#define THERMISTOR_FIXED_INIT(RSeries, RParallel, VSeries, p_FixedCoeffs) \
    THERMISTOR_INIT((RSeries), (RParallel), (VSeries), (Thermistor_Coeffs_T *)(p_FixedCoeffs), (p_FixedCoeffs))

/*
    Pointer-identity sentinel: P_COEFFS == P_CONST_COEFFS ⇒ aliased fixed instance.
    Release-safe: returns false when P_COEFFS is NULL (misconfig); assert traps it in debug.
*/
static inline bool Thermistor_IsFixed(const Thermistor_T * p_therm) { return (p_therm->P_COEFFS == p_therm->P_CONST_COEFFS); }
/* Get as read only */
static inline const Thermistor_Coeffs_T * Thermistor_GetCoeffs(const Thermistor_T * p_therm) { return p_therm->P_COEFFS; }
/* Get as configurable — non-NULL only when not fixed */
static inline Thermistor_Coeffs_T * Thermistor_GetCoeffsConfigurable(const Thermistor_T * p_therm) { return Thermistor_IsFixed(p_therm) ? NULL : p_therm->P_COEFFS; }
/* Const baseline source (NVM or .rodata) — non-NULL only when not fixed */
static inline const Thermistor_Coeffs_T * Thermistor_GetCoeffsNvm(const Thermistor_T * p_therm) { return Thermistor_IsFixed(p_therm) ? NULL : p_therm->P_CONST_COEFFS; }
/* Get as fixed */
static inline const Thermistor_Coeffs_T * Thermistor_GetCoeffsFixed(const Thermistor_T * p_therm) { return Thermistor_IsFixed(p_therm) ? p_therm->P_CONST_COEFFS : NULL; }


/******************************************************************************/
/*
*/
/******************************************************************************/
static inline thermal_t _Thermistor_CelsiusOfKelvin(thermal_t kelvin) { return (kelvin + ABSOLUTE_ZERO_CELSIUS); }
static inline thermal_t _Thermistor_KelvinOfCelsius(thermal_t celsius) { return (celsius - ABSOLUTE_ZERO_CELSIUS); }

/******************************************************************************/
/*
*/
/******************************************************************************/

static inline uint16_t Thermistor_GetB(const Thermistor_T * p_therm) { return Thermistor_GetCoeffs(p_therm)->B; }
static inline uint32_t Thermistor_GetR0(const Thermistor_T * p_therm) { return Thermistor_GetCoeffs(p_therm)->R0; }
static inline thermal_t Thermistor_GetT0(const Thermistor_T * p_therm) { return Thermistor_GetCoeffs(p_therm)->T0; }
static inline thermal_t Thermistor_GetT0_Kelvin(const Thermistor_T * p_therm) { return Thermistor_GetCoeffs(p_therm)->T0; }
static inline thermal_t Thermistor_GetT0_Celsius(const Thermistor_T * p_therm) { return (Thermistor_GetT0(p_therm) + ABSOLUTE_ZERO_CELSIUS); }
static inline int32_t Thermistor_GetLinearDeltaR(const Thermistor_T * p_therm) { return Thermistor_GetCoeffs(p_therm)->DeltaR; }
static inline int16_t Thermistor_GetLinearDeltaT(const Thermistor_T * p_therm) { return Thermistor_GetCoeffs(p_therm)->DeltaT; }
// static inline Thermistor_Type_T Thermistor_GetType(const Thermistor_T * p_therm)    { return Thermistor_GetCoeffs(p_therm)->Type; }
// static inline bool Thermistor_IsPulldown(const Thermistor_T * p_therm)

static inline uint32_t Thermistor_GetRSeries(const Thermistor_T * p_therm) { return p_therm->R_SERIES; }
static inline uint32_t Thermistor_GetRParallel(const Thermistor_T * p_therm) { return p_therm->R_PARALLEL; } /* 0 for Disable */
static inline uint16_t _Thermistor_GetVInRef_MilliV(const Thermistor_T * p_therm) { return p_therm->V_SERIES_MV; } /* If VRef is different than ADC */

static inline uint16_t Thermistor_GetVInRef_MilliV(const Thermistor_T * p_therm) { return (p_therm->V_SERIES_MV == 0U) ? ANALOG_REFERENCE.ADC_VREF_MILLIV : p_therm->V_SERIES_MV; } /* If VRef is different than ADC */
static inline uint16_t Thermistor_GetVAdcRef_MilliV(void) { return ANALOG_REFERENCE.ADC_VREF_MILLIV; }
static inline uint16_t Thermistor_GetVAdcMax(void) { return ANALOG_REFERENCE.ADC_MAX; }

/* Unchecked setters — caller MUST check Thermistor_IsFixed first.
   Internal use only (Id-table dispatchers). External callers should use the public API. */
static inline void _Thermistor_SetR0(const Thermistor_T * p_therm, uint32_t value) { p_therm->P_COEFFS->R0 = value; }
static inline void _Thermistor_SetT0(const Thermistor_T * p_therm, thermal_t value) { p_therm->P_COEFFS->T0 = (uint16_t)value; } /* Degrees Kelvin */
static inline void _Thermistor_SetT0_Celsius(const Thermistor_T * p_therm, thermal_t value) { p_therm->P_COEFFS->T0 = (uint16_t)(value - ABSOLUTE_ZERO_CELSIUS); }
static inline void _Thermistor_SetB(const Thermistor_T * p_therm, uint16_t value) { p_therm->P_COEFFS->B = value; }
static inline void _Thermistor_SetLinearDeltaR(const Thermistor_T * p_therm, int32_t value) { p_therm->P_COEFFS->DeltaR = value; }
static inline void _Thermistor_SetLinearDeltaT(const Thermistor_T * p_therm, int16_t value) { p_therm->P_COEFFS->DeltaT = value; }
// static inline void _Thermistor_SetType(const Thermistor_T * p_therm, uint16_t value)      { p_therm->P_COEFFS->Type = value; }

/******************************************************************************/
/*
*/
/******************************************************************************/
extern void Thermistor_InitFrom(const Thermistor_T * p_therm, const Thermistor_Coeffs_T * p_config);
extern void Thermistor_Init(const Thermistor_T * p_therm);

extern uint32_t Thermistor_ROhmOfAdcu(const Thermistor_T * p_therm, uint16_t adcu);
extern uint16_t Thermistor_AdcuOfROhm(const Thermistor_T * p_therm, uint32_t rThermistor);

extern void Thermistor_ToLinear_ROhmsPerAdcu(const Thermistor_T * p_therm, Linear_T * p_result);
extern void Thermistor_ToLinear_CelsiusPerAdcu(const Thermistor_T * p_therm, Linear_T * p_result);
extern void Thermistor_ToLinear_CelsiusPerROhms(const Thermistor_T * p_therm, Linear_T * p_result);

extern thermal_t Thermistor_CelsiusOfAdcu(const Thermistor_T * p_therm, uint16_t adcu);
extern uint16_t Thermistor_AdcuOfCelsius(const Thermistor_T * p_therm, thermal_t celsius);

/******************************************************************************/
/*
    Id Access
*/
/******************************************************************************/
typedef enum Thermistor_ConfigId
{
    /* Board RefId */
    THERMISTOR_BOARD_R_SERIES, // All instance Read-Only
    THERMISTOR_BOARD_R_PARALLEL, // All instance Read-Only
    THERMISTOR_BOARD_V_SERIES_MV, // All instance Read-Only
    /* Config CoeffId */
    THERMISTOR_CONFIG_TYPE, /* Resv */
    THERMISTOR_CONFIG_B,
    THERMISTOR_CONFIG_R0,
    THERMISTOR_CONFIG_T0,
    THERMISTOR_CONFIG_LINEAR_DELTA_R,
    THERMISTOR_CONFIG_LINEAR_DELTA_T,
}
Thermistor_ConfigId_T;

extern int32_t _Thermistor_ConfigId_Get(const Thermistor_T * p_therm, Thermistor_ConfigId_T id);
extern void _Thermistor_ConfigId_Set(const Thermistor_T * p_therm, Thermistor_ConfigId_T id, int32_t value);

extern int Thermistor_ConfigId_Get(const Thermistor_T * p_therm, Thermistor_ConfigId_T id);
extern void Thermistor_ConfigId_Set(const Thermistor_T * p_therm, Thermistor_ConfigId_T id, int value);