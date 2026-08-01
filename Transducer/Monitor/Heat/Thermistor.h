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


// typedef enum Thermistor_Type
// {
//     THERMISTOR_TYPE_NTC,
//     THERMISTOR_TYPE_PTC,
// }
// Thermistor_Type_T;
typedef enum { THERM_FIT_NONE, THERM_FIT_LINEAR, THERM_FIT_STEINHART, THERM_FIT_QUADRATIC } Thermistor_Fit_T;

/*
    Configurable Config, or FIXED to Board
    Determine coefficent sign from values filled.    0 = unused
    Alternatively use explicit parameter.
*/
typedef struct Thermistor_Coeffs
{
    /*
        Per-type curve coefficients. Anonymous structs keep members flat (->B, ->Alpha, ->Beta).
        A sensor is one type at a time, so these overlap.
    */
    struct { uint16_t B; };              /* NTC: Steinhart-Hart B-parameter [Kelvin] */
    struct { float Alpha; float Beta; }; /* PTC: quadratic R = R0*(1 + Alpha*dT + Beta*dT^2). KTY silicon / RTD Callendar-Van Dusen */

    uint32_t R0;
    uint16_t T0; /* In Kelvin. */

    /*
        Two-point linear model: (R0, T0) .. (R0 + DeltaR, T0 + DeltaT).
        Primary R<->T for PTC. Backup local linearization for NTC.
        For a pull-down PTC, DeltaR and DeltaT share sign (R rises with T).
    */
    int32_t DeltaR;
    int16_t DeltaT;

    /*
        optionally resolvable from parameters
        must be set first to determine which parameters are effective and shown on the UI
        when it is THERM_FIT_NONE, show all parameters, resolve locally.
    */
   Thermistor_Fit_T Fit;
    // Thermistor_Type_T Type;
}
Thermistor_Coeffs_T;

#define THERMISTOR_COEFF_ALLOC_ZERO() (&(Thermistor_Coeffs_T){0})

/*
    PTC / linear silicon sensor characterization (e.g. KTY84).
    Two points define the line: (R0, T0) and (R0 + DeltaR, T0 + DeltaT).
*/
#define THERMISTOR_COEFF_LINEAR_INIT(R0Value, T0_Kelvin, DeltaROhm, DeltaTKelvin) (Thermistor_Coeffs_T) \
    { .R0 = (R0Value), .T0 = (T0_Kelvin), .DeltaR = (DeltaROhm), .DeltaT = (DeltaTKelvin), .Fit = THERM_FIT_LINEAR, }

/*
    Silicon PTC quadratic characterization (KTY81/83/84, or RTD Callendar-Van Dusen).
    R(T) = R0 * (1 + Alpha*(T - T0) + Beta*(T - T0)^2), T0 in Kelvin.
    Optionally also set DeltaR/DeltaT for the non-float (linear) build fallback.
*/
#define THERMISTOR_COEFF_QUADRATIC_INIT(AlphaValue, BetaValue, R0Value, T0_Kelvin) (Thermistor_Coeffs_T) \
    { .R0 = (R0Value), .T0 = (T0_Kelvin), .Alpha = (AlphaValue), .Beta = (BetaValue), .Fit = THERM_FIT_QUADRATIC, }


#define THERMISTOR_COEFF_STEINHART_INIT(BValue, R0Value, T0_Kelvin) (Thermistor_Coeffs_T) \
    { .B = (BValue), .R0 = (R0Value), .T0 = (T0_Kelvin), .Fit = THERM_FIT_STEINHART, }


/* NTC Steinhart-Hart B-parameter model applies; negative tempco (R falls as T rises). */
static inline bool Thermistor_IsNtc(const Thermistor_Coeffs_T * p_coeffs) { return (p_coeffs->B != 0U); }

/* Positive tempco: resistance rises with temperature (silicon PTC). For pull-down wiring, ADC rises with temperature. */
static inline bool Thermistor_IsPtc(const Thermistor_Coeffs_T * p_coeffs) { return (p_coeffs->Alpha != 0.0f) || (p_coeffs->Beta != 0.0f); }

/* using DetaT as authoritative */
// static inline bool Thermistor_IsNtc(const Thermistor_Coeffs_T * p_coeffs) { return (p_coeffs->DeltaT < 0); }
// static inline bool Thermistor_IsPtc(const Thermistor_Coeffs_T * p_coeffs) { return (p_coeffs->DeltaT > 0); }

/* Resolve the R<->T model from the populated coefficients. Operates on the bare descriptor (wire-portable). */
static inline Thermistor_Fit_T Thermistor_FitOf(const Thermistor_Coeffs_T * p_coeffs)
{
#if defined(THERMISTOR_UNITS_FLOAT)
    if (p_coeffs->B != 0U)                                        return THERM_FIT_STEINHART;
    if ((p_coeffs->Alpha != 0.0f) || (p_coeffs->Beta != 0.0f))    return THERM_FIT_QUADRATIC;
#endif
    if (p_coeffs->DeltaR != 0)                                    return THERM_FIT_LINEAR;
    // if ( )                                                     return THERM_FIT_LUT;
    return THERM_FIT_NONE;
}

/*
    assume Pull-down only
*/
// typedef enum { THERMISTOR_WIRING_PULLDOWN, THERMISTOR_WIRING_PULLUP } Thermistor_Wiring_T;
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
        static const Thermistor_Coeffs_T NAME = THERMISTOR_COEFF_STEINHART_INIT(...);
        THERMISTOR_FIXED_INIT(..., &NAME)
    Set guarded by Thermistor_IsFixed (pointer-identity sentinel).
*/
#define THERMISTOR_FIXED_INIT(RSeries, RParallel, VSeries, p_FixedCoeffs) \
    THERMISTOR_INIT((RSeries), (RParallel), (VSeries), (Thermistor_Coeffs_T *)(p_FixedCoeffs), (p_FixedCoeffs))

/*
    Pointer-identity sentinel: P_COEFFS == P_CONST_COEFFS ⇒ aliased fixed instance.
    Release-safe: returns false when P_COEFFS is NULL (misconfig); assert traps it in debug.
*/
static inline bool Thermistor_IsFixed(const Thermistor_T * p_therm) { return (p_therm->P_COEFFS == p_therm->P_CONST_COEFFS); }
/* Get as read only */
static inline const Thermistor_Coeffs_T * Thermistor_Coeffs(const Thermistor_T * p_therm) { return p_therm->P_COEFFS; }
/* Get as configurable — non-NULL only when not fixed */
static inline Thermistor_Coeffs_T * Thermistor_CoeffsConfigurable(const Thermistor_T * p_therm) { return Thermistor_IsFixed(p_therm) ? NULL : p_therm->P_COEFFS; }
/* Const baseline source (NVM or .rodata) — non-NULL only when not fixed */
static inline const Thermistor_Coeffs_T * Thermistor_CoeffsNvm(const Thermistor_T * p_therm) { return Thermistor_IsFixed(p_therm) ? NULL : p_therm->P_CONST_COEFFS; }
/* Get as fixed */
static inline const Thermistor_Coeffs_T * Thermistor_CoeffsFixed(const Thermistor_T * p_therm) { return Thermistor_IsFixed(p_therm) ? p_therm->P_CONST_COEFFS : NULL; }


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
static inline uint16_t Thermistor_GetB(const Thermistor_T * p_therm) { return Thermistor_Coeffs(p_therm)->B; }
static inline float Thermistor_GetAlpha(const Thermistor_T * p_therm) { return Thermistor_Coeffs(p_therm)->Alpha; } /* PTC quadratic linear tempco [1/K] */
static inline float Thermistor_GetBeta(const Thermistor_T * p_therm) { return Thermistor_Coeffs(p_therm)->Beta; }   /* PTC quadratic 2nd-order tempco [1/K^2] */
static inline uint32_t Thermistor_GetR0(const Thermistor_T * p_therm) { return Thermistor_Coeffs(p_therm)->R0; }
static inline thermal_t Thermistor_GetT0(const Thermistor_T * p_therm) { return Thermistor_Coeffs(p_therm)->T0; }
static inline thermal_t Thermistor_GetT0_Kelvin(const Thermistor_T * p_therm) { return Thermistor_Coeffs(p_therm)->T0; }
static inline thermal_t Thermistor_GetT0_Celsius(const Thermistor_T * p_therm) { return (Thermistor_GetT0(p_therm) + ABSOLUTE_ZERO_CELSIUS); }
static inline int32_t Thermistor_GetLinearDeltaR(const Thermistor_T * p_therm) { return Thermistor_Coeffs(p_therm)->DeltaR; }
static inline int16_t Thermistor_GetLinearDeltaT(const Thermistor_T * p_therm) { return Thermistor_Coeffs(p_therm)->DeltaT; }
static inline Thermistor_Fit_T Thermistor_GetFit(const Thermistor_T * p_therm) { return Thermistor_Coeffs(p_therm)->Fit; }



static inline uint32_t Thermistor_GetRSeries(const Thermistor_T * p_therm) { return p_therm->R_SERIES; }
static inline uint32_t Thermistor_GetRParallel(const Thermistor_T * p_therm) { return p_therm->R_PARALLEL; } /* 0 for Disable */
static inline uint16_t _Thermistor_GetVInRef_MilliV(const Thermistor_T * p_therm) { return p_therm->V_SERIES_MV; } /* If VRef is different than ADC */

static inline uint16_t Thermistor_GetVInRef_MilliV(const Thermistor_T * p_therm) { return (p_therm->V_SERIES_MV == 0U) ? ANALOG_REFERENCE.ADC_VREF_MILLIV : p_therm->V_SERIES_MV; } /* If VRef is different than ADC */
static inline uint16_t Thermistor_GetVAdcRef_MilliV(void) { return ANALOG_REFERENCE.ADC_VREF_MILLIV; }
static inline uint16_t Thermistor_GetVAdcMax(void) { return ANALOG_REFERENCE.ADC_MAX; }

/*
    Unchecked setters — caller MUST check Thermistor_IsFixed first.
    Internal use only (Id-table dispatchers). External callers should use the public API.
*/
static inline void _Thermistor_SetR0(const Thermistor_T * p_therm, uint32_t value) { p_therm->P_COEFFS->R0 = value; }
static inline void _Thermistor_SetT0(const Thermistor_T * p_therm, thermal_t value) { p_therm->P_COEFFS->T0 = (uint16_t)value; } /* Degrees Kelvin */
static inline void _Thermistor_SetT0_Celsius(const Thermistor_T * p_therm, thermal_t value) { p_therm->P_COEFFS->T0 = (uint16_t)(value - ABSOLUTE_ZERO_CELSIUS); }
static inline void _Thermistor_SetB(const Thermistor_T * p_therm, uint16_t value) { p_therm->P_COEFFS->B = value; }
static inline void _Thermistor_SetAlpha(const Thermistor_T * p_therm, float value) { p_therm->P_COEFFS->Alpha = value; }
static inline void _Thermistor_SetBeta(const Thermistor_T * p_therm, float value) { p_therm->P_COEFFS->Beta = value; }
static inline void _Thermistor_SetLinearDeltaR(const Thermistor_T * p_therm, int32_t value) { p_therm->P_COEFFS->DeltaR = value; }
static inline void _Thermistor_SetLinearDeltaT(const Thermistor_T * p_therm, int16_t value) { p_therm->P_COEFFS->DeltaT = value; }
static inline void _Thermistor_SetFit(const Thermistor_T * p_therm, Thermistor_Fit_T value) { p_therm->P_COEFFS->Fit = value; }



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
    /* Board Ref */
    THERMISTOR_BOARD_R_SERIES, // All instance Read-Only
    THERMISTOR_BOARD_R_PARALLEL, // All instance Read-Only
    THERMISTOR_BOARD_V_SERIES_MV, // All instance Read-Only
    /* Config Coeff */
    THERMISTOR_CONFIG_FIT,
    THERMISTOR_CONFIG_B,
    THERMISTOR_CONFIG_ALPHA, /* float bits (IEEE-754) in the int32 slot */
    THERMISTOR_CONFIG_BETA,  /* float bits (IEEE-754) in the int32 slot */
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