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
#include "Config.h"
#include "Peripheral/Analog/AnalogReference.h"
#include "Math/Linear/Linear_ADC.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#if     defined(CONFIG_THERMISTOR_UNITS_LINEAR) || defined(CONFIG_THERMISTOR_UNITS_LUT)
    typedef int16_t thermal_t;
    static const thermal_t ABSOLUTE_ZERO_CELSIUS = -273;
    // static const thermal_t KELVIN_OFFSET = 273;
#elif   defined(CONFIG_THERMISTOR_UNITS_FLOAT)
    typedef float thermal_t;
    static const thermal_t ABSOLUTE_ZERO_CELSIUS = -273.15F;
#endif

/*   */
typedef enum Thermistor_Type
{
    THERMISTOR_TYPE_NTC,
    THERMISTOR_TYPE_PTC,
    THERMISTOR_TYPE_LINEAR,
    THERMISTOR_TYPE_LUT,
}
Thermistor_Type_T;

// in configurable Config, or FIXED
typedef struct Thermistor_Coeffs
{
    Thermistor_Type_T Type;
    uint16_t B;
    uint32_t R0;
    uint16_t T0; /* In Kelvin. */
    // uint16_t VInRef_MilliV;

    /* Back Up Linear Unit Conversion. */
    uint32_t DeltaR;
    uint16_t DeltaT;
    // uint32_t LinearR0;
    // uint16_t LinearT0;

    // T_Adcu/T;
    // uint16_t LinearT0_Adcu;
    // uint16_t DeltaT_Adcu;
    // V/T;
    // uint16_t LinearV0;
    // uint32_t DeltaV;
}
Thermistor_Coeffs_T;

#define THERMISTOR_COEFF_INIT(TypeValue, BValue, R0Value, T0_Kelvin) { .Type = TypeValue, .B = BValue, .R0 = R0Value, .T0 = T0_Kelvin, }
#define THERMISTOR_COEFF_ALLOC(TypeValue, BValue, R0Value, T0_Kelvin) (&(Thermistor_Coeffs_T)THERMISTOR_COEFF_INIT(TypeValue, BValue, R0Value, T0_Kelvin))

/*

*/
typedef const struct Thermistor
{
    /* Board */
    uint32_t R_SERIES;    /* Pull-up */
    uint32_t R_PARALLEL;  /* Parallel pull-down if applicable. 0 for Disable */
    uint16_t V_SERIES_MV; /* If VSeries is different than AdcVRef */

    /* Non detachable, disable Coefficient set functions */
    const Thermistor_Coeffs_T * P_FIXED_COEFFS; /* Set as Read Only */
    Thermistor_Coeffs_T * P_COEFFS; /* NULL for Fixed */

    const Thermistor_Coeffs_T * P_NVM_COEFFS; /*  */

    /* Service */
    // Linear_T * P_LINEAR_R_OHMS;  /* R per Adcu */
    // Linear_T * P_LINEAR_T_CELCIUS;
}
Thermistor_T;

#define _THERMISTOR_BOARD_INIT(RSeries, RParallel, VSeries) { .R_SERIES = RSeries, .R_PARALLEL = RParallel, .V_SERIES_MV = VSeries, }

#define THERMISTOR_INIT(RSeries, RParallel, VSeries, p_Fixed, p_Coeffs) \
    { .R_SERIES = RSeries, .R_PARALLEL = RParallel, .V_SERIES_MV = VSeries, .P_FIXED_COEFFS = p_Fixed, .P_COEFFS = p_Coeffs, }

    // static_assert(p_Fixed == NULL || p_Coeffs == NULL, "Thermistor must have either fixed or configurable coefficients, not both.");

/* Init as Configurable/Detachable */
#define THERMISTOR_WIRED_INIT(RSeries, RParallel, VSeries, p_Coeffs, p_NvmCoeffs) \
    { .R_SERIES = RSeries, .R_PARALLEL = RParallel, .V_SERIES_MV = VSeries, .P_FIXED_COEFFS = NULL, .P_COEFFS = p_Coeffs, .P_NVM_COEFFS = p_NvmCoeffs }

/* Init as Fixed to Board */
#define THERMISTOR_FIXED_INIT(RSeries, RParallel, VSeries, TypeValue, BValue, R0Value, T0_Kelvin) \
    THERMISTOR_INIT(RSeries, RParallel, VSeries, THERMISTOR_COEFF_ALLOC(TypeValue, BValue, R0Value, T0_Kelvin), NULL)


/******************************************************************************/
/*
*/
/******************************************************************************/
typedef enum Thermistor_ConfigId
{
    THERMISTOR_CONFIG_R_SERIES, // All instance Read-Only
    THERMISTOR_CONFIG_R_PARALLEL, // All instance Read-Only
    THERMISTOR_CONFIG_V_SERIES_MV, // All instance Read-Only
    THERMISTOR_CONFIG_TYPE,
    THERMISTOR_CONFIG_B,
    THERMISTOR_CONFIG_R0,
    THERMISTOR_CONFIG_T0,
    THERMISTOR_CONFIG_LINEAR_DELTA_R,
    THERMISTOR_CONFIG_LINEAR_DELTA_T,
}
Thermistor_ConfigId_T;

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
static inline Thermistor_Type_T _Thermistor_GetType(const Thermistor_T * p_therm)   { return p_therm->P_COEFFS->Type; }
static inline uint32_t _Thermistor_GetR0(const Thermistor_T * p_therm)              { return p_therm->P_COEFFS->R0; }
static inline uint16_t _Thermistor_GetT0(const Thermistor_T * p_therm)              { return p_therm->P_COEFFS->T0; } /* Degrees Kelvin */
static inline uint16_t _Thermistor_GetT0_DegC(const Thermistor_T * p_therm)         { return p_therm->P_COEFFS->T0 + ABSOLUTE_ZERO_CELSIUS; }
static inline uint16_t _Thermistor_GetB(const Thermistor_T * p_therm)               { return p_therm->P_COEFFS->B; }

static inline void _Thermistor_SetType(const Thermistor_T * p_therm, uint16_t value)      { p_therm->P_COEFFS->Type = value; }
static inline void _Thermistor_SetR0(const Thermistor_T * p_therm, uint32_t value)        { p_therm->P_COEFFS->R0 = value; }
static inline void _Thermistor_SetT0(const Thermistor_T * p_therm, uint16_t value)        { p_therm->P_COEFFS->T0 = value; } /* Degrees Kelvin */
static inline void _Thermistor_SetT0_DegC(const Thermistor_T * p_therm, uint16_t value)   { p_therm->P_COEFFS->T0 = value - ABSOLUTE_ZERO_CELSIUS; }
static inline void _Thermistor_SetB(const Thermistor_T * p_therm, uint16_t value)         { p_therm->P_COEFFS->B = value; }
static inline void _Thermistor_SetLinearDeltaR(const Thermistor_T * p_therm, uint32_t value) { p_therm->P_COEFFS->DeltaR = value; }
static inline void _Thermistor_SetLinearDeltaT(const Thermistor_T * p_therm, uint16_t value) { p_therm->P_COEFFS->DeltaT = value; }

static inline const Thermistor_Coeffs_T * Thermistor_GetCoeffs(const Thermistor_T * p_therm) { return (p_therm->P_FIXED_COEFFS != NULL) ? p_therm->P_FIXED_COEFFS : p_therm->P_COEFFS; }

static inline Thermistor_Type_T Thermistor_GetType(const Thermistor_T * p_therm)    { return Thermistor_GetCoeffs(p_therm)->Type; }
static inline uint16_t Thermistor_GetB(const Thermistor_T * p_therm)                { return Thermistor_GetCoeffs(p_therm)->B; }
static inline uint32_t Thermistor_GetR0(const Thermistor_T * p_therm)               { return Thermistor_GetCoeffs(p_therm)->R0; }
static inline uint16_t Thermistor_GetT0(const Thermistor_T * p_therm)               { return Thermistor_GetCoeffs(p_therm)->T0; }
static inline uint16_t Thermistor_GetT0_Kelvin(const Thermistor_T * p_therm)        { return Thermistor_GetCoeffs(p_therm)->T0; }
static inline uint16_t Thermistor_GetT0_Celsius(const Thermistor_T * p_therm)       { return (Thermistor_GetT0(p_therm) + ABSOLUTE_ZERO_CELSIUS); }
static inline uint16_t Thermistor_GetLinearDeltaR(const Thermistor_T * p_therm)     { return Thermistor_GetCoeffs(p_therm)->DeltaR; }
static inline uint16_t Thermistor_GetLinearDeltaT(const Thermistor_T * p_therm)     { return Thermistor_GetCoeffs(p_therm)->DeltaT; }

static inline uint32_t Thermistor_GetRSeries(const Thermistor_T * p_therm)          { return p_therm->R_SERIES; }
static inline uint32_t Thermistor_GetRParallel(const Thermistor_T * p_therm)        { return p_therm->R_PARALLEL; } /* 0 for Disable */
static inline uint16_t _Thermistor_GetVInRef_MilliV(const Thermistor_T * p_therm)   { return p_therm->V_SERIES_MV; } /* If VRef is different than ADC */
static inline uint16_t Thermistor_GetVInRef_MilliV(const Thermistor_T * p_therm)    { return (p_therm->V_SERIES_MV == 0U) ? ANALOG_REFERENCE.ADC_VREF_MILLIV : p_therm->V_SERIES_MV; } /* If VRef is different than ADC */

// static inline bool Thermistor_IsPulldown(const Thermistor_T * p_therm)
static inline uint16_t Thermistor_GetVAdcRef_MilliV(void) { return ANALOG_REFERENCE.ADC_VREF_MILLIV; }
static inline uint16_t Thermistor_GetVAdcMax(void)        { return ANALOG_REFERENCE.ADC_MAX; }


/******************************************************************************/
/*
*/
/******************************************************************************/
extern void Thermistor_InitFrom(const Thermistor_T * p_therm, const Thermistor_Coeffs_T * p_config);

extern uint32_t Thermistor_ROhmOfAdcu(const Thermistor_T * p_therm, uint16_t adcu);
extern uint16_t Thermistor_AdcuOfROhm(const Thermistor_T * p_therm, uint32_t rThermistor);

extern void Thermistor_ToLinear_ROhmsPerAdcu(const Thermistor_T * p_therm, Linear_T * p_result);
extern void Thermistor_ToLinear_CelsiusPerAdcu(const Thermistor_T * p_therm, Linear_T * p_result);
extern void Thermistor_ToLinear_CelsiusPerROhms(const Thermistor_T * p_therm, Linear_T * p_result);

extern thermal_t Thermistor_CelsiusOfAdcu(const Thermistor_T * p_therm, uint16_t adcu);
extern uint16_t Thermistor_AdcuOfCelsius(const Thermistor_T * p_therm, thermal_t degC);

/******************************************************************************/
/*
*/
/******************************************************************************/
extern int32_t _Thermistor_ConfigId_Get(const Thermistor_T * p_therm, Thermistor_ConfigId_T id);
extern void _Thermistor_ConfigId_Set(const Thermistor_T * p_therm, Thermistor_ConfigId_T id, int32_t value);

extern int Thermistor_ConfigId_Get(const Thermistor_T * p_therm, int id);
extern void Thermistor_ConfigId_Set(const Thermistor_T * p_therm, int id, int value);