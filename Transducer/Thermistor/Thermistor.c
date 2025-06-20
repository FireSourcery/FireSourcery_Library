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
    @file   Thermistor.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Thermistor.h"

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
static inline uint32_t r_pulldown_of_adcu(uint32_t rPullUp, uint16_t vIn, uint16_t adcVRef, uint16_t adcMax, uint16_t adcu)
{
    return ((uint64_t)rPullUp * adcVRef * adcu) / (vIn * adcMax - adcVRef * adcu);
}

/* Thermistor as pull-up */
static inline uint32_t r_pullup_of_adcu(uint32_t rPullDown, uint16_t vIn, uint16_t adcVRef, uint16_t adcMax, uint16_t adcu)
{
    return ((uint64_t)rPullDown * vIn * adcMax) / (adcVRef * adcu) - rPullDown;
}

/* Convert Resistance [Ohm] to ADCU */
static inline uint16_t adcu_of_r(uint16_t adcMax, uint16_t adcVRef, uint16_t vIn, uint32_t rPullUp, uint32_t rPullDown)
{
    return ((uint64_t)vIn * adcMax * rPullDown) / ((uint64_t)adcVRef * (rPullUp + rPullDown));
}

/* 1 / (1/r1 + 1/r2) */
static inline uint32_t r_net(uint32_t rParallel1, uint32_t rParallel2)
{
    return ((uint64_t)rParallel1 * rParallel2) / (rParallel1 + rParallel2);
}

/* 1 / (1/rNet - 1/rParallel) */
static inline uint32_t r_parallel(uint32_t rNet, uint32_t rParallel)
{
    return ((uint64_t)rNet * rParallel) / (rNet - rParallel);
}

/*!
    1/T = 1/T0 + (1/B)*ln(R/R0)
    @return 1/T
*/
static inline double steinhart(uint32_t b, uint32_t t0, uint32_t r0, uint32_t r_thermal)
{
    return (log((double)r_thermal / r0) / b) + (1.0F / t0);
}

/*!
    @return R_Thermistor
*/
static inline double inv_steinhart(uint32_t b, uint32_t t0, uint32_t r0, double invT_Kelvin)
{
    return exp((invT_Kelvin - 1.0F / t0) * b) * r0;
}

/******************************************************************************/
/*
    Direct conversion using steinhart coefficients.
*/
/******************************************************************************/
#if defined(CONFIG_THERMISTOR_UNITS_FLOAT)
float Thermistor_KelvinOfR_Steinhart(const Thermistor_T * p_therm, uint32_t r_thermal)
{
    return (float)((double)1.0F / steinhart(Thermistor_GetB(p_therm), Thermistor_GetT0(p_therm), Thermistor_GetR0(p_therm), r_thermal));
}

uint32_t Thermistor_ROfKelvin_Steinhart(const Thermistor_T * p_therm, float degK)
{
    return (uint32_t)inv_steinhart(Thermistor_GetB(p_therm), Thermistor_GetT0(p_therm), Thermistor_GetR0(p_therm), (double)1.0F / (degK));
}

static float KelvinOfAdcu_Steinhart(const Thermistor_T * p_therm, uint16_t adcu) { return Thermistor_KelvinOfR_Steinhart(p_therm, Thermistor_ROhmOfAdcu(p_therm, adcu)); }
static uint16_t AdcuOfKelvin_Steinhart(const Thermistor_T * p_therm, float degK) { return Thermistor_AdcuOfROhm(p_therm, Thermistor_ROfKelvin_Steinhart(p_therm, degK)); }

static float CelsiusOfAdcu_Steinhart(const Thermistor_T * p_therm, uint16_t adcu) { return (KelvinOfAdcu_Steinhart(p_therm, adcu) - 273.15F); }
static uint16_t AdcuOfCelsius_Steinhart(const Thermistor_T * p_therm, float degC) { return AdcuOfKelvin_Steinhart(p_therm, degC + 273.15F); }
#endif

// static int16_t CelsiusOfAdcu_Linear(const Thermistor_T * p_therm, uint16_t adcu) { return Linear_Of(&p_therm->LinearUnits, adcu); }
// static uint16_t AdcuOfCelsius_Linear(const Thermistor_T * p_therm, int16_t degC) { return Linear_InvOf(&p_therm->LinearUnits, degC); }

// static thermal_t CelsiusOfAdcu(const Thermistor_T * p_therm, uint16_t adcu)
// {
//     thermal_t degC = 0;
// #if     defined(CONFIG_THERMISTOR_UNITS_LINEAR)
//     // degC = CelsiusOfAdcu_Linear(p_therm, adcu);
// #elif   defined(CONFIG_THERMISTOR_UNITS_FLOAT)
//     degC = CelsiusOfAdcu_Steinhart(p_therm, adcu);
// #elif   defined(CONFIG_THERMISTOR_UNITS_LUT)
//     degC = CelsiusOfAdcu_Lut(p_therm, adcu);
// #endif
//     return degC;
// }

// static uint16_t AdcuOfCelsius(const Thermistor_T * p_therm, thermal_t degC)
// {
//     uint16_t adcu = 0;
// #if     defined(CONFIG_THERMISTOR_UNITS_LINEAR)
//     // adcu = AdcuOfCelsius_Linear(p_therm, degC);
// #elif   defined(CONFIG_THERMISTOR_UNITS_FLOAT)
//     adcu = AdcuOfCelsius_Steinhart(p_therm, degC);
// #elif   defined(CONFIG_THERMISTOR_UNITS_LUT)
//     adcu = AdcuOfCelsius_Lut(p_therm, degC);
// #endif
//     return adcu;
// }

// thermal_t Thermistor_CelsiusOfAdcu(const Thermistor_T * p_therm, uint16_t adcu) { return CelsiusOfAdcu(p_therm, adcu); }
// uint16_t Thermistor_AdcuOfCelsius(const Thermistor_T * p_therm, thermal_t degC) { return AdcuOfCelsius(p_therm, degC); }

/* Direct calculation without linear precomputed */
uint32_t Thermistor_ROhmOfAdcu(const Thermistor_T * p_therm, uint16_t adcu)
{
    uint32_t rNet = r_pulldown_of_adcu(p_therm->R_SERIES, Thermistor_GetVInRef_MilliV(p_therm), ANALOG_REFERENCE.ADC_VREF_MILLIV, ANALOG_REFERENCE.ADC_MAX, adcu);
    return (p_therm->R_PARALLEL != 0U) ? r_parallel(rNet, p_therm->R_PARALLEL) : rNet;
}

uint16_t Thermistor_AdcuOfROhm(const Thermistor_T * p_therm, uint32_t rThermistor)
{
    uint32_t rNet = (p_therm->R_PARALLEL != 0U) ? r_net(p_therm->R_PARALLEL, rThermistor) : rThermistor;
    return adcu_of_r(ANALOG_REFERENCE.ADC_MAX, ANALOG_REFERENCE.ADC_VREF_MILLIV, Thermistor_GetVInRef_MilliV(p_therm), p_therm->R_SERIES, rNet);
}

/******************************************************************************/
/*

*/
/******************************************************************************/
void Thermistor_InitFrom(const Thermistor_T * p_therm, const Thermistor_Coeffs_T * p_config)
{
    if ((p_therm->P_FIXED_COEFFS == NULL) && (p_therm->P_COEFFS != NULL))
    {
        if (p_config != NULL) { *p_therm->P_COEFFS = *p_config; }
    }
}

void Thermistor_Init(const Thermistor_T * p_therm)
{
    Thermistor_InitFrom(p_therm, p_therm->P_NVM_COEFFS);
}

/******************************************************************************/
/*
    Precomputed shift divide
*/
/******************************************************************************/
/*
    Linear T0 may select a different T0 than the Coefficients T0.
*/
void Thermistor_ToLinear_ROhmsPerAdcu(const Thermistor_T * p_therm, Linear_T * p_result)
{
    uint32_t adcu0 = Thermistor_AdcuOfROhm(p_therm, Thermistor_GetR0(p_therm));
    uint32_t adcu1 = Thermistor_AdcuOfROhm(p_therm, Thermistor_GetR0(p_therm) + Thermistor_GetLinearDeltaR(p_therm));
    Linear_Map_Init(p_result, adcu0, adcu1, Thermistor_GetR0(p_therm), Thermistor_GetR0(p_therm) + Thermistor_GetLinearDeltaR(p_therm));
}

void Thermistor_ToLinear_CelsiusPerAdcu(const Thermistor_T * p_therm, Linear_T * p_result)
{
    uint32_t adcu0 = Thermistor_AdcuOfROhm(p_therm, Thermistor_GetR0(p_therm));
    uint32_t adcu1 = Thermistor_AdcuOfROhm(p_therm, Thermistor_GetR0(p_therm) + Thermistor_GetLinearDeltaR(p_therm));
    Linear_Map_Init(p_result, adcu0, adcu1, Thermistor_GetT0_Celsius(p_therm), Thermistor_GetT0_Celsius(p_therm) + Thermistor_GetLinearDeltaT(p_therm));
}

void Thermistor_ToLinear_CelsiusPerROhms(const Thermistor_T * p_therm, Linear_T * p_result)
{
    uint32_t r0 = Thermistor_GetR0(p_therm);
    uint32_t r1 = r0 + Thermistor_GetLinearDeltaR(p_therm);
    uint32_t t0 = Thermistor_GetT0_Celsius(p_therm);
    uint32_t t1 = t0 + Thermistor_GetLinearDeltaT(p_therm);
    Linear_Map_Init(p_result, r0, r1, t0, t1);
}



/******************************************************************************/
/*

*/
/******************************************************************************/
int32_t _Thermistor_ConfigId_Get(const Thermistor_T * p_therm, Thermistor_ConfigId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case THERMISTOR_CONFIG_R_SERIES:        value = p_therm->R_SERIES / 10U;                    break;
        case THERMISTOR_CONFIG_R_PARALLEL:      value = p_therm->R_PARALLEL / 10U;                  break;
        case THERMISTOR_CONFIG_V_SERIES_MV:     value = p_therm->V_SERIES_MV;                       break;
        // case THERMISTOR_CONFIG_TYPE:            value = Thermistor_GetType(p_therm);                break;
        case THERMISTOR_CONFIG_B:               value = Thermistor_GetB(p_therm);                   break;
        case THERMISTOR_CONFIG_R0:              value = Thermistor_GetR0(p_therm) / 10U;            break;
        case THERMISTOR_CONFIG_T0:              value = Thermistor_GetT0(p_therm);                  break;
        case THERMISTOR_CONFIG_LINEAR_DELTA_R:  value = Thermistor_GetLinearDeltaR(p_therm) ;       break;
        case THERMISTOR_CONFIG_LINEAR_DELTA_T:  value = Thermistor_GetLinearDeltaT(p_therm);        break;
        default: break;
    }
    return value;
}

void _Thermistor_ConfigId_Set(const Thermistor_T * p_therm, Thermistor_ConfigId_T id, int32_t value)
{
    if (p_therm->P_COEFFS != NULL)
    {
        switch (id)
        {
            case THERMISTOR_CONFIG_R_SERIES:        break;
            case THERMISTOR_CONFIG_R_PARALLEL:      break;
            case THERMISTOR_CONFIG_V_SERIES_MV:     break;
            // case THERMISTOR_CONFIG_TYPE:            _Thermistor_SetType(p_therm, value);                break;
            case THERMISTOR_CONFIG_B:               _Thermistor_SetB(p_therm, value);                   break;
            case THERMISTOR_CONFIG_R0:              _Thermistor_SetR0(p_therm, value * 10U);            break;
            case THERMISTOR_CONFIG_T0:              _Thermistor_SetT0(p_therm, value);                  break;
            case THERMISTOR_CONFIG_LINEAR_DELTA_R:  _Thermistor_SetLinearDeltaR(p_therm, value);        break;
            case THERMISTOR_CONFIG_LINEAR_DELTA_T:  _Thermistor_SetLinearDeltaT(p_therm, value);        break;
            default: break;
        }
    }
}

int Thermistor_ConfigId_Get(const Thermistor_T * p_therm, Thermistor_ConfigId_T id) { return (p_therm != NULL) ? _Thermistor_ConfigId_Get(p_therm, id) : 0; }

void Thermistor_ConfigId_Set(const Thermistor_T * p_therm, Thermistor_ConfigId_T id, int value) { if (p_therm != NULL) { _Thermistor_ConfigId_Set(p_therm, id, value); } }