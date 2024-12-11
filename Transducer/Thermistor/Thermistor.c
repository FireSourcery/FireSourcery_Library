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
    @file   Thermistor.c
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
    Linear_ADC_Init(&p_therm->HeatLimit, p_therm->Config.FaultTrigger_Adcu, p_therm->Config.WarningTrigger_Adcu); /* for scalar16 only */

    // Linear_Init_Map(&p_therm->LinearUnits, p_therm->Config.LinearT0_Adcu, p_therm->Config.LinearT1_Adcu, p_therm->Config.LinearT0_DegC, p_therm->Config.LinearT1_DegC);
}

void Thermistor_Init(Thermistor_T * p_therm)
{
    if (p_therm->CONST.P_CONFIG != NULL) { memcpy(&p_therm->Config, p_therm->CONST.P_CONFIG, sizeof(Thermistor_Config_T)); }

    ResetUnitsLinear(p_therm);

    // type = LinearT_DegC/LinearT_Adcu < 0

    if (p_therm->Config.FaultTrigger_Adcu == 0U)         { p_therm->Config.IsMonitorEnable = false; }
    if (p_therm->Config.FaultThreshold_Adcu == 0U)       { p_therm->Config.IsMonitorEnable = false; }
    if (p_therm->Config.WarningTrigger_Adcu == 0U)       { p_therm->Config.IsMonitorEnable = false; }
    if (p_therm->Config.WarningThreshold_Adcu == 0U)     { p_therm->Config.IsMonitorEnable = false; }

    p_therm->Adcu = p_therm->Config.WarningThreshold_Adcu; /* Set to a nominal value, so poll on init does not fault. */
    p_therm->Status = THERMISTOR_STATUS_OK;
}

/******************************************************************************/
/*!
    Limits Monitor
*/
/******************************************************************************/
bool CheckThreshold(uint16_t threshold, bool isActive, uint16_t adcu) { return ((adcu < threshold) && (isActive)); }
// bool Check(uint16_t trigger, uint16_t threshold, bool isActive, uint16_t adcu) { return((adcu < threshold) && (isActive)); }

/*!
    Monitor: heat < WarningThreshold_DegC < Warning_DegC < FaultThreshold_DegC < Fault_DegC
    Lower adcu is higher heat, ntc, as pulldown
    @return
*/
Thermistor_Status_T Thermistor_PollMonitor(Thermistor_T * p_therm, uint16_t captureAdcu)
{
    Thermistor_Status_T status;
    uint16_t adcu = (captureAdcu + p_therm->Adcu) / 2U;

    // todo linear ntc/ptc
    // int16_t adcuCompare = (Type == PTC) ? adcu : -adcu;

    if(p_therm->Config.IsMonitorEnable == true)
    {
        if      (adcu > p_therm->Config.WarningThreshold_Adcu)                                                           { p_therm->Status = THERMISTOR_STATUS_OK; }
        else if (adcu < p_therm->Config.FaultTrigger_Adcu)                                                               { p_therm->Status = THERMISTOR_STATUS_FAULT; }
        else if (CheckThreshold(p_therm->Config.FaultThreshold_Adcu, Thermistor_IsFault(p_therm), adcu) == true)         { p_therm->Status = THERMISTOR_STATUS_FAULT_THRESHOLD; }
        else if (adcu < p_therm->Config.WarningTrigger_Adcu)                                                             { p_therm->Status = THERMISTOR_STATUS_WARNING; }
        else if (CheckThreshold(p_therm->Config.WarningThreshold_Adcu, Thermistor_IsWarning(p_therm), adcu) == true)     { p_therm->Status = THERMISTOR_STATUS_WARNING_THRESHOLD; }
        else                                                                                                             { p_therm->Status = THERMISTOR_STATUS_OK; }

        p_therm->Adcu = adcu;
    }

    return p_therm->Status;
}

// with trigger
// Thermistor_PollMonitor_Edges(Thermistor_T * p_therm, uint16_t captureAdcu)
// {
//     switch (p_therm->Status)
//     {
//         case THERMISTOR_STATUS_OK:
//             if (captureAdcu < p_therm->Config.WarningThreshold_Adcu) { p_therm->Status = THERMISTOR_STATUS_WARNING; }
//             if (captureAdcu < p_therm->Config.FaultThreshold_Adcu)   { p_therm->Status = THERMISTOR_STATUS_FAULT; }
//             break;
//     }
// }

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

static inline uint32_t r_net(uint32_t rParallel1, uint32_t rParallel2)
{
    return ((uint64_t)rParallel1 * rParallel2) / (rParallel1 + rParallel2);     /* 1 / (1/r1 + 1/r2) */
}

static inline uint32_t r_parallel(uint32_t rNet, uint32_t rParallel)
{
    return ((uint64_t)rNet * rParallel) / (rNet - rParallel);                 /* 1 / (1/rNet - 1/rParallel) */
}

/*!
    1/T = 1/T0 + (1/B)*ln(R/R0)
    @return 1/T
*/
static inline double steinhart(double b, double t0, double r0, double rTh)
{
    return (log(rTh / r0) / b) + (1.0F / t0);
}

/*!
    @return R_Thermistor
*/
static inline double invsteinhart(double b, double t0, double r0, double invT_Kelvin)
{
    return exp((invT_Kelvin - 1.0F / t0) * b) * r0;
}

#if defined(CONFIG_THERMISTOR_UNITS_FLOAT)
static float ConvertAdcuToDegK_Steinhart(const Thermistor_T * p_therm, uint16_t adcu)
{
    uint32_t rNet = r_pulldown_of_adcu(p_therm->CONST.R_SERIES, p_therm->Config.VInRef_MilliV, GLOBAL_ANALOG.ADC_VREF_MILLIV, GLOBAL_ANALOG.ADC_MAX, adcu);
    uint32_t rTh = (p_therm->CONST.R_PARALLEL != 0U) ? r_parallel(rNet, p_therm->CONST.R_PARALLEL) : rNet;
    double invT_Kelvin = steinhart(p_therm->Config.B, p_therm->Config.T0, p_therm->Config.R0, rTh);
    return (1.0F / invT_Kelvin);
}

static uint16_t ConvertDegKToAdcu_Steinhart(const Thermistor_T * p_therm, float degK)
{
    double invT_Kelvin = (double)1.0F / (degK);
    uint32_t rTh = invsteinhart(p_therm->Config.B, p_therm->Config.T0, p_therm->Config.R0, invT_Kelvin);
    uint32_t rNet = (p_therm->CONST.R_PARALLEL != 0U) ? r_net(p_therm->CONST.R_PARALLEL, rTh) : rTh;
    return adcu_of_r(GLOBAL_ANALOG.ADC_MAX, GLOBAL_ANALOG.ADC_VREF_MILLIV, p_therm->Config.VInRef_MilliV, p_therm->CONST.R_SERIES, rNet);
}
static float ConvertAdcuToDegC_Steinhart(const Thermistor_T * p_therm, uint16_t adcu)
{
    return (ConvertAdcuToDegK_Steinhart(p_therm, adcu) - 273.15F);
}

static uint16_t ConvertDegCToAdcu_Steinhart(const Thermistor_T * p_therm, float degC)
{
    return ConvertDegKToAdcu_Steinhart(p_therm, degC + 273.15F);
}
#endif

static int16_t ConvertAdcuToDegC_Linear(const Thermistor_T * p_therm, uint16_t adcu)     { return Linear_Of(&p_therm->LinearUnits, adcu); }
static uint16_t ConvertDegCToAdcu_Linear(const Thermistor_T * p_therm, int16_t degC)     { return Linear_InvOf(&p_therm->LinearUnits, degC); }

static thermal_t ConvertAdcuToDegC(const Thermistor_T * p_therm, uint16_t adcu)
{
    thermal_t degC;
#if     defined(CONFIG_THERMISTOR_UNITS_LINEAR)
    degC = ConvertAdcuToDegC_Linear(p_therm, adcu);
#elif   defined(CONFIG_THERMISTOR_UNITS_FLOAT)
    degC = ConvertAdcuToDegC_Steinhart(p_therm, adcu);
#elif   defined(CONFIG_THERMISTOR_UNITS_LUT)
    degC = ConvertAdcuToDegC_Lut(p_therm, adcu);
#endif
    return degC;
}

static uint16_t ConvertDegCToAdcu(const Thermistor_T * p_therm, thermal_t degC)
{
    uint16_t adcu;
#if     defined(CONFIG_THERMISTOR_UNITS_LINEAR)
    adcu = ConvertDegCToAdcu_Linear(p_therm, degC);
#elif   defined(CONFIG_THERMISTOR_UNITS_FLOAT)
    adcu = ConvertDegCToAdcu_Steinhart(p_therm, degC);
#elif   defined(CONFIG_THERMISTOR_UNITS_LUT)
    adcu = ConvertDegCToAdcu_Lut(p_therm, degC);
#endif
    return adcu;
}

thermal_t Thermistor_ConvertToDegC(const Thermistor_T * p_therm, uint16_t adcu)                         { return ConvertAdcuToDegC(p_therm, adcu); }
uint16_t Thermistor_ConvertToAdcu_DegC(const Thermistor_T * p_therm, thermal_t degC)                    { return ConvertDegCToAdcu(p_therm, degC); }


/******************************************************************************/
/*!
    Set Limits Config
*/
/******************************************************************************/
void Thermistor_SetFault_DegC(Thermistor_T * p_therm, thermal_t fault_degC, thermal_t faultThreshold_degC)
{
    p_therm->Config.FaultTrigger_Adcu = Thermistor_ConvertToAdcu_DegC(p_therm, fault_degC);
    p_therm->Config.FaultThreshold_Adcu = Thermistor_ConvertToAdcu_DegC(p_therm, faultThreshold_degC);
}

void Thermistor_SetWarning_DegC(Thermistor_T * p_therm, thermal_t warning_degC, thermal_t warningThreshold_degC)
{
    p_therm->Config.WarningTrigger_Adcu = Thermistor_ConvertToAdcu_DegC(p_therm, warning_degC);
    p_therm->Config.WarningThreshold_Adcu = Thermistor_ConvertToAdcu_DegC(p_therm, warningThreshold_degC);
}

void Thermistor_SetLimits_DegC(Thermistor_T * p_therm, thermal_t fault, thermal_t faultThreshold, thermal_t warning, thermal_t warningThreshold)
{
    Thermistor_SetFault_DegC(p_therm, fault, faultThreshold);
    Thermistor_SetWarning_DegC(p_therm, warning, warningThreshold);
    ResetUnitsLinear(p_therm);
}

thermal_t Thermistor_GetFault_DegC(const Thermistor_T * p_therm)              { return ConvertAdcuToDegC(p_therm, p_therm->Config.FaultTrigger_Adcu); }
thermal_t Thermistor_GetFaultThreshold_DegC(const Thermistor_T * p_therm)     { return ConvertAdcuToDegC(p_therm, p_therm->Config.FaultThreshold_Adcu); }
thermal_t Thermistor_GetWarning_DegC(const Thermistor_T * p_therm)            { return ConvertAdcuToDegC(p_therm, p_therm->Config.WarningTrigger_Adcu); }
thermal_t Thermistor_GetWarningThreshold_DegC(const Thermistor_T * p_therm)   { return ConvertAdcuToDegC(p_therm, p_therm->Config.WarningThreshold_Adcu); }

