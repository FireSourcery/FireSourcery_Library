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
    Linear_ADC_Init(&p_therm->HeatLimit, p_therm->Config.FaultTrigger_Adcu, p_therm->Config.WarningTrigger_Adcu); /* for Percent16 only */

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


// bool isGreaterHeat(Thermistor_Type_T type, uint16_t adcu1, uint16_t adcu2) { return (type == THERMISTOR_TYPE_NTC) ? (adcu1 > adcu2) : (adcu1 < adcu2); }
// bool isLesserHeat(Thermistor_Type_T type, uint16_t adcu1, uint16_t adcu2) { return (type == THERMISTOR_TYPE_NTC) ? (adcu1 < adcu2) : (adcu1 > adcu2); }

/******************************************************************************/
/*!
    Limits Monitor
*/
/******************************************************************************/

/*!
    Monitor: heat < WarningThreshold_DegC < Warning_DegC < FaultThreshold_DegC < Fault_DegC
    As pulldown
     NTC Lower adcu is higher heat
     PTC Lower adcu is lower heat
    @return
*/
Thermistor_Status_T Thermistor_PollMonitor(Thermistor_T * p_therm, uint16_t captureAdcu)
{
    Thermistor_Status_T status;
    uint16_t adcu;

    adcu = (captureAdcu + p_therm->Adcu) / 2U;

    if(p_therm->Config.IsMonitorEnable == true)
    {
        // if      (adcu > p_therm->Config.WarningThreshold_Adcu)                                      { p_therm->Status = THERMISTOR_STATUS_OK; }
        // else if (adcu < p_therm->Config.FaultTrigger_Adcu)                                          { p_therm->Status = THERMISTOR_STATUS_FAULT; }
        // else if ((adcu < p_therm->Config.FaultThreshold_Adcu) && Thermistor_IsFault(p_therm))       { p_therm->Status = THERMISTOR_STATUS_FAULT_THRESHOLD; }
        // else if (adcu < p_therm->Config.WarningTrigger_Adcu)                                        { p_therm->Status = THERMISTOR_STATUS_WARNING; }
        // else if ((adcu < p_therm->Config.WarningThreshold_Adcu) && Thermistor_IsWarning(p_therm))   { p_therm->Status = THERMISTOR_STATUS_WARNING_THRESHOLD; }
        // else                                                                                        { p_therm->Status = THERMISTOR_STATUS_OK; }

        bool isNtc = (p_therm->Config.Type == THERMISTOR_TYPE_NTC);
        int16_t adcuCompare = isNtc ? adcu : -adcu;
        int16_t warningThresholdCompare = isNtc ? p_therm->Config.WarningThreshold_Adcu : -p_therm->Config.WarningThreshold_Adcu;
        int16_t faultTriggerCompare = isNtc ? p_therm->Config.FaultTrigger_Adcu : -p_therm->Config.FaultTrigger_Adcu;
        int16_t faultThresholdCompare = isNtc ? p_therm->Config.FaultThreshold_Adcu : -p_therm->Config.FaultThreshold_Adcu;
        int16_t warningTriggerCompare = isNtc ? p_therm->Config.WarningTrigger_Adcu : -p_therm->Config.WarningTrigger_Adcu;

        if      (adcuCompare > warningThresholdCompare)                                     { p_therm->Status = THERMISTOR_STATUS_OK; }
        else if (adcuCompare < faultTriggerCompare)                                         { p_therm->Status = THERMISTOR_STATUS_FAULT; }
        else if (adcuCompare < faultThresholdCompare && Thermistor_IsFault(p_therm))        { p_therm->Status = THERMISTOR_STATUS_FAULT_THRESHOLD; }
        else if (adcuCompare < warningTriggerCompare)                                       { p_therm->Status = THERMISTOR_STATUS_WARNING; }
        else if (adcuCompare < warningThresholdCompare && Thermistor_IsWarning(p_therm))    { p_therm->Status = THERMISTOR_STATUS_WARNING_THRESHOLD; }
        else                                                                                { p_therm->Status = THERMISTOR_STATUS_OK; }
    }

    p_therm->Adcu = adcu;

    return p_therm->Status;
}


// with trigger
// Thermistor_Status_T Thermistor_PollMonitor_Edges(Thermistor_T * p_therm, uint16_t adcu)
// {
//     switch (p_therm->Status)
//     {
//         case THERMISTOR_STATUS_OK:
//             if (adcu < p_therm->Config.FaultThreshold_Adcu) { p_therm->Status = THERMISTOR_STATUS_FAULT; }
//             else if (adcu < p_therm->Config.WarningThreshold_Adcu) { p_therm->Status = THERMISTOR_STATUS_WARNING; }
//             break;
//         case THERMISTOR_STATUS_FAULT:
//             if (adcu > p_therm->Config.WarningThreshold_Adcu) { p_therm->Status = THERMISTOR_STATUS_OK; }
//             break;
//         case THERMISTOR_STATUS_FAULT_THRESHOLD:
//             if (adcu > p_therm->Config.WarningThreshold_Adcu) { p_therm->Status = THERMISTOR_STATUS_OK; }
//             break;
//         case THERMISTOR_STATUS_WARNING:
//             if (adcu > p_therm->Config.WarningThreshold_Adcu) { p_therm->Status = THERMISTOR_STATUS_OK; }
//             else if (adcu < p_therm->Config.FaultTrigger_Adcu) { p_therm->Status = THERMISTOR_STATUS_FAULT; }
//             break;
//         case THERMISTOR_STATUS_WARNING_THRESHOLD:
//             if (adcu > p_therm->Config.WarningThreshold_Adcu) { p_therm->Status = THERMISTOR_STATUS_OK; }
//             else if (adcu < p_therm->Config.FaultTrigger_Adcu) { p_therm->Status = THERMISTOR_STATUS_FAULT; }
//             break;
//         default:
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



/******************************************************************************/
/*

*/
/******************************************************************************/
// int32_t Thermistor_VarId_Get(const Thermistor_T * p_thermistor, Thermistor_VarId id)
// {
//     int32_t value = 0;
//     switch (id)
//     {

//         case THERMISTOR_CONFIG_VALUE_ADCU:     value = p_thermistor->Adcu;                         break;
//         case THERMISTOR_CONFIG_STATUS:         value = Thermistor_GetStatus(p_thermistor);         break;
//         default: value = 0; break;
//     }
//     return value;
// }

int32_t Thermistor_ConfigId_Get(const Thermistor_T * p_thermistor, Thermistor_ConfigId_T id)
{
    int32_t value = 0;
    // if (p_thermistor != NULL)
    {
        switch (id)
        {
            case THERMISTOR_CONFIG_R_SERIES:                   value = p_thermistor->CONST.R_SERIES / 10U;                 break;
            case THERMISTOR_CONFIG_R_PARALLEL:                 value = p_thermistor->CONST.R_PARALLEL / 10U;               break;
            case THERMISTOR_CONFIG_R0:                         value = Thermistor_GetR0(p_thermistor) / 10U;               break;
            case THERMISTOR_CONFIG_T0:                         value = Thermistor_GetT0(p_thermistor);                     break;
            case THERMISTOR_CONFIG_B:                          value = Thermistor_GetB(p_thermistor);                      break;
            case THERMISTOR_CONFIG_TYPE:                       value = Thermistor_GetType(p_thermistor);                   break;
            case THERMISTOR_CONFIG_FAULT_TRIGGER_ADCU:         value = Thermistor_GetFaultTrigger_Adcu(p_thermistor);      break;
            case THERMISTOR_CONFIG_FAULT_THRESHOLD_ADCU:       value = Thermistor_GetFaultThreshold_Adcu(p_thermistor);    break;
            case THERMISTOR_CONFIG_WARNING_TRIGGER_ADCU:       value = Thermistor_GetWarningTrigger_Adcu(p_thermistor);    break;
            case THERMISTOR_CONFIG_WARNING_THRESHOLD_ADCU:     value = Thermistor_GetWarningThreshold_Adcu(p_thermistor);  break;
            case THERMISTOR_CONFIG_IS_MONITOR_ENABLE:          value = Thermistor_IsMonitorEnable(p_thermistor);           break;
            default: break;
        }
    }
    return value;
}

void Thermistor_ConfigId_Set(Thermistor_T * p_thermistor, Thermistor_ConfigId_T id, int32_t value)
{
    // if (p_thermistor != NULL)
    {
        switch (id)
        {
            case THERMISTOR_CONFIG_R_SERIES:                   break;
            case THERMISTOR_CONFIG_R_PARALLEL:                 break;
            case THERMISTOR_CONFIG_R0:                         Thermistor_SetR0(p_thermistor, value);                     break;
            case THERMISTOR_CONFIG_T0:                         Thermistor_SetT0(p_thermistor, value);                     break;
            case THERMISTOR_CONFIG_B:                          Thermistor_SetB(p_thermistor, value);                      break;
            case THERMISTOR_CONFIG_TYPE:                       Thermistor_SetType(p_thermistor, value);                   break;
            case THERMISTOR_CONFIG_FAULT_TRIGGER_ADCU:         Thermistor_SetFaultTrigger_Adcu(p_thermistor, value);      break;
            case THERMISTOR_CONFIG_FAULT_THRESHOLD_ADCU:       Thermistor_SetFaultThreshold_Adcu(p_thermistor, value);    break;
            case THERMISTOR_CONFIG_WARNING_TRIGGER_ADCU:       Thermistor_SetWarningTrigger_Adcu(p_thermistor, value);    break;
            case THERMISTOR_CONFIG_WARNING_THRESHOLD_ADCU:     Thermistor_SetWarningThreshold_Adcu(p_thermistor, value);  break;
            case THERMISTOR_CONFIG_IS_MONITOR_ENABLE:          Thermistor_SetIsMonitorEnable(p_thermistor, value);        break;
        }
    }
}