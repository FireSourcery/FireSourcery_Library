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
    @file   _Monitor_Config.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Monitor.h"
#include "RangeMonitor.h"


/******************************************************************************/
/*
    Base Configuration Function
*/
/******************************************************************************/
/* Base parameter instantiation macros */
#define MONITOR_CONFIG_INIT(nominal, warnSet, warnReset, faultLimit, enabled) (Monitor_Config_T) \
{                                                                             \
    .Nominal = (nominal),                                                     \
    .Warning = { .Setpoint = (warnSet), .Resetpoint = (warnReset) },          \
    .Fault = { .Limit = (faultLimit) },                                       \
    .IsEnabled = (enabled),                                                   \
}

static inline Monitor_Config_T Monitor_Config_Create(int32_t nominal, int32_t warnSet, int32_t warnReset, int32_t faultLimit)
{
    return (Monitor_Config_T)
    {
        .Nominal = nominal,
        .Warning = { .Setpoint = warnSet, .Resetpoint = warnReset },
        .Fault = { .Limit = faultLimit }
        // .IsEnabled = enabled,
    };
    // return (Monitor_Config_T)(MONITOR_CONFIG_INIT(nominal, warnSet, warnReset, faultLimit, true));
}


/* Generic percentage-based monitoring */
static inline Monitor_Config_T Monitor_Config_OverLimit(int32_t nominal, uint8_t warnPercent, uint8_t faultPercent)
{
    int32_t warnSet = nominal + (nominal * warnPercent / 100);
    int32_t warnReset = warnSet - (nominal * 5 / 100);
    int32_t faultLimit = nominal + (nominal * faultPercent / 100);
    return Monitor_Config_Create(nominal, warnSet, warnReset, faultLimit);
}

static inline Monitor_Config_T Monitor_Config_UnderLimit(int32_t nominal, uint8_t warnPercent, uint8_t faultPercent)
{
    int32_t warnSet = nominal - (nominal * warnPercent / 100);
    int32_t warnReset = warnSet + (nominal * 5 / 100);
    int32_t faultLimit = nominal - (nominal * faultPercent / 100);
    return Monitor_Config_Create(nominal, warnSet, warnReset, faultLimit);
}

static inline Monitor_Config_T Monitor_Config_AbsoluteOverLimit(int32_t nominal, int32_t warnThreshold, int32_t faultThreshold)
{
    int32_t hysteresis = (warnThreshold - nominal) / 10;
    int32_t warnReset = warnThreshold - hysteresis;
    return Monitor_Config_Create(nominal, warnThreshold, warnReset, faultThreshold);
}

static inline Monitor_Config_T Monitor_Config_Disabled(void)
{
    return Monitor_Config_Create(0, INT32_MAX, INT32_MAX, INT32_MAX);
}



/******************************************************************************/
/*
    Range-based Configuration Functions (for RangeMonitor compatibility)
*/
/******************************************************************************/

/* Create high-side config from RangeMonitor_Config_T */
static inline Monitor_Config_T Monitor_Config_FromRangeHigh(const RangeMonitor_Config_T * p_config)
{
    bool isValid = (p_config->FaultOverLimit.Limit > p_config->Warning.LimitHigh) && (p_config->Warning.LimitHigh > p_config->Nominal);

    return (Monitor_Config_T)
    {
        .IsEnabled = isValid && p_config->IsEnabled,
        .Nominal = p_config->Nominal,
        .Warning = { .Setpoint = p_config->Warning.LimitHigh, .Resetpoint = p_config->Warning.LimitHigh - p_config->Warning.Hysteresis },
        .Fault = p_config->FaultOverLimit
    };
}

/* Create low-side config from RangeMonitor_Config_T */
static inline Monitor_Config_T Monitor_Config_FromRangeLow(const RangeMonitor_Config_T * p_config)
{
    bool isValid = (p_config->FaultUnderLimit.Limit < p_config->Warning.LimitLow) && (p_config->Warning.LimitLow < p_config->Nominal);

    return (Monitor_Config_T)
    {
        .IsEnabled = isValid && p_config->IsEnabled,
        .Nominal = p_config->Nominal,
        .Warning = { .Setpoint = p_config->Warning.LimitLow, .Resetpoint = p_config->Warning.LimitLow + p_config->Warning.Hysteresis },
        .Fault = p_config->FaultUnderLimit
    };
}

/******************************************************************************/
/*
    Generic RangeMonitor_Config_T Instantiation Functions
*/
/******************************************************************************/

/* Base configuration function with full parameter control */
static inline RangeMonitor_Config_T RangeMonitor_Config_Create(int32_t nominal, int32_t faultOverLimit, int32_t faultUnderLimit, int32_t warnLimitHigh, int32_t warnLimitLow, uint32_t hysteresis)
{
    return (RangeMonitor_Config_T)
    {
        .Nominal = nominal,
        .FaultOverLimit = { .Limit = faultOverLimit },
        .FaultUnderLimit = { .Limit = faultUnderLimit },
        .Warning =
        {
            .LimitHigh = warnLimitHigh,
            .LimitLow = warnLimitLow,
            .Hysteresis = hysteresis
        // .IsEnabled = enabled,
        }
    };
}

/* Symmetric range monitoring (equal over/under thresholds from nominal) */
static inline RangeMonitor_Config_T RangeMonitor_Config_Symmetric(int32_t nominal, int32_t faultOffset, int32_t warnOffset, uint32_t hysteresis)
{
    return RangeMonitor_Config_Create
    (
        nominal,
        nominal + faultOffset,  /* Fault over */
        nominal - faultOffset,  /* Fault under */
        nominal + warnOffset,   /* Warning high */
        nominal - warnOffset,   /* Warning low */
        hysteresis
    );
}

/* Percentage-based symmetric monitoring */
static inline RangeMonitor_Config_T RangeMonitor_Config_SymmetricPercent(int32_t nominal, uint8_t faultPercent, uint8_t warnPercent, uint8_t hystPercent)
{
    return RangeMonitor_Config_Symmetric(nominal, (nominal * faultPercent) / 100, (nominal * warnPercent) / 100, (nominal * hystPercent) / 100);
}


