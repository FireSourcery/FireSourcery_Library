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
#define MONITOR_CONFIG_INIT(nominal, warnSet, warnReset, faultLimit, enabled) \
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

/* Factory Functions (Pattern 1) - Return by Value */
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

/* In-Place Initialization Functions (Pattern 2) - Modify by Reference */
static inline void Monitor_Config_InitAsOverLimit(Monitor_Config_T * p_config, int32_t nominal, uint8_t warnPercent, uint8_t faultPercent)
{
    int32_t warnSet = nominal + (nominal * warnPercent / 100);
    int32_t warnReset = warnSet - (nominal * 5 / 100);
    int32_t faultLimit = nominal + (nominal * faultPercent / 100);

    p_config->Nominal = nominal;
    p_config->Warning.Setpoint = warnSet;
    p_config->Warning.Resetpoint = warnReset;
    p_config->Fault.Limit = faultLimit;
    p_config->IsEnabled = true;
}

static inline void Monitor_Config_InitAsUnderLimit(Monitor_Config_T * p_config, int32_t nominal, uint8_t warnPercent, uint8_t faultPercent)
{
    int32_t warnSet = nominal - (nominal * warnPercent / 100);
    int32_t warnReset = warnSet + (nominal * 5 / 100);
    int32_t faultLimit = nominal - (nominal * faultPercent / 100);

    p_config->Nominal = nominal;
    p_config->Warning.Setpoint = warnSet;
    p_config->Warning.Resetpoint = warnReset;
    p_config->Fault.Limit = faultLimit;
    p_config->IsEnabled = true;
}

static inline void Monitor_Config_InitAsAbsoluteOverLimit(Monitor_Config_T * p_config, int32_t nominal, int32_t warnThreshold, int32_t faultThreshold)
{
    int32_t hysteresis = (warnThreshold - nominal) / 10;
    int32_t warnReset = warnThreshold - hysteresis;

    p_config->Nominal = nominal;
    p_config->Warning.Setpoint = warnThreshold;
    p_config->Warning.Resetpoint = warnReset;
    p_config->Fault.Limit = faultThreshold;
    p_config->IsEnabled = true;
}

static inline void Monitor_Config_InitAsDisabled(Monitor_Config_T * p_config)
{
    p_config->Nominal = 0;
    p_config->Warning.Setpoint = INT32_MAX;
    p_config->Warning.Resetpoint = INT32_MAX;
    p_config->Fault.Limit = INT32_MAX;
    p_config->IsEnabled = false;
}

/* Hybrid Functions - Both patterns available */
static inline void Monitor_Config_UpdateAsOverLimit(Monitor_Config_T * p_config, int32_t nominal, uint8_t warnPercent, uint8_t faultPercent)
{
    /* Update existing config while preserving IsEnabled state */
    bool wasEnabled = p_config->IsEnabled;
    Monitor_Config_InitAsOverLimit(p_config, nominal, warnPercent, faultPercent);
    p_config->IsEnabled = wasEnabled;
}

static inline Monitor_Config_T Monitor_Config_FromExisting(const Monitor_Config_T * p_existing, int32_t newNominal)
{
    /* Create new config based on existing one with new nominal */
    Monitor_Config_T newConfig = *p_existing;
    newConfig.Nominal = newNominal;
    return newConfig;
}

/* Batch Operations using Pattern 2 */
static inline void Monitor_Config_InitArray(Monitor_Config_T * p_configs, uint8_t count, int32_t baseNominal, uint8_t warnPercent, uint8_t faultPercent, int32_t nominalStep)
{
    for (uint8_t i = 0; i < count; i++)
    {
        Monitor_Config_InitAsOverLimit(&p_configs[i], baseNominal + (i * nominalStep), warnPercent, faultPercent);
    }
}



/******************************************************************************/
/*
    Range-based Configuration Functions (for RangeMonitor compatibility)
*/
/******************************************************************************/

/* Create high-side config from RangeMonitor_Config_T */
static inline Monitor_Config_T Monitor_Config_FromRangeHigh(const RangeMonitor_Config_T * p_rangeConfig)
{
    bool isValid = (p_rangeConfig->FaultOverLimit.Limit > p_rangeConfig->Warning.LimitHigh) &&
                   (p_rangeConfig->Warning.LimitHigh > p_rangeConfig->Nominal);

    return (Monitor_Config_T)
    {
        .IsEnabled = isValid && p_rangeConfig->IsEnabled,
        .Nominal = p_rangeConfig->Nominal,
        .Warning =
        {
            .Setpoint = p_rangeConfig->Warning.LimitHigh,
            .Resetpoint = p_rangeConfig->Warning.LimitHigh - p_rangeConfig->Warning.Hysteresis
        },
        .Fault = p_rangeConfig->FaultOverLimit
    };
}

/* Create low-side config from RangeMonitor_Config_T */
static inline Monitor_Config_T Monitor_Config_FromRangeLow(const RangeMonitor_Config_T * p_rangeConfig)
{
    bool isValid = (p_rangeConfig->FaultUnderLimit.Limit < p_rangeConfig->Warning.LimitLow) &&
                   (p_rangeConfig->Warning.LimitLow < p_rangeConfig->Nominal);

    return (Monitor_Config_T)
    {
        .IsEnabled = isValid && p_rangeConfig->IsEnabled,
        .Nominal = p_rangeConfig->Nominal,
        .Warning =
        {
            .Setpoint = p_rangeConfig->Warning.LimitLow,
            .Resetpoint = p_rangeConfig->Warning.LimitLow + p_rangeConfig->Warning.Hysteresis
        },
        .Fault = p_rangeConfig->FaultUnderLimit
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
    int32_t warnOffset = (nominal * warnPercent) / 100;
    int32_t faultOffset = (nominal * faultPercent) / 100;
    uint32_t hysteresis = (nominal * hystPercent) / 100;

    return RangeMonitor_Config_Symmetric(nominal, faultOffset, warnOffset, hysteresis);
}



// static inline void RangeMonitor_ConfigZone_InitSymmetric(RangeMonitor_Zone_T * p_zone, int32_t nominal, int32_t offset, uint32_t hysteresis)
// {
//     p_zone->LimitHigh = nominal + offset;
//     p_zone->LimitLow = nominal - offset;
//     p_zone->Hysteresis = hysteresis;
// }

// static inline void RangeMonitor_Config_InitZoneSymmetric(RangeMonitor_Config_T * p_this, Monitor_Status_T level, int32_t nominal, int32_t offset, uint32_t hysteresis)
// {
//     switch (level)
//     {
//         case MONITOR_STATUS_NORMAL: break;
//         case MONITOR_STATUS_WARNING: RangeMonitor_ConfigZone_InitSymmetric(&p_this->Warning, nominal, offset, hysteresis); return;
//         case MONITOR_STATUS_FAULT: return;
//         default: return;
//     }
// }


/*
    In-place configuration for symmetric monitoring
    - Builds warning limits symmetrically around nominal
    - Builds fault limits symmetrically around nominal
    - Hysteresis is applied to both warning limits
*/
static inline void RangeMonitor_Config_InitSymmetric(RangeMonitor_Config_T * p_this, int32_t nominal, int32_t faultOffset, int32_t warnOffset, uint32_t warnHysteresis)
{
    p_this->Nominal = nominal;
    p_this->FaultOverLimit.Limit = nominal + faultOffset;
    p_this->FaultUnderLimit.Limit = nominal - faultOffset;
    p_this->Warning.LimitHigh = nominal + warnOffset;
    p_this->Warning.LimitLow = nominal - warnOffset;
    p_this->Warning.Hysteresis = warnHysteresis;
}

/* Init all except enable */
static inline void RangeMonitor_Config_InitSymmetricPercent(RangeMonitor_Config_T * p_this, int32_t nominal, uint8_t faultPercent, uint8_t warnPercent, uint8_t hystPercent)
{
    RangeMonitor_Config_InitSymmetric
    (
        p_this,
        nominal,
        (nominal * faultPercent) / 100,
        (nominal * warnPercent) / 100,
        (nominal * hystPercent) / 100
    );
    // *p_this = RangeMonitor_Config_SymmetricPercent(nominal, faultPercent, warnPercent, hystPercent);
}
