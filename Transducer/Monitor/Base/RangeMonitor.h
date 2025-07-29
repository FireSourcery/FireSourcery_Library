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
    @file   RangeMonitor.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Monitor.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
    @brief RangeMonitor - 2-sided version of Monitor
*/
/******************************************************************************/

/******************************************************************************/
/*!
    @brief  RangeMonitor Status Enum - Intentionally Mathematically Aligned

    FAULT_UNDER(-2) < WARNING_LOW(-1) < NORMAL(0) < WARNING_HIGH(1) < FAULT_OVER(2)

    DESIGN PRINCIPLE: Enum values are intentionally aligned to enable
    mathematical operations for elegant query functions:

    - abs(status) gives severity level (0=normal, 1=warning, 2=fault)
    - Sign indicates direction (negative=under, positive=over)
    - Zero represents normal condition
    - Magnitude comparison works for severity assessment

    @warning Do not change these values without updating all abs() operations
*/
/******************************************************************************/
typedef enum RangeMonitor_Status
{
    MONITOR_STATUS_FAULT_UNDER_LIMIT = (MONITOR_STATUS_FAULT * -1),
    MONITOR_STATUS_WARNING_LOW = (MONITOR_STATUS_WARNING * -1),
    RANGE_MONITOR_STATUS_NORMAL = MONITOR_STATUS_NORMAL,
    MONITOR_STATUS_WARNING_HIGH = MONITOR_STATUS_WARNING,
    MONITOR_STATUS_FAULT_OVER_LIMIT = MONITOR_STATUS_FAULT,
}
RangeMonitor_Status_T;

/*
    RangeMonitor simplify config
*/
// typedef struct RangeMonitor_Threshold
typedef struct RangeMonitor_Zone
{
    int32_t LimitHigh;
    int32_t LimitLow;
    uint32_t Hysteresis;     /* Hysteresis band. Applies to both directions */
    //alternatively bnadwith via setter
    // Monitor_Setpoint_T High;
    // Monitor_Setpoint_T Low;
}
RangeMonitor_Zone_T;

/*
    Simplified config with shared hysteresis offset for each level
*/
typedef struct RangeMonitor_Config
{
    /* Fault thresholds (no hysteresis) */
    Monitor_FaultLimit_T FaultOverLimit;     /* Fault level (hard limit, no hysteresis) */
    Monitor_FaultLimit_T FaultUnderLimit;    /* Fault level (hard limit, no hysteresis) */

    /* Warning thresholds with hysteresis */
    RangeMonitor_Zone_T Warning;   /* Warning level with hysteresis */

    int32_t Nominal;  /*  */
    bool IsEnabled;
    // RangeMonitor_Direction_T Direction;
}
RangeMonitor_Config_T;


typedef struct RangeMonitor
{
    Monitor_Base_T MonitorHigh;
    Monitor_Base_T MonitorLow;

    /* Combined state */
    // Monitor_Result_T Result;
    RangeMonitor_Status_T Status;        /* Overall status */
    RangeMonitor_Status_T StatusPrev;    /* Previous status for edge detection */
    int32_t LastInput;                   /* Last input value */

    RangeMonitor_Config_T Config;           /* Hold for runtime update */
}
RangeMonitor_T;


/******************************************************************************/
/*!
    @brief  Query functions leveraging intentional enum alignment
*/
/******************************************************************************/
/*
    Status
*/
/* Severity detection using absolute values */
/* Get severity level regardless of direction */
static inline RangeMonitor_Status_T RangeMonitor_GetStatus(const RangeMonitor_T * p_monitor) { return p_monitor->Status; }
static inline Monitor_Status_T RangeMonitor_GetSeverity(const RangeMonitor_T * p_monitor) { return (Monitor_Status_T)abs(p_monitor->Status); }
/* abs(±2) == 2 */
static inline bool RangeMonitor_IsAnyFault(const RangeMonitor_T * p_monitor) { return abs(p_monitor->Status) == MONITOR_STATUS_FAULT; }
/* abs(±1) == 1 */
static inline bool RangeMonitor_IsAnyWarning(const RangeMonitor_T * p_monitor) { return abs(p_monitor->Status) == MONITOR_STATUS_WARNING; }
/* Direction detection using enum values */
static inline bool RangeMonitor_IsHighSideCondition(const RangeMonitor_T * p_monitor) { return p_monitor->Status > RANGE_MONITOR_STATUS_NORMAL; }
static inline bool RangeMonitor_IsLowSideCondition(const RangeMonitor_T * p_monitor) { return p_monitor->Status < RANGE_MONITOR_STATUS_NORMAL; }

static inline bool RangeMonitor_IsAnyCondition(const RangeMonitor_T * p_monitor) { return p_monitor->Status != RANGE_MONITOR_STATUS_NORMAL; }
/* Check which direction caused the issue */
static inline bool RangeMonitor_IsHighSideFault(const RangeMonitor_T * p_monitor) { return p_monitor->Status == MONITOR_STATUS_FAULT_OVER_LIMIT; }
static inline bool RangeMonitor_IsLowSideFault(const RangeMonitor_T * p_monitor) { return p_monitor->Status == MONITOR_STATUS_FAULT_UNDER_LIMIT; }
static inline bool RangeMonitor_IsHighSideWarning(const RangeMonitor_T * p_monitor) { return p_monitor->Status == MONITOR_STATUS_WARNING_HIGH; }
static inline bool RangeMonitor_IsLowSideWarning(const RangeMonitor_T * p_monitor) { return p_monitor->Status == MONITOR_STATUS_WARNING_LOW; }


/*
    Event detection
*/
/* Enhanced edge detection using enum alignment */
static inline bool RangeMonitor_IsFaultTriggering(const RangeMonitor_T * p_monitor) { return (abs(p_monitor->Status) == MONITOR_STATUS_FAULT) && (abs(p_monitor->StatusPrev) != MONITOR_STATUS_FAULT); }
static inline bool RangeMonitor_IsFaultClearing(const RangeMonitor_T * p_monitor) { return (abs(p_monitor->StatusPrev) == MONITOR_STATUS_FAULT) && (abs(p_monitor->Status) != MONITOR_STATUS_FAULT); }
static inline bool RangeMonitor_IsNormalEntering(const RangeMonitor_T * p_monitor) { return (p_monitor->Status == RANGE_MONITOR_STATUS_NORMAL) && (p_monitor->StatusPrev != RANGE_MONITOR_STATUS_NORMAL); }
static inline bool RangeMonitor_IsNormalExiting(const RangeMonitor_T * p_monitor) { return (p_monitor->StatusPrev == RANGE_MONITOR_STATUS_NORMAL) && (p_monitor->Status != RANGE_MONITOR_STATUS_NORMAL); }

// static inline bool RangeMonitor_IsSeverityRising(const RangeMonitor_T * p_monitor) { return abs(p_monitor->Status) > abs(p_monitor->StatusPrev); }
// static inline bool RangeMonitor_IsSeverityFalling(const RangeMonitor_T * p_monitor) { return abs(p_monitor->Status) < abs(p_monitor->StatusPrev); }
static inline bool RangeMonitor_IsTriggeringEdge(const RangeMonitor_T * p_monitor) { return abs(p_monitor->Status) > abs(p_monitor->StatusPrev); }
static inline bool RangeMonitor_IsClearingEdge(const RangeMonitor_T * p_monitor) { return abs(p_monitor->Status) < abs(p_monitor->StatusPrev); }

/* Direction change detection */
static inline bool RangeMonitor_IsDirectionCrossing(const RangeMonitor_T * p_monitor)
{
    return ((p_monitor->Status > RANGE_MONITOR_STATUS_NORMAL) != (p_monitor->StatusPrev > RANGE_MONITOR_STATUS_NORMAL)) &&
            (p_monitor->Status != RANGE_MONITOR_STATUS_NORMAL) &&
            (p_monitor->StatusPrev != RANGE_MONITOR_STATUS_NORMAL);
}


/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
/* Check if monitor is enabled */
static inline bool RangeMonitor_IsEnabled(const RangeMonitor_T * p_monitor) { return p_monitor->Config.IsEnabled; }
/* Check if high-side monitoring is properly configured */
// static inline bool RangeMonitor_IsHighSideEnabled(const RangeMonitor_T * p_monitor) { return p_monitor->MonitorHigh.Config.IsEnabled; }
/* Check if low-side monitoring is properly configured */
// static inline bool RangeMonitor_IsLowSideEnabled(const RangeMonitor_T * p_monitor) { return p_monitor->MonitorLow.Config.IsEnabled; }
static inline void RangeMonitor_SetEnabled(RangeMonitor_T * p_monitor, bool isEnabled) { p_monitor->Config.IsEnabled = isEnabled; }
static inline void RangeMonitor_Enable(RangeMonitor_T * p_monitor) { RangeMonitor_SetEnabled(p_monitor, true); }
static inline void RangeMonitor_Disable(RangeMonitor_T * p_monitor) { RangeMonitor_SetEnabled(p_monitor, false); }

/* Getters format */
static inline int32_t RangeMonitor_GetFaultOverLimit(const RangeMonitor_T * p_monitor) { return p_monitor->Config.FaultOverLimit.Limit; }
static inline int32_t RangeMonitor_GetFaultUnderLimit(const RangeMonitor_T * p_monitor) { return p_monitor->Config.FaultUnderLimit.Limit; }
static inline int32_t RangeMonitor_GetWarningLimitHigh(const RangeMonitor_T * p_monitor) { return p_monitor->Config.Warning.LimitHigh; }
static inline int32_t RangeMonitor_GetWarningLimitLow(const RangeMonitor_T * p_monitor) { return p_monitor->Config.Warning.LimitLow; }
static inline int32_t RangeMonitor_GetWarningDeadband(const RangeMonitor_T * p_monitor) { return p_monitor->Config.Warning.Hysteresis; }
static inline int32_t RangeMonitor_GetNominal(const RangeMonitor_T * p_monitor) { return p_monitor->Config.Nominal; }

/* Setters without propagating reset */
static inline void _RangeMonitor_SetFaultOverLimit(RangeMonitor_T * p_monitor, int32_t limit) { p_monitor->Config.FaultOverLimit.Limit = limit; }
static inline void _RangeMonitor_SetFaultUnderLimit(RangeMonitor_T * p_monitor, int32_t limit) { p_monitor->Config.FaultUnderLimit.Limit = limit; }
static inline void _RangeMonitor_SetWarningLimitHigh(RangeMonitor_T * p_monitor, int32_t limit) { p_monitor->Config.Warning.LimitHigh = limit; }
static inline void _RangeMonitor_SetWarningLimitLow(RangeMonitor_T * p_monitor, int32_t limit) { p_monitor->Config.Warning.LimitLow = limit; }
static inline void _RangeMonitor_SetWarningDeadband(RangeMonitor_T * p_monitor, int32_t deadband) { p_monitor->Config.Warning.Hysteresis = deadband; }
static inline void _RangeMonitor_SetNominal(RangeMonitor_T * p_monitor, int32_t nominal) { p_monitor->Config.Nominal = nominal; }


/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void RangeMonitor_InitFrom(RangeMonitor_T * p_monitor, const RangeMonitor_Config_T * p_config);
extern RangeMonitor_Status_T RangeMonitor_Evaluate(RangeMonitor_T * p_monitor, int32_t input);
extern RangeMonitor_Status_T RangeMonitor_Poll(RangeMonitor_T * p_monitor, int32_t input);
extern void RangeMonitor_Reset(RangeMonitor_T * p_monitor);

extern void RangeMonitor_SetFaultOverLimit(RangeMonitor_T * p_monitor, int32_t limit);
extern void RangeMonitor_SetFaultUnderLimit(RangeMonitor_T * p_monitor, int32_t limit);
extern void RangeMonitor_SetWarningLimitHigh(RangeMonitor_T * p_monitor, int32_t limit);
extern void RangeMonitor_SetWarningLimitLow(RangeMonitor_T * p_monitor, int32_t limit);
extern void RangeMonitor_SetWarningDeadband(RangeMonitor_T * p_monitor, int32_t deadband);
extern void RangeMonitor_SetNominal(RangeMonitor_T * p_monitor, int32_t nominal);


/******************************************************************************/
/*
    Configuration IDs
*/
/******************************************************************************/
typedef enum RangeMonitor_VarId
{
    RANGE_MONITOR_VAR_STATUS,
    RANGE_MONITOR_VAR_VALUE,
}
RangeMonitor_VarId_T;

int32_t _RangeMonitor_VarId_Get(const RangeMonitor_T * p_monitor, RangeMonitor_VarId_T varId);
int32_t RangeMonitor_VarId_Get(const RangeMonitor_T * p_monitor, RangeMonitor_VarId_T varId);

typedef enum RangeMonitor_ConfigId
{
    RANGE_MONITOR_CONFIG_FAULT_OVER_LIMIT,
    RANGE_MONITOR_CONFIG_FAULT_UNDER_LIMIT,
    RANGE_MONITOR_CONFIG_WARNING_LIMIT_HIGH,
    RANGE_MONITOR_CONFIG_WARNING_LIMIT_LOW,
    RANGE_MONITOR_CONFIG_WARNING_HYSTERESIS_BAND,
    RANGE_MONITOR_CONFIG_NOMINAL,
    RANGE_MONITOR_CONFIG_IS_ENABLED,
}
RangeMonitor_ConfigId_T;

int32_t _RangeMonitor_ConfigId_Get(const RangeMonitor_T * p_monitor, RangeMonitor_ConfigId_T configId);
void _RangeMonitor_ConfigId_Set(RangeMonitor_T * p_monitor, RangeMonitor_ConfigId_T configId, int32_t value);

int RangeMonitor_ConfigId_Get(const RangeMonitor_T * p_monitor, int configId);
void RangeMonitor_ConfigId_Set(RangeMonitor_T * p_monitor, int configId, int value);