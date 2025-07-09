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
    @file   Monitor.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Transducer/Hysteresis/Hysteresis.h"

#include <stdint.h>
#include <stdbool.h>

typedef int32_t monitor_t;
// typedef uint16_t monitor_t; /* Directly ADC values */

/******************************************************************************/
/*
    Industry Standard Status Levels - ISA-18.2 / ANSI Compliant
*/
/******************************************************************************/
typedef enum Monitor_Status
{
    MONITOR_STATUS_NORMAL = 0,          /* Normal operating condition */
    MONITOR_STATUS_WARNING,             /* Warning condition - operator response required */
    MONITOR_STATUS_FAULT,               /* System fault - automatic protection active */
    /* expandable to 6 level status */
}
Monitor_Status_T;

typedef enum Monitor_Mode
{
    MONITOR_THRESHOLD_LOW = -1,     /* Monitor low side (undervoltage, undercurrent) */
    MONITOR_DISABLED = 0,           /* Monitor disabled */
    MONITOR_THRESHOLD_HIGH = 1,     /* Monitor high side (overvoltage, overtemp) */
}
Monitor_Mode_T;

/******************************************************************************/
/*
    Generic Setpoint Structure
*/
/******************************************************************************/
/*
    Base Config
*/
typedef struct Monitor_Setpoint
{
    int32_t Setpoint;
    int32_t Resetpoint; /* single direction use resetpoint. Dual direction derive from band width */
    // bool IsEnabled; /* Per level disable */
}
Monitor_Setpoint_T;

typedef struct Monitor_FaultLimit
{
    int32_t Limit;
    int32_t RecoveryThreshold;
    // uint32_t CooldownTime;      /* Required cooldown before restart */
    // bool AutoRestartEnabled;    /* Allow automatic restart */
}
Monitor_FaultLimit_T;

/*
    Base Runtime
*/
/*
    Monitor inner state, for common polling logic.
*/
typedef struct Monitor_Base
{
    int32_t FaultLimit;             /* Fault level (hard limit, no hysteresis) */
    Hysteresis_T Warning;           /* Warning level with hysteresis */
    // Hysteresis_T Levels[]//correspond with status for fast access evaluted output
}
Monitor_Base_T;

/*
    Cache derived values
*/
// typedef struct Monitor_Result
// {
//     Monitor_Status_T Status;            /* Current status for this direction */
//     Monitor_Status_T StatusPrev;        /* Previous status for edge detection */
//     int32_t LastInput;                  /* Last input value */
// }
// Monitor_Result_T;


/******************************************************************************/
/*

*/
/******************************************************************************/
typedef struct Monitor_Config
{
    Monitor_FaultLimit_T Fault;     /* Fault level (hard limit, no hysteresis) */
    Monitor_Setpoint_T Warning;     /* Warning level with hysteresis */
    int32_t Nominal;
    bool IsEnabled;

    /* alternatively */
    // Monitor_Setpoint_T Thresholds[]
    // Monitor_Setpoint_T Alarm;           /* Alarm level with hysteresis */
    // Monitor_Setpoint_T Advisory;        /* Advisory level with hysteresis */
}
Monitor_Config_T;

/*
    Single direction monitoring side
*/
typedef struct Monitor
{
    Monitor_Base_T Base;        /* Base configuration for this monitor */
    Monitor_Mode_T Direction;   /* Which direction this monitors */

    /* Cached Output */
    Monitor_Status_T Status;            /* Current status for this direction */
    Monitor_Status_T StatusPrev;        /* Previous status for edge detection */
    int32_t LastInput;                  /* Last input value */

    Monitor_Config_T Config;            /* Hold for runtime update */
}
Monitor_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
/* -1 for low-acting, 1 for high-acting */
/* 0 for disable */
static inline int32_t _Monitor_DirectionOf(Monitor_Setpoint_T * p_setpoint) { return (p_setpoint->Setpoint - p_setpoint->Resetpoint); }

/* Direction normalized compare value. Invert if lower value is higher serverity */
static inline int32_t Monitor_GetLastInputComparable(const Monitor_T * p_monitor) { return p_monitor->LastInput * p_monitor->Direction; }
static inline bool Monitor_IsEnabled(const Monitor_T * p_monitor) { return p_monitor->Direction != MONITOR_DISABLED; }

/* LastOutput = output state of last status hystersis level. set on status poll, or switch on status */

/******************************************************************************/
/*
    Industry Standard Query Functions
*/
/******************************************************************************/
/*
    Status
*/
static inline Monitor_Status_T Monitor_GetStatus(const Monitor_T * p_monitor) { return p_monitor->Status; }
static inline bool Monitor_IsNormal(const Monitor_T * p_monitor) { return p_monitor->Status == MONITOR_STATUS_NORMAL; }
static inline bool Monitor_IsAbnormal(const Monitor_T * p_monitor) { return p_monitor->Status != MONITOR_STATUS_NORMAL; }

/*

*/
// static inline bool Monitor_RequiresOperatorAttention(const Monitor_T * p_monitor) { return p_monitor->Status >= MONITOR_STATUS_WARNING; }
// static inline bool Monitor_RequiresImmediateAction(const Monitor_T * p_monitor) { return p_monitor->Status >= MONITOR_STATUS_ALARM; }
// static inline bool Monitor_IsInEmergencyState(const Monitor_T * p_monitor) { return (p_monitor->Status == MONITOR_STATUS_CRITICAL) || (p_monitor->Status == MONITOR_STATUS_FAULT); }


/*
    Event detection
*/
/* Specific condition edge detection */
static inline bool Monitor_IsFaultTriggering(const Monitor_T * p_monitor) { return (p_monitor->Status == MONITOR_STATUS_FAULT) && (p_monitor->StatusPrev != MONITOR_STATUS_FAULT); }
static inline bool Monitor_IsFaultClearing(const Monitor_T * p_monitor) { return (p_monitor->StatusPrev == MONITOR_STATUS_FAULT) && (p_monitor->Status != MONITOR_STATUS_FAULT); }
static inline bool Monitor_IsWarningTriggering(const Monitor_T * p_monitor) { return (p_monitor->Status == MONITOR_STATUS_WARNING) && (p_monitor->StatusPrev != MONITOR_STATUS_WARNING); }
static inline bool Monitor_IsWarningClearing(const Monitor_T * p_monitor) { return (p_monitor->StatusPrev == MONITOR_STATUS_WARNING) && (p_monitor->Status != MONITOR_STATUS_WARNING); }

/* Edge direction detection */
static inline bool Monitor_IsStatusTriggering(const Monitor_T * p_monitor) { return p_monitor->Status > p_monitor->StatusPrev; }
static inline bool Monitor_IsStatusClearing(const Monitor_T * p_monitor) { return p_monitor->Status < p_monitor->StatusPrev; }
static inline bool Monitor_IsStatusEdge(const Monitor_T * p_monitor) { return p_monitor->Status != p_monitor->StatusPrev; }

/*
    -1 for clear, Status decreased in severity
    +1 for set, Status increased in severity
    0 for no change, Status unchanged
*/
static inline int32_t Monitor_GetStatusEdge(const Monitor_T * p_monitor) { return math_sign(p_monitor->Status - p_monitor->StatusPrev); }


/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
/* Getters */
static inline int32_t Monitor_GetFaultLimit(const Monitor_T * p_monitor) { return p_monitor->Config.Fault.Limit; }
static inline int32_t Monitor_GetWarningSetpoint(const Monitor_T * p_monitor) { return p_monitor->Config.Warning.Setpoint; }
static inline int32_t Monitor_GetWarningResetpoint(const Monitor_T * p_monitor) { return p_monitor->Config.Warning.Resetpoint; }
static inline int32_t Monitor_GetNominal(const Monitor_T * p_monitor) { return p_monitor->Config.Nominal; }

/* Setters */
static inline void _Monitor_SetFaultLimit(Monitor_T * p_monitor, int32_t limit) { p_monitor->Config.Fault.Limit = limit; }
static inline void _Monitor_SetWarningSetpoint(Monitor_T * p_monitor, int32_t setpoint) { p_monitor->Config.Warning.Setpoint = setpoint; }
static inline void _Monitor_SetWarningResetpoint(Monitor_T * p_monitor, int32_t resetpoint) { p_monitor->Config.Warning.Resetpoint = resetpoint; }
static inline void _Monitor_SetNominal(Monitor_T * p_monitor, int32_t nominal) { p_monitor->Config.Nominal = nominal; }

static inline bool Monitor_IsConfigEnabled(const Monitor_T * p_monitor) { return p_monitor->Config.IsEnabled; }
static inline void Monitor_SetEnabled(Monitor_T * p_monitor, bool isEnabled) { p_monitor->Config.IsEnabled = isEnabled; }
static inline void Monitor_Enable(Monitor_T * p_monitor) { Monitor_SetEnabled(p_monitor, true); }
static inline void Monitor_Disable(Monitor_T * p_monitor) { Monitor_SetEnabled(p_monitor, false); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void _Monitor_InitFrom(Monitor_Base_T * p_monitor, const Monitor_Config_T * p_config);
extern Monitor_Status_T _Monitor_EvaluateAsHigh(Monitor_Base_T * p_monitor, int32_t input);
extern Monitor_Status_T _Monitor_EvaluateAsLow(Monitor_Base_T * p_monitor, int32_t input);

/******************************************************************************/
/*

*/
/******************************************************************************/
extern void Monitor_InitFrom(Monitor_T * p_monitor, const Monitor_Config_T * p_config);
extern Monitor_Status_T Monitor_Evaluate(Monitor_T * p_monitor, int32_t input);
extern Monitor_Status_T Monitor_Poll(Monitor_T * p_monitor, int32_t input);

extern void Monitor_Reset(Monitor_T * p_monitor);
extern bool Monitor_IsValidConfig(const Monitor_Config_T * p_config);
extern void Monitor_SetFaultLimit(Monitor_T * p_monitor, int32_t limit) ;
extern void Monitor_SetWarningSetpoint(Monitor_T * p_monitor, int32_t setpoint) ;
extern void Monitor_SetWarningResetpoint(Monitor_T * p_monitor, int32_t resetpoint);
extern void Monitor_SetNominal(Monitor_T * p_monitor, int32_t nominal);


/******************************************************************************/
/*

*/
/******************************************************************************/
typedef enum Monitor_VarId
{
    MONITOR_VAR_STATUS,
    MONITOR_VAR_VALUE,
}
Monitor_VarId_T;

int _Monitor_VarId_Get(const Monitor_T * p_monitor, Monitor_VarId_T varId);
int Monitor_VarId_Get(const Monitor_T * p_monitor, Monitor_VarId_T varId);

typedef enum Monitor_ConfigId
{
    MONITOR_CONFIG_FAULT_LIMIT,
    MONITOR_CONFIG_WARNING_SETPOINT,
    MONITOR_CONFIG_WARNING_RESETPOINT,
    MONITOR_CONFIG_NOMINAL,
    MONITOR_CONFIG_IS_ENABLED,
}
Monitor_ConfigId_T;

int _Monitor_ConfigId_Get(const Monitor_T * p_monitor, Monitor_ConfigId_T id);
void _Monitor_ConfigId_Set(Monitor_T * p_monitor, Monitor_ConfigId_T id, int value);

int Monitor_ConfigId_Get(const Monitor_T * p_monitor, Monitor_ConfigId_T id);
void Monitor_ConfigId_Set(Monitor_T * p_monitor, Monitor_ConfigId_T id, int value);