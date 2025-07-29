#pragma once

/******************************************************************************/
/*!
    @file   Monitor.h
    @author FireSourcery
    @brief  Generic multi-level monitoring with conventional naming
*/
/******************************************************************************/
#include "Transducer/Math/math_hysteresis.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Industry Standard Status Levels - ISA-18.2 / ANSI Compliant
*/
/******************************************************************************/
// typedef enum Monitor_Status
// {
//     MONITOR_STATUS_NORMAL = 0,          /* Normal operating condition */
//     MONITOR_STATUS_ADVISORY,            /* Advisory notification (lowest concern) */
//     MONITOR_STATUS_CAUTION,             /* Caution condition (attention required) */
//     MONITOR_STATUS_WARNING,             /* Warning condition (action recommended) */
//     MONITOR_STATUS_ALARM,               /* Alarm condition (immediate action required) */
//     MONITOR_STATUS_CRITICAL,            /* Critical condition (emergency action) */
//     MONITOR_STATUS_FAULT,               /* Fault condition (system protection active) */
// }
// Monitor_Status_T;

// /* Alternative naming for specific domains */
// typedef enum Monitor_Level
// {
//     MONITOR_LEVEL_SAFE = 0,             /* Safe operating region */
//     MONITOR_LEVEL_NORMAL,               /* Normal operating region */
//     MONITOR_LEVEL_ATTENTION,            /* Requires attention */
//     MONITOR_LEVEL_CONCERN,              /* Cause for concern */
//     MONITOR_LEVEL_DANGER,               /* Dangerous condition */
//     MONITOR_LEVEL_EMERGENCY,            /* Emergency condition */
// }
// Monitor_Level_T;

// /* Directional status for asymmetric monitoring (voltage, current, etc.) */
// typedef enum Monitor_DirectionalStatus
// {
//     MONITOR_STATUS_NORMAL = 0,

//     /* Lower direction conditions */
//     MONITOR_STATUS_LOW_ADVISORY,        /* Below normal but acceptable */
//     MONITOR_STATUS_LOW_CAUTION,         /* Low caution */
//     MONITOR_STATUS_LOW_WARNING,         /* Low warning */
//     MONITOR_STATUS_LOW_ALARM,           /* Low alarm */
//     MONITOR_STATUS_LOW_CRITICAL,        /* Low critical */
//     MONITOR_STATUS_UNDERVOLTAGE,        /* Specific: undervoltage fault */
//     MONITOR_STATUS_UNDERTEMPERATURE,    /* Specific: undertemperature */
//     MONITOR_STATUS_UNDERCURRENT,        /* Specific: undercurrent */

//     /* Upper direction conditions */
//     MONITOR_STATUS_HIGH_ADVISORY,       /* Above normal but acceptable */
//     MONITOR_STATUS_HIGH_CAUTION,        /* High caution */
//     MONITOR_STATUS_HIGH_WARNING,        /* High warning */
//     MONITOR_STATUS_HIGH_ALARM,          /* High alarm */
//     MONITOR_STATUS_HIGH_CRITICAL,       /* High critical */
//     MONITOR_STATUS_OVERVOLTAGE,         /* Specific: overvoltage fault */
//     MONITOR_STATUS_OVERTEMPERATURE,     /* Specific: overtemperature */
//     MONITOR_STATUS_OVERCURRENT,         /* Specific: overcurrent */
// }
// Monitor_DirectionalStatus_T;

/******************************************************************************/
/*
    Conventional Event/Edge Naming - Based on Industrial Standards
*/
/******************************************************************************/
// typedef enum Monitor_Event
// {
//     MONITOR_EVENT_NONE = 0,

//     /* Condition onset events */
//     MONITOR_EVENT_ADVISORY_ONSET,       /* Advisory condition started */
//     MONITOR_EVENT_CAUTION_ONSET,        /* Caution condition started */
//     MONITOR_EVENT_WARNING_ONSET,        /* Warning condition started */
//     MONITOR_EVENT_ALARM_ONSET,          /* Alarm condition started */
//     MONITOR_EVENT_CRITICAL_ONSET,       /* Critical condition started */
//     MONITOR_EVENT_FAULT_ONSET,          /* Fault condition started */

//     /* Condition cleared events */
//     MONITOR_EVENT_ADVISORY_CLEARED,     /* Advisory condition cleared */
//     MONITOR_EVENT_CAUTION_CLEARED,      /* Caution condition cleared */
//     MONITOR_EVENT_WARNING_CLEARED,      /* Warning condition cleared */
//     MONITOR_EVENT_ALARM_CLEARED,        /* Alarm condition cleared */
//     MONITOR_EVENT_CRITICAL_CLEARED,     /* Critical condition cleared */
//     MONITOR_EVENT_FAULT_CLEARED,        /* Fault condition cleared */

//     /* System events */
//     MONITOR_EVENT_NORMAL_RESTORED,      /* Normal operation restored */
//     MONITOR_EVENT_MONITOR_ENABLED,      /* Monitoring enabled */
//     MONITOR_EVENT_MONITOR_DISABLED,     /* Monitoring disabled */
// }
// Monitor_Event_T;

/* Directional events for asymmetric monitoring */
// typedef enum Monitor_DirectionalEvent
// {
//     MONITOR_EVENT_NONE = 0,

//     /* Low direction events */
//     MONITOR_EVENT_LOW_WARNING_ONSET,
//     MONITOR_EVENT_LOW_WARNING_CLEARED,
//     MONITOR_EVENT_LOW_ALARM_ONSET,
//     MONITOR_EVENT_LOW_ALARM_CLEARED,
//     MONITOR_EVENT_LOW_CRITICAL_ONSET,
//     MONITOR_EVENT_LOW_CRITICAL_CLEARED,

//     /* High direction events */
//     MONITOR_EVENT_HIGH_WARNING_ONSET,
//     MONITOR_EVENT_HIGH_WARNING_CLEARED,
//     MONITOR_EVENT_HIGH_ALARM_ONSET,
//     MONITOR_EVENT_HIGH_ALARM_CLEARED,
//     MONITOR_EVENT_HIGH_CRITICAL_ONSET,
//     MONITOR_EVENT_HIGH_CRITICAL_CLEARED,

//     /* Normal restoration */
//     MONITOR_EVENT_NORMAL_RESTORED,
// }
// Monitor_DirectionalEvent_T;

/******************************************************************************/
/*
    Monitor Priority Levels - Conventional Industry Standards
*/
/******************************************************************************/
// typedef enum Monitor_Priority
// {
//     MONITOR_PRIORITY_INFO = 0,          /* Informational (no action required) */
//     MONITOR_PRIORITY_LOW = 1,           /* Low priority (log only) */
//     MONITOR_PRIORITY_MEDIUM = 2,        /* Medium priority (operator attention) */
//     MONITOR_PRIORITY_HIGH = 3,          /* High priority (immediate attention) */
//     MONITOR_PRIORITY_CRITICAL = 4,      /* Critical priority (emergency response) */
// }
// Monitor_Priority_T;

// static inline Monitor_Priority_T Monitor_GetPriority(const Monitor_T * p_monitor)
// {
//     switch (p_monitor->Status)
//     {
//         case MONITOR_STATUS_NORMAL:     return MONITOR_PRIORITY_NONE;
//         case MONITOR_STATUS_ADVISORY:   return MONITOR_PRIORITY_LOW;
//         case MONITOR_STATUS_WARNING:    return MONITOR_PRIORITY_MEDIUM;
//         case MONITOR_STATUS_ALARM:      return MONITOR_PRIORITY_HIGH;
//         case MONITOR_STATUS_CRITICAL:   return MONITOR_PRIORITY_CRITICAL;
//         case MONITOR_STATUS_FAULT:      return MONITOR_PRIORITY_CRITICAL;
//         default:                        return MONITOR_PRIORITY_NONE;
//     }
// }

/******************************************************************************/
/*
    Response Actions - Conventional Industrial Practices
*/
/******************************************************************************/
// typedef enum Monitor_ResponseAction
// {
//     MONITOR_ACTION_NONE = 0,            /* No action required */
//     MONITOR_ACTION_LOG_ONLY,            /* Log event only */
//     MONITOR_ACTION_NOTIFY_OPERATOR,     /* Notify operator */
//     MONITOR_ACTION_REDUCE_LOAD,         /* Reduce system load */
//     MONITOR_ACTION_ACTIVATE_PROTECTION, /* Activate protection systems */
//     MONITOR_ACTION_EMERGENCY_SHUTDOWN,  /* Emergency shutdown */
//     MONITOR_ACTION_ISOLATE_SYSTEM,      /* Isolate affected system */
// }
// Monitor_ResponseAction_T;

/******************************************************************************/
/*
    Generic Monitor Configuration
*/
/******************************************************************************/
// typedef struct Monitor_ThresholdConfig
// {
//     int32_t Normal_Upper;               /* Upper bound of normal range */
//     int32_t Normal_Lower;               /* Lower bound of normal range */

//     int32_t Advisory_Upper;             /* Advisory upper threshold */
//     int32_t Advisory_Lower;             /* Advisory lower threshold */

//     int32_t Caution_Upper;              /* Caution upper threshold */
//     int32_t Caution_Lower;              /* Caution lower threshold */

//     int32_t Warning_Upper;              /* Warning upper threshold */
//     int32_t Warning_Lower;              /* Warning lower threshold */

//     int32_t Alarm_Upper;                /* Alarm upper threshold */
//     int32_t Alarm_Lower;                /* Alarm lower threshold */

//     int32_t Critical_Upper;             /* Critical upper threshold */
//     int32_t Critical_Lower;             /* Critical lower threshold */

//     /* Hysteresis settings */
//     int32_t Hysteresis_Band;            /* Hysteresis band width */
//     bool Enable_Hysteresis;             /* Enable hysteresis behavior */

//     /* Monitor settings */
//     bool Enable_Monitoring;             /* Master enable */
//     bool Enable_Directional;            /* Enable directional monitoring */
// }
// Monitor_ThresholdConfig_T;



/******************************************************************************/
/*
    Event Detection - Simplified
*/
/******************************************************************************/
// static inline Monitor_Event_T Monitor_GetLastEvent(const Monitor_T * p_monitor)
// {
//     if (!Monitor_HasStatusChanged(p_monitor))
//         return MONITOR_EVENT_NONE;

//     /* Status increased in severity */
//     if (p_monitor->Status > p_monitor->StatusPrev)
//     {
//         switch (p_monitor->Status)
//         {
//             case MONITOR_STATUS_ADVISORY:   return MONITOR_EVENT_ADVISORY_ONSET;
//             case MONITOR_STATUS_WARNING:    return MONITOR_EVENT_WARNING_ONSET;
//             case MONITOR_STATUS_ALARM:      return MONITOR_EVENT_ALARM_ONSET;
//             case MONITOR_STATUS_CRITICAL:   return MONITOR_EVENT_CRITICAL_ONSET;
//             case MONITOR_STATUS_FAULT:      return MONITOR_EVENT_FAULT_ONSET;
//             default:                        return MONITOR_EVENT_NONE;
//         }
//     }
//     /* Status decreased in severity */
//     else
//     {
//         switch (p_monitor->StatusPrev)
//         {
//             case MONITOR_STATUS_ADVISORY:   return MONITOR_EVENT_ADVISORY_CLEARED;
//             case MONITOR_STATUS_WARNING:    return MONITOR_EVENT_WARNING_CLEARED;
//             case MONITOR_STATUS_ALARM:      return MONITOR_EVENT_ALARM_CLEARED;
//             case MONITOR_STATUS_CRITICAL:   return MONITOR_EVENT_CRITICAL_CLEARED;
//             case MONITOR_STATUS_FAULT:      return MONITOR_EVENT_FAULT_CLEARED;
//             default:                        return MONITOR_EVENT_NORMAL_RESTORED;
//         }
//     }
// }
