#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   VBus_Monitor.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "VBus.h"
#include "Transducer/Monitor/Base/RangeMonitor.h"
#include "Transducer/Monitor/Voltage/VMonitor.h"

/******************************************************************************/
/*!
    Monitor Integration — entry points called by MotorController layer.

    MotorController_Thread converts ADCU -> fract16 and calls VBus_CaptureAndPoll
    every monitor tick. The returned RangeMonitor_Status_T drives fault dispatch,
    beep, LimitArray, and multi-motor disable at the controller layer.
*/
/******************************************************************************/
// typedef VMonitor_State_T VBus_Monitor_T;
// static inline VMonitor_Status_T VBus_CaptureAndPoll(VBus_T * p_vbus, VMonitor_State_T * p_monitorState, uint16_t fract16)
// {
//     VBus_CaptureFract16(p_vbus, fract16);
//     return (VMonitor_Status_T)RangeMonitor_Poll(p_monitorState, p_vbus->VBus_Fract16);
// }

static inline VMonitor_State_T * VBus_Monitor(VBus_T * p_vbus) { return &p_vbus->MonitorState; }

/* Poll Monitor Only */
static inline VMonitor_Status_T VBus_PollMonitor(VBus_T * p_vbus) { return (VMonitor_Status_T)RangeMonitor_Poll(&p_vbus->MonitorState, p_vbus->VBus_Fract16); }

static inline VMonitor_Status_T VBus_PollCaptureMonitor(VBus_T * p_vbus, uint16_t fract16) { return (VMonitor_Status_T)RangeMonitor_Poll(&p_vbus->MonitorState, fract16); }

/*

*/
static inline VMonitor_Status_T VBus_Status(const VBus_T * p_vbus) { return (VMonitor_Status_T)p_vbus->MonitorState.Status; }
// static inline bool VBus_IsUnderWarning(const VBus_T * p_vbus) { return RangeMonitor_IsUnderWarning(&p_vbus->MonitorState); }
// static inline bool VBus_IsOverWarning(const VBus_T * p_vbus) { return RangeMonitor_IsOverWarning(&p_vbus->MonitorState); }
// static inline bool VBus_IsUnderFault(const VBus_T * p_vbus) { return RangeMonitor_IsUnderFault(&p_vbus->MonitorState); }
// static inline bool VBus_IsOverFault(const VBus_T * p_vbus) { return RangeMonitor_IsOverFault(&p_vbus->MonitorState); }
// static inline bool VBus_IsUnderNominal(const VBus_T * p_vbus) { return RangeMonitor_IsUnderNominal(&p_vbus->MonitorState); }
// static inline bool VBus_IsOverNominal(const VBus_T * p_vbus) { return RangeMonitor_IsOverNominal(&p_vbus->MonitorState); }

static inline bool VBus_IsTriggeringEdge(const VBus_T * p_vbus) { return RangeMonitor_IsTriggeringEdge(&p_vbus->MonitorState); }
static inline bool VBus_IsClearingEdge(const VBus_T * p_vbus) { return RangeMonitor_IsClearingEdge(&p_vbus->MonitorState); }
static inline bool VBus_IsAnyFault(const VBus_T * p_vbus) { return RangeMonitor_IsAnyFault(&p_vbus->MonitorState); }
static inline bool VBus_IsEnabled(const VBus_T * p_vbus) { return RangeMonitor_IsEnabled(&p_vbus->MonitorState.Config); }