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
    @file   Motor_Intervention.h
    @author FireSourcery
    @brief  Motor Intervention SubStates - Hypervisor-layer safe stop functions

    Hypervisor-layer safe stop functions at the Motor level.
    Intervention State removes user command from the control loop.

    SubState structure:
    ┌──────────────────────────────────────────────────┐
    │              INTERVENTION (parent)                │
    │  Entry: MatchIVState=, TorqueRamp => 0            │
    │  Common: fault, control input routing             │
    │                                                   │
    │  ┌────────────────┐     ┌──────────────────────┐  │
    │  │  TORQUE_ZERO   │────>│    RAMP_SAFE         │  │
    │  │  (initial)     │ timeout                    │  │
    │  │  SS0: coast    │     │  SS1: active decel   │  │
    │  └────────────────┘     └──────────────────────┘  │
    │          │                        │               │
    │          v                        v               │
    │     [speed < limit]         [speed < limit]       │
    │          │                        │               │
    │          └────────> PASSIVE <─────┘               │
    └──────────────────────────────────────────────────┘

    Two substates provide SS0/SS1 semantics:
      TORQUE_ZERO  (SS0) - Immediate torque removal, coast-down, user may resume.
      RAMP_SAFE    (SS1) - Active deceleration to zero speed, no user resume.
*/
/******************************************************************************/
#include "../Motor_StateMachine.h"

/******************************************************************************/
/*
    Intervention timeout - ticks at PWM frequency (20kHz)
    Coast timeout before escalation to active deceleration
*/
/******************************************************************************/
#ifndef MOTOR_INTERVENTION_COAST_TIMEOUT
#define MOTOR_INTERVENTION_COAST_TIMEOUT (60000U) /* 3 seconds at 20kHz */
#endif

/* Ramp-down watchdog - if speed not decreasing after this many ticks, escalate to fault */
#ifndef MOTOR_INTERVENTION_RAMP_WATCHDOG
#define MOTOR_INTERVENTION_RAMP_WATCHDOG (200000U) /* 10 seconds at 20kHz */
#endif

/******************************************************************************/
/*
    SubState externs - for direct entry from Run_InputRelease
*/
/******************************************************************************/
extern const State_T INTERVENTION_STATE_TORQUE_ZERO;
extern const State_T INTERVENTION_STATE_RAMP_SAFE;

