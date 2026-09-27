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
    @brief

    Intervention State removes user command from the control loop.

    Hypervisor-layer safe stop functions at the Motor level.
    Motor States are divided by control logic, this can be implemented here.

    SubStates:
      TORQUE_ZERO  (ZTC) - coast/Regen, user may resume.
      RAMP_DOWN          - Active deceleration to zero speed, user may resume.
      SAFE_STOP    (SS1) - RAMP_DOWN child. User resume rejected, watchdog to Fault.
*/
/******************************************************************************/
#include "Motor_StateMachine.h"


/******************************************************************************/
/*
    SubState externs - for direct entry from Run and Motor_User
*/
/******************************************************************************/
extern const State_T INTERVENTION_STATE_TORQUE_ZERO;
extern const State_T INTERVENTION_STATE_RAMP_DOWN;
extern const State_T INTERVENTION_STATE_SAFE_STOP;

