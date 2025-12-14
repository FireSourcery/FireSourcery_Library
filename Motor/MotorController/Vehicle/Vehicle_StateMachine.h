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
    @file   Vehicle_StateMachine.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Vehicle.h"
#include "Utility/StateMachine/StateMachine.h"

/******************************************************************************/
/*
    State Machine
    caller handle Park state
*/
/******************************************************************************/
/******************************************************************************/
/*
    MotorController (System Level)
        │
        ├─ PARK State ────────────► Motors: STOP, Direction: NULL
        │                            Output: V0 or Float
        │                            Use: Parking brake, standby
        │
        └─ MAIN State
            │
            └─ VEHICLE Substate (Vehicle Level)
                │
                ├─ NEUTRAL ───────► Motors: PASSIVE, Direction: CW/CCW (retained)
                │                   Output: Float (or brake if commanded)
                │                   Use: Clutch disengaged, ready to engage
                │
                └─ DRIVE ─────────► Motors: RUN, Direction: CW or CCW
                    │               Output: PWM (active control)
                    ├─ Forward Mode  Direction: Calibrated CCW/CW
                    └─ Reverse Mode  Direction: Calibrated CCW/CW
*/
/******************************************************************************/
typedef enum Vehicle_State_Input
{
    VEHICLE_STATE_INPUT_DIRECTION,    /* Drive Direction */
    VEHICLE_STATE_INPUT_DRIVE_CMD,    /* On Edge, Drive Cmd -Throttle/Brake/Release */
}
Vehicle_State_Input_T;

typedef enum Vehicle_StateId
{
    VEHICLE_STATE_ID_DRIVE,
    VEHICLE_STATE_ID_NEUTRAL,
}
Vehicle_StateId_T;

extern const StateMachine_Machine_T VEHICLE_MACHINE;

#define VEHICLE_STATE_MACHINE_INIT(p_VehicleContext, VehicleStateAlloc) STATE_MACHINE_INIT((p_VehicleContext), &VEHICLE_MACHINE, &((VehicleStateAlloc).StateMachine))





