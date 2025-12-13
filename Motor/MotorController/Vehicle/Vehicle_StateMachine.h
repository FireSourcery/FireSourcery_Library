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
                    ├─ Forward Mode  Direction: CCW
                    └─ Reverse Mode  Direction: CW
*/
/******************************************************************************/
/*
    State Machine
*/
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



/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/* also returns NEUTRAL on error */
/* Alternatively use substates */
static Motor_User_Direction_T Vehicle_StateMachine_GetDirection(const Vehicle_T * p_vehicle)
{
    Motor_User_Direction_T direction;
    switch (StateMachine_GetActiveStateId(p_vehicle->STATE_MACHINE.P_ACTIVE))
    {
        case VEHICLE_STATE_ID_NEUTRAL:    direction = MOTOR_DIRECTION_NONE; break;
        case VEHICLE_STATE_ID_DRIVE:      direction = _MotMotors_GetDirectionAll(&p_vehicle->MOTORS); break; /* NULL is error */
        default:                          direction = 0;           break;
    }
    return direction;
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
// /* Separate Check direction with alarm, so Motor set can use SetSyncInput */
// bool Vehicle_User_CheckDirection(const Vehicle_T * p_vehicle, Motor_User_Direction_T direction)
// {
//     bool isSuccess = (Vehicle_User_GetDirection(p_vehicle) == direction);
//     if (isSuccess == false) { Blinky_Blink(p_vehicle->P_BUZZER, 500U); }
//     return isSuccess;
// }



