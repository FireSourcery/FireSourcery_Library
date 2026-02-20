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
    @file   MotorController_Vehicle.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Vehicle.h"

#include "../MotorController_App.h"
#include "Utility/StateMachine/StateMachine.h"

/* Part of MotorController */
/* Let AppTable include so Var can include without explicitly listing all app headers*/
struct MotorController;
typedef const struct MotorController MotorController_T;

/******************************************************************************/
/*
    State Machine
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
    VEHICLE_STATE_INPUT_DIRECTION = STATE_INPUT_MAPPER_START_ID, // = STATE_INPUT_MAPPER_ID_START(MCSM_INPUT_APP_USER),   /* Drive Direction */
    VEHICLE_STATE_INPUT_DRIVE_CMD,              /* On Edge, Drive Cmd -Throttle/Brake/Release */
}
Vehicle_State_Input_T;

typedef enum Vehicle_StateId
{
    VEHICLE_STATE_ID_DRIVE,
    VEHICLE_STATE_ID_NEUTRAL,
}
Vehicle_StateId_T;

extern State_T MC_STATE_MAIN_VEHICLE;

extern MotorController_App_T MC_APP_VEHICLE;


/******************************************************************************/
/*!

*/
/******************************************************************************/

/* StateMachine Input */
extern void MotorController_Vehicle_ApplyDirection(MotorController_T * p_mc, sign_t direction);
extern void MotorController_Vehicle_ApplyStartCmd(MotorController_T * p_mc, Vehicle_Cmd_T cmd);
extern void MotorController_Vehicle_StartThrottle(MotorController_T * p_mc);
extern void MotorController_Vehicle_StartBrake(MotorController_T * p_mc);
extern void MotorController_Vehicle_StartRelease(MotorController_T * p_mc);

/*  */
extern void MotorController_Vehicle_PollStartCmd(MotorController_T * p_mc);
extern void MotorController_Vehicle_PollThrottle(MotorController_T * p_mc, uint16_t userCmd);
extern void MotorController_Vehicle_PollBrake(MotorController_T * p_mc, uint16_t userCmd);
extern void MotorController_Vehicle_PollRelease(MotorController_T * p_mc);

extern void MotorController_Vehicle_ProcAnalogUser(MotorController_T * p_mc);
extern sign_t MotorController_Vehicle_GetDirection(MotorController_T * p_mc);

extern void MotorController_Vehicle_VarId_Set(MotorController_T * p_mc, Vehicle_VarId_T id, int value);
extern int MotorController_Vehicle_VarId_Get(MotorController_T * p_mc, Vehicle_VarId_T id);



