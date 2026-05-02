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
    @file   MotorController_Traction.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Traction.h"

#include "../MotorController_App.h"
#include "../MotorController_StateMachine.h"


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
            └─ TRACTION (Specialized Level)
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
typedef enum Traction_StateInput
{
    TRACTION_STATE_INPUT_DIRECTION,      /* Drive Direction */
    TRACTION_STATE_INPUT_DRIVE_CMD,      /* On Edge, Drive Cmd -Throttle/Brake/Release */
    TRACTION_STATE_INPUT_THROTTLE_VALUE,
    TRACTION_STATE_INPUT_BRAKE_VALUE,
}
Traction_StateInput_T;

typedef enum Traction_StateId
{
    TRACTION_STATE_ID_DRIVE /* = _MC_STATE_ID_END */,
    TRACTION_STATE_ID_NEUTRAL,
}
Traction_StateId_T;

// extern State_T MC_STATE_MAIN_TRACTION;

extern const MotorController_App_T MC_APP_TRACTION;


/******************************************************************************/
/*!

*/
/******************************************************************************/
/* MotorController.c */
static inline Traction_T * MotorController_Traction_Cast(MotorController_T * p_mc) { return (Traction_T *)(p_mc->P_APP_STATE); }
static inline const Traction_Config_T * MotorController_Traction_CastNvm(MotorController_T * p_mc) { return (const Traction_Config_T *)(p_mc->P_APP_NVM_CONFIG); }


extern void MotorController_Traction_PollStartCmd(MotorController_T * p_mc);
extern void MotorController_Traction_SetThrottle(MotorController_T * p_mc, uint16_t userCmd);
extern void MotorController_Traction_SetBrake(MotorController_T * p_mc, uint16_t userCmd);
extern void MotorController_Traction_SetRelease(MotorController_T * p_mc);
extern void MotorController_Traction_SetThrottleBrake(MotorController_T * p_mc, uint16_t throttle, uint16_t brake);

extern void MotorController_Traction_ApplyDirectionCmd(MotorController_T * p_mc, sign_t direction);
extern void MotorController_Traction_CaptureDirection(MotorController_T * p_mc, sign_t direction);

extern void MotorController_Traction_ProcAnalogUser(MotorController_T * p_mc);
extern sign_t MotorController_Traction_GetDirection(MotorController_T * p_mc);

/*

*/
extern void MotorController_Traction_VarId_Set(MotorController_T * p_mc, Traction_VarId_T id, int value);
extern int MotorController_Traction_VarId_Get(MotorController_T * p_mc, Traction_VarId_T id);

int MotorController_Traction_ConfigId_Get(MotorController_T * p_mc, Traction_ConfigId_T id);
void MotorController_Traction_ConfigId_Set(MotorController_T * p_mc, Traction_ConfigId_T id, int value);




/* StateMachine Input */
// extern void MotorController_Traction_ApplyDirection(MotorController_T * p_mc, sign_t direction);
// extern void MotorController_Traction_ApplyStartCmd(MotorController_T * p_mc, Traction_Cmd_T cmd);
// extern void MotorController_Traction_StartThrottle(MotorController_T * p_mc);
// extern void MotorController_Traction_StartBrake(MotorController_T * p_mc);
// extern void MotorController_Traction_StartRelease(MotorController_T * p_mc);