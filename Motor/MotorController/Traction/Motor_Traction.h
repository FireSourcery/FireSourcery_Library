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
    @file   Motor_Traction.h
    @author FireSourcery
    @brief  Adapter mapping Traction onto one Motor, through the Motor's own interface.
            The MotorController Traction states run these over the motor table, and keep the decisions that read every motor.
            Form chosen over a common Motor_Input_T: Docs/Decisions/0013_TractionMotorAdapter.md

    Mapping summary, by [Traction_Config_T]:
      Throttle start      → SPEED, TORQUE: Motor_ApplyFeedbackMode (SPEED_CURRENT, CURRENT), then Motor_ApplyControlState PWM
                            VOLTAGE: Motor_ApplyControlState Z
      Throttle value      → SPEED: Motor_SetSpeedMotoringCmd_Norm, TORQUE: Motor_SetICmd_Norm, value / 2
      Lever value         → SPEED: Motor_SetSpeedCmd_Norm, TORQUE: Motor_SetTorqueCmd_Norm, signed, user Forward frame
      Brake start         → TORQUE: Motor_ApplyFeedbackMode CURRENT, then Motor_ApplyControlState PWM
                            PASSIVE, VOLTAGE: Motor_ApplyControlState Z
      Brake value         → TORQUE: Motor_SetICmd_Norm, minus value / 2
      Brake hold          → TORQUE: Motor_ApplySpeedZero. When, is the caller's
      Release (ZeroMode)  → IZERO: Motor_ApplyTorqueZero; ZERO: throttle value 0; FLOAT, REGEN: Motor_ApplyControlState Z

    Two layers, as Motor_Cia402:
      Outer   Motor_Traction_*(Motor_T *, const Traction_T *), one signature. The values come from [Traction_Input_T].
      Inner   _Traction_Apply*(Motor_T *, mode, value), the one config field read and the value. For a value that is not the input's.
*/
/******************************************************************************/
#include "Traction.h"
#include "Motor/Motor/Motor_User.h"


/******************************************************************************/
/*
    Protected
*/
/******************************************************************************/
extern void _Traction_ApplyThrottleValue(Motor_T * p_motor, Traction_ThrottleMode_T mode, uint16_t userCmdThrottle);
extern void _Traction_ApplyLeverValue(Motor_T * p_motor, Traction_ThrottleMode_T mode, int16_t lever);
extern void _Traction_ApplyBrakeValue(Motor_T * p_motor, Traction_BrakeMode_T mode, uint16_t userCmdBrake);

/******************************************************************************/
/*
    Public
*/
/******************************************************************************/
extern void Motor_Traction_StartThrottleMode(Motor_T * p_motor, const Traction_T * p_traction);
extern void Motor_Traction_ApplyThrottleValue(Motor_T * p_motor, const Traction_T * p_traction);
extern void Motor_Traction_ApplyLeverValue(Motor_T * p_motor, const Traction_T * p_traction);
extern void Motor_Traction_StartBrakeMode(Motor_T * p_motor, const Traction_T * p_traction);
extern void Motor_Traction_ApplyBrakeValue(Motor_T * p_motor, const Traction_T * p_traction);
extern void Motor_Traction_ApplyBrakeHold(Motor_T * p_motor, const Traction_T * p_traction);
extern void Motor_Traction_StartDriveZero(Motor_T * p_motor, const Traction_T * p_traction);
extern void Motor_Traction_ProcRelease(Motor_T * p_motor, const Traction_T * p_traction);
