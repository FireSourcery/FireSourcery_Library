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
    @file   Motor_Traction.c
    @author FireSourcery
    @brief  Traction onto one Motor. Mapping summary in [Motor_Traction.h]
*/
/******************************************************************************/
#include "Motor_Traction.h"
#include "Motor/Motor/Motor_User.h"


/* alternatively convert input only */
void Motor_Traction_StartThrottleMode(Motor_T * p_motor, const Traction_T * p_traction)
{
    switch (p_traction->Config.ThrottleMode)
    {
        case TRACTION_THROTTLE_MODE_SPEED:  Motor_ApplyFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT);     break;
        case TRACTION_THROTTLE_MODE_TORQUE: Motor_ApplyFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT);           break;
        default: Motor_ApplyControlState(p_motor, PHASE_VOUT_Z); return; /* VOLTAGE: no cmd path. Release, never drive the last cmd */
    }

    /* alternatively from Release only */
    Motor_ApplyControlState(p_motor, PHASE_VOUT_PWM);
}

// void Motor_Traction_ApplyThrottleSpeedValue(Motor_T * p_motor, const Traction_T * p_traction, uint16_t userCmdThrottle)
// {
//     Motor_SetSpeedMotoringCmd_Norm(p_motor->P_MOTOR, (int32_t)userCmdThrottle / 2);
// }

/* if handle each value update with statemachine on input */
void Motor_Traction_ApplyThrottleValue(Motor_T * p_motor, const Traction_T * p_traction, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;

    switch (p_traction->Config.ThrottleMode)
    {
        case TRACTION_THROTTLE_MODE_SPEED:  Motor_SetSpeedMotoringCmd_Norm(p_motor->P_MOTOR, cmdValue);    break;
        case TRACTION_THROTTLE_MODE_TORQUE: Motor_SetICmd_Norm(p_motor->P_MOTOR, cmdValue);                break;
        default: break;
    }
}



/*!
    User Forward frame, independent of the drive direction.
    Transition through zero speed is resolved by the Motor.
    @param[in] lever [-32768:32767]
*/
void Motor_Traction_ApplyLeverValue(Motor_T * p_motor, const Traction_T * p_traction, int16_t lever)
{
    switch (p_traction->Config.ThrottleMode)
    {
        case TRACTION_THROTTLE_MODE_SPEED:  Motor_SetSpeedCmd_Norm(p_motor->P_MOTOR, lever);     break;
        case TRACTION_THROTTLE_MODE_TORQUE: Motor_SetTorqueCmd_Norm(p_motor->P_MOTOR, lever);    break;
        default: break;
    }
}

void Motor_Traction_StartBrakeMode(Motor_T * p_motor, const Traction_T * p_traction)
{
    switch (p_traction->Config.BrakeMode)
    {
        case TRACTION_BRAKE_MODE_TORQUE:
            Motor_ApplyFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT);
            Motor_ApplyControlState(p_motor, PHASE_VOUT_PWM);     /* alternatively from Release only */
            break;
            // case TRACTION_BRAKE_MODE_VOLTAGE: Motor_ApplyFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_VOLTAGE);  break;
        case TRACTION_BRAKE_MODE_PASSIVE: /* Motors stay released */
        default: Motor_ApplyControlState(p_motor, PHASE_VOUT_Z); break; /* VOLTAGE: no cmd path. Release, never drive the last cmd */
    }
}

/*!
    Always request opposite direction current
    req opposite iq, bound vq to 0 for no plugging brake

    transition from accelerating to decelerating,
    use signed ramp to transition through 0 without discontinuity
    ramp from in-direction torque to 0 to counter-direction torque

    @param[in] brake [0:32767]
*/
void Motor_Traction_ApplyBrakeValue(Motor_T * p_motor, const Traction_T * p_traction, uint16_t userCmdBrake)
{
    int16_t cmdValue = 0 - ((int32_t)userCmdBrake / 2); // 32767 max

    switch (p_traction->Config.BrakeMode)
    {
        case TRACTION_BRAKE_MODE_TORQUE: Motor_SetICmd_Norm(p_motor->P_MOTOR, cmdValue); break; /* note: torqueRamp also written in voltage mode */
            // case TRACTION_BRAKE_MODE_VOLTAGE: Motor_SetRegenCmd(p_motor->P_MOTOR, 0); break;
        default: break;
    }
}

/*
    Opposing torque cannot hold standstill - anti-plugging clamps it to short-circuit braking once speed crosses zero.
    Hold zero speed. The caller decides when: once no motor advances, a decision over every motor.
    Repeat calls while holding are no-op.
    Exits: Throttle resumes Run. Release by ZeroMode.
*/
void Motor_Traction_ApplyBrakeHold(Motor_T * p_motor, const Traction_T * p_traction)
{
    switch (p_traction->Config.BrakeMode)
    {
        case TRACTION_BRAKE_MODE_TORQUE: Motor_ApplySpeedZero(p_motor); break;
        default: break;
    }
}

/* an alternate cmd for float is required */
void Motor_Traction_StartDriveZero(Motor_T * p_motor, const Traction_T * p_traction)
{
    switch (p_traction->Config.ZeroMode)
    {
        case TRACTION_ZERO_MODE_IZERO:   Motor_ApplyTorqueZero(p_motor);                                break;
        case TRACTION_ZERO_MODE_ZERO:    Motor_Traction_ApplyThrottleValue(p_motor, p_traction, 0U);     break; /* The release edge carries no value */
        case TRACTION_ZERO_MODE_REGEN:   /* Traction_SetRegenMotorAll(p_this); not implemented, float */
        case TRACTION_ZERO_MODE_FLOAT:
        default: Motor_ApplyControlState(p_motor, PHASE_VOUT_Z); break;
    }
}

// optionally select speed ramp down
void Motor_Traction_ProcRelease(Motor_T * p_motor, const Traction_T * p_traction)
{
    (void)p_motor; (void)p_traction;
    // Motor_Hypervisor(p_motor);
}
