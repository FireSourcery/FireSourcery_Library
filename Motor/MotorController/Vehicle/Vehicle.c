/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Vehicle.c
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#include "Vehicle.h"


void _Vehicle_InitFrom(Vehicle_State_T * p_fields, const Vehicle_Config_T * p_config)
{
    if (p_config != NULL) { p_fields->Config = *p_config; }
}

void Vehicle_Init(const Vehicle_T * p_handle)
{
    _Vehicle_InitFrom(p_handle->P_VEHICLE_STATE, p_handle->P_NVM_CONFIG);
    // StateMachine_Init(&p_handle->STATE_MACHINE);
}



/******************************************************************************/
/*
    Called from StateMachine
    State Mode use full context
*/
/******************************************************************************/
/* alternatively convert input only */
void Vehicle_StartThrottleMode(const Vehicle_T * p_vehicle)
{
    switch (p_vehicle->P_VEHICLE_STATE->Config.ThrottleMode)
    {
        case VEHICLE_THROTTLE_MODE_SPEED:  Motor_Table_ApplyFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_SPEED_CURRENT);     break;
        case VEHICLE_THROTTLE_MODE_TORQUE: Motor_Table_ApplyFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);           break;
        default: break;
    }

    /* alternatively from Release only */
    Motor_Table_ActivateVOutput(&p_vehicle->MOTORS, PHASE_OUTPUT_VPWM);
}

/* if handle each value update with statemachine on input */
void Vehicle_SetThrottleValue(const Vehicle_T * p_vehicle, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;

    switch (p_vehicle->P_VEHICLE_STATE->Config.ThrottleMode)
    {
        case VEHICLE_THROTTLE_MODE_SPEED:  Motor_Table_SetCmdWith(&p_vehicle->MOTORS, Motor_SetSpeedCmd_Scalar, (int32_t)cmdValue);     break;
        case VEHICLE_THROTTLE_MODE_TORQUE: Motor_Table_SetCmdWith(&p_vehicle->MOTORS, Motor_SetICmd_Scalar, (int32_t)cmdValue);         break;
        default: break;
    }
}

void Vehicle_ProcThrottleValue(const Vehicle_T * p_vehicle)
{
    Vehicle_SetThrottleValue(p_vehicle, p_vehicle->P_VEHICLE_STATE->Input.ThrottleValue);
}

// apply hold on low speed
void Vehicle_StartBrakeMode(const Vehicle_T * p_vehicle)
{
    switch (p_vehicle->P_VEHICLE_STATE->Config.BrakeMode)
    {
        case VEHICLE_BRAKE_MODE_TORQUE:  Motor_Table_ApplyFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);  break;
            // case VEHICLE_BRAKE_MODE_VOLTAGE: Motor_Table_ApplyFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_VOLTAGE);  break;
        default: break;
    }

    /* alternatively from Release only */
    Motor_Table_ActivateVOutput(&p_vehicle->MOTORS, PHASE_OUTPUT_VPWM);
}

/*!
    Always request opposite direction current
    req opposite iq, bound vq to 0 for no plugging brake

    transition from accelerating to decelerating,
    use signed ramp to transition through 0 without discontinuity
    ramp from in-direction torque to 0 to counter-direction torque

    @param[in] brake [0:32767]
*/
void Vehicle_SetBrakeValue(const Vehicle_T * p_vehicle, uint16_t userCmdBrake)
{
    int16_t cmdValue = 0 - ((int32_t)userCmdBrake / 2); // 32767 max

    switch (p_vehicle->P_VEHICLE_STATE->Config.BrakeMode)
    {
        case VEHICLE_BRAKE_MODE_TORQUE: Motor_Table_SetCmdWith(&p_vehicle->MOTORS, Motor_SetICmd_Scalar, cmdValue); break;
            // case VEHICLE_BRAKE_MODE_VOLTAGE: Motor_Table_SetCmdWith(&p_vehicle->MOTORS, Motor_SetRegenCmd, 0); break;
        default: break;
    }
}

void Vehicle_ProcBrakeValue(const Vehicle_T * p_vehicle)
{
    Vehicle_SetBrakeValue(p_vehicle, p_vehicle->P_VEHICLE_STATE->Input.BrakeValue);
}

/* an alternate cmd for float is required */
void Vehicle_StartDriveZero(const Vehicle_T * p_vehicle)
{
    switch (p_vehicle->P_VEHICLE_STATE->Config.ZeroMode)
    {
        case VEHICLE_ZERO_MODE_FLOAT:     Motor_Table_ActivateVOutput(&p_vehicle->MOTORS, PHASE_OUTPUT_FLOAT);        break;
        case VEHICLE_ZERO_MODE_CRUISE:    Motor_Table_ApplyFeedbackMode(&p_vehicle->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);    break;
        case VEHICLE_ZERO_MODE_REGEN:       /* Vehicle_SetRegenMotorAll(p_this); */ break;
        default: break;
    }
}

/*
    Check Stop / Zero Throttle
    Eventually release for stop transition
*/
void Vehicle_ProcDriveZero(const Vehicle_T * p_vehicle)
{
    switch (p_vehicle->P_VEHICLE_STATE->Config.ZeroMode)
    {
        case VEHICLE_ZERO_MODE_FLOAT: break;
        case VEHICLE_ZERO_MODE_CRUISE: Motor_Table_SetCmdWith(&p_vehicle->MOTORS, Motor_SetICmd_Scalar, 0); break;
        case VEHICLE_ZERO_MODE_REGEN: break;
        default: break;
    }
}


/******************************************************************************/
/*
    Var Id
*/
/******************************************************************************/
int Vehicle_ConfigId_Get(const Vehicle_State_T * p_vehicleState, Vehicle_ConfigId_T id)
{
    int value = 0;
    switch (id)
    {
        case VEHICLE_CONFIG_THROTTLE_MODE:    value = p_vehicleState->Config.ThrottleMode;                 break;
        case VEHICLE_CONFIG_BRAKE_MODE:       value = p_vehicleState->Config.BrakeMode;                    break;
        case VEHICLE_CONFIG_ZERO_MODE:        value = p_vehicleState->Config.ZeroMode;                     break;
    }
    return value;
}

void Vehicle_ConfigId_Set(Vehicle_State_T * p_vehicleState, Vehicle_ConfigId_T id, int value)
{
    switch (id)
    {
        case VEHICLE_CONFIG_THROTTLE_MODE:    p_vehicleState->Config.ThrottleMode = (Vehicle_ThrottleMode_T)value;     break;
        case VEHICLE_CONFIG_BRAKE_MODE:       p_vehicleState->Config.BrakeMode = (Vehicle_BrakeMode_T)value;           break;
        case VEHICLE_CONFIG_ZERO_MODE:        p_vehicleState->Config.ZeroMode = (Vehicle_ZeroMode_T)value;             break;
        default: break;
    }
}



// void Vehicle_VarId_Set(const Vehicle_T * p_vehicle, Vehicle_VarId_T id, int value)
// {
//     switch (id)
//     {
//         case VEHICLE_VAR_DIRECTION:   Vehicle_User_ApplyDirection(p_vehicle, (sign_t)value);       break; // call outer passthrough in most cases
//         case VEHICLE_VAR_THROTTLE:    Vehicle_User_ApplyThrottle(p_vehicle, (uint16_t)value);      break;
//         case VEHICLE_VAR_BRAKE:       Vehicle_User_ApplyBrake(p_vehicle, (uint16_t)value);         break;
//     }
// }

// int Vehicle_VarId_Get(const Vehicle_T * p_vehicle, Vehicle_VarId_T id)
// {
//     int value = 0;
//     switch (id)
//     {
//         case VEHICLE_VAR_DIRECTION:   value = Vehicle_StateMachine_GetDirection(p_vehicle);     break;
//         case VEHICLE_VAR_THROTTLE:    value = p_vehicle->P_VEHICLE_STATE->Input.ThrottleValue;  break;
//         case VEHICLE_VAR_BRAKE:       value = p_vehicle->P_VEHICLE_STATE->Input.BrakeValue;     break;
//     }
//     return value;
// }



// alternatively as input conversion,

// interface for   data common

// static inline void Vehicle_Input_FromAnalogUser(Vehicle_Input_T * p_user, MotAnalogUser_T * P_analog)
// {
//     p_user->ThrottleValue = MotAnalogUser_GetAInValue(P_analog, MOT_ANALOG_USER_AIN_THROTTLE);
//     p_user->BrakeValue = MotAnalogUser_GetAInValue(P_analog, MOT_ANALOG_USER_AIN_BRAKE);
// }

// void Vehicle_Input_ToMotorInput (const Vehicle_Input_T * p_user, Motor_Input_T * P_input )
// {
    //     switch (dir)
    //     {
    //         case MOTOR_CONTROLLER_DIRECTION_PARK:
    //             P_input->Direction = MOTOR_DIRECTION_NULL;
    //             P_input->PhaseOutput = PHASE_OUTPUT_FLOAT;
    //             break;
    //         case MOTOR_CONTROLLER_DIRECTION_REVERSE:
    //             P_input->Direction = MOTOR_DIRECTION_CW;
    //             P_input->PhaseOutput = PHASE_OUTPUT_VPWM;
    //             break;
    //         case MOTOR_CONTROLLER_DIRECTION_FORWARD:
    //             P_input->Direction = MOTOR_DIRECTION_CCW;
    //             P_input->PhaseOutput = PHASE_OUTPUT_VPWM;
    //             break;
    //         case MOTOR_CONTROLLER_DIRECTION_NEUTRAL:
    //             P_input->PhaseOutput = PHASE_OUTPUT_FLOAT;
    //             break;
    //         default:
    //             break;
    //     }
//
// }

// static inline void Input_FromThrottle(Motor_Input_T * p_user, Vehicle_ThrottleMode_T mode, uint16_t throttle)
// {
//     p_user->CmdValue = (int32_t)throttle / 2; // [0:32767]

//     switch (mode)
//     {
//         case VEHICLE_THROTTLE_MODE_SPEED:   p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_SPEED_CURRENT;      break;
//         case VEHICLE_THROTTLE_MODE_TORQUE:  p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT;            break;
//         default: break;
//     }
// }

// static inline void Vehicle_Input_FromProtocol(Vehicle_T * vehicle, Motor_Input_T * p_user, id, value)

// static inline void Vehicle_Input_FromAnalogUser(Vehicle_T * vehicle, Motor_Input_T * p_user, MotAnalogUser_T * P_analog)
// {
//     p_user->CmdValue = (int32_t)MotAnalogUser_GetThrottle(P_analog) / 2; // [0:32767]

//     switch (vehicle->P_VEHICLE_STATE->Config.ThrottleMode)
//     {
//         case VEHICLE_THROTTLE_MODE_SPEED:   p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_SPEED_CURRENT;      break;
//         case VEHICLE_THROTTLE_MODE_TORQUE:  p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT;            break;
//         default: break;
//     }
// }
