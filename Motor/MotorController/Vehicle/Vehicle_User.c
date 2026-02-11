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
    @file   Vehicle_User.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Vehicle_User.h"


/*
    Command Polling
    Cmd "state" == input, changes on edge, handle outside of StateMachine
*/
void Vehicle_User_PollStartCmd(Vehicle_T * p_vehicle)
{
    Vehicle_Input_T * p_input = &p_vehicle->P_VEHICLE_STATE->Input;
    if (Vehicle_Input_PollCmdEdge(p_input)) { Vehicle_User_ApplyStartCmd(p_vehicle, p_input->Cmd); }
}

/*
    Input ~10-50ms
    Proc State/Buffer ~1ms
    Apply state changes immediately
    on protocol input per value response, 50ms. alternatively buffer and poll.
    alternatively call value update,
*/
void Vehicle_User_ApplyThrottle(Vehicle_T * p_vehicle, uint16_t userCmd)
{
    p_vehicle->P_VEHICLE_STATE->Input.ThrottleValue = userCmd;
    Vehicle_User_PollStartCmd(p_vehicle);
    // Vehicle_User_ApplyCmd(p_vehicle, p_input->Cmd);  /* statemachine handle check edge, + set value */
}

void Vehicle_User_ApplyBrake(Vehicle_T * p_vehicle, uint16_t userCmd)
{
    p_vehicle->P_VEHICLE_STATE->Input.BrakeValue = userCmd;
    Vehicle_User_PollStartCmd(p_vehicle);
}

void Vehicle_User_ApplyZero(Vehicle_T * p_vehicle)
{
    p_vehicle->P_VEHICLE_STATE->Input.ThrottleValue = 0U;
    p_vehicle->P_VEHICLE_STATE->Input.BrakeValue = 0U;
    Vehicle_User_PollStartCmd(p_vehicle);
}


/* or move this to app interface layer */
/******************************************************************************/
/*
    Var Id
*/
/******************************************************************************/
void Vehicle_VarId_Set(const Vehicle_T * p_vehicle, Vehicle_VarId_T id, int value)
{
    switch (id)
    {
        case VEHICLE_VAR_DIRECTION:   Vehicle_User_ApplyDirection(p_vehicle, (sign_t)value);       break; // call outer passthrough in most cases
        case VEHICLE_VAR_THROTTLE:    Vehicle_User_ApplyThrottle(p_vehicle, (uint16_t)value);      break;
        case VEHICLE_VAR_BRAKE:       Vehicle_User_ApplyBrake(p_vehicle, (uint16_t)value);         break;
    }
}

int Vehicle_VarId_Get(const Vehicle_T * p_vehicle, Vehicle_VarId_T id)
{
    int value = 0;
    switch (id)
    {
        case VEHICLE_VAR_DIRECTION:   value = Vehicle_StateMachine_GetDirection(p_vehicle);     break;
        case VEHICLE_VAR_THROTTLE:    value = p_vehicle->P_VEHICLE_STATE->Input.ThrottleValue;  break;
        case VEHICLE_VAR_BRAKE:       value = p_vehicle->P_VEHICLE_STATE->Input.BrakeValue;     break;
    }
    return value;
}


/******************************************************************************/
/* */
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
