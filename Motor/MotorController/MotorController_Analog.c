
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
    @file   MotorController_Analog.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotorController_Analog.h"


/******************************************************************************/
/*!
    @brief CalibrateAdc SubState
*/
/******************************************************************************/
void StartCalibrateAdc(MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;
    p_mc->StateCounter = 0U;
    Analog_Conversion_Mark(&p_dev->ANALOG_USER_CONVERSIONS.THROTTLE);
    Analog_Conversion_Mark(&p_dev->ANALOG_USER_CONVERSIONS.BRAKE);
    Analog_Conversion_ClearResult(&p_dev->ANALOG_USER_CONVERSIONS.THROTTLE);
    Analog_Conversion_ClearResult(&p_dev->ANALOG_USER_CONVERSIONS.BRAKE);
    Accumulator_Init(&p_mc->AvgBuffer0);
    Accumulator_Init(&p_mc->AvgBuffer1);
    // Motor_Table_EnterCalibrateAdc(&p_dev->MOTORS); /* Motor handles it own state */
    // p_mc->LockOpStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_PROCESSING;
}

/* Proc Per ms */
void ProcCalibrateAdc(MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    if (p_mc->StateCounter != 0U) /* skip first time */
    {
        Accumulator_Avg(&p_mc->AvgBuffer0, Analog_Conversion_GetResult(&p_dev->ANALOG_USER_CONVERSIONS.THROTTLE));
        Accumulator_Avg(&p_mc->AvgBuffer1, Analog_Conversion_GetResult(&p_dev->ANALOG_USER_CONVERSIONS.BRAKE));
        Analog_Conversion_Mark(&p_dev->ANALOG_USER_CONVERSIONS.THROTTLE);
        Analog_Conversion_Mark(&p_dev->ANALOG_USER_CONVERSIONS.BRAKE);
    }

    p_mc->StateCounter++;
}

static State_T * EndCalibrateAdc(MotorController_T * p_dev)
{
    const uint32_t TIME = 2000U; /* > Motor calibrate adc time */

    MotorController_State_T * p_mc = p_dev->P_MC;
    State_T * p_nextState = NULL;

    if (p_mc->StateCounter > TIME)
    {
        MotAnalogUser_SetThrottleZero(&p_dev->ANALOG_USER, Accumulator_Avg(&p_mc->AvgBuffer0, Analog_Conversion_GetResult(&p_dev->ANALOG_USER_CONVERSIONS.THROTTLE)));
        MotAnalogUser_SetBrakeZero(&p_dev->ANALOG_USER, Accumulator_Avg(&p_mc->AvgBuffer1, Analog_Conversion_GetResult(&p_dev->ANALOG_USER_CONVERSIONS.BRAKE)));
        p_mc->LockOpStatus = 0; /* success */

        p_nextState = &MC_STATE_LOCK; /* return to lock state */
    }

    return p_nextState;
}

const State_T MC_STATE_LOCK_CALIBRATE_ADC =
{
    .ID = MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC, // valid during subsstae only
    .P_TOP = &MC_STATE_LOCK,
    .P_PARENT = &MC_STATE_LOCK,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)StartCalibrateAdc,
    .LOOP = (State_Action_T)ProcCalibrateAdc,
    .NEXT = (State_Input0_T)EndCalibrateAdc,
};



static inline State_T * Lock_CalibrateAdc(const MotorController_T * p_dev, state_value_t lockId)
{
    return &MC_STATE_LOCK_CALIBRATE_ADC;
}

void MotorController_Lock_CalibrateAdc(const MotorController_T * p_dev)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MC_STATE_LOCK, .NEXT = (State_Input_T)Lock_CalibrateAdc, };
    StateMachine_Tree_InvokeTransition(&p_dev->STATE_MACHINE, &CMD, 0);
}
