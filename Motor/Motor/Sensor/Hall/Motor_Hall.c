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
    @file   Motor_Hall.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Motor_Hall.h"
#include "Hall.h"

// #include "../../Motor_Config.h"
#include "../../Motor_StateMachine.h"


/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline Hall_T * GetHall(const Motor_T * p_motor) { return &p_motor->SENSOR_TABLE.HALL.HALL; }

/******************************************************************************/
/*!
    Calibration State
*/
/******************************************************************************/
/* Include [Phase] and [P_PARENT] State */
static void Calibration_Entry(const Motor_T * p_motor)
{
    // Timer_StartPeriod(&p_motor->P_MOTOR_STATE->ControlTimer, p_motor->P_MOTOR_STATE->Config.AlignTime_Cycles);
    TimerT_Periodic_Set(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR_STATE->Config.AlignTime_Cycles);
    Phase_ActivateOutputV0(&p_motor->PHASE);
    p_motor->P_MOTOR_STATE->CalibrationStateIndex = 0U;
}

static void Calibration_Proc(const Motor_T * p_motor)
{
    static_assert(HALL_SENSORS_VIRTUAL_A == PHASE_ID_A);
    static_assert(HALL_SENSORS_VIRTUAL_B == PHASE_ID_B);
    static_assert(HALL_SENSORS_VIRTUAL_C == PHASE_ID_C);

    const uint16_t duty = Motor_GetVAlign_Duty(p_motor->P_MOTOR_STATE);

    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        switch (p_motor->P_MOTOR_STATE->CalibrationStateIndex) /* Phase_ReadAlign() */
        {
            case 0U: Hall_StartCalibrate(GetHall(p_motor));                               Phase_Align(&p_motor->PHASE, PHASE_ID_A, duty);        p_motor->P_MOTOR_STATE->CalibrationStateIndex = 1U;    break;
            case 1U: Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_A);       Phase_Align(&p_motor->PHASE, PHASE_ID_INV_C, duty);    p_motor->P_MOTOR_STATE->CalibrationStateIndex = 2U;    break;
            case 2U: Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_INV_C);   Phase_Align(&p_motor->PHASE, PHASE_ID_B, duty);        p_motor->P_MOTOR_STATE->CalibrationStateIndex = 3U;    break;
            case 3U: Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_B);       Phase_Align(&p_motor->PHASE, PHASE_ID_INV_A, duty);    p_motor->P_MOTOR_STATE->CalibrationStateIndex = 4U;    break;
            case 4U: Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_INV_A);   Phase_Align(&p_motor->PHASE, PHASE_ID_C, duty);        p_motor->P_MOTOR_STATE->CalibrationStateIndex = 5U;    break;
            case 5U: Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_C);       Phase_Align(&p_motor->PHASE, PHASE_ID_INV_B, duty);    p_motor->P_MOTOR_STATE->CalibrationStateIndex = 6U;    break;
            case 6U: Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_INV_B);   Phase_Float(&p_motor->PHASE);                          p_motor->P_MOTOR_STATE->CalibrationStateIndex = 7U;    break;
            default: break;
            // case PHASE_ID_A: Hall_CalibrateState(&p_motor->Hall, HALL_SENSORS_VIRTUAL_A); Phase_AlignNext(&p_motor->PHASE, duty); break;
        }
    }
}

// static void Calibration_Proc_IFeedback(const Motor_T * p_motor)
// {
//     uint16_t vPhase = PID_ProcPI(&p_motor->PidId, FOC_GetId(&p_motor->Foc), Motor_GetVAlign_Duty( p_motor)); // use phase pid or align to angle
//     const uint16_t duty = vPhase * Phase_VBus_Inv_Fract32() >> 16;
// }

static State_T * Calibration_End(const Motor_T * p_motor)
{
    bool isComplete = (p_motor->P_MOTOR_STATE->CalibrationStateIndex >= 7U); /* (Phase_ReadAlign(&p_motor->PHASE) == PHASE_ID_0) */
    if (isComplete == true) { p_motor->P_MOTOR_STATE->FaultFlags.PositionSensor = !Hall_IsTableValid(GetHall(p_motor)->P_STATE); }
    return isComplete ? &MOTOR_STATE_CALIBRATION : NULL;
}

static const State_T CALIBRATION_STATE_HALL =
{
    .ID         = 0U,
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)Calibration_Entry,
    .LOOP       = (State_Action_T)Calibration_Proc,
    .NEXT       = (State_InputVoid_T)Calibration_End,
};

/******************************************************************************/
/*!

*/
/******************************************************************************/
void Motor_Hall_Calibrate(const Motor_T * p_motor)
{
    StateMachine_Branch_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE_HALL);
    // StateMachine_Branch_InvokeTransition(&p_motor->STATE_MACHINE, &(const StateMachine_TransitionInput_T) {.P_START = &MOTOR_STATE_CALIBRATION, .TRANSITION = Calibration_Start }, 0);
}

