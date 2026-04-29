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

#include "../../Motor_FOC.h"
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
/*
    Phase_Id_T and Hall_VirtualId_T share numeric values - one table drives both.
    Sequence: 6 vectors at 60deg electrical; conventional Align -> Settle -> Read.
    Align runs every tick (current PID closes loop); timer paces vector advance.
*/
static_assert(HALL_SENSORS_VIRTUAL_A     == PHASE_ID_A);
static_assert(HALL_SENSORS_VIRTUAL_INV_C == PHASE_ID_INV_C);
static_assert(HALL_SENSORS_VIRTUAL_B     == PHASE_ID_B);
static_assert(HALL_SENSORS_VIRTUAL_INV_A == PHASE_ID_INV_A);
static_assert(HALL_SENSORS_VIRTUAL_C     == PHASE_ID_C);
static_assert(HALL_SENSORS_VIRTUAL_INV_B == PHASE_ID_INV_B);


#define CAL_STEP_COUNT    (6U)

/* Include [Phase] and [P_PARENT] State */
static void Calibration_Entry(const Motor_T * p_motor)
{
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles);
    Phase_ActivateV0(&p_motor->PHASE);
    Hall_StartCalibrate(GetHall(p_motor));
    PID_Reset(&p_motor->P_MOTOR->PidId);
    Ramp_SetOutputState(&p_motor->P_MOTOR->TorqueRamp, 0);
    Ramp_SetOutputLimit(&p_motor->P_MOTOR->TorqueRamp, 0, Motor_GetIAlign(&p_motor->P_MOTOR->Config));
    Motor_SetFeedbackMode(p_motor->P_MOTOR, MOTOR_FEEDBACK_MODE_CURRENT);
    p_motor->P_MOTOR->CalibrationStateIndex = 0U;
}

/*
    Voltage-mode align: open-loop fixed duty (one-shot per call is sufficient).
*/
static void Calibration_Align_V(const Motor_T * p_motor, Phase_Id_T id)
{
    Phase_Align(&p_motor->PHASE, id, Motor_GetVAlign_Duty(&p_motor->P_MOTOR->Config));
}

/*
    Current-mode align: PID Id -> Vd, projected onto vector angle via FOC chain.
    Must run every tick to close the current loop. Torque ramp ramps Id from 0
    up to IAlign across the settle window, avoiding a current step at each new
    vector; ramp is reset to 0 at every vector boundary.
*/
static void Calibration_Align_I(const Motor_T * p_motor, Phase_Id_T id)
{
    const fract16_t idReq = Motor_OpenLoopTorqueRampOf(p_motor->P_MOTOR, Motor_GetIAlign(&p_motor->P_MOTOR->Config));
    Motor_FOC_AngleControl(p_motor->P_MOTOR, Phase_AngleOf(id), idReq, 0);
    Motor_FOC_WriteDuty(p_motor);
}

/*
    Drive each tick (current loop runs continuously across the settle window),
    sample hall and advance vector at the boundary, reset ramp for next vector.
*/
static void Calibration_Proc(const Motor_T * p_motor)
{
    assert(p_motor->P_MOTOR->CalibrationStateIndex < CAL_STEP_COUNT);

    static const Phase_Id_T CAL_STEP[CAL_STEP_COUNT] = { PHASE_ID_A, PHASE_ID_INV_C, PHASE_ID_B, PHASE_ID_INV_A, PHASE_ID_C, PHASE_ID_INV_B, };
    Phase_Id_T id = CAL_STEP[p_motor->P_MOTOR->CalibrationStateIndex];

    Calibration_Align_I(p_motor, id);

    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        Hall_CalibrateState(GetHall(p_motor), (Hall_Id_T)id);
        Ramp_SetOutputState(&p_motor->P_MOTOR->TorqueRamp, 0);
        p_motor->P_MOTOR->CalibrationStateIndex++;
    }
}

static State_T * Calibration_End(const Motor_T * p_motor)
{
    if (p_motor->P_MOTOR->CalibrationStateIndex >= CAL_STEP_COUNT)
    {
        Phase_Deactivate(&p_motor->PHASE);
        p_motor->P_MOTOR->FaultFlags.PositionSensor = !Hall_IsTableValid(GetHall(p_motor)->P_STATE);
        return &MOTOR_STATE_CALIBRATION;
    }
    return NULL;
}

static const State_T CALIBRATION_STATE_HALL =
{
    .ID         = 0U,
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)Calibration_Entry,
    .LOOP       = (State_Action_T)Calibration_Proc,
    .NEXT       = (State_Input0_T)Calibration_End,
};


/******************************************************************************/
/*!

*/
/******************************************************************************/
static State_T * Calibration_Start(const Motor_T * p_motor, state_value_t value) { (void)p_motor; (void)value; return &CALIBRATION_STATE_HALL; }

void Motor_Hall_Calibrate(const Motor_T * p_motor)
{
    // StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE_HALL);
    static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Calibration_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

void Motor_Hall_Cmd(const Motor_T * p_motor, int varId, int varValue)
{
    (void)varValue;
    if (!RotorSensor_Validate(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR->p_ActiveSensor, ROTOR_SENSOR_ID_HALL)) return;
    if (p_motor->P_MOTOR->Config.SensorMode != ROTOR_SENSOR_ID_HALL) return;

    switch (varId)
    {
        case 0:   Motor_Hall_Calibrate(p_motor); break;
        default:  Motor_Hall_Calibrate(p_motor); break;
    }
}

// void Motor_Hall_GetStateVar(const Motor_T * p_motor, int varId, int varValue)
// {
//     if (!RotorSensor_Validate(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR->p_ActiveSensor, ROTOR_SENSOR_ID_HALL)) return;
//     if (p_motor->P_MOTOR->Config.SensorMode != ROTOR_SENSOR_ID_HALL) return;

//     switch (varId)
//     {
//         case HALL_PULSE_RPM:   Hall_VarId_Get(p_motor); break;
//     }
// }



// static void Calibration_Proc(const Motor_T * p_motor)
// {
//     if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) != true) return;

//     const uint16_t duty = Motor_GetVAlign_Duty(&p_motor->P_MOTOR->Config);

//     switch (p_motor->P_MOTOR->CalibrationStateIndex)
//     {
//         case CAL_STEP_ALIGN_A:      Phase_Align(&p_motor->PHASE, PHASE_ID_A, duty); break;
//         case CAL_STEP_READ_A:       Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_A);     break;

//         case CAL_STEP_ALIGN_INV_C:  Phase_Align(&p_motor->PHASE, PHASE_ID_INV_C, duty); break;
//         case CAL_STEP_READ_INV_C:   Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_INV_C); break;

//         case CAL_STEP_ALIGN_B:      Phase_Align(&p_motor->PHASE, PHASE_ID_B, duty); break;
//         case CAL_STEP_READ_B:       Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_B);     break;

//         case CAL_STEP_ALIGN_INV_A:  Phase_Align(&p_motor->PHASE, PHASE_ID_INV_A, duty); break;
//         case CAL_STEP_READ_INV_A:   Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_INV_A); break;

//         case CAL_STEP_ALIGN_C:      Phase_Align(&p_motor->PHASE, PHASE_ID_C, duty); break;
//         case CAL_STEP_READ_C:       Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_C);     break;

//         case CAL_STEP_ALIGN_INV_B:  Phase_Align(&p_motor->PHASE, PHASE_ID_INV_B, duty); break;
//         case CAL_STEP_READ_INV_B:   Hall_CalibrateState(GetHall(p_motor), HALL_SENSORS_VIRTUAL_INV_B); break;

//         case CAL_STEP_DONE:         Phase_Deactivate(&p_motor->PHASE); break;
//         default: break;
//     }

//     p_motor->P_MOTOR->CalibrationStateIndex++;
// }


