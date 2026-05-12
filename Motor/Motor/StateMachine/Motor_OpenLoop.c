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
    @file   Motor_OpenLoop.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Motor_OpenLoop.h"

/* part of Motor_StateMachine */


/* StateMachine_Tree_InvokeTransition from any OpenLoop State */

/******************************************************************************/
/*
    Open Loop Cmds - Without SubState
*/
/******************************************************************************/
/*
    sets the phase state without exiting openloop.
*/
static State_T * OpenLoop_PhaseOutput(Motor_T * p_motor, state_value_t phaseState)
{
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR);
    Phase_ActivateVOut(&p_motor->PHASE, (Phase_VOutMode_T)phaseState); // clear outputState V, or enable last duty
    return NULL; /* remain in MOTOR_STATE_OPEN_LOOP*/
}

/* ActivateOutput */
void Motor_OpenLoop_SetPhaseOutput(Motor_T * p_motor, Phase_VOutMode_T phase)
{
    static const StateMachine_TransitionCmd_T OPEN_LOOP_CMD_PHASE = { .P_START = &MOTOR_STATE_OPEN_LOOP, .NEXT = (State_Input_T)OpenLoop_PhaseOutput, };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_PHASE, phase);
}

/*
    alternatively use substate to retain is active and ensure exit other open loop state
    //todo chack again
*/
static State_T * OpenLoop_Jog(Motor_T * p_motor, state_value_t direction)
{
    (void)direction;
    // if (Phase_IsFloat(&p_motor->PHASE) == 0) Phase_ActivateV0(&p_motor->PHASE);
    if (Phase_ReadAlign(&p_motor->PHASE) == 0) { Phase_Align(&p_motor->PHASE, PHASE_ID_A, Motor_GetVAlign_Duty(&p_motor->P_MOTOR->Config)); }
    else
    {
        Angle_CaptureAngle(&p_motor->P_MOTOR->OpenLoopAngle, Phase_AngleOf(Phase_JogNext(&p_motor->PHASE, Motor_GetVAlign_Duty(&p_motor->P_MOTOR->Config))));
    }
    return NULL;
}

void Motor_OpenLoop_SetJog(Motor_T * p_motor, int8_t direction)
{
    static const StateMachine_TransitionCmd_T OPEN_LOOP_CMD_JOG = { .P_START = &MOTOR_STATE_OPEN_LOOP, .NEXT = (State_Input_T)OpenLoop_Jog, };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_JOG, direction);
}

/******************************************************************************/
/*
    Open Loop SubStates/Cmds
*/
/******************************************************************************/
/******************************************************************************/
/*
    Angle Align Cmd
    With Current Loop only
*/
/******************************************************************************/
static void AngleAlign_Entry(Motor_T * p_motor)
{
    Phase_ActivateT0(&p_motor->PHASE);
    Motor_FOC_StartAlignCmd(p_motor->P_MOTOR); // using commutation mode
}

static void AngleAlign_Loop(Motor_T * p_motor)
{
    Motor_FOC_ProcAlignCmd(p_motor);
    //Motor_FOC_WriteDuty(p_motor);
}

/*
    Angle Cmd and User Current/Voltage Cmd
*/
static const State_T OPEN_LOOP_STATE_ANGLE_ALIGN =
{
    // .ID         = MOTOR_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)AngleAlign_Entry,
    .LOOP = (State_Action_T)AngleAlign_Loop,
    .NEXT = NULL,
};

static State_T * OpenLoop_AngleAlign(Motor_T * p_motor, state_value_t angle)
{
    Angle_CaptureAngle(&p_motor->P_MOTOR->OpenLoopAngle, angle);
    return &OPEN_LOOP_STATE_ANGLE_ALIGN;
}

void Motor_OpenLoop_SetAngleAlign(Motor_T * p_motor, angle16_t angle)
{
    static const StateMachine_TransitionCmd_T OPEN_LOOP_CMD_ANGLE_ALIGN = { .P_START = &MOTOR_STATE_OPEN_LOOP, .NEXT = (State_Input_T)OpenLoop_AngleAlign, };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_ANGLE_ALIGN, angle);
}

void Motor_OpenLoop_SetAngleAlign_Phase(Motor_T * p_motor, Phase_Id_T align)
{
    Motor_OpenLoop_SetAngleAlign(p_motor, Phase_AngleOf(align));
}


/******************************************************************************/
/*
    Run Chain
*/
/******************************************************************************/
static void Run_Entry(Motor_T * p_motor)
{
    Motor_FOC_StartOpenLoop(p_motor->P_MOTOR);
}

static void Run_Proc(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    FOC_Sensorless_Step(&p_state->Foc, &p_state->FocSensorless);
    Motor_FOC_ProcOpenLoop(p_motor);
    // capture for dbugging
}

/*
*/
State_T OPEN_LOOP_STATE_RUN =
{
    // .ID         = MOTOR_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)Run_Entry,
    .LOOP = (State_Action_T)Run_Proc,
    .NEXT = NULL,
};

/******************************************************************************/
/*
    StartUp Align
*/
/******************************************************************************/
static void StartUp_Entry(Motor_T * p_motor)
{
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles);
    Motor_FOC_StartStartUpAlign(p_motor->P_MOTOR);
    FOC_Sensorless_SeedAngle(&p_motor->P_MOTOR->FocSensorless, 0, 0);
}

static void StartUp_Proc(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    Motor_FOC_ProcStartUpAlign(p_motor);
}

State_T * StartUpAlign_Next(Motor_T * p_motor)
{
    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        return &OPEN_LOOP_STATE_RUN;
    }
    return NULL;
    // if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    // {
    //     _StateMachine_TransitionTo(&p_motor->P_MOTOR->StateMachine, (void *)p_motor, &OPEN_LOOP_STATE_RUN);
    //     // switch (_Phase_ReadDutyAlign(&p_motor->PHASE).Bits)
    //     // {
    //     //     case PHASE_ID_A: p_motor->P_MOTOR->OpenLoopAngle.Angle = Phase_AngleOf(PHASE_ID_INV_C); break;
    //     //     case PHASE_ID_INV_C: p_motor->P_MOTOR->OpenLoopAngle.Angle = Phase_AngleOf(PHASE_ID_B); break;
    //     //     case PHASE_ID_B: _StateMachine_TransitionTo(&p_motor->P_MOTOR->StateMachine, (void *)p_motor, &OPEN_LOOP_STATE_RUN); break;
    //     //     default: _StateMachine_TransitionTo(&p_motor->P_MOTOR->StateMachine, (void *)p_motor, &MOTOR_STATE_FAULT); break;

    //     //     // case PHASE_ID_A: p_motor->P_MOTOR->OpenLoopAngle.Angle = Phase_AngleOf(PHASE_ID_B); break;
    //     //     // case PHASE_ID_INV_A: p_motor->P_MOTOR->OpenLoopAngle.Angle = Phase_AngleOf(PHASE_ID_B); break;
    //     //     // case PHASE_ID_C: p_motor->P_MOTOR->OpenLoopAngle.Angle = Phase_AngleOf(PHASE_ID_INV_B); break;
    //     //     // case PHASE_ID_INV_B: p_motor->P_MOTOR->OpenLoopAngle.Angle = Phase_AngleOf(PHASE_ID_INV_C); break;
    //     // }
    // }
}

State_T OPEN_LOOP_STATE_START_UP_ALIGN =
{
    // .ID         = MOTOR_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)StartUp_Entry,
    .LOOP = (State_Action_T)StartUp_Proc,
    .NEXT = (State_Input0_T)StartUpAlign_Next,
};

static State_T * OpenLoop_StartUpRun(Motor_T * p_motor, state_value_t null)
{
    (void)null;
    if (p_motor->P_MOTOR->Direction != MOTOR_DIRECTION_NULL)
    {
        Phase_ActivateV0(&p_motor->PHASE);
        return &OPEN_LOOP_STATE_START_UP_ALIGN;
    }
    return NULL;
}

void Motor_OpenLoop_StartRunChain(Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T OPEN_LOOP_CMD_RUN = { .P_START = &MOTOR_STATE_OPEN_LOOP, .NEXT = (State_Input_T)OpenLoop_StartUpRun, };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_RUN, 0);
}






/******************************************************************************/
/*
    Phase Align. With or without I Feedback. V mode apply ramp only.
*/
/******************************************************************************/
// static void PhaseAlign_Entry(Motor_T * p_motor)
// {
//     Phase_ActivateV0(&p_motor->PHASE);
//     Motor_FOC_StartAlignCmd(p_motor->P_MOTOR);

//     // TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles);
//     // Phase_ActivateV0(&p_motor->PHASE);
//     // PID_Reset(&p_motor->P_MOTOR->Foc.PidId);
//     Ramp_SetOutputState(&p_motor->P_MOTOR->TorqueRamp, 0);
//     // Ramp_SetOutputLimit(&p_motor->P_MOTOR->TorqueRamp, 0, Motor_GetIAlign(&p_motor->P_MOTOR->Config));
//     // Motor_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT);
//     // p_motor->P_MOTOR->CalibrationStateIndex = 0U;
// }

// /*
//     Voltage-mode align: open-loop fixed duty (one-shot per call is sufficient).
// */
// static void Calibration_Align_V(Motor_T * p_motor, Phase_Id_T id)
// {
//     Phase_Align(&p_motor->PHASE, id, Motor_GetVAlign_Duty(&p_motor->P_MOTOR->Config));
// }

// static void Calibration_Align_I(Motor_T * p_motor, Phase_Id_T id)
// {
//     Motor_FOC_ProcAngleAlignOf(p_motor->P_MOTOR, Phase_AngleOf(id), Motor_GetIAlign(&p_motor->P_MOTOR->Config));
//     //Motor_FOC_WriteDuty(p_motor);
// }

// static void PhaseAlign_Loop(Motor_T * p_motor)
// {
//     Motor_State_T * const p_values = p_motor->P_MOTOR;
//     Phase_Id_T alignPhase = Phase_ReadAlign(&p_motor->PHASE);

//     Motor_ProcTorqueRampOpenLoop(p_values);
//     if (p_values->FeedbackMode.Current == 1U)
//     {
//         // if (CaptureIabc)
//         // PID_ProcPI(&p_motor->P_MOTOR->PidId, PhaseInput_IAligned_Fract16(&p_motor, alignPhase), Ramp_GetOutput(&p_motor->P_MOTOR->TorqueRamp));
//         Phase_Align(&p_motor->PHASE, alignPhase, PID_GetOutput(&p_values->PidId)); /* Id or Iphase */
//     }
//     else
//     {
//         // Phase_Align(&p_motor->PHASE, alignPhase, Motor_GetVAlign_Duty(p_motor->P_MOTOR));
//         Phase_Align(&p_motor->PHASE, alignPhase, Ramp_GetOutput(&p_values->TorqueRamp));
//     }
// }

// static const State_T OPEN_LOOP_STATE_PHASE_ALIGN =
// {
//     // .ID = MOTOR_STATE_ID_OPEN_LOOP,
//     .P_TOP = &MOTOR_STATE_OPEN_LOOP,
//     .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
//     .DEPTH = 1U,
//     .ENTRY = (State_Action_T)PhaseAlign_Entry,
//     .LOOP = (State_Action_T)PhaseAlign_Loop,
//     .NEXT = NULL,
// };

// static State_T * OpenLoop_PhaseAlign(Motor_T * p_motor, state_value_t phaseAlign)
// {
//     return &OPEN_LOOP_STATE_PHASE_ALIGN;
// }

// void Motor_OpenLoop_SetPhaseAlign(Motor_T * p_motor, Phase_Id_T align)
// {
//     static const StateMachine_TransitionCmd_T OPEN_LOOP_CMD_V_ALIGN = { .P_START = &MOTOR_STATE_OPEN_LOOP, .NEXT = (State_Input_T)OpenLoop_PhaseAlign, };
//     StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_V_ALIGN, align);
// }



