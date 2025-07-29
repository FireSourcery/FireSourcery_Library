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


/* StateMachine_InvokeBranchTransition from any OpenLoop State */

/******************************************************************************/
/*
    Open Loop Cmds - Without SubState
*/
/******************************************************************************/
/*
    sets the phase state without exiting openloop
*/
static State_T * OpenLoop_PhaseOutput(const Motor_T * p_motor, state_value_t phaseState)
{
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR_STATE);
    Phase_ActivateOutputState(&p_motor->PHASE, (Phase_Output_T)phaseState); // clear outputState V, or enable last duty
    return &MOTOR_STATE_OPEN_LOOP;
}

/* ActivateOutput */
void Motor_OpenLoop_SetPhaseOutput(const Motor_T * p_motor, Phase_Output_T phase)
{
    static const StateMachine_TransitionInput_T OPEN_LOOP_CMD_PHASE = { .P_START = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)OpenLoop_PhaseOutput, };
    StateMachine_InvokeBranchTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_PHASE, phase);
}

/*
*/
static State_T * OpenLoop_Jog(const Motor_T * p_motor, state_value_t direction)
{
    // p_motor->P_MOTOR_STATE->ElectricalAngle = Phase_AngleOf(Phase_JogNext(&p_motor->PHASE, Motor_GetVAlign_Duty(p_motor->P_MOTOR_STATE)));
    return &MOTOR_STATE_OPEN_LOOP;
}

void Motor_OpenLoop_SetJog(const Motor_T * p_motor, int8_t direction)
{
    static const StateMachine_TransitionInput_T OPEN_LOOP_CMD_JOG = { .P_START = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)OpenLoop_Jog, };
    StateMachine_InvokeBranchTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_JOG, direction);
}

/******************************************************************************/
/*
    Open Loop SubStates/Cmds
*/
/******************************************************************************/
/******************************************************************************/
/*
    Phase Align. With or without I Feedback. V mode appy ramp only.
*/
/******************************************************************************/
/* ElectricalAngle set by caller */
void Motor_StartAlignCmd(Motor_State_T * p_motor)
{
    Ramp_SetOutput(&p_motor->TorqueRamp, 0);
    if (p_motor->FeedbackMode.Current == 1U)
    {
    }
    // else
    // {
    //     Phase_Align(&p_motor->PHASE, Phase_ReadAlign(&p_motor->PHASE), Motor_GetVAlign_Duty(p_motor));
    // }
}

void Motor_ProcPhaseAlign(Motor_State_T * p_motor)
{
    Motor_ProcTorqueRampOpenLoop(p_motor);
}

static void PhaseAlign_Entry(const Motor_T * p_motor)
{
    Phase_ActivateOutputT0(&p_motor->PHASE);
    Motor_StartAlignCmd(p_motor->P_MOTOR_STATE);
}

static void PhaseAlign_Loop(const Motor_T * p_motor)
{
    Phase_Id_T alignPhase = Phase_ReadAlign(&p_motor->PHASE);
    Motor_ProcPhaseAlign(p_motor->P_MOTOR_STATE);
    if (p_motor->P_MOTOR_STATE->FeedbackMode.Current == 1U)
    {
        // if (CaptureIabc)
        // PID_ProcPI(&p_motor->P_MOTOR_STATE->PidId, Motor_GetIPhase_Fract16(&p_motor, alignPhase), Ramp_GetOutput(&p_motor->P_MOTOR_STATE->TorqueRamp));
        Phase_Align(&p_motor->PHASE, alignPhase, PID_GetOutput(&p_motor->P_MOTOR_STATE->PidId)); /* Id or Iphase */
    }
    else
    {
        // Phase_Align(&p_motor->PHASE, alignPhase, Motor_GetVAlign_Duty(p_motor->P_MOTOR_STATE));
        Phase_Align(&p_motor->PHASE, alignPhase, Ramp_GetOutput(&p_motor->P_MOTOR_STATE->TorqueRamp));
    }
}

static const State_T OPEN_LOOP_STATE_PHASE_ALIGN =
{
    // .ID = MSM_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)PhaseAlign_Entry,
    .LOOP = (State_Action_T)PhaseAlign_Loop,
    .NEXT = NULL,
};

static State_T * OpenLoop_PhaseAlign(const Motor_T * p_motor, state_value_t phaseAlign)
{
    return &OPEN_LOOP_STATE_PHASE_ALIGN;
}

void Motor_OpenLoop_SetPhaseAlign(const Motor_T * p_motor, Phase_Id_T align)
{
    static const StateMachine_TransitionInput_T OPEN_LOOP_CMD_V_ALIGN = { .P_START = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)OpenLoop_PhaseAlign, };
    StateMachine_InvokeBranchTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_V_ALIGN, align);
}

/******************************************************************************/
/*
    Angle Align
    With Current Loop only
*/
/******************************************************************************/
static void AngleAlign_Entry(const Motor_T * p_motor)
{
    Phase_ActivateOutputT0(&p_motor->PHASE);
    Motor_FOC_StartAlignCmd(p_motor->P_MOTOR_STATE); // using commutation mode
}

static void AngleAlign_Loop(const Motor_T * p_motor)
{
    Motor_FOC_ProcAlignCmd(p_motor->P_MOTOR_STATE);
}

/*
    Angle Cmd and User Current/Voltage Cmd
*/
static const State_T OPEN_LOOP_STATE_ANGLE_ALIGN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)AngleAlign_Entry,
    .LOOP = (State_Action_T)AngleAlign_Loop,
    .NEXT = NULL,
};

static State_T * OpenLoop_AngleAlign(const Motor_T * p_motor, state_value_t angle)
{
    // p_motor->P_MOTOR_STATE->ElectricalAngle = angle;
    // Motor_SetElAngleFeedforward(p_motor->P_MOTOR_STATE, angle);
    return &OPEN_LOOP_STATE_ANGLE_ALIGN;
}

void Motor_OpenLoop_SetAngleAlign(const Motor_T * p_motor, angle16_t angle)
{
    static const StateMachine_TransitionInput_T OPEN_LOOP_CMD_ANGLE_ALIGN = { .P_START = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)OpenLoop_AngleAlign, };
    StateMachine_InvokeBranchTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_ANGLE_ALIGN, angle);
}

void Motor_OpenLoop_SetAngleAlign_Phase(const Motor_T * p_motor, Phase_Id_T align)
{
    Motor_OpenLoop_SetAngleAlign(p_motor, Phase_AngleOf(align));
}


/******************************************************************************/
/*
    Run Chain
*/
/******************************************************************************/
static void Run_Entry(const Motor_T * p_motor)
{
    Motor_FOC_StartOpenLoop(p_motor->P_MOTOR_STATE);
}

static void Run_Loop(const Motor_T * p_motor)
{
    Motor_FOC_ProcOpenLoop(p_motor->P_MOTOR_STATE);
}

/*
*/
static const State_T OPEN_LOOP_STATE_RUN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)Run_Entry,
    .LOOP = (State_Action_T)Run_Loop,
    .NEXT = NULL,
};

/******************************************************************************/
/*
    Run Chain
*/
/******************************************************************************/
static void StartUp_Entry(const Motor_T * p_motor)
{
    TimerT_Start(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR_STATE->Config.AlignTime_Cycles);
    Motor_FOC_StartStartUpAlign(p_motor->P_MOTOR_STATE);
}

static void StartUp_Loop(const Motor_T * p_motor)
{
    Motor_FOC_ProcStartUpAlign(p_motor->P_MOTOR_STATE);
}

static State_T * StartUp_Next(const Motor_T * p_motor)
{
    /* todo  algin b */
    return (Timer_Periodic_Poll(&p_motor->P_MOTOR_STATE->ControlTimer) == true) ? &OPEN_LOOP_STATE_RUN : NULL;
}

/* Align with Aux Ramp */
static const State_T OPEN_LOOP_STATE_START_UP_ALIGN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)StartUp_Entry,
    .LOOP = (State_Action_T)StartUp_Loop,
    .NEXT = (State_InputVoid_T)StartUp_Next,
};

// test branch
// static const State_T OPEN_LOOP_STATE_RUN_START_UP =
// {
//     // .ID         = MSM_STATE_ID_OPEN_LOOP,
//     .P_PARENT   = &OPEN_LOOP_STATE_START_UP_ALIGN,
//     .DEPTH      = 2U,
//     .ENTRY      = (State_Action_T)OpenLoop_StartUpAlign_EntryTimer,
//     .NEXT       = (State_InputVoid_T)OpenLoop_StartUpAlign_Transition,
// };

static State_T * OpenLoop_StartUpRun(const Motor_T * p_motor, state_value_t null)
{
    // Motor_FOC_ActivateOutputZero(p_motor); // in case it has been disabled
    Phase_ActivateOutputT0(&p_motor->PHASE);
    return &OPEN_LOOP_STATE_START_UP_ALIGN;
}

void Motor_OpenLoop_StartRunChain(const Motor_T * p_motor)
{
    static const StateMachine_TransitionInput_T OPEN_LOOP_CMD_RUN = { .P_START = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)OpenLoop_StartUpRun, };
    StateMachine_InvokeBranchTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_RUN, 0);
}




/******************************************************************************/
/*
    Vard Id
*/
/******************************************************************************/
// void _Motor_OpenLoop_VarCmd(const Motor_T * p_motor, Motor_OpenLoop_Cmd_T varId, int32_t varValue)
// {
//     switch (varId)
//     {
//         case MOTOR_VAR_OPEN_LOOP_ENTER:         Motor_OpenLoop_Enter(p_motor);                                      break;
//         case MOTOR_VAR_OPEN_LOOP_PHASE_OUTPUT:  Motor_OpenLoop_SetPhaseOutput(p_motor, (Phase_Output_T)varValue);   break;
//         case MOTOR_VAR_OPEN_LOOP_PHASE_ALIGN:   Motor_OpenLoop_SetPhaseAlign(p_motor, (Phase_Id_T)varValue);        break;
//         case MOTOR_VAR_OPEN_LOOP_ANGLE_ALIGN:   Motor_OpenLoop_SetAngleAlign(p_motor, varValue);                    break;
//         case MOTOR_VAR_OPEN_LOOP_JOG:           Motor_OpenLoop_SetJog(p_motor, varValue);                           break;
//         case MOTOR_VAR_OPEN_LOOP_RUN:           Motor_OpenLoop_StartRunChain(p_motor);                              break;
//         // case MOTOR_VAR_OPEN_LOOP_HOMING:     break;
//     }
// }