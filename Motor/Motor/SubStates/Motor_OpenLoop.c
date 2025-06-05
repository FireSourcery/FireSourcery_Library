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
/******************************************************************************/
#include "Motor_OpenLoop.h"

/* part of Motor_StateMachine */

/******************************************************************************/
/*
    Open Loop Cmds
*/
/******************************************************************************/
/*
    sets the phase state without exiting openloop
*/
static State_T * OpenLoop_PhaseOutput(const Motor_T * p_motor, state_input_value_t phaseState)
{
    Motor_FOC_ClearFeedbackState(p_motor->P_ACTIVE);
    Phase_ActivateOutputState(&p_motor->PHASE, (Phase_Output_T)phaseState); // clear outputState V, or enable last duty
    return &MOTOR_STATE_OPEN_LOOP;
}

/* ActivateOutput */
void Motor_OpenLoop_SetPhaseOutput(const Motor_T * p_motor, Phase_Output_T phase)
{
    static const State_TransitionInput_T OPEN_LOOP_CMD_PHASE = { .P_VALID = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)OpenLoop_PhaseOutput, };
    StateMachine_InvokeBranchTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_PHASE, phase);
}

/*
*/
static State_T * OpenLoop_Jog(const Motor_T * p_motor, state_input_value_t direction)
{
    // p_motor->P_ACTIVE->ElectricalAngle = Phase_AngleOf(Phase_JogNext(&p_motor->PHASE, Motor_GetVAlign_Duty(p_motor->P_ACTIVE)));
    return &MOTOR_STATE_OPEN_LOOP;
}

void Motor_OpenLoop_SetJog(const Motor_T * p_motor, int8_t direction)
{
    static const State_TransitionInput_T OPEN_LOOP_CMD_JOG = { .P_VALID = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)OpenLoop_Jog, };
    StateMachine_InvokeBranchTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_JOG, direction);
}

/******************************************************************************/
/*
    Open Loop SubStates/Cmds
*/
/******************************************************************************/
/******************************************************************************/
/*
    Phase Align. With or without I Feedback
    V only for now
*/
/******************************************************************************/
/* ElectricalAngle set by caller */
/* same as FOC  */
void Motor_StartAlignCmd(const Motor_T * p_motor)
{
    Phase_ActivateOutput(&p_motor->PHASE);
    Ramp_SetOutput(&p_motor->P_ACTIVE->TorqueRamp, 0);

    if (p_motor->P_ACTIVE->FeedbackMode.Current == 1U)
    {
    }
    // else
    // {
    //     Phase_Align(&p_motor->PHASE, Phase_ReadAlign(&p_motor->PHASE), Motor_GetVAlign_Duty(p_motor));
    // }

}

void Motor_ProcPhaseAlign(const Motor_T * p_motor)
{
    // int16_t req = Motor_ProcTorqueRampOpenLoop(p_motor->P_ACTIVE);

    if (p_motor->P_ACTIVE->FeedbackMode.Current == 1U)
    {
        // if (CaptureIabc)
        // PID_ProcPI(&p_motor->IPhase, GetIPhase(&p_motor, Phase_ReadAlign(&p_motor->PHASE)), req ); //feedforward align angle or set up analog conversion
    }
    /* VAlign remain in OpenLoop SubState */
}

static const State_T OPEN_LOOP_STATE_PHASE_ALIGN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)Motor_StartAlignCmd,
    .LOOP = (State_Action_T)Motor_ProcPhaseAlign,
    .NEXT = NULL,
};

static State_T * OpenLoop_PhaseAlign(const Motor_T * p_motor, state_input_value_t phaseAlign)
{
    Motor_SetElecAngleFeedforward(p_motor->P_ACTIVE, Phase_AngleOf(phaseAlign));

    if (p_motor->P_ACTIVE->FeedbackMode.Current == 0U)
    {
        Phase_Align(&p_motor->PHASE, phaseAlign, Motor_GetVAlign_Duty(p_motor->P_ACTIVE));
        Phase_ActivateOutput(&p_motor->PHASE);
        //optional return open loop
    }

    return &OPEN_LOOP_STATE_PHASE_ALIGN;
}

void Motor_OpenLoop_SetPhaseAlign(const Motor_T * p_motor, Phase_Id_T align)
{
    static const State_TransitionInput_T OPEN_LOOP_CMD_V_ALIGN = { .P_VALID = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)OpenLoop_PhaseAlign, };
    StateMachine_InvokeBranchTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_V_ALIGN, align);
}

/******************************************************************************/
/*
    Angle Align
    With Current Loop only
*/
/******************************************************************************/
/*
    Angle Cmd and User Current/Voltage Cmd
*/
static const State_T OPEN_LOOP_STATE_ANGLE_ALIGN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)Motor_FOC_StartAlignCmd,
    .LOOP = (State_Action_T)Motor_FOC_ProcAlignCmd,
    .NEXT = NULL,
};

static State_T * OpenLoop_AngleAlign(const Motor_T * p_motor, state_input_value_t angle)
{
    Phase_ActivateOutput(&p_motor->PHASE);
    // p_motor->P_ACTIVE->ElectricalAngle = angle;
    Motor_SetElecAngleFeedforward(p_motor->P_ACTIVE, angle);
    return &OPEN_LOOP_STATE_ANGLE_ALIGN;
}

void Motor_OpenLoop_SetAngleAlign(const Motor_T * p_motor, angle16_t angle)
{
    static const State_TransitionInput_T OPEN_LOOP_CMD_ANGLE_ALIGN = { .P_VALID = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)OpenLoop_AngleAlign, };
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
/*
*/
static const State_T OPEN_LOOP_STATE_RUN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)Motor_FOC_StartOpenLoop,
    .LOOP = (State_Action_T)Motor_FOC_ProcOpenLoop,
    .NEXT = NULL,
};


static State_T * OpenLoop_StartUpAlignEnd(const Motor_T * p_motor)
{
    /* todo if algin b */
    return (Timer_Periodic_Poll(&p_motor->P_ACTIVE->ControlTimer) == true) ? &OPEN_LOOP_STATE_RUN : NULL;
}

/* Align with Aux Ramp */
static const State_T OPEN_LOOP_STATE_START_UP_ALIGN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_TOP = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)Motor_FOC_StartStartUpAlign,
    .LOOP = (State_Action_T)Motor_FOC_ProcStartUpAlign,
    .NEXT = (State_InputVoid_T)OpenLoop_StartUpAlignEnd,
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

static State_T * OpenLoop_StartUpRun(const Motor_T * p_motor, state_input_value_t null)
{
    // Motor_FOC_ActivateOutputZero(p_motor); //in case it has been disabled
    Phase_ActivateOutputT0(&p_motor->PHASE);
    return &OPEN_LOOP_STATE_START_UP_ALIGN;
}

void Motor_OpenLoop_StartRunChain(const Motor_T * p_motor)
{
    static const State_TransitionInput_T OPEN_LOOP_CMD_RUN = { .P_VALID = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (State_Input_T)OpenLoop_StartUpRun, };
    StateMachine_InvokeBranchTransition(&p_motor->STATE_MACHINE, &OPEN_LOOP_CMD_RUN, 0);
}

