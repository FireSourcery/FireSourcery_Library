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
    @file   MotorStateMachine.c
    @author FireSourcery
    @brief  MotorStateMachine
    @version V0
*/
/******************************************************************************/
#include "Motor_StateMachine.h"


#define MSM_TRANSITION_TABLE_LENGTH (7U)

/******************************************************************************/
/*!
    @brief State Machine
*/
/******************************************************************************/
const StateMachine_Machine_T MSM_MACHINE =
{
    .P_STATE_INITIAL = &MOTOR_STATE_INIT,
    .TRANSITION_TABLE_LENGTH = MSM_TRANSITION_TABLE_LENGTH,
};

static StateMachine_State_T * TransitionFault(Motor_T * p_motor, state_machine_value_t faultFlags) { return &MOTOR_STATE_FAULT; }

static void Motor_PollFaultFlags(Motor_T * p_motor)
{
    p_motor->FaultFlags.Overheat = Thermistor_IsFault(&p_motor->Thermistor);
    p_motor->FaultFlags.PositionSensor = !Motor_VerifySensorCalibration(p_motor);
}

/******************************************************************************/
/*!
    @brief States
        Proc on PWM thread, high priority
        Inputs on main thread, low priority
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Init_Entry(Motor_T * p_motor)
{
    /* alternatively null until a direction is set */
    /* Sets Speed/I Limits using direction */
    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_SetDirectionForward, Motor_SetDirectionForward);
}

static void Init_Proc(Motor_T * p_motor)
{

}

static StateMachine_State_T * Init_Next(Motor_T * p_motor)
{
    StateMachine_State_T * p_nextState = NULL;

    if (SysTime_GetMillis() > MOTOR_STATE_MACHINE_INIT_WAIT) /* wait for Speed and Heat sensors */
    {
        // (Motor_CheckConfig() == true)    // check params
        if (MotorAnalogRef_IsLoaded() == false) { p_motor->FaultFlags.InitCheck = 1U; }
        Motor_PollFaultFlags(p_motor); /* Clear the fault flags once */

        if (p_motor->FaultFlags.Value != 0U) { p_nextState = &MOTOR_STATE_FAULT; }
        else                                 { p_nextState = &MOTOR_STATE_STOP; }
    }

    return p_nextState;
}

static const StateMachine_Input_T INIT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = NULL,
    [MSM_INPUT_DIRECTION]       = NULL,
    [MSM_INPUT_CALIBRATION]     = NULL,
};

const StateMachine_State_T MOTOR_STATE_INIT =
{
    .ID                 = MSM_STATE_ID_INIT,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Init_Entry,
    .LOOP               = (StateMachine_Function_T)Init_Proc,
    .NEXT               = (StateMachine_InputVoid_T)Init_Next,
};

/******************************************************************************/
/*!
    @brief Stop State

    Safe State with Transition to Calibration
    Motor is in floating state with 0 speed
*/
/******************************************************************************/
/*
    Enters upon reaching 0 Speed
*/
static void Stop_Entry(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ClearFeedbackState, NULL); /* Unobserved values remain 0 for user read */
    p_motor->ControlTimerBase = 0U; /* ok to reset timer */
    _StateMachine_EndSubState(&p_motor->StateMachine);
}

static void Stop_Proc(Motor_T * p_motor)
{
    Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL);
    // if (p_motor->Speed_Fract16 > 0)
    // {
    //     if (Motor_IsHold(p_motor) == false) { _StateMachine_SetState(&p_motor->StateMachine, &MOTOR_STATE_FREEWHEEL); }
    //     else                                { p_motor->StateFlags.Alarm = 1U;}
    // }
}

static StateMachine_State_T * Stop_InputDirection(Motor_T * p_motor, state_machine_value_t direction)
{
    Motor_CommutationModeFn_Call(p_motor, Motor_FOC_SetDirection_Cast, Motor_SetDirection_Cast, direction);
    return (direction != MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_PASSIVE : NULL;
}

static StateMachine_State_T * Stop_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
{
    StateMachine_State_T * p_nextState = NULL;

    switch ((Phase_Mode_T)phaseMode)
    {
        case PHASE_OUTPUT_FLOAT:    Phase_Float(&p_motor->Phase);               break;
        case PHASE_OUTPUT_V0:       Phase_ActivateOutputV0(&p_motor->Phase);    break;
        // case PHASE_OUTPUT_VPWM:     p_nextState = &MOTOR_STATE_PASSIVE;         break;
        default: break;
    }

    return p_nextState;
}

static StateMachine_State_T * Stop_InputFeedbackMode(Motor_T * p_motor, state_machine_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
    return NULL;
}

/* Transition for user input */
// static StateMachine_State_T * Stop_InputOpenLoop(Motor_T * p_motor, state_machine_value_t state)
// {
//     return &MOTOR_STATE_OPEN_LOOP;
// }

/* Calibration go directly to SubState */
static StateMachine_State_T * Stop_InputCalibration(Motor_T * p_motor, state_machine_value_t statePtr)
{
    StateMachine_State_T * p_state = (StateMachine_State_T *)statePtr;

    assert(/* p_state == 0U || */ p_state == &MOTOR_STATE_STOP || p_state == &MOTOR_STATE_CALIBRATION || p_state->P_ROOT == &MOTOR_STATE_CALIBRATION);

    return p_state;
}

static const StateMachine_Input_T STOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
    [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)Stop_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (StateMachine_Input_T)Stop_InputFeedbackMode,
    [MSM_INPUT_DIRECTION]       = (StateMachine_Input_T)Stop_InputDirection,
    [MSM_INPUT_CALIBRATION]     = (StateMachine_Input_T)Stop_InputCalibration,
    // [MSM_INPUT_OPEN_LOOP]       = (StateMachine_Input_T)Stop_InputOpenLoop,
    [MSM_INPUT_OPEN_LOOP]       = NULL,
};

const StateMachine_State_T MOTOR_STATE_STOP =
{
    .ID                 = MSM_STATE_ID_STOP,
    .P_TRANSITION_TABLE = &STOP_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Stop_Entry,
    .LOOP               = (StateMachine_Function_T)Stop_Proc,
};


/******************************************************************************/
/*!
    @brief Passive State
        Begin Freewheel/Hold
*/
/******************************************************************************/
static void Passive_Entry(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ClearFeedbackState, NULL);
    Ramp_Init(&p_motor->TorqueRamp, p_motor->Config.TorqueRampTime_Cycles, MotorAnalogRef_GetIRatedPeak_Fract16());
    _StateMachine_EndSubState(&p_motor->StateMachine);
    // p_motor->ControlTimerBase = 0U; /* ok to reset timer */
}

static void Passive_Proc(Motor_T * p_motor)
{
    /* Match Feedback to ProcAngleBemf on Resume */
    Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL /* Motor_SixStep_ProcPhaseObserve */);
}

static StateMachine_State_T * Passive_Next(Motor_T * p_motor)
{
    // if (p_motor->Speed_Fract16 > 0)
    // {
    //     if (Motor_IsHold(p_motor) == false) { _StateMachine_SetState(&p_motor->StateMachine, &MOTOR_STATE_FREEWHEEL); }
    //     else                                { p_motor->StateFlags.Alarm = 1U;}
    // }
}

/* alternatively separate state for speed 0 */
static StateMachine_State_T * Passive_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
{
    StateMachine_State_T * p_nextState = NULL;

    switch ((Phase_Mode_T)phaseMode)
    {
        case PHASE_OUTPUT_FLOAT:
            Phase_Float(&p_motor->Phase);
            // if (Phase_ReadOutputState(&p_motor->Phase) == PHASE_OUTPUT_FLOAT) { p_nextState = &MOTOR_STATE_STOP; } /*  */
            break;
        case PHASE_OUTPUT_V0:
            if (p_motor->Speed_Fract16 == 0U) { Phase_ActivateOutputV0(&p_motor->Phase); }
            // if (Phase_ReadOutputState(&p_motor->Phase) == PHASE_OUTPUT_V0) { p_nextState = &MOTOR_STATE_STOP; } /* outside handle */
            break;
        case PHASE_OUTPUT_VPWM:
            // Motor_SetSensorInitial(p_motor);
            /* p_nextState = Motor_GetSensorStartUpState(p_motor); */
            if (Motor_IsClosedLoop(p_motor) == true)
            {
                // Motor_ZeroSensor(p_motor);
                p_nextState = &MOTOR_STATE_RUN;
            }
            else if (p_motor->Speed_Fract16 == 0U) /* Openloop at 0 speed */
            {
                p_nextState = &MOTOR_STATE_OPEN_LOOP; /* Motor_GetSensorStartUpState() */
                // p_nextState = &OPEN_LOOP_STATE_START_UP_ALIGN; /* Motor_GetSensorStartUpState() */
            }
            break;
    }

    return p_nextState;
}

/* alternatively use calibration, direction 0, or repeat phase mode, as return to stop */
static StateMachine_State_T * Passive_InputDirection(Motor_T * p_motor, state_machine_value_t direction)
{
    StateMachine_State_T * p_nextState = NULL;
    /* Alternatively, use another 0 speed state */
    if (p_motor->Speed_Fract16 == 0U)
    {
        Motor_CommutationModeFn_Call(p_motor, Motor_FOC_SetDirection_Cast, Motor_SetDirection_Cast, direction);
        if (direction == MOTOR_DIRECTION_NULL) { p_nextState = &MOTOR_STATE_STOP; }
    }
    return p_nextState;
}

static StateMachine_State_T * Passive_InputFeedbackMode(Motor_T * p_motor, state_machine_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
    return NULL;
}

static StateMachine_State_T * Passive_InputOpenLoop(Motor_T * p_motor, state_machine_value_t state)
{
    return &MOTOR_STATE_OPEN_LOOP; /* User OpenLoop */
}

static const StateMachine_Input_T PASSIVE_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
    [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)Passive_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (StateMachine_Input_T)Passive_InputFeedbackMode,
    [MSM_INPUT_DIRECTION]       = (StateMachine_Input_T)Passive_InputDirection,
    [MSM_INPUT_OPEN_LOOP]       = (StateMachine_Input_T)Passive_InputOpenLoop,
    [MSM_INPUT_CALIBRATION]     = NULL,
};

const StateMachine_State_T MOTOR_STATE_PASSIVE =
{
    .ID                 = MSM_STATE_ID_PASSIVE,
    .P_TRANSITION_TABLE = &PASSIVE_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Passive_Entry,
    .LOOP               = (StateMachine_Function_T)Passive_Proc,
};


// const StateMachine_State_T MOTOR_STATE_PASSIVE_STOP =
// {
//     .P_PARENT = &MOTOR_STATE_PASSIVE,
// }

// const StateMachine_State_T MOTOR_STATE_PASSIVE_FREEWHEEL =
// {
//     .P_PARENT = &MOTOR_STATE_PASSIVE,
// }

/******************************************************************************/
/*!
    @brief  Run State
    Spin with Feedback State
    Active Control, FeedbackLoop is in effect
        UserCmd => RampOutput => PID => AngleControl
*/
/******************************************************************************/
static void Run_Entry(Motor_T * p_motor)
{
    /* Optionally poll angle sensor or let interpolate angle be off by 1 */
    // Motor_UpdateSpeedControlLimits(p_motor); /* Alternatively run in SetFeedbackMode does not need to update  unless feedback mode changed */
    Motor_CommutationModeFn_Call(p_motor, Motor_FOC_MatchFeedbackState, NULL);
    Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ActivateOutput, NULL);
    _StateMachine_EndSubState(&p_motor->StateMachine);
}

static void Run_Proc(Motor_T * p_motor)
{
    Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcAngleControl, NULL/* Motor_SixStep_ProcPhaseControl */);
}

// static StateMachine_State_T * Run_Next(Motor_T * p_motor)
// {
//     StateMachine_State_T * p_nextState = NULL;

//     if (p_motor->Speed_Fract16 < ) { p_nextState = &MOTOR_STATE_; }

//     return p_nextState;
// }

/*
    StateMachine in Sync mode, [ProcInput] in the same thread as [Run_Proc]/[ProcAngleControl]
    Process [Motor_FOC_MatchFeedbackState] before [Motor_FOC_ProcAngleControl]
*/
static StateMachine_State_T * Run_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
{
    StateMachine_State_T * p_nextState = NULL;

    switch ((Phase_Mode_T)phaseMode)
    {
        case PHASE_OUTPUT_FLOAT:
            //   (Motor_CheckSpeed(p_motor) == true) ? &MOTOR_STATE_FREEWHEEL : 0U; // check speed range
            p_nextState = &MOTOR_STATE_PASSIVE;
            break;
        case PHASE_OUTPUT_V0:
            //   (Motor_CheckSpeed(p_motor) == true) ? &MOTOR_STATE_FREEWHEEL : 0U; // check speed range
            p_nextState = &MOTOR_STATE_PASSIVE;
            // switch(p_motor->FeedbackMode)
            break;
        case PHASE_OUTPUT_VPWM: break;
            // switch(p_motor->FeedbackMode)
    }

    return p_nextState;
}

static StateMachine_State_T * Run_InputFeedbackMode(Motor_T * p_motor, state_machine_value_t feedbackMode)
{
    StateMachine_State_T * p_nextState = NULL;

    if (feedbackMode != p_motor->FeedbackMode.Value)
    {
        Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
        p_nextState = &MOTOR_STATE_RUN; /* repeat entry function */ /* Alternatively, transition through Freewheel */
    }

    return p_nextState;
}

static const StateMachine_Input_T RUN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
    [MSM_INPUT_FEEDBACK_MODE]   = (StateMachine_Input_T)Run_InputFeedbackMode,
    [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)Run_InputControl,
    [MSM_INPUT_DIRECTION]       = NULL,
    [MSM_INPUT_CALIBRATION]     = NULL,
    [MSM_INPUT_OPEN_LOOP]       = NULL,
};

const StateMachine_State_T MOTOR_STATE_RUN =
{
    .ID                 = MSM_STATE_ID_RUN,
    .P_TRANSITION_TABLE = &RUN_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Run_Entry,
    .LOOP               = (StateMachine_Function_T)Run_Proc,
};

// /******************************************************************************/
// /*!
//     @brief State
//     alternatively make this passive state, substate freewheel, hold
//     Release Control,
// */
// /******************************************************************************/
// static void Freewheel_Entry(Motor_T * p_motor)
// {
//     Phase_Float(&p_motor->Phase);
//     Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ClearFeedbackState, NULL);
// }

// static void Freewheel_Proc(Motor_T * p_motor)
// {
//     Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL /* Motor_SixStep_ProcPhaseObserve */);
//     // alternatively wait for input
//     // if (p_motor->Speed_Fract16 == 0U) { _StateMachine_SetState(&p_motor->StateMachine, &MOTOR_STATE_STOP); }
// }

// // static StateMachine_State_T * Freewheel_Next(Motor_T * p_motor)
// // {
// //     return (p_motor->Speed_Fract16 == 0U) ? &MOTOR_STATE_STOP : NULL;
// // }

// /* Match Feedback to ProcAngleBemf on Resume */
// static StateMachine_State_T * Freewheel_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
// {
//     StateMachine_State_T * p_nextState = NULL;

//     switch ((Phase_Mode_T)phaseMode)
//     {
//         case PHASE_OUTPUT_FLOAT:  if (p_motor->Speed_Fract16 == 0U) { p_nextState = &MOTOR_STATE_STOP; }    break;
//         case PHASE_OUTPUT_V0: if (p_motor->Speed_Fract16 == 0U) { p_nextState = &MOTOR_STATE_STOP; }    break;
//         case PHASE_OUTPUT_VPWM:
//             if (Motor_IsClosedLoop(p_motor) == true) { p_nextState = &MOTOR_STATE_RUN; } /* If flags set */
//             /* OpenLoop does not resume */
//             break;
//     }

//     return p_nextState;
// }

// static StateMachine_State_T * Freewheel_InputFeedbackMode(Motor_T * p_motor, state_machine_value_t feedbackMode)
// {
//     Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
//     return NULL;
// }

// static const StateMachine_Input_T FREEWHEEL_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
// {
//     [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
//     [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)Freewheel_InputControl,
//     [MSM_INPUT_FEEDBACK_MODE]   = (StateMachine_Input_T)Freewheel_InputFeedbackMode,
//     [MSM_INPUT_DIRECTION]       = NULL,
//     [MSM_INPUT_CALIBRATION]     = NULL,
// };

// const StateMachine_State_T MOTOR_STATE_FREEWHEEL =
// {
//     .ID                 = MSM_STATE_ID_FREEWHEEL,
//     .P_TRANSITION_TABLE = &FREEWHEEL_TRANSITION_TABLE[0U],
//     .ENTRY              = (StateMachine_Function_T)Freewheel_Entry,
//     .LOOP               = (StateMachine_Function_T)Freewheel_Proc,
// };


/******************************************************************************/
/*!
    @brief State OpenLoop - OpenLoop, Align, and Start Up, Feedback Acquisition

    Only Entry is from [STOP] State
*/
/******************************************************************************/
static void OpenLoop_Entry(Motor_T * p_motor)
{
    p_motor->FeedbackMode.OpenLoop = 1U; /* limits user cmd input */
    // Phase_ActivateOutputV0(&p_motor->Phase);
    FOC_ClearControlState(&p_motor->Foc);
    Motor_FOC_ActivateOutputZero(p_motor);
    _StateMachine_EndSubState(&p_motor->StateMachine); /* 'unmount' last operation */
    // determine initial substate from stop

    // p_motor->ElectricalAngle = 0U;
}


static void OpenLoop_Proc(Motor_T * p_motor)
{
    _StateMachine_ProcBranch_Nested(&p_motor->StateMachine);
}

/* as motor state */
/* maintain consistent interface with other states, use substate cmd for phase state without exiting */
static StateMachine_State_T * OpenLoop_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
{
    StateMachine_State_T * p_nextState = NULL;

    switch ((Phase_Mode_T)phaseMode)
    {
        case PHASE_OUTPUT_FLOAT:    p_nextState = &MOTOR_STATE_PASSIVE;              break;
        case PHASE_OUTPUT_V0:       /* Phase_ActivateOutputV0(&p_motor->Phase); */   break;
        case PHASE_OUTPUT_VPWM:     /* Motor_FOC_ActivateOutputZero(p_motor); */     break;
        /* No resume from OpenLoop, freewheel state check stop */
    }

    if (p_nextState == &MOTOR_STATE_PASSIVE) { _StateMachine_EndSubState(&p_motor->StateMachine); }
    return p_nextState;
}

static StateMachine_State_T * OpenLoop_InputFeedbackMode(Motor_T * p_motor, state_machine_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor, feedbackMode); /* a different flag mode will change ramp limits */
    return (p_motor->FeedbackMode.OpenLoop == 0U) ? &MOTOR_STATE_PASSIVE : NULL;
    // return NULL;
}

/*
    using openloop substate inputs
    multiple cmds per subState. Implemented with [StateMachine_TransitionInput_T]
*/
static StateMachine_State_T * OpenLoop_InputOpenLoop(Motor_T * p_motor, state_machine_value_t openLoop)
{
    return &MOTOR_STATE_OPEN_LOOP;
}

static const StateMachine_Input_T OPEN_LOOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
    [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)OpenLoop_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (StateMachine_Input_T)OpenLoop_InputFeedbackMode,
    [MSM_INPUT_OPEN_LOOP]       = (StateMachine_Input_T)OpenLoop_InputOpenLoop,
    [MSM_INPUT_DIRECTION]       = NULL,
    [MSM_INPUT_CALIBRATION]     = NULL,
};

const StateMachine_State_T MOTOR_STATE_OPEN_LOOP =
{
    .ID                 = MSM_STATE_ID_OPEN_LOOP,
    .ENTRY              = (StateMachine_Function_T)OpenLoop_Entry,
    .LOOP               = (StateMachine_Function_T)OpenLoop_Proc,
    .P_TRANSITION_TABLE = &OPEN_LOOP_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*
    Open Loop Cmds
*/
/******************************************************************************/
/*
    sets the phase state without exiting openloop
*/
static StateMachine_State_T * OpenLoop_PhaseOutput(Motor_T * p_motor, state_machine_value_t phaseState)
{
    Motor_FOC_ClearFeedbackState(p_motor);
    Phase_ActivateOutputState(&p_motor->Phase, (Phase_Output_T)phaseState); // clear outputState V, or enable last duty
    return &MOTOR_STATE_OPEN_LOOP;
}

void Motor_OpenLoop_SetPhaseOutput(Motor_T * p_motor, Phase_Output_T phase)
{
    static const StateMachine_TransitionInput_T OPEN_LOOP_CMD_PHASE = { .P_VALID = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (StateMachine_Input_T)OpenLoop_PhaseOutput, };
    StateMachine_InvokeBranchTransition(&p_motor->StateMachine, &OPEN_LOOP_CMD_PHASE, phase);
}

/*
*/
static StateMachine_State_T * OpenLoop_Jog(Motor_T * p_motor, state_machine_value_t direction)
{
    Ramp_SetOutputState(&p_motor->TorqueRamp, Ramp_GetTarget(&p_motor->TorqueRamp));
    p_motor->ElectricalAngle = Phase_AngleOf(Phase_JogNext(&p_motor->Phase, Ramp_GetTarget(&p_motor->TorqueRamp)));
    return &MOTOR_STATE_OPEN_LOOP;
}

void Motor_OpenLoop_SetJog(Motor_T * p_motor, int8_t direction)
{
    static const StateMachine_TransitionInput_T OPEN_LOOP_CMD_JOG = { .P_VALID = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (StateMachine_Input_T)OpenLoop_Jog, };
    StateMachine_InvokeBranchTransition(&p_motor->StateMachine, &OPEN_LOOP_CMD_JOG, direction);
}

/******************************************************************************/
/*
    Open Loop SubStates/Cmds
*/
/******************************************************************************/
/*
    Angle Cmd and User Current/Voltage Cmd
*/
static const StateMachine_State_T OPEN_LOOP_STATE_ANGLE_ALIGN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_ROOT     = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)Motor_FOC_StartAlignCmd,
    .LOOP       = (StateMachine_Function_T)Motor_FOC_ProcAlignCmd,
    .NEXT       = NULL,
};

static StateMachine_State_T * OpenLoop_AngleAlign(Motor_T * p_motor, state_machine_value_t angle)
{
    // p_motor->ElectricalAngle = angle;
    Motor_SetElecAngleFeedforward(p_motor, angle);
    return &OPEN_LOOP_STATE_ANGLE_ALIGN;
}

void Motor_OpenLoop_SetAngleAlign(Motor_T * p_motor, angle16_t angle)
{
    static const StateMachine_TransitionInput_T OPEN_LOOP_CMD_ANGLE_ALIGN = { .P_VALID = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (StateMachine_Input_T)OpenLoop_AngleAlign, };
    StateMachine_InvokeBranchTransition(&p_motor->StateMachine, &OPEN_LOOP_CMD_ANGLE_ALIGN, angle);
}

void Motor_OpenLoop_SetAngleAlign_Phase(Motor_T * p_motor, Phase_Id_T align)
{
    Motor_OpenLoop_SetAngleAlign(p_motor, Phase_AngleOf(align));
}

/******************************************************************************/
/*
    Phase Align. With or without I Feedback
    V only for now
*/
/******************************************************************************/
/* User Cmd ramp */
/* ElectricalAngle set by caller  */
void Motor_StartAlignCmd(Motor_T * p_motor)
{
    /* ElectricalAngle set by caller  */
    Phase_ActivateOutput(&p_motor->Phase); /* reset the voltage to start at 0 */
    Ramp_Set(&p_motor->TorqueRamp, p_motor->Config.AlignTime_Cycles, 0, p_motor->Config.AlignPower_Fract16);
}

void Motor_StartAlignPreset(Motor_T * p_motor)
{
    Phase_ActivateOutput(&p_motor->Phase);
    Ramp_Set(&p_motor->OpenLoopIRamp, p_motor->Config.AlignTime_Cycles, 0, p_motor->Config.AlignPower_Fract16);
}

void Motor_ProcPhaseAlign(Motor_T * p_motor)
{
    // // Motor_IReqLimitOf
    // if (CaptureIabc)
    // PID_ProcPI(&p_motor->IPhase, FOC_GetIq(&p_motor->Foc), Ramp_ProcOutput(&p_motor->TorqueRamp));
    // Phase_Align(&p_motor->Phase, Phase_ReadAlign(&p_motor->Phase), Ramp_ProcOutput(&p_motor->TorqueRamp)); /* limited by p_motor->Config.AlignPower_Fract16 */
}

static const StateMachine_State_T OPEN_LOOP_STATE_PHASE_ALIGN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_ROOT     = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)Motor_StartAlignCmd,
    .LOOP       = (StateMachine_Function_T)Motor_ProcPhaseAlign,
    .NEXT       = NULL,
};

static StateMachine_State_T * OpenLoop_PhaseAlign(Motor_T * p_motor, state_machine_value_t phaseAlign)
{
    Phase_ActivateOutput(&p_motor->Phase);

    if(p_motor->FeedbackMode.Current == 1U)
    {
        Phase_Align(&p_motor->Phase, (Phase_Id_T)phaseAlign, Ramp_ProcOutput(&p_motor->TorqueRamp)); /* limited by p_motor->Config.AlignPower_Fract16 */
    }
    else
    {
        /* Set state variables for User Interface consistency */
        Ramp_SetOutputState(&p_motor->TorqueRamp, Ramp_GetTarget(&p_motor->TorqueRamp));
        Phase_Align(&p_motor->Phase, (Phase_Id_T)phaseAlign, Ramp_GetTarget(&p_motor->TorqueRamp));
    }

    Motor_SetElecAngleFeedforward(p_motor, Phase_AngleOf((Phase_Id_T)phaseAlign));

    return &MOTOR_STATE_OPEN_LOOP;
}

void Motor_OpenLoop_SetPhaseAlign(Motor_T * p_motor, Phase_Id_T align)
{
    static const StateMachine_TransitionInput_T OPEN_LOOP_CMD_V_ALIGN = { .P_VALID = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (StateMachine_Input_T)OpenLoop_PhaseAlign, };
    StateMachine_InvokeBranchTransition(&p_motor->StateMachine, &OPEN_LOOP_CMD_V_ALIGN, align);
}


/*
*/
static const StateMachine_State_T OPEN_LOOP_STATE_RUN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_ROOT     = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)Motor_FOC_StartOpenLoop,
    .LOOP       = (StateMachine_Function_T)Motor_FOC_ProcOpenLoop,
    .NEXT       = NULL,
};


/******************************************************************************/
/*
    Run Chain
*/
/******************************************************************************/
static StateMachine_State_T * OpenLoop_StartUpAlignEnd(Motor_T * p_motor)
{
    /* todo if algin b */
    return (Timer_Periodic_Poll(&p_motor->ControlTimer) == true) ? &OPEN_LOOP_STATE_RUN : NULL;
}

/* Align with Aux Ramp */
static const StateMachine_State_T OPEN_LOOP_STATE_START_UP_ALIGN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_ROOT     = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)Motor_FOC_StartStartUpAlign,
    .LOOP       = (StateMachine_Function_T)Motor_FOC_ProcStartUpAlign,
    .NEXT       = (StateMachine_InputVoid_T)OpenLoop_StartUpAlignEnd,
};

// test branch
// static const StateMachine_State_T OPEN_LOOP_STATE_RUN_START_UP =
// {
//     // .ID         = MSM_STATE_ID_OPEN_LOOP,
//     .P_PARENT   = &OPEN_LOOP_STATE_START_UP_ALIGN,
//     .DEPTH      = 2U,
//     .ENTRY      = (StateMachine_Function_T)OpenLoop_StartUpAlign_EntryTimer,
//     .NEXT       = (StateMachine_InputVoid_T)OpenLoop_StartUpAlign_Transition,
// };

static StateMachine_State_T * OpenLoop_StartUpRun(Motor_T * p_motor, state_machine_value_t null)
{
    Motor_FOC_ActivateOutputZero(p_motor); //in case it has been disabled
    return &OPEN_LOOP_STATE_START_UP_ALIGN;
}

void Motor_OpenLoop_StartRunChain(Motor_T * p_motor)
{
    static const StateMachine_TransitionInput_T OPEN_LOOP_CMD_RUN = { .P_VALID = &MOTOR_STATE_OPEN_LOOP, .TRANSITION = (StateMachine_Input_T)OpenLoop_StartUpRun, };
    StateMachine_InvokeBranchTransition(&p_motor->StateMachine, &OPEN_LOOP_CMD_RUN, 0);
}





/******************************************************************************/
/*!
    @brief Calibration State
*/
/******************************************************************************/
static void Calibration_Entry(Motor_T * p_motor)
{
    p_motor->ControlTimerBase = 0U;
    p_motor->CalibrationStateIndex = 0U;
    Phase_ActivateOutputV0(&p_motor->Phase);
    _StateMachine_EndSubState(&p_motor->StateMachine);
}

static void Calibration_Proc(Motor_T * p_motor)
{
    _StateMachine_ProcBranch_Nested(&p_motor->StateMachine);
}

static StateMachine_State_T * Calibration_Next(Motor_T * p_motor)
{
    return (p_motor->FaultFlags.Value != 0U) ? &MOTOR_STATE_FAULT : NULL;
}

/* Calibration State Exit with Direction == 0 and InputCalibration(STOP) */
static StateMachine_State_T * Calibration_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
{
    StateMachine_State_T * p_nextState = NULL;

    switch ((Phase_Mode_T)phaseMode)
    {
        case PHASE_OUTPUT_FLOAT:    Phase_Float(&p_motor->Phase);               _StateMachine_EndSubState(&p_motor->StateMachine);      break;
        case PHASE_OUTPUT_V0:       Phase_ActivateOutputV0(&p_motor->Phase);     _StateMachine_EndSubState(&p_motor->StateMachine);     break;
        case PHASE_OUTPUT_VPWM:     break;
    }

    return p_nextState;
}

static StateMachine_State_T * Calibration_InputStop(Motor_T * p_motor, state_machine_value_t direction)
{
    return (direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_STOP : NULL;
}

/*
    Passing a substate must be called with SubState Input Function
*/
static StateMachine_State_T * Calibration_InputCalibration(Motor_T * p_motor, state_machine_value_t statePtr)
{
    StateMachine_State_T * p_state = (StateMachine_State_T *)statePtr;

    assert(p_state == NULL ||  p_state == &MOTOR_STATE_STOP || p_state == &MOTOR_STATE_CALIBRATION || p_state->P_ROOT == &MOTOR_STATE_CALIBRATION);

    // if (p_state == &MOTOR_STATE_STOP) { _StateMachine_EndSubState(&p_motor->StateMachine); }
    return p_state;
}

static const StateMachine_Input_T CALIBRATION_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
    [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)Calibration_InputControl,
    [MSM_INPUT_CALIBRATION]     = (StateMachine_Input_T)Calibration_InputCalibration,
    [MSM_INPUT_DIRECTION]       = (StateMachine_Input_T)Calibration_InputStop,
    [MSM_INPUT_FEEDBACK_MODE]   = NULL,
    [MSM_INPUT_OPEN_LOOP]       = NULL,
};

const StateMachine_State_T MOTOR_STATE_CALIBRATION =
{
    .ID                 = MSM_STATE_ID_CALIBRATION,
    .P_TRANSITION_TABLE = &CALIBRATION_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Calibration_Entry,
    .LOOP               = (StateMachine_Function_T)Calibration_Proc,
};

/******************************************************************************/
/*!  */
/******************************************************************************/
/*
    with position sensor without position feedback loop
*/
static void Calibration_HomeEntry(Motor_T * p_motor)
{
    // for now
    p_motor->ControlTimerBase = 0U;
    p_motor->CalibrationStateIndex = 0U;
    p_motor->FeedbackMode.Current = 0U;

    Phase_ActivateOutput(&p_motor->Phase);
    Timer_StartPeriod_Millis(&p_motor->ControlTimer, 20); // ~1rpm
    Ramp_Set(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopIRamp_Cycles, 0, Motor_DirectionalValueOf(p_motor, p_motor->Config.OpenLoopIFinal_Fract16));

    p_motor->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    p_motor->MechanicalAngle = Motor_GetMechanicalAngle(p_motor);

    Motor_CommutationModeFn_Call(p_motor, Motor_FOC_SetDirection_Cast, Motor_SetDirection_Cast, MOTOR_DIRECTION_CCW);
}

// todo step speed calc
static void Calibration_ProcHome(Motor_T * p_motor)
{
    // uint16_t angleDelta = Encoder_GetHomingAngle(&p_motor->Encoder); // * direction
    /* alternatively use openloop speed */
    uint16_t angleDelta = 65536/1000;

    // MotorSensor_GetMechanicalAngle(p_motor->Sensor) get direction

    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        Motor_PollSensorAngle(p_motor); /*  */

        // p_motor->ElectricalAngle = (Motor_GetMechanicalAngle(p_motor) + angleDelta) * p_motor->Config.PolePairs;
        p_motor->ElectricalAngle += (angleDelta * p_motor->Config.PolePairs);
        // Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, Ramp_ProcOutput(&p_motor->AuxRamp), 0);
        Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, p_motor->Config.OpenLoopIFinal_Fract16 * 2, 0);
    }
}

static StateMachine_State_T * Calibration_HomeEnd(Motor_T * p_motor)
{
    StateMachine_State_T * p_nextState = NULL;
    uint16_t angleDelta = 65536 / 1000;

    /* error on full rev todo */
    if (angle16_cycle(Motor_GetMechanicalAngle(p_motor), Motor_GetMechanicalAngle(p_motor) + angleDelta, (p_motor->Direction == MOTOR_DIRECTION_CCW)) == true)
    {
        Phase_ActivateOutputV0(&p_motor->Phase);
        _StateMachine_EndSubState(&p_motor->StateMachine);
        p_nextState = &MOTOR_STATE_CALIBRATION;
    }

    p_motor->MechanicalAngle = Motor_GetMechanicalAngle(p_motor);

    return NULL;
}

static const StateMachine_State_T CALIBRATION_STATE_HOMING =
{
    // .ID         = MSM_STATE_ID_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .P_ROOT     = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)Calibration_HomeEntry,
    .LOOP       = (StateMachine_Function_T)Calibration_ProcHome,
    .NEXT       = (StateMachine_InputVoid_T)Calibration_HomeEnd,
};

static StateMachine_State_T * Calibration_StartHome(Motor_T * p_motor, state_machine_value_t null)
{
    // if (MotorSensor_IsAngleHomeSet(p_motor->Sensor) == false) { return &MOTOR_STATE_CALIBRATION; }
    return &CALIBRATION_STATE_HOMING;
}


/* Transition from any Calibration State */
void Motor_Calibration_StartHome(Motor_T * p_motor)
{
    static const StateMachine_TransitionInput_T CALIBRATION_STATE_HOMING_TRANSITION =
        { .P_VALID = &MOTOR_STATE_CALIBRATION, .TRANSITION = (StateMachine_Input_T)Calibration_StartHome, };

    StateMachine_InvokeBranchTransition(&p_motor->StateMachine, &CALIBRATION_STATE_HOMING_TRANSITION, 0U);
}


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Fault_Entry(Motor_T * p_motor) { Phase_Float(&p_motor->Phase); }

static void Fault_Proc(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    /* If auto exit Fault State */
    // if (p_motor->FaultFlags.Value == 0U) { _StateMachine_SetState(&p_motor->StateMachine, &MOTOR_STATE_STOP); }
}

// static StateMachine_State_T * Fault_Next(Motor_T * p_motor)
// {
//     return (p_motor->FaultFlags.Value == 0U) ? &MOTOR_STATE_STOP : NULL;
// }

static StateMachine_State_T * Fault_InputClearFault(Motor_T * p_motor, state_machine_value_t faultFlags)
{
    // p_motor->FaultFlags.Value &= ~faultFlags;
    // p_motor->FaultFlags.Value = 0U;
    // p_motor->FaultFlags.PositionSensor = !Motor_VerifySensorCalibration(p_motor); // non polling check
    Motor_PollFaultFlags(p_motor);

    return (p_motor->FaultFlags.Value == 0U) ? &MOTOR_STATE_STOP : NULL;
}

static StateMachine_State_T * Fault_InputCalibration(Motor_T * p_motor, state_machine_value_t state)
{
    StateMachine_State_T * p_nextState = NULL;

    if (p_motor->FaultFlags.Overheat == 0U)
    {
        // p_motor->CalibrationState = state;
        p_nextState = &MOTOR_STATE_CALIBRATION;
    }

    return p_nextState;
}

static const StateMachine_Input_T FAULT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]       = (StateMachine_Input_T)Fault_InputClearFault,
    [MSM_INPUT_CALIBRATION] = (StateMachine_Input_T)Fault_InputCalibration,
    [MSM_INPUT_DIRECTION]   = NULL,
};

const StateMachine_State_T MOTOR_STATE_FAULT =
{
    .ID                 = MSM_STATE_ID_FAULT,
    .P_TRANSITION_TABLE = &FAULT_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Fault_Entry,
    .LOOP               = (StateMachine_Function_T)Fault_Proc,
};


/******************************************************************************/
/* Fault interface functions */
/******************************************************************************/

bool Motor_StateMachine_IsFault(const Motor_T * p_motor) { return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT); }

void Motor_StateMachine_EnterFault(Motor_T * p_motor)
{
    if (Motor_StateMachine_IsFault(p_motor) == false) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, 0U); }
}

/*! @return true if no fault remains */
bool Motor_StateMachine_ExitFault(Motor_T * p_motor)
{
    if (Motor_StateMachine_IsFault(p_motor) == true) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, 0U); }
    return Motor_StateMachine_IsFault(p_motor);
}

void Motor_StateMachine_SetFault(Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    p_motor->FaultFlags.Value |= faultFlags.Value;
    if (Motor_StateMachine_IsFault(p_motor) == false) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, faultFlags.Value); }
}

/*! @return true if cleared applies, fault to non fault */
bool Motor_StateMachine_ClearFault(Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    bool isFault = Motor_StateMachine_IsFault(p_motor);
    if (Motor_StateMachine_IsFault(p_motor) == true) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, faultFlags.Value); }
    return (Motor_StateMachine_IsFault(p_motor) != isFault);
}
