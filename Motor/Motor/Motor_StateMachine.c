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
    @file   Motor_StateMachine.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Motor_StateMachine.h"


/******************************************************************************/
/*!
    @brief State Machine
*/
/******************************************************************************/
const StateMachine_Machine_T MSM_MACHINE =
{
    .P_STATE_INITIAL = &MOTOR_STATE_INIT,
    .P_STATE_FAULT = &MOTOR_STATE_FAULT,
    .TRANSITION_TABLE_LENGTH = MSM_TRANSITION_TABLE_LENGTH,
};

/******************************************************************************/
/*!
    Common
*/
/******************************************************************************/
static State_T * TransitionFault(const Motor_T * p_motor, state_value_t faultFlags) { return &MOTOR_STATE_FAULT; }

static void Motor_ClearFaultFlags(const Motor_T * p_motor)
{
    // p_motor->P_MOTOR_STATE->FaultFlags.Overheat = HeatMonitor_IsFault(&p_motor->P_MOTOR_STATE->Thermistor);
    p_motor->P_MOTOR_STATE->FaultFlags.PositionSensor = !RotorSensor_VerifyCalibration(p_motor->P_MOTOR_STATE->p_ActiveSensor);
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
static void Init_Entry(const Motor_T * p_motor)
{
    /* alternatively null until a direction is set */
    /* Sets Speed/I Limits using direction */
    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_SetDirectionForward, Motor_SetDirectionForward);

    // Motor_Sensor_ResetUnits(p_motor->P_MOTOR_STATE );
}

static void Init_Proc(const Motor_T * p_motor)
{
    // Motor_FOC_ProcCaptureAngleVBemf(p_motor->P_MOTOR_STATE);
    // Motor_Sensor_PollCaptureSpeed(p_motor);
}

static State_T * Init_Next(const Motor_T * p_motor)
{
    State_T * p_nextState = NULL;

    if (SysTime_GetMillis() > MOTOR_STATE_MACHINE_INIT_WAIT) /* wait for Speed and Heat sensors */
    {
        // (Motor_CheckConfig() == true)    // check params, sync config limits
        if (Phase_Calibration_IsLoaded() == false) { p_motor->P_MOTOR_STATE->FaultFlags.InitCheck = 1U; } /* alternatively go to fault, outter module parse */
        p_motor->P_MOTOR_STATE->FaultFlags.PositionSensor = !RotorSensor_VerifyCalibration(p_motor->P_MOTOR_STATE->p_ActiveSensor);
        // if (Motor_ValidateIabcZeroRef(p_motor) == false) { p_motor->P_MOTOR_STATE->FaultFlags.InitCheck = 1U; }
        // p_motor->P_MOTOR_STATE->FaultFlags.Overheat = HeatMonitor_IsFault(&p_motor->P_MOTOR_STATE->Thermistor);

        // Motor_ClearFaultFlags(p_motor); /* Clear the fault flags once */

        if (p_motor->P_MOTOR_STATE->FaultFlags.Value != 0U) { p_nextState = &MOTOR_STATE_FAULT; }
        else                                                { p_nextState = &MOTOR_STATE_STOP; }
    }

    return p_nextState;
}

static const State_Input_T INIT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = NULL,
    [MSM_INPUT_DIRECTION]       = NULL,
    [MSM_INPUT_CALIBRATION]     = NULL,
};

const State_T MOTOR_STATE_INIT =
{
    .ID                 = MSM_STATE_ID_INIT,
    .ENTRY              = (State_Action_T)Init_Entry,
    .LOOP               = (State_Action_T)Init_Proc,
    .NEXT               = (State_InputVoid_T)Init_Next,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
};


// static inline const Motor_T * Motor_GetContext(const StateMachine_T * p_StateMachine) { return (const Motor_T *)p_StateMachine->P_CONTEXT; }


/******************************************************************************/
/*!
    @brief Stop State

    Safe State with Transition to Calibration
    Phase OutputFloat or V0
*/
/******************************************************************************/
static void Stop_Entry(const Motor_T * p_motor)
{
    Phase_Float(&p_motor->PHASE);
    // _StateMachine_EndSubState(p_motor->STATE_MACHINE.P_ACTIVE);
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR_STATE);
    // Motor_CommutationModeFn_Call(p_motor->P_MOTOR_STATE, Motor_FOC_ClearFeedbackState, NULL); /* Unobserved values remain 0 for user read */
    p_motor->P_MOTOR_STATE->ControlTimerBase = 0U; /* ok to reset timer */
}

static void Stop_Proc(const Motor_T * p_motor)
{
    // Motor_PollCaptureSensor(p_motor->P_MOTOR_STATE); /* altneratively move to thread */
    Motor_FOC_ProcCaptureAngleVBemf(p_motor->P_MOTOR_STATE);
    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL);
}

static State_T * Stop_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;

    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_OUTPUT_FLOAT:    Phase_Float(&p_motor->PHASE);               break;
        case PHASE_OUTPUT_V0:       Phase_ActivateOutputV0(&p_motor->PHASE);    break;
        case PHASE_OUTPUT_VPWM:         break;
        default: break;
    }

    return p_nextState;
}

/*
    Transition to Ready State on Direction only
*/
/* may need to immediately transition */
static State_T * Stop_InputDirection(const Motor_T * p_motor, state_value_t direction)
{
    State_T * p_nextState = NULL;

    switch ((Motor_Direction_T)direction)
    {
        case MOTOR_DIRECTION_NULL: p_nextState = NULL; break; /* remain in stop */
        case MOTOR_DIRECTION_CW:  /* Intentional fall-through */
        case MOTOR_DIRECTION_CCW:
            Motor_FOC_SetDirection(p_motor->P_MOTOR_STATE, direction);
            p_nextState = &MOTOR_STATE_PASSIVE;
            break;
        default: break; /* Invalid direction */
    }

    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_SetDirection_Cast, Motor_SetDirection_Cast, direction);
    return p_nextState;
}

static State_T * Stop_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode);
    return NULL;
}

/* Transition for user input */
// static State_T * Stop_InputOpenLoop(const Motor_T * p_motor, state_value_t state)
// {
//     return &MOTOR_STATE_OPEN_LOOP;
// }

/* Calibration go directly to SubState */
static State_T * Stop_InputCalibration(const Motor_T * p_motor, state_value_t statePtr)
{
    State_T * p_state = (State_T *)statePtr;

    assert(p_state == &MOTOR_STATE_STOP || p_state == &MOTOR_STATE_CALIBRATION || p_state->P_TOP == &MOTOR_STATE_CALIBRATION);

    return p_state;
}

static const State_Input_T STOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)Stop_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (State_Input_T)Stop_InputFeedbackMode,
    [MSM_INPUT_DIRECTION]       = (State_Input_T)Stop_InputDirection,
    [MSM_INPUT_CALIBRATION]     = (State_Input_T)Stop_InputCalibration,
    [MSM_INPUT_OPEN_LOOP]       = NULL,
    // [MSM_INPUT_OPEN_LOOP]    = (State_Input_T)Stop_InputOpenLoop,
};

const State_T MOTOR_STATE_STOP =
{
    .ID                 = MSM_STATE_ID_STOP,
    .ENTRY              = (State_Action_T)Stop_Entry,
    .LOOP               = (State_Action_T)Stop_Proc,
    .P_TRANSITION_TABLE = &STOP_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief Passive State - Freewheel or Hold
*/
/******************************************************************************/
static void Passive_Entry(const Motor_T * p_motor)
{
    assert(p_motor->P_MOTOR_STATE->Direction != MOTOR_DIRECTION_NULL); /* Set on exit stop */
    Phase_Float(&p_motor->PHASE);
    // _StateMachine_EndSubState(p_motor->STATE_MACHINE.P_ACTIVE);
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR_STATE);
    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ClearFeedbackState, NULL);
    p_motor->P_MOTOR_STATE->ControlTimerBase = 0U; /* ok to reset timer */
}

static void Passive_Proc(const Motor_T * p_motor)
{
    /* Match Feedback to ProcAngleBemf on Resume */
    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL /* Motor_SixStep_ProcPhaseObserve */);
    Motor_FOC_ProcCaptureAngleVBemf(p_motor->P_MOTOR_STATE);
    // Motor_PollCaptureSensor(p_motor->P_MOTOR_STATE); /* alternatively move to thread */
}

// auto return to stop state
// static State_T * Passive_Next(const Motor_T * p_motor)
// {
//     // return (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0) ? &MOTOR_STATE_STOP : NULL;
//     // if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) > 0)
//     // {
//     //     if (Motor_IsHold(p_motor) == false) { _StateMachine_Transition(&p_motor->STATE_MACHINE, &MOTOR_STATE_FREEWHEEL); }
//     //     else                                { }
//     // }
// }

/* alternatively separate state for speed 0 */
static State_T * Passive_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;

    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_OUTPUT_FLOAT:
            Phase_Float(&p_motor->PHASE);
            break;
        case PHASE_OUTPUT_V0:
            /* Effectively Run cmd V0, without Ramp/Feedback Loop */
            // if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) < ) { Phase_ActivateOutputV0(&p_motor->PHASE); }
            // if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U) { Phase_ActivateOutputV0(&p_motor->PHASE); }
            break;
        case PHASE_OUTPUT_VPWM:
            if (p_motor->P_MOTOR_STATE->Direction != MOTOR_DIRECTION_NULL)
            {
                if (RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR_STATE->p_ActiveSensor) == true)
                {
                    // RotorSensor_ZeroInitial(p_motor->P_MOTOR_STATE->p_ActiveSensor); not needed if capture sensor runs in thread
                    p_nextState = &MOTOR_STATE_RUN;
                }
                else if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U) /* OpenLoop start at 0 speed */
                {
                    p_nextState = &MOTOR_STATE_OPEN_LOOP;
                    /* p_nextState = Motor_Sensor_GetStartUpState(p_motor); */ /* get substate */
                    // p_nextState = &OPEN_LOOP_STATE_START_UP_ALIGN; /* Motor_GetSensorStartUpState() */
                }
            }
            break;
    }

    return p_nextState;
}

/* caller handle input validation */

static State_T * Passive_InputDirection(const Motor_T * p_motor, state_value_t direction)
{
    State_T * p_nextState = NULL;

    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U)
    {
        /* direction 0 as return to stop */
        switch ((Motor_Direction_T)direction)
        {
            case MOTOR_DIRECTION_NULL: p_nextState = &MOTOR_STATE_STOP;  /* Intentional fall-through */
            case MOTOR_DIRECTION_CW:
            case MOTOR_DIRECTION_CCW:
                Motor_FOC_SetDirection(p_motor->P_MOTOR_STATE, (Motor_Direction_T)direction);
                break;
            default: break; /* Invalid direction */
        }
    }

    return p_nextState;
}

static State_T * Passive_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode);
    return NULL;
}

static State_T * Passive_InputOpenLoop(const Motor_T * p_motor, state_value_t state)
{
    return &MOTOR_STATE_OPEN_LOOP; /* User OpenLoop */
}

static const State_Input_T PASSIVE_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)Passive_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (State_Input_T)Passive_InputFeedbackMode,
    [MSM_INPUT_DIRECTION]       = (State_Input_T)Passive_InputDirection,
    [MSM_INPUT_OPEN_LOOP]       = (State_Input_T)Passive_InputOpenLoop,
    [MSM_INPUT_CALIBRATION]     = NULL,
};

const State_T MOTOR_STATE_PASSIVE =
{
    .ID                 = MSM_STATE_ID_PASSIVE,
    .ENTRY              = (State_Action_T)Passive_Entry,
    .LOOP               = (State_Action_T)Passive_Proc,
    .P_TRANSITION_TABLE = &PASSIVE_TRANSITION_TABLE[0U],
};

/* */
// const State_T MOTOR_STATE_PASSIVE_HOLD =
// {
//     .P_PARENT = &MOTOR_STATE_PASSIVE,
// }
// const State_T MOTOR_STATE_PASSIVE_FREEWHEEL =
// {
//     .P_PARENT = &MOTOR_STATE_PASSIVE,
// }

/******************************************************************************/
/*!
    @brief Run State
    Spin with Feedback State
    Active Control, FeedbackLoop is in effect
        UserCmd => RampOutput => PID => AngleControl
*/
/******************************************************************************/
static void Run_Entry(const Motor_T * p_motor)
{
    // Motor_CommutationModeFn_Call(p_motor->P_MOTOR_STATE, Motor_FOC_MatchFeedbackState, NULL);
    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ActivateOutput, NULL);
    // _StateMachine_EndSubState(p_motor->STATE_MACHINE.P_ACTIVE); /* including substate of another state */
    Motor_FOC_MatchFeedbackState(p_motor->P_MOTOR_STATE);
    Phase_ActivateOutputT0(&p_motor->PHASE);
    /* alterntaively Update Vbus on start Angle Control */
}

static void Run_Proc(const Motor_T * p_motor)
{
    Motor_ProcOuterFeedback(p_motor->P_MOTOR_STATE);
    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcAngleControl, NULL/* Motor_SixStep_ProcPhaseControl */);
    Motor_FOC_ProcAngleControl(p_motor->P_MOTOR_STATE);
    // Motor_FOC_WriteDuty(p_motor);
}

// static State_T * Run_Next(const Motor_T * p_motor)
// {
//     State_T * p_nextState = NULL;
//     if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) < ) { p_nextState = &MOTOR_STATE_; }
//     return p_nextState;
// }

static State_T * Run_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;

    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_OUTPUT_FLOAT: p_nextState = &MOTOR_STATE_PASSIVE; break;
        //   (Motor_CheckSpeed(p_motor) == true) ? &MOTOR_STATE_FREEWHEEL : 0U; // check speed range
            // switch(p_motor->P_MOTOR_STATE->FeedbackMode)
        case PHASE_OUTPUT_V0:   break;
        case PHASE_OUTPUT_VPWM: break;
        default: break;
    }

    return p_nextState;
}

static State_T * Run_InputStop(const Motor_T * p_motor, state_value_t direction)
{
    State_T * p_nextState = NULL;
    switch ((Motor_Direction_T)direction)
    {
        case MOTOR_DIRECTION_NULL: p_nextState = &MOTOR_STATE_PASSIVE; break;
        case MOTOR_DIRECTION_CW: break;
        case MOTOR_DIRECTION_CCW: break;
        default: break; /* Invalid direction */
    }
    return p_nextState;
    /* check speed or return to passive */
    // return (direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_PASSIVE : NULL;
    // return (direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_STOP : NULL;
}

/*
todo may need immediate transition
    StateMachine in Sync mode, [ProcInput] in the same thread as [Run_Proc]/[ProcAngleControl]
    Process [Motor_FOC_MatchFeedbackState] before [Motor_FOC_ProcAngleControl]
*/
static State_T * Run_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    State_T * p_nextState = NULL;
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode);
    p_nextState = &MOTOR_STATE_RUN;
    /* Run_Entry Procs synchronous */ /* Alternatively, transition through Freewheel */
    return p_nextState;
}

static const State_Input_T RUN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_FEEDBACK_MODE]   = (State_Input_T)Run_InputFeedbackMode,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)Run_InputControl,
    [MSM_INPUT_DIRECTION]       = (State_Input_T)Run_InputStop,
    [MSM_INPUT_CALIBRATION]     = NULL,
    [MSM_INPUT_OPEN_LOOP]       = NULL,
};

const State_T MOTOR_STATE_RUN =
{
    .ID                 = MSM_STATE_ID_RUN,
    .P_TRANSITION_TABLE = &RUN_TRANSITION_TABLE[0U],
    .ENTRY              = (State_Action_T)Run_Entry,
    .LOOP               = (State_Action_T)Run_Proc,
};



/******************************************************************************/
/*!
    @brief State OpenLoop - OpenLoop, Align, and Start Up, Feedback Acquisition

    Only Entry is from [STOP] State
*/
/******************************************************************************/
static void OpenLoop_Entry(const Motor_T * p_motor)
{
    Phase_ActivateOutputT0(&p_motor->PHASE);
    // Phase_ActivateOutputV0(&p_motor->PHASE);
    FOC_ClearCaptureState(&p_motor->P_MOTOR_STATE->Foc);
    // _StateMachine_EndSubState(p_motor->STATE_MACHINE.P_ACTIVE); /* 'unmount' last operation */
    // determine initial substate from stop
    p_motor->P_MOTOR_STATE->FeedbackMode.OpenLoop = 1U; /* limits user cmd input */
}

/*
    Prco SubState Tree
*/
static void OpenLoop_Proc(const Motor_T * p_motor)
{
    // _StateMachine_ProcBranch_Nested(p_motor->STATE_MACHINE.P_ACTIVE, (void *)p_motor);
}

/* maintain consistent interface with other states, use substate cmd for phase output without exiting */
static State_T * OpenLoop_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;

    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_OUTPUT_FLOAT:    p_nextState = &MOTOR_STATE_PASSIVE;              break;
        case PHASE_OUTPUT_V0:       /* Phase_ActivateOutputV0(&p_motor->PHASE); */   break;
        case PHASE_OUTPUT_VPWM:     /* Phase_ActivateOutputT0(&p_motor->PHASE); */   break;
        /* No resume from OpenLoop, freewheel state check stop */
    }

    // if (p_nextState == &MOTOR_STATE_PASSIVE) { _StateMachine_EndSubState(p_motor->STATE_MACHINE.P_ACTIVE); }
    return p_nextState;
}

static State_T * Openloop_InputStop(const Motor_T * p_motor, state_value_t direction)
{
    // return (direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_PASSIVE : NULL;
    return (direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_STOP : NULL;
}

static State_T * OpenLoop_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode); /* a different flag mode will change ramp limits */
    return (p_motor->P_MOTOR_STATE->FeedbackMode.OpenLoop == 0U) ? &MOTOR_STATE_PASSIVE : NULL;
    // return NULL;
}

/*
    using openloop substate inputs
    multiple cmds per subState. Implemented with [StateMachine_TransitionInput_T]
*/
static State_T * OpenLoop_InputOpenLoop(const Motor_T * p_motor, state_value_t openLoop)
{
    return &MOTOR_STATE_OPEN_LOOP;
}

static const State_Input_T OPEN_LOOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)OpenLoop_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (State_Input_T)OpenLoop_InputFeedbackMode,
    [MSM_INPUT_OPEN_LOOP]       = (State_Input_T)OpenLoop_InputOpenLoop,
    [MSM_INPUT_DIRECTION]       = (State_Input_T)Openloop_InputStop,
    [MSM_INPUT_CALIBRATION]     = NULL,
};

const State_T MOTOR_STATE_OPEN_LOOP =
{
    .ID                 = MSM_STATE_ID_OPEN_LOOP,
    .ENTRY              = (State_Action_T)OpenLoop_Entry,
    .LOOP               = (State_Action_T)OpenLoop_Proc,
    .P_TRANSITION_TABLE = &OPEN_LOOP_TRANSITION_TABLE[0U],
};



/******************************************************************************/
/*!
    @brief Calibration State
*/
/******************************************************************************/
static void Calibration_Entry(const Motor_T * p_motor)
{
    // Phase_ActivateOutputV0(&p_motor->PHASE);
    // _StateMachine_EndSubState(p_motor->STATE_MACHINE.P_ACTIVE);
    p_motor->P_MOTOR_STATE->ControlTimerBase = 0U;
    p_motor->P_MOTOR_STATE->CalibrationStateIndex = 0U;
}

static void Calibration_Proc(const Motor_T * p_motor)
{
    // _StateMachine_ProcBranch_Nested(p_motor->STATE_MACHINE.P_ACTIVE, (void *)p_motor);
}

static State_T * Calibration_Next(const Motor_T * p_motor)
{
    return (p_motor->P_MOTOR_STATE->FaultFlags.Value != 0U) ? &MOTOR_STATE_FAULT : NULL;
}

/* Calibration State Exit with Direction == 0 and InputCalibration(STOP) */
static State_T * Calibration_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;

    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_OUTPUT_FLOAT:    Phase_Float(&p_motor->PHASE);                break;
        case PHASE_OUTPUT_V0:       Phase_ActivateOutputV0(&p_motor->PHASE);     break;
        case PHASE_OUTPUT_VPWM:     break;
    }

    // return p_nextState;
    return &MOTOR_STATE_CALIBRATION; /* Stops the SubState */
}

static State_T * Calibration_InputStop(const Motor_T * p_motor, state_value_t direction)
{
    // Phase_Float(&p_motor->PHASE);
    return (direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_STOP : NULL;
}

/*
    Passing a substate must be called with SubState Input Function
*/
static State_T * Calibration_InputCalibration(const Motor_T * p_motor, state_value_t statePtr)
{
    State_T * p_state = (State_T *)statePtr;

    assert(p_state == NULL || p_state == &MOTOR_STATE_STOP || p_state == &MOTOR_STATE_CALIBRATION || p_state->P_TOP == &MOTOR_STATE_CALIBRATION);

    return p_state;
}

static const State_Input_T CALIBRATION_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)Calibration_InputControl,
    [MSM_INPUT_CALIBRATION]     = (State_Input_T)Calibration_InputCalibration,
    [MSM_INPUT_DIRECTION]       = (State_Input_T)Calibration_InputStop,
    [MSM_INPUT_FEEDBACK_MODE]   = NULL,
    [MSM_INPUT_OPEN_LOOP]       = NULL,
};

const State_T MOTOR_STATE_CALIBRATION =
{
    .ID                 = MSM_STATE_ID_CALIBRATION,
    .P_TRANSITION_TABLE = &CALIBRATION_TRANSITION_TABLE[0U],
    .ENTRY              = (State_Action_T)Calibration_Entry,
    .LOOP               = (State_Action_T)Calibration_Proc,
};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Fault_Entry(const Motor_T * p_motor) { Phase_Float(&p_motor->PHASE); }

static void Fault_Proc(const Motor_T * p_motor) { Phase_Float(&p_motor->PHASE); }

static State_T * Fault_InputClearFault(const Motor_T * p_motor, state_value_t faultFlags)
{
    // p_motor->P_MOTOR_STATE->FaultFlags.Value &= ~faultFlags;
    // p_motor->P_MOTOR_STATE->FaultFlags.Value = 0U;
    // p_motor->P_MOTOR_STATE->FaultFlags.PositionSensor = !Motor_VerifySensorCalibration(p_motor); // non polling check
    Motor_ClearFaultFlags(p_motor);

    return (p_motor->P_MOTOR_STATE->FaultFlags.Value == 0U) ? &MOTOR_STATE_STOP : NULL;
}

static State_T * Fault_InputCalibration(const Motor_T * p_motor, state_value_t state)
{
    State_T * p_nextState = NULL;

    if (p_motor->P_MOTOR_STATE->FaultFlags.Overheat == 0U)
    {
        p_nextState = &MOTOR_STATE_CALIBRATION;
    }

    return p_nextState;
}

static const State_Input_T FAULT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]       = (State_Input_T)Fault_InputClearFault,
    [MSM_INPUT_CALIBRATION] = (State_Input_T)Fault_InputCalibration,
    [MSM_INPUT_DIRECTION]   = NULL,
};

const State_T MOTOR_STATE_FAULT =
{
    .ID                 = MSM_STATE_ID_FAULT,
    .P_TRANSITION_TABLE = &FAULT_TRANSITION_TABLE[0U],
    .ENTRY              = (State_Action_T)Fault_Entry,
    .LOOP               = (State_Action_T)Fault_Proc,
};


/******************************************************************************/
/*
    Fault interface functions
*/
/******************************************************************************/
void Motor_StateMachine_EnterFault(const Motor_T * p_motor)
{
    if (Motor_StateMachine_IsFault(p_motor) == false) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_FAULT, -1); }
}

/*! @return true if no fault remains */
bool Motor_StateMachine_ExitFault(const Motor_T * p_motor)
{
    if (Motor_StateMachine_IsFault(p_motor) == true) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_FAULT, 0); }
    return !Motor_StateMachine_IsFault(p_motor);
}

/*
    individual flag access todo
*/
void Motor_StateMachine_SetFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    if (Motor_StateMachine_IsFault(p_motor) == false) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_FAULT, faultFlags.Value); }
}

/* faultFlags unused for now */
void Motor_StateMachine_ClearFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    if (Motor_StateMachine_IsFault(p_motor) == true) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_FAULT, ~faultFlags.Value); }
}




