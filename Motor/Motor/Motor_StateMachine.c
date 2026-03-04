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
    Motor_T * context for immediate Phase Output update
*/
/******************************************************************************/

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
    // .P_STATES = [
    //     &MOTOR_STATE_INIT,
    //     &MOTOR_STATE_STOP,
    //     &MOTOR_STATE_READY,
    //     &MOTOR_STATE_CALIBRATION,
    //     &MOTOR_STATE_STARTUP,
    //     &MOTOR_STATE_RUN,
    //     &MOTOR_STATE_FAULT,
    // ],
};

/******************************************************************************/
/*!
    Common
*/
/******************************************************************************/
static State_T * TransitionFault(const Motor_T * p_motor, state_value_t faultFlags) { return &MOTOR_STATE_FAULT; }

static void Motor_CaptureFaultFlags(const Motor_T * p_motor)
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
    SysTime_Millis = 0U; /* Reset SysTime in case of reboot */
}

static void Init_Proc(const Motor_T * p_motor)
{
}

static State_T * Init_Next(const Motor_T * p_motor)
{
    State_T * p_nextState = NULL;

    if (SysTime_GetMillis() > MOTOR_STATE_MACHINE_INIT_WAIT) /* wait for Speed and Heat sensors */
    {
        // (Motor_CheckConfig() == true)    // check params, sync config limits
        if (Phase_Calibration_IsLoaded() == false) { p_motor->P_MOTOR_STATE->FaultFlags.InitCheck = 1U; } /* alternatively go to fault, outer module parse */
        // if (Motor_ValidateIabcZeroRef(p_motor) == false) { p_motor->P_MOTOR_STATE->FaultFlags.InitCheck = 1U; }
        p_motor->P_MOTOR_STATE->FaultFlags.PositionSensor = !RotorSensor_VerifyCalibration(p_motor->P_MOTOR_STATE->p_ActiveSensor);
        // Motor_CaptureFaultFlags(p_motor); /* Clear the fault flags once */

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
    .NEXT               = (State_Handler_T)Init_Next,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
};




/******************************************************************************/
/*!
    @brief Stop State

    Safe State with Transition to Calibration
    Phase V0
*/
/******************************************************************************/
/*
    Map to inverter state. -> handles low layer task. app layer handles user cmd semantics.
        may need extra state to safeguard transition to calibration, if stop state is not sufficient.
*/
/*
    Let Direction double as sematics for transition be an extension.
    Direction cannot be used as transition to ready state for app layer to remain in its safe state and also propagate direction set to motor layer.
    Direction null exits to stop. but setting direction from stop does not transition to ready.
        exiting a state on loss of any condition, but entering may not be guaranteed by setting a single condition.
        app layer exit park state may preserve stop state while storing direction in motor state.
    maintains compatibility with none direction based handling.
*/
static void Stop_Entry(const Motor_T * p_motor)
{
    Phase_Deactivate(&p_motor->PHASE);
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR_STATE); // Motor_CommutationModeFn_Call(p_motor->P_MOTOR_STATE, Motor_FOC_ClearFeedbackState, NULL); /* Unobserved values remain 0 for user read */
    Motor_FOC_SetDirection(p_motor->P_MOTOR_STATE, MOTOR_DIRECTION_NULL);
    p_motor->P_MOTOR_STATE->ControlTimerBase = 0U; /* ok to reset timer */
}

static void Stop_Proc(const Motor_T * p_motor)
{
    Motor_FOC_ProcCaptureAngleVBemf(p_motor->P_MOTOR_STATE); // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL);
}

static State_T * Stop_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;
    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_0:      Phase_ActivateV0(&p_motor->PHASE);      break;
        case PHASE_VOUT_Z:      Phase_Deactivate(&p_motor->PHASE);      break;
        case PHASE_VOUT_PWM:    p_nextState = &MOTOR_STATE_PASSIVE;     break;
        default: break;
    }
    return p_nextState;
}

static State_T * Stop_InputDirection(const Motor_T * p_motor, state_value_t direction)
{
    switch ((Motor_Direction_T)direction)
    {
        case MOTOR_DIRECTION_NULL:
        case MOTOR_DIRECTION_CW:
        case MOTOR_DIRECTION_CCW:
            Motor_FOC_SetDirection(p_motor->P_MOTOR_STATE, direction); break;
        default: break; /* Invalid direction */
    }
    // if (direction != MOTOR_DIRECTION_NULL) { p_nextState = &MOTOR_STATE_PASSIVE; }
    return NULL;
}

static State_T * Stop_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode);
    return NULL;
}

/* Transition for user input */
static State_T * Stop_InputOpenLoop(const Motor_T * p_motor, state_value_t state)
{
    // return &MOTOR_STATE_OPEN_LOOP;
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U) { return &MOTOR_STATE_OPEN_LOOP; }
    else { return NULL; }
}

/* Calibration go directly to SubState */
static State_T * Stop_InputCalibration(const Motor_T * p_motor, state_value_t statePtr)
{
    // State_T * p_state = (State_T *)statePtr;
    // /* Compile time known */
    // assert(p_state == &MOTOR_STATE_STOP || p_state == &MOTOR_STATE_CALIBRATION || p_state->P_TOP == &MOTOR_STATE_CALIBRATION);
    // return &MOTOR_STATE_CALIBRATION;
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U) { return &MOTOR_STATE_CALIBRATION; }
    else { return NULL; }
}

static const State_Input_T STOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)Stop_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (State_Input_T)Stop_InputFeedbackMode,
    [MSM_INPUT_DIRECTION]       = (State_Input_T)Stop_InputDirection,
    [MSM_INPUT_CALIBRATION]     = (State_Input_T)Stop_InputCalibration, /*  */
    [MSM_INPUT_OPEN_LOOP]       = NULL, // [MSM_INPUT_OPEN_LOOP]    = (State_Input_T)Stop_InputOpenLoop,
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
/*
    V0 passive and VZ passive require different entries. both "control loop off." they can't share an entry action.
    for each state to be represented by an invariant entry. the states should be separate for cleaner abstraction or parameterize entry.
    optionally substate with entry action.
    however transition to v0 passive has the same constraints as transition to stop. does app layer need to directly transition to v0 without going through vz first?
    if not, then the entry action can be shared as vz default. allow remaining in passive for v0
*/
static void Passive_Entry(const Motor_T * p_motor)
{
    assert(p_motor->P_MOTOR_STATE->Direction != MOTOR_DIRECTION_NULL); /* Set on exit stop */
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) < Motor_GetSpeedRated_Fract16(p_motor->P_MOTOR_STATE)) { Phase_Deactivate(&p_motor->PHASE); }
    else { Phase_ActivateV0(&p_motor->PHASE); }
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR_STATE); // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ClearFeedbackState, NULL);
    p_motor->P_MOTOR_STATE->ControlTimerBase = 0U; /* ok to reset timer */

}

static void Passive_Proc(const Motor_T * p_motor)
{
    /* Match Feedback to ProcAngleBemf on Resume */
    Motor_FOC_ProcCaptureAngleVBemf(p_motor->P_MOTOR_STATE);    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL /* Motor_SixStep_ProcPhaseObserve */);
}

static State_T * Passive_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;

    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_Z:
            if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) < Motor_GetSpeedRated_Fract16(p_motor->P_MOTOR_STATE)) { Phase_Deactivate(&p_motor->PHASE); }
            break;
        case PHASE_VOUT_0:
            Phase_ActivateV0(&p_motor->PHASE);
            break;
        case PHASE_VOUT_PWM:
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
                // else no transition
            }
            break;
    }

    return p_nextState;
}

static State_T * Passive_InputDirection(const Motor_T * p_motor, state_value_t direction)
{
    State_T * p_nextState = NULL;

    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U)
    {
        switch ((Motor_Direction_T)direction)
        {
            case MOTOR_DIRECTION_NULL: /* Intentional fall-through */
            case MOTOR_DIRECTION_CW:
            case MOTOR_DIRECTION_CCW:
                Motor_FOC_SetDirection(p_motor->P_MOTOR_STATE, direction); break;
            default: break; /* Invalid direction */
        }
        /* If set during motor spinning. Rotor sensor maintain separate direction for interpolation */
        /* Null direction can function as stop while other directions not function as start. */
        if (direction == MOTOR_DIRECTION_NULL) { p_nextState = &MOTOR_STATE_STOP; }
    }
    // alternatively set let application layer handle permission for cw/ccw, updates plugging limits

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
    Motor_FOC_MatchFeedbackState(p_motor->P_MOTOR_STATE);    // Motor_CommutationModeFn_Call(p_motor->P_MOTOR_STATE, Motor_FOC_MatchFeedbackState, NULL);
    Phase_ActivateT0(&p_motor->PHASE);    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ActivateOutput, NULL);
    /* alternatively Update Vbus on start Angle Control */
    // if caller handles rotor direction mismatch
    // if (Motor_GetRotorDirection(p_motor) != p_state->Direction)
    // {
    //     // Runtime reversal detected – update applied direction for control
    // }

    // ProcAngleOutput
    // Motor_FOC_WriteDuty(p_motor);
    // Phase_ActivateOutput(&p_motor->PHASE);
}

static void Run_Proc(const Motor_T * p_motor)
{
    Motor_ProcOuterFeedback(p_motor->P_MOTOR_STATE);
    Motor_FOC_ProcAngleControl(p_motor->P_MOTOR_STATE); // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcAngleControl, NULL/* Motor_SixStep_ProcPhaseControl */);
    Motor_FOC_WriteDuty(p_motor);
}

static State_T * Run_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;

    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_Z:      p_nextState = &MOTOR_STATE_PASSIVE; break;
        case PHASE_VOUT_0:      p_nextState = &MOTOR_STATE_PASSIVE; break;
        case PHASE_VOUT_PWM:     break;
        default: break;
    }

    return p_nextState;
}

static State_T * Run_InputStop(const Motor_T * p_motor, state_value_t direction)
{
    State_T * p_nextState = NULL;
    switch ((Motor_Direction_T)direction)
    {
        case MOTOR_DIRECTION_NULL: p_nextState = &MOTOR_STATE_PASSIVE; break; /*   (Motor_CheckSpeed(p_motor) == true) */
        case MOTOR_DIRECTION_CW:    break;
        case MOTOR_DIRECTION_CCW:   break;
        default: break; /* Invalid direction */
    }
    return p_nextState;
    // if (direction == MOTOR_DIRECTION_NULL) { p_nextState = &MOTOR_STATE_PASSIVE; }
}

/*
    StateMachine in Sync mode, [ProcInput] in the same thread as [Run_Proc]/[ProcAngleControl]
    Process [Motor_FOC_MatchFeedbackState] before [Motor_FOC_ProcAngleControl]
*/
static State_T * Run_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode);
    return &MOTOR_STATE_RUN;  /* Run_Entry Procs synchronous  Alternatively, transition through Freewheel */
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
    Phase_ActivateV0(&p_motor->PHASE);
    FOC_ClearCaptureState(&p_motor->P_MOTOR_STATE->Foc);
    // determine initial substate from stop
    p_motor->P_MOTOR_STATE->FeedbackMode.OpenLoop = 1U; /* limits user cmd input */
}

/*
    Proc SubState Tree
*/
static void OpenLoop_Proc(const Motor_T * p_motor)
{
}

/* maintain consistent interface with other states, use substate cmd for phase output without exiting */
static State_T * OpenLoop_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;
    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_Z:      p_nextState = &MOTOR_STATE_PASSIVE;     break;
        case PHASE_VOUT_0:      p_nextState = &MOTOR_STATE_PASSIVE;     break;
        // case PHASE_VOUT_0:       /* Phase_ActivateV0(&p_motor->PHASE); */   break;
        case PHASE_VOUT_PWM:     /* Phase_ActivateT0(&p_motor->PHASE); */   break;
        default: break;
        /* No resume from OpenLoop, freewheel state check stop */
    }
    return p_nextState;
}

static State_T * Openloop_InputStop(const Motor_T * p_motor, state_value_t direction)
{
    return (direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_PASSIVE : NULL;
    // return (direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_STOP : NULL;
}

static State_T * OpenLoop_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode); /* a different flag mode will change ramp limits */
    return (p_motor->P_MOTOR_STATE->FeedbackMode.OpenLoop == 0U) ? &MOTOR_STATE_PASSIVE : NULL;
    // return NULL;
}

/*
    using openloop substate inputs
    multiple cmds per subState. Implemented with [StateMachine_TransitionCmd_T]
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
    Phase_ActivateV0(&p_motor->PHASE); /* Transition from Stop or Substates */
    p_motor->P_MOTOR_STATE->ControlTimerBase = 0U;
    p_motor->P_MOTOR_STATE->CalibrationStateIndex = 0U;
}

static void Calibration_Proc(const Motor_T * p_motor)
{
}


/* Calibration State Exit with Direction == 0 and InputCalibration(STOP) */
static State_T * Calibration_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    // State_T * p_nextState = NULL;

    // switch ((Phase_Output_T)phaseOutput)
    // {
    //     case PHASE_VOUT_Z:    Phase_Deactivate(&p_motor->PHASE);                break;
    //     case PHASE_VOUT_0:       Phase_ActivateV0(&p_motor->PHASE);     break;
    //     case PHASE_VOUT_PWM:     break;
    // }

    // // return p_nextState;
    // return &MOTOR_STATE_CALIBRATION; /* Stops the SubState */
    State_T * p_nextState = NULL;
    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_Z:    p_nextState = &MOTOR_STATE_PASSIVE;     break;
        case PHASE_VOUT_0:       p_nextState = &MOTOR_STATE_PASSIVE;     break;
        case PHASE_VOUT_PWM:     break;
    }
    return p_nextState;
}

static State_T * Calibration_InputStop(const Motor_T * p_motor, state_value_t direction)
{
    return (direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_PASSIVE : NULL;
    // return ((Motor_Direction_T)direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_STOP : NULL;
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
    .ENTRY              = (State_Action_T)Calibration_Entry,
    .LOOP               = (State_Action_T)Calibration_Proc,
    .P_TRANSITION_TABLE = &CALIBRATION_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Fault_Entry(const Motor_T * p_motor) { Phase_Deactivate(&p_motor->PHASE); }

static void Fault_Proc(const Motor_T * p_motor) { Phase_Deactivate(&p_motor->PHASE); }

static State_T * Fault_InputClearFault(const Motor_T * p_motor, state_value_t faultFlags)
{
    // p_motor->P_MOTOR_STATE->FaultFlags.Value &= ~faultFlags;
    // p_motor->P_MOTOR_STATE->FaultFlags.Value = 0U;
    // p_motor->P_MOTOR_STATE->FaultFlags.PositionSensor = !Motor_VerifySensorCalibration(p_motor); // non polling check
    Motor_CaptureFaultFlags(p_motor);

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
    if (Motor_IsFault(p_motor) == false) { StateMachine_Tree_InputAsyncTransition(&p_motor->STATE_MACHINE, MSM_INPUT_FAULT, -1); }
}

/*! @return true if no fault remains */
bool Motor_StateMachine_ExitFault(const Motor_T * p_motor)
{
    if (Motor_IsFault(p_motor) == true) { StateMachine_Tree_InputAsyncTransition(&p_motor->STATE_MACHINE, MSM_INPUT_FAULT, 0); }
    return !Motor_IsFault(p_motor);
}

/*
    individual flag access todo
*/
void Motor_StateMachine_SetFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    if (Motor_IsFault(p_motor) == false) { StateMachine_Tree_InputAsyncTransition(&p_motor->STATE_MACHINE, MSM_INPUT_FAULT, faultFlags.Value); }
}

/* faultFlags unused for now */
void Motor_StateMachine_ClearFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    if (Motor_IsFault(p_motor) == true) { StateMachine_Tree_InputAsyncTransition(&p_motor->STATE_MACHINE, MSM_INPUT_FAULT, ~faultFlags.Value); }
}




