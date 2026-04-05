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
#include "SubStates/Motor_Intervention.h"


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
    // .P_STATE_FAULT = &MOTOR_STATE_FAULT,
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
/* Non-Fault states: apply set/clear, transition to Fault if any flags remain */
static State_T * TransitionFault(const Motor_T * p_motor, state_value_t faultCmd)
{
    Motor_FaultCmd_T cmd = { .Value = faultCmd };
    p_motor->P_MOTOR_STATE->FaultFlags.Value |= cmd.FaultSet;
    return (p_motor->P_MOTOR_STATE->FaultFlags.Value != 0U) ? &MOTOR_STATE_FAULT : NULL;
}

/* Monitor Flags */
static void Motor_PollFaultFlags(const Motor_T * p_motor)
{
    // p_motor->P_MOTOR_STATE->FaultFlags.Overheat = HeatMonitor_IsFault(&p_motor->P_MOTOR_STATE->Thermistor);
    p_motor->P_MOTOR_STATE->FaultFlags.Overheat = 0;
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
        Motor_PollFaultFlags(p_motor); /* Clear the fault flags once */

        if (p_motor->P_MOTOR_STATE->FaultFlags.Value != 0U) { p_nextState = &MOTOR_STATE_FAULT; } else { p_nextState = &MOTOR_STATE_STOP; }
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
    .NEXT               = (State_Input0_T)Init_Next,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief Stop State as Disabled Runtime

    Safe State with Transition to Calibration
    Phase V0/VZ
*/
/******************************************************************************/
/*
    Optionally self transition to Stop on 0 speed. let app layer handle safe state and guard transition to calibration.
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

// can depreciate for Transition command
static State_T * Stop_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;
    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_Z:      Phase_Deactivate(&p_motor->PHASE);      break;
        case PHASE_VOUT_0:      p_nextState = &MOTOR_STATE_PASSIVE;     break; // optional check usercmd
        case PHASE_VOUT_PWM:    p_nextState = &MOTOR_STATE_PASSIVE;     break;
        default: break;
    }
    return p_nextState;
}

/*
    Let Direction double as sematics for transition be an extension.
    Direction cannot be used as transition to ready state for app layer to remain in its safe state and also propagate direction set to motor layer.
    Direction null exits to stop. but setting direction from stop does not transition to ready.
        exiting a state on loss of any condition, but entering may not be guaranteed by setting a single condition.
        app layer exit park state may preserve stop state while storing direction in motor state.
    maintains compatibility with none direction based handling.
*/
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
    return NULL;
}

static State_T * Stop_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode);
    return NULL;
}

/* Transition for user req */
static State_T * Stop_InputOpenLoop(const Motor_T * p_motor, state_value_t state)
{
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U) { return &MOTOR_STATE_OPEN_LOOP; } else { return NULL; }
}

/* Calibration go directly to SubState */
static State_T * Stop_InputCalibration(const Motor_T * p_motor, state_value_t statePtr)
{
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U) { return &MOTOR_STATE_CALIBRATION; } else { return NULL; }
}

static const State_Input_T STOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)Stop_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (State_Input_T)Stop_InputFeedbackMode,
    [MSM_INPUT_DIRECTION]       = (State_Input_T)Stop_InputDirection,
    [MSM_INPUT_CALIBRATION]     = (State_Input_T)Stop_InputCalibration, /*  */
    [MSM_INPUT_OPEN_LOOP]       = (State_Input_T)Stop_InputOpenLoop, /* Valid but can be omitted */
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
    V0 passive and VZ passive require different entries. both "control loop off."
    optionally substate with entry action.
*/
static void Passive_Entry(const Motor_T * p_motor)
{
    // if (Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR_STATE)) { Phase_Deactivate(&p_motor->PHASE); } else { Phase_ActivateV0(&p_motor->PHASE); }
    Phase_Deactivate(&p_motor->PHASE);
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR_STATE); // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ClearFeedbackState, NULL);
    p_motor->P_MOTOR_STATE->ControlTimerBase = 0U; /* ok to reset timer */
}

static void Passive_Proc(const Motor_T * p_motor)
{
    /* Match Feedback to ProcAngleBemf on Resume */
    Motor_FOC_ProcCaptureAngleVBemf(p_motor->P_MOTOR_STATE);    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL /* Motor_SixStep_ProcPhaseObserve */);
    switch (Phase_ReadOutputState(&p_motor->PHASE))
    {
        case PHASE_VOUT_0:  if (Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR_STATE)) { Phase_Deactivate(&p_motor->PHASE); }    break;
            // case PHASE_VOUT_Z:  if (!Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR_STATE)) { Phase_ActivateV0(&p_motor->PHASE); }   break;
        default: break;
    }
}

static State_T * Passive_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;

    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_Z:
            if (Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR_STATE)) { Phase_Deactivate(&p_motor->PHASE); }
            break;
        case PHASE_VOUT_0:
            Phase_ActivateV0(&p_motor->PHASE);
            break;
        case PHASE_VOUT_PWM:
            if (p_motor->P_MOTOR_STATE->Direction != MOTOR_DIRECTION_NULL)
            {
                // if (p_motor->P_MOTOR_STATE->Foc.Vq == 0) { Debug_Beep(); }
                if (RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR_STATE->p_ActiveSensor) == true) { p_nextState = &MOTOR_STATE_RUN; }
                // else if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U) /* OpenLoop start at 0 speed */
                // {
                //     p_nextState = &MOTOR_STATE_OPEN_LOOP;
                //     /* p_nextState = Motor_Sensor_GetStartUpState(p_motor); */ /* get substate */
                // }
                // else no transition
            }
            break;
    }

    return p_nextState;
}

static State_T * Passive_InputDirection(const Motor_T * p_motor, state_value_t direction)
{
    State_T * p_nextState = NULL;

    /* If set during motor spinning. Rotor sensor maintain separate direction for interpolation */
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
        /* Null direction can function as stop while other directions not function as start. */
        // if (direction == MOTOR_DIRECTION_NULL) { p_nextState = &MOTOR_STATE_STOP; }
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
    (void)state;
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U) { return &MOTOR_STATE_OPEN_LOOP; }  else { return NULL; }
}

// static State_T * InputStop(const Motor_T * p_motor, state_value_t value)
// {
//     (void)value;
//     if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U) { return &MOTOR_STATE_STOP; }
//     return NULL;
// }

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

    // Motor_UpdateSpeedTorqueLimits(p_motor->P_MOTOR_STATE, Motor_ILimitCw(p_motor->P_MOTOR_STATE), Motor_ILimitCcw(p_motor->P_MOTOR_STATE));
    // RotorSensor_ZeroInitial(p_motor->P_MOTOR_STATE->p_ActiveSensor); not needed if capture sensor runs in thread

    // Motor_FOC_ProcAngleControl(p_motor->P_MOTOR_STATE);
    // Motor_FOC_WriteDuty(p_motor);
    // Phase_ActivateOutput(p_motor->P_MOTOR_STATE);
}

static void Run_Proc(const Motor_T * p_motor)
{
    Motor_ProcOuterFeedback(p_motor->P_MOTOR_STATE);
    Motor_FOC_ProcAngleControl(p_motor->P_MOTOR_STATE); // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcAngleControl, NULL/* Motor_SixStep_ProcPhaseControl */);
    Motor_FOC_WriteDuty(p_motor);
}

static State_T * Run_InputRelease(const Motor_T * p_motor)
{
    if (Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR_STATE)) { return &MOTOR_STATE_PASSIVE; } else { return &INTERVENTION_STATE_TORQUE_ZERO; }
}

/* App layer handle conditional release for now. substate for passive torque 0. change input from user cmd */
static State_T * Run_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_Z:      return Run_InputRelease(p_motor);
        case PHASE_VOUT_0:      return Run_InputRelease(p_motor);
        case PHASE_VOUT_PWM:    return NULL;
        default:                return NULL;
    }
}

// depreciate
// static State_T * Run_InputStop(const Motor_T * p_motor, state_value_t direction)
// {
//     State_T * p_nextState = NULL;
//     switch ((Motor_Direction_T)direction)
//     {
//         case MOTOR_DIRECTION_NULL:  p_nextState = &MOTOR_STATE_PASSIVE; break; /*   (Motor_CheckSpeed(p_motor) == true) */
//         case MOTOR_DIRECTION_CW:    break;
//         case MOTOR_DIRECTION_CCW:   break;
//         default: break; /* Invalid direction */
//     }
//     return p_nextState;
//     // if (direction == MOTOR_DIRECTION_NULL) { p_nextState = &MOTOR_STATE_PASSIVE; }
// }

/*
    Process [Motor_FOC_MatchFeedbackState] before [Motor_FOC_ProcAngleControl]
    Match inline without self-transition to avoid Phase_ActivateT0 zero-voltage glitch in Run_Entry
*/
// return &MOTOR_STATE_RUN;  /* Run_Entry Procs synchronous  Alternatively, transition through Freewheel */
static State_T * Run_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode);
    Motor_FOC_MatchFeedbackState(p_motor->P_MOTOR_STATE);
    return NULL;
}

static const State_Input_T RUN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_FEEDBACK_MODE]   = (State_Input_T)Run_InputFeedbackMode,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)Run_InputControl,
    // [MSM_INPUT_DIRECTION]       = (State_Input_T)Run_InputStop,
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
    @brief State Intervention - Motor-level safety supervisor
    Proc Feedback independent from user input.
    Owns the controlled shutdown path.

    SubStates (in SubStates/Motor_Intervention.c):
      TORQUE_ZERO (SS0) - Coast with zero torque, user may resume
      RAMP_SAFE   (SS1) - Active deceleration to zero, no resume
*/
/******************************************************************************/
static void Intervention_Entry(const Motor_T * p_motor)
{
    Motor_FOC_MatchIVState(p_motor->P_MOTOR_STATE);
    Ramp_SetOutputState(&p_motor->P_MOTOR_STATE->TorqueRamp, 0);
    p_motor->P_MOTOR_STATE->UserTorqueReq = 0;
}

static void Intervention_Proc(const Motor_T * p_motor){}

static State_T * Intervention_InputRelease(const Motor_T * p_motor)
{
    if (Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR_STATE)) { return &MOTOR_STATE_PASSIVE; }
    return NULL;
}

static State_T * Intervention_InputResume(const Motor_T * p_motor)
{
    if (StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &INTERVENTION_STATE_TORQUE_ZERO)
        && RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR_STATE->p_ActiveSensor) == true)
    {
        return &MOTOR_STATE_RUN;
    }
    return NULL;
}

/*
    Resume to Run only from TorqueZero substate.
    RampSafe substate rejects resume — committed to safe stop.
*/
static State_T * Intervention_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_Z:
        case PHASE_VOUT_0:
            return Intervention_InputRelease(p_motor);
        case PHASE_VOUT_PWM:
            return Intervention_InputResume(p_motor);
        default: return NULL;
    }
}

/* Store only */
static State_T * Intervention_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode);
    return NULL;
}

static const State_Input_T INTERVENTION_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)Intervention_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (State_Input_T)Intervention_InputFeedbackMode,
};

const State_T MOTOR_STATE_INTERVENTION =
{
    .ID                 = MSM_STATE_ID_INTERVENTION,
    .P_TRANSITION_TABLE = &INTERVENTION_TRANSITION_TABLE[0U],
    .ENTRY              = (State_Action_T)Intervention_Entry,
    .LOOP               = (State_Action_T)Intervention_Proc,
};

/******************************************************************************/
/*!
    @brief State OpenLoop - OpenLoop, Align, and Start Up, Feedback Acquisition

    Only Entry is from [PASSIVE] State
*/
/******************************************************************************/
static void OpenLoop_Entry(const Motor_T * p_motor)
{
    Phase_ActivateV0(&p_motor->PHASE);
    FOC_ClearCaptureState(&p_motor->P_MOTOR_STATE->Foc);
}

/*
    Proc SubState Tree
*/
static void OpenLoop_Proc(const Motor_T * p_motor) {}

/* maintain consistent interface with Run, use substate cmd for phase output without exiting */
/* No resume from OpenLoop, freewheel state check stop */
static State_T * OpenLoop_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_Z:      return &MOTOR_STATE_PASSIVE;
        case PHASE_VOUT_0:      return &MOTOR_STATE_PASSIVE;
        case PHASE_VOUT_PWM:    return NULL;
        default:                return NULL;
    }
}

static State_T * OpenLoop_InputDirection(const Motor_T * p_motor, state_value_t direction)
{
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
    }
    return NULL;
}

static State_T * OpenLoop_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode); /* a different flag mode will change ramp limits */
    return NULL;
}

/*
    handle exit, optionally handle cmds starting with a substate
    call sequence is preserved, same for cmds with and without substate. OpenLoop->Substate. OpenLoop->Cmd
*/
static State_T * OpenLoop_InputOpenLoop(const Motor_T * p_motor, state_value_t statePtr)
{
    State_T * p_state = (State_T *)statePtr;

    if (p_state == NULL) { return &MOTOR_STATE_PASSIVE; }
    if (p_state == &MOTOR_STATE_OPEN_LOOP) { return &MOTOR_STATE_OPEN_LOOP; }
    if (p_state->P_TOP == &MOTOR_STATE_OPEN_LOOP) { return p_state; }

    return &MOTOR_STATE_PASSIVE;
}

static const State_Input_T OPEN_LOOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)OpenLoop_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (State_Input_T)OpenLoop_InputFeedbackMode,
    [MSM_INPUT_OPEN_LOOP]       = (State_Input_T)OpenLoop_InputOpenLoop,
    [MSM_INPUT_DIRECTION]       = (State_Input_T)OpenLoop_InputDirection,
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
    switch ((Phase_Output_T)phaseOutput)
    {
        case PHASE_VOUT_Z:      return &MOTOR_STATE_PASSIVE;
        case PHASE_VOUT_0:      return &MOTOR_STATE_PASSIVE;
        case PHASE_VOUT_PWM:    return NULL;
        default:                return NULL;
    }
}

// static State_T * Calibration_InputStop(const Motor_T * p_motor, state_value_t direction)
// {
//     // return (direction == MOTOR_DIRECTION_NULL) ? &MOTOR_STATE_PASSIVE : NULL;
// }

/*
    provide exit, optionally handle cmds starting with a substate
*/
static State_T * Calibration_InputCalibration(const Motor_T * p_motor, state_value_t statePtr)
{
    State_T * p_state = (State_T *)statePtr;

    assert(p_state == NULL || p_state == &MOTOR_STATE_STOP || p_state == &MOTOR_STATE_CALIBRATION || p_state->P_TOP == &MOTOR_STATE_CALIBRATION);

    if (p_state == NULL) { return &MOTOR_STATE_PASSIVE; }
    if (p_state == &MOTOR_STATE_CALIBRATION) { return &MOTOR_STATE_CALIBRATION; }
    if (p_state->P_TOP == &MOTOR_STATE_CALIBRATION) { return p_state; }

    return &MOTOR_STATE_PASSIVE;

    // return p_state;
}

static const State_Input_T CALIBRATION_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MSM_INPUT_PHASE_OUTPUT]    = (State_Input_T)Calibration_InputControl,
    [MSM_INPUT_CALIBRATION]     = (State_Input_T)Calibration_InputCalibration,
    // [MSM_INPUT_DIRECTION]       = (State_Input_T)Calibration_InputStop,
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

/* Fault State: Set accumulates (latches), Clear removes flags and re-verifies */
static State_T * Fault_InputFault(const Motor_T * p_motor, state_value_t faultCmd)
{
    Motor_FaultCmd_T cmd = { .Value = faultCmd };
    p_motor->P_MOTOR_STATE->FaultFlags.Value |= cmd.FaultSet;
    if (cmd.FaultClear != 0U)
    {
        p_motor->P_MOTOR_STATE->FaultFlags.Value &= ~cmd.FaultClear;
        Motor_PollFaultFlags(p_motor); /* Re-verify conditions resolved before allowing exit */
    }
    return (p_motor->P_MOTOR_STATE->FaultFlags.Value == 0U) ? &MOTOR_STATE_STOP : NULL;
}

static State_T * Fault_InputCalibration(const Motor_T * p_motor, state_value_t state)
{
    State_T * p_nextState = NULL;

    // if (p_motor->P_MOTOR_STATE->FaultFlags.Value & ~POSITIONSENSOR)
    if (p_motor->P_MOTOR_STATE->FaultFlags.Overheat == 0U)
    {
        p_nextState = &MOTOR_STATE_CALIBRATION;
    }

    return p_nextState;
}

static const State_Input_T FAULT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]       = (State_Input_T)Fault_InputFault,
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
/* Pass only delta flags — handler applies OR FaultSet / AND-NOT FaultClear atomically */
void Motor_StateMachine_SetFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    StateMachine_Tree_InputAsyncTransition(&p_motor->STATE_MACHINE, MSM_INPUT_FAULT, (Motor_FaultCmd_T){ .FaultSet = faultFlags.Value }.Value);
}

void Motor_StateMachine_ClearFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    StateMachine_Tree_InputAsyncTransition(&p_motor->STATE_MACHINE, MSM_INPUT_FAULT, (Motor_FaultCmd_T){ .FaultClear = faultFlags.Value }.Value);
}

bool Motor_StateMachine_TryClearFaultAll(const Motor_T * p_motor)
{
    Motor_StateMachine_ClearFault(p_motor, (Motor_FaultFlags_T){ .Value = UINT16_MAX });
    return !Motor_IsFault(p_motor);
}



// Orthogonal Composition
// The critical architectural point: State objects naturally compose as orthogonal regions because each provides an independent behavioral filter over shared canonical data.
///The key is: States don't hold independent data — they provide a behavioral lens over canonical data.

/* Alternatively orthogonal state  */
// static State_T * Ccw_Input(const Motor_T * p_motor, state_value_t value)
// {

// }

// static void Ccw_Process(const Motor_T * p_motor, state_value_t value)
// {
//     // uint16_t iLimitCcw = p_motor->P_MOTOR_STATE->ILimitMotoring_Fract16;
//     // uint16_t iLimitCw = p_motor->P_MOTOR_STATE->ILimitGenerating_Fract16;
// }

// const State_T CCW_STATE =
// {
//     // .ENTRY = (State_Action_T)Ccw,
//     // .LOOP = (State_Action_T)Ccw,
// .GetPhaseSign = _Forward_GetPhaseSign,     // returns +1
// .GetCommandSign = _Forward_GetCommandSign,

// };

// const State_T FORWARD_MOTORING_STATE =
// {
// };

// transition using user value alone.
// typedef struct
// {
//     // State_T Base;
//     // State_Input0_T ProcRamp;
// }
// FeedbackState_T;

// const State_T SPEED_FEEDBACK_STATE =
// {
//     // .ENTRY = (State_Action_T)Ccw,
//     // .LOOP = (State_Action_T)Ccw,
// };

/* State X State */
// static inline fract16_t  FeedbackProc(Motor_T * p_motor, State_T * p_DirectionState, State_T * p_FeedbackState)
// {
//     p_FeedbackState->ProcRamp(p_motor, p_DirectionState->CwValue(), p_DirectionState->CcwValue());
// }
// state_value_t Torque0_Ramp(const Motor_T * p_motor) { return 0; }

// // State_DataVector_T TORQUE_0_DATA_VECTOR =
// // {
// //     (State_Data_T)Torque0_Ramp,
// //     (State_Data_T)Torque0_Ramp,
// // };

// typedef struct RunState_Vector
// {
//     // State_T BASE;
//     int(*TorqueCmd)(Motor_T * p_motor);
//     // State_Data_T * p_DataVector;
// } RunState_Vector_T;

// const RunState_Vector_T TORQUE_0_DATA_VECTOR =
// {
//     .TorqueCmd = Torque0_Ramp,
// };

// const RunState_Vector_T USER_CMD_DATA_VECTOR =
// {
//     .TorqueCmd = Torque0_Ramp,
// };

// static void Run_ProcOrthogonal(const Motor_T * p_motor)
// {
//     RunState_Vector_T * p_state = (RunState_Vector_T *)p_motor->P_MOTOR_STATE->StateMachine.p_Orthogonal;
//     fract16_t cmd = (p_state->TorqueCmd)(p_motor);
//     // Motor_FOC_ProcInnerFeedback(p_motor->P_MOTOR_STATE, 0, cmd);
//     // Motor_FOC_ProcAngleOutput(p_motor->P_MOTOR_STATE);
// }


// typedef struct State_VTable
// {
//     State_T * (*InputControl)(const Motor_T * p_motor, state_value_t value);
//     State_T * (*InputDirection)(const Motor_T * p_motor, state_value_t direction);
//     State_T * (*InputFeedbackMode)(const Motor_T * p_motor, state_value_t feedbackMode);
//     State_T * (*InputOpenLoop)(const Motor_T * p_motor, state_value_t statePtr);
//     State_T * (*InputCalibration)(const Motor_T * p_motor, state_value_t statePtr);
// } State_VTable_T;

/* Orthgonal States */
// typedef struct Motor_StateIntervention
// {
//     State_T BASE;
//     struct
//     {
//         // state_value_t(*Condition)(const Motor_State_T *);
//         state_value_t(*REQ_OVERRIDE) (const Motor_State_T *);
//     };
// }
// Motor_StateIntervention_T;

// pass top state return?
// static void Torque0_Proc(Motor_State_T * p_motor)
// {

// }

// static state_value_t Torque0_Ramp(const Motor_State_T * p_motor) { return 0; }

// Motor_StateIntervention_T MOTOR_STATE_TORQUE_ZERO =
// {
//     .BASE =
//     {
//         // .ENTRY = (State_Action_T)Torque0_Entry,
//         .LOOP = (State_Action_T)Torque0_Proc,
//         // .P_DATA_VECTOR = (State_Data_T[])
//         // {
//         //     [0] = (State_Data_T)Torque0_Ramp,
//         // },
//     },
//     // .Condition = Motor_IsSpeedFreewheelLimitRange,
//     .REQ_OVERRIDE =  Torque0_Ramp,
// };

// static void ProcTorque0(Motor_State_T * p_motor)
// {
//  int a =   MOTOR_STATE_TORQUE_ZERO.DataVector[1](p_motor);
//     // Motor_FOC_ProcInnerFeedback(p_motor, p_motor->SensorState.AngleSpeed.Angle, 0, Motor_IRampOf(p_motor, 0));
//     // Motor_FOC_ProcAngleOutput(p_motor);
// }
