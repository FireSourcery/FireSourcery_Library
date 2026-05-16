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
#include "Motor_Intervention.h"
#include "../Motor_Config.h"


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
    .TRANSITION_TABLE_LENGTH = MOTOR_TRANSITION_TABLE_LENGTH,
    // .P_STATES = [
    //     &MOTOR_STATE_INIT,
    //     &MOTOR_STATE_DEACTIVATED,
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
static State_T * TransitionFault(Motor_T * p_motor, state_value_t faultCmd)
{
    Motor_FaultCmd_T cmd = { .Value = faultCmd };
    p_motor->P_MOTOR->FaultFlags.Value |= cmd.FaultSet;
    return (p_motor->P_MOTOR->FaultFlags.Value != 0U) ? &MOTOR_STATE_FAULT : NULL;
}

/* Monitor Flags */
static void Motor_PollFaultFlags(Motor_T * p_motor)
{
    // p_motor->P_MOTOR->FaultFlags.Overheat = HeatMonitor_IsFault(&p_motor->P_MOTOR->Thermistor);
    p_motor->P_MOTOR->FaultFlags.Overheat = 0;
}

/*
    Set on Direction change
    Clip opposite user direction range, no plugging.
*/
static void _Motor_SetDirection(Motor_T * p_dev, Motor_Direction_T direction)
{
    Motor_SetDirection(p_dev, direction);
    // FOC_SetVAntiPlugging(p_foc, direction, vPhaseLimit);
    FOC_SetVLimits(&p_dev->P_MOTOR->Foc, (sign_t)direction, VBus_GetVPhaseRefSvpwm(p_dev->P_VBUS)); /* overwritten on loop */
}

static inline void _Motor_SetFeedbackMode_Cast(Motor_T * p_motor, state_value_t mode) { Motor_SetFeedbackMode(p_motor, Motor_FeedbackMode_Cast(mode)); }
static inline void _Motor_SetDirection_Cast(Motor_T * p_motor, state_value_t mode) { Motor_SetDirection(p_motor, Motor_Direction_Cast(mode)); }


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
    (void)p_motor;
    SysTime_Millis = 0U; /* Reset SysTime in case of reboot */
}

static void Init_Proc(Motor_T * p_motor)
{
    (void)p_motor;
}

static State_T * Init_Next(Motor_T * p_motor)
{
    State_T * p_nextState = NULL;

    if (SysTime_GetMillis() > MOTOR_STATE_MACHINE_INIT_WAIT) /* wait for Speed and Heat sensors */
    {
        if (Phase_Calibration_IsValid() == false) { p_motor->P_MOTOR->FaultFlags.InitCheck = 1U; } /* alternatively go to fault, outer module parse */
        if (Motor_Config_IsValid(&p_motor->P_MOTOR->Config) == false) { p_motor->P_MOTOR->FaultFlags.InitCheck = 1U; }

        p_motor->P_MOTOR->FaultFlags.PositionSensor = !RotorSensor_VerifyCalibration(p_motor->P_MOTOR->p_ActiveSensor);
        Motor_PollFaultFlags(p_motor); /* Clear the fault flags once */

        if (p_motor->P_MOTOR->FaultFlags.Value != 0U) { p_nextState = &MOTOR_STATE_FAULT; } else { p_nextState = &MOTOR_STATE_DEACTIVATED; }
    }

    return p_nextState;
}

static const State_Input_T INIT_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]           = NULL,
    [MOTOR_STATE_INPUT_DIRECTION]       = NULL,
    [MOTOR_STATE_INPUT_CALIBRATION]     = NULL,
};

const State_T MOTOR_STATE_INIT =
{
    .ID                 = MOTOR_STATE_ID_INIT,
    .ENTRY              = (State_Action_T)Init_Entry,
    .LOOP               = (State_Action_T)Init_Proc,
    .NEXT               = (State_Input0_T)Init_Next,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief Deactivated State

    Quiescent gateway: phases off, direction null. Operation disabled until
    explicitly transitioned. Sole entry to CALIBRATION.
    Reached from INIT (post-init) and from FAULT (after clear).
*/
/******************************************************************************/
static void Deactivated_Entry(Motor_T * p_motor)
{
    Phase_Deactivate(&p_motor->PHASE);
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR); // Motor_CommutationModeFn_Call(p_motor->P_MOTOR, Motor_FOC_ClearFeedbackState, NULL); /* Unobserved values remain 0 for user read */
    _Motor_SetDirection(p_motor, MOTOR_DIRECTION_NULL);
    p_motor->P_MOTOR->ControlTimerBase = 0U; /* ok to reset timer */
}

static void Deactivated_Proc(Motor_T * p_motor)
{
    Motor_FOC_ProcCaptureAngleVBemf(p_motor->P_MOTOR); // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL);
}

static State_T * Deactivated_InputDirection(Motor_T * p_motor, state_value_t direction)
{
    switch ((Motor_Direction_T)direction)
    {
        case MOTOR_DIRECTION_NULL:
        case MOTOR_DIRECTION_CW:
        case MOTOR_DIRECTION_CCW:
            _Motor_SetDirection(p_motor, direction); break;
        default: break; /* Invalid direction */
    }
    return NULL;
}

static State_T * Deactivated_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
{
    _Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
    return NULL;
}

/* Transition for user req */
static State_T * Deactivated_InputOpenLoop(Motor_T * p_motor, state_value_t state)
{
    (void)state;
    // if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) == 0U) { return &MOTOR_STATE_OPEN_LOOP; } else { return NULL; }
}

/* Calibration go directly to SubState */
static State_T * Deactivated_InputCalibration(Motor_T * p_motor, state_value_t statePtr)
{
    (void)statePtr;
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) == 0U) { return &MOTOR_STATE_CALIBRATION; } else { return NULL; }
}

static const State_Input_T DEACTIVATED_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = (State_Input_T)Deactivated_InputFeedbackMode,
    [MOTOR_STATE_INPUT_DIRECTION]       = (State_Input_T)Deactivated_InputDirection,
    [MOTOR_STATE_INPUT_CALIBRATION]     = (State_Input_T)Deactivated_InputCalibration,
    [MOTOR_STATE_INPUT_OPEN_LOOP]       = (State_Input_T)Deactivated_InputOpenLoop,
};

const State_T MOTOR_STATE_DEACTIVATED =
{
    .ID                 = MOTOR_STATE_ID_DEACTIVATED,
    .ENTRY              = (State_Action_T)Deactivated_Entry,
    .LOOP               = (State_Action_T)Deactivated_Proc,
    .P_TRANSITION_TABLE = &DEACTIVATED_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief Passive State - Freewheel or V0
*/
/******************************************************************************/
/*
    V0 passive and VZ passive require different entries. both "control loop off."
    optionally substate with entry action.
*/
static void Passive_Entry(Motor_T * p_motor)
{
    // if (Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR)) { Phase_Deactivate(&p_motor->PHASE); } else { Phase_ActivateV0(&p_motor->PHASE); }
    Phase_Deactivate(&p_motor->PHASE);
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR); // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ClearFeedbackState, NULL);
    p_motor->P_MOTOR->ControlTimerBase = 0U; /* ok to reset timer */
}

static void Passive_Proc(Motor_T * p_motor)
{
    /* Match Feedback to ProcAngleBemf on Resume */
    Motor_FOC_ProcCaptureAngleVBemf(p_motor->P_MOTOR);    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL /* Motor_SixStep_ProcPhaseObserve */);
    // switch (Phase_ReadVOut(&p_motor->PHASE))
    // {
    //     case PHASE_VOUT_0:  if (Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR)) { Phase_Deactivate(&p_motor->PHASE); }    break;
    //     // case PHASE_VOUT_Z:  if (!Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR)) { Phase_ActivateV0(&p_motor->PHASE); }   break;
    //     default: break;
    // }
}

static State_T * Passive_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;

    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:
            if (Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR)) { Phase_Deactivate(&p_motor->PHASE); }
            break;
        case PHASE_VOUT_0:
            Phase_ActivateV0(&p_motor->PHASE);
            break;
        case PHASE_VOUT_PWM:
            if (p_motor->P_MOTOR->Direction != MOTOR_DIRECTION_NULL)
            {
                if (RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR->p_ActiveSensor) == true) { p_nextState = &MOTOR_STATE_RUN; }
                // else if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) == 0U) /* OpenLoop start at 0 speed */
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

/*
    Let Direction double as sematics for transition be an extension.
    Direction cannot be used as transition to ready state for app layer to remain in its safe state and also propagate direction set to motor layer.
        exiting a state on loss of any condition, but entering may not be guaranteed by setting a single condition.
        app layer exit park state may preserve disabled state while storing direction in motor state.
    maintains compatibility with none direction based handling.
    (direction != p_motor->P_MOTOR->Direction) needs to stay in syync
*/
static State_T * Passive_InputDirection(Motor_T * p_motor, state_value_t direction)
{
    State_T * p_nextState = NULL;

    /* If set during motor spinning. Rotor sensor maintain separate direction for interpolation */
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) == 0U)
    {
        switch ((Motor_Direction_T)direction)
        {
            case MOTOR_DIRECTION_NULL: /* Intentional fall-through */
            case MOTOR_DIRECTION_CW:
            case MOTOR_DIRECTION_CCW:
                _Motor_SetDirection(p_motor, direction); break;
            default: break; /* Invalid direction */
        }
        /* Null direction can function as disable while other directions not function as enable. */
        // if (direction == MOTOR_DIRECTION_NULL) { p_nextState = &MOTOR_STATE_DEACTIVATED; }
    }

    return p_nextState;
}

static State_T * Passive_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
{
    _Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
    return NULL;
}

static State_T * Passive_InputOpenLoop(Motor_T * p_motor, state_value_t state)
{
    (void)state;
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) == 0U) { return &MOTOR_STATE_OPEN_LOOP; }  else { return NULL; }
}

static const State_Input_T PASSIVE_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MOTOR_STATE_INPUT_PHASE_OUTPUT]    = (State_Input_T)Passive_InputControl,
    [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = (State_Input_T)Passive_InputFeedbackMode,
    [MOTOR_STATE_INPUT_DIRECTION]       = (State_Input_T)Passive_InputDirection,
    [MOTOR_STATE_INPUT_OPEN_LOOP]       = (State_Input_T)Passive_InputOpenLoop,
    [MOTOR_STATE_INPUT_CALIBRATION]     = NULL,
};

const State_T MOTOR_STATE_PASSIVE =
{
    .ID                 = MOTOR_STATE_ID_PASSIVE,
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
static void Run_Entry(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    /* Vabc is either bemf or 0 on entryy */
    if (p_context->FeedbackMode.Current == 1U) { Motor_FOC_MatchTorqueIState(p_context); } else { Motor_FOC_MatchTorqueVState(p_context); }
    if (p_context->FeedbackMode.Speed == 1U) { _Motor_MatchSpeedTorqueState(p_context, Ramp_GetOutput(&p_context->TorqueRamp)); }

    Phase_ActivateT0(&p_motor->PHASE);    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ActivateOutput, NULL);
}

static void Run_Proc(Motor_T * p_motor)
{
    // #ifdef MOTOR_EXTERN_CONTROL_ENABLE
    //     Motor_ExternControl(p_motor);
    // #endif

    Motor_Context_T * p_context = p_motor->P_MOTOR;
    if (p_context->FeedbackMode.Current == 1U) { Motor_FOC_ProcAngleControl(p_motor); }
    else { Motor_FOC_ProcVControl(p_motor); }
}

static void Run_OnSpeed(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    FOC_CaptureSpeed(&p_context->Foc, Motor_GetSpeedFeedback(p_context));
    if (p_context->FeedbackMode.Speed == 1U) { Ramp_SetTarget(&p_context->TorqueRamp, Motor_ProcSpeedControl(p_context)); }
}

static const State_Action_T RUN_ACTION_TABLE[MOTOR_STATE_ACTION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_ON_SPEED] = (State_Action_T)Run_OnSpeed,
    [MOTOR_STATE_INPUT_ON_PHASE] = NULL,
};

static State_T * Run_InputRelease(Motor_T * p_motor)
{
    return Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR)? &MOTOR_STATE_PASSIVE : &INTERVENTION_STATE_RAMP_SAFE;
}

/* App layer handle conditional release for now. substate for passive torque 0. change input from user cmd */
static State_T * Run_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:      return Run_InputRelease(p_motor);
        case PHASE_VOUT_0:      return Run_InputRelease(p_motor);
        case PHASE_VOUT_PWM:    return NULL;
        default:                return NULL;
    }
}

// depreciate
static State_T * Run_InputStop(Motor_T * p_motor, state_value_t direction)
{
    switch ((Motor_Direction_T)direction)
    {
        case MOTOR_DIRECTION_NULL:  return Run_InputRelease(p_motor);
        case MOTOR_DIRECTION_CW:    return NULL;
        case MOTOR_DIRECTION_CCW:   return NULL;
        default: return NULL; /* Invalid direction */
    }
}

/*

*/
static State_T * Run_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
{
    _Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
    return &MOTOR_STATE_RUN;  /* Run_Entry Procs synchronous  Alternatively, transition through Freewheel */
    // Motor_FOC_MatchFeedbackState(p_motor); //  Match inline without self-transition
    // return NULL;
}

static const State_Input_T RUN_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = (State_Input_T)Run_InputFeedbackMode,
    [MOTOR_STATE_INPUT_PHASE_OUTPUT]    = (State_Input_T)Run_InputControl,
    // [MOTOR_STATE_INPUT_DIRECTION]       = (State_Input_T)Run_InputStop,
    [MOTOR_STATE_INPUT_CALIBRATION]     = NULL,
    [MOTOR_STATE_INPUT_OPEN_LOOP]       = NULL,
};

const State_T MOTOR_STATE_RUN =
{
    .ID                 = MOTOR_STATE_ID_RUN,
    .P_TRANSITION_TABLE = &RUN_TRANSITION_TABLE[0U],
    .P_ACTION_TABLE     = &RUN_ACTION_TABLE[0U],
    .ENTRY              = (State_Action_T)Run_Entry,
    .LOOP               = (State_Action_T)Run_Proc,
};


/******************************************************************************/
/*!
    @brief State Intervention - Motor-level safety supervisor
    Active control with an alternative control variable path.
      TORQUE_ZERO (ZTC) - Coast with zero torque, user may resume
      RAMP_SAFE   (SS1) - Active deceleration to zero, no resume
*/
/******************************************************************************/
static void Intervention_Entry(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    // if (p_context->FeedbackMode.Current == 0U)
    // {
    // }
    Ramp_SetLimits(&p_context->TorqueRamp, Motor_ILimitCw(p_context), Motor_ILimitCcw(p_context)); //switch to i limits or use vramp for voltage
    Motor_FOC_MatchTorqueIState(p_context);
    // FOC_MatchIVState(&p_context->Foc);
    // Ramp_SetOutputState(&p_context->TorqueRamp, FOC_Iq(&p_context->Foc));
}

// min(GeneratingOnly, _Motor_GeneratingOnly PID_GetOutput(&p_state->PidSpeed));  substates inherit
// Motor_FOC_ProcTorqueReq(p_motor, _Motor_GeneratingOnly(p_state, Ramp_GetTarget(&p_state->TorqueRamp)));
static void Intervention_Proc(Motor_T * p_motor) { (void)p_motor; }

/*
    Resume to Run only from TorqueZero substate.
    RampSafe substate override to reject resume — committed to safe stop.
*/
static State_T * Intervention_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:
        case PHASE_VOUT_0:      return Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR) ? &MOTOR_STATE_PASSIVE : &INTERVENTION_STATE_RAMP_SAFE;
        case PHASE_VOUT_PWM:    return RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR->p_ActiveSensor) ? &MOTOR_STATE_RUN : NULL;
        default: return NULL;
    }
}

/* Store only */
static State_T * Intervention_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
{
    _Motor_SetFeedbackMode_Cast(p_motor, feedbackMode); /* transition from speed to i mode does not need match */
    return NULL;
}

static const State_Input_T INTERVENTION_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MOTOR_STATE_INPUT_PHASE_OUTPUT]    = (State_Input_T)Intervention_InputControl,
    [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = (State_Input_T)Intervention_InputFeedbackMode,
};

const State_T MOTOR_STATE_INTERVENTION =
{
    .ID                 = MOTOR_STATE_ID_INTERVENTION,
    .ENTRY              = (State_Action_T)Intervention_Entry,
    .LOOP               = (State_Action_T)Intervention_Proc,
    .P_TRANSITION_TABLE = &INTERVENTION_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief State OpenLoop - OpenLoop, Align, and Start Up, Feedback Acquisition

    Only Entry is from [PASSIVE] State
*/
/******************************************************************************/
static void OpenLoop_Entry(Motor_T * p_motor)
{
    Phase_ActivateV0(&p_motor->PHASE);
    FOC_ClearCaptureState(&p_motor->P_MOTOR->Foc);
}

/*
    Proc SubState Tree
*/
static void OpenLoop_Proc(Motor_T * p_motor) { (void)p_motor; }

/* maintain consistent interface with Run, use substate cmd for phase output without exiting */
/* No resume from OpenLoop, freewheel state check stop */
static State_T * OpenLoop_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:      return &MOTOR_STATE_PASSIVE;
        case PHASE_VOUT_0:      return &MOTOR_STATE_PASSIVE;
        case PHASE_VOUT_PWM:    return NULL;
        default:                return NULL;
    }
}

static State_T * OpenLoop_InputDirection(Motor_T * p_motor, state_value_t direction)
{
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) == 0U)
    {
        switch ((Motor_Direction_T)direction)
        {
            case MOTOR_DIRECTION_NULL:  return &MOTOR_STATE_PASSIVE;
            /* Intentional fall-through */
            case MOTOR_DIRECTION_CW:
            case MOTOR_DIRECTION_CCW:
                _Motor_SetDirection(p_motor, direction); break;
            default: break; /* Invalid direction */
        }
    }
    return NULL;
}

static State_T * OpenLoop_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
{
    _Motor_SetFeedbackMode_Cast(p_motor, feedbackMode); /* a different flag mode will change ramp limits */
    return NULL;
}

/*
    handle exit, optionally handle cmds starting with a substate
    call sequence is preserved, same for cmds with and without substate. OpenLoop->Substate. OpenLoop->Cmd
*/
static State_T * OpenLoop_InputOpenLoop(Motor_T * p_motor, state_value_t statePtr)
{
    State_T * p_state = (State_T *)statePtr;

    if (p_state == NULL) { return &MOTOR_STATE_PASSIVE; }
    if (p_state == &MOTOR_STATE_OPEN_LOOP) { return &MOTOR_STATE_OPEN_LOOP; }
    if (p_state->P_TOP == &MOTOR_STATE_OPEN_LOOP) { return p_state; }

    return &MOTOR_STATE_PASSIVE;
}

static const State_Input_T OPEN_LOOP_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MOTOR_STATE_INPUT_PHASE_OUTPUT]    = (State_Input_T)OpenLoop_InputControl,
    [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = (State_Input_T)OpenLoop_InputFeedbackMode,
    [MOTOR_STATE_INPUT_OPEN_LOOP]       = (State_Input_T)OpenLoop_InputOpenLoop,
    [MOTOR_STATE_INPUT_DIRECTION]       = (State_Input_T)OpenLoop_InputDirection,
    [MOTOR_STATE_INPUT_CALIBRATION]     = NULL,
};

const State_T MOTOR_STATE_OPEN_LOOP =
{
    .ID                 = MOTOR_STATE_ID_OPEN_LOOP,
    .ENTRY              = (State_Action_T)OpenLoop_Entry,
    .LOOP               = (State_Action_T)OpenLoop_Proc,
    .P_TRANSITION_TABLE = &OPEN_LOOP_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief Calibration State
*/
/******************************************************************************/
static void Calibration_Entry(Motor_T * p_motor)
{
    Phase_ActivateV0(&p_motor->PHASE); /* Transition from Deactivated or Substates */
    p_motor->P_MOTOR->ControlTimerBase = 0U;
    // p_motor->P_MOTOR->CalibrationStateIndex = 0U;
}

static void Calibration_Proc(Motor_T * p_motor)
{
    // p_motor->P_MOTOR->CalibrationTimer++;
}


/* Calibration State and InputCalibration(DEACTIVATED) */
static State_T * Calibration_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:     Phase_Deactivate(&p_motor->PHASE); break;
        case PHASE_VOUT_0:     Phase_ActivateV0(&p_motor->PHASE); break;
        case PHASE_VOUT_PWM:   Phase_ActivateV0(&p_motor->PHASE); break; /* stay in calibration, user cmd to step through substates */
        default:               return NULL;
    }
    return NULL;
}

static State_T * Calibration_InputDirection(Motor_T * p_motor, state_value_t direction)
{
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) == 0U)
    {
        switch ((Motor_Direction_T)direction)
        {
            case MOTOR_DIRECTION_NULL:  /* Intentional fall-through */
            case MOTOR_DIRECTION_CW:
            case MOTOR_DIRECTION_CCW:
                _Motor_SetDirection(p_motor, direction); break;
            default: break; /* Invalid direction */
        }
    }
    return NULL;
}

/*
    provide exit, optionally handle cmds starting with a substate
*/
static State_T * Calibration_InputCalibration(Motor_T * p_motor, state_value_t statePtr)
{
    State_T * p_state = (State_T *)statePtr;

    assert(p_state == NULL || p_state == &MOTOR_STATE_DEACTIVATED || p_state == &MOTOR_STATE_CALIBRATION || p_state->P_TOP == &MOTOR_STATE_CALIBRATION);

    if (p_state == NULL) { return &MOTOR_STATE_PASSIVE; }
    if (p_state == &MOTOR_STATE_CALIBRATION) { return &MOTOR_STATE_CALIBRATION; }
    if (p_state->P_TOP == &MOTOR_STATE_CALIBRATION) { return p_state; }

    return &MOTOR_STATE_PASSIVE;

    // return p_state;
}

static const State_Input_T CALIBRATION_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MOTOR_STATE_INPUT_PHASE_OUTPUT]    = (State_Input_T)Calibration_InputControl,
    [MOTOR_STATE_INPUT_CALIBRATION]     = (State_Input_T)Calibration_InputCalibration,
    [MOTOR_STATE_INPUT_DIRECTION]       = (State_Input_T)Calibration_InputDirection,
    [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = NULL,
    [MOTOR_STATE_INPUT_OPEN_LOOP]       = NULL,
};

const State_T MOTOR_STATE_CALIBRATION =
{
    .ID                 = MOTOR_STATE_ID_CALIBRATION,
    .ENTRY              = (State_Action_T)Calibration_Entry,
    .LOOP               = (State_Action_T)Calibration_Proc,
    .P_TRANSITION_TABLE = &CALIBRATION_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Fault_Entry(Motor_T * p_motor) { Phase_Deactivate(&p_motor->PHASE); }

static void Fault_Proc(Motor_T * p_motor) { Phase_Deactivate(&p_motor->PHASE); }

/* Fault State: Set accumulates (latches), Clear removes flags and re-verifies */
static State_T * Fault_InputFault(Motor_T * p_motor, state_value_t faultCmd)
{
    Motor_FaultCmd_T cmd = { .Value = faultCmd };
    p_motor->P_MOTOR->FaultFlags.Value |= cmd.FaultSet;
    if (cmd.FaultClear != 0U)
    {
        p_motor->P_MOTOR->FaultFlags.Value &= ~cmd.FaultClear;
        Motor_PollFaultFlags(p_motor); /* Re-verify conditions resolved before allowing exit */
    }
    return (p_motor->P_MOTOR->FaultFlags.Value == 0U) ? &MOTOR_STATE_DEACTIVATED : NULL;
}

static State_T * Fault_InputCalibration(Motor_T * p_motor, state_value_t state)
{
    (void)state;
    State_T * p_nextState = NULL;

    // if (p_motor->P_MOTOR->FaultFlags.Value & ~POSITIONSENSOR)
    if (p_motor->P_MOTOR->FaultFlags.Overheat == 0U)
    {
        p_nextState = &MOTOR_STATE_CALIBRATION;
    }

    return p_nextState;
}

static const State_Input_T FAULT_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]       = (State_Input_T)Fault_InputFault,
    [MOTOR_STATE_INPUT_CALIBRATION] = (State_Input_T)Fault_InputCalibration,
    [MOTOR_STATE_INPUT_DIRECTION]   = NULL,
};

const State_T MOTOR_STATE_FAULT =
{
    .ID                 = MOTOR_STATE_ID_FAULT,
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
// void Motor_StateMachine_SetFault(Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
// {
//     StateMachine_Tree_InputAsyncTransition(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_FAULT, (Motor_FaultCmd_T){ .FaultSet = faultFlags.Value }.Value);
// }

// void Motor_StateMachine_ClearFault(Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
// {
//     StateMachine_Tree_InputAsyncTransition(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_FAULT, (Motor_FaultCmd_T){ .FaultClear = faultFlags.Value }.Value);
// }

// bool Motor_StateMachine_TryClearFaultAll(Motor_T * p_motor)
// {
//     Motor_StateMachine_ClearFault(p_motor, (Motor_FaultFlags_T){ .Value = UINT16_MAX });
//     return !Motor_IsFault(p_motor);
// }


// // inline bool Motor_IsClosedLoop(Motor_T * p_motor)
// // {
// //     return ((_Motor_IsSensorAvailable(p_motor) == true) && (_Motor_IsOpenLoop(p_motor) == false));
// // }

// Orthogonal Composition
// The critical architectural point: State objects naturally compose as orthogonal regions because each provides an independent behavioral filter over shared canonical data.
///The key is: States don't hold independent data — they provide a behavioral lens over canonical data.
// typedef state_value_t(*State_Data_T)(void * context);
// typedef State_Data_T State_DataVector_T[];

/* Alternatively orthogonal state  */
// static State_T * Ccw_Input(Motor_T * p_motor, state_value_t value)
// {

// }

// static void Ccw_Process(Motor_T * p_motor, state_value_t value)
// {
//     // uint16_t iLimitCcw = p_motor->P_MOTOR->ILimitMotoring_Fract16;
//     // uint16_t iLimitCw = p_motor->P_MOTOR->ILimitGenerating_Fract16;
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
// state_value_t Torque0_Ramp(Motor_T * p_motor) { return 0; }

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

// static void Run_ProcOrthogonal(Motor_T * p_motor)
// {
//     RunState_Vector_T * p_state = (RunState_Vector_T *)p_motor->P_MOTOR->StateMachine.p_Orthogonal;
//     fract16_t cmd = (p_state->TorqueCmd)(p_motor);
//     // Motor_FOC_ProcInnerFeedback(p_motor->P_MOTOR, 0, cmd);
//     // Motor_FOC_ProcAngleOutput(p_motor->P_MOTOR);
// }


// typedef struct State_VTable
// {
//     State_T * (*InputControl)(Motor_T * p_motor, state_value_t value);
//     State_T * (*InputDirection)(Motor_T * p_motor, state_value_t direction);
//     State_T * (*InputFeedbackMode)(Motor_T * p_motor, state_value_t feedbackMode);
//     State_T * (*InputOpenLoop)(Motor_T * p_motor, state_value_t statePtr);
//     State_T * (*InputCalibration)(Motor_T * p_motor, state_value_t statePtr);
// } State_VTable_T;

/* Orthgonal States */
// typedef struct Motor_StateIntervention
// {
//     State_T BASE;
//     struct
//     {
//         // state_value_t(*Condition)(const Motor_Context_T *);
//         state_value_t(*REQ_OVERRIDE) (const Motor_Context_T *);
//     };
// }
// Motor_StateIntervention_T;

// pass top state return?
// static void Torque0_Proc(Motor_Context_T * p_motor)
// {

// }

// static state_value_t Torque0_Ramp(const Motor_Context_T * p_motor) { return 0; }

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

// static void ProcTorque0(Motor_Context_T * p_motor)
// {
//  int a =   MOTOR_STATE_TORQUE_ZERO.DataVector[1](p_motor);
//     // Motor_FOC_ProcInnerFeedback(p_motor, p_motor->SensorState.AngleSpeed.Angle, 0, Motor_IRampOf(p_motor, 0));
//     // Motor_FOC_ProcAngleOutput(p_motor);
// }
