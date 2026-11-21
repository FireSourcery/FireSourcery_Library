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
#include "Sensor/Motor_Sensor.h" /* Motor_Sensor_StartUpState */
#include "Motor/Motor/VBus/VBus_Monitor.h"
#include "StateMachine/Motor_Intervention.h"
#include "Motor_Config.h"


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

/* Run and Intervention: already on the requested branch is a no-op, repeated requests hold without re-entry */
static State_T * TransitionIntervention(Motor_T * p_motor, state_value_t statePtr)
{
    State_T * p_state = (State_T *)statePtr;
    if (p_state == NULL) { return NULL; }
    if (p_state->P_TOP != &MOTOR_STATE_INTERVENTION) { return NULL; }
    return StateMachine_IsActiveBranch(p_motor->STATE_MACHINE.P_ACTIVE, p_state) ? NULL : p_state;
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
// static void SetDirection(Motor_T * p_dev, Motor_Direction_T direction)
// {
//     Motor_SetDirection(p_dev, direction);
//     Motor_FOC_SetDirection(p_dev, direction); // let foc set direction  handle vlimit
// }

/* if eliminating Motor_Context.Direction which shadows FOC.VLimit */
// static Motor_Direction_T _Motor_DirectionV(const Motor_T * p_dev)
// {
//     return sign_of(p_dev->P_MOTOR->Foc.VLimit);
// }

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
        Motor_ValidateConfig(p_motor); /* limit VAlign and field weakening. todo handle by var id */
        // if (Phase_Board_IsValid() == false) { p_motor->P_MOTOR->FaultFlags.InitCheck = 1U; } /* alternatively go to fault, outer module parse */
        if (Motor_IsConfigValid(p_motor) == false) { p_motor->P_MOTOR->FaultFlags.InitCheck = 1U; }

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
    .PATH_ID            = { .Depth0 = MOTOR_STATE_ID_INIT },
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
    Phase_WriteDuty(&p_motor->PHASE, PWM_DUTY_MAX / 2U, PWM_DUTY_MAX / 2U, PWM_DUTY_MAX / 2U);/* clear the duties for resume cycle. */
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR); // Motor_CommutationModeFn_Call(p_motor->P_MOTOR, Motor_FOC_ClearFeedbackState, NULL); /* Unobserved values remain 0 for user read */
    Motor_SetDirection(p_motor, MOTOR_DIRECTION_NULL);
    Motor_ResetControlTimer(p_motor->P_MOTOR); /* ok to reset timer */
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
            Motor_SetDirection(p_motor, direction); break;
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
    return NULL;
}

/* Calibration go directly to SubState */
static State_T * Deactivated_InputCalibration(Motor_T * p_motor, state_value_t statePtr)
{
    if ((State_T *)statePtr == NULL) { return NULL; }
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
    .PATH_ID            = { .Depth0 = MOTOR_STATE_ID_DEACTIVATED },
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
    Motor_ActivatePassiveOutput(p_motor);
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR); // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ClearFeedbackState, NULL);
    Motor_ResetControlTimer(p_motor->P_MOTOR); /* ok to reset timer */
 // optionally set backup vspeed
}

static void Passive_Proc(Motor_T * p_motor)
{
    /* Match Feedback to ProcAngleBemf on Resume */
    Motor_FOC_ProcCaptureAngleVBemf(p_motor->P_MOTOR);    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL /* Motor_SixStep_ProcPhaseObserve */);
    // switch (Phase_ReadVOut(&p_motor->PHASE))
    // {
        // case PHASE_VOUT_Z:  if (!Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR)) { Phase_ActivateV0(&p_motor->PHASE); }   break;
    //     default: break;
    // }
}

static State_T * Passive_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    State_T * p_nextState = NULL;

    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:
            if (Motor_IsSpeedFreewheelLimitRange(p_motor)) { Phase_Deactivate(&p_motor->PHASE); }
            break;
        case PHASE_VOUT_0:
            Phase_ActivateV0(&p_motor->PHASE);
            break;
        case PHASE_VOUT_PWM:
            if (Motor_Direction(p_motor->P_MOTOR) != MOTOR_DIRECTION_NULL)
            {
                //optionally substate for wait bemf capture
                if (RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR->p_ActiveSensor) == true) { p_nextState = &MOTOR_STATE_RUN; }
                /* From standstill, the active sensor's start-up chain, a sub-state of OPEN_LOOP. NULL where it has none: no transition */
                else if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) == 0U) { p_nextState = Motor_Sensor_StartUpState(p_motor); }
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
                Motor_SetDirection(p_motor, direction); break;
            default: break; /* Invalid direction */
        }
    }

    return p_nextState;
}

static State_T * Passive_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
{
    _Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
    return NULL;
}


/*
    Accept a branch entry, same as OpenLoop_InputOpenLoop — a cmd that begins at a
    substate (sensorless/encoder start-up chains) must reach its substate directly,
    not just the OPEN_LOOP root. Traverse runs MOTOR_STATE_OPEN_LOOP.ENTRY first.
*/
static State_T * Passive_InputOpenLoop(Motor_T * p_motor, state_value_t statePtr)
{
    State_T * p_state = (State_T *)statePtr;

    if (p_state == NULL) { return NULL; } /* Exit request, not on this branch */
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) != 0U) { return NULL; }
    if (p_state->P_TOP == &MOTOR_STATE_OPEN_LOOP) { return p_state; }
    // if (p_state == NULL) { return &MOTOR_STATE_OPEN_LOOP; }
    // if (p_state == &MOTOR_STATE_OPEN_LOOP) { return &MOTOR_STATE_OPEN_LOOP; }
    // if (p_state->P_TOP == &MOTOR_STATE_OPEN_LOOP) { return p_state; }
    return &MOTOR_STATE_OPEN_LOOP;
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
    .PATH_ID            = { .Depth0 = MOTOR_STATE_ID_PASSIVE },
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

    /* Takes the slots before matching, which clamps against them */
    Motor_FOC_ResolveISupplyLimitsOnEntry(p_motor);
    Motor_ResolveFeedbackLimits(p_motor);
    Motor_ResolveSpeedLimits(p_motor->P_MOTOR);

    if (Phase_ReadVOut(&p_motor->PHASE) == PHASE_VOUT_Z) /* prev state is freewheel */
    {
        /* Vabc maybe 0 on entry, if capture vbemf did not complete */
        Motor_FOC_MatchVFreewheel(p_context, VBus_Pu(p_motor->P_VBUS));
        Phase_ActivateT0(&p_motor->PHASE);
    }
    else  /* else previous state is run/intervention */
    {
        // if (p_context->FeedbackMode.Current == 0U)
        Motor_FOC_MatchVOutput(p_context);
    }

    if (Motor_FeedbackMode_IsVoltage(p_context->FeedbackMode)) { Motor_FOC_MatchTorqueVState(p_context); } else { Motor_FOC_MatchTorqueIState(p_context); }

    /* UserCmd units follow FeedbackMode, which may have changed since the last user write [Run_InputFeedbackMode]. Held until the next, ~1-50ms */
    if (p_context->FeedbackMode.Speed == 1U)
    {
        Motor_MatchSpeedTorqueState(p_context, Ramp_GetOutput(&p_context->TorqueRamp));
        p_context->UserCmd = Motor_GetSpeedFeedback(p_context);
    }
    else
    {
        p_context->UserCmd = Ramp_GetOutput(&p_context->TorqueRamp); /* Iq or Vq, as matched */
    }

    // Phase_ActivateT0(&p_motor->PHASE);    // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_ActivateOutput, NULL);
}

static void Run_Proc(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    fract16_t req = (p_context->FeedbackMode.Speed == 1U) ? PID_GetOutput(&p_context->PidSpeed) : p_context->UserCmd;

    if (Motor_FeedbackMode_IsVoltage(p_context->FeedbackMode)) { Motor_FOC_ProcAngleV(p_motor, req); } else { Motor_FOC_ProcTorqueReq(p_motor, req); }

    // #ifdef MOTOR_EXTERN_CONTROL_ENABLE
    //     Motor_ExternControl(p_motor);
    // #endif

    // _Motor_FOC_ProcOnInput
    // RotorSensor_Callback
    // _Motor_FOC_ProcReqStep
    // Motor_FOC_ProcIControl

    Motor_FOC_WriteDuty(p_motor);
}

/******************************************************************************/
/*!
    [RUN_ACTION_TABLE]
*/
/******************************************************************************/
/*
    I Bus power balance is derived from speed sign
*/
static void Run_OnSpeed(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    Motor_FOC_CaptureVSpeed(p_context);
    Motor_FOC_ResolveISupplyLimitsOnSpeed(p_motor);
    Motor_ResolveFeedbackLimits(p_motor); /* Before the speed loop clamps against them */
    Motor_ResolveSpeedLimits(p_context);
    if (p_context->FeedbackMode.Speed == 1U) { Motor_ProcSpeedControlOf(p_context, p_context->UserCmd); }
}

// static void Run_OnIPhase(Motor_T * p_motor)
// {
//     Motor_Context_T * p_context = p_motor->P_MOTOR;
//     Motor_FOC_ProcTorqueFeedback(p_motor);
//     Motor_FOC_WriteDuty(p_motor);
// }

static const State_Action_T RUN_ACTION_TABLE[MOTOR_STATE_ACTION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_ON_SPEED] = (State_Action_T)Run_OnSpeed,
#if defined(MOTOR_I_FEEDBACK_ON_PHASE)
    [MOTOR_STATE_INPUT_ON_PHASE] = (State_Action_T)Motor_FOC_ProcIFeedback,
#endif
};

/******************************************************************************/
/*!
    [RUN_TRANSITION_TABLE]
*/
/******************************************************************************/
static State_T * Run_InputRelease(Motor_T * p_motor)
{
    return Motor_IsSpeedFreewheelLimitRange(p_motor) ? &MOTOR_STATE_PASSIVE : &INTERVENTION_STATE_SPEED_ZERO;
}

/* App layer handle conditional release for now. substate for passive torque 0. change input from user cmd */
static State_T * Run_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:      return Run_InputRelease(p_motor);
        case PHASE_VOUT_0:      return Run_InputRelease(p_motor);  /* return Motor_IsSpeedFreewheelLimitRange(p_motor) ? &MOTOR_STATE_PASSIVE_V0 : &INTERVENTION_STATE_SPEED_ZERO; */
        case PHASE_VOUT_PWM:    return NULL;
        default:                return NULL;
    }
}

/* */
static State_T * Run_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
{
    _Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
    return &MOTOR_STATE_RUN;  /* Run_Entry Procs synchronous  Alternatively, transition through Freewheel */
    // Motor_FOC_MatchFeedbackState(p_motor); //  Match inline without self-transition
    // return NULL;
}

/*
    Feedback lost: the NEXT of the states that drive on the sensor's angle, RUN and INTERVENTION, and the substates that fall through to them.
    Hall and SinCos never report it, Encoder holds its alignment, the observer releases its lock [AnglePll_FilterTuning_T].
*/
static State_T * _Motor_TransitionFeedbackLost(Motor_T * p_motor)
{
    if (RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR->p_ActiveSensor) == true) { return NULL; }
#if defined(MOTOR_OPEN_LOOP_FALLBACK_ENABLE)
    State_T * p_fallback = Motor_Sensor_FallbackState(p_motor); /* The open loop from the sensor's last angle, NULL where it has none or lost it */
    if (p_fallback != NULL) { return p_fallback; }
#endif
    if (Motor_IsSpeedFreewheelLimitRange(p_motor) == true) { return &MOTOR_STATE_PASSIVE; }
    p_motor->P_MOTOR->FaultFlags.PositionSensor = 1U;
    return &MOTOR_STATE_FAULT;
}

static const State_Input_T RUN_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = (State_Input_T)Run_InputFeedbackMode,
    [MOTOR_STATE_INPUT_PHASE_OUTPUT]    = (State_Input_T)Run_InputControl,
    // [MOTOR_STATE_INPUT_DIRECTION]       = (State_Input_T)Run_InputStop, /* optionally accept transition to passive */
    [MOTOR_STATE_INPUT_CALIBRATION]     = NULL,
    [MOTOR_STATE_INPUT_INTERVENTION]    = (State_Input_T)TransitionIntervention,
    [MOTOR_STATE_INPUT_OPEN_LOOP]       = NULL,
};

const State_T MOTOR_STATE_RUN =
{
    .ID                 = MOTOR_STATE_ID_RUN,
    .PATH_ID            = { .Depth0 = MOTOR_STATE_ID_RUN },
    .P_TRANSITION_TABLE = &RUN_TRANSITION_TABLE[0U],
    .P_ACTION_TABLE     = &RUN_ACTION_TABLE[0U],
    .ENTRY              = (State_Action_T)Run_Entry,
    .LOOP               = (State_Action_T)Run_Proc,
    .NEXT               = (State_Input0_T)_Motor_TransitionFeedbackLost,
};


/******************************************************************************/
/*!
    @brief State Intervention - Motor-level safety supervisor
    Active control with an alternative control variable path.
      TORQUE_ZERO (ZTC) - Coast with zero torque, user may resume
      SPEED_ZERO        - Active deceleration to zero, user may resume
      SAFE_STOP   (SS1) - SPEED_ZERO, resume rejected

    FeedbackMode flags ignored, userCmd augmented.
*/
/******************************************************************************/
static void Intervention_Entry(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    Motor_FOC_ResolveISupplyLimitsOnEntry(p_motor);
    Motor_ResolveILimits(p_context); /* Current control regardless of FeedbackMode. SpeedPid output is I. */
    Motor_FOC_MatchTorqueIState(p_context);
    Motor_FOC_MatchVOutput(p_context);
}

// min(GeneratingOnly, _Motor_GeneratingOnly PID_GetOutput(&p_state->PidSpeed));  substates inherit
// Motor_FOC_ProcTorqueReq(p_motor, _Motor_GeneratingOnly(p_state, Ramp_GetTarget(&p_state->TorqueRamp)));
static void Intervention_Proc(Motor_T * p_motor) { (void)p_motor; }

/*
    Inherited by TorqueZero and SpeedZero.
    SafeStop overrides to reject resume.
*/
static State_T * Intervention_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:
        case PHASE_VOUT_0:      return Motor_IsSpeedFreewheelLimitRange(p_motor) ? &MOTOR_STATE_PASSIVE : &INTERVENTION_STATE_SPEED_ZERO;
        case PHASE_VOUT_PWM:    return RotorSensor_IsFeedbackAvailable(p_motor->P_MOTOR->p_ActiveSensor) ? &MOTOR_STATE_RUN : NULL; /* TorqueZero may run without feedback */
        default: return NULL;
    }
}

/* Run_Entry resolves and matches on resume */
static State_T * Intervention_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
{
    p_motor->P_MOTOR->FeedbackMode = Motor_FeedbackMode_Cast(feedbackMode);
    return NULL;
}

static const State_Input_T INTERVENTION_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    [MOTOR_STATE_INPUT_PHASE_OUTPUT]    = (State_Input_T)Intervention_InputControl,
    [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = (State_Input_T)Intervention_InputFeedbackMode,
    [MOTOR_STATE_INPUT_INTERVENTION]    = (State_Input_T)TransitionIntervention,
};

const State_T MOTOR_STATE_INTERVENTION =
{
    .ID                 = MOTOR_STATE_ID_INTERVENTION,
    .PATH_ID            = { .Depth0 = MOTOR_STATE_ID_INTERVENTION },
    .ENTRY              = (State_Action_T)Intervention_Entry,
    .LOOP               = (State_Action_T)Intervention_Proc,
    .NEXT               = (State_Input0_T)_Motor_TransitionFeedbackLost, /* the substates' NEXT run first; a lost angle ends SPEED_ZERO as it ends RUN */
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
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR);
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
    // Phase_ActivateVOut(&p_motor->PHASE, (Phase_VOutMode_T)phaseState);
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
                Motor_SetDirection(p_motor, direction); break;
            default: break; /* Invalid direction */
        }
    }
    return NULL;
}

static State_T * OpenLoop_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
{
    _Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
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
    .PATH_ID            = { .Depth0 = MOTOR_STATE_ID_OPEN_LOOP },
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
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR);
    // Motor_SetDirection(p_motor, Motor_GetDirectionForward(p_motor->P_MOTOR)); /* reset direction limits */
    Motor_ResetControlTimer(p_motor->P_MOTOR);
}

static void Calibration_Proc(Motor_T * p_motor)
{

}

/* Calibration State and InputCalibration(DEACTIVATED) */
static State_T * Calibration_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:     Phase_Deactivate(&p_motor->PHASE); return &MOTOR_STATE_PASSIVE;
        case PHASE_VOUT_0:     Phase_ActivateV0(&p_motor->PHASE); break;
        case PHASE_VOUT_PWM:   Phase_ActivateV0(&p_motor->PHASE); break; /* stay in calibration, user cmd to step through substates */
        default:    return NULL;
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
                Motor_SetDirection(p_motor, direction); break;
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
    .PATH_ID            = { .Depth0 = MOTOR_STATE_ID_CALIBRATION },
    .ENTRY              = (State_Action_T)Calibration_Entry,
    .LOOP               = (State_Action_T)Calibration_Proc,
    .P_TRANSITION_TABLE = &CALIBRATION_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
/*
    The passive output for every flag, RequestV0 included: float within the freewheel limit, V0 above it.
    Within the limit the back-EMF is below the bus, so a floated bridge feeds it nothing,
    while V0 would short the windings through the low sides with no current control.
*/
static void Fault_Entry(Motor_T * p_motor)
{
    Motor_ActivatePassiveOutput(p_motor);
}

/* Escalate only: a floated bridge pushed past the freewheel limit rectifies into the bus. V0 then holds until the clear */
static void Fault_Proc(Motor_T * p_motor)
{
    if ((Motor_IsSpeedFreewheelLimitRange(p_motor) == false) && (Phase_IsFloat(&p_motor->PHASE) == true)) { Phase_ActivateV0(&p_motor->PHASE); }
}

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

/*
    No CALIBRATION from FAULT: entering it would leave FAULT with the flags still set, past the clear.
    Calibration commands are admitted in [MC_STATE_LOCK] only, reached after a clear.
*/
static const State_Input_T FAULT_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]       = (State_Input_T)Fault_InputFault,
    [MOTOR_STATE_INPUT_CALIBRATION] = NULL,
    [MOTOR_STATE_INPUT_DIRECTION]   = NULL,
};

const State_T MOTOR_STATE_FAULT =
{
    .ID                 = MOTOR_STATE_ID_FAULT,
    .PATH_ID            = { .Depth0 = MOTOR_STATE_ID_FAULT },
    .P_TRANSITION_TABLE = &FAULT_TRANSITION_TABLE[0U],
    .ENTRY              = (State_Action_T)Fault_Entry,
    .LOOP               = (State_Action_T)Fault_Proc,
};

