/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Motor_Sensorless.c
    @author FireSourcery
    @brief  Sensorless start-up substate chain under MOTOR_STATE_OPEN_LOOP.

        SENSORLESS_ALIGN     — DC align, observer runs to settle î against the
                               real current. Timer expiry → SENSORLESS_START_UP.
        SENSORLESS_START_UP  — open-loop speed/current ramp, observer tracks the
                               rising EMF. Observer lock → MOTOR_STATE_RUN.

    The observer corrects after the state machine, on every tick a state wrote the FOC's command [Motor_CaptureStator]:
    here, in RUN and in INTERVENTION alike. The chain's FOC runs at the ramp, so the correction Parks its take at θ̂.

    Both substates mount directly on MOTOR_STATE_OPEN_LOOP rather than on the
    generic OPEN_LOOP_STATE_* states. Inheriting LOOP from those would also
    inherit their NEXT: State_TransitionOfOutputUp keeps walking to the parent
    whenever a child NEXT returns NULL, so OPEN_LOOP_STATE_START_UP_ALIGN's
    timer check would still fire and divert the chain to OPEN_LOOP_STATE_RUN.
    MOTOR_STATE_OPEN_LOOP has an empty LOOP and no NEXT, so it is inert to walk
    up to — the actuation is instead composed here from the Motor_FOC procs.
*/
/******************************************************************************/
#include "Motor_Sensorless.h"

// #if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)

static const State_T SENSORLESS_COAST;
static const State_T SENSORLESS_ALIGN;
static const State_T SENSORLESS_START_UP;
static const State_T SENSORLESS_FALLBACK;

/* Keep it compiling. */
#if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
static inline const Sensorless_Sensor_T * GetSensorless(Motor_T * p_motor) { return &p_motor->SENSOR_TABLE.SENSORLESS; }
#else
static inline const Sensorless_Sensor_T * GetSensorless(Motor_T * p_motor) { return NULL; }
#endif


/******************************************************************************/
/*
    Coast — a start pending under PASSIVE until PASSIVE has lasted the coast time [MOTOR_SENSORLESS_COAST_CYCLES], then ALIGN.
    The chain's entry from PASSIVE at speed 0 [Motor_Sensorless_StartUpState]: the request is taken and waited out, not refused,
    as the application sends it once, on the throttle's edge.
    No entry, no LOOP: PASSIVE's phase state stays, and its LOOP keeps the capture, the observer's input, true.
    Entered and left within PASSIVE, PASSIVE is not entered again: ControlTimerBase counts on from its entry [Motor_ResetControlTimer].
*/
/******************************************************************************/
/* Cancelled by a cleared direction. Caught where the observer reaches feedback, as PASSIVE's PWM request is */
static State_T * Coast_Next(Motor_T * p_motor)
{
    if (Motor_Direction(p_motor->P_MOTOR) == MOTOR_DIRECTION_NULL) { return &MOTOR_STATE_PASSIVE; }
    if (RotorSensor_IsFeedbackAvailable(&GetSensorless(p_motor)->BASE) == true) { return &MOTOR_STATE_RUN; }
    return (p_motor->P_MOTOR->ControlTimerBase >= MOTOR_SENSORLESS_COAST_CYCLES) ? &SENSORLESS_ALIGN : NULL;
}

/* Overrides PASSIVE's: Z or 0 cancels the pending start, back to PASSIVE, with the phase state PASSIVE applies. PWM is already pending */
static State_T * Coast_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:
            if (Motor_IsSpeedFreewheelLimitRange(p_motor)) { Phase_Deactivate(&p_motor->PHASE); }
            return &MOTOR_STATE_PASSIVE;
        case PHASE_VOUT_0:
            Phase_ActivateV0(&p_motor->PHASE);
            return &MOTOR_STATE_PASSIVE;
        case PHASE_VOUT_PWM:    return NULL;
        default:                return NULL;
    }
}

static const State_Input_T COAST_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_PHASE_OUTPUT] = (State_Input_T)Coast_InputControl,
};

static const State_T SENSORLESS_COAST =
{
    .PATH_ID    = { .Depth0 = MOTOR_STATE_ID_PASSIVE }, /* sub-state id not yet allocated: reports as the root state */

    .P_TOP    = &MOTOR_STATE_PASSIVE,
    .P_PARENT = &MOTOR_STATE_PASSIVE,
    .DEPTH    = 1U,
    .NEXT     = (State_Input0_T)Coast_Next,
    .P_TRANSITION_TABLE = &COAST_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*
    Align — hold θ=0 on a current ramp until the rotor settles.
*/
/******************************************************************************/
static void Align_Entry(Motor_T * p_motor)
{
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles);
    Phase_ActivateV0(&p_motor->PHASE); /* OpenLoop may have been left floating by a phase-output cmd */
    Motor_FOC_StartStartUpAlign(p_motor->P_MOTOR); /* zeroes OpenLoopAngle, selects current feedback */
    RotorSensor_ZeroInitial(&GetSensorless(p_motor)->BASE);
}
// FOC_SetVLimits(&p_context->Foc, (sign_t)Motor_GetDirectionForward(p_context), VBus_GetVPhaseRefSvpwm(p_motor->P_VBUS));


/*
    Observer runs through align even though EMF is ~0 and lock cannot assert:
    it converges the SMO current estimate î onto the applied DC, so the ramp
    does not start from a step error. It corrects after the state machine [Motor_CaptureStator].
*/
static void Align_Proc(Motor_T * p_motor)
{
    Motor_FOC_ProcStartUpAlign(p_motor);
    Motor_FOC_WriteDuty(p_motor);
}

static State_T * Align_Next(Motor_T * p_motor)
{
    return (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true) ? &SENSORLESS_START_UP : NULL;
}

static const State_T SENSORLESS_ALIGN =
{
    .PATH_ID    = { .Depth0 = MOTOR_STATE_ID_OPEN_LOOP }, /* sub-state id not yet allocated: reports as the root state */

    .P_TOP    = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)Align_Entry,
    .LOOP     = (State_Action_T)Align_Proc,
    .NEXT     = (State_Input0_T)Align_Next,
};


/******************************************************************************/
/*
    Start Up — open-loop ramp, observer tracks until it reports lock.
*/
/******************************************************************************/
/* The PLL at the ramp's angle and speed */
static void StartUp_SeedRamp(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    EmfObserver_SeedAngle(GetSensorless(p_motor)->P_OBSERVER, Angle_Value(&p_context->OpenLoopAngle), Angle_Delta(&p_context->OpenLoopAngle));
}

static void StartUp_Entry(Motor_T * p_motor)
{
    Motor_FOC_StartOpenLoop(p_motor->P_MOTOR);
    StartUp_SeedRamp(p_motor); /* converges from the ramp rather than from 0 */
}

/*
    The ramp is the reference until |ê| reads above the floor: below it the correction holds θ̂, ω̂ [EmfObserver_IsIndeterminate].
    Seeded only at entry, the ramp then at 0 speed, θ̂ stays there while the rotor turns. The first errors above the floor land at
    whatever phase the rotor has: about half the time they drive ω̂ against it, and sign(ω̂) then inverts the rotor vector [_EmfObserver_Direction].
    After the write, before the correction [Motor_CaptureStator]: re-seeded, θ̂ is the angle the FOC Parked at this tick, and the correction takes its Id, Iq.
    Once |ê| holds above the floor, θ̂ leaves the ramp: the correction still takes Id, Iq, Parked at the ramp, off θ̂ by the load angle [EmfObserver_ParkI].
*/
static void StartUp_Proc(Motor_T * p_motor)
{
    Motor_FOC_ProcOpenLoop(p_motor);
    Motor_FOC_WriteDuty(p_motor);
    if (EmfObserver_IsIndeterminate(GetSensorless(p_motor)->P_OBSERVER) == true) { StartUp_SeedRamp(p_motor); }
}

/*
    Hand off to closed loop. θ̂ is already published into RotorSensor_State by
    CAPTURE_ANGLE each tick, so MOTOR_STATE_RUN reads a live angle on entry;
    lock (strong |ê| + tight PLL error, sustained) is what bounds the step
    between the commanded ramp angle and θ̂. The lock is the last tick's: the correction runs after the state machine.
    The FOC's operating point re-Parked from the ramp's angle to θ̂, which Run_Entry seeds the loop and the ramp from.
*/
static State_T * StartUp_Next(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    if (RotorSensor_IsFeedbackAvailable(&GetSensorless(p_motor)->BASE) == false) { return NULL; }
    FOC_MatchTheta(&p_context->Foc, Angle_Value(&p_context->SensorState.AngleSpeed));
    p_context->FeedbackMode.OpenLoop = 0U;
    return &MOTOR_STATE_RUN;
}

static const State_T SENSORLESS_START_UP =
{
    .PATH_ID    = { .Depth0 = MOTOR_STATE_ID_OPEN_LOOP }, /* sub-state id not yet allocated: reports as the root state */

    .P_TOP    = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)StartUp_Entry,
    .LOOP     = (State_Action_T)StartUp_Proc,
    .NEXT     = (State_Input0_T)StartUp_Next,
};


/******************************************************************************/
/*
    Fallback — from RUN, |ê| fallen below MagRun with the lock still held [Motor_Sensorless_FallbackState].
    START_UP's seed reversed: the ramp takes the observer's angle and speed, its current the final value at once. Then START_UP's ramp and hand-off,
    back to RUN once |ê| holds above MagRun again.
    The ramp drives toward OpenLoopRampSpeedFinal, whatever the user's command: a speed-held load's fallback, not traction's.
*/
/******************************************************************************/
static void Fallback_Entry(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    Angle_SetAngle(&p_context->OpenLoopAngle, Angle_Value(&GetSensorless(p_motor)->P_OBSERVER->Pll.Base));
    Ramp_SetOutputState(&p_context->OpenLoopSpeedRamp, Motor_GetSpeedFeedback(p_context));
    Ramp_SetOutputState(&p_context->OpenLoopIRamp, math_min(p_context->Config.OpenLoopRampIFinal_Pu, Motor_ILimitMotoring(p_context)) * Motor_Direction(p_context));
}

static const State_T SENSORLESS_FALLBACK =
{
    .PATH_ID    = { .Depth0 = MOTOR_STATE_ID_OPEN_LOOP }, /* sub-state id not yet allocated: reports as the root state */

    .P_TOP    = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH    = 1U,
    .ENTRY    = (State_Action_T)Fallback_Entry,
    .LOOP     = (State_Action_T)StartUp_Proc,
    .NEXT     = (State_Input0_T)StartUp_Next,
};


/******************************************************************************/
/*
    Entry points
*/
/******************************************************************************/
/*
    Start the sensorless align → ramp → closed-loop chain.
    Valid only while the sensorless adapter is the selected sensor — the chain
    hands to MOTOR_STATE_RUN on the observer's lock, and RUN reads its angle
    from p_ActiveSensor.
*/
void Motor_Sensorless_StartRunChain(Motor_T * p_motor)
{
    if (Motor_Direction(p_motor->P_MOTOR) == MOTOR_DIRECTION_NULL) { return; }
    if (p_motor->P_MOTOR->p_ActiveSensor != &GetSensorless(p_motor)->BASE) { return; }
    Motor_OpenLoop_EnterBranch(p_motor, &SENSORLESS_ALIGN); /* a command: no coast, which is PASSIVE's */
}

/*
    The chain that acquires the angle from standstill, for an entry from outside: a PWM request in PASSIVE, or a cmd.
    NULL unless the sensorless adapter is the active sensor: the chain waits on the observer, which only the active sensor runs [Motor_CaptureStator].
    Config.SensorMode can name it before a reboot makes it active.
*/
/* ALIGN once PASSIVE has lasted the coast time, the coast first otherwise: a PASSIVE substate, entered without leaving PASSIVE */
State_T * Motor_Sensorless_StartUpState(Motor_T * p_motor)
{
    if (p_motor->P_MOTOR->p_ActiveSensor != &GetSensorless(p_motor)->BASE) { return NULL; }
    return (p_motor->P_MOTOR->ControlTimerBase >= MOTOR_SENSORLESS_COAST_CYCLES) ? &SENSORLESS_ALIGN : &SENSORLESS_COAST;
}

// State_T * Motor_Sensorless_StartUpState(Motor_T * p_motor) { (void)p_motor; return &SENSORLESS_ALIGN; }

/*
    The open-loop fallback, for RUN's feedback loss [Motor_Sensor_FallbackState]: the ramp from θ̂, ω̂ while they are still the rotor's.
    Locked, |ê| below MagRun: the observer still tracks. A lost lock leaves no angle to seed: NULL, and RUN leaves as without the fallback.
*/
State_T * Motor_Sensorless_FallbackState(Motor_T * p_motor)
{
    if (p_motor->P_MOTOR->p_ActiveSensor != &GetSensorless(p_motor)->BASE) { return NULL; }
    return (EmfObserver_IsLocked(GetSensorless(p_motor)->P_OBSERVER) == true) ? &SENSORLESS_FALLBACK : NULL;
}


/******************************************************************************/
/*
    Sequence B, not wired: the correction between the take and the loop.
    The correction on the FOC's take reads i(k) and v(k−1) in either order, the same ω̂ for the next prediction:
        A   after the write, built: the sensor's CAPTURE_STATOR after the state machine [Motor_CaptureStator]. The hold keeps v(k−1)
        B   between the take and the loop: Foc still holds v(k−1), no hold; the write later by the correction's cost
*/
/******************************************************************************/
/*
    B: the torque path [_Motor_FOC_ProcTorqueReq] with the correction between the take and the loop. The caller writes.
    i(k) at θ̂(k) from the take, v(k−1) in Valpha/Vbeta, the loop not yet run.
    Inlines a second copy of the loop beside [Motor_FOC_AngleControl]'s.
    Its LOOP writes through Motor_FOC_WriteDuty, so CAPTURE_STATOR would correct again after the state machine: B's state writes without the mark, or the mark is cleared.
*/
// void Motor_Sensorless_ProcTorqueReq(Motor_T * p_motor, fract16_t req)
// {
//     Motor_Context_T * p_context = p_motor->P_MOTOR;
//     const FOC_T * p_foc = &p_context->Foc;
//     ufract16_t vBus = VBus_Pu(p_motor->P_VBUS);
//     fract16_t id = _Motor_FOC_ProcIdReq(p_context, vBus);
//     fract16_t iq = Ramp_ProcNextOf(&p_context->TorqueRamp, req);

//     Motor_FOC_CaptureI(p_context, Angle_Value(&p_context->SensorState.AngleSpeed));
//     EmfObserver_Eemf_ProcVAlphaBeta(GetSensorless(p_motor)->P_OBSERVER, FOC_Id(p_foc), FOC_Iq(p_foc), p_foc->Valpha, p_foc->Vbeta);
//     Motor_FOC_ProcIControl(p_context, vBus, id, iq);
// }

/*
    SENSORLESS_RUN, kept for B: an alternate MOTOR_STATE_RUN, top level, entered from START_UP's lock.
    RUN's entry, torque path and action table, exported from Motor_StateMachine.c: _Motor_Run_Entry, MOTOR_RUN_ACTION_TABLE,
    _Motor_TransitionFault, _Motor_TransitionFeedbackLost. Its LOOP runs B's torque path [Motor_Sensorless_ProcTorqueReq], then the write.
    Release and intervention requests coast in PASSIVE; every exit resets the observer [ExitObserver].
*/
// static const State_T SENSORLESS_RUN;
//
// static State_T * ExitObserver(Motor_T * p_motor, State_T * p_next)
// {
//     if (p_next != NULL) { RotorSensor_ZeroInitial(&GetSensorless(p_motor)->BASE); }
//     return p_next;
// }
//
// static void Run_Proc(Motor_T * p_motor)
// {
//     Motor_Context_T * p_context = p_motor->P_MOTOR;
//     Motor_Sensorless_ProcTorqueReq(p_motor, (p_context->FeedbackMode.Speed == 1U) ? PID_GetOutput(&p_context->PidSpeed) : p_context->UserCmd);
//     Motor_FOC_WriteDuty(p_motor);
// }
//
// /* The lock is this tick's: the LOOP corrected before */
// static State_T * Run_Next(Motor_T * p_motor) { return ExitObserver(p_motor, _Motor_TransitionFeedbackLost(p_motor)); }
//
// static State_T * Run_InputFault(Motor_T * p_motor, state_value_t faultCmd) { return ExitObserver(p_motor, _Motor_TransitionFault(p_motor, faultCmd)); }
//
// /* RUN's entry matches the new mode */
// static State_T * Run_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
// {
//     Motor_SetFeedbackMode(p_motor, Motor_FeedbackMode_Cast(feedbackMode));
//     return &SENSORLESS_RUN;
// }
//
// static State_T * Run_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
// {
//     switch ((Phase_VOutMode_T)phaseOutput)
//     {
//         case PHASE_VOUT_Z:      return ExitObserver(p_motor, &MOTOR_STATE_PASSIVE);
//         case PHASE_VOUT_0:      return ExitObserver(p_motor, &MOTOR_STATE_PASSIVE);
//         case PHASE_VOUT_PWM:    return NULL;
//         default:                return NULL;
//     }
// }
//
// /* An INTERVENTION sub-state, as RUN accepts it, coasts */
// static State_T * Run_InputIntervention(Motor_T * p_motor, state_value_t statePtr)
// {
//     State_T * p_state = (State_T *)statePtr;
//     if (p_state == NULL) { return NULL; }
//     if (p_state->P_TOP != &MOTOR_STATE_INTERVENTION) { return NULL; }
//     return ExitObserver(p_motor, &MOTOR_STATE_PASSIVE);
// }
//
// static const State_Input_T RUN_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
// {
//     [MOTOR_STATE_INPUT_FAULT]           = (State_Input_T)Run_InputFault,
//     [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = (State_Input_T)Run_InputFeedbackMode,
//     [MOTOR_STATE_INPUT_PHASE_OUTPUT]    = (State_Input_T)Run_InputControl,
//     [MOTOR_STATE_INPUT_CALIBRATION]     = NULL,
//     [MOTOR_STATE_INPUT_INTERVENTION]    = (State_Input_T)Run_InputIntervention,
//     [MOTOR_STATE_INPUT_OPEN_LOOP]       = NULL,
// };
//
// static const State_T SENSORLESS_RUN =
// {
//     .ID                 = MOTOR_STATE_ID_RUN,
//     .PATH_ID            = { .Depth0 = MOTOR_STATE_ID_RUN }, /* reports as RUN */
//     .P_TRANSITION_TABLE = &RUN_TRANSITION_TABLE[0U],
//     .P_ACTION_TABLE     = &MOTOR_RUN_ACTION_TABLE[0U],
//     .ENTRY              = (State_Action_T)_Motor_Run_Entry,
//     .LOOP               = (State_Action_T)Run_Proc,
//     .NEXT               = (State_Input0_T)Run_Next,
// };
