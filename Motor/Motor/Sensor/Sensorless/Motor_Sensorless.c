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
    @brief  The observer's states: the start-up chain under MOTOR_STATE_OPEN_LOOP, and the alternate RUN.

        SENSORLESS_ALIGN     — DC align, observer runs to settle î against the
                               real current. Timer expiry → SENSORLESS_START_UP.
        SENSORLESS_START_UP  — open-loop speed/current ramp, observer tracks the
                               rising EMF. Observer lock → SENSORLESS_RUN.
        SENSORLESS_RUN       — RUN's torque path at θ̂, the correction on its take.
                               Every exit resets the observer.

    Each state corrects the observer in its own LOOP, after its write. No other state corrects it:
    CAPTURE_ANGLE still predicts every control tick, on the ω̂ the last correction left.

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

static const State_T SENSORLESS_ALIGN;
static const State_T SENSORLESS_START_UP;
static const State_T SENSORLESS_RUN;

/* Keep it compiling. */
#if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
static inline const Sensorless_Sensor_T * GetSensorless(Motor_T * p_motor) { return &p_motor->SENSOR_TABLE.SENSORLESS; }
#else
static inline const Sensorless_Sensor_T * GetSensorless(Motor_T * p_motor) { return NULL; }
#endif


/******************************************************************************/
/*
    The observer's correction, after the write.
    CAPTURE_ANGLE predicts θ̂ each control tick; the correction sets ω̂ once per sample, for the next prediction.
    Off the write's deadline: the FOC already Parked the sample at the θ̂ this tick published, the frame the correction Parks it in.
    The set in PhaseInput landed before this cycle's top, I while driven [Motor_ADC_MarkPhases]: the states that correct drive PWM.
*/
/******************************************************************************/
/*
    i(k) in the frame θ̂, after the write. v(k−1) from the hold, then holds v(k): the last write before the next sample.
    Out of line: one copy of the law for every state that corrects. -O3 otherwise inlines it into each LOOP.
*/
static __attribute__((noinline)) void CorrectDq(Motor_T * p_motor, fract16_t i_d, fract16_t i_q)
{
    const FOC_T * p_foc = &p_motor->P_MOTOR->Foc;
    EmfObserver_T * p_obs = GetSensorless(p_motor)->P_OBSERVER;

    EmfObserver_Eemf_ProcHeld(p_obs, i_d, i_q);
    EmfObserver_HoldV(p_obs, p_foc->Valpha, p_foc->Vbeta);
}

/*
    The correction on the FOC's Clarke, for a state whose current loop runs at another angle: the chain's, at the ramp.
    i(k) the take's Iα, Iβ, Parked here at θ̂.
*/
static void CorrectOnClarke(Motor_T * p_motor)
{
    const FOC_T * p_foc = &p_motor->P_MOTOR->Foc;
    struct foc_dq i = EmfObserver_ParkI(GetSensorless(p_motor)->P_OBSERVER, p_foc->Ialpha, p_foc->Ibeta);
    CorrectDq(p_motor, i.d, i.q);
}

/*
    The correction on the FOC's take, for a state whose current loop runs at θ̂: i(k) is the take's Id/Iq [Motor_FOC_CaptureI].
    Both forms read i(k) and v(k−1), the same ω̂ for the next prediction:
        A   after the write, [SENSORLESS_RUN]: the LOOP is the other sensors' torque path, the correction holds v(k) for the next sample
        B   between the take and the loop, not wired: Foc still holds v(k−1), no hold; the write later by the correction's cost
*/
/*
    A: after the LOOP's write. Foc keeps the take through the loop: i(k) at θ̂(k).
*/
void Motor_Sensorless_CorrectOnTake(Motor_T * p_motor)
{
    CorrectDq(p_motor, FOC_Id(&p_motor->P_MOTOR->Foc), FOC_Iq(&p_motor->P_MOTOR->Foc));
}

/*
    B: the torque path [_Motor_FOC_ProcTorqueReq] with the correction between the take and the loop. The caller writes.
    i(k) at θ̂(k) from the take, v(k−1) in Valpha/Vbeta, the loop not yet run.
    Inlines a second copy of the loop beside [Motor_FOC_AngleControl]'s.
*/
void Motor_Sensorless_ProcTorqueReq(Motor_T * p_motor, fract16_t req)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    const FOC_T * p_foc = &p_context->Foc;
    ufract16_t vBus = VBus_Pu(p_motor->P_VBUS);
    fract16_t id = _Motor_FOC_ProcIdReq(p_context, vBus);
    fract16_t iq = Ramp_ProcNextOf(&p_context->TorqueRamp, req);

    Motor_FOC_CaptureI(p_context, Angle_Value(&p_context->SensorState.AngleSpeed));
    EmfObserver_Eemf_ProcVAlphaBeta(GetSensorless(p_motor)->P_OBSERVER, FOC_Id(p_foc), FOC_Iq(p_foc), p_foc->Valpha, p_foc->Vbeta);
    Motor_FOC_ProcIControl(p_context, vBus, id, iq);
}

/*
    Leaving the observer's states: θ̂, ω̂ and the lock cleared [ZERO_INITIAL], so no state that does not correct reads a lock it no longer holds.
    Passes the transition through, the reset only on one.
*/
static State_T * ExitObserver(Motor_T * p_motor, State_T * p_next)
{
    if (p_next != NULL) { RotorSensor_ZeroInitial(&GetSensorless(p_motor)->BASE); }
    return p_next;
}


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
    does not start from a step error.
*/
static void Align_Proc(Motor_T * p_motor)
{
    Motor_FOC_ProcStartUpAlign(p_motor);
    Motor_FOC_WriteDuty(p_motor);
    CorrectOnClarke(p_motor);
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
    After the write, before the correction Parks i at θ̂: the angle the FOC Parked it at this tick.
*/
static void StartUp_Proc(Motor_T * p_motor)
{
    Motor_FOC_ProcOpenLoop(p_motor);
    Motor_FOC_WriteDuty(p_motor);
    if (EmfObserver_IsIndeterminate(GetSensorless(p_motor)->P_OBSERVER) == true) { StartUp_SeedRamp(p_motor); }
    CorrectOnClarke(p_motor);
}

/*
    Hand off to closed loop. θ̂ is already published into RotorSensor_State by
    CAPTURE_ANGLE each tick, so SENSORLESS_RUN reads a live angle on entry;
    lock (strong |ê| + tight PLL error, sustained) is what bounds the step
    between the commanded ramp angle and θ̂. The lock is this tick's: the LOOP corrected before.
    The FOC's operating point re-Parked from the ramp's angle to θ̂, which RUN's entry seeds the loop and the ramp from.
*/
static State_T * StartUp_Next(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    if (RotorSensor_IsFeedbackAvailable(&GetSensorless(p_motor)->BASE) == false) { return NULL; }
    FOC_MatchTheta(&p_context->Foc, Angle_Value(&p_context->SensorState.AngleSpeed));
    p_context->FeedbackMode.OpenLoop = 0U;
    return &SENSORLESS_RUN;
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
    Run — an alternate MOTOR_STATE_RUN, top level, entered from START_UP's lock.
    RUN's entry, torque path and action table: ON_SPEED maps as RUN's. Its LOOP then corrects on the take, form A.
    No sensorless intervention state yet: release and intervention requests coast in PASSIVE.
    Every exit resets the observer [ExitObserver]: no other state corrects it.
*/
/******************************************************************************/
static void Run_Proc(Motor_T * p_motor)
{
    _Motor_Run_Proc(p_motor);
    Motor_Sensorless_CorrectOnTake(p_motor);
}

/* The lock is this tick's: the LOOP corrected before */
static State_T * Run_Next(Motor_T * p_motor)
{
    return ExitObserver(p_motor, _Motor_TransitionFeedbackLost(p_motor));
}

static State_T * Run_InputFault(Motor_T * p_motor, state_value_t faultCmd)
{
    return ExitObserver(p_motor, _Motor_TransitionFault(p_motor, faultCmd));
}

/* RUN's entry matches the new mode */
static State_T * Run_InputFeedbackMode(Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode(p_motor, Motor_FeedbackMode_Cast(feedbackMode));
    return &SENSORLESS_RUN;
}

static State_T * Run_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:      return ExitObserver(p_motor, &MOTOR_STATE_PASSIVE);
        case PHASE_VOUT_0:      return ExitObserver(p_motor, &MOTOR_STATE_PASSIVE);
        case PHASE_VOUT_PWM:    return NULL;
        default:                return NULL;
    }
}

/* An INTERVENTION sub-state, as RUN accepts it, coasts */
static State_T * Run_InputIntervention(Motor_T * p_motor, state_value_t statePtr)
{
    State_T * p_state = (State_T *)statePtr;
    if (p_state == NULL) { return NULL; }
    if (p_state->P_TOP != &MOTOR_STATE_INTERVENTION) { return NULL; }
    return ExitObserver(p_motor, &MOTOR_STATE_PASSIVE);
}

static const State_Input_T RUN_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FAULT]           = (State_Input_T)Run_InputFault,
    [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = (State_Input_T)Run_InputFeedbackMode,
    [MOTOR_STATE_INPUT_PHASE_OUTPUT]    = (State_Input_T)Run_InputControl,
    [MOTOR_STATE_INPUT_CALIBRATION]     = NULL,
    [MOTOR_STATE_INPUT_INTERVENTION]    = (State_Input_T)Run_InputIntervention,
    [MOTOR_STATE_INPUT_OPEN_LOOP]       = NULL,
};

static const State_T SENSORLESS_RUN =
{
    .ID                 = MOTOR_STATE_ID_RUN,
    .PATH_ID            = { .Depth0 = MOTOR_STATE_ID_RUN }, /* reports as RUN */
    .P_TRANSITION_TABLE = &RUN_TRANSITION_TABLE[0U],
    .P_ACTION_TABLE     = &MOTOR_RUN_ACTION_TABLE[0U],
    .ENTRY              = (State_Action_T)_Motor_Run_Entry,
    .LOOP               = (State_Action_T)Run_Proc,
    .NEXT               = (State_Input0_T)Run_Next,
};


/******************************************************************************/
/*
    Entry points
*/
/******************************************************************************/
/*
    Start the sensorless align → ramp → closed-loop chain.
    Valid only while the sensorless adapter is the selected sensor — the chain
    hands to SENSORLESS_RUN on the observer's lock, and RUN reads its angle
    from p_ActiveSensor.
*/
void Motor_Sensorless_StartRunChain(Motor_T * p_motor)
{
    if (Motor_Direction(p_motor->P_MOTOR) == MOTOR_DIRECTION_NULL) { return; }
    if (p_motor->P_MOTOR->p_ActiveSensor != &GetSensorless(p_motor)->BASE) { return; }
    Motor_OpenLoop_EnterBranch(p_motor, &SENSORLESS_ALIGN);
}

/*
    The chain that acquires the angle from standstill, for an entry from outside: a PWM request in PASSIVE, or a cmd.
    NULL unless the sensorless adapter is the active sensor: CAPTURE_ANGLE predicts, and publishes the angle RUN reads, for the active sensor only.
    Config.SensorMode can name it before a reboot makes it active.
*/
State_T * Motor_Sensorless_StartUpState(Motor_T * p_motor)
{
    return (p_motor->P_MOTOR->p_ActiveSensor == &GetSensorless(p_motor)->BASE) ? &SENSORLESS_ALIGN : NULL;
}

// State_T * Motor_Sensorless_StartUpState(Motor_T * p_motor) { (void)p_motor; return &SENSORLESS_ALIGN; }


/******************************************************************************/
/*
    The observer before the FOC proc.
    Called on the PWM thread before the state machine, on the phase-sample event, or as a state's ON_PHASE action.
    Superseded by the correction after the write, in the states' LOOPs: before the LOOP, its cost lands ahead of the write.
*/
/******************************************************************************/
/*
    One control tick, before the FOC proc: the sample closes the period, with the v held across it.
    The phase output across the period decides that v. The analog thread samples I while driven, V while floating [Motor_ADC_MarkPhases].
    Valid only while the sensorless adapter is the selected sensor.
*/
// void Motor_Sensorless_CaptureStator(Motor_T * p_motor)
// {

// }
