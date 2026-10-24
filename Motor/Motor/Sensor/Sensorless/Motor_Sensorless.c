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

/* Keep it compiling. */
#if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
static inline const Sensorless_Sensor_T * GetSensorless(Motor_T * p_motor) { return &p_motor->SENSOR_TABLE.SENSORLESS; }
#else
static inline const Sensorless_Sensor_T * GetSensorless(Motor_T * p_motor) { return NULL; }
#endif



// static inline void Motor_FOC_IOutput(Motor_Context_T * p_motor, ufract16_t vBus, angle16_t theta, fract16_t dReq, fract16_t qReq)
// {
//     FOC_SetTheta(&p_motor->Foc, theta);
//     _FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I.Values);
//     FOC_ProcIFeedback(&p_motor->Foc, vBus, dReq, qReq);
//     FOC_ProcInvClarkePark(&p_motor->Foc);
// }

// static inline void Sensorless_Run(Motor_T * p_dev)
// {
//     Motor_Context_T * p_motor = (Motor_Context_T *)p_dev;
//     FOC_T * p_foc = &p_motor->Foc;
//     fract16_t vBus = VBus_GetVPhaseRefSvpwm(p_dev->P_VBUS);

//     _FOC_CaptureIabc(p_foc, &p_motor->PhaseInput.I.Values);     //  FOC_ProcClarkePark(p_foc, p_iabc->A, p_iabc->B, p_iabc->C);
//     angle16_t theta = EmfObserver_Eemf_Proc(GetSensorless(p_dev)->P_OBSERVER, p_foc->Id, p_foc->Iq, p_foc->Vd, p_foc->Vq);

//     FOC_SetTheta(p_foc, theta);
//     FOC_ProcIFeedback(p_foc, vBus, 0, Ramp_ProcNextOf(&p_motor->TorqueRamp, p_motor->UserCmd));
//     FOC_ProcInvClarkePark(p_foc);
// }

// static void Run_OnPhaseI(Motor_T * p_motor)
// {
//     Motor_Context_T * p_context = p_motor->P_MOTOR;
//     // FOC_ProcInput(&p_context->Foc);
//     // FOC_SetTheta(&p_context->Foc, Angle_Value(&p_context->SensorState.AngleSpeed));
//     // _FOC_CaptureIabc(&p_context->Foc, &p_context->PhaseInput.I.Values);
//     FOC_ProcClarkePark(&p_context->Foc, p_context->PhaseInput.I.A, p_context->PhaseInput.I.B, p_context->PhaseInput.I.C);
// }

// static const State_Action_T RUN_ACTION_TABLE[MOTOR_STATE_ACTION_TABLE_LENGTH] =
// {
//     // [MOTOR_STATE_INPUT_ON_SPEED] = (State_Action_T)Run_OnSpeed,
//     [MOTOR_STATE_INPUT_ON_PHASE] = (State_Action_T)Run_OnPhaseI,
// };

// static const State_T SENSORLESS_RUN =
// {
//     .PATH_ID    = { .Depth0 = MOTOR_STATE_ID_RUN }, /* sub-state id not yet allocated: reports as the root state */

//     .P_TOP    = &MOTOR_STATE_RUN,
//     .P_PARENT = &MOTOR_STATE_RUN,
//     .DEPTH    = 1U,
//     // .ENTRY    = (State_Action_T)Run_Entry,
//     // .LOOP     = (State_Action_T)Run_Proc,
//     .P_ACTION_TABLE     = &RUN_ACTION_TABLE[0U],
// };



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
    Sensorless_Sensor_Proc(GetSensorless(p_motor), &p_motor->P_MOTOR->Foc);
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
static void StartUp_Entry(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    Motor_FOC_StartOpenLoop(p_context);
    /* Seed the PLL at the commanded angle so it converges from the ramp rather than from 0. */
    EmfObserver_SeedAngle(GetSensorless(p_motor)->P_OBSERVER, Angle_Value(&p_context->OpenLoopAngle), Angle_Delta(&p_context->OpenLoopAngle));
}

static void StartUp_Proc(Motor_T * p_motor)
{
    Motor_FOC_ProcOpenLoop(p_motor);
    Motor_FOC_WriteDuty(p_motor);
    Sensorless_Sensor_Proc(GetSensorless(p_motor), &p_motor->P_MOTOR->Foc);
}


/*
    Hand off to closed loop. θ̂ is already published into RotorSensor_State by
    CAPTURE_ANGLE each tick, so MOTOR_STATE_RUN reads a live angle on entry;
    lock (strong |ê| + tight PLL error, sustained) is what bounds the step
    between the commanded ramp angle and θ̂.
*/
static State_T * StartUp_Next(Motor_T * p_motor)
{
    if (RotorSensor_IsFeedbackAvailable(&GetSensorless(p_motor)->BASE) == false) { return NULL; }
    p_motor->P_MOTOR->FeedbackMode.OpenLoop = 0U;
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
    Motor_OpenLoop_EnterBranch(p_motor, &SENSORLESS_ALIGN);
}

/* The chain that acquires the angle from standstill, for an entry from outside: a PWM request in PASSIVE, or a cmd */
State_T * Motor_Sensorless_StartUpState(Motor_T * p_motor) { (void)p_motor; return &SENSORLESS_ALIGN; }


/******************************************************************************/
/*
    The observer before the FOC proc.
    Called on the PWM thread before the state machine, on the phase-sample event, or as a state's ON_PHASE action.
*/
/******************************************************************************/
/*
    One control tick, before the FOC proc: the sample closes the period, with the v held across it.
    The phase output across the period decides that v. The analog thread samples I while driven, V while floating [_Motor_Analog_Thread].
    Valid only while the sensorless adapter is the selected sensor.
*/
void Motor_Sensorless_CaptureStator(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    const volatile Phase_Triplet_T * p_iabc = &p_context->PhaseInput.I.Values;
    const volatile Phase_Triplet_T * p_vabc = &p_context->PhaseInput.V.Values;
    struct foc_alphabeta i = { 0 };
    struct foc_alphabeta v = { 0 };

    switch (Phase_ReadVOut(&p_motor->PHASE))
    {
        case PHASE_VOUT_PWM:    /* the last command */
            i = foc_clarke(p_iabc->A, p_iabc->B, p_iabc->C);
            v = (struct foc_alphabeta){ .alpha = p_context->Foc.Valpha, .beta = p_context->Foc.Vbeta };
            break;
        case PHASE_VOUT_0:      /* shorted: the EMF drives i through the winding */
            i = foc_clarke(p_iabc->A, p_iabc->B, p_iabc->C);
            break;
        case PHASE_VOUT_Z:      /* floating: no current, the terminals read the EMF */
            v = foc_clarke(p_vabc->A, p_vabc->B, p_vabc->C);
            break;
        default: break;
    }

    Sensorless_Sensor_ProcEemf(GetSensorless(p_motor), fract16_sat(i.alpha), fract16_sat(i.beta), fract16_sat(v.alpha), fract16_sat(v.beta));
    // EmfObserver_AlphaBeta_ProcHeld(GetSensorless(p_motor)->P_OBSERVER, fract16_sat(i.alpha), fract16_sat(i.beta), fract16_sat(v.alpha), fract16_sat(v.beta));
}

/*
    The observer's plant, from the model on the same view as FOC's [Motor_FOC_InitParams]: Lq on both axes, the extended EMF.
    Re-resolve with it, on a change of the model. An invalid model keeps the prior winding, as it keeps FOC's view.
*/
void Motor_Sensorless_InitWinding(Motor_T * p_motor)
{
    const Motor_Electrical_T * p_electrical = &p_motor->P_MOTOR->Config.Electrical;
    if (Motor_Electrical_IsValid(p_electrical) == true) { EmfObserver_InitWinding(GetSensorless(p_motor)->P_OBSERVER, p_electrical->Rs, Motor_Lq_PuTurn(p_electrical)); }
}

// #endif /* MOTOR_SENSOR_SENSORLESS_ENABLE */
