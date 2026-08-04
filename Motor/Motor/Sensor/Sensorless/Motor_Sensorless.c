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
#include "Sensorless_Sensor.h"

#include "../RotorSensor.h"
#include "../../StateMachine/Motor_StateMachine.h"
#include "../../StateMachine/Motor_OpenLoop.h"
#include "../../Motor_FOC.h"

#if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)

static const State_T SENSORLESS_ALIGN;
static const State_T SENSORLESS_START_UP;

static inline const Sensorless_Sensor_T * GetSensorless(Motor_T * p_motor) { return &p_motor->SENSOR_TABLE.SENSORLESS; }

/*
    Observer drive. Runs after the FOC proc so i_αβ is this tick's sample and
    v_αβ is this tick's commit — see Sensorless_Sensor_Proc for the ordering.
*/
static void ProcObserver(Motor_T * p_motor)
{
    Sensorless_Sensor_Proc(GetSensorless(p_motor), &p_motor->P_MOTOR->Foc);
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

/*
    Observer runs through align even though EMF is ~0 and lock cannot assert:
    it converges the SMO current estimate î onto the applied DC, so the ramp
    does not start from a step error.
*/
static void Align_Proc(Motor_T * p_motor)
{
    Motor_FOC_ProcStartUpAlign(p_motor);
    ProcObserver(p_motor);
}

static State_T * Align_Next(Motor_T * p_motor)
{
    return (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true) ? &SENSORLESS_START_UP : NULL;
}

static const State_T SENSORLESS_ALIGN =
{
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
    Sensorless_Sensor_SeedAngle(GetSensorless(p_motor), Angle_Value(&p_context->OpenLoopAngle), Angle_Delta(&p_context->OpenLoopAngle));
}

static void StartUp_Proc(Motor_T * p_motor)
{
    Motor_FOC_ProcOpenLoop(p_motor);
    ProcObserver(p_motor);
}
// FOC_Sensorless_ResetState(&p_motor->P_MOTOR->FocSensorless);
// FOC_Sensorless_SeedAngle(&p_motor->P_MOTOR->FocSensorless, 0, 0);
// FOC_Sensorless_SeedAngle(&p_context->FocSensorless, Angle_Value(&p_context->OpenLoopAngle), Angle_Delta(&p_context->OpenLoopAngle));

// fract16_t speed = Ramp_ProcNextOf(&p_context->OpenLoopSpeedRamp, (int32_t)p_context->Config.OpenLoopRampSpeedFinal_Fract16 * p_context->Direction);
// angle16_t angle = Angle_IntegrateSpeed_Fract16(&p_context->OpenLoopAngle, &p_context->OpenLoopSpeedRef, speed);
// fract16_t iq = Ramp_ProcNextOf(&p_context->OpenLoopIRamp, (int32_t)p_context->Config.OpenLoopRampIFinal_Fract16 * p_context->Direction);
// FOC_SetTheta(&p_context->Foc, angle);
// FOC_CaptureIabc(&p_context->Foc, &p_context->PhaseInput.I);
// FOC_ProcIFeedback(&p_context->Foc, VBus_Fract16(p_motor->P_VBUS), 0, iq);
// // FOC_Sensorless_Step(&p_context->Foc, &p_context->FocSensorless);
// FOC_ProcInvClarkePark(&p_context->Foc);
// // FOC_Sensorless_CaptureVoltage(&p_context->FocSensorless, p_context->Foc.Valpha, p_context->Foc.Vbeta);
// // capture for debugging

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
    if (p_motor->P_MOTOR->Direction == MOTOR_DIRECTION_NULL) { return; }
    if (p_motor->P_MOTOR->p_ActiveSensor != &GetSensorless(p_motor)->BASE) { return; }
    Motor_OpenLoop_EnterBranch(p_motor, &SENSORLESS_ALIGN);
}

#endif /* MOTOR_SENSOR_SENSORLESS_ENABLE */
