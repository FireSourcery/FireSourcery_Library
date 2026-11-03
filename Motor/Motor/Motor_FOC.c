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
    @file   Motor_FOC.c
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "Motor_FOC.h"
#include "Analog/Motor_Analog.h"


/*
    Out of line: one copy for every state that drives output.
*/
void Motor_FOC_WriteDuty(Motor_T * p_motor)
{
    Phase_WriteSvpwm(&p_motor->PHASE, VBus_Inv_Pu(p_motor->P_VBUS), FOC_Va(&p_motor->P_MOTOR->Foc), FOC_Vb(&p_motor->P_MOTOR->Foc), FOC_Vc(&p_motor->P_MOTOR->Foc));
}


/******************************************************************************/
/*!
    functions mapping to StateMachine handlers
*/
/******************************************************************************/

/*
    The sample the current loop runs on, once, Parked at the held angle.
    Out of line: every current-mode proc takes it, and inline each would carry its own Clarke and Park.
*/
// __attribute__((noinline)) static bool Motor_FOC_TakeI(Motor_Context_T * p_motor) { return FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I); }
// __attribute__((noinline)) static bool Motor_FOC_ClarkePark(Motor_Context_T * p_motor) { return _FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I); }


/* The torque path's i_d request as the LOOP last stepped it */
// static inline fract16_t Motor_FOC_IdTorqueReq(const Motor_Context_T * p_context)
// {
// #if defined(MOTOR_FOC_FIELD_WEAKENING_ENABLE)
//     return FOC_IdFw(&p_context->Foc);
// #else
//     (void)p_context; return 0;
// #endif
// }

/*
    The current loop at the I set's completion [MOTOR_I_FEEDBACK_ON_PHASE], on the requests the torque path holds.
    The LOOP steps them [_Motor_FOC_ProcTorqueReq] on the I Sample Cycle, the rate their slopes and gains are scaled to, and the state entries match them.
    The take Parks at the angle the LOOP last set, the sample's tick.
    ON_PHASE of the states whose LOOP runs the torque path. Without it, the LOOP takes the sample.
*/
// void Motor_FOC_ProcTorqueFeedback(Motor_T * p_motor)
// {
//     Motor_Context_T * p_context = p_motor->P_MOTOR;
//     if (Motor_FOC_TakeI(p_context) == true)
//     {
//         FOC_ProcIFeedback(&p_context->Foc, VBus_Pu(p_motor->P_VBUS), Motor_FOC_IdTorqueReq(p_context), Ramp_GetOutput(&p_context->TorqueRamp));
//         FOC_ProcInvClarkePark(&p_context->Foc);
//         Motor_FOC_WriteDuty(p_motor);
//     }
// }


/*
    Current-mode angle control. Req Id/Iq to Vabc.
    Out of line: one copy for every current-mode proc. -O3 otherwise inlines it into each align proc, 364 B each on KE.
*/
__attribute__((noinline)) void Motor_FOC_AngleControl(Motor_Context_T * p_motor, ufract16_t vBus, angle16_t theta, fract16_t dReq, fract16_t qReq)
{
    FOC_SetTheta(&p_motor->Foc, theta);
#if (MOTOR_CONTROL_FREQ != MOTOR_I_LOOP_FREQ) /* update angle only for voutput, until divider from adc is set */
    // if (Motor_FOC_TakeI(p_motor)) { FOC_ProcIFeedback(&p_motor->Foc, vBus, dReq, qReq); }
    if (FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I)) { FOC_ProcIFeedback(&p_motor->Foc, vBus, dReq, qReq); }
#else
    _FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I.Values);
    FOC_ProcIFeedback(&p_motor->Foc, vBus, dReq, qReq);
#endif
    FOC_ProcInvClarkePark(&p_motor->Foc);
}


/* Common state machine call for torque */
/*
    FW integrator advances on the V held from the previous cycle, then supplies th
    is
    cycle's idReq — one cycle of V→idReq latency, same as advancing it after AngleControl.

    The first tick after resume reads the VBemf capture rather than an applied V. That is
    only a one-tick perturbation because FOC_MatchIdFieldWeakening has already seeded IdFw
    from that same capture; without the seed this ordering ramps from 0 over the settling
    cycles instead.

    The requests step on the I Sample Cycle, at MOTOR_I_LOOP_FREQ, and hold on the others [Motor_IsAnalogCycle]. Under the hardware
    trigger the I Feedback Cycle is the next one, so nothing runs between its top and its take.
*/
void _Motor_FOC_ProcTorqueReq(Motor_Context_T * p_context, ufract16_t vbus, fract16_t req)
{
    bool step = _Motor_IsAnalogCycle(p_context->ControlTimerBase);
#if defined(MOTOR_FOC_FIELD_WEAKENING_ENABLE)
    fract16_t id = step ? FOC_ProcIdFieldWeakening(&p_context->Foc, vbus) : FOC_IdFw(&p_context->Foc);
#else
    fract16_t id = 0;
#endif
    fract16_t iq = step ? Ramp_ProcNextOf(&p_context->TorqueRamp, req) : Ramp_GetOutput(&p_context->TorqueRamp);

    Motor_FOC_AngleControl(p_context, vbus, Angle_Value(&p_context->SensorState.AngleSpeed), id, iq);
}

void Motor_FOC_ProcTorqueReq(Motor_T * p_motor, fract16_t req)
{
    _Motor_FOC_ProcTorqueReq(p_motor->P_MOTOR, VBus_Pu(p_motor->P_VBUS), req);
}

/* Common state machine call for align */
/* id limits update with FOC_DECOUPLE_ENABLE. id limits set at direction only when FOC_DECOUPLE disabled */
/* idReq input is userReq or preset IAlign. */
/* TorqueRamp steps on the I Sample Cycle, as in the torque path, whose slope it shares */
void _Motor_FOC_ProcAngleAlign(Motor_Context_T * p_motor, ufract16_t vBus, angle16_t angle, fract16_t idReq)
{
    fract16_t id = _Motor_IsAnalogCycle(p_motor->ControlTimerBase) ? _Ramp_ProcNextOf(&p_motor->TorqueRamp, math_clamp(idReq, 0, _Motor_GetIAlign(p_motor))) : Ramp_GetOutput(&p_motor->TorqueRamp);

    Motor_FOC_AngleControl(p_motor, vBus, angle, id, 0);
}

void Motor_FOC_ProcAngleAlign(Motor_T * p_motor, angle16_t angle, fract16_t idReq)
{
    _Motor_FOC_ProcAngleAlign(p_motor->P_MOTOR, VBus_Pu(p_motor->P_VBUS), angle, idReq);
}

/*
    Feed forward voltage angle
    set once, no feedback
    SetVAngle
*/
void Motor_FOC_ProcAngleFeedforwardV(Motor_Context_T * p_motor, angle16_t theta, fract16_t vd, fract16_t vq)
{
    FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I); /* Update for measurements */
    FOC_SetAngleV(&p_motor->Foc, theta, vd, vq);
}

/******************************************************************************/
/*!
    Run State
    Per-PWM-tick FOC pipelines.
    StateMachine picks one based on FeedbackMode.Current; no inner-loop dispatch.
*/
/******************************************************************************/
/*
    Current-mode pipeline. Sensor theta, TorqueRamp on Iq, Id from field weakening.
*/
void Motor_FOC_ProcAngleControl(Motor_T * p_motor)
{
    Motor_FOC_ProcTorqueReq(p_motor, p_motor->P_MOTOR->UserCmd);
}

/* on feedback mode update */
void Motor_FOC_MatchVOutput(Motor_Context_T * p_context)
{
    FOC_MatchIVState(&p_context->Foc);
}

/*
    Match Feedback State/Output to Output Voltage
    On FeedbackMode update and Freewheel to Run
    StateMachine blocks feedback proc
*/
/* From open terminals, (I≈0, ≤ base speed) */
/* Id, Iq cleared on freewheel entry */
/* vq == 0 from passive before bemf sample completes */
/* Vabc is either 0 from clear on entry to PASSIVE or set by CaptureAngleVBemf */
void Motor_FOC_MatchVFreewheel(Motor_Context_T * p_context, ufract16_t vbus)
{
    Motor_FOC_CaptureVSpeed(p_context);
#if defined(MOTOR_V_MATCH_SENSOR)
    FOC_MatchIVSensor(&p_context->Foc);
#elif defined(MOTOR_V_MATCH_SPEED)
    FOC_MatchIVSpeed(&p_context->Foc);
#endif
#if defined(MOTOR_FOC_FIELD_WEAKENING_ENABLE)
    FOC_MatchIdFieldWeakening(&p_context->Foc, vbus); /* pre-load from the matched V, before the integrator takes over on applied V */
#else
    (void)vbus;
#endif
}


void Motor_FOC_MatchTorqueIState(Motor_Context_T * p_context)
{
    Ramp_SetOutputState(&p_context->TorqueRamp, FOC_Iq(&p_context->Foc)); /* transitioning without release into freewheel, math iq */
    p_context->UserCmd = FOC_Iq(&p_context->Foc); /* may be ~1-50ms before next user input */
}


/*
    Voltage-mode on a separate path
*/
void Motor_FOC_ProcAngleV(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    fract16_t vReq = _Motor_IsAnalogCycle(p_context->ControlTimerBase) ? _Ramp_ProcNextOf(&p_context->TorqueRamp, p_context->UserCmd) : Ramp_GetOutput(&p_context->TorqueRamp); /* inner loop clips with VLimit. The I Sample Cycle, as the torque path */
    FOC_SetTheta(&p_context->Foc, Angle_Value(&p_context->SensorState.AngleSpeed));
    FOC_CaptureIabc(&p_context->Foc, &p_context->PhaseInput.I); /* Still capture I for overcurrent */
    FOC_ProcVControl(&p_context->Foc, fract16_mul(VBus_Pu(p_motor->P_VBUS), FRACT16_1_DIV_SQRT3), 0, vReq);
    FOC_ProcInvClarkePark(&p_context->Foc);
}


void Motor_FOC_MatchTorqueVState(Motor_Context_T * p_context)
{
    Ramp_SetOutputState(&p_context->TorqueRamp, FOC_Vq(&p_context->Foc)); //different units for now
    p_context->UserCmd = FOC_Vq(&p_context->Foc);
}


/******************************************************************************/
/*!
    Clear Feedback State
*/
/******************************************************************************/
/*
    also clears user view during vbemf, Ifeedback not updated
*/
void Motor_FOC_ClearFeedbackState(Motor_Context_T * p_motor)
{
    FOC_ClearCaptureState(&p_motor->Foc); /* Clear for view, updated again on enter control */
    FOC_ClearOutputState(&p_motor->Foc); /* Emergency stop, capture bemf subsitute */
    FOC_ResetFeedbackState(&p_motor->Foc);
    PID_Reset(&p_motor->PidSpeed);
    Phase_Input_ClearI(&p_motor->PhaseInput);
    Phase_Input_ClearV(&p_motor->PhaseInput);
    Ramp_SetOutputState(&p_motor->SpeedRamp, 0);
    Ramp_SetOutputState(&p_motor->TorqueRamp, 0);
    p_motor->UserCmd = 0;
}


/******************************************************************************/
/*
    Passive State
    ProcAngleObserve
    Angle Observe VBemf FreeWheel and Stop State
    Updates Vd, Vq
*/
/******************************************************************************/
void Motor_FOC_ProcCaptureAngleVBemf(Motor_Context_T * p_motor)
{
    FOC_SetTheta(&p_motor->Foc, Angle_Value(&p_motor->SensorState.AngleSpeed));
    FOC_CaptureVBemf(&p_motor->Foc, &p_motor->PhaseInput.V);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
void Motor_FOC_SetDirection(Motor_T * p_dev, Motor_Direction_T direction)
{
    Motor_SetDirection(p_dev, direction);
    // FOC_SetVAntiPlugging(p_foc, direction, vPhaseLimit);
    FOC_SetVLimits(&p_dev->P_MOTOR->Foc, (sign_t)direction, VBus_GetVPhaseRefSvpwm(p_dev->P_VBUS));
}

/*
    Supply current limits at this motor's operating point, [Cw:Ccw]. One law per [VBUS_I_LIMIT_*].
*/
static inline interval_t Motor_VBusILimits_PowerBalance(Motor_T * p_motor) { return VBus_FOC_IqLimits(p_motor->P_VBUS, FOC_Vd(&p_motor->P_MOTOR->Foc), FOC_Id(&p_motor->P_MOTOR->Foc), FOC_Vq(&p_motor->P_MOTOR->Foc)); }

static inline interval_t Motor_VBusILimits_PhaseBound(Motor_T * p_motor) { return VBus_FOC_IqPhaseLimits(p_motor->P_VBUS, math_sign(Motor_GetSpeedFeedback(p_motor->P_MOTOR)), FOC_Id(&p_motor->P_MOTOR->Foc)); }

/* [Motor_SetDirection] holds v_q to the Direction half-plane. Motoring draws, Generating returns */
static inline interval_t Motor_VBusILimits_VDerate(Motor_T * p_motor) { return VBus_FOC_IqDerateLimits(p_motor->P_VBUS, (sign_t)Motor_Direction(p_motor->P_MOTOR), p_motor->P_MOTOR->Config.ILimitMotoring_Pu, p_motor->P_MOTOR->Config.ILimitGenerating_Pu); }

/*
    Out of line: one copy for every entry, speed tick and periodic thread.
*/
void Motor_FOC_ResolveISupplyLimits(Motor_T * p_motor)
{
#if defined(VBUS_I_LIMIT_POWER_BALANCE)
    p_motor->P_MOTOR->ISupplyLimits = Motor_VBusILimits_PowerBalance(p_motor);
#elif defined(VBUS_I_LIMIT_PHASE_BOUND)
    p_motor->P_MOTOR->ISupplyLimits = Motor_VBusILimits_PhaseBound(p_motor);
#elif defined(VBUS_I_LIMIT_V_DERATE)
    p_motor->P_MOTOR->ISupplyLimits = Motor_VBusILimits_VDerate(p_motor);
#endif
}

/******************************************************************************/
/*!
    Open Loop
    StateMachine mapping
*/
/******************************************************************************/
/*
    Align using TorqueRamp.Target
*/
/* ElectricalAngle set by caller. Does not Angle_ZeroCaptureState */
void Motor_FOC_SetAlignCmdAngle(Motor_Context_T * p_motor, angle16_t angle)
{
    Angle_SetAngle(&p_motor->OpenLoopAngle, angle);
}

void Motor_FOC_StartAlignCmd(Motor_Context_T * p_motor)
{
    Motor_FOC_ClearFeedbackState(p_motor); /* reset TorqueRamp i/v to start at 0 */
}

void Motor_FOC_ProcAlignCmd(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    _Motor_FOC_ProcAngleAlign(p_context, VBus_Pu(p_motor->P_VBUS), Angle_Value(&p_context->OpenLoopAngle), p_context->UserCmd);
}


/******************************************************************************/
/*  */
/******************************************************************************/
/*
    Ramp towards Preset IAlign
    Caller sets time
*/
void Motor_FOC_StartStartUpAlign(Motor_Context_T * p_motor)
{
    Motor_FOC_ClearFeedbackState(p_motor); /* Resets Ramp */
    Angle_ZeroCaptureState(&p_motor->OpenLoopAngle); /* always align A */
}

void Motor_FOC_ProcStartUpAlign(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    _Motor_FOC_ProcAngleAlign(p_context, VBus_Pu(p_motor->P_VBUS), Angle_Value(&p_context->OpenLoopAngle), _Motor_GetIAlign(p_context));
}


/******************************************************************************/
/*  */
/******************************************************************************/
/*
    OpenLoop Spin - Feed forward input angle, enable/disable current feedback
    ElectricalAngle => integrate speed to angle
*/
void Motor_FOC_StartOpenLoop(Motor_Context_T * p_motor)
{
    // Ramp_SetOutputState(&p_motor->OpenLoopIRamp, _Motor_GetIAlign(p_motor) * p_motor->Direction);  /* continue from align current */
    Ramp_SetOutputState(&p_motor->OpenLoopIRamp, 0);
    Ramp_SetOutputState(&p_motor->OpenLoopSpeedRamp, 0);
    /* continue from OpenLoopAngle */
}

/*
    ElectricalAngle += (RPM * PolePairs * ANGLE16_PER_REV) / (60 * CONTROL_FREQ)
*/
void Motor_FOC_ProcOpenLoop(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    fract16_t speed = Ramp_ProcNextOf(&p_context->OpenLoopSpeedRamp, (int32_t)p_context->Config.OpenLoopRampSpeedFinal_Pu * Motor_Direction(p_context));
    angle16_t angle = Angle_IntegrateSpeed_Pu(&p_context->OpenLoopAngle, &p_context->SensorState.UnitRef.SpeedPuRef, speed); //todo change to delta
    fract16_t iq = Ramp_ProcNextOf(&p_context->OpenLoopIRamp, math_min(p_context->Config.OpenLoopRampIFinal_Pu, Motor_ILimitMotoring(p_context)) * Motor_Direction(p_context));
    Motor_FOC_AngleControl(p_context, VBus_Pu(p_motor->P_VBUS), angle, 0, iq);
}


// static inline void ProcClarkePark(Motor_Context_T * p_motor)
// {
// #if     defined(MOTOR_I_SENSORS_AB)
//     FOC_ProcClarkePark_AB(&p_motor->Foc);
// #elif   defined(MOTOR_I_SENSORS_ABC)
//     FOC_ProcClarkePark(&p_motor->Foc);
// #endif
// }



