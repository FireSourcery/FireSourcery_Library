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
    @file   Motor_FOC.h
    @author FireSourcery
    @brief  Motor FOC submodule. Bridge to StateMachine mapping
*/
/******************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "Motor.h"
#include "Math/FOC.h"
#include "Math/FOC_Ext.h"
#include "Phase/Phase_Svpwm.h"


/******************************************************************************/
/*
    +/- Sign indicates absolute virtual direction. NOT along/against direction selected.
    +/- <-> CCW/CW
    A->B virtual CCW as Positive.
    Iq sign is rotation direction, NOT relative to Vq direction.

    +ω +Vq +Iq => Forward Motoring Q1, Vq > VBemf
    +ω +Vq -Iq => Forward Regen Q2, VBemf > Vq > 0
    +ω -Vq -Iq => Forward Plugging Q2

    -ω -Vq -Iq => Reverse Motoring Q3, Vq < VBemf
    -ω -Vq +Iq => Reverse Regen Q4, VBemf < Vq < 0
    -ω +Vq +Iq => Reverse Plugging Q4
*/
/******************************************************************************/
static inline bool Motor_FOC_IsMotoring(const Motor_Context_T * p_motor) { return (FOC_Iq(&p_motor->Foc) * Motor_GetSpeedFeedback(p_motor) > 0); }
static inline bool Motor_FOC_IsGenerating(const Motor_Context_T * p_motor) { return (FOC_Iq(&p_motor->Foc) * Motor_GetSpeedFeedback(p_motor) < 0); }
static inline bool _Motor_FOC_IsRegen(const Motor_Context_T * p_motor) { return (FOC_Vq(&p_motor->Foc) * FOC_Iq(&p_motor->Foc) < 0); }

/*
    Plugging: applied Vq opposes back-EMF direction (speed sign)
    Speed sign gives true rotor direction regardless of Vq/Iq
*/
/* Vq and Speed have opposite signs - applied voltage opposes rotor back-EMF */
static inline bool Motor_FOC_IsPlugging(const Motor_Context_T * p_motor) { return (FOC_Vq(&p_motor->Foc) * Motor_GetSpeedFeedback(p_motor) < 0); }

/*
    Regen: Iq opposes Vq direction (generating), but Vq aligns with speed
*/
/* Generating  AND Vq aligns with speed - true regeneration */
static inline bool Motor_FOC_IsRegen(const Motor_Context_T * p_motor) { return (Motor_FOC_IsGenerating(p_motor) && !Motor_FOC_IsPlugging(p_motor)); }





/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    Decoupling-basis ω input for FOC_CaptureSpeed. Δθ [angle16/Ts], the turn-base ω_pu of [FOC_Electrical_T]. Free of the speed base.
*/
static inline angle16_t Motor_GetDecouplingOmega(const Motor_Context_T * p_motor) { return RotorSensor_GetElectricalDelta(p_motor->p_ActiveSensor); }

static inline void Motor_FOC_CaptureVSpeed(Motor_Context_T * p_motor) { FOC_CaptureSpeed(&p_motor->Foc, Motor_GetDecouplingOmega(p_motor)); }

/*
    The I set's flags are the token: one set runs the current loop once.
    The torque procs Park it at their take [Motor_FOC_TakeI], at the angle they set.
    A state that Parks it earlier, e.g. in its ON_PHASE action, at the angle [FOC_T] holds there, pairs the two below.
*/
/* Parks at the held angle. The flags stay set */
// static inline void Motor_FOC_CaptureI(Motor_Context_T * p_motor) { _FOC_CaptureIabc(&p_motor->Foc, &p_motor->PhaseInput.I.Values); }


/* Directly read register state */
/* state machine handles write gate state */
/* Duty register must read written value */
static inline void Motor_FOC_WriteDuty_Thread(Motor_T * p_dev)
{
    switch (Phase_ReadVOut(&p_dev->PHASE))
    {
        case PHASE_VOUT_Z: break;
        case PHASE_VOUT_0: break;
        case PHASE_VOUT_PWM:
            Phase_WriteSvpwm(&p_dev->PHASE, VBus_Inv_Pu(p_dev->P_VBUS), FOC_Va(&p_dev->P_MOTOR->Foc), FOC_Vb(&p_dev->P_MOTOR->Foc), FOC_Vc(&p_dev->P_MOTOR->Foc));
            break;
        default: break;
    }
}



/*
    Supply window [ISupplyLimits], from the operating point and VBus, by the build's [VBUS_I_LIMIT_*] law.
    Store only. Takes effect on the next [Motor_ResolveILimits].
*/
extern void Motor_FOC_ResolveISupplyLimits(Motor_T * p_motor);

/*
    The state running the current loop resolves the held inputs, on entry and on its speed tick.
    The supply window is resolved periodically on the selected thread, and on entry under the Direction the state runs with:
        MOTOR_I_SUPPLY_LIMITS_ON_SPEED  the speed tick, [Motor_FOC_ProcISupplyLimitsOnSpeed]
        MOTOR_I_SUPPLY_LIMITS_ON_VBUS   the VBus thread, 1 ms
        MOTOR_I_SUPPLY_LIMITS_ON_MAIN   the main thread, 16 ms
        none                            never resolved, the window stays open from [Motor_Init]
*/
static inline void Motor_FOC_ResolveISupplyLimitsOnEntry(Motor_T * p_motor)
{
#if defined(MOTOR_I_SUPPLY_LIMITS_ON_SPEED) || defined(MOTOR_I_SUPPLY_LIMITS_ON_VBUS) || defined(MOTOR_I_SUPPLY_LIMITS_ON_MAIN)
    Motor_FOC_ResolveISupplyLimits(p_motor);
#else
    (void)p_motor;
#endif
}

static inline void Motor_FOC_ProcISupplyLimitsOnSpeed(Motor_T * p_motor)
{
#if defined(MOTOR_I_SUPPLY_LIMITS_ON_SPEED)
    Motor_FOC_ResolveISupplyLimits(p_motor);
#else
    (void)p_motor;
#endif
}

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_FOC_WriteDuty(Motor_T * p_motor);

extern void Motor_FOC_ProcAngleFeedforwardV(Motor_Context_T * p_motor, angle16_t angle, fract16_t vd, fract16_t vq);
// extern void _Motor_FOC_ProcAngleAlign(Motor_Context_T * p_motor, fract16_t vBus, angle16_t angle, fract16_t idReq);
extern void Motor_FOC_ProcAngleAlign(Motor_T * p_motor, angle16_t angle, fract16_t idReq);
extern void Motor_FOC_ProcTorqueReq(Motor_T * p_motor, fract16_t qReq);
extern void Motor_FOC_ProcTorqueFeedback(Motor_T * p_motor);

extern void Motor_FOC_ProcAngleV(Motor_T * p_motor);
extern void Motor_FOC_ProcAngleControl(Motor_T * p_motor);
extern void Motor_FOC_ProcCaptureAngleVBemf(Motor_Context_T * p_motor);

extern void Motor_FOC_ClearFeedbackState(Motor_Context_T * p_motor);
extern void Motor_FOC_MatchTorqueVState(Motor_Context_T * p_state);
extern void Motor_FOC_MatchTorqueIState(Motor_Context_T * p_state);

void Motor_FOC_MatchVOutput(Motor_Context_T * p_context);
void Motor_FOC_MatchVFreewheel(Motor_Context_T * p_context, ufract16_t vbus);

extern void Motor_FOC_StartAlignCmd(Motor_Context_T * p_motor);
extern void Motor_FOC_ProcAlignCmd(Motor_T * p_motor);

extern void Motor_FOC_StartStartUpAlign(Motor_Context_T * p_motor);
extern void Motor_FOC_ProcStartUpAlign(Motor_T * p_motor);
extern void Motor_FOC_StartAlignValidate(Motor_Context_T * p_motor);

extern void Motor_FOC_StartOpenLoop(Motor_Context_T * p_motor);
extern void Motor_FOC_ProcOpenLoop(Motor_T * p_motor);

#ifdef MOTOR_EXTERN_CONTROL_ENABLE
extern void Motor_ExternControl(Motor_Context_T * p_motor);
#endif


#endif
