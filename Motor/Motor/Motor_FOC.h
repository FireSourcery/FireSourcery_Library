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
    @brief  Motor FOC submodule. FOC control functions.
*/
/******************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "Math/FOC.h"
#include "Motor.h"
#include "Math/FOC_Ext.h"
#include "Phase/Phase_Svpwm.h"


/******************************************************************************/
/*
    +/- Sign indicates absolute virtual direction, CW/CCW. NOT along/against direction selected.
    A->B virtual CCW as Positive.
    Iq sign is rotation direction, NOT relative to Vq direction.

    CCW +Vq +Iq => Forward Motoring Q1
    CCW +Vq -Iq => Forward Regen Q2, VBemf > Vq > 0
    CCW -Vq -Iq => Forward Plugging Q2

    CW -Vq -Iq => Reverse Motoring Q3
    CW -Vq +Iq => Reverse Regen Q4, VBemf < Vq < 0
    CW +Vq +Iq => Reverse Plugging Q4
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
/* Supply current limits at this motor's operating point, [Cw:Ccw] */
#if defined(VBUS_I_LIMIT_POWER_BALANCE)

// static inline interval_t _Motor_VBusILimits(const VBus_T * p_vbus, const FOC_T * p_foc)
// {
//     return foc_iq_limits(_VBus_ILimits(p_vbus), VBus_Pu(p_vbus), foc_power_d(FOC_Vd(p_foc), FOC_Id(p_foc)), FOC_Vq(p_foc));
// }

static inline interval_t Motor_VBusILimits(Motor_T * p_motor) { return VBus_FOC_IqLimits(p_motor->P_VBUS, FOC_Vd(&p_motor->P_MOTOR->Foc), FOC_Id(&p_motor->P_MOTOR->Foc), FOC_Vq(&p_motor->P_MOTOR->Foc)); }
#elif defined(VBUS_I_LIMIT_PHASE_BOUND)
static inline interval_t Motor_VBusILimits(Motor_T * p_motor) { return VBus_FOC_IqPhaseLimits(p_motor->P_VBUS, math_sign(Motor_GetSpeedFeedback(p_motor->P_MOTOR)), FOC_Id(&p_motor->P_MOTOR->Foc)); }
#endif

/* Held for the re-resolves on Direction, FeedbackMode and state entry. Takes effect on the next resolve. */
static inline void Motor_CaptureISupplyLimits(Motor_T * p_motor) { p_motor->P_MOTOR->ISupplyLimits = Motor_VBusILimits(p_motor); }

static inline void Motor_ResolveISupplyLimits(Motor_T * p_motor)
{
    Motor_CaptureISupplyLimits(p_motor);
    Motor_ResolveILimits(p_motor->P_MOTOR);
    // interval_t iLimits = p_motor->P_MOTOR->ISupplyLimits;
    // Ramp_SetLimits(&p_motor->P_MOTOR->TorqueRamp, iLimits.low, iLimits.high);
    // PID_SetOutputLimits(&p_motor->P_MOTOR->PidSpeed, iLimits.low, iLimits.high);
}


/*
    Decoupling-basis ω input for FOC_CaptureSpeed. Δθ [angle16/Ts], the turn-base ω_pu of [FOC_Electrical_T]. Free of the speed base.
*/
static inline angle16_t Motor_GetDecouplingOmega(const Motor_Context_T * p_motor) { return RotorSensor_GetElectricalDelta(p_motor->p_ActiveSensor); }

// static inline void Motor_FOC_CaptureVSpeed(Motor_Context_T * p_motor) { FOC_CaptureSpeed(&p_motor->P_MOTOR->Foc, Motor_GetDecouplingOmega(p_motor)); }


/* Directly read register state */
/* state machine handles write gate state */
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

extern void Motor_FOC_ProcAngleV(Motor_T * p_motor);
extern void Motor_FOC_ProcAngleControl(Motor_T * p_motor);
extern void Motor_FOC_ProcCaptureAngleVBemf(Motor_Context_T * p_motor);

void Motor_FOC_SetDirection(Motor_T * p_dev, Motor_Direction_T direction);

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
