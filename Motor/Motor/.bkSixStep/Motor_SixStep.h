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
    @file   Motor_SixStep.h
    @author FireSourcery
    @brief  Six-Step Commutation. Polar PWM.
*/
/******************************************************************************/
#ifndef MOTOR_SIXSTEP_H
#define MOTOR_SIXSTEP_H

#include "Motor.h"
#include "Phase/Phase_Polar.h"

#if defined(MOTOR_SIX_STEP_ENABLE)

/******************************************************************************/
/*!
    Cycle Capture

    Peak magnitude of the phase waveform, double-buffered on the half-cycle
    boundary. Half-wave symmetry makes the half-cycle window sufficient, so the
    latch refreshes at 2x the electrical frequency.
*/
/******************************************************************************/
static inline void CaptureVPeak(Motor_Context_T * p_motor, uint16_t adcu)
{
    p_motor->VBemfPeakTemp_Adcu = math_max(p_motor->VBemfPeakTemp_Adcu, adcu);
}

static inline void CaptureIPeak(Motor_Context_T * p_motor, uint16_t zeroRef_Adcu, uint16_t adcu)
{
    p_motor->IPhasePeakTemp_Adcu = math_max(p_motor->IPhasePeakTemp_Adcu, math_abs((int16_t)adcu - (int16_t)zeroRef_Adcu));
}

/*
    Latch on half-cycle boundary. Caller holds the previous angle.
*/
static inline void Motor_SixStep_CaptureCycle(Motor_Context_T * p_motor, angle16_t anglePrev, angle16_t angle)
{
    if (angle16_cycle2(anglePrev, angle) == true)
    {
        p_motor->VBemfPeak_Adcu = p_motor->VBemfPeakTemp_Adcu;
        p_motor->IPhasePeak_Adcu = p_motor->IPhasePeakTemp_Adcu;
        p_motor->VBemfPeakTemp_Adcu = 0U;
        p_motor->IPhasePeakTemp_Adcu = 0U;
    }
}

/******************************************************************************/
/*!
    Commutation

    Sector derives from the [RotorSensor] angle, not from a per-sensor branch.
    [Hall_Id_T] commutation aliases and [Phase_Polar_T] share the 3-bit encoding,
    so the Hall path collapses to an identity; it is not a separate case.
*/
/******************************************************************************/
static inline Phase_Id_T Motor_SixStep_CommutationOf(angle16_t angle)
{
    return Phase_IdOfAngle(angle + ANGLE16_90);
}

/*
    Prior sector read before the write; the compare is not sequenced against it otherwise.
*/
static inline bool Motor_SixStep_PollCommutation(Motor_Context_T * p_motor, angle16_t angle)
{
    Phase_Id_T commutationPhase = Motor_SixStep_CommutationOf(angle);
    bool isCommutation = (commutationPhase != p_motor->CommutationPhase);
    p_motor->CommutationPhase = commutationPhase;
    return isCommutation;
}

/******************************************************************************/
/*!
    Phase Output

    Observe is shared with the passive path: PASSIVE must keep CommutationPhase
    current so RUN entry is bumpless. It is the six-step counterpart to
    [Motor_FOC_ProcCaptureAngleVBemf].
*/
/******************************************************************************/
static inline bool Motor_SixStep_ProcCapturePhaseVBemf(Motor_T * p_motor)
{
    return Motor_SixStep_PollCommutation(p_motor->P_MOTOR, Angle_Value(Motor_AngleSpeed(p_motor)));
}

/*
    Per PWM tick, Run state.
    Re-gate only on commutation; bipolar mode deactivates and inverts polarity there.
*/
static inline void Motor_SixStep_ProcPhaseControl(Motor_T * p_motor)
{
    bool isCommutation = Motor_SixStep_ProcCapturePhaseVBemf(p_motor); /* advances CommutationPhase */
    Phase_Polar_T commutationPhase = (Phase_Polar_T)p_motor->P_MOTOR->CommutationPhase;

    if (isCommutation == true) { Phase_Polar_ActivateOutput(&p_motor->PHASE, commutationPhase); }

    Phase_Polar_ActivateDuty(&p_motor->PHASE, commutationPhase, p_motor->P_MOTOR->VPwm);
}

#endif /* MOTOR_SIX_STEP_ENABLE */
#endif
