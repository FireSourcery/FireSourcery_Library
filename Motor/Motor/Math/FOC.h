#pragma once

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
    @file   FOC.h
    @author FireSourcery
    @brief  FOC math functions - state wrapper for pure math functions.
*/
/******************************************************************************/
#include "foc_math.h"
#include "svpwm_math.h"
#include "Math/Fixed/fract16.h"
#include "Math/PID/PID.h"

typedef struct FOC
{
    /* Theta - Save for Inverse Park */
    alignas(4) fract16_t Sine, Cosine;

    /* Feedback Variable */
    alignas(4) fract16_t Id, Iq;

    /* Control Variable. VOutput. Feedforward Input. */
    /* VBemf inputs during Freewheel */
    alignas(4) fract16_t Vd, Vq; /* VdReq, VqReq */

    /* VOut */
    alignas(4) fract16_t Va, Vb, Vc;

    /* Inner current loop — pair has no semantic life outside the d-q frame.
       Inputs (Iq/Id) and outputs (Vq/Vd) live in this same object. */
    PID_T PidIq;
    PID_T PidId;

    /* Inputs - Capture by ADC */
    // fract16_t Ia, Ib, Ic;

    /* Intermediate values */
    // fract16_t Ialpha, Ibeta;
    // fract16_t Valpha, Vbeta;

    // ufract16_t Modulation;
}
FOC_T;

/******************************************************************************/
/*
    Capture Inputs
*/
/******************************************************************************/
static inline void FOC_SetTheta(FOC_T * p_foc, angle16_t theta)
{
    struct fract16_xy vec = fract16_vector(theta);
    p_foc->Cosine = vec.x;
    p_foc->Sine = vec.y;
}

/*
    InputI
*/
static inline void FOC_ProcClarkePark(FOC_T * p_foc, fract16_t ia, fract16_t ib, fract16_t ic)
{
    struct foc_dq i = foc_clarke_park(ia, ib, ic, p_foc->Sine, p_foc->Cosine);
    p_foc->Id = i.d;
    p_foc->Iq = i.q;
}


// static inline void FOC_ProcClarkePark_AB(FOC_T * p_foc)
// {
//     foc_clarke_ab(&p_foc->Ialpha, &p_foc->Ibeta, p_foc->Ia, p_foc->Ib);
//     foc_park_vector(&p_foc->Id, &p_foc->Iq, p_foc->Ialpha, p_foc->Ibeta, p_foc->Sine, p_foc->Cosine);
// }

/*
    OutputV
*/

/* Feedback/Feedforward Control */
static inline void FOC_SetVd(FOC_T * p_foc, fract16_t vd) { p_foc->Vd = vd; }
static inline void FOC_SetVq(FOC_T * p_foc, fract16_t vq) { p_foc->Vq = vq; }


/* modulation [0:1/sqrt3] */
static inline bool _FOC_ProcVectorLimit(FOC_T * p_foc, ufract16_t vPhaseLimit)
{
    return foc_circle_limit_q(&p_foc->Vd, &p_foc->Vq, vPhaseLimit);
}

// assert(abs(p_foc->Vd) <= vPhaseLimit); /* set by feedback output */
static inline bool FOC_ProcVectorLimit(FOC_T * p_foc, ufract16_t vBus)
{
    return _FOC_ProcVectorLimit(p_foc, fract16_mul(vBus, FRACT16_1_DIV_SQRT3));
}

static inline ufract16_t _FOC_VqCircleLimit(const FOC_T * p_foc, ufract16_t vPhaseLimit)
{
    assert(abs(p_foc->Vd) <= vPhaseLimit); /* set by feedback output */
    return fixed_sqrt((uint32_t)vPhaseLimit * vPhaseLimit - (int32_t)p_foc->Vd * p_foc->Vd);
}

static inline ufract16_t FOC_VqCircleLimit(const FOC_T * p_foc, ufract16_t vBus)
{
    return _FOC_VqCircleLimit(p_foc, fract16_mul(vBus, FRACT16_1_DIV_SQRT3));
}

static inline ufract16_t _FOC_CircleLimit(FOC_T * p_foc, ufract16_t vPhaseLimit, fract16_t vd)
{
    p_foc->Vd = vd;
    return _FOC_VqCircleLimit(p_foc, vPhaseLimit);
}

// static inline void FOC_ProcInvClarkePark(FOC_T * p_foc, fract16_t vd, fract16_t vq)
static inline void FOC_ProcInvClarkePark(FOC_T * p_foc)
{
    struct foc_abc v = foc_inv_clarke_park(p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
    p_foc->Va = v.a;
    p_foc->Vb = v.b;
    p_foc->Vc = v.c;
}


/******************************************************************************/
/*
    VBemf
*/
/******************************************************************************/
static inline void FOC_ProcVBemfClarkePark(FOC_T * p_foc, fract16_t va, fract16_t vb, fract16_t vc)
{
    struct foc_dq v = foc_clarke_park(va, vb, vc, p_foc->Sine, p_foc->Cosine);
    p_foc->Vd = v.d;
    p_foc->Vq = v.q;
}

/******************************************************************************/
/*!
    Inner current loop — d-q PID pair operations.
    Co-located with the d-q vectors they read from / write to.
*/
/******************************************************************************/
/*
    Limit-first: compute Vq budget from voltage circle after Vd, set PidIq limits before proc
*/
static inline void FOC_ProcIFeedback(FOC_T * p_foc, ufract16_t vBus, sign_t direction, int16_t idReq, int16_t iqReq)
{
    p_foc->Vd = PID_ProcPI(&p_foc->PidId, p_foc->Id, idReq);
    int16_t vqLimit = _FOC_VqCircleLimit(p_foc, fract16_mul(vBus, FRACT16_1_DIV_2));
    interval_t band = interval_of_sign(direction, vqLimit); /* sign-keying  needs direction */
    PID_CaptureOutputLimits(&p_foc->PidIq, band.low, band.high);
    p_foc->Vq = PID_ProcPI(&p_foc->PidIq, p_foc->Iq, iqReq);
}

/* Post-proc circle limit — windup correction after both PIDs run. */
/* the combine output state can still grow outside of circle limit. limit after proc may still have windup. propagate if limited. */
static inline void FOC_ProcIFeedback_BackLimit(FOC_T * p_foc, ufract16_t vBus, int16_t idReq, int16_t iqReq)
{
    p_foc->Vd = PID_ProcPI(&p_foc->PidId, p_foc->Id, idReq);
    p_foc->Vq = PID_ProcPI(&p_foc->PidIq, p_foc->Iq, iqReq);
    if (FOC_ProcVectorLimit(p_foc, vBus)) { _PID_SetOutputState(&p_foc->PidIq, p_foc->Vq); }  // immediately snaps integral
    //   if (FOC_ProcVectorLimit(p_foc, vBus)) { if (math_abs(FOC_Vq(&p_motor->Foc)) > PID_GetIntegral(&p_motor->Foc.PidIq)) { PID_SetOutputState(&p_motor->Foc.PidIq, FOC_Vq(&p_motor->Foc)); }}
}

static inline void FOC_MatchIVState(FOC_T * p_foc, int16_t vd, int16_t vq)
{
    PID_SetOutputState(&p_foc->PidId, vd);
    PID_SetOutputState(&p_foc->PidIq, vq);
}

static inline void FOC_ResetFeedbackLoop(FOC_T * p_foc)
{
    PID_Reset(&p_foc->PidIq);
    PID_Reset(&p_foc->PidId);
}

static inline void FOC_SetVLimits(FOC_T * p_foc, sign_t direction, uint16_t vPhaseLimit)
{
    interval_t v = interval_of_sign(direction, vPhaseLimit);
    PID_SetOutputLimits(&p_foc->PidIq, v.low, v.high);
    PID_SetOutputLimits(&p_foc->PidId, 0 - vPhaseLimit, vPhaseLimit);
}


static inline void FOC_FeedforwardAngleV(FOC_T * p_foc, fract16_t theta, fract16_t vd, fract16_t vq)
{
    FOC_SetTheta(p_foc, theta);
    FOC_SetVd(p_foc, vd);
    FOC_SetVq(p_foc, vq);
    FOC_ProcInvClarkePark(p_foc);
}


/******************************************************************************/
/*!
    Query
*/
/******************************************************************************/
/*

*/
static inline fract16_t FOC_Id(const FOC_T * p_foc) { return p_foc->Id; }
static inline fract16_t FOC_Iq(const FOC_T * p_foc) { return p_foc->Iq; }
static inline fract16_t FOC_Vd(const FOC_T * p_foc) { return p_foc->Vd; }
static inline fract16_t FOC_Vq(const FOC_T * p_foc) { return p_foc->Vq; }

/* VBemf Common */
static inline fract16_t FOC_Va(const FOC_T * p_foc) { return p_foc->Va; }
static inline fract16_t FOC_Vb(const FOC_T * p_foc) { return p_foc->Vb; }
static inline fract16_t FOC_Vc(const FOC_T * p_foc) { return p_foc->Vc; }



/*
    Run-time derived
*/

/* [0:32767] <=> [0:1]AnalogRefMax */
static inline ufract16_t FOC_GetIMagnitude(const FOC_T * p_foc) { return fract16_vector_magnitude(p_foc->Id, p_foc->Iq); }
static inline ufract16_t FOC_GetVMagnitude(const FOC_T * p_foc) { return fract16_vector_magnitude(p_foc->Vd, p_foc->Vq); }

/* [-32768:32767] <=> [-1:1]AnalogRefMax */
static inline fract16_t FOC_GetIPhase(const FOC_T * p_foc) { return FOC_GetIMagnitude(p_foc) * math_sign(p_foc->Iq); }
static inline fract16_t FOC_GetVPhase(const FOC_T * p_foc) { return FOC_GetVMagnitude(p_foc) * math_sign(p_foc->Vq); }

/*
    Unit calculations
    Optionally handle by host
*/
static inline accum32_t FOC_GetTorquePower(const FOC_T * p_foc) { return fract16_mul(p_foc->Iq, p_foc->Vq) * 3 / 2; }
static inline accum32_t FOC_GetMagnetizingPower(const FOC_T * p_foc) { return fract16_mul(p_foc->Id, p_foc->Vd) * 3 / 2; }

static inline accum32_t FOC_GetReactivePower(const FOC_T * p_foc) { return (fract16_mul(p_foc->Vq, p_foc->Id) - fract16_mul(p_foc->Vd, p_foc->Iq)) * 3 / 2; }

/* [0:32767] */
static inline accum32_t _FOC_GetActivePower(const FOC_T * p_foc) { return fract16_mul(p_foc->Vd, p_foc->Id) + fract16_mul(p_foc->Vq, p_foc->Iq); }
// static inline accum32_t _FOC_GetActivePower(const FOC_T * p_foc) { return math_abs(fract16_mul(p_foc->Vd, p_foc->Id) + fract16_mul(p_foc->Vq, p_foc->Iq)); }
/* keep sign withing fract16 */
static inline accum32_t _FOC_GetIBus(const FOC_T * p_foc, ufract16_t vBus_fract16) { return (int32_t)fract16_div(_FOC_GetActivePower(p_foc), vBus_fract16); }

/* [0:49150] */
static inline accum32_t FOC_GetActivePower(const FOC_T * p_foc) { return _FOC_GetActivePower(p_foc) * 3 / 2; }

static inline accum32_t FOC_GetApparentPower(const FOC_T * p_foc) { return fract16_mul(FOC_GetIMagnitude(p_foc), FOC_GetVMagnitude(p_foc)) * 3 / 2; }

static inline accum32_t FOC_GetPowerFactor(const FOC_T * p_foc) { return fract16_div(_FOC_GetActivePower(p_foc), fract16_mul(FOC_GetIMagnitude(p_foc), FOC_GetVMagnitude(p_foc))); }

static inline accum32_t FOC_GetIBus(const FOC_T * p_foc, ufract16_t vBus_fract16) { return (int32_t)fract16_div(_FOC_GetActivePower(p_foc), vBus_fract16) * 3 / 2; }


/******************************************************************************/
/*!
    Quadrant Detection
    Sign convention: CCW/Forward = Positive Vq, Positive Iq
    Q1: Forward Motoring   - +Vq, +Iq (Vq*Iq > 0, Iq > 0)
    Q2: Forward Generating - +Vq, -Iq (Vq*Iq < 0, Iq > 0)
    Q3: Reverse Motoring   - -Vq, -Iq (Vq*Iq > 0, Iq < 0)
    Q4: Reverse Generating - -Vq, +Iq (Vq*Iq < 0, Iq < 0)

    Plugging: Voltage opposes back-EMF direction, active braking beyond regeneration
    Forward Plugging: -Vq with -Iq (motor spinning CCW, voltage reversed)
    Reverse Plugging: +Vq with +Iq (motor spinning CW, voltage reversed)

    Inner layer assume plugging is clamped. speed sign = v sign.
*/
/******************************************************************************/
// typedef enum FOC_Quadrant
// {
//     FOC_QUADRANT_0 = 0,    /* Zero / Idle */
//     FOC_QUADRANT_FORWARD_MOTORING   = 1,    /* Q1: +Vq, +Iq */
//     FOC_QUADRANT_FORWARD_GENERATING = 2,    /* Q2: +Vq, -Iq */
//     FOC_QUADRANT_REVERSE_MOTORING   = 3,    /* Q3: -Vq, -Iq */
//     FOC_QUADRANT_REVERSE_GENERATING = 4,    /* Q4: -Vq, +Iq */
// }
// FOC_Quadrant_T;

// static inline FOC_Quadrant_T FOC_GetQuadrant(const FOC_T * p_foc)
// {
//     if (p_foc->Iq == 0 && p_foc->Vq == 0) { return FOC_QUADRANT_0; }
//     if (p_foc->Iq > 0)  { return (p_foc->Vq >= 0) ? FOC_QUADRANT_FORWARD_MOTORING : FOC_QUADRANT_REVERSE_GENERATING; }
//     else                { return (p_foc->Vq <= 0) ? FOC_QUADRANT_REVERSE_MOTORING : FOC_QUADRANT_FORWARD_GENERATING; }
// }

static inline bool _FOC_IsMotoring(const FOC_T * p_foc) { return (p_foc->Vq * p_foc->Iq > 0); }
static inline bool _FOC_IsGenerating(const FOC_T * p_foc) { return (p_foc->Vq * p_foc->Iq < 0); }



/******************************************************************************/
/*!

*/
/******************************************************************************/

static void FOC_Init(FOC_T * p_foc)
{
    (void)p_foc;
}

// void FOC_SetModulation(FOC_T * p_foc, fract16_t modulation)
// {
//     p_foc->VModulation = fract16_mul(modulation, FRACT16_1_DIV_SQRT3);
// }

/* Prep Align using input intensity */
static void FOC_SetAlign(FOC_T * p_foc, fract16_t vd)
{
    p_foc->Vd = vd;
    p_foc->Vq = 0;
    p_foc->Sine = 0;
    p_foc->Cosine = FRACT16_MAX;
}


static void FOC_ClearCaptureState(FOC_T * p_foc)
{
    p_foc->Id = 0;
    p_foc->Iq = 0;
    // p_foc->Ialpha = 0; /* User view Phase values */
    // p_foc->Ibeta = 0;
}

static void FOC_ClearOutputState(FOC_T * p_foc)
{
    p_foc->Vd = 0;
    p_foc->Vq = 0;
    p_foc->Va = 0;
    p_foc->Vb = 0;
    p_foc->Vc = 0;
}



/******************************************************************************/
/*!
*/
/******************************************************************************/


/*
    FOC State
    Read-Only, RealTime
*/
typedef enum Motor_Var_Foc
{
    MOTOR_VAR_FOC_IA,
    MOTOR_VAR_FOC_IB,
    MOTOR_VAR_FOC_IC, /* move to phase input */
    MOTOR_VAR_FOC_ID,
    MOTOR_VAR_FOC_IQ,
    MOTOR_VAR_FOC_VD,
    MOTOR_VAR_FOC_VQ,
    MOTOR_VAR_FOC_VA,
    MOTOR_VAR_FOC_VB,
    MOTOR_VAR_FOC_VC,
    MOTOR_VAR_FOC_REQ_D,
    MOTOR_VAR_FOC_REQ_Q, /* Iq or Vq Req */
    // MOTOR_VAR_FOC_ID_REQ, /* return I or 0 */
    // MOTOR_VAR_FOC_IQ_REQ,
    MOTOR_VAR_FOC_INTEGRAL_D,
    MOTOR_VAR_FOC_INTEGRAL_Q,
}
Motor_Var_Foc_T;

static int _Motor_Var_Foc_Get(FOC_T * p_foc, Motor_Var_Foc_T varId)
{
    int value = 0;
    switch (varId)
    {
        // case MOTOR_VAR_FOC_IA:      value = p_state->PhaseInput.I.Values.A;   break;
        // case MOTOR_VAR_FOC_IB:      value = p_state->PhaseInput.I.Values.B;   break;
        // case MOTOR_VAR_FOC_IC:      value = p_state->PhaseInput.I.Values.C;   break;
        case MOTOR_VAR_FOC_ID:      value = p_foc->Id;                  break;
        case MOTOR_VAR_FOC_IQ:      value = p_foc->Iq;                  break;
        case MOTOR_VAR_FOC_VD:      value = p_foc->Vd;                  break;
        case MOTOR_VAR_FOC_VQ:      value = p_foc->Vq;                  break;
        case MOTOR_VAR_FOC_VA:      value = p_foc->Va;                  break;
        case MOTOR_VAR_FOC_VB:      value = p_foc->Vb;                  break;
        case MOTOR_VAR_FOC_VC:      value = p_foc->Vc;                  break;
        case MOTOR_VAR_FOC_INTEGRAL_D:    value = PID_GetIntegral(&p_foc->PidId);   break;
        case MOTOR_VAR_FOC_INTEGRAL_Q:    value = PID_GetIntegral(&p_foc->PidIq);   break;
    }
    return value;
}
// int _Motor_Var_Foc_Get(Motor_T * p_motor, Motor_Var_Foc_T varId);