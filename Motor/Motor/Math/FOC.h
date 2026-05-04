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
    /* VBemf during Freewheel */
    alignas(4) fract16_t Vd, Vq; /* VdReq, VqReq */

    /* VBemf Inputs during Freewheel - Capture by ADC */
    /* Intermediate VOut */
    alignas(4) fract16_t Va, Vb, Vc;

    /* Scalar. optionally handle by phase */
    alignas(4) ufract16_t DutyA, DutyB, DutyC;

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
/*!
    Inner current loop — d-q PID pair operations.
    Co-located with the d-q vectors they read from / write to.
*/
/******************************************************************************/
static inline void FOC_MatchIVState(FOC_T * p_foc, int16_t vd, int16_t vq)
{
    PID_SetOutputState(&p_foc->PidId, vd);
    PID_SetOutputState(&p_foc->PidIq, vq);
}

static inline void FOC_ResetCurrentLoop(FOC_T * p_foc)
{
    PID_Reset(&p_foc->PidIq);
    PID_Reset(&p_foc->PidId);
}

// static void FOC_ProcIFeedback(FOC_T * p_foc, ufract16_t vBus, int16_t idReq, int16_t iqReq)
// {
//     p_foc->Vd = PID_ProcPI(&p_foc->PidId, p_foc->Id, idReq);
//     int16_t vqLimit = _FOC_VqCircleLimit(p_foc, fract16_mul(vBus, FRACT16_1_DIV_2));
//     interval_t band = interval_of_sign(math_sign(p_foc->Vq), vqLimit); /* sign-keying  needs direction */
//     PID_CaptureOutputLimits(&p_foc->PidIq, band.low, band.high);
//     p_foc->Vq = PID_ProcPI(&p_foc->PidIq, p_foc->Iq, iqReq);
// }


/******************************************************************************/
/*!
    VDuty Out
*/
/******************************************************************************/
/* precomputed vBusInv_fract32 <=> 1.0F/VBus */
static inline void FOC_ProcSvpwm(FOC_T * p_foc, uint32_t vBusInv_fract32)
{
    struct svpwm_abc duty = svpwm_midclamp_vbus(vBusInv_fract32, p_foc->Va, p_foc->Vb, p_foc->Vc);
    p_foc->DutyA = duty.a;
    p_foc->DutyB = duty.b;
    p_foc->DutyC = duty.c;
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
static inline ufract16_t FOC_Va(const FOC_T * p_foc) { return p_foc->Va; }
static inline ufract16_t FOC_Vb(const FOC_T * p_foc) { return p_foc->Vb; }
static inline ufract16_t FOC_Vc(const FOC_T * p_foc) { return p_foc->Vc; }

/* VDuty */
static inline ufract16_t FOC_DutyA(const FOC_T * p_foc) { return p_foc->DutyA; }
static inline ufract16_t FOC_DutyB(const FOC_T * p_foc) { return p_foc->DutyB; }
static inline ufract16_t FOC_DutyC(const FOC_T * p_foc) { return p_foc->DutyC; }


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
static inline accum32_t _FOC_GetActivePower(const FOC_T * p_foc) { return math_abs(fract16_mul(p_foc->Vd, p_foc->Id) + fract16_mul(p_foc->Vq, p_foc->Iq)); }

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

static inline bool FOC_IsMotoring(const FOC_T * p_foc) { return (p_foc->Vq * p_foc->Iq > 0); }
static inline bool FOC_IsGenerating(const FOC_T * p_foc) { return (p_foc->Vq * p_foc->Iq < 0); }



/******************************************************************************/
/*!

*/
/******************************************************************************/
extern void FOC_Init(FOC_T * p_foc);
extern void FOC_SetAlign(FOC_T * p_foc, fract16_t vd);
extern void FOC_ZeroSvpwm(FOC_T * p_foc);
extern void FOC_ClearCaptureState(FOC_T * p_foc);
extern void FOC_ClearOutputState(FOC_T * p_foc);
// extern void FOC_SetVectorMax(FOC_T * p_foc, fract16_t dMax);


/******************************************************************************/
/*!
*/
/******************************************************************************/


