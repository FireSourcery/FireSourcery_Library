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
    @file   FOC.h
    @author FireSourcery
    @brief  FOC math functions only. Wrapper for pure math functions.
            Only this modules calls math_foc, math_svpwm
    @version V0
*/
/******************************************************************************/
#ifndef FOC_H
#define FOC_H

#include "math_foc.h"
#include "math_svpwm.h"
#include "Math/Fixed/fract16.h"

#include "System/SysTime/SysTime.h"

typedef struct FOC
{
    /* Config */
    // fract16_t IdqMagnitudeMax;
    // fract16_t IdMax; /* default 1/sqrt3 */

    /*
        VDuty Outputs during AngleControl
        VBemf Inputs during Freewheel - Capture by ADC
    */
    ufract16_t Va;
    ufract16_t Vb;
    ufract16_t Vc;

    /* Inputs - Capture by ADC */
    fract16_t Ia;
    fract16_t Ib;
    fract16_t Ic;

    fract16_t Ialpha;
    fract16_t Ibeta;

    /* Theta - Save for Inverse Park */
    fract16_t Sine;
    fract16_t Cosine;

    /* Feedback */
    /* PID Feedback Variable */
    fract16_t Id;
    fract16_t Iq;

    /* Request in V or I  */
    /* PID Setpoint Variable - From Ramp, SpeedPid, OpenLoop */
    fract16_t ReqD;
    fract16_t ReqQ;

    /* VOutput/VBemf */
    /* PID Control Variable, or intermediate input bypass current feedback */
    fract16_t Vd;
    fract16_t Vq;

    fract16_t Valpha;
    fract16_t Vbeta;

    /* DutyA, DutyB, DutyC -> 16 bits, q0.15, always positive */
    // ufract16_t DutyA;
    // ufract16_t DutyB;
    // ufract16_t DutyC;
}
FOC_T;

static inline void FOC_ProcClarkePark(FOC_T * p_foc)
{
    foc_clarke(&p_foc->Ialpha, &p_foc->Ibeta, p_foc->Ia, p_foc->Ib, p_foc->Ic);
    foc_park_vector(&p_foc->Id, &p_foc->Iq, p_foc->Ialpha, p_foc->Ibeta, p_foc->Sine, p_foc->Cosine);
}

static inline void FOC_ProcClarkePark_AB(FOC_T * p_foc)
{
    foc_clarke_ab(&p_foc->Ialpha, &p_foc->Ibeta, p_foc->Ia, p_foc->Ib);
    foc_park_vector(&p_foc->Id, &p_foc->Iq, p_foc->Ialpha, p_foc->Ibeta, p_foc->Sine, p_foc->Cosine);
}

static inline void FOC_ProcInvParkInvClarkeSvpwm(FOC_T * p_foc)
{
    foc_circle_limit(&p_foc->Vd, &p_foc->Vq, FRACT16_MAX, FRACT16_1_DIV_SQRT3);
    foc_invpark_vector(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
    // svpwm_midclamp(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, p_foc->Valpha, p_foc->Vbeta);
    svpwm_midclamp(&p_foc->Va, &p_foc->Vb, &p_foc->Vc, p_foc->Valpha, p_foc->Vbeta);
}

/* VBemf */
static inline void FOC_ProcVBemfClarkePark(FOC_T * p_foc)
{
    foc_clarke(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Va, p_foc->Vb, p_foc->Vc);
    foc_park_vector(&p_foc->Vd, &p_foc->Vq, p_foc->Valpha, p_foc->Vbeta, p_foc->Sine, p_foc->Cosine);
}

/* [0:32767] <=> [0:1] */
static inline ufract16_t FOC_GetIMagnitude(const FOC_T * p_foc)     { return fract16_vector_magnitude(p_foc->Ialpha, p_foc->Ibeta); }
static inline ufract16_t FOC_GetVMagnitude(const FOC_T * p_foc)     { return fract16_vector_magnitude(p_foc->Valpha, p_foc->Vbeta); }
static inline ufract16_t FOC_GetIMagnitude_Idq(const FOC_T * p_foc) { return fract16_vector_magnitude(p_foc->Id, p_foc->Iq); }

/* [-32768:32767] <=> [-1:1] */
static inline fract16_t FOC_GetIPhase(const FOC_T * p_foc) { return FOC_GetIMagnitude(p_foc) * math_sign(p_foc->Iq); }
static inline fract16_t FOC_GetVPhase(const FOC_T * p_foc) { return FOC_GetVMagnitude(p_foc) * math_sign(p_foc->Vq); }

/* [0:49152] <=> [0:1.5] */
static inline accum32_t FOC_GetPower(const FOC_T * p_foc) { return (fract16_mul(FOC_GetIPhase(p_foc), FOC_GetVPhase(p_foc)) * 3 / 2); }

static inline void FOC_SetTheta(FOC_T * p_foc, angle16_t theta) { fract16_vector(&p_foc->Cosine, &p_foc->Sine, theta); }

static inline void FOC_SetIa(FOC_T * p_foc, fract16_t ia) { p_foc->Ia = ia; }
static inline void FOC_SetIb(FOC_T * p_foc, fract16_t ib) { p_foc->Ib = ib; }
static inline void FOC_SetIc(FOC_T * p_foc, fract16_t ic) { p_foc->Ic = ic; }
static inline void FOC_SetId(FOC_T * p_foc, fract16_t id) { p_foc->Id = id; }
static inline void FOC_SetIq(FOC_T * p_foc, fract16_t iq) { p_foc->Iq = iq; }
static inline void FOC_SetVd(FOC_T * p_foc, fract16_t vd) { p_foc->Vd = vd; }
static inline void FOC_SetVq(FOC_T * p_foc, fract16_t vq) { p_foc->Vq = vq; }

// static inline ufract16_t FOC_GetDutyA(const FOC_T * p_foc) { return p_foc->DutyA; }
// static inline ufract16_t FOC_GetDutyB(const FOC_T * p_foc) { return p_foc->DutyB; }
// static inline ufract16_t FOC_GetDutyC(const FOC_T * p_foc) { return p_foc->DutyC; }
static inline ufract16_t FOC_GetDutyA(const FOC_T * p_foc) { return p_foc->Va; }
static inline ufract16_t FOC_GetDutyB(const FOC_T * p_foc) { return p_foc->Vb; }
static inline ufract16_t FOC_GetDutyC(const FOC_T * p_foc) { return p_foc->Vc; }
static inline fract16_t FOC_GetId(const FOC_T * p_foc) { return p_foc->Id; }
static inline fract16_t FOC_GetIq(const FOC_T * p_foc) { return p_foc->Iq; }
static inline fract16_t FOC_GetVd(const FOC_T * p_foc) { return p_foc->Vd; }
static inline fract16_t FOC_GetVq(const FOC_T * p_foc) { return p_foc->Vq; }
static inline fract16_t FOC_GetIa(const FOC_T * p_foc) { return p_foc->Ia; }
static inline fract16_t FOC_GetIb(const FOC_T * p_foc) { return p_foc->Ib; }
static inline fract16_t FOC_GetIc(const FOC_T * p_foc) { return p_foc->Ic; }
static inline fract16_t FOC_GetIalpha(const FOC_T * p_foc) { return p_foc->Ialpha; }
static inline fract16_t FOC_GetIbeta(const FOC_T * p_foc) { return p_foc->Ibeta; }

/*  */
static inline void FOC_SetVBemfA(FOC_T * p_foc, ufract16_t va) { p_foc->Va = va; }
static inline void FOC_SetVBemfB(FOC_T * p_foc, ufract16_t vb) { p_foc->Vb = vb; }
static inline void FOC_SetVBemfC(FOC_T * p_foc, ufract16_t vc) { p_foc->Vc = vc; }
static inline ufract16_t FOC_GetVBemfA(const FOC_T * p_foc) { return p_foc->Va; }
static inline ufract16_t FOC_GetVBemfB(const FOC_T * p_foc) { return p_foc->Vb; }
static inline ufract16_t FOC_GetVBemfC(const FOC_T * p_foc) { return p_foc->Vc; }

// or change duty to Vabc
// static inline void FOC_MatchDuty(FOC_T * p_foc)
// {
//     p_foc->DutyA = p_foc->Va;
//     p_foc->DutyB = p_foc->Vb;
//     p_foc->DutyC = p_foc->Vc;
// }

static inline void FOC_SetReqD(FOC_T * p_foc, fract16_t d) { p_foc->ReqD = d; }
static inline void FOC_SetReqQ(FOC_T * p_foc, fract16_t q) { p_foc->ReqQ = q; }
static inline fract16_t FOC_GetReqD(const FOC_T * p_foc) { return p_foc->ReqD; }
static inline fract16_t FOC_GetReqQ(const FOC_T * p_foc) { return p_foc->ReqQ; }

extern void FOC_Init(FOC_T * p_foc);
extern void FOC_SetAlign(FOC_T * p_foc, fract16_t vd);
extern void FOC_ZeroSvpwm(FOC_T * p_foc);
extern void FOC_ClearControlState(FOC_T * p_foc);
extern void FOC_ClearObserveState(FOC_T * p_foc);
// extern void FOC_SetVectorMax(FOC_T * p_foc, fract16_t dMax);

#endif
