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
#include "Math/Q/QFrac16.h"

typedef struct FOC
{
    /* Params */
    // qfrac16_t IdqMagnitudeMax;
    // qfrac16_t IdMax; /* default 1/sqrt3 */

    /* VBemf Inputs during Freewheel - Capture by ADC */
    qfrac16_t Va;
    qfrac16_t Vb;
    qfrac16_t Vc;

    /* Inputs - Capture by ADC */
    qfrac16_t Ia;
    qfrac16_t Ib;
    qfrac16_t Ic;

    qfrac16_t Ialpha;
    qfrac16_t Ibeta;

    /* Theta - Save for Inverse Park */
    qfrac16_t Sine;
    qfrac16_t Cosine;

    /* Feedback */
    /* PID Feedback Variable */
    qfrac16_t Id;
    qfrac16_t Iq;

    /* Request in V or I  */
    /* PID Setpoint Variable - From Ramp, SpeedPid, OpenLoop */
    qfrac16_t DReq;
    qfrac16_t QReq;

    /* VOutput/VBemf */
    /* PID Control Variable, or intermediate input bypass current feedback */
    qfrac16_t Vd;
    qfrac16_t Vq;

    uint16_t ReqMagnitude;   /* UFrac16, Q1.15 Unsigned, vector magnitude prior to limit */

    qfrac16_t Valpha;
    qfrac16_t Vbeta;

    /* DutyA, DutyB, DutyC -> 16 bits, q0.16, always positive */
    uint16_t DutyA;
    uint16_t DutyB;
    uint16_t DutyC;
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
    // foc_circlelimit_dmax(&p_foc->Vd, &p_foc->Vq, p_foc->VectorMaxMagnitude, p_foc->VectorMaxD);
    p_foc->ReqMagnitude = foc_circlelimit(&p_foc->Vd, &p_foc->Vq, QFRAC16_MAX);
    foc_invpark_vector(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
    svpwm_midclamp(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, p_foc->Valpha, p_foc->Vbeta);
}

/* VBemf */
static inline void FOC_ProcVBemfClarkePark(FOC_T * p_foc)
{
    foc_clarke(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Va, p_foc->Vb, p_foc->Vc);
    foc_park_vector(&p_foc->Vd, &p_foc->Vq, p_foc->Valpha, p_foc->Vbeta, p_foc->Sine, p_foc->Cosine);
}

/* [0:32767] <=> [0:1] */
static inline uint16_t FOC_GetIMagnitude(const FOC_T * p_foc)     { return qfrac16_vectormagnitude(p_foc->Ialpha, p_foc->Ibeta); }
static inline uint16_t FOC_GetVMagnitude(const FOC_T * p_foc)     { return qfrac16_vectormagnitude(p_foc->Valpha, p_foc->Vbeta); }
static inline uint16_t FOC_GetIMagnitude_Idq(const FOC_T * p_foc) { return qfrac16_vectormagnitude(p_foc->Id, p_foc->Iq); }

/* [-32768:32767] <=> [-1:1] */
static inline qfrac16_t FOC_GetIPhase(const FOC_T * p_foc) { return FOC_GetIMagnitude(p_foc) * math_sign(p_foc->Iq); }
static inline qfrac16_t FOC_GetVPhase(const FOC_T * p_foc) { return FOC_GetVMagnitude(p_foc) * math_sign(p_foc->Vq); }

/* [0:49152] <=> [0:1.5] */
static inline int32_t FOC_GetPower(const FOC_T * p_foc) { return (qfrac16_mul(FOC_GetIPhase(p_foc), FOC_GetVPhase(p_foc)) * 3 / 2); }

static inline void FOC_SetTheta(FOC_T * p_foc, qangle16_t theta) { qfrac16_vector(&p_foc->Cosine, &p_foc->Sine, theta); }
static inline void FOC_SetIa(FOC_T * p_foc, qfrac16_t ia) { p_foc->Ia = ia; }
static inline void FOC_SetIb(FOC_T * p_foc, qfrac16_t ib) { p_foc->Ib = ib; }
static inline void FOC_SetIc(FOC_T * p_foc, qfrac16_t ic) { p_foc->Ic = ic; }
static inline void FOC_SetId(FOC_T * p_foc, qfrac16_t id) { p_foc->Id = id; }
static inline void FOC_SetIq(FOC_T * p_foc, qfrac16_t iq) { p_foc->Iq = iq; }
static inline void FOC_SetVd(FOC_T * p_foc, qfrac16_t vd) { p_foc->Vd = vd; }
static inline void FOC_SetVq(FOC_T * p_foc, qfrac16_t vq) { p_foc->Vq = vq; }

static inline uint16_t FOC_GetDutyA(const FOC_T * p_foc) { return p_foc->DutyA; }
static inline uint16_t FOC_GetDutyB(const FOC_T * p_foc) { return p_foc->DutyB; }
static inline uint16_t FOC_GetDutyC(const FOC_T * p_foc) { return p_foc->DutyC; }
static inline qfrac16_t FOC_GetId(const FOC_T * p_foc) { return p_foc->Id; }
static inline qfrac16_t FOC_GetIq(const FOC_T * p_foc) { return p_foc->Iq; }
static inline qfrac16_t FOC_GetVd(const FOC_T * p_foc) { return p_foc->Vd; }
static inline qfrac16_t FOC_GetVq(const FOC_T * p_foc) { return p_foc->Vq; }
static inline qfrac16_t FOC_GetIa(const FOC_T * p_foc) { return p_foc->Ia; }
static inline qfrac16_t FOC_GetIb(const FOC_T * p_foc) { return p_foc->Ib; }
static inline qfrac16_t FOC_GetIc(const FOC_T * p_foc) { return p_foc->Ic; }
static inline qfrac16_t FOC_GetIalpha(const FOC_T * p_foc) { return p_foc->Ialpha; }
static inline qfrac16_t FOC_GetIbeta(const FOC_T * p_foc) { return p_foc->Ibeta; }

/*  */
static inline void FOC_SetVBemfA(FOC_T * p_foc, qfrac16_t va) { p_foc->Va = va; }
static inline void FOC_SetVBemfB(FOC_T * p_foc, qfrac16_t vb) { p_foc->Vb = vb; }
static inline void FOC_SetVBemfC(FOC_T * p_foc, qfrac16_t vc) { p_foc->Vc = vc; }
static inline qfrac16_t FOC_GetVBemfA(const FOC_T * p_foc) { return p_foc->Va; }
static inline qfrac16_t FOC_GetVBemfB(const FOC_T * p_foc) { return p_foc->Vb; }
static inline qfrac16_t FOC_GetVBemfC(const FOC_T * p_foc) { return p_foc->Vc; }

static inline void FOC_SetDReq(FOC_T * p_foc, qfrac16_t id) { p_foc->DReq = id; }
static inline void FOC_SetQReq(FOC_T * p_foc, qfrac16_t iq) { p_foc->QReq = iq; }
static inline qfrac16_t FOC_GetDReq(const FOC_T * p_foc) { return p_foc->DReq; }
static inline qfrac16_t FOC_GetQReq(const FOC_T * p_foc) { return p_foc->QReq; }

extern void FOC_Init(FOC_T * p_foc);
extern void FOC_SetAlign(FOC_T * p_foc, qfrac16_t vd);
extern void FOC_ZeroSvpwm(FOC_T * p_foc);
extern void FOC_ClearControlState(FOC_T * p_foc);
extern void FOC_ClearObserveState(FOC_T * p_foc);
// extern void FOC_SetVectorMax(FOC_T * p_foc, qfrac16_t dMax);

#endif
