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
    @version V0
    @brief  FOC math functions with state. Wrapper for pure math functions.
*/
/******************************************************************************/
#ifndef FOC_H
#define FOC_H

#include "math_foc.h"
#include "math_svpwm.h"
#include "Math/Fixed/fract16.h"

typedef struct FOC
{
    ufract16_t Modulation;
    ufract16_t VBus;
    ufract16_t VBusInvScalar;
    ufract16_t VPhaseLimit;
    ufract16_t VdLimit;

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
    /* PID Control Variable, or intermediate feedforward input */
    fract16_t Vd;
    fract16_t Vq;

    fract16_t Valpha;
    fract16_t Vbeta;

    /* VBemf Inputs during Freewheel - Capture by ADC */
    ufract16_t Va;
    ufract16_t Vb;
    ufract16_t Vc;

    /* Scalar */
    ufract16_t DutyA;
    ufract16_t DutyB;
    ufract16_t DutyC;

}
FOC_T;

/* Update VBusInvScalar */
// static inline void FOC_CaptureVBus(FOC_T * p_foc, fract16_t vBus)
// {
//     p_foc->VBus = vBus;
//     p_foc->VBusInvScalar = INT32_MAX / p_foc->VBus; /* vDuty16 = vPhase/VBus */
//     p_foc->VdLimit = fract16_mul(p_foc->VBus, FRACT16_1_DIV_SQRT3);
//     p_foc->VPhaseLimit = fract16_mul(p_foc->VBus, FRACT16_1_DIV_SQRT3);
// }

/* ProcInputI */
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

// static inline void FOC_ProcIFeedback(FOC_T * p_foc, PID_T * p_pidId, PID_T * p_pidIq)
// {
//     p_foc->Vq = PID_ProcPI(p_pidIq, p_foc->Iq, p_foc->ReqQ);
//     p_foc->Vd = PID_ProcPI(p_pidId, p_foc->Id, p_foc->ReqD);
// }

static inline void FOC_ProcVectorLimit(FOC_T * p_foc)
{
    foc_circle_limit(&p_foc->Vd, &p_foc->Vq, p_foc->VPhaseLimit, p_foc->VdLimit);
}

// static inline void FOC_ProcVectorLimit(FOC_T * p_foc, fract16_t vBus)
// {
//     foc_circle_limit(&p_foc->Vd, &p_foc->Vq, fract16_mul(vBus, FRACT16_1_DIV_SQRT3), fract16_mul(vBus, FRACT16_1_DIV_SQRT3));
// }

static inline void FOC_ProcInvClarkePark(FOC_T * p_foc)
{
    foc_invpark_vector(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
    foc_invclarke(&p_foc->Va, &p_foc->Vb, &p_foc->Vc, p_foc->Valpha, p_foc->Vbeta);
}

/*
    ProcVout
*/

/*
    Normalize V to Duty Scalar
    Svpwm Duty as VPhase/(VBus/sqrt(3))
    [-VBus/sqrt3:VBus/sqrt3] <=> [-1:1]
*/
// static inline ufract16_t _FOC_DutyOfV(const FOC_T * p_foc, ufract16_t v_fract)
// {
// }

/* with Scaled DutyAlpha, DutyBeta */
/*
    Normalized by a factor of sqrt(3)/2,
        alpha 1/sqrt(3) => a = .5
        alphaDuty 1.15 => a = 1.0
*/
// int32_t betaDuty = fract16_mul(beta, FRACT16_SQRT3_DIV_2);
// int32_t alphaDuty = fract16_mul(alpha, FRACT16_SQRT3_DIV_2);
// static inline void FOC_ProcInvParkInvClarkeSvpwm(FOC_T * p_foc)
// {
//     foc_invpark_vector(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
//     svpwm_midclamp(&p_foc->Va, &p_foc->Vb, &p_foc->Vc, _FOC_DutyOfV(p_foc, p_foc->Valpha), _FOC_DutyOfV(p_foc, p_foc->Vbeta));
// }

/*
    Normalize V to VBus Scalar
    Svpwm Duty as VPhase/VBus

    [-VBus/sqrt3:VBus/sqrt3] <=> [-1/sqrt3:1/sqrt3]
*/
// static inline fract16_t _FOC_VBusScalarOf(const FOC_T * p_foc, fract16_t v_fract)
// {
//     assert(v_fract < p_foc->VBus);
//     assert(v_fract < p_foc->VPhaseLimit);
//     return v_fract * p_foc->VBusInvScalar / 65536;
// }

// static inline void FOC_ProcSvpwm(FOC_T * p_foc)
// {
//     svpwm_midclamp_vbus(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, _FOC_VBusScalarOf(p_foc, p_foc->Va), _FOC_VBusScalarOf(p_foc, p_foc->Vb), _FOC_VBusScalarOf(p_foc, p_foc->Vc));
// }

static inline fract16_t _FOC_Scale(int32_t vDutyScalar, fract16_t v_fract) { return v_fract * vDutyScalar / 65536; }

/* precomputed vDutyScalar 1.0F/VBus */
static inline void FOC_ProcSvpwm(FOC_T * p_foc, int32_t vDutyScalar)
{
    svpwm_midclamp_vbus(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, _FOC_Scale(vDutyScalar, p_foc->Va), _FOC_Scale(vDutyScalar, p_foc->Vb), _FOC_Scale(vDutyScalar, p_foc->Vc));
}

static inline void FOC_ProcOutputV(FOC_T * p_foc, int32_t vDutyScalar)
{
    FOC_ProcInvClarkePark(p_foc);
    FOC_ProcSvpwm(p_foc, vDutyScalar);
}

/* VBemf */
static inline void FOC_ProcVBemfClarkePark(FOC_T * p_foc)
{
    foc_clarke(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Va, p_foc->Vb, p_foc->Vc);
    foc_park_vector(&p_foc->Vd, &p_foc->Vq, p_foc->Valpha, p_foc->Vbeta, p_foc->Sine, p_foc->Cosine);
}

/*
    Capture Inputs
*/
static inline void FOC_SetTheta(FOC_T * p_foc, angle16_t theta) { fract16_vector(&p_foc->Cosine, &p_foc->Sine, theta); }

static inline void FOC_SetIa(FOC_T * p_foc, fract16_t ia) { p_foc->Ia = ia; }
static inline void FOC_SetIb(FOC_T * p_foc, fract16_t ib) { p_foc->Ib = ib; }
static inline void FOC_SetIc(FOC_T * p_foc, fract16_t ic) { p_foc->Ic = ic; }

static inline void FOC_SetReqD(FOC_T * p_foc, fract16_t d) { p_foc->ReqD = d; }
static inline void FOC_SetReqQ(FOC_T * p_foc, fract16_t q) { p_foc->ReqQ = q; }

/* Feedback Control */
static inline void FOC_SetVd(FOC_T * p_foc, fract16_t vd) { p_foc->Vd = vd; }
static inline void FOC_SetVq(FOC_T * p_foc, fract16_t vq) { p_foc->Vq = vq; }

/* VDuty */
static inline ufract16_t FOC_GetDutyA(const FOC_T * p_foc) { return p_foc->DutyA; }
static inline ufract16_t FOC_GetDutyB(const FOC_T * p_foc) { return p_foc->DutyB; }
static inline ufract16_t FOC_GetDutyC(const FOC_T * p_foc) { return p_foc->DutyC; }

/*
    Run-time derived
*/
/* [0:32767] <=> [0:1]AnalogRefMax */
static inline ufract16_t FOC_GetIMagnitude(const FOC_T * p_foc)     { return fract16_vector_magnitude(p_foc->Ialpha, p_foc->Ibeta); }
static inline ufract16_t FOC_GetVMagnitude(const FOC_T * p_foc)     { return fract16_vector_magnitude(p_foc->Valpha, p_foc->Vbeta); }
static inline ufract16_t FOC_GetIMagnitude_Idq(const FOC_T * p_foc) { return fract16_vector_magnitude(p_foc->Id, p_foc->Iq); }

/* [-32768:32767] <=> [-1:1]AnalogRefMax */
static inline fract16_t FOC_GetIPhase(const FOC_T * p_foc) { return FOC_GetIMagnitude(p_foc) * math_sign(p_foc->Iq); }
static inline fract16_t FOC_GetVPhase(const FOC_T * p_foc) { return FOC_GetVMagnitude(p_foc) * math_sign(p_foc->Vq); }

/* [0:49152] <=> [0:1.5]AnalogRefMax */
// static inline accum32_t FOC_GetPower(const FOC_T * p_foc) { return fract16_mul(FOC_GetIPhase(p_foc), FOC_GetVPhase(p_foc)) * 3 / 2; }
static inline accum32_t FOC_GetPower(const FOC_T * p_foc) { return (fract16_mul(p_foc->Id, p_foc->Vd) + fract16_mul(p_foc->Iq, p_foc->Vq)) * 3 / 2; }

static inline bool FOC_IsMotoring(const FOC_T * p_foc)      { return (math_sign(p_foc->Vq) == math_sign(p_foc->Iq)); }
static inline bool FOC_IsGenerating(const FOC_T * p_foc)    { return (math_sign(p_foc->Vq) != math_sign(p_foc->Iq)); }


#ifdef CONFIG_FOC_V_AS_SCALAR
/* V_Fract16 when Vd Vq are normalized values of VBus = VDutyMax */
static inline ufract16_t FOC_VOfDuty(const FOC_T * p_foc, fract16_t duty) { return fract16_mul(duty, fract16_mul(p_foc->VBus, FRACT16_1_DIV_SQRT3)); }
static inline ufract16_t FOC_GetVa(const FOC_T * p_foc) { return FOC_VOfDuty(p_foc, p_foc->Va); }
static inline ufract16_t FOC_GetVb(const FOC_T * p_foc) { return FOC_VOfDuty(p_foc, p_foc->Vb); }
static inline ufract16_t FOC_GetVc(const FOC_T * p_foc) { return FOC_VOfDuty(p_foc, p_foc->Vc); }
static inline ufract16_t FOC_GetValpha(const FOC_T * p_foc) { return FOC_VOfDuty(p_foc, p_foc->Valpha); }
static inline ufract16_t FOC_GetVMagnitude(const FOC_T * p_foc) { return fract16_vector_magnitude(p_foc->Valpha, p_foc->Vbeta); }
#endif

// static inline void FOC_SetId(FOC_T * p_foc, fract16_t id) { p_foc->Id = id; }
// static inline void FOC_SetIq(FOC_T * p_foc, fract16_t iq) { p_foc->Iq = iq; }

static inline ufract16_t FOC_GetVa(const FOC_T * p_foc) { return p_foc->Va; }
static inline ufract16_t FOC_GetVb(const FOC_T * p_foc) { return p_foc->Vb; }
static inline ufract16_t FOC_GetVc(const FOC_T * p_foc) { return p_foc->Vc; }

static inline fract16_t FOC_GetId(const FOC_T * p_foc) { return p_foc->Id; }
static inline fract16_t FOC_GetIq(const FOC_T * p_foc) { return p_foc->Iq; }
static inline fract16_t FOC_GetVd(const FOC_T * p_foc) { return p_foc->Vd; }
static inline fract16_t FOC_GetVq(const FOC_T * p_foc) { return p_foc->Vq; }
static inline fract16_t FOC_GetIa(const FOC_T * p_foc) { return p_foc->Ia; }
static inline fract16_t FOC_GetIb(const FOC_T * p_foc) { return p_foc->Ib; }
static inline fract16_t FOC_GetIc(const FOC_T * p_foc) { return p_foc->Ic; }
static inline fract16_t FOC_GetIalpha(const FOC_T * p_foc) { return p_foc->Ialpha; }
static inline fract16_t FOC_GetIbeta(const FOC_T * p_foc) { return p_foc->Ibeta; }

/* Capture */
static inline void FOC_SetVBemfA(FOC_T * p_foc, ufract16_t va) { p_foc->Va = va; }
static inline void FOC_SetVBemfB(FOC_T * p_foc, ufract16_t vb) { p_foc->Vb = vb; }
static inline void FOC_SetVBemfC(FOC_T * p_foc, ufract16_t vc) { p_foc->Vc = vc; }
static inline ufract16_t FOC_GetVBemfA(const FOC_T * p_foc) { return p_foc->Va; }
static inline ufract16_t FOC_GetVBemfB(const FOC_T * p_foc) { return p_foc->Vb; }
static inline ufract16_t FOC_GetVBemfC(const FOC_T * p_foc) { return p_foc->Vc; }

// static inline ufract16_t FOC_GetVBemf(const FOC_T * p_foc)
// {
//     FOC_ProcVBemfClarkePark(p_foc);
//     return FOC_GetVPhase(p_foc);
// }

static inline fract16_t FOC_GetReqD(const FOC_T * p_foc) { return p_foc->ReqD; }
static inline fract16_t FOC_GetReqQ(const FOC_T * p_foc) { return p_foc->ReqQ; }

extern void FOC_Init(FOC_T * p_foc, fract16_t vBus);
extern void FOC_SetAlign(FOC_T * p_foc, fract16_t vd);
extern void FOC_ZeroSvpwm(FOC_T * p_foc);
extern void FOC_ClearControlState(FOC_T * p_foc);
// extern void FOC_SetVectorMax(FOC_T * p_foc, fract16_t dMax);

#endif

// alternatively, caller handles filter
// static inline void FOC_CaptureIa(FOC_T * p_foc, fract16_t ia) { p_foc->Ia = (ia + p_foc->Ia) / 2; }
// static inline void FOC_CaptureIb(FOC_T * p_foc, fract16_t ib) { p_foc->Ib = (ib + p_foc->Ib) / 2; }
// static inline void FOC_CaptureIc(FOC_T * p_foc, fract16_t ic) { p_foc->Ic = (ic + p_foc->Ic) / 2; }

// static inline void FOC_CaptureIabc(FOC_T * p_foc, fract16_t ia, fract16_t ib, fract16_t ic)
// {
//     FOC_CaptureIa(p_foc, ia);
//     FOC_CaptureIb(p_foc, ib);
//     FOC_CaptureIc(p_foc, ic);
// }

/*

*/
// typedef enum Motor_SectorId
// {
//     MOTOR_SECTOR_ID_0 = 0U,
//     MOTOR_SECTOR_ID_1 = 1U, /* 0_60 */
//     MOTOR_SECTOR_ID_2 = 2U,
//     MOTOR_SECTOR_ID_3 = 3U,
//     MOTOR_SECTOR_ID_4 = 4U,
//     MOTOR_SECTOR_ID_5 = 5U,
//     MOTOR_SECTOR_ID_6 = 6U,
//     MOTOR_SECTOR_ID_7 = 7U,
// }
// Motor_SectorId_T;

// Motor_SectorId_T Motor_SectorIdOfAngle(angle16_t angle)
// {
//     Motor_SectorId_T sectorId;
//     if (angle < ANGLE16_180)
//     {
//         if (angle < ANGLE16_60) { sectorId = MOTOR_SECTOR_ID_1; }
//         else if (angle < ANGLE16_120) { sectorId = MOTOR_SECTOR_ID_2; }
//         else { sectorId = MOTOR_SECTOR_ID_3; }
//     }
//     else
//     {
//         if (angle < ANGLE16_240) { sectorId = MOTOR_SECTOR_ID_4; }
//         else if (angle < ANGLE16_300) { sectorId = MOTOR_SECTOR_ID_5; }
//         else { sectorId = MOTOR_SECTOR_ID_6; }
//     }
//     return sectorId;
// }