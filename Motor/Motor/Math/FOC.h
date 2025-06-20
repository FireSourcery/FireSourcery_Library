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
#include "math_foc.h"
#include "math_svpwm.h"
#include "Math/Fixed/fract16.h"

typedef struct FOC
{
    ufract16_t Modulation;
    // ufract16_t VBus;
    // ufract16_t VBusInvScalar;
    // ufract16_t VPhaseLimit;
    // ufract16_t VdLimit;
    // value_t * p_VBus;

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


/* ProcInputI */
// static inline void FOC_ProcClarkePark(FOC_T * p_foc, a, b, c)
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

// static inline void FOC_ProcVectorLimit(FOC_T * p_foc)
// {
//     foc_circle_limit(&p_foc->Vd, &p_foc->Vq, p_foc->VPhaseLimit, p_foc->VdLimit);
// }

static inline bool FOC_ProcVectorLimit(FOC_T * p_foc, fract16_t vBus)
{
    fract16_t vBusLimit = fract16_mul(vBus, FRACT16_1_DIV_SQRT3);
    return foc_circle_limit(&p_foc->Vd, &p_foc->Vq, vBusLimit, vBusLimit);
}

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
    [-VBus/sqrt3:VBus/sqrt3] <=> [-1:1]
    Svpwm input as VPhase/(VBus/sqrt(3))
*/
// static inline accum32_t _FOC_VOfVNorm(int32_t vBus_fract, fract16_t vNorm) { return fract16_mul(vNorm, fract16_mul(vBus_fract, FRACT16_1_DIV_SQRT3)); }
// static inline accum32_t _FOC_VNormOfV(int32_t vBus_fract, fract16_t v_fract) { return fract16_div_sat(v_fract, fract16_mul(vBus_fract, FRACT16_1_DIV_SQRT3)); }

/*
    Normalize V to VBus as 1.0F
    [-VBus/sqrt3:VBus/sqrt3] <=> [-1/sqrt3:1/sqrt3]
    Svpwm input as VPhase/VBus

    v_fract16 < vBus_fract16
    vBusInv_fract32 = INT32_MAX / vBus_fract16
    v_fract16 * vBusInv_fract32 < INT32_MAX
*/
static inline fract16_t _FOC_VOfVNorm(fract16_t vBus_fract16, fract16_t vNorm) { return fract16_mul(vNorm, vBus_fract16); }
static inline fract16_t _FOC_VNormOfV(fract16_t vBus_fract16, fract16_t v_fract16) { return fract16_div(v_fract16, vBus_fract16); }
static inline fract16_t _FOC_VNormOfV_Inv32(int32_t vBusInv_fract32, fract16_t v_fract16) { return v_fract16 * vBusInv_fract32 / 65536; }

/* precomputed vBusInv_fract32 1.0F/VBus */
static inline void FOC_ProcSvpwm(FOC_T * p_foc, int32_t vBusInv_fract32)
{
    /* Validate vBusInv is reasonable */
    // assert(vBusInv_fract32 > 0);
    fract16_t vNormA = _FOC_VNormOfV_Inv32(vBusInv_fract32, p_foc->Va);
    fract16_t vNormB = _FOC_VNormOfV_Inv32(vBusInv_fract32, p_foc->Vb);
    fract16_t vNormC = _FOC_VNormOfV_Inv32(vBusInv_fract32, p_foc->Vc);
    svpwm_midclamp_vbus(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, vNormA, vNormB, vNormC);
}

static inline void FOC_ProcOutputV_VBusInv(FOC_T * p_foc, int32_t vBusInv_fract32)
{
    FOC_ProcInvClarkePark(p_foc);
    FOC_ProcSvpwm(p_foc, vBusInv_fract32); /* alternatively phase handle with acess to vbus */
}

static inline void FOC_ProcOutputV_VBus(FOC_T * p_foc, fract16_t vBus_fract16)
{
    FOC_ProcOutputV_VBusInv(p_foc, (INT32_MAX / vBus_fract16));
}

/* VBemf */
static inline void FOC_ProcVBemfClarkePark(FOC_T * p_foc)
{
    foc_clarke(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Va, p_foc->Vb, p_foc->Vc);
    foc_park_vector(&p_foc->Vd, &p_foc->Vq, p_foc->Valpha, p_foc->Vbeta, p_foc->Sine, p_foc->Cosine);
}


/* Update VBusInvScalar */
// static inline void FOC_CaptureVBus(FOC_T * p_foc, fract16_t vBus)
// {
//     p_foc->VBus = vBus;
//     p_foc->VBusInvScalar = INT32_MAX / p_foc->VBus; /* vDuty16 = vPhase/VBus */
//     p_foc->VdLimit = fract16_mul(p_foc->VBus, FRACT16_1_DIV_SQRT3);
//     p_foc->VPhaseLimit = fract16_mul(p_foc->VBus, FRACT16_1_DIV_SQRT3);
// }

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

/* Store as max modulation, derive vbus */
/* V_Fract16 when Vd Vq are stored as normalized values */
#ifdef CONFIG_FOC_V_AS_SCALAR
// static inline ufract16_t FOC_GetVa(const FOC_T * p_foc) { return _FOC_VOfVNorm(p_foc, p_foc->Va); }
// static inline ufract16_t FOC_GetVb(const FOC_T * p_foc) { return _FOC_VOfVNorm(p_foc, p_foc->Vb); }
// static inline ufract16_t FOC_GetVc(const FOC_T * p_foc) { return _FOC_VOfVNorm(p_foc, p_foc->Vc); }

// static inline ufract16_t FOC_GetValpha(const FOC_T * p_foc) { return _FOC_VOfVNorm(p_foc, p_foc->Valpha); }
// static inline ufract16_t FOC_GetVMagnitude(const FOC_T * p_foc) { return fract16_vector_magnitude(FOC_GetValpha(p_foc), FOC_GetVbeta(p_foc)); }

// static inline void _FOC_SetVBemfA_AsNorm(FOC_T * p_foc, int32_t vBusInv_fract32, ufract16_t va) { p_foc->Va = _FOC_VNormOfV_Inv32(vBusInv_fract32, va); }
// static inline void _FOC_SetVBemfB_AsNorm(FOC_T * p_foc, int32_t vBusInv_fract32, ufract16_t vb) { p_foc->Vb = _FOC_VNormOfV_Inv32(vBusInv_fract32, vb); }
// static inline void _FOC_SetVBemfC_AsNorm(FOC_T * p_foc, int32_t vBusInv_fract32, ufract16_t vc) { p_foc->Vc = _FOC_VNormOfV_Inv32(vBusInv_fract32, vc); }
#else
static inline ufract16_t FOC_GetVa(const FOC_T * p_foc) { return p_foc->Va; }
static inline ufract16_t FOC_GetVb(const FOC_T * p_foc) { return p_foc->Vb; }
static inline ufract16_t FOC_GetVc(const FOC_T * p_foc) { return p_foc->Vc; }
#endif




/*
    Run-time derived
*/
static inline bool FOC_IsMotoring(const FOC_T * p_foc) { return (math_sign(p_foc->Vq) == math_sign(p_foc->Iq)); }
static inline bool FOC_IsGenerating(const FOC_T * p_foc) { return (math_sign(p_foc->Vq) != math_sign(p_foc->Iq)); }


/* [0:32767] <=> [0:1]AnalogRefMax */
static inline ufract16_t FOC_GetIMagnitude(const FOC_T * p_foc)     { return fract16_vector_magnitude(p_foc->Ialpha, p_foc->Ibeta); }
static inline ufract16_t FOC_GetVMagnitude(const FOC_T * p_foc)     { return fract16_vector_magnitude(p_foc->Valpha, p_foc->Vbeta); }
static inline ufract16_t FOC_GetIMagnitude_Idq(const FOC_T * p_foc) { return fract16_vector_magnitude(p_foc->Id, p_foc->Iq); }

/* [-32768:32767] <=> [-1:1]AnalogRefMax */
static inline fract16_t FOC_GetIPhase(const FOC_T * p_foc) { return FOC_GetIMagnitude(p_foc) * math_sign(p_foc->Iq); }
static inline fract16_t FOC_GetVPhase(const FOC_T * p_foc) { return FOC_GetVMagnitude(p_foc) * math_sign(p_foc->Vq); }


/*
    Power calculations
    Optionally handle by host
*/
/* [0:49152] <=> [0:1.5]*AnalogRefMax */
static inline accum32_t FOC_GetPower(const FOC_T * p_foc) { return (fract16_mul(p_foc->Id, p_foc->Vd) + fract16_mul(p_foc->Iq, p_foc->Vq)) * 3 / 2; }

/* Active power (torque-producing) */
static inline accum32_t FOC_GetActivePower(const FOC_T * p_foc) { return fract16_mul(p_foc->Iq, p_foc->Vq) * 3 / 2; }
/* Reactive power (magnetizing) */
static inline accum32_t FOC_GetReactivePower(const FOC_T * p_foc) { return fract16_mul(p_foc->Id, p_foc->Vd) * 3 / 2; }
static inline accum32_t FOC_GetTotalPower(const FOC_T * p_foc) { return FOC_GetActivePower(p_foc) + FOC_GetReactivePower(p_foc); }

static inline ufract16_t FOC_GetPowerFactor(const FOC_T * p_foc) { return fract16_div_sat(FOC_GetActivePower(p_foc), FOC_GetTotalPower(p_foc)); }

/* Estimate efficiency based on I²R losses */
// static inline ufract16_t FOC_GetEfficiency(const FOC_T * p_foc, fract16_t rPhase)
// {
//     accum32_t activePower = FOC_GetActivePower(p_foc);
//     if (activePower <= 0) return 0;

//     ufract16_t iMag = FOC_GetIMagnitude(p_foc);
//     accum32_t resistiveLoss = fract16_mul(fract16_mul(iMag, iMag), rPhase) * 3;

//     return (activePower << 15) / (activePower + resistiveLoss);
// }

static inline fract16_t FOC_GetId(const FOC_T * p_foc) { return p_foc->Id; }
static inline fract16_t FOC_GetIq(const FOC_T * p_foc) { return p_foc->Iq; }
static inline fract16_t FOC_GetVd(const FOC_T * p_foc) { return p_foc->Vd; }
static inline fract16_t FOC_GetVq(const FOC_T * p_foc) { return p_foc->Vq; }
static inline fract16_t FOC_GetIa(const FOC_T * p_foc) { return p_foc->Ia; }
static inline fract16_t FOC_GetIb(const FOC_T * p_foc) { return p_foc->Ib; }
static inline fract16_t FOC_GetIc(const FOC_T * p_foc) { return p_foc->Ic; }
static inline fract16_t FOC_GetIalpha(const FOC_T * p_foc) { return p_foc->Ialpha; }
static inline fract16_t FOC_GetIbeta(const FOC_T * p_foc) { return p_foc->Ibeta; }

static inline fract16_t FOC_GetReqD(const FOC_T * p_foc) { return p_foc->ReqD; }
static inline fract16_t FOC_GetReqQ(const FOC_T * p_foc) { return p_foc->ReqQ; }


/******************************************************************************/
/*!
    Bemf
*/
/******************************************************************************/
/* Capture */
static inline void FOC_SetVBemfA(FOC_T * p_foc, ufract16_t va) { p_foc->Va = va; }
static inline void FOC_SetVBemfB(FOC_T * p_foc, ufract16_t vb) { p_foc->Vb = vb; }
static inline void FOC_SetVBemfC(FOC_T * p_foc, ufract16_t vc) { p_foc->Vc = vc; }
static inline ufract16_t FOC_GetVBemfA(const FOC_T * p_foc) { return p_foc->Va; }
static inline ufract16_t FOC_GetVBemfB(const FOC_T * p_foc) { return p_foc->Vb; }
static inline ufract16_t FOC_GetVBemfC(const FOC_T * p_foc) { return p_foc->Vc; }


// static inline void FOC_SetId(FOC_T * p_foc, fract16_t id) { p_foc->Id = id; }
// static inline void FOC_SetIq(FOC_T * p_foc, fract16_t iq) { p_foc->Iq = iq; }


/******************************************************************************/
/*!

*/
/******************************************************************************/
extern void FOC_Init(FOC_T * p_foc);
extern void FOC_SetAlign(FOC_T * p_foc, fract16_t vd);
extern void FOC_ZeroSvpwm(FOC_T * p_foc);
extern void FOC_ClearCaptureState(FOC_T * p_foc);
// extern void FOC_SetVectorMax(FOC_T * p_foc, fract16_t dMax);


/******************************************************************************/
/*!
*/
/******************************************************************************/
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

/* with Scaled DutyAlpha, DutyBeta */
/*
    Normalized by a factor of sqrt(3)/2,
        alpha 1/sqrt(3) => a = .5
        alphaDuty 1.15 => a = 1.0
*/
// static inline void FOC_ProcInvParkInvClarkeSvpwm(FOC_T * p_foc)
// {
//     foc_invpark_vector(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
//     svpwm_midclamp(&p_foc->Va, &p_foc->Vb, &p_foc->Vc, _FOC_DutyOfV(p_foc, p_foc->Valpha), _FOC_DutyOfV(p_foc, p_foc->Vbeta));
// }


// Add field weakening support
// static inline fract16_t FOC_CalcFieldWeakeningId(const FOC_T * p_foc, fract16_t vBus, fract16_t speed)
// {
//     /* Calculate Id for field weakening at high speeds */
//     fract16_t vLimit = fract16_mul(vBus, FRACT16_1_DIV_SQRT3);
//     fract16_t vBemfEst = fract16_mul(speed, MOTOR_KE_CONSTANT);  /* Requires motor constant */

//     if (vBemfEst > vLimit)
//     {
//         /* Need field weakening */
//         return -(vBemfEst - vLimit);  /* Negative Id for field weakening */
//     }

//     return 0;  /* No field weakening needed */
// }

// // Add maximum torque per amp (MTPA) calculation
// static inline void FOC_CalcMTPA(FOC_T * p_foc, fract16_t iqReq)
// {
//     /* Simplified MTPA for surface-mounted PM motors */
//     p_foc->ReqD = 0;  /* Id = 0 for SMPM motors */
//     p_foc->ReqQ = iqReq;

//     /* For interior PM motors, add MTPA calculation:
//      * Id_opt = -λ_pm/(2*Ld) + sqrt((λ_pm/(2*Ld))² + Iq²)
//      */
// }

// // Add six-step commutation for high-speed operation
// static inline void FOC_ProcSixStep(FOC_T * p_foc, angle16_t angle, fract16_t magnitude)
// {
//     Motor_SectorId_T sector = FOC_GetSectorId(angle);

//     /* Six-step commutation pattern */
//     switch (sector)
//     {
//         case MOTOR_SECTOR_ID_1:  /* A+, B- */
//             p_foc->Va = magnitude;  p_foc->Vb = -magnitude; p_foc->Vc = 0; break;
//         case MOTOR_SECTOR_ID_2:  /* A+, C- */
//             p_foc->Va = magnitude;  p_foc->Vb = 0; p_foc->Vc = -magnitude; break;
//         case MOTOR_SECTOR_ID_3:  /* B+, C- */
//             p_foc->Va = 0; p_foc->Vb = magnitude;  p_foc->Vc = -magnitude; break;
//         case MOTOR_SECTOR_ID_4:  /* B+, A- */
//             p_foc->Va = -magnitude; p_foc->Vb = magnitude;  p_foc->Vc = 0; break;
//         case MOTOR_SECTOR_ID_5:  /* C+, A- */
//             p_foc->Va = -magnitude; p_foc->Vb = 0; p_foc->Vc = magnitude; break;
//         case MOTOR_SECTOR_ID_6:  /* C+, B- */
//             p_foc->Va = 0; p_foc->Vb = -magnitude; p_foc->Vc = magnitude; break;
//     }
// }