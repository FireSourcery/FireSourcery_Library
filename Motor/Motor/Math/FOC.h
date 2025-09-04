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
    /* Theta - Save for Inverse Park */
    fract16_t Sine, Cosine;

    /* Feedback Variable */
    fract16_t Id, Iq;

    /* Control Variable. VOutput. Feedforward Input. */
    /* VBemf during Freewheel */
    fract16_t Vd, Vq; /* VdReq, VqReq */

    /* Scalar */
    ufract16_t DutyA, DutyB, DutyC;

    /* Inputs - Capture by ADC */
    // fract16_t Ia, Ib, Ic;

    /* VBemf Inputs during Freewheel - Capture by ADC */

    /* Intermediate VOut */
    fract16_t Va, Vb, Vc;

    /* Request in V or I  */
    /* Setpoint Variable - From TorqueRamp, SpeedPid, OpenLoop */
    /* Alternatively caller holds */
    // Async set or
    // Optionally store 1 point of access to simplify UI access
    // fract16_t ReqD, ReqQ;

    /* Intermediate values */
    fract16_t Ialpha, Ibeta;
    fract16_t Valpha, Vbeta;

    // ufract16_t Modulation;
}
FOC_T;

/******************************************************************************/
/*
    Capture Inputs
*/
/******************************************************************************/
static inline void FOC_SetTheta(FOC_T * p_foc, angle16_t theta) { fract16_vector(&p_foc->Cosine, &p_foc->Sine, theta); }


/*
    InputI
*/
static inline void FOC_ProcClarkePark(FOC_T * p_foc, fract16_t ia, fract16_t ib, fract16_t ic)
{
    foc_clarke_park(&p_foc->Id, &p_foc->Iq, ia, ib, ic, p_foc->Sine, p_foc->Cosine);
}

// static inline void FOC_CaptureIabc(FOC_T * p_foc, fract16_t ia, fract16_t ib, fract16_t ic)
// {
//     FOC_CaptureIa(p_foc, ia);
//     FOC_CaptureIb(p_foc, ib);
//     FOC_CaptureIc(p_foc, ic);
// }

// static inline void FOC_ProcClarkePark(FOC_T * p_foc)
// {
//     foc_clarke_park(&p_foc->Id, &p_foc->Iq, p_foc->Ia, p_foc->Ib, p_foc->Ic, p_foc->Sine, p_foc->Cosine);
// }

// static inline void FOC_ProcClarkePark_AB(FOC_T * p_foc)
// {
//     foc_clarke_ab(&p_foc->Ialpha, &p_foc->Ibeta, p_foc->Ia, p_foc->Ib);
//     foc_park_vector(&p_foc->Id, &p_foc->Iq, p_foc->Ialpha, p_foc->Ibeta, p_foc->Sine, p_foc->Cosine);
// }

/*
    OutputV
*/

/* Buffer */
// static inline void FOC_SetReqD(FOC_T * p_foc, fract16_t d) { p_foc->ReqD = d; }
// static inline void FOC_SetReqQ(FOC_T * p_foc, fract16_t q) { p_foc->ReqQ = q; }

/* Feedback/Feedforward Control */
static inline void FOC_SetVd(FOC_T * p_foc, fract16_t vd) { p_foc->Vd = vd; }
static inline void FOC_SetVq(FOC_T * p_foc, fract16_t vq) { p_foc->Vq = vq; }

// static inline void FOC_SetVdq(FOC_T * p_foc, fract16_t vd, fract16_t vq, fract16_t vBus)
// {
//     p_foc->Vd = vd;
//     p_foc->Vq = vq;
//     foc_circle_limit_q
// }

/*  */
static inline bool FOC_ProcVectorLimit(FOC_T * p_foc, fract16_t vBus)
{
    fract16_t vPhaseLimit = fract16_mul(vBus, FRACT16_1_DIV_SQRT3); /* optionally * modulation */
    // return foc_circle_limit(&p_foc->Vd, &p_foc->Vq, vPhaseLimit, vPhaseLimit); /* p_foc->ModulationVLimit, p_foc->VdLimit */
    // assert(abs(p_foc->Vd) <= vPhaseLimit); /* set by feedback output */
    return foc_circle_limit_q(&p_foc->Vd, &p_foc->Vq, vPhaseLimit);
}

// static inline void FOC_ProcInvClarkePark(FOC_T * p_foc, fract16_t vd, fract16_t vq)
static inline void FOC_ProcInvClarkePark(FOC_T * p_foc)
{
    foc_inv_clarke_park(&p_foc->Va, &p_foc->Vb, &p_foc->Vc, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
}

/******************************************************************************/
/*!
    VOut
*/
/******************************************************************************/
// static inline ufract16_t FOC_VNormA(const FOC_T * p_foc, uint32_t vBusInv_fract32) { return svpwm_norm_vbus_inv(vBusInv_fract32, p_foc->Va); }
// static inline ufract16_t FOC_VNormB(const FOC_T * p_foc, uint32_t vBusInv_fract32) { return svpwm_norm_vbus_inv(vBusInv_fract32, p_foc->Vb); }
// static inline ufract16_t FOC_VNormC(const FOC_T * p_foc, uint32_t vBusInv_fract32) { return svpwm_norm_vbus_inv(vBusInv_fract32, p_foc->Vc); }

/* precomputed vBusInv_fract32 <=> 1.0F/VBus */
static inline void FOC_ProcSvpwm(FOC_T * p_foc, uint32_t vBusInv_fract32)
{
    fract16_t vNormA = svpwm_norm_vbus_inv(vBusInv_fract32, p_foc->Va);
    fract16_t vNormB = svpwm_norm_vbus_inv(vBusInv_fract32, p_foc->Vb);
    fract16_t vNormC = svpwm_norm_vbus_inv(vBusInv_fract32, p_foc->Vc);
    svpwm_midclamp_vbus(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, vNormA, vNormB, vNormC);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline void FOC_ProcOutputV_VBusInv(FOC_T * p_foc, uint32_t vBusInv_fract32)
{
    FOC_ProcInvClarkePark(p_foc);
    FOC_ProcSvpwm(p_foc, vBusInv_fract32);
}

static inline void FOC_ProcOutputV_VBus(FOC_T * p_foc, ufract16_t vBus_fract16)
{
    FOC_ProcOutputV_VBusInv(p_foc, ((uint32_t)FRACT16_MAX * 65536U / vBus_fract16));
}


/* Store as max modulation, derive V_Fract16 by vbus */
/* V_Fract16 when Vd Vq are stored as normalized scalar values */
#ifdef FOC_V_AS_SCALAR
// static inline ufract16_t FOC_GetVa(const FOC_T * p_foc) { return _FOC_VOfVDuty(p_foc, p_foc->Va); }
// static inline ufract16_t FOC_GetVb(const FOC_T * p_foc) { return _FOC_VOfVDuty(p_foc, p_foc->Vb); }
// static inline ufract16_t FOC_GetVc(const FOC_T * p_foc) { return _FOC_VOfVDuty(p_foc, p_foc->Vc); }

// static inline ufract16_t FOC_GetValpha(const FOC_T * p_foc) { return _FOC_VOfVDuty(p_foc, p_foc->Valpha); }
// static inline ufract16_t FOC_GetVMagnitude(const FOC_T * p_foc) { return fract16_vector_magnitude(FOC_GetValpha(p_foc), FOC_GetVbeta(p_foc)); }

// static inline void _FOC_SetVBemfA(FOC_T * p_foc, int32_t vBusInv_fract32, ufract16_t va) { p_foc->Va = _FOC_VDutyOfV_Inv32(vBusInv_fract32, va); }
// static inline void _FOC_SetVBemfB(FOC_T * p_foc, int32_t vBusInv_fract32, ufract16_t vb) { p_foc->Vb = _FOC_VDutyOfV_Inv32(vBusInv_fract32, vb); }
// static inline void _FOC_SetVBemfC(FOC_T * p_foc, int32_t vBusInv_fract32, ufract16_t vc) { p_foc->Vc = _FOC_VDutyOfV_Inv32(vBusInv_fract32, vc); }
#else
static inline ufract16_t FOC_GetVa(const FOC_T * p_foc) { return p_foc->Va; }
static inline ufract16_t FOC_GetVb(const FOC_T * p_foc) { return p_foc->Vb; }
static inline ufract16_t FOC_GetVc(const FOC_T * p_foc) { return p_foc->Vc; }
#endif

/* VDuty */
static inline ufract16_t FOC_GetDutyA(const FOC_T * p_foc) { return p_foc->DutyA; }
static inline ufract16_t FOC_GetDutyB(const FOC_T * p_foc) { return p_foc->DutyB; }
static inline ufract16_t FOC_GetDutyC(const FOC_T * p_foc) { return p_foc->DutyC; }

/******************************************************************************/
/*
    VBemf
*/
/******************************************************************************/
/* Capture */
// static inline void FOC_SetVBemfA(FOC_T * p_foc, ufract16_t va) { p_foc->Va = va; }
// static inline void FOC_SetVBemfB(FOC_T * p_foc, ufract16_t vb) { p_foc->Vb = vb; }
// static inline void FOC_SetVBemfC(FOC_T * p_foc, ufract16_t vc) { p_foc->Vc = vc; }

// static inline void FOC_ProcVBemfClarkePark(FOC_T * p_foc)
// {
//     // foc_clarke(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Va, p_foc->Vb, p_foc->Vc);
//     // foc_park_vector(&p_foc->Vd, &p_foc->Vq, p_foc->Valpha, p_foc->Vbeta, p_foc->Sine, p_foc->Cosine);
//     foc_clarke_park(&p_foc->Vd, &p_foc->Vq, p_foc->Va, p_foc->Vb, p_foc->Vc, p_foc->Sine, p_foc->Cosine);
// }

static inline void FOC_ProcVBemfClarkePark(FOC_T * p_foc, fract16_t va, fract16_t vb, fract16_t vc)
{
    foc_clarke_park(&p_foc->Vd, &p_foc->Vq, va, vb, vc, p_foc->Sine, p_foc->Cosine);
}

// static inline ufract16_t FOC_GetVBemfA(const FOC_T * p_foc) { return p_foc->Va; }
// static inline ufract16_t FOC_GetVBemfB(const FOC_T * p_foc) { return p_foc->Vb; }
// static inline ufract16_t FOC_GetVBemfC(const FOC_T * p_foc) { return p_foc->Vc; }

/******************************************************************************/
/*!
    Query
*/
/******************************************************************************/
/*
    Run-time derived
*/
static inline bool FOC_IsMotoring(const FOC_T * p_foc) { return (math_sign(p_foc->Vq) * math_sign(p_foc->Iq) == 1); }
/* p_foc->Vq != 0 */
static inline bool FOC_IsGenerating(const FOC_T * p_foc) { return (math_sign(p_foc->Vq) * math_sign(p_foc->Iq) == -1); }


/* [0:32767] <=> [0:1]AnalogRefMax */
static inline ufract16_t FOC_GetIMagnitude(const FOC_T * p_foc)     { return fract16_vector_magnitude(p_foc->Ialpha, p_foc->Ibeta); }
static inline ufract16_t FOC_GetVMagnitude(const FOC_T * p_foc)     { return fract16_vector_magnitude(p_foc->Valpha, p_foc->Vbeta); }
static inline ufract16_t FOC_GetIMagnitude_Idq(const FOC_T * p_foc) { return fract16_vector_magnitude(p_foc->Id, p_foc->Iq); }

/* [-32768:32767] <=> [-1:1]AnalogRefMax */
static inline fract16_t FOC_GetIPhase(const FOC_T * p_foc) { return FOC_GetIMagnitude(p_foc) * math_sign(p_foc->Iq); }
static inline fract16_t FOC_GetVPhase(const FOC_T * p_foc) { return FOC_GetVMagnitude(p_foc) * math_sign(p_foc->Vq); }

/*
    Unit calculations
    Optionally handle by host
*/
/* Active power (torque-producing) */
static inline accum32_t FOC_GetActivePower(const FOC_T * p_foc) { return fract16_mul(p_foc->Iq, p_foc->Vq) * 3 / 2; }
/* Reactive power (magnetizing) */
static inline accum32_t FOC_GetReactivePower(const FOC_T * p_foc) { return fract16_mul(p_foc->Id, p_foc->Vd) * 3 / 2; }
/* Total power (active + reactive) */ /* [0:49152] <=> [0:1.5F] */
static inline accum32_t FOC_GetPower(const FOC_T * p_foc) { return (fract16_mul(p_foc->Id, p_foc->Vd) + fract16_mul(p_foc->Iq, p_foc->Vq)) * 3 / 2; }

static inline ufract16_t FOC_GetPowerFactor(const FOC_T * p_foc) { return fract16_div_sat(FOC_GetActivePower(p_foc), FOC_GetPower(p_foc)); }
// static inline ufract16_t FOC_GetIBus(const FOC_T * p_foc, ufract16_t vBus_fract16) { return fract16_div_sat(FOC_GetPower(p_foc), vBus_fract16); }

static inline fract16_t FOC_GetId(const FOC_T * p_foc) { return p_foc->Id; }
static inline fract16_t FOC_GetIq(const FOC_T * p_foc) { return p_foc->Iq; }
static inline fract16_t FOC_GetVd(const FOC_T * p_foc) { return p_foc->Vd; }
static inline fract16_t FOC_GetVq(const FOC_T * p_foc) { return p_foc->Vq; }

// static inline fract16_t FOC_GetIa(const FOC_T * p_foc) { return p_foc->Ia; }
// static inline fract16_t FOC_GetIb(const FOC_T * p_foc) { return p_foc->Ib; }
// static inline fract16_t FOC_GetIc(const FOC_T * p_foc) { return p_foc->Ic; }
// static inline fract16_t FOC_GetIalpha(const FOC_T * p_foc) { return p_foc->Ialpha; }
// static inline fract16_t FOC_GetIbeta(const FOC_T * p_foc) { return p_foc->Ibeta; }
// static inline fract16_t FOC_GetReqD(const FOC_T * p_foc) { return p_foc->ReqD; }
// static inline fract16_t FOC_GetReqQ(const FOC_T * p_foc) { return p_foc->ReqQ; }


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


/******************************************************************************/
/*!
*/
/******************************************************************************/

/* with Scaled DutyAlpha, DutyBeta */
/*
    Normalized by a factor of sqrt(3)/2,
        alpha 1/sqrt(3) => a = .5
        alphaDuty 1.15 => a = 1.0
*/
// static inline void FOC_ProcInvParkInvClarkeSvpwm(FOC_T * p_foc)
// {
//     foc_inv_park_vector(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
//     svpwm_midclamp(&p_foc->Va, &p_foc->Vb, &p_foc->Vc, _FOC_DutyOfV(p_foc, p_foc->Valpha), _FOC_DutyOfV(p_foc, p_foc->Vbeta));
// }



/* Estimate efficiency based on I²R losses */
// static inline ufract16_t FOC_GetEfficiency(const FOC_T * p_foc, fract16_t rPhase)
// {
//     accum32_t activePower = FOC_GetActivePower(p_foc);
//     if (activePower <= 0) return 0;

//     ufract16_t iMag = FOC_GetIMagnitude(p_foc);
//     accum32_t resistiveLoss = fract16_mul(fract16_mul(iMag, iMag), rPhase) * 3;

//     return (activePower << 15) / (activePower + resistiveLoss);
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

// static inline void FOC_AnalyzeCurrentHarmonics(const FOC_T * p_foc, uint16_t * p_thd)
// {
//     /* Total Harmonic Distortion estimation */
//     ufract16_t fundamental = FOC_GetIMagnitude(p_foc);

//     /* Estimate harmonics from current ripple */
//     fract16_t ripple_a = p_foc->Ia - (p_foc->Ialpha * FRACT16_COS_0 + p_foc->Ibeta * FRACT16_SIN_0);
//     /* Add similar calculations for other phases */

//     if (fundamental > 0)
//     {
//         *p_thd = (abs(ripple_a) << 15) / fundamental;
//     }
// }



/* Direct DQ0 Transform - replaces Clarke + Park */
// static inline void FOC_ProcDQ0Transform(FOC_T * p_foc)
// {
//     foc_dq_transform(&p_foc->Id, &p_foc->Iq, p_foc->Ia, p_foc->Ib, p_foc->Ic, p_foc->Sine, p_foc->Cosine);
// }

// /* Direct DQ0 Transform for two-phase sensing (assumes Ic = -(Ia + Ib)) */
// static inline void FOC_ProcDQ0Transform_AB(FOC_T * p_foc)
// {
//     fract16_t ic = -(p_foc->Ia + p_foc->Ib);  /* Calculate third phase */
//     foc_dq_transform(&p_foc->Id, &p_foc->Iq, p_foc->Ia, p_foc->Ib, ic, p_foc->Sine, p_foc->Cosine);
// }

// /* Inverse DQ0 Transform - replaces Inverse Park + Inverse Clarke */
// static inline void FOC_ProcInvDQ0Transform(FOC_T * p_foc)
// {
//     foc_inv_dq_transform(&p_foc->Va, &p_foc->Vb, &p_foc->Vc, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
// }

// /* Updated output processing using direct DQ0 transform */
// static inline void FOC_ProcOutputV_VBusInv_DQ0(FOC_T * p_foc, uint32_t vBusInv_fract32)
// {
//     FOC_ProcInvDQ0Transform(p_foc);
//     FOC_ProcSvpwm(p_foc, vBusInv_fract32);
// }

// static inline void FOC_ProcOutputV_VBus_DQ0(FOC_T * p_foc, ufract16_t vBus_fract16)
// {
//     FOC_ProcOutputV_VBusInv_DQ0(p_foc, ((uint32_t)FRACT16_MAX * 65536U / vBus_fract16));
// }

// /* VBemf processing using direct DQ0 transform */
// static inline void FOC_ProcVBemfDQ0Transform(FOC_T * p_foc)
// {
//     foc_dq_transform(&p_foc->Vd, &p_foc->Vq, p_foc->Va, p_foc->Vb, p_foc->Vc, p_foc->Sine, p_foc->Cosine);
// }
