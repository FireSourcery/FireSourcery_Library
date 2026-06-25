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
#include "Math/Fixed/fract16.h"
#include "Math/PID/PID.h"


#if !defined(FOC_DECOUPLE_ENABLE) && !defined(FOC_DECOUPLE_DISABLE)
#define FOC_DECOUPLE_DISABLE
#endif

#if !defined(FOC_CLARKE_AB) && !defined(FOC_CLARKE_ABC)
#define FOC_CLARKE_ABC
#endif

// move to caller
#if !defined(FOC_V_MATCH_BEMF) && !defined(FOC_V_MATCH_SPEED)
#define FOC_V_MATCH_BEMF
#endif

/*
    Runtime
*/
typedef struct
{
    accum32_t OmegaLd;
    accum32_t OmegaLq;
    accum32_t OmegaPsi;
}
FOC_ElectricalSpeed_T;

/*
    Config
*/
typedef struct
{
    // fract16_t IsMax;     /* Current circle radius */
    ufract16_t IdLimit;     /* [0:32767] max demagnetizing I magnitude; 0 = FW disabled */
    ufract16_t IdGain;      /* Field weakening integrator gain per control cycle */
}
FOC_FieldWeakeningTuning_T;

/* same shape for SI and PU  */
typedef struct
{
    uint32_t Ld;
    uint32_t Lq;
    uint32_t Rs;
    uint32_t Psi;
}
FOC_Electrical_T;

/*
    Nvm Storage. caller owns Config nvm mapping
*/
typedef struct
{
    // PID_Config_T PidIqConfig;
    // PID_Config_T PidIdConfig;
    FOC_FieldWeakeningTuning_T FieldWeakening;
    FOC_Electrical_T Electrical; /* Electrical parameters in control PU */
    // ufract16_t Modulation;
}
FOC_Config_T;

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

    /* Intermediate values */
    fract16_t Ialpha, Ibeta;
    fract16_t Valpha, Vbeta;

    /*
        Inner current loop — pair has no semantic life outside the d-q frame.
        Inputs (Iq/Id) and outputs (Vq/Vd) live in this same object.
    */
    PID_T PidIq;
    PID_T PidId;

    /*
        Runtime V parameters directly visible in the FOC object, rather than passed per call.
    */
    // ufract16_t Modulation;
    interval_t VLimit;
    // fract16_t VqLimit; a single value encodes direction and magnitude. derives cw and ccw
    fract16_t VWindow;

    FOC_ElectricalSpeed_T ElectricalSpeed; /*  Cache on speed loop for feedforward and decoupling */
    fract16_t IdFw;   /* field weakening d-axis integrator state */

    FOC_Config_T Config;
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
// static inline void FOC_ProcClarkePark(FOC_T * p_foc, fract16_t ia, fract16_t ib, fract16_t ic)
// {
//     struct foc_dq i = foc_clarke_park(ia, ib, ic, p_foc->Sine, p_foc->Cosine);
//     p_foc->Id = i.d;
//     p_foc->Iq = i.q;
// }

static inline void FOC_ProcClarkePark(FOC_T * p_foc, fract16_t ia, fract16_t ib, fract16_t ic)
{
    struct foc_alphabeta iab = foc_clarke(ia, ib, ic);
    struct foc_dq idq = foc_park_vector(iab.alpha, iab.beta, p_foc->Sine, p_foc->Cosine);
    p_foc->Ialpha = iab.alpha;
    p_foc->Ibeta = iab.beta;
    p_foc->Id = idq.d;
    p_foc->Iq = idq.q;
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

/* vPhaseLimit as modulation [0:1/sqrt3] */
static inline bool _FOC_ProcVCircle(FOC_T * p_foc, ufract16_t vCircle, int32_t vdReq, int32_t vqReq)
{
    struct foc_dq vdq = foc_circle_limit_ff(vdReq, vqReq, vCircle);
    p_foc->Vd = vdq.d;
    p_foc->Vq = vdq.q;
    return ((accum32_t)p_foc->Vd != vdReq) || ((accum32_t)p_foc->Vq != vqReq);
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


// static inline void FOC_ProcInvClarkePark(FOC_T * p_foc)
// {
//     struct foc_abc v = foc_inv_clarke_park(p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
//     p_foc->Va = v.a;
//     p_foc->Vb = v.b;
//     p_foc->Vc = v.c;
// }

static inline void FOC_ProcInvClarkePark(FOC_T * p_foc)
{
    struct foc_alphabeta vab = foc_inv_park_vector(p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
    struct foc_abc v = foc_inv_clarke(vab.alpha, vab.beta);
    p_foc->Valpha = vab.alpha;
    p_foc->Vbeta = vab.beta;
    p_foc->Va = v.a;
    p_foc->Vb = v.b;
    p_foc->Vc = v.c;
}

/******************************************************************************/
/*
    VBemf of Freewheel
*/
/******************************************************************************/
static inline void FOC_ProcVBemfClarkePark(FOC_T * p_foc, fract16_t va, fract16_t vb, fract16_t vc)
{
    struct foc_dq v = foc_clarke_park(va, vb, vc, p_foc->Sine, p_foc->Cosine);
    p_foc->Vd = v.d;
    p_foc->Vq = v.q;
}

// static inline void FOC_ProcVBemf_Speed(FOC_T * p_foc, accum32_t speed)
// {
//     p_foc->ElectricalSpeed.OmegaPsi = fract16_sat(accum32_mul(p_foc->Config.Electrical.Psi, speed));
//     p_foc->Vd = 0;
//     p_foc->Vq = p_foc->ElectricalSpeed.OmegaPsi;
//     // FOC_ProcInvClarkePark(p_foc);
// }

/******************************************************************************/
/*!
    V Policy config
*/
/******************************************************************************/
/*
    Dynamic limits
    dynamic overlay to intersect with base
*/
/*
    window centered on the BEMF estimate:
    applied Vq must stay near omega_psi, not just on the same side
    useful to limit load angle, soften braking, or keep a no-plug brake inside a bounded slip window
*/
static inline interval_t _FOC_VBemfWindow(accum32_t omega_psi, int32_t window) { return interval_symmetric(omega_psi, window); }
static inline interval_t FOC_VBemfWindow(const FOC_T * p_foc, accum32_t omega_psi) { return interval_symmetric(omega_psi, p_foc->VWindow); }
static inline void FOC_SetVWindow(FOC_T * p_foc, int32_t window) { p_foc->VWindow = window; }


/******************************************************************************/
/*!
    Direct V control
*/
/******************************************************************************/
static inline bool FOC_ProcVControl(FOC_T * p_foc, ufract16_t vCircle, fract16_t vdReq, fract16_t vqReq)
{
    int32_t vq = interval_clamp(p_foc->VLimit, vqReq);
    return _FOC_ProcVCircle(p_foc, vCircle, vdReq, vq);
}

/*
    e.g. OpenLoop Align
*/
static inline void FOC_SetAngleV(FOC_T * p_foc, angle16_t theta, fract16_t vd, fract16_t vq)
{
    FOC_SetTheta(p_foc, theta);
    p_foc->Vd = vd;
    p_foc->Vq = vq;
    FOC_ProcInvClarkePark(p_foc);
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
static inline void FOC_ProcIFeedback_Base(FOC_T * p_foc, ufract16_t vBus, int16_t idReq, int16_t iqReq)
{
    p_foc->Vd = PID_ProcPI(&p_foc->PidId, p_foc->Id, idReq);
    ufract16_t vqCircleLimit = foc_vq_circle_limit(fract16_mul(vBus, FRACT16_1_DIV_2), p_foc->Vd);
    interval_t vqBand = interval_intersect(interval_symmetric(0, vqCircleLimit), p_foc->VLimit);
    PID_CaptureOutputLimits(&p_foc->PidIq, vqBand.low, vqBand.high);
    p_foc->Vq = PID_ProcPI(&p_foc->PidIq, p_foc->Iq, iqReq);
}

/* Post-proc circle limit — windup correction after both PIDs run. */
/* no sqrt during common case  */
/* the combine output state can still grow outside of circle limit. limit after proc may still have windup. propagate if limited. */
// static inline void FOC_ProcIFeedback_BackLimit(FOC_T * p_foc, ufract16_t vBus, int16_t idReq, int16_t iqReq)
// {
//     p_foc->Vd = PID_ProcPI(&p_foc->PidId, p_foc->Id, idReq);
//     p_foc->Vq = PID_ProcPI(&p_foc->PidIq, p_foc->Iq, iqReq);
    // if (FOC_ProcVectorLimit(p_foc, vBus)) { _PID_SetOutputState(&p_foc->PidIq, p_foc->Vq); }  // immediately snaps integral
// }


/******************************************************************************/
/*
    Cross-coupling decoupling feedforward, limit-first ordering.
        Vd_ff = -omega_e * Lq * Iq
        Vq_ff = +omega_e * Ld * Id + omega_e * psi_f
*/
/******************************************************************************/
/*
    accept the wide multiply by default
    Only M0 benefits from micro-optimizing.
*/
static void FOC_CaptureSpeed(FOC_T * p_foc, accum32_t speed)
{
    p_foc->ElectricalSpeed.OmegaLd = fract16_sat(accum32_mul(p_foc->Config.Electrical.Ld, speed));
    p_foc->ElectricalSpeed.OmegaLq = fract16_sat(accum32_mul(p_foc->Config.Electrical.Lq, speed));
    p_foc->ElectricalSpeed.OmegaPsi = fract16_sat(accum32_mul(p_foc->Config.Electrical.Psi, speed));
}

static inline accum32_t FOC_VdFeedforward(const FOC_T * p_foc) { return foc_vd_ff(p_foc->ElectricalSpeed.OmegaLq, p_foc->Iq); }
static inline accum32_t FOC_VqFeedforward(const FOC_T * p_foc) { return foc_vq_ff(p_foc->ElectricalSpeed.OmegaLd, p_foc->ElectricalSpeed.OmegaPsi, p_foc->Id); }

// static void FOC_CaptureSpeed(FOC_T * p_foc, accum32_t speed)
// {
//     p_foc->ElectricalSpeed.OmegaLd = (accum32_mul(p_foc->Config.Electrical.Ld, speed));
//     p_foc->ElectricalSpeed.OmegaLq = (accum32_mul(p_foc->Config.Electrical.Lq, speed));
//     p_foc->ElectricalSpeed.OmegaPsi = (accum32_mul(p_foc->Config.Electrical.Psi, speed));
// }
// static inline accum32_t FOC_VdFeedforward(const FOC_T * p_foc) { return foc_vd_ff_wide(p_foc->ElectricalSpeed.OmegaLq, p_foc->Iq); }
// static inline accum32_t FOC_VqFeedforward(const FOC_T * p_foc) { return foc_vq_ff_wide(p_foc->ElectricalSpeed.OmegaLd, p_foc->ElectricalSpeed.OmegaPsi, p_foc->Id); }

/*
    Vd_cmd = Vd_PI + Vd_ff,
        Vd_PI ∈ [−vPhaseLim − Vd_ff, vPhaseLim − Vd_ff]
        Vd_cmd ∈ [−vPhaseLim, vPhaseLim]

    Vq_PI ∈ [−Vcircle − Vq_ff, Vcircle − Vq_ff]
*/
// static inline void _FOC_ProcIFeedback_Decouple(FOC_T * p_foc, ufract16_t vPhaseLimit, accum32_t vd_ff, accum32_t vq_ff, int16_t idReq, int16_t iqReq)
static inline void FOC_ProcIFeedback_Decouple(FOC_T * p_foc, ufract16_t vBus, int16_t idReq, int16_t iqReq)
{
    fract16_t vPhaseLimit = fract16_mul(vBus, FRACT16_1_DIV_2);
    accum32_t vd_ff = FOC_VdFeedforward(p_foc); /* May exceeed INT16_MAX range */
    accum32_t vq_ff = FOC_VqFeedforward(p_foc);

    PID_CaptureOutputLimits(&p_foc->PidId, fract16_sat(-vPhaseLimit - vd_ff), fract16_sat(vPhaseLimit - vd_ff)); /* vPhaseLimit < (INT16_MAX / 2) */
    p_foc->Vd = math_clamp(PID_ProcPI(&p_foc->PidId, p_foc->Id, idReq) + vd_ff, -vPhaseLimit, vPhaseLimit);

    ufract16_t vqCircleLimit = foc_vq_circle_limit(vPhaseLimit, p_foc->Vd);  /* VqCircleLimit < vPhaseLimit < (INT16_MAX / 2) */
    interval_t vqBand = interval_intersect(interval_symmetric(0, vqCircleLimit), p_foc->VLimit);  /* policy band */
    // interval_t vqLimit = interval_intersect(vqBand, FOC_VBemfWindow(p_foc, p_foc->ElectricalSpeed.OmegaPsi));
    PID_CaptureOutputLimits(&p_foc->PidIq, fract16_sat(vqBand.low - vq_ff), fract16_sat(vqBand.high - vq_ff));
    p_foc->Vq = math_clamp(PID_ProcPI(&p_foc->PidIq, p_foc->Iq, iqReq) + vq_ff, vqBand.low, vqBand.high); /* lands back in vqBand */
}

static inline void FOC_ProcIFeedback(FOC_T * p_foc, ufract16_t vBus, int16_t idReq, int16_t iqReq)
{
#if defined(FOC_DECOUPLE_ENABLE)
    FOC_ProcIFeedback_Decouple(p_foc, vBus, idReq, iqReq);
#else
    FOC_ProcIFeedback_Base(p_foc, vBus, idReq, iqReq);
#endif
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
// static inline void _FOC_MatchIVState(FOC_T * p_foc, int16_t vd, int16_t vq)
// {
//     p_foc->Vd = vd;
//     p_foc->Vq = vq;
// #if defined(FOC_DECOUPLE_ENABLE)
//     PID_SetOutputState(&p_foc->PidId, fract16_sat(vd - FOC_VdFeedforward(p_foc)));
//     PID_SetOutputState(&p_foc->PidIq, fract16_sat(vq - FOC_VqFeedforward(p_foc)));
// #else
//     PID_SetOutputState(&p_foc->PidId, vd);
//     PID_SetOutputState(&p_foc->PidIq, vq);
// #endif
// }

/* Seed integrators so the next ProcIFeedback reproduces (vd, vq) at the present current. */
/* From an actively-driven state (loaded / field-weakening) and FOC_ProcVBemfClarkePark capture */
static inline void FOC_MatchIVState(FOC_T * p_foc)
{
#if defined(FOC_DECOUPLE_ENABLE)
    /* Vq_measured = PI_out + Vq_ff */
    PID_SetOutputState(&p_foc->PidId, fract16_sat(p_foc->Vd - FOC_VdFeedforward(p_foc)));
    PID_SetOutputState(&p_foc->PidIq, fract16_sat(p_foc->Vq - FOC_VqFeedforward(p_foc)));
#else
    PID_SetOutputState(&p_foc->PidId, p_foc->Vd);
    PID_SetOutputState(&p_foc->PidIq, p_foc->Vq);
#endif
}

static inline void FOC_MatchIVEstimate(FOC_T * p_foc)
{
    p_foc->Vd = 0;
    p_foc->Vq = p_foc->ElectricalSpeed.OmegaPsi;    /* Caller handle FOC_CaptureSpeed */
    FOC_MatchIVState(p_foc);
}

/* From open terminals, freewheel-no-sense, (I≈0, ≤ base speed) */
/* Id, Iq cleared on freewheel entry */
/* Outputlimits updated prior to calling */
static inline void FOC_MatchIVFreewheel(FOC_T * p_foc)
{
#if defined(FOC_V_MATCH_BEMF)
    if (p_foc->Vq == 0) { FOC_MatchIVEstimate(p_foc); } /* if no Vq, fallback to speed estimate */
    else { FOC_MatchIVState(p_foc); } /* FOC_ProcVBemfClarkePark */
#elif defined(FOC_V_MATCH_SPEED)
    FOC_MatchIVEstimate(p_foc);
#endif
}




/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    Mostly static base, update on direction change
    set VLimit directly
*/
/* Symmetric */
static inline void FOC_SetVSymmetric(FOC_T * p_foc, uint16_t vq) { p_foc->VLimit = interval_symmetric(0, vq); }

/* Half-plane anti-plugging */
static inline void FOC_SetVAntiPlugging(FOC_T * p_foc, sign_t sign, uint16_t vq) { p_foc->VLimit = interval_of_sign(sign, vq); }

static inline void FOC_SetVLimits(FOC_T * p_foc, sign_t direction, uint16_t vPhaseLimit)
{
    interval_t v = interval_of_sign(direction, vPhaseLimit);
    FOC_SetVAntiPlugging(p_foc, direction, vPhaseLimit);
    PID_SetOutputLimits(&p_foc->PidIq, v.low, v.high);
    PID_SetOutputLimits(&p_foc->PidId, 0 - vPhaseLimit, vPhaseLimit);
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

/* [0:32767] */
static inline accum32_t _FOC_GetActivePower(const FOC_T * p_foc) { return fract16_mul(p_foc->Vd, p_foc->Id) + fract16_mul(p_foc->Vq, p_foc->Iq); }
static inline accum32_t _FOC_GetReactivePower(const FOC_T * p_foc) { return (fract16_mul(p_foc->Vq, p_foc->Id) - fract16_mul(p_foc->Vd, p_foc->Iq)); }
/* keep sign within fract16 */
static inline accum32_t _FOC_GetIBus(const FOC_T * p_foc, ufract16_t vBus_fract16) { return fract16_div(_FOC_GetActivePower(p_foc), vBus_fract16); }

/* [0:49150] */
static inline accum32_t FOC_GetActivePower(const FOC_T * p_foc) { return _FOC_GetActivePower(p_foc) * 3 / 2; }
static inline accum32_t FOC_GetReactivePower(const FOC_T * p_foc) { return _FOC_GetReactivePower(p_foc) * 3 / 2; }
static inline accum32_t FOC_GetApparentPower(const FOC_T * p_foc) { return fract16_mul(FOC_GetIMagnitude(p_foc), FOC_GetVMagnitude(p_foc)) * 3 / 2; }
static inline accum32_t FOC_GetPowerFactor(const FOC_T * p_foc) { return fract16_div(_FOC_GetActivePower(p_foc), fract16_mul(FOC_GetIMagnitude(p_foc), FOC_GetVMagnitude(p_foc))); }
static inline accum32_t FOC_GetIBus(const FOC_T * p_foc, ufract16_t vBus_fract16) { return fract16_div(_FOC_GetActivePower(p_foc), vBus_fract16) * 3 / 2; }


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
static inline bool _FOC_IsMotoring(const FOC_T * p_foc) { return (p_foc->Vq * p_foc->Iq > 0); }
static inline bool _FOC_IsGenerating(const FOC_T * p_foc) { return (p_foc->Vq * p_foc->Iq < 0); }

static inline bool FOC_IsMotoring(const FOC_T * p_foc, int32_t speed) { return (p_foc->Iq * (int32_t)speed > 0); }
static inline bool FOC_IsGenerating(const FOC_T * p_foc, int32_t speed) { return (p_foc->Iq * (int32_t)speed < 0); }

/*
    Plugging: applied Vq opposes back-EMF direction (speed sign)
    Speed sign gives true rotor direction regardless of Vq/Iq
*/
/* Vq and Speed have opposite signs - applied voltage opposes rotor back-EMF */
static inline bool FOC_IsPlugging(const FOC_T * p_foc, int32_t speed) { return (p_foc->Vq * (int32_t)speed < 0); }
/*
    Regen: Iq opposes Vq direction (generating), but Vq aligns with speed
*/
/* Generating  AND Vq aligns with speed - true regeneration */
static inline bool FOC_IsRegen(const FOC_T * p_foc, int32_t speed) { return (FOC_IsGenerating(p_foc, speed) && !FOC_IsPlugging(p_foc, speed)); }

static inline bool _FOC_IsGeneratingReq(int32_t speed, int16_t iqReq) { return (speed * iqReq) < 0; }


/******************************************************************************/
/*
    Fw
*/
/******************************************************************************/
/*
    Id_fw = -(λ_pm / Ld) + sqrt((Vs_max / (ωe·Ld))² - Iq²)
    Id_fw = (Vs_max/ωe - λ_pm) / Ld

    Voltage-feedback FW integrator. Returns Id setpoint in [-(IdFwLimit), 0].
    Id_fw += Ki · (Vbus/2 - √(Vd² + Vq²))   (integrator, Id ≤ 0)
*/
static inline fract16_t FOC_ProcIdFieldWeakening(FOC_T * p_foc, fract16_t vBus)
{
    int32_t error = fract16_mul(vBus, FRACT16_1_DIV_2) - FOC_GetVMagnitude(p_foc);
    p_foc->IdFw = math_clamp(p_foc->IdFw + fract16_mul(error, p_foc->Config.FieldWeakening.IdGain), -p_foc->Config.FieldWeakening.IdLimit, 0);
    return p_foc->IdFw;
}


static inline void FOC_DisableFieldWeakening(FOC_T * p_foc) { p_foc->Config.FieldWeakening.IdLimit = 0; }

// static inline bool FOC_IsFieldWeakeningEnabled(FOC_T * p_foc) { return (p_foc->Config.FieldWeakening.IdLimit > 0); }


// static inline void FOC_ProcIFeedback_FieldWeakening(FOC_T * p_foc, ufract16_t vBus, sign_t direction, int16_t iqReq)
// {
//     FOC_ProcIFeedback_Decouple(p_foc, vBus, direction, FOC_ProcIdFieldWeakening(p_foc, vBus), iqReq);
// }

// static inline void FOC_ProcITorque(FOC_T * p_foc, ufract16_t vBus, sign_t direction, int16_t iqReq)
// {
//     FOC_ProcIFeedback(p_foc, vBus, direction, 0, iqReq);
// }


// static inline uint16_t FOC_GetLqInv(FOC_T * p_foc)
// {
//     /* 1 / (Lq · I_max · Fs / V_max) — EEMF integrator gain */
//     return fract16_div(FRACT16_1, fract16_mul(p_foc->Config.Electrical.Lq, FRACT16_1_DIV_SQRT3));
// }

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    alternatively if runtime functions are implemented with collaborator pattern, caller must handled var config segment
*/
static void FOC_InitElectrical(FOC_T * p_foc, const FOC_Electrical_T * p_electrical)
{
    p_foc->Config.Electrical = *p_electrical;
}

static void FOC_InitFieldWeakening(FOC_T * p_foc, const FOC_FieldWeakeningTuning_T * p_fwTuning)
{
    p_foc->Config.FieldWeakening = *p_fwTuning;
}

static void FOC_Init(FOC_T * p_foc, const FOC_Config_T * p_config)
{
    p_foc->Config = *p_config;
    /*     PID_InitFrom(&p_motor->Foc.PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->Foc.PidId, &p_motor->Config.PidI); */
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
    // FOC_CaptureSpeed(p_foc, 0);
}

static void FOC_ClearOutputState(FOC_T * p_foc)
{
    p_foc->Vd = 0;
    p_foc->Vq = 0;
    p_foc->Va = 0;
    p_foc->Vb = 0;
    p_foc->Vc = 0;
}

static inline void FOC_ResetFeedbackState(FOC_T * p_foc)
{
    PID_Reset(&p_foc->PidIq);
    PID_Reset(&p_foc->PidId);
    PID_CaptureOutputLimits(&p_foc->PidId, -math_abs(p_foc->VLimit.high), math_abs(p_foc->VLimit.high));
    PID_CaptureOutputLimits(&p_foc->PidIq, p_foc->VLimit.low, p_foc->VLimit.high);
    p_foc->IdFw = 0;
    // p_foc->ElectricalSpeed.OmegaLd = 0;
    // p_foc->ElectricalSpeed.OmegaLq = 0;
    // p_foc->ElectricalSpeed.OmegaPsi = 0;
}

static inline void FOC_Reset(FOC_T * p_foc)
{


}


static inline bool FOC_Config_IsFwEnabled(const FOC_Config_T * p_config) { return (p_config->FieldWeakening.IdLimit > 0U); }

static inline int FOC_Config_IsValid(const FOC_Config_T * p_config)
{

}


/******************************************************************************/
/*!
    Var id interface
*/
/******************************************************************************/
/*
    FOC State
    Read-Only, RealTime
*/
typedef enum Motor_Var_Foc
{
    MOTOR_VAR_FOC_ID,
    MOTOR_VAR_FOC_IQ,
    MOTOR_VAR_FOC_VD,
    MOTOR_VAR_FOC_VQ,
    MOTOR_VAR_FOC_VA, /* VOutput only */
    MOTOR_VAR_FOC_VB,
    MOTOR_VAR_FOC_VC,
    MOTOR_VAR_FOC_REQ_D,
    MOTOR_VAR_FOC_REQ_Q, /* Iq or Vq Req sign unadjusted */
    MOTOR_VAR_FOC_INTEGRAL_D,
    MOTOR_VAR_FOC_INTEGRAL_Q,
}
Motor_Var_Foc_T;

static int Foc_Var_Get(FOC_T * p_foc, Motor_Var_Foc_T varId)
{
    int value = 0;
    switch (varId)
    {
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


/*
    Call holds struct
*/
typedef enum FOC_ConfigId
{
    FOC_CONFIG_FW_ID_LIMIT,
    FOC_CONFIG_FW_ID_GAIN,
    FOC_CONFIG_ELECTRICAL_LD,
    FOC_CONFIG_ELECTRICAL_LQ,
    FOC_CONFIG_ELECTRICAL_RS,
    FOC_CONFIG_ELECTRICAL_PSI,
}
FOC_ConfigId_T;


static int FOC_Config_Get(const FOC_Config_T * p_config, FOC_ConfigId_T var)
{
    switch (var)
    {
        case FOC_CONFIG_FW_ID_LIMIT:    return p_config->FieldWeakening.IdLimit;
        case FOC_CONFIG_FW_ID_GAIN:     return p_config->FieldWeakening.IdGain;
        // temporarily
        case FOC_CONFIG_ELECTRICAL_LD:  return p_config->Electrical.Ld / 1000;
        case FOC_CONFIG_ELECTRICAL_LQ:  return p_config->Electrical.Lq / 1000;
        case FOC_CONFIG_ELECTRICAL_RS:  return p_config->Electrical.Rs;
        case FOC_CONFIG_ELECTRICAL_PSI: return p_config->Electrical.Psi;
        default: return 0;
    }
}

static void FOC_Config_Set(FOC_Config_T * p_config, FOC_ConfigId_T var, int value)
{
    switch (var)
    {
        case FOC_CONFIG_FW_ID_LIMIT:    p_config->FieldWeakening.IdLimit = value;        break;
        case FOC_CONFIG_FW_ID_GAIN:     p_config->FieldWeakening.IdGain = value;         break;
        case FOC_CONFIG_ELECTRICAL_LD:  p_config->Electrical.Ld = value * 1000;    break;
        case FOC_CONFIG_ELECTRICAL_LQ:  p_config->Electrical.Lq = value * 1000;    break;
        case FOC_CONFIG_ELECTRICAL_RS:  p_config->Electrical.Rs = value;            break;
        case FOC_CONFIG_ELECTRICAL_PSI: p_config->Electrical.Psi = value;           break;
        default: break;
    }
}

/* Optional runtime tuning */
// static int Foc_Var_SetTuning(FOC_T * p_foc,  Motor_Var_Foc_T varId, int value)
// {
//     int value = 0;
//     switch (varId)
//     {

//     }
//     return value;
// }
