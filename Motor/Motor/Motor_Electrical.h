#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Motor_Electrical.h
    @author FireSourcery
    @brief  Motor electrical model. Speed base shapes, electrical parameters, field weakening tuning.
*/
/******************************************************************************/
#include "Motor/Motor/Math/motor_electrical_math.h"
#include "Motor/Motor/Math/FOC.h"
#include "Math/Angle/angle_speed_math.h"
#include "Math/math_general.h"

#include "Motor/Motor/Motor_Clock.h"
#include "Motor/Motor/Phase_Input/Phase_Board.h"


/******************************************************************************/
/*
    [Motor_Electrical_T] Motor model. Canonical storage.
        F(V) = AngleFreqPerVolt · V     [angle16/s], V phase peak. 1 / ψ
        F(X) = AngleFreqPerOhm · X      [angle16/s], X = ω · L. 1 / Ld, 1 / Lq
        The rates are free of rpm, rad/s, Fs, and the board base.
        Rs [V_base / I_base].
    Kv, rpm, and µH are views at the user boundary.
*/
/******************************************************************************/
typedef struct
{
    uint8_t PolePairs;
    angle_freq_t AngleFreqPerVolt;  /* Electrical [angle16/s / V], phase peak. 1 / ψ */
    angle_freq_t AngleFreqPerOhmD;  /* Electrical [angle16/s · A/V]. 1 / Ld */
    angle_freq_t AngleFreqPerOhmQ;  /* Electrical [angle16/s · A/V]. 1 / Lq */
    uint32_t Rs;
}
Motor_Electrical_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    Set ψ_pu == v_pu/speed_pu = .5, where V_Base is inverter max with margin
    SpeedRated_pu = VNominal_pu, SpeedBase ~ ψ_pu
    Speed_pu = V_phase_pu * 2
    fw_max => V_Base, saturate, (e.g. 72 V Nominal, 94 V base, fw saturate at 1.30)
    SpeedBase [rpm] = Kv * V_Base
    SpeedRated [rpm] = Kv * VNominal

    ψ_pu = 1.0
    SpeedRated_pu = VNominal_Pu / √3
    SpeedBase [rpm] = Kv * V_Base * √3
    fw_max => V_Base * 1.73

    alternatively materialize the base speed and store as an additional parameter
    when SpeedBase = SpeedRated * 2 = Kv * V_Nominal * 2
    Ke = VNominal * 2
    ψ_pu = V_Nominal / V_Base = V_Nominal_pu
    fw_max => 2 * V_Nominal
*/
/*
    alteratively pid use separate base, ui use angle for invariant
*/
/* VBemf_Pu at the speed base, so ψ_pu at the speed base */
/* Speed_pu = 1.0 where phase peak BEMF = V_base / 2, the phase peak at VBus = V_base (M = 2). Equals ψ_pu at the speed base */
/* VPhaseMax_Pu */
/*
    When PWM is off (fault or freewheel), the body diodes rectify the line-to-line peak BEMF into the bus. That one quantity sets two limits:
    k = √3 makes speed_pu the line-to-line BEMF over V_MAX. Then:
    SpeedVLimit = VBus and SpeedRated = VNominal hold exactly for SVPWM.
    Cost: 0.2 bit, MOTOR_SPEED_BASE_V_PU = FRACT16_1_DIV_SQRT3, and VSpeed = fract16_mul(speed, FRACT16_1_DIV_SQRT3) instead of /2.
*/
/*
    V_MAX / 2, e.g. 94 V / 2 = 47 V => V_emf_ll = 0.866 · V_MAX = 81.4 V
    V_Rated / √3, e.g. 72 V / √3 ≈ 41.6 V
*/
#define MOTOR_SPEED_BASE_V_PU (FRACT16_SCALE / 2)



/*
    One base, three encodings: AngleFreq [angle16/s], AngleDt = AngleFreq / Fs [angle16/Ts], Rpm [mechanical]
*/
static inline angle_freq_t Motor_SpeedBase_AngleFreq(const Motor_Electrical_T * p_electrical) { return angle_freq_at_v(p_electrical->AngleFreqPerVolt, Phase_VMaxVolts(), MOTOR_SPEED_BASE_V_PU); }
// static inline angle_freq_t Motor_SpeedBase_AngleFreq(const Motor_Electrical_T * p_electrical) { return accum32_mul(p_electrical->AngleFreqPerVolt * Phase_VMaxVolts(), MOTOR_SPEED_BASE_V_PU); }
static inline angle_dt_t Motor_SpeedBase_AngleDt(const Motor_Electrical_T * p_electrical) { return angle_dt_of_angle_freq(MOTOR_CONTROL_FREQ, Motor_SpeedBase_AngleFreq(p_electrical)); }
static inline uint32_t Motor_SpeedBase_Rpm(const Motor_Electrical_T * p_electrical) { return mech_rpm_of_el_angle_freq(p_electrical->PolePairs, Motor_SpeedBase_AngleFreq(p_electrical)); }

/* Local Unit Conversion */
static inline accum32_t Motor_Speed_PuOfRpm(const Motor_Electrical_T * p_electrical, int16_t speed_rpm) { return speed_rpm * INT16_MAX / (int32_t)Motor_SpeedBase_Rpm(p_electrical); }
static inline int16_t Motor_Speed_RpmOfPu(const Motor_Electrical_T * p_electrical, accum32_t speed_pu) { return speed_pu * (int32_t)Motor_SpeedBase_Rpm(p_electrical) / 32768; }
// static inline int16_t Motor_Rated_AngleOfRpm(const Motor_Rated_T * p_rated, accum32_t speed_rpm) { return el_angle_dt_of_mech_rpm(MOTOR_CONTROL_FREQ, p_rated->PolePairs, speed_rpm); }
// static inline int16_t Motor_Rated_RpmOfAngle(const Motor_Rated_T * p_rated, accum32_t speed_angle16) { return mech_rpm_of_el_angle_dt(MOTOR_CONTROL_FREQ, p_rated->PolePairs, speed_angle16); }

/*

*/
/* ψ_tick = Fs·2^30 / (AngleFreqPerVolt·V_base), v_emf = Δθ·ψ_tick */
static inline uint32_t Motor_Electrical_Psi(const Motor_Electrical_T * p_electrical) { return psi_pu_of_angle_freq(angle_freq_nyquist(MOTOR_CONTROL_FREQ), Phase_VMaxVolts(), p_electrical->AngleFreqPerVolt); }


/******************************************************************************/
/*
    [Motor_Electrical_T] Ld, Lq, Rs
    Same shape in SI: Rs [µΩ]. PolePairs and the rates are free of the board base
*/
/******************************************************************************/
/* L_pu at a base [angle16/s] and the rate are reciprocal. A rate ≤ 0 is unset, and L_pu = 0 has no rate: both read 0 */
static inline uint32_t _Motor_Electrical_LPuOf(angle_freq_t base, angle_freq_t rate) { return (rate > 0) ? l_pu_of_angle_freq(base, Phase_VMaxVolts(), Phase_IMaxAmps(), rate) : 0U; }
static inline angle_freq_t _Motor_Electrical_RateOfLPu(angle_freq_t base, uint32_t l_pu) { return (l_pu != 0U) ? angle_freq_of_l_pu(base, Phase_VMaxVolts(), Phase_IMaxAmps(), l_pu) : 0; }

/*
    Tick base [V_base / I_base · Ts / π], identification in samples.
    Ls = (Ld + Lq) / 2, the αβ average for the observer.
*/
// todo: Ls as the per-phase inductance, the phase to neutral winding self-inductance in the abc frame
static inline uint32_t Motor_Electrical_LsTick(const Motor_Electrical_T * p_electrical)
{
    return ((uint64_t)_Motor_Electrical_LPuOf(angle_freq_nyquist(MOTOR_CONTROL_FREQ), p_electrical->AngleFreqPerOhmD) + _Motor_Electrical_LPuOf(angle_freq_nyquist(MOTOR_CONTROL_FREQ), p_electrical->AngleFreqPerOhmQ)) / 2U;
}

static inline void Motor_Electrical_SetLdqTick(Motor_Electrical_T * p_electrical, uint32_t ld_tick, uint32_t lq_tick)
{
    p_electrical->AngleFreqPerOhmD = _Motor_Electrical_RateOfLPu(angle_freq_nyquist(MOTOR_CONTROL_FREQ), ld_tick);
    p_electrical->AngleFreqPerOhmQ = _Motor_Electrical_RateOfLPu(angle_freq_nyquist(MOTOR_CONTROL_FREQ), lq_tick);
}

/*
    SI -> PU on the board base [Phase_Board_T].
*/
// static inline uint32_t Motor_Electrical_RsPu(const Motor_Electrical_T * p_si) { return rs_pu_of_ohm(Phase_VMaxVolts(), Phase_IMaxAmps(), p_si->Rs, 1000000UL); }

/* same shape for SI and PU  */
// static inline Motor_Electrical_T Motor_Electrical_PuOfSi(const Motor_Electrical_T * p_si)
// {
//     return (Motor_Electrical_T)
//     {
//         .PolePairs          = p_si->PolePairs,
//         .AngleFreqPerVolt   = p_si->AngleFreqPerVolt, /* or SI use Hz or rad/s */
//         .AngleFreqPerOhmD   = p_si->AngleFreqPerOhmD,
//         .AngleFreqPerOhmQ   = p_si->AngleFreqPerOhmQ,
//         .Rs                 = rs_pu_of_ohm(Phase_VMaxVolts(), Phase_IMaxAmps(), p_si->Rs, 1000000UL),
//     };
// }

/******************************************************************************/
/*
    FOC view, d-q frame at the speed base.
    Pure in the current base, no prior base to rebase from.
*/
/******************************************************************************/
// todo this moves to FOC
static inline FOC_Electrical_T Motor_Electrical_FocOf(const Motor_Electrical_T * p_electrical)
{
    return (FOC_Electrical_T)
    {
        .Ld     = _Motor_Electrical_LPuOf(Motor_SpeedBase_AngleFreq(p_electrical), p_electrical->AngleFreqPerOhmD),
        .Lq     = _Motor_Electrical_LPuOf(Motor_SpeedBase_AngleFreq(p_electrical), p_electrical->AngleFreqPerOhmQ),
        .Rs     = p_electrical->Rs,
        .Psi    = MOTOR_SPEED_BASE_V_PU,
    };
}

/* Ld, Lq, Rs from the FOC view. ψ is 1 / AngleFreqPerVolt, not stored */
static inline void Motor_Electrical_SetFoc(Motor_Electrical_T * p_electrical, const FOC_Electrical_T * p_foc)
{
    p_electrical->AngleFreqPerOhmD = _Motor_Electrical_RateOfLPu(Motor_SpeedBase_AngleFreq(p_electrical), p_foc->Ld);
    p_electrical->AngleFreqPerOhmQ = _Motor_Electrical_RateOfLPu(Motor_SpeedBase_AngleFreq(p_electrical), p_foc->Lq);
    p_electrical->Rs = p_foc->Rs;
}




// typedef struct
// {
//     // fract16_t IsMax;     /* Current circle radius */
//     ufract16_t IfwLimit;     /* [0:32767] max demagnetizing I magnitude; 0 = FW disabled */
//     ufract16_t IfwGain;      /* Field weakening integrator gain per control cycle */
// }
// Motor_FieldWeakeningTuning_T;

/******************************************************************************/
/*
    Field Weakening
    Id budget bounded by the board FW rating. Iq budget is the remaining component of the rated current vector.
*/
/******************************************************************************/
static inline ufract16_t Motor_FieldWeakening_IdLimit(const FOC_FieldWeakeningTuning_T * p_tuning) { return math_min(p_tuning->IdLimit, Phase_IRatedFw_Pu()); }
static inline ufract16_t Motor_FieldWeakening_IqLimit(const FOC_FieldWeakeningTuning_T * p_tuning) { return fract16_vector_component(Motor_FieldWeakening_IdLimit(p_tuning), Phase_IRatedPeak_Pu()); }

