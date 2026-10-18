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
#include "Model/Motor_Types.h"
#include "Motor/Motor/Math/motor_electrical_math.h"
#include "Math/Angle/angle_speed_math.h"
#include "Math/math_general.h"

#include "Motor/Motor/Motor_Clock.h"
#include "Motor/Motor/Phase_Input/Phase_Board.h"


/******************************************************************************/
/*
    Unit bases. The control algorithm holds none: per-unit values derive here, the composite assembles each algorithm's view
        Owns        speed base          Motor_SpeedBase_AngleFreq, from AngleFreqPerVolt
        Mediates    V_base, I_base      [Phase_Board_T]
                    Fs, turn base       MOTOR_CONTROL_FREQ [Motor_Clock]
*/
/******************************************************************************/

/******************************************************************************/
/*
    [Motor_Electrical_T] Motor model. Canonical storage.
    Stores ψ and L as reciprocal, s in the denominator, radians scaled to angle16.
    Independent of Fs, V_base, I_base.

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
    angle_freq_t AngleFreqPerVolt;  /* 1 / ψ  [angle16/s / V], phase peak */
    angle_freq_t AngleFreqPerOhmD;  /* 1 / Ld [angle16/s · A/V] */
    angle_freq_t AngleFreqPerOhmQ;  /* 1 / Lq [angle16/s · A/V] */
    uint32_t Rs;
}
Motor_Electrical_T;

/* The divisors set: PolePairs and the rates. Their conversions assume it, unguarded */
static inline bool Motor_Electrical_IsValid(const Motor_Electrical_T * p_electrical)
{
    return (p_electrical->PolePairs != 0U) && (p_electrical->AngleFreqPerVolt > 0) && (p_electrical->AngleFreqPerOhmD > 0) && (p_electrical->AngleFreqPerOhmQ > 0);
}

/******************************************************************************/
/*
    Non inverted [V/(rad/s)] form, independent of ω_base
    Turn base [V_base · Ts / π]/[<I_base>], identification in samples.
*/
/******************************************************************************/
/* ψ_turn = Fs·2^30 / (AngleFreqPerVolt·V_base), v_emf = Δθ·ψ_turn */
static inline uint32_t Motor_Psi_PuTurn(const Motor_Electrical_T * p_electrical) { return psi_pu_of_angle_freq(MOTOR_ANGLE_FREQ_MAX, Phase_VMaxVolts(), p_electrical->AngleFreqPerVolt); }
static inline void Motor_SetPsi_PuTurn(Motor_Electrical_T * p_electrical, uint32_t psi_turn) { p_electrical->AngleFreqPerVolt = angle_freq_of_psi_pu(MOTOR_ANGLE_FREQ_MAX, Phase_VMaxVolts(), psi_turn); }

static inline uint16_t Motor_Kv(const Motor_Electrical_T * p_electrical) { return kv_of_angle_freq(p_electrical->AngleFreqPerVolt, p_electrical->PolePairs); }
static inline void Motor_SetKv(Motor_Electrical_T * p_electrical, uint16_t kv) { p_electrical->AngleFreqPerVolt = angle_freq_of_kv(kv, p_electrical->PolePairs); }

/* L_turn = Fs · 2^30 · I_base / (AngleFreqPerOhm · V_base), v_L = Δθ·L·i_pu  */
static inline uint32_t Motor_Ld_PuTurn(const Motor_Electrical_T * p_electrical) { return l_pu_of_angle_freq(MOTOR_ANGLE_FREQ_MAX, Phase_VMaxVolts(), Phase_IMaxAmps(), p_electrical->AngleFreqPerOhmD); }
static inline uint32_t Motor_Lq_PuTurn(const Motor_Electrical_T * p_electrical) { return l_pu_of_angle_freq(MOTOR_ANGLE_FREQ_MAX, Phase_VMaxVolts(), Phase_IMaxAmps(), p_electrical->AngleFreqPerOhmQ); }

static inline void Motor_SetLd_PuTurn(Motor_Electrical_T * p_electrical, uint32_t ld_turn) { p_electrical->AngleFreqPerOhmD = angle_freq_of_l_pu(MOTOR_ANGLE_FREQ_MAX, Phase_VMaxVolts(), Phase_IMaxAmps(), ld_turn); }
static inline void Motor_SetLq_PuTurn(Motor_Electrical_T * p_electrical, uint32_t lq_turn) { p_electrical->AngleFreqPerOhmQ = angle_freq_of_l_pu(MOTOR_ANGLE_FREQ_MAX, Phase_VMaxVolts(), Phase_IMaxAmps(), lq_turn); }

/* Ls = (Ld + Lq) / 2, the αβ average for the observer. */
// alternatively, Ls as the per-phase inductance, the phase to neutral winding self-inductance in the abc frame
static inline uint32_t Motor_Ls_PuTurn(const Motor_Electrical_T * p_electrical) { return ((uint64_t)Motor_Ld_PuTurn(p_electrical) + Motor_Lq_PuTurn(p_electrical)) / 2U; }


/* e = ω · ψ as s = θ · r, at a unit arc: ψ in Wb, L in H are the radius, the rate is the angle. Unset reads 0 both ways */
static inline int32_t _Motor_Electrical_SiOf(angle_freq_t rate, uint32_t scale) { return (rate > 0) ? angle16_arc_radius(rate, scale) : 0; }
static inline angle_freq_t _Motor_Electrical_RateOfSi(int32_t si, uint32_t scale) { return (si > 0) ? angle16_of_arc(scale, si) : 0; }


// #define MOTOR_ANGLE_FREQ_RAD     ANGLE_FREQ_OF_RADS(MOTOR_CONTROL_FREQ)  /* 1 rad per Ts. ω_base = Fs [rad/s] */
/* L_rad = L / (R_base · Ts) = Fs · 2^30 · I_base / (π · AngleFreqPerOhm · V_base) */
// static inline uint32_t Motor_Ld_PuRad(const Motor_Electrical_T * p_electrical)  { return l_pu_of_angle_freq(MOTOR_ANGLE_FREQ_RAD, Phase_VMaxVolts(), Phase_IMaxAmps(), p_electrical->AngleFreqPerOhmD); }
// static inline uint32_t Motor_Psi_PuRad(const Motor_Electrical_T * p_electrical) { return psi_pu_of_angle_freq(MOTOR_ANGLE_FREQ_RAD, Phase_VMaxVolts(), p_electrical->AngleFreqPerVolt); }


/*
    Non-inverted form
    Fs, V_pu, I_pu rad base. Still independent of ω_base

    same shape for SI and PU
*/
// typedef struct
// {
//     uint8_t PolePairs;
//     uint32_t Psi;    /* [V_base · Ts] */
//     uint32_t Ld;     /* [V_base / I_base · Ts] */
//     uint32_t Lq;
//     uint32_t Rs;
// }
// Motor_Stator_T;

/*
    SI -> PU on the board base [Phase_Board_T].
*/
// static inline uint32_t Motor_Electrical_RsPu(const Motor_Stator_T * p_si) { return rs_pu_of_ohm(Phase_VMaxVolts(), Phase_IMaxAmps(), p_si->Rs, 1000000UL); }

// static inline Motor_Stator_T Motor_Electrical_PuOfSi(const Motor_Stator_T * p_si)
// {
//     return (Motor_Stator_T)
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
    ω_base
    PID and Speed input
*/
/******************************************************************************/
/*
    Set ψ_pu == v_pu/speed_pu = .5, where V_MAX is inverter max with margin
    SpeedRated_pu = VNominal_pu, SpeedBase ~ ψ_pu
    Speed_pu = V_phase_pu * 2
    SpeedBase [rpm] = Kv * V_MAX
    SpeedRated [rpm] = Kv * VNominal
    SpeedLimitDefault [rpm] = Kv * V_Rated
    fw_max => V_MAX, saturate fw at V_MAX

    ψ_pu = 1.0
    SpeedRated_pu = VNominal_Pu / √3
    SpeedBase [rpm] = Kv * V_MAX * √3
    fw_max => V_MAX * 1.73

    alternatively materialize the base speed via VBus and store as an additional parameter
    when SpeedBase = SpeedRated * 2 = Kv * V_Nominal * 2
    Ke = VNominal * 2
    ψ_pu = V_Nominal / V_MAX = V_Nominal_pu
    fw_max => 2 * V_Nominal
*/
/* VBemf_Pu at the speed base, so ψ_pu at the speed base */
/* Speed_pu = 1.0 where phase peak BEMF = V_base / 2, the phase peak at VBus = V_base (M = 2). Equals ψ_pu at the speed base */
/* VPhaseMax_Pu */
/*
    When PWM is off (fault or freewheel), the body diodes rectify the line-to-line peak BEMF into the bus.
    V_MAX as MOSFETs rating -> k = 2
    V_MAX as line-to-line max -> k = √3

    V_MAX / 2 => ω_pu = 1.0, v_emf_pu = 0.5 (capped):
    V_emf_ll = V_MAX / 2 · √3 = 0.866 · V_MAX
    e.g. v_emf_pk = 94 V / 2 = 47 V => V_emf_ll = 81.4 V
    V_Rated / √3, e.g. 72 V / √3 ≈ 41.6 V => ω_pu = .885,

    k = √3 makes speed_pu the line-to-line BEMF / V_MAX.
    SpeedVLimit = VBus and SpeedRated = VNominal hold exactly for SVPWM.
*/
#define MOTOR_SPEED_BASE_V_PU (FRACT16_SCALE / 2)


/*
    Speed Pu base scaled to V_pu, bounds PID input.  optionally User interface use angle dt.
    One base, three encodings: AngleFreq [angle16/s], AngleDt = AngleFreq / Fs [angle16/Ts], Rpm [mechanical]
    Phase_VMaxVolts is configurable VBus Max.
*/
/*
    ψ_pu fixed to 0.5
    OmegaPsi = 0.5, when ω_pu = 1.0 pu
*/
/* Speed at a bus voltage, phase peak back-EMF = V_bus / 2. V_MAX gives the base, VNominal the rated speed */
static inline angle_freq_t Motor_Speed_AngleFreqOfVBus(const Motor_Electrical_T * p_electrical, uint16_t vBus_V) { return (uint64_t)p_electrical->AngleFreqPerVolt * vBus_V * MOTOR_SPEED_BASE_V_PU / FRACT16_SCALE; }

static inline angle_freq_t Motor_SpeedBase_AngleFreq(const Motor_Electrical_T * p_electrical) { return Motor_Speed_AngleFreqOfVBus(p_electrical, Phase_VMaxVolts()); }
static inline angle_dt_t Motor_SpeedBase_AngleDt(const Motor_Electrical_T * p_electrical) { return angle_dt_of_angle_freq(MOTOR_CONTROL_FREQ, Motor_SpeedBase_AngleFreq(p_electrical)); }
static inline uint32_t Motor_SpeedBase_Rpm(const Motor_Electrical_T * p_electrical) { return mech_rpm_of_el_angle_freq(p_electrical->PolePairs, Motor_SpeedBase_AngleFreq(p_electrical)); }

/* Local Unit Conversion */
static inline accum32_t Motor_Speed_PuOfRpm(const Motor_Electrical_T * p_electrical, int16_t speed_rpm) { return speed_rpm * INT16_MAX / (int32_t)Motor_SpeedBase_Rpm(p_electrical); }
static inline int16_t Motor_Speed_RpmOfPu(const Motor_Electrical_T * p_electrical, accum32_t speed_pu) { return speed_pu * (int32_t)Motor_SpeedBase_Rpm(p_electrical) / 32768; }
// static inline int16_t Motor_Rated_AngleOfRpm(const Motor_Rated_T * p_rated, accum32_t speed_rpm) { return el_angle_dt_of_mech_rpm(MOTOR_CONTROL_FREQ, p_rated->PolePairs, speed_rpm); }
// static inline int16_t Motor_Rated_RpmOfAngle(const Motor_Rated_T * p_rated, accum32_t speed_angle16) { return mech_rpm_of_el_angle_dt(MOTOR_CONTROL_FREQ, p_rated->PolePairs, speed_angle16); }

/******************************************************************************/
/*
    Field Weakening
    Id budget bounded by the board FW rating. Iq budget is the remaining component of the rated current vector.
*/
/******************************************************************************/
typedef struct
{
    // fract16_t IsMax;     /* Current circle radius */
    ufract16_t IfwLimit;     /* [0:32767] max demagnetizing I magnitude; 0 = FW disabled */
    ufract16_t IfwGain;      /* Field weakening integrator gain per control cycle */
}
Motor_FieldWeakeningTuning_T;

static inline ufract16_t Motor_FieldWeakening_IsMax(const Motor_FieldWeakeningTuning_T * p_tuning) { return Phase_IRatedPeak_Pu(); }
static inline ufract16_t Motor_FieldWeakening_IdLimit(const Motor_FieldWeakeningTuning_T * p_tuning) { return math_min(p_tuning->IfwLimit, Phase_IRatedFw_Pu()); }
static inline ufract16_t Motor_FieldWeakening_IqLimit(const Motor_FieldWeakeningTuning_T * p_tuning) { return fract16_vector_component(Motor_FieldWeakening_IdLimit(p_tuning), Phase_IRatedPeak_Pu()); }
static inline bool Motor_FieldWeakening_IsEnabled(const Motor_FieldWeakeningTuning_T * p_tuning) { return (Motor_FieldWeakening_IdLimit(p_tuning) > 0U); }


static inline Motor_UnitBasis_T Motor_UnitBasis_Board(void) { return (Motor_UnitBasis_T) { .V = Phase_VMaxVolts(), .I = Phase_IMaxAmps(), .F = MOTOR_ANGLE_FREQ_MAX }; }

