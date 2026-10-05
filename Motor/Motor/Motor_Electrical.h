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
        F(V) = AngleFreqPerVolt · V     [angle16/s], V phase peak. Free of rpm, rad/s, and the board base
        AngleFreqPerVolt stores Psi without V_base.
        Rs [V_base / I_base].
        Ls [V_base / I_base · Ts].
        Ls = (Ld + Lq) / 2, Ldelta = Lq - Ld at the tick base. SPM: Ldelta = 0
    Kv and rpm are views at the user boundary.
*/
/******************************************************************************/
typedef struct
{
    uint8_t PolePairs;
    angle_freq_t AngleFreqPerVolt;  /* Electrical [angle16/s per V], phase peak. 1 / ψ */
    uint32_t Ls;                    /* speed base: L_pu = L_tick · AngleDtBase / 32768 */
    uint32_t Ldelta;
    uint32_t Rs;
}
Motor_Electrical_T;

/* PM rotor: Lq >= Ld */
#define MOTOR_LS_TICK_OF_UH(Fs, V_Base, I_Base, Ld_uH, Lq_uH)        MOTOR_L_PU_TICK(Fs, V_Base, I_Base, ((Ld_uH) + (Lq_uH)) * 0.5E-6F)
#define MOTOR_LDELTA_TICK_OF_UH(Fs, V_Base, I_Base, Ld_uH, Lq_uH)    MOTOR_L_PU_TICK(Fs, V_Base, I_Base, ((Lq_uH) - (Ld_uH)) * 1.0E-6F)

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    Set ψ_pu = v_pu/speed_pu = ~ .5, where V_Max is inverter max with margin
    when SpeedBase = Kv * V_Max
    SpeedRated_pu = VNominal_pu, SpeedRated_Rpm = Kv * VNominal
    Speed_pu = V_pu = V_phase_pu * 2
    ke_pu = 1.0
    ψ_pu = .5

    alternatively materialize the base speed and store as an additional parameter
    when SpeedBase = SpeedRated * 2 = Kv * V_Nominal * 2
    Ke = VNominal * 2
    ψ_pu = V_Nominal / V_Max = V_Nominal_pu
*/
/*
    alteratively pid use separate base 2x vnominal ~10000, ui use angle for invariant ui
*/
/* VBemf_Pu at the speed base, so ψ_pu at the speed base */
/* Speed_pu = 1.0 where phase peak BEMF = V_base / 2, the phase peak at VBus = V_base (M = 2). Equals ψ_pu at the speed base */
/* VPhaseMax_Pu */
#define MOTOR_SPEED_BASE_V_PU (FRACT16_SCALE / 2)

/*
    One base, three encodings: AngleFreq [angle16/s], AngleDt = AngleFreq / Fs [angle16/Ts], Rpm [mechanical]
*/
static inline angle_freq_t Motor_SpeedBase_AngleFreq(const Motor_Electrical_T * p_electrical) { return angle_freq_at_v(p_electrical->AngleFreqPerVolt, Phase_VMaxVolts(), MOTOR_SPEED_BASE_V_PU); }
// static inline angle_freq_t Motor_SpeedBase_AngleFreq(const Motor_Electrical_T * p_electrical) { return accum32_mul(p_electrical->AngleFreqPerVolt * Phase_VMaxVolts(), MOTOR_SPEED_BASE_V_PU); }
static inline angle_dt_t Motor_SpeedBase_AngleDt(const Motor_Electrical_T * p_electrical) { return angle_dt_of_angle_freq(MOTOR_CONTROL_FREQ, Motor_SpeedBase_AngleFreq(p_electrical)); }
static inline uint32_t Motor_SpeedBase_Rpm(const Motor_Electrical_T * p_electrical) { return mech_rpm_of_el_angle_freq(p_electrical->PolePairs, Motor_SpeedBase_AngleFreq(p_electrical)); }

/*
    ψ, L between the tick base and the speed base.
    Up to the midpoint of the interval x_pu stands for, (x_pu + 1/2) · F_tick / F_base, so Motor_SpeedBase_PuOfTick returns x_pu.
*/
static inline uint32_t Motor_SpeedBase_PuOfTick(const Motor_Electrical_T * p_electrical, uint32_t x_tick) { return pu_tau_rebase(x_tick, angle_freq_nyquist(MOTOR_CONTROL_FREQ), Motor_SpeedBase_AngleFreq(p_electrical)); }
static inline uint32_t Motor_SpeedBase_TickOfPu(const Motor_Electrical_T * p_electrical, uint32_t x_pu) { return ((uint64_t)x_pu * 2U + 1U) * angle_freq_nyquist(MOTOR_CONTROL_FREQ) / ((uint64_t)Motor_SpeedBase_AngleFreq(p_electrical) * 2U); }

/* Local Unit Conversion */
static inline accum32_t Motor_Speed_PuOfRpm(const Motor_Electrical_T * p_electrical, int16_t speed_rpm) { return speed_rpm * INT16_MAX / (int32_t)Motor_SpeedBase_Rpm(p_electrical); }
static inline int16_t Motor_Speed_RpmOfPu(const Motor_Electrical_T * p_electrical, accum32_t speed_pu) { return speed_pu * (int32_t)Motor_SpeedBase_Rpm(p_electrical) / 32768; }
// static inline int16_t Motor_Rated_AngleOfRpm(const Motor_Rated_T * p_rated, accum32_t speed_rpm) { return el_angle_dt_of_mech_rpm(MOTOR_CONTROL_FREQ, p_rated->PolePairs, speed_rpm); }
// static inline int16_t Motor_Rated_RpmOfAngle(const Motor_Rated_T * p_rated, accum32_t speed_angle16) { return mech_rpm_of_el_angle_dt(MOTOR_CONTROL_FREQ, p_rated->PolePairs, speed_angle16); }



/******************************************************************************/
/*
    [Motor_Electrical_T] Ld, Lq, Rs
    Same shape in SI: Rs [µΩ], Ls, Ldelta [µH]. PolePairs and AngleFreqPerVolt are free of the board base
*/
/******************************************************************************/
/* Either shape. Lq from Ld, exact where Ls + Ldelta / 2 truncates twice */
static inline uint32_t Motor_Electrical_Ld(const Motor_Electrical_T * p_electrical) { return p_electrical->Ls - p_electrical->Ldelta / 2U; }
static inline uint32_t Motor_Electrical_Lq(const Motor_Electrical_T * p_electrical) { return Motor_Electrical_Ld(p_electrical) + p_electrical->Ldelta; }

/* PM rotor Lq >= Ld. Ld > Lq clamps to non-salient */
static inline void Motor_Electrical_SetLdq(Motor_Electrical_T * p_electrical, uint32_t ld, uint32_t lq) { p_electrical->Ls = (ld + lq) / 2U; p_electrical->Ldelta = lq - math_min(ld, lq); }

/*
    SI -> PU on the board base [Phase_Board_T], tick base.
*/
static inline uint32_t Motor_Electrical_RsPu(const Motor_Electrical_T * p_si) { return rs_pu_of_ohm(Phase_VMaxVolts(), Phase_IMaxAmps(), p_si->Rs, 1000000UL); }

static inline Motor_Electrical_T Motor_Electrical_PuOfSi(const Motor_Electrical_T * p_si)
{
    return (Motor_Electrical_T)
    {
        .PolePairs          = p_si->PolePairs,
        .AngleFreqPerVolt   = p_si->AngleFreqPerVolt,
        .Ls                 = l_pu_tick_of_h(MOTOR_CONTROL_FREQ, Phase_VMaxVolts(), Phase_IMaxAmps(), p_si->Ls, 1000000UL),
        .Ldelta             = l_pu_tick_of_h(MOTOR_CONTROL_FREQ, Phase_VMaxVolts(), Phase_IMaxAmps(), p_si->Ldelta, 1000000UL),
        .Rs                 = Motor_Electrical_RsPu(p_si),
    };
}

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
        .Ld     = Motor_SpeedBase_PuOfTick(p_electrical, Motor_Electrical_Ld(p_electrical)),
        .Lq     = Motor_SpeedBase_PuOfTick(p_electrical, Motor_Electrical_Lq(p_electrical)),
        .Rs     = p_electrical->Rs,
        .Psi    = MOTOR_SPEED_BASE_V_PU,
    };
}

/* Ld, Lq, Rs from the FOC view. ψ is 1 / AngleFreqPerVolt, not stored */
static inline void Motor_Electrical_SetFoc(Motor_Electrical_T * p_electrical, const FOC_Electrical_T * p_foc)
{
    Motor_Electrical_SetLdq(p_electrical, Motor_SpeedBase_TickOfPu(p_electrical, p_foc->Ld), Motor_SpeedBase_TickOfPu(p_electrical, p_foc->Lq));
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

