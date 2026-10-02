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
    Storage as Kv, Derive SpeedTypeMax and SpeedRated
    store entirely motor side property, whereas SpeedRated is a function of v supply
*/
/******************************************************************************/
typedef struct
{
    uint8_t  PolePairs;
    uint16_t Kv;
    uint16_t VSpeedAdjustment; /* Additional adjustment for VBemf match. ensure resume control at lower speed. */
    /* alternatively store speedRated_Rpm, with option to resolve with VBus during init. */
}
Motor_Kv_T;

static inline int16_t _Motor_AngleOfRpm(const Motor_Kv_T * p_config, accum32_t speed_rpm) { return el_angle_dt_of_mech_rpm(MOTOR_CONTROL_FREQ, p_config->PolePairs, speed_rpm); }
static inline int16_t _Motor_RpmOfAngle(const Motor_Kv_T * p_config, accum32_t speed_angle16) { return mech_rpm_of_el_angle_dt(MOTOR_CONTROL_FREQ, p_config->PolePairs, speed_angle16); }


/******************************************************************************/
/*
    Numerical Type Max
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
static inline uint32_t _Motor_GetSpeedTypeMax_Rpm(const Motor_Kv_T * p_config) { return Phase_VMaxVolts() * p_config->Kv; }
static inline uint32_t _Motor_GetSpeedTypeMax_ElRpm(const Motor_Kv_T * p_config) { return (uint32_t)p_config->PolePairs * _Motor_GetSpeedTypeMax_Rpm(p_config); }
static inline uint32_t _Motor_GetSpeedTypeMax_Rads(const Motor_Kv_T * p_config) { return el_rads_of_mech_rpm(p_config->PolePairs, _Motor_GetSpeedTypeMax_Rpm(p_config)); }
static inline uint16_t _Motor_GetSpeedTypeMax_Angle(const Motor_Kv_T * p_config) { return _Motor_AngleOfRpm(p_config, _Motor_GetSpeedTypeMax_Rpm(p_config)); }

/* Local Unit Conversion */
static inline accum32_t Motor_Speed_Fract16OfRpm(const Motor_Kv_T * p_config, int16_t speed_rpm) { return speed_rpm * INT16_MAX / _Motor_GetSpeedTypeMax_Rpm(p_config); }
static inline int16_t Motor_Speed_RpmOfFract16(const Motor_Kv_T * p_config, accum32_t speed_fract16) { return speed_fract16 * _Motor_GetSpeedTypeMax_Rpm(p_config) / 32768; }

/*
    [V_Fract16 / Speed_Fract16]
*/
static inline accum32_t _Motor_GetKe_Fract16(const Motor_Kv_T * p_config) { return ke_pu_rpm_of_kv(Phase_VMaxVolts(), _Motor_GetSpeedTypeMax_Rpm(p_config), p_config->Kv); }
static inline accum32_t _Motor_GetPsi_Fract16(const Motor_Kv_T * p_config) { return psi_pu_rpm_of_kv(Phase_VMaxVolts(), _Motor_GetSpeedTypeMax_Rpm(p_config), p_config->Kv); }
// static inline accum32_t Motor_GetPsi_Angle16(const Motor_Kv_T * p_config) { return psi_pu_angle_of_kv(Phase_VMaxVolts(), Motor_GetSpeedTypeMax_Rpm(p_config), p_config->Kv); }


// /******************************************************************************/
// /*
//     Alternative input shapes.
//     Each resolves the same speed base as [Motor_Kv_T]: the speed at which ψ_pu = MOTOR_SPEED_BASE_PSI_PU.
// */
// /******************************************************************************/
// #define MOTOR_SPEED_BASE_PSI_PU (FRACT16_SCALE / 2)

// /* Ke in the Kv voltage basis. Ke = 1000 / Kv */
// typedef struct
// {
//     uint8_t  PolePairs;
//     uint32_t Ke_mVPerKrpm;
// }
// Motor_Ke_T;

// /* ψ_f phase peak */
// typedef struct
// {
//     uint8_t  PolePairs;
//     uint32_t Psi_uWb;
// }
// Motor_Psi_T;

// static inline uint32_t Motor_Ke_GetSpeedTypeMax_Rpm(const Motor_Ke_T * p_config) { return rpm_of_ke_v(p_config->Ke_mVPerKrpm, Phase_VMaxVolts()); }
// static inline uint32_t Motor_Psi_GetSpeedTypeMax_Rpm(const Motor_Psi_T * p_config) { return speed_base_rpm_of_psi_wb(Phase_VMaxVolts(), p_config->PolePairs, MOTOR_SPEED_BASE_PSI_PU, p_config->Psi_uWb, 1000000UL); }


// typedef struct
// {
//     // fract16_t IsMax;     /* Current circle radius */
//     ufract16_t IfwLimit;     /* [0:32767] max demagnetizing I magnitude; 0 = FW disabled */
//     ufract16_t IfwGain;      /* Field weakening integrator gain per control cycle */
// }
// Motor_FieldWeakeningTuning_T;

// /* same shape for SI and PU  */
// /*

// */
typedef struct
{
    uint32_t Ls; /* alternatively L_tau = L_tick · SpeedMax_Angle16 / 32768 */
    uint32_t Ldelta;
    uint32_t Rs;
    uint32_t Psi;
}
Motor_Electrical_T;

// static inline accum32_t _Motor_Psi_Pu( ) {


// /******************************************************************************/
// /*
//     Field Weakening
//     Id budget bounded by the board FW rating. Iq budget is the remaining component of the rated current vector.
// */
// /******************************************************************************/
// static inline ufract16_t Motor_FieldWeakening_IdLimit(const Motor_FieldWeakeningTuning_T * p_tuning) { return math_min(p_tuning->IfwLimit, Phase_IRatedFw_Fract16()); }
// static inline ufract16_t Motor_FieldWeakening_IqLimit(const Motor_FieldWeakeningTuning_T * p_tuning) { return fract16_vector_component(Motor_FieldWeakening_IdLimit(p_tuning), Phase_IRatedPeak_Fract16()); }


// /******************************************************************************/
// /*
//     [Motor_Electrical_T]
//     SI storage: Rs [µΩ], Ls, Ldelta [µH], Psi [µWb]
//     Ls = (Ld + Lq) / 2, Ldelta = Lq - Ld. SPM: Ldelta = 0
// */
// /******************************************************************************/
// /* Either shape */
// static inline uint32_t Motor_Electrical_Ld(const Motor_Electrical_T * p_electrical) { return p_electrical->Ls - p_electrical->Ldelta / 2U; }
// static inline uint32_t Motor_Electrical_Lq(const Motor_Electrical_T * p_electrical) { return p_electrical->Ls + p_electrical->Ldelta / 2U; }

// /* PM rotor Lq >= Ld. Ld > Lq clamps to non-salient */
// static inline void Motor_Electrical_SetLdq(Motor_Electrical_T * p_electrical, uint32_t ld, uint32_t lq) { p_electrical->Ls = (ld + lq) / 2U; p_electrical->Ldelta = lq - math_min(ld, lq); }

// /*
//     SI -> PU on the board base [Phase_Board_T] and the speed base.
//     Pure. Re-resolve on a change of either base.
//     Psi 0 is unset, ψ_pu by the speed base definition.
// */
// static inline uint32_t Motor_Electrical_RsPu(const Motor_Electrical_T * p_si) { return rs_pu_of_ohm(Phase_VMaxVolts(), Phase_IMaxAmps(), p_si->Rs, 1000000UL); }

// static inline Motor_Electrical_T Motor_Electrical_PuOfSi(const Motor_Electrical_T * p_si, uint32_t speedBase_Rpm, uint8_t polePairs)
// {
//     return (Motor_Electrical_T)
//     {
//         .Ls     = l_pu_rpm_of_uh(Phase_VMaxVolts(), Phase_IMaxAmps(), speedBase_Rpm, polePairs, p_si->Ls),
//         .Ldelta = l_pu_rpm_of_uh(Phase_VMaxVolts(), Phase_IMaxAmps(), speedBase_Rpm, polePairs, p_si->Ldelta),
//         .Rs     = Motor_Electrical_RsPu(p_si),
//         .Psi    = (p_si->Psi != 0U) ? psi_pu_rpm_of_uwb(Phase_VMaxVolts(), speedBase_Rpm, polePairs, p_si->Psi) : MOTOR_SPEED_BASE_PSI_PU,
//     };
// }

// /* FOC view, d-q frame */
// static inline FOC_Electrical_T Motor_Electrical_FocOf(const Motor_Electrical_T * p_pu)
// {
//     return (FOC_Electrical_T) { .Ld = Motor_Electrical_Ld(p_pu), .Lq = Motor_Electrical_Lq(p_pu), .Rs = p_pu->Rs, .Psi = p_pu->Psi, };
// }








/*
    Alternative Storage

    resolved speed
    alternatively store as control domain units angldt, rps <<15 as 1 step from either
*/

/* separate data object for ui */
/* si units for per motor base */
/* optionally resolve reference on device side*/
// struct Motor_ElectricalBase
// {
//     int32_t V ;
//     int32_t I ;
//     int32_t W ;
//     int32_t Psi ;
//     int32_t Tau ;
//     int32_t L ;
//     int32_t R ;
//     int32_t T ;
// } Motor_ElectricalBase_T;
