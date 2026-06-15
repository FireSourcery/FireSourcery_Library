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
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../Math/motor_electrical_math.h"
#include "Math/Angle/angle_speed_math.h"
#include "../Motor_ControlFreq.h"
#include "../Phase_Input/Phase_Calibration.h"


/*
    PU encoding basis for ψ_pu, L_pu
        RPM     ω_base = π·P·n_max/30 = π·P·Kv·V/30  (motor anchor — Fs-independent)
                L_pu = L · I_base · ω_base / V_base
                ψ_pu = ψ_f · ω_base / V_base

        ANGLE16 ω_base = π·Fs           (controller anchor — Fs-locked, motor independent)
                L_pu = L · I_base · π · Fs / V_base
                ψ_pu = ψ_f · π · Fs / V_base

    define ω_pu corresponding to  L_pu·ω_pu and ψ_pu·ω_pu  without basis awareness

    motor speed anchored:
    Fs independent
    ω_mech_pu = ω_elec_pu
    θ_increment = ω_pu · (ω_base / Fs)

    Fs-anchored, motor independent ω_pu
    accept int64 intermediary or
    direct-multiply (no Q15 scaling), delta_angle16 * L_pu → v_pu without /32768 => ~0.5% resolution loss
*/
#if !defined(MOTOR_PU_BASIS_RPM) && !defined(MOTOR_PU_BASIS_ANGLE16)
#define MOTOR_PU_BASIS_RPM
#endif

/* Saturation Max ANGLE16 base only */
#define MOTOR_EL_ANGLE_SPEED_MAX_RADS  ANGLE_SPEED_MAX_RADS(MOTOR_CONTROL_FREQ)
#define MOTOR_EL_ANGLE_SPEED_MAX_RPM   ANGLE_SPEED_MAX_RPM(MOTOR_CONTROL_FREQ)
// #define MOTOR_EL_ANGLE_SPEED_MAX        (32768)
/*
    alternatively /k
    e.g.
        speed max = 6000 rpm /60 · 4 = 400 Hz
        speed base = anglespeedmax · f_s / 65536 = 2048 · 20000 / 65536 = 625 Hz
*/
// #define MOTOR_EL_ANGLE_SPEED_MAX_HZ   ANGLE_SPEED_MAX * MOTOR_CONTROL_FREQ / 65536


/******************************************************************************/
/*
    config via Kv
    motor side rating, indepedent of inverter and supply
*/
/******************************************************************************/
typedef struct
{
    uint8_t  PolePairs;
    uint16_t Kv;
    uint16_t VSpeedAdjustment; /* Additional adjustment for VBemf match. ensure resume control at lower speed. */
}
Motor_ElectricalSpeedRating_T;

static inline int16_t _Motor_AngleOfRpm(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_rpm) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_config->PolePairs, speed_rpm); }
static inline int16_t _Motor_RpmOfAngle(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_degPerCycle) { return mech_rpm_of_el_angle(MOTOR_CONTROL_FREQ, p_config->PolePairs, speed_degPerCycle); }


/******************************************************************************/
/*
    Numerical Type Max
*/
/******************************************************************************/
/*  inverter max
    vbus max with margin
    sets the fw speed limit
*/
static inline uint16_t _Motor_GetSpeedTypeMax_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return Phase_Calibration_GetVMaxVolts() * p_config->Kv; }
/* either saturation max or nameplate rated for fw speed limit */
// static inline uint16_t Motor_GetSpeedTypeMax_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return (Phase_Calibration_GetVRated_V() * p_config->Kv); }
static inline uint16_t _Motor_GetSpeedTypeMax_Angle(const Motor_ElectricalSpeedRating_T * p_config) { return _Motor_AngleOfRpm(p_config, _Motor_GetSpeedTypeMax_Rpm(p_config)); }
static inline uint16_t _Motor_GetSpeedTypeMax_Rads(const Motor_ElectricalSpeedRating_T * p_config) { return el_rads_of_mech_rpm(p_config->PolePairs, _Motor_GetSpeedTypeMax_Rpm(p_config)); }


/* Local Unit Conversion */
static inline accum32_t Motor_Speed_Fract16OfRpm(const Motor_ElectricalSpeedRating_T * p_config, int16_t speed_rpm) { return speed_rpm * INT16_MAX / _Motor_GetSpeedTypeMax_Rpm(p_config); }
static inline int16_t Motor_Speed_RpmOfFract16(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_fract16) { return speed_fract16 * _Motor_GetSpeedTypeMax_Rpm(p_config) / 32768; }

/*
    [V_Fract16 / Speed_Fract16]
    Ke_pu = 1.0
    ψ_pu = .5
*/
static inline accum32_t _Motor_GetKe_Fract16(const Motor_ElectricalSpeedRating_T * p_config) { return ke_pu_rpm_of_kv(Phase_Calibration_GetVMaxVolts(), _Motor_GetSpeedTypeMax_Rpm(p_config), p_config->Kv); }
static inline accum32_t _Motor_GetPsi_Fract16(const Motor_ElectricalSpeedRating_T * p_config) { return psi_pu_rpm_of_kv(Phase_Calibration_GetVMaxVolts(), _Motor_GetSpeedTypeMax_Rpm(p_config), p_config->Kv); }
// static inline accum32_t Motor_GetPsi_Angle16(const Motor_ElectricalSpeedRating_T * p_config) { return psi_pu_angle_of_kv(Phase_Calibration_GetVMaxVolts(), Motor_GetSpeedTypeMax_Rpm(p_config), p_config->Kv); }


// static inline accum32_t _Motor_GetPsiBase_Wb(const Motor_ElectricalSpeedRating_T * p_config) { return psi_wb_of_kv(p_config->Kv, p_config->PolePairs, 1000000UL); }
// static inline accum32_t _Motor_GetPsiBase_Wb(const Motor_ElectricalSpeedRating_T * p_config) { return _Motor_GetSpeedTypeMax_Rads(p_config) / Phase_Calibration_GetVMaxVolts(); }


/*
    Derived Parameters during initialization or from Host
    alternatively store as control domain units
*/
// typedef struct
// {
//     uint16_t SpeedRated;
//     uint16_t VSpeedRatio;
// }
// Motor_ElectricalSpeedRating_T;

/* si units for per motor base */
// struct Motor_ElectricalBase
// {
//     int32_t V_base;
//     int32_t I_base;
//     int32_t W_base;
//     int32_t Psi_base;
//     int32_t L_base;
//     int32_t R_base;
//     int32_t T_base;
// };


