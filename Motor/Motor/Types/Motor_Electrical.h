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
    PU encoding basis for ψ_pu / L_pu in ElectricalParams_Pu.

        RPM     ω_base = π·P·n_max/30   (motor anchor — Fs-independent)
                L_pu = L · I_max · ω_base / V_max
                ψ_pu = ψ_f · ω_base / V_max

        ANGLE16 ω_base = π·Fs           (controller anchor — Fs-locked)
                L_pu = L · I_max · π · Fs / V_max
                ψ_pu = ψ_f · π · Fs / V_max

    define ω_pu corresponding to  L_pu·ω_pu and ψ_pu·ω_pu  without basis awareness
*/
/*
    motor speed anchored:
    Fs independent
    ω_mech_pu = ω_elec_pu
    θ_increment = ω_pu · (ω_base / Fs)
*/
/*
    Fs-anchored, motor independent ω_pu
    accept int64 intermediary or
    direct-multiply (no Q15 scaling), delta_angle16 * L_pu → v_pu without /32768 => ~0.5% resolution loss
*/
#if !defined(MOTOR_PU_BASIS_RPM) && !defined(MOTOR_PU_BASIS_ANGLE16)
#define MOTOR_PU_BASIS_RPM
#endif

/*
    config via Kv
*/
typedef struct
{
    uint8_t  PolePairs;
    uint16_t Kv;
    uint16_t SpeedRated_Rpm;
    uint16_t VSpeedAdjustment;      /* Additional adjustment for VBemf match. ensure resume control at lower speed. */
    /* collapse one */
}
Motor_ElectricalSpeedRating_T;
// Motor_KSpeed_T;
// Motor_Kv_T;


// typedef struct
// {
//     uint32_t Rs;
//     uint32_t Ld; /* store the foc version   */
//     uint32_t Lq;
// }
// Motor_RL_T;

/* Saturation Max */
#define MOTOR_EL_ANGLE_SPEED_MAX_RADS  ANGLE_SPEED_MAX_RADS(MOTOR_CONTROL_FREQ)
#define MOTOR_EL_ANGLE_SPEED_MAX_RPM   ANGLE_SPEED_MAX_RPM(MOTOR_CONTROL_FREQ)

static inline int16_t _Motor_AngleOfRpm(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_rpm) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_config->PolePairs, speed_rpm); }
static inline int16_t _Motor_RpmOfAngle(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_degPerCycle) { return mech_rpm_of_el_angle(MOTOR_CONTROL_FREQ, p_config->PolePairs, speed_degPerCycle); }


/******************************************************************************/
/*
    Num Type Max
*/
/******************************************************************************/
/*
    Numerical max as ~2x kv*V
    account for 1.15 factor and field weakening range.
*/
#ifndef MOTOR_SPEED_TYPE_MAX_RPM /* defined as MOTOR_SPEED_TYPE_MAX_RPM(...) */
static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_ElectricalSpeedRating_T * p_config); /* forward declare depedency, for runtime derived */
#define MOTOR_SPEED_TYPE_MAX_RPM(p_config) ((uint32_t)Motor_GetSpeedRated_Rpm(p_config)*2)
// #define SPEED_FRACT16(RpmMax,Rpm) FRACT16((float)Rpm / RpmMax)
#else /* Compile time def only */
#define SPEED_FRACT16_OF_RPM(Rpm)      (Rpm * INT16_MAX / MOTOR_SPEED_TYPE_MAX_RPM())
#define SPEED_RPM_OF_FRACT16(Fract16)  (Fract16 * MOTOR_SPEED_TYPE_MAX_RPM() / 32768)
#endif
//selection between phase_max or vnominal
// #define MOTOR_SPEED_TYPE_MAX_RPM(Kv, VBase)

static inline uint16_t Motor_GetSpeedTypeMax_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return Motor_GetSpeedRated_Rpm(p_config) * 2; }
static inline uint16_t Motor_GetSpeedTypeMax_Angle(const Motor_ElectricalSpeedRating_T * p_config) { return _Motor_AngleOfRpm(p_config, Motor_GetSpeedTypeMax_Rpm(p_config)); }
static inline uint16_t Motor_GetSpeedTypeMax_Rads(const Motor_ElectricalSpeedRating_T * p_config) { return el_rads_of_mech_rpm(Motor_GetSpeedTypeMax_Rpm(p_config), p_config->PolePairs); }



/* alternatively inverter max, with rate  */
// static inline uint16_t Motor_SpeedTypeMax_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return Phase_Calibration_GetVMaxVolts() * p_config->Kv; }
/* 94v => 108v equalent speed during fw */
// static inline uint16_t Motor_SpeedTypeMax_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return  (Phase_Calibration_GetVMaxVolts() *p_config->Kv) *2 / sqrt(3); }
// static inline uint16_t Motor_SpeedTypeMax_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return Vnominal * kv * 2; }


/* Local Unit Conversion */
static inline accum32_t Motor_Speed_Fract16OfRpm(const Motor_ElectricalSpeedRating_T * p_config, int16_t speed_rpm) { return speed_rpm * INT16_MAX / Motor_GetSpeedTypeMax_Rpm(p_config); }
static inline int16_t Motor_Speed_RpmOfFract16(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_fract16) { return speed_fract16 * Motor_GetSpeedTypeMax_Rpm(p_config) / 32768; }


/* [SpeedRated_Rpm] = [KvVBusNominal] = [SpeedTypeMax_Rpm] / 2  */
/* Keep instanced */
static inline uint16_t Motor_GetSpeedRated_Fract16(const Motor_ElectricalSpeedRating_T * p_config) { (void)p_config; return INT16_MAX / 2; }



/******************************************************************************/
/*
    [SpeedRated] via direct Parameter
    Simplify user set and decouples TypeMax from VBus module
    alternatively user calibration speedtypemax determine from user set vnomial
    Config set Kv mediates
*/
/******************************************************************************/
static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return p_config->SpeedRated_Rpm; }
// static inline uint32_t Motor_GetSpeedRated_ERpm(const Motor_ElectricalSpeedRating_T * p_config) { return (uint32_t)p_config->SpeedRated_Rpm * p_config->PolePairs; }
// static inline uint16_t Motor_GetSpeedRated_Angle(const Motor_ElectricalSpeedRating_T * p_config) { return _Motor_AngleOfRpm(p_config, Motor_GetSpeedRated_Rpm(p_config)); }

// static inline uint16_t Motor_SpeedBase_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return rpm_of_kv_v(p_config->Kv, volts); }
// static inline uint16_t Motor_SpeedVMatch_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return p_config->SpeedRated_Rpm; }
// static inline uint16_t Motor_GetSpeedVMatch_(const Motor_ElectricalSpeedRating_T * p_motor) { return fract16_mul(p_motor->VSpeedAdjustment, Motor_GetSpeedRated_Fract16(p_motor)); }



/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    [V_Fract16 / Speed_Fract16]
    V_fract16 = fract16_mul(Ke_SpeedFract, Speed_fract16)
    when SpeedTypeMax = Kv * VNominal * 2
    Ke = VNominal * 2

    ψ_pu -
        a rate, kv or ψ_si, with SpeedMax known > speed at VNominal
        SpeedRated at Vnominal, with Vnominal
        a rate, with Vnominal
*/
static inline accum32_t _Motor_Ke_Fract16(const Motor_ElectricalSpeedRating_T * p_config) { return ke_pu_rpm_of_kv(Phase_Calibration_GetVMaxVolts(), Motor_GetSpeedTypeMax_Rpm(p_config), p_config->Kv); }
static inline accum32_t _Motor_Psi_Fract16(const Motor_ElectricalSpeedRating_T * p_config) { return psi_pu_rpm_of_kv(Phase_Calibration_GetVMaxVolts(), Motor_GetSpeedTypeMax_Rpm(p_config), p_config->Kv); }

// static inline accum32_t Motor_GetPsi_Angle16(const Motor_ElectricalSpeedRating_T * p_config) { return ke_pu_rpm_of_kv(Phase_Calibration_GetVMaxVolts(), Motor_GetSpeedTypeMax_Rpm(p_config), p_config->Kv); }




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


