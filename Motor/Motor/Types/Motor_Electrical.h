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
#include "../Math/motor_params_math.h"
#include "../Motor_ControlFreq.h"
#include "../Phase_Input/Phase_Calibration.h"




/* Ke/Psi/SpeedMax */
typedef struct
{
    uint8_t  PolePairs;
    uint16_t Kv;
    uint16_t SpeedRated_Rpm;
}
Motor_ElectricalSpeedRating_T;
// Motor_Electrical_T;
// Motor_Kv_T;

typedef struct
{
    // uint16_t SpeedRated_Angle16;
    uint16_t Ke_Angle16;
    uint16_t Ke_SpeedFract16;
}
Motor_ElectricalSpeedRef_T;


/*  optionally replace with primitive layer */
static inline int16_t _Motor_AngleOfRpm(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_rpm) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_config->PolePairs, speed_rpm); }
static inline int16_t _Motor_RpmOfAngle(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_degPerCycle) { return mech_rpm_of_el_angle(MOTOR_CONTROL_FREQ, p_config->PolePairs, speed_degPerCycle); }

/*
    Speed/V relation based on Kv.
    Rpm of Kv * V
*/
static inline uint16_t Motor_RpmOfKv(const Motor_ElectricalSpeedRating_T * p_config, uint16_t v_fract16) { return fract16_mul(v_fract16, (int32_t)p_config->Kv * Phase_Calibration_GetVMaxVolts()); }
static inline uint16_t Motor_VFract16OfKv(const Motor_ElectricalSpeedRating_T * p_config, uint16_t rpm) { return fract16_div(rpm, (int32_t)p_config->Kv * Phase_Calibration_GetVMaxVolts()); }
static inline uint16_t Motor_AngleDeltaOfKv(const Motor_ElectricalSpeedRating_T * p_config, uint16_t v_fract16) { return _Motor_AngleOfRpm(p_config, Motor_RpmOfKv(p_config, v_fract16)); }

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
#define SPEED_FRACT16(Rpm, RpmMax) FRACT16((float)Rpm / RpmMax)
#else /* Compile time def only */
#define SPEED_FRACT16_OF_RPM(Rpm)      (Rpm * INT16_MAX / MOTOR_SPEED_TYPE_MAX_RPM())
#define SPEED_RPM_OF_FRACT16(Fract16)  (Fract16 * MOTOR_SPEED_TYPE_MAX_RPM() / 32768)
#endif


static inline uint16_t Motor_GetSpeedTypeMax_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return MOTOR_SPEED_TYPE_MAX_RPM(p_config); }
static inline uint16_t Motor_GetSpeedTypeMax_Angle(const Motor_ElectricalSpeedRating_T * p_config) { return _Motor_AngleOfRpm(p_config, Motor_GetSpeedTypeMax_Rpm(p_config)); }

/* Local Unit Conversion */
static inline accum32_t Motor_Speed_Fract16OfRpm(const Motor_ElectricalSpeedRating_T * p_config, int16_t speed_rpm) { return speed_rpm * INT16_MAX / Motor_GetSpeedTypeMax_Rpm(p_config); }
static inline int16_t Motor_Speed_RpmOfFract16(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_fract16) { return speed_fract16 * Motor_GetSpeedTypeMax_Rpm(p_config) / 32768; }


/* [SpeedRated_Rpm] = [KvVBusNominal] = [SpeedTypeMax_Rpm] / 2  */
/* Keep instanced */
static inline uint16_t Motor_GetSpeedRated_Fract16(const Motor_ElectricalSpeedRating_T * p_config) { (void)p_config; return INT16_MAX / 2; }


/*
    Speed VBus Ref
    Generally [SpeedRated] via Kv * V
*/


/*
    [SpeedRated] via direct Parameter
    Simplify user set and decouples TypeMax from VBus module
    alternatively user calibration speedtypemax determine from user set vnomial
    Config set Kv mediates
*/
static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_ElectricalSpeedRating_T * p_config) { return p_config->SpeedRated_Rpm; }
// static inline uint32_t Motor_GetSpeedRated_ERpm(const Motor_ElectricalSpeedRating_T * p_config, uint16_t volts) { return (uint32_t)p_config->SpeedRated_Rpm * p_config->PolePairs; }
// static inline uint16_t Motor_GetSpeedRated_Angle(const Motor_ElectricalSpeedRating_T * p_config, uint16_t volts) { return _Motor_AngleOfRpm(p_config, Motor_GetSpeedRated_Rpm(p_config, volts)); }

/* alternatively kv subsitutes VSpeedScalar_Fract16 for electrical */
// static inline uint16_t Motor_SpeedVRef_Rpm(const Motor_ElectricalSpeedRating_T * p_config, uint16_t vNominal_fract16) { return Motor_RpmOfKv(p_config, vNominal_fract16); }
// static inline uint16_t Motor_EffectiveSpeedRated_Rpm(const Motor_ElectricalSpeedRating_T * p_motor, uint16_t volts) { return fract16_mul(p_motor->VSpeedScalar_Fract16, (int32_t)p_motor->Kv * volts); }

//
static inline uint16_t Motor_SpeedBase_Rpm(const Motor_ElectricalSpeedRating_T * p_config, uint16_t volts) { return rpm_of_kv_vfract16(Phase_Calibration_GetVMaxVolts(), p_config->Kv, volts); }
static inline uint16_t Motor_SpeedVMatch_Rpm(const Motor_ElectricalSpeedRating_T * p_config, uint16_t volts) { return p_config->SpeedRated_Rpm; }

// static inline uint16_t Motor_GetSpeedVMatch_(const Motor_ElectricalSpeedRating_T * p_motor) { return fract16_mul(p_motor->VSpeedScalar_Fract16, Motor_GetSpeedRated_Fract16(p_motor)); }

/* [V / (rad/s)] */
static inline accum32_t Motor_GetKe_SiFract16(const Motor_ElectricalSpeedRating_T * p_config) { return ke_vrads_fract16(p_config->Kv); }

/* [VFract16 / (Angle16/ControlCycle)] */
static inline accum32_t Motor_GetKe_Angle16(const Motor_ElectricalSpeedRating_T * p_config) { return ke_vfract16_per_angle16(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), p_config->Kv); }

/*
    [VFract16 / SpeedFract16]
    V_fract16 = fract16_mul(Ke_SpeedFract, Speed_fract16)
        where omega_fract16 = ω / ω_max × FRACT16_MAX

    when SpeedTypeMax = Kv * VNominal * 2
    Ke = VNominal * 2
*/
static inline accum32_t Motor_GetKe_SpeedFract16(const Motor_ElectricalSpeedRating_T * p_config) { return ke_speed_fract16(Phase_Calibration_GetVMaxVolts(), Motor_GetSpeedTypeMax_Rpm(p_config), p_config->Kv); }
// static inline accum32_t Motor_GetKe_SpeedFract16(const Motor_ElectricalSpeedRating_T * p_config, uint16_t vNominal_fract16) { return fract16_div(vNominal_fract16, Motor_GetSpeedRated_Fract16(p_config)); }


// FluxLinkage [Wb] = Ke_mech / PolePairs
// λm = 60 / (2π · Kv · pp · VMax)
static inline accum32_t Motor_GetFluxLinkage_Angle16(const Motor_ElectricalSpeedRating_T * p_config) { return Motor_GetKe_Angle16(p_config) / p_config->PolePairs; }

// V_fract16 = fract16_mul(FluxLinkage_SpeedFract16, speedElec_fract16)
static inline accum32_t Motor_GetFluxLinkage_SpeedFract16(const Motor_ElectricalSpeedRating_T * p_config) { return Motor_GetKe_SpeedFract16(p_config) / p_config->PolePairs; }


static inline Motor_ElectricalSpeedRef_T Motor_ElectricalSpeedRef_FromSpeedRating(const Motor_ElectricalSpeedRating_T * p_speedRating)
{
    return (Motor_ElectricalSpeedRef_T)
    {
        .Ke_Angle16 = Motor_GetKe_Angle16(p_speedRating),
        .Ke_SpeedFract16 = Motor_GetKe_SpeedFract16(p_speedRating),
    };
}

/******************************************************************************/
/*!
    V Bemf <=> Speed conversion based on Kv.
    Speed/SpeedRated => V/VBusRef
    @return V_Fract16 - VBusRef of Speed Kv
*/
/******************************************************************************/
/*
    when SpeedTypeMax = Kv * VNominal * 2
    VSpeed  = VBus_GetVNominal() * speed_fract16 / Motor_GetSpeedRated_Fract16(p_config)
*/
static inline accum32_t Motor_VPhaseOfSpeed_Fract16(const Motor_ElectricalSpeedRef_T * p_speedRef, accum32_t speed_fract16) { return fract16_mul(p_speedRef->Ke_SpeedFract16, speed_fract16); }
static inline accum32_t Motor_SpeedOfVPhase_Fract16(const Motor_ElectricalSpeedRef_T * p_speedRef, accum32_t v_fract16) { return fract16_div(v_fract16, p_speedRef->Ke_SpeedFract16); }
// static inline accum32_t Motor_VBusOfSpeed_Fract16(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_fract16) { return Phase_VBus_GetVNominal() * speed_fract16 / Motor_GetSpeedRated_Fract16(p_config); }
// static inline accum32_t Motor_SpeedOfVBus_Fract16(const Motor_ElectricalSpeedRating_T * p_config, accum32_t v_fract16) { return v_fract16 * Motor_GetSpeedRated_Fract16(p_config) / Phase_VBus_GetVNominal(); }

// /* Phase peak. */
// static inline accum32_t Motor_VPhaseOfSpeed_Fract16(const Motor_ElectricalSpeedRating_T * p_config, accum32_t speed_fract16) { (void)p_config; return fract16_mul(Phase_VBus_GetVNominal(), speed_fract16); }




/*
    round trip debug check
*/
// static inline uint16_t _Motor_GetVSpeedRated_Fract16(const Motor_ElectricalSpeedRating_T * p_config) { return Motor_VFract16OfKv(p_config, Motor_GetSpeedRated_Rpm(p_config)); }



/*
    Derived Parameters during initialization or from Host
    alternatively store as control domain units
*/
/* Config stored in Electrical Degrees need to sync with pole pairs */

// #define _MOTOR_SPEED_TYPE_MAX_ERPM(PolePairs) ((uint32_t)(PolePairs) * MOTOR_SPEED_TYPE_MAX_RPM())
// #define _MOTOR_SPEED_TYPE_MAX_DIGIT(erpm)  ANGLE16_OF_RPM(MOTOR_CONTROL_FREQ, erpm)
// #define MOTOR_SPEED_MAX_DIGITAL
// 32767 600000 erpm
// 16383 300000 erpm
// 8192 150000 erpm
// 4096 75000 erpm
// 2048 37500 erpm
// 1024 18750 erpm, 8 pole pairs 2344 rpm

