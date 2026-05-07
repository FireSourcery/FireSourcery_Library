#pragma once
/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Motor_Config.h
    @author FireSourcery

    @brief  Config Fields interface
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "Math/motor_params_math.h"

/******************************************************************************/
/*
    Helpers
*/
/******************************************************************************/
static inline int16_t _Motor_AngleOfRpm(const Motor_Config_T * p_config, accum32_t speed_rpm) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_config->PolePairs, speed_rpm); }
static inline int16_t _Motor_RpmOfAngle(const Motor_Config_T * p_config, accum32_t speed_degPerCycle) { return mech_rpm_of_el_angle(MOTOR_CONTROL_FREQ, p_config->PolePairs, speed_degPerCycle); }

/*
    Speed/V relation based on Kv.
    Rpm of Kv * V
*/
static inline uint16_t Motor_RpmOfKv(const Motor_Config_T * p_config, uint16_t v_fract16) { return fract16_mul(v_fract16, (int32_t)p_config->Kv * Phase_Calibration_GetVMaxVolts()); }
static inline uint16_t Motor_VFract16OfKv(const Motor_Config_T * p_config, uint16_t rpm) { return fract16_div(rpm, (int32_t)p_config->Kv * Phase_Calibration_GetVMaxVolts()); }


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
static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_Config_T * p_config); /* forward declare depedency, for runtime derived */
#define MOTOR_SPEED_TYPE_MAX_RPM(p_config) ((uint32_t)Motor_GetSpeedRated_Rpm(p_config)*2)
#define SPEED_FRACT16(Rpm, RpmMax) FRACT16((float)Rpm / RpmMax)
#else /* Compile time def only */
#define SPEED_FRACT16_OF_RPM(Rpm)      (Rpm * INT16_MAX / MOTOR_SPEED_TYPE_MAX_RPM())
#define SPEED_RPM_OF_FRACT16(Fract16)  (Fract16 * MOTOR_SPEED_TYPE_MAX_RPM() / 32768)
#endif


static inline uint16_t Motor_GetSpeedTypeMax_Rpm(const Motor_Config_T * p_config) { return MOTOR_SPEED_TYPE_MAX_RPM(p_config); }
static inline uint16_t Motor_GetSpeedTypeMax_Angle(const Motor_Config_T * p_config) { return _Motor_AngleOfRpm(p_config, Motor_GetSpeedTypeMax_Rpm(p_config)); }

/* Local Unit Conversion */
static inline accum32_t Motor_Speed_Fract16OfRpm(const Motor_Config_T * p_config, int16_t speed_rpm) { return speed_rpm * INT16_MAX / Motor_GetSpeedTypeMax_Rpm(p_config); }
static inline int16_t Motor_Speed_RpmOfFract16(const Motor_Config_T * p_config, accum32_t speed_fract16) { return speed_fract16 * Motor_GetSpeedTypeMax_Rpm(p_config) / 32768; }


/* [SpeedRated_Rpm] = [KvVBusNominal] = [SpeedTypeMax_Rpm] / 2  */
/* Keep instanced */
static inline uint16_t Motor_GetSpeedRated_Fract16(const Motor_Config_T * p_config) { (void)p_config; return INT16_MAX / 2; }

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    Speed VBus Ref
    Generally [SpeedRated] via Kv * V
*/
/* Motor_GetKvVBus_Rpm */
static inline uint16_t Motor_GetSpeedVNominalRef_Rpm(const Motor_Config_T * p_config) { return Motor_RpmOfKv(p_config, Phase_VBus_GetVNominal()); }
static inline uint16_t Motor_GetSpeedVNominalRef_Angle(const Motor_Config_T * p_config) { return _Motor_AngleOfRpm(p_config, Motor_GetSpeedVNominalRef_Rpm(p_config)); }

static inline uint16_t Motor_SpeedVRef_Rpm(const Motor_Config_T * p_config, uint16_t vNominal_fract16) { return Motor_RpmOfKv(p_config, vNominal_fract16); }
static inline uint16_t Motor_SpeedVRef_Angle(const Motor_Config_T * p_config, uint16_t vNominal_fract16) { return _Motor_AngleOfRpm(p_config, Motor_SpeedVRef_Rpm(p_config, vNominal_fract16)); }


/*
    [SpeedRated] via direct Parameter
    Simplify user set and decouples from VBus module
    Config set Kv mediates
*/
static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_Config_T * p_config) { return p_config->SpeedRated_Rpm; }
static inline uint32_t Motor_GetSpeedRated_ERpm(const Motor_Config_T * p_config) { return (uint32_t)p_config->SpeedRated_Rpm * p_config->PolePairs; }
static inline uint16_t Motor_GetSpeedRated_Angle(const Motor_Config_T * p_config) { return _Motor_AngleOfRpm(p_config, Motor_GetSpeedRated_Rpm(p_config)); }

static inline uint16_t Motor_GetEffectiveSpeedRated_Rpm(const Motor_Config_T * p_motor, uint16_t volts) { return fract16_mul(p_motor->VSpeedScalar_Fract16, (int32_t)p_motor->Kv * volts); }


/* [V / (rad/s)] */
static inline accum32_t Motor_GetKe_Fract32(const Motor_Config_T * p_config) { return (int64_t)FRACT32_SCALE * 60 / (2 * FRACT16_PI * p_config->Kv); }

/* [VFract16 / (Angle16/ControlCycle)] */
static inline accum32_t Motor_GetKe_Angle16(const Motor_Config_T * p_config) { return Motor_VFract16OfKv(p_config, Motor_GetSpeedRated_Rpm(p_config)) / Motor_GetSpeedRated_Angle(p_config); }

/*
    [VFract16 / SpeedFract16]
    V_fract16 = fract16_mul(Ke_SpeedFract, Speed_fract16)
    where omega_fract16 = ω / ω_max × FRACT16_MAX

    when SpeedTypeMax = Kv * VNominal * 2
    Ke = VNominal * 2
*/
// static inline accum32_t Motor_GetKe_SpeedFract16(const Motor_Config_T * p_config, uint16_t vNominal_fract16) { return fract16_div(vNominal_fract16, Motor_GetSpeedRated_Fract16(p_config)); }
static inline accum32_t Motor_GetKe_SpeedFract16(const Motor_Config_T * p_config) { return fract16_div(Motor_VFract16OfKv(p_config, Motor_GetSpeedRated_Rpm(p_config)), Motor_GetSpeedRated_Fract16(p_config)); }


// FluxLinkage [Wb] = Ke_mech / PolePairs
// λm = 60 / (2π · Kv · pp · VMax)
static inline accum32_t Motor_GetFluxLinkage_Angle16(const Motor_Config_T * p_config) { return Motor_GetKe_Angle16(p_config) / p_config->PolePairs; }

// V_fract16 = fract16_mul(FluxLinkage_SpeedFract16, speedElec_fract16)
static inline accum32_t Motor_GetFluxLinkage_SpeedFract16(const Motor_Config_T * p_config) { return Motor_GetKe_SpeedFract16(p_config) / p_config->PolePairs; }

// static inline uint16_t _Motor_GetVSpeedRated_Fract16(const Motor_Config_T * p_config) { return Motor_VFract16OfKv(p_config, Motor_GetSpeedRated_Rpm(p_config)); }
// static inline uint16_t Motor_GetVSpeedRated_Fract16(const Motor_Config_T * p_config, const VBus_Config_T * p_vbus) { (void)p_config; return VBus_VNominal_Fract16(p_vbus); }
// static inline uint16_t Motor_GetSpeedVMatch_(const Motor_Config_T * p_motor) { return fract16_mul(p_motor->VSpeedScalar_Fract16, Motor_GetSpeedRated_Fract16(p_motor)); }

/******************************************************************************/
/*
    decoupling K's
*/
/******************************************************************************/

static void Motor_ResolveDecouplingCoeffs(Motor_Config_T * p_config)
{
// #if defined(MOTOR_DECOUPLE_ENABLE)
    p_config->Decoupling.KLd_Fract16 = kl_fract16_of_uh(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_config->ElectricalParams.Ld_MicroHenries);
    p_config->Decoupling.KLq_Fract16 = kl_fract16_of_uh(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_config->ElectricalParams.Lq_MicroHenries);
    p_config->Decoupling.KPsi_Fract16 = kpsi_fract16_of_kv(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), p_config->Kv);
// #endif
}

/******************************************************************************/
/*
    OpenLoop
*/
/******************************************************************************/
/* symetrical limit */
static inline uint16_t Motor_OpenLoopILimit(const Motor_Config_T * p_motor) { return fract16_mul(p_motor->OpenLoopLimitScalar_Fract16, p_motor->ILimitMotoring_Fract16); }
static inline int16_t Motor_OpenLoopILimitOf(const Motor_Config_T * p_motor, int16_t iReq) { return math_clamp(iReq, (int32_t)0 - Motor_OpenLoopILimit(p_motor), Motor_OpenLoopILimit(p_motor)); }

/* alternatively seperate IAlign and Vduty */
static inline uint16_t Motor_GetIAlign(const Motor_Config_T * p_motor) { return fract16_mul(p_motor->AlignScalar_Fract16, p_motor->ILimitMotoring_Fract16); }


/* fract16_div((uint32_t)Motor_GetVAlign(p_motor) * 2 / 3, Phase_VBus_GetVRef()) */
static inline uint16_t Motor_GetVAlign_Duty(const Motor_Config_T * p_motor) { return (uint32_t)p_motor->AlignScalar_Fract16 * 3 / 4; }

/*  */
// static inline uint16_t Motor_OpenLoopVLimit(const Motor_Config_T * p_motor) { return fract16_mul(p_motor->OpenLoopLimitScalar_Fract16, Phase_VBus_GetVRef()); }
// static inline int16_t Motor_OpenLoopVLimitOf(const Motor_Config_T * p_motor, int16_t vReq) { return math_clamp(vReq, (int32_t)0 - Motor_OpenLoopVLimit(p_motor), Motor_OpenLoopVLimit(p_motor)); }
static inline uint16_t Motor_GetVAlign(const Motor_Config_T * p_motor, const VBus_T * p_vbus) { return fract16_mul(p_motor->AlignScalar_Fract16, VBus_Fract16(p_vbus)); }


// static inline uint16_t Motor_GetSpeedFreewheelLimit_UFract16(const Motor_Config_T * p_config) { return fract16_mul(Motor_GetSpeedRated_Fract16(p_config), FRACT16_2_DIV_SQRT3); }
static inline uint16_t Motor_GetSpeedFreewheelLimit_UFract16(const Motor_Config_T * p_config) { return Motor_GetSpeedRated_Fract16(p_config); }
static inline bool Motor_IsSpeedFreewheelLimit(const Motor_Config_T * p_config, accum32_t speed_fract16) { return (math_abs(speed_fract16) < Motor_GetSpeedFreewheelLimit_UFract16(p_config)); }

/******************************************************************************/
/*!
    V <=> Speed conversion based on Kv.
    Speed/SpeedRated => V/VBusRef
    @return V_Fract16 - VBusRef of Speed Kv
*/
/******************************************************************************/
/* when SpeedTypeMax = Kv * VNominal * 2 */
static inline accum32_t Motor_VBusOfSpeed_Fract16(const Motor_Config_T * p_config, accum32_t speed_fract16) { return Phase_VBus_GetVNominal() * speed_fract16 / Motor_GetSpeedRated_Fract16(p_config); }
static inline accum32_t Motor_SpeedOfVBus_Fract16(const Motor_Config_T * p_config, accum32_t v_fract16) { return v_fract16 * Motor_GetSpeedRated_Fract16(p_config) / Phase_VBus_GetVNominal(); }

/* Phase peak. */
static inline accum32_t Motor_VPhaseOfSpeed_Fract16(const Motor_Config_T * p_config, accum32_t speed_fract16) { (void)p_config; return fract16_mul(Phase_VBus_GetVNominal(), speed_fract16); }

/******************************************************************************/
/*
   Validate
*/
/******************************************************************************/
/* scale to rated max */
#ifndef MOTOR_OPEN_LOOP_CEILING
#define MOTOR_OPEN_LOOP_CEILING FRACT16(0.1F)
#endif

// #ifndef MOTOR_OPEN_LOOP_V_CEILING
// #define MOTOR_OPEN_LOOP_V_CEILING FRACT16(0.1F)
// #endif


/*
    Rated Limit - applied on Config Bounds
    the setting limit by TypeMax, SpeedRated ~VBusRef
    When SpeedTypeMax is scaled, INT16_MAX is 2x RatedSpeed
*/
static inline uint16_t Motor_IRatedLimitOf(uint16_t i_fract16) { return math_min(Phase_Calibration_GetIRatedPeak_Fract16(), i_fract16); }
static inline uint16_t Motor_VRatedLimitOf(uint16_t v_fract16) { return math_min(Phase_Calibration_GetVRated_Fract16(), v_fract16); }
static inline uint16_t _Motor_SpeedRatedLimit(const Motor_Config_T * p_config) { (void)p_config; return FRACT16_1_DIV_SQRT3; }
static inline uint16_t _Motor_SpeedRatedLimitFw(const Motor_Config_T * p_config) { (void)p_config; return INT16_MAX; }
static inline uint16_t Motor_SpeedRatedLimitOf(const Motor_Config_T * p_config, uint16_t speed_fract16) { return math_min(_Motor_SpeedRatedLimit(p_config), speed_fract16); }

/* local limit */
static inline uint16_t _Motor_GetOpenLoopScalarLimit(const Motor_Config_T * p_config) { return math_min(p_config->OpenLoopLimitScalar_Fract16, MOTOR_OPEN_LOOP_CEILING); }
static inline uint16_t _Motor_GetOpenLoopILimit_Fract16(const Motor_Config_T * p_config) { return fract16_mul(_Motor_GetOpenLoopScalarLimit(p_config), Phase_Calibration_GetIRatedPeak_Fract16()); }
static inline uint16_t _Motor_GetOpenLoopVLimit_Fract16(const Motor_Config_T * p_config) { return fract16_mul(_Motor_GetOpenLoopScalarLimit(p_config), Phase_Calibration_GetVRated_Fract16()); }
static inline uint16_t _Motor_OpenLoopIRatedLimitOf(const Motor_Config_T * p_config, uint16_t fract16) { return math_min(fract16, _Motor_GetOpenLoopILimit_Fract16(p_config)); }
static inline uint16_t _Motor_OpenLoopSpeedRatedLimitOf(const Motor_Config_T * p_config, uint16_t fract16) { return math_min(fract16, _Motor_SpeedRatedLimit(p_config) / 2); }


static inline void Motor_Config_Validate(Motor_Config_T * p_config)
{
    p_config->SpeedRated_Rpm              = math_min(p_config->SpeedRated_Rpm, Motor_RpmOfKv(p_config, Phase_Calibration_GetVRated_Fract16()));
    p_config->VSpeedScalar_Fract16        = math_min(p_config->VSpeedScalar_Fract16, INT16_MAX);

    p_config->SpeedLimitForward_Fract16   = Motor_SpeedRatedLimitOf(p_config, p_config->SpeedLimitForward_Fract16);
    p_config->SpeedLimitReverse_Fract16   = Motor_SpeedRatedLimitOf(p_config, p_config->SpeedLimitReverse_Fract16);
    p_config->ILimitMotoring_Fract16      = Motor_IRatedLimitOf(p_config->ILimitMotoring_Fract16);
    p_config->ILimitGenerating_Fract16    = Motor_IRatedLimitOf(p_config->ILimitGenerating_Fract16);

    p_config->OpenLoopLimitScalar_Fract16 = math_min(p_config->OpenLoopLimitScalar_Fract16, MOTOR_OPEN_LOOP_CEILING);
    p_config->AlignScalar_Fract16         = math_min(p_config->AlignScalar_Fract16, MOTOR_OPEN_LOOP_CEILING);
    p_config->OpenLoopRampIFinal_Fract16        = _Motor_OpenLoopIRatedLimitOf(p_config, p_config->OpenLoopRampIFinal_Fract16);
    p_config->OpenLoopRampSpeedFinal_Fract16    = _Motor_OpenLoopSpeedRatedLimitOf(p_config, p_config->OpenLoopRampSpeedFinal_Fract16);
}

static inline bool Motor_Config_IsValid(const Motor_Config_T * p_config)
{
    return
    (
        (p_config->DirectionForward != MOTOR_DIRECTION_NULL) &&
        (p_config->IabcZeroRef_Adcu.A != 0U) &&
        (p_config->IabcZeroRef_Adcu.B != 0U) &&
        (p_config->IabcZeroRef_Adcu.C != 0U) &&
        (p_config->PolePairs != 0U)
        && (p_config->Kv != 0U)
        && (p_config->SpeedRated_Rpm != 0U)
        && (p_config->SpeedRated_Rpm              <= Motor_GetSpeedVNominalRef_Rpm(p_config) * 2)
        && (p_config->VSpeedScalar_Fract16        <= INT16_MAX)
        && (p_config->SpeedLimitForward_Fract16   <= _Motor_SpeedRatedLimit(p_config))
        && (p_config->SpeedLimitReverse_Fract16   <= _Motor_SpeedRatedLimit(p_config))
        && (p_config->ILimitMotoring_Fract16      <= Phase_Calibration_GetIRatedPeak_Fract16())
        && (p_config->ILimitGenerating_Fract16    <= Phase_Calibration_GetIRatedPeak_Fract16())
        && (p_config->OpenLoopLimitScalar_Fract16 <= MOTOR_OPEN_LOOP_CEILING)
        && (p_config->AlignScalar_Fract16         <= MOTOR_OPEN_LOOP_CEILING)
        && (p_config->OpenLoopRampIFinal_Fract16  <= _Motor_GetOpenLoopILimit_Fract16(p_config))
    );
}

static inline void Motor_Config_ValidateFw(Motor_Config_T * p_config)
{
    uint16_t speedCeiling = (p_config->FieldWeakening.IdFwMax > 0U) ? (uint16_t)INT16_MAX : _Motor_SpeedRatedLimit(p_config);

    p_config->SpeedLimitForward_Fract16             = math_min(speedCeiling, p_config->SpeedLimitForward_Fract16);
    p_config->SpeedLimitReverse_Fract16             = math_min(speedCeiling, p_config->SpeedLimitReverse_Fract16);
    p_config->FieldWeakening.IdFwMax        = Motor_IRatedLimitOf(p_config->FieldWeakening.IdFwMax);
}

static inline bool Motor_Config_IsValidFw(const Motor_Config_T * p_config)
{
    return
    (
        (p_config->SpeedLimitForward_Fract16 <= ((p_config->FieldWeakening.IdFwMax > 0U) ? (uint16_t)INT16_MAX : _Motor_SpeedRatedLimit(p_config)))
        && (p_config->SpeedLimitReverse_Fract16 <= ((p_config->FieldWeakening.IdFwMax > 0U) ? (uint16_t)INT16_MAX : _Motor_SpeedRatedLimit(p_config)))
    );
}



/*

*/
// static inline void Motor_Config_SetDirectionCalibration(Motor_Config_T * p_motor, Motor_Direction_T forward) { if (forward != MOTOR_DIRECTION_NULL) { p_motor->DirectionForward = forward; } }
// static inline bool Motor_Config_IsCcwPositive(Motor_Config_T * p_motor) { return p_motor->DirectionForward == MOTOR_DIRECTION_CCW; }
// static inline void Motor_Config_SetCcwPositive(Motor_Config_T * p_motor, bool isCcwPositive) { p_motor->DirectionForward = (isCcwPositive) ? MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW; }




/*



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

// typedef struct Motor_SpeedConfig_ControlDomain
// {
//     uint16_t Kv ;     /* Kv in control domain units */
//     uint16_t SpeedRated ;        /* electrical degrees per control cycle */
// } Motor_SpeedConfig_ControlDomain_T;

/* Base in electrical domain */
// typedef struct Motor_SpeedConfig_ControlDomain
// {
//     uint16_t Kv_DegPerCyclePerVFract16;     /* Kv in control domain units */
//     uint16_t SpeedRated_DegPerCycle;        /* electrical degrees per control cycle */
// }Motor_SpeedConfig_ControlDomain_T;
// static inline uint32_t Motor_GetSpeedRated_Angle(const Motor_SpeedConfig_ControlDomain_T * p_config) { return p_config->SpeedRated_DegPerCycle; }
// static inline uint32_t Motor_GetSpeedRated_ERpm(const Motor_SpeedConfig_ControlDomain_T * p_config) { return rpm_of_angle(MOTOR_CONTROL_FREQ, p_config->SpeedRated_DegPerCycle); }
// static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_SpeedConfig_ControlDomain_T * p_config) { return mech_rpm_of_el_angle(MOTOR_CONTROL_FREQ, p_config->PolePairs, p_config->SpeedRated_DegPerCycle); }


// #if defined(MOTOR_UNIT_CONVERSION_LOCAL) && defined(MOTOR_SURFACE_SPEED_ENABLE)
// uint16_t SurfaceDiameter;
// uint16_t GearRatioOutput;
// uint16_t GearRatioInput;
// #endif