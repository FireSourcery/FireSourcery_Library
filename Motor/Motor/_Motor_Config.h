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

    @brief inner config
*/
/******************************************************************************/
#include "Motor.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Speed Ref / Conversion
    Derived From Config
    DegPerCycle as Electrical
*/
/******************************************************************************/
static inline int16_t _Motor_DegOfRpm(const Motor_State_T * p_motor, accum32_t speed_rpm) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, speed_rpm); }
static inline int16_t _Motor_RpmOfDeg(const Motor_State_T * p_motor, accum32_t speed_degPerCycle) { return mech_rpm_of_el_angle(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, speed_degPerCycle); }

/*
    Speed/V relation based on Kv.
    Rpm of Kv * V
*/
static inline uint16_t Motor_RpmOfKv(const Motor_State_T * p_motor, uint16_t v_fract16) { return fract16_mul(v_fract16, (int32_t)p_motor->Config.Kv * Phase_Calibration_GetVMaxVolts()); }
static inline uint16_t Motor_VFract16OfKv(const Motor_State_T * p_motor, uint16_t rpm)  { return fract16_div(rpm, (int32_t)p_motor->Config.Kv * Phase_Calibration_GetVMaxVolts()); }
// static inline uint16_t Motor_GetKt(const Motor_State_T * p_motor) { 60 / (2 * pi * kV);  }

/*
   Calibration values derive with system config
*/
/*
    Speed VBus Ref
    Generally [SpeedRated] via Kv * V
*/
static inline uint16_t Motor_GetSpeedVNominalRef_Rpm(const Motor_State_T * p_motor) { return Motor_RpmOfKv(p_motor, Phase_VBus_GetVNominal()); }
static inline uint16_t Motor_GetSpeedVNominalRef_DegPerCycle(const Motor_State_T * p_motor) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, Motor_GetSpeedVNominalRef_Rpm(p_motor)); }

/* [SpeedRated] via direct Parameter */
/* can constrain Set with Kv on config */
/* Defined as VBusRef without config adjustment */
static inline uint16_t _Motor_GetSpeedRated_Rpm(const Motor_State_T * p_motor) { return Motor_GetSpeedVNominalRef_Rpm(p_motor); }
static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_State_T * p_motor) { return p_motor->Config.SpeedRated_Rpm; }
static inline uint32_t Motor_GetSpeedRated_ERpm(const Motor_State_T * p_motor) { return (uint32_t)p_motor->Config.SpeedRated_Rpm * p_motor->Config.PolePairs; }
static inline uint16_t Motor_GetSpeedRated_DegPerCycle(const Motor_State_T * p_motor) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, p_motor->Config.SpeedRated_Rpm); }

static inline uint16_t _Motor_GetVSpeedRated_Fract16(const Motor_State_T * p_motor) { return Motor_VFract16OfKv(p_motor, Motor_GetSpeedRated_Rpm(p_motor)); }

/*
    Num Max
*/
/*
    Numerical max as ~2x kv*V
    account for 1.15 factor and field weakening range.
*/
/* alternatively move static config with vbus nominal / angel max shift */
#ifndef MOTOR_SPEED_TYPE_MAX_RPM
#define MOTOR_SPEED_TYPE_MAX_RPM(...) ((uint32_t)Motor_GetSpeedRated_Rpm(__VA_ARGS__)*2)
#else /* Compile time def only */
#define SPEED_FRACT16_OF_RPM(Rpm)      (Rpm * INT16_MAX / MOTOR_SPEED_TYPE_MAX_RPM())
#define SPEED_RPM_OF_FRACT16(Fract16)  (Fract16 * MOTOR_SPEED_TYPE_MAX_RPM() / 32768)
#endif

/* alternatively move static config with vbus nominal */
static inline uint16_t Motor_GetSpeedTypeMax_Rpm(const Motor_State_T * p_motor) { return MOTOR_SPEED_TYPE_MAX_RPM(p_motor); }
static inline uint16_t Motor_GetSpeedTypeMax_DegPerCycle(const Motor_State_T * p_motor) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->Config.PolePairs, Motor_GetSpeedTypeMax_Rpm(p_motor)); }

/* Local Unit Conversion */
static inline accum32_t Motor_Speed_Fract16OfRpm(const Motor_State_T * p_motor, int16_t speed_rpm)      { return speed_rpm * INT16_MAX / Motor_GetSpeedTypeMax_Rpm(p_motor); }
static inline int16_t Motor_Speed_RpmOfFract16(const Motor_State_T * p_motor, accum32_t speed_fract16)  { return speed_fract16 * Motor_GetSpeedTypeMax_Rpm(p_motor) / 32768; }

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
/* Base in electrical domain */
// typedef struct Motor_Specs_ControlDomain
// {
//     uint16_t Kv_DegPerCyclePerVFract16;     /* Kv in control domain units */
//     uint16_t SpeedRated_DegPerCycle;        /* electrical degrees per control cycle */
// }Motor_Specs_ControlDomain_T;
// static inline uint32_t Motor_GetSpeedRated_DegPerCycle(const Motor_Specs_ControlDomain_T * p_motor) { return p_motor->SpeedRated_DegPerCycle; }
// static inline uint32_t Motor_GetSpeedRated_ERpm(const Motor_Specs_ControlDomain_T * p_motor) { return rpm_of_angle(MOTOR_CONTROL_FREQ, p_motor->SpeedRated_DegPerCycle); }
// static inline uint16_t Motor_GetSpeedRated_Rpm(const Motor_Specs_ControlDomain_T * p_motor) { return mech_rpm_of_el_angle(MOTOR_CONTROL_FREQ, p_motor->PolePairs, p_motor->SpeedRated_DegPerCycle); }


// static inline int16_t _Motor_DegOfRpm(const Motor_Config_T * p_motor, accum32_t speed_rpm) { return el_angle_of_mech_rpm(MOTOR_CONTROL_FREQ, p_motor->PolePairs, speed_rpm); }
// static inline int16_t _Motor_RpmOfDeg(const Motor_Config_T * p_motor, accum32_t speed_degPerCycle) { return mech_rpm_of_el_angle(MOTOR_CONTROL_FREQ, p_motor->PolePairs, speed_degPerCycle); }

// /*
//     Speed/V relation based on Kv.
//     Rpm of Kv * V
// */
// static inline uint16_t Motor_RpmOfKv(const Motor_Config_T * p_motor, uint16_t v_fract16) { return fract16_mul(v_fract16, (int32_t)p_motor->Kv * Phase_Calibration_GetVMaxVolts()); }
// static inline uint16_t Motor_VFract16OfKv(const Motor_Config_T * p_motor, uint16_t rpm) { return fract16_div(rpm, (int32_t)p_motor->Kv * Phase_Calibration_GetVMaxVolts()); }