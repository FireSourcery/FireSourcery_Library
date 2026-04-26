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
    @file   SinCos.h
    @author FireSourcery
    @brief  Analog Sin/Cos resolver decode. Quadrature analog inputs -> mech angle.
            Pure decode core; sensor adapter (RotorSensor VTable) lives in SinCos_Sensor.h.
*/
/******************************************************************************/
#include "Peripheral/Analog/Linear_ADC.h"
#include "Math/Linear/Linear_Q16.h"
#include "Math/Fixed/fract16.h"

#include <stdbool.h>
#include <stdint.h>

/*
    [SinCos_Config_T]
    f(adcuMin..adcuMax) -> [-32768..32767] fract16, then atan2 -> angle16
*/
typedef struct SinCos_Config
{
    uint16_t Min_Adcu;
    uint16_t Max_Adcu;
    uint16_t Min_MilliV;
    uint16_t Max_MilliV;
    uint16_t ElectricalRotationsRatio;  /* PolePairs / CyclesPerMechRotation */
    angle16_t AngleOffset;              /* Sensor-to-rotor zero offset, subtracted on capture */
    bool IsCcwPositive;                 /* Calibrates virtual CCW as increasing angle */
}
SinCos_Config_T;

/*
    [SinCos_State_T]
*/
typedef struct SinCos_State
{
    SinCos_Config_T Config;
    Linear_T UnitsAngle;        /* ADC -> fract16 scaling, derived from Config Min/Max */
    angle16_t Angle;            /* Last captured mechanical angle (post-offset, post-sign) */
}
SinCos_State_T;

/*
    [SinCos_T] const handle.
    Decode core only — ADC sourcing lives in the sensor adapter (SinCos_Sensor).
*/
typedef const struct SinCos
{
    SinCos_State_T * P_STATE;
    const SinCos_Config_T * P_NVM_CONFIG;
}
SinCos_T;

#define SIN_COS_STATE_ALLOC() (&(SinCos_State_T){0})

#define SIN_COS_INIT(p_State, p_Config) (SinCos_T) { .P_STATE = (p_State), .P_NVM_CONFIG = (p_Config), }


/******************************************************************************/
/*
    Inline decode — caller activates ADC, passes raw counts.
*/
/******************************************************************************/
/* Raw atan2 decode, no offset/sign correction */
static inline angle16_t _SinCos_DecodeAngle(const SinCos_State_T * p_state, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
    fract16_t sin = Linear_Q16_Fract(&p_state->UnitsAngle, sin_Adcu);
    fract16_t cos = Linear_Q16_Fract(&p_state->UnitsAngle, cos_Adcu);
    return fract16_atan2(sin, cos);
}

/* Calibrated mechanical angle: offset-shifted, sign-corrected to virtual CCW-positive */
static inline angle16_t SinCos_CaptureAngle(SinCos_State_T * p_state, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
    angle16_t angle = _SinCos_DecodeAngle(p_state, sin_Adcu, cos_Adcu) - p_state->Config.AngleOffset;
    p_state->Angle = p_state->Config.IsCcwPositive ? angle : (angle16_t)(0 - angle);
    return p_state->Angle;
}

/******************************************************************************/
/*
    Query
*/
/******************************************************************************/
static inline angle16_t SinCos_GetMechanicalAngle(const SinCos_State_T * p_state) { return p_state->Angle; }

/* Project mech -> electrical via configured ratio */
static inline angle16_t SinCos_GetElectricalAngle(const SinCos_State_T * p_state)
{
    return (angle16_t)((int32_t)p_state->Angle * p_state->Config.ElectricalRotationsRatio);
}


/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void SinCos_Init(const SinCos_T * p_sincos);
extern void SinCos_State_ResetUnits(SinCos_State_T * p_state);
extern void SinCos_Config_SetAngleRatio(SinCos_State_T * p_state, uint16_t electricalRotationsRatio);
extern void SinCos_Config_SetAdc(SinCos_State_T * p_state, uint16_t min_Adcu, uint16_t max_Adcu, uint16_t min_mV, uint16_t max_mV);
extern void SinCos_Config_SetAdc_mV(SinCos_State_T * p_state, uint16_t adcVref_mV, uint16_t min_mV, uint16_t max_mV);
extern void SinCos_CalibrateAngleOffset(SinCos_State_T * p_state, uint16_t sin_Adcu, uint16_t cos_Adcu);
extern void SinCos_CalibrateCcwPositive(SinCos_State_T * p_state, uint16_t sin_Adcu, uint16_t cos_Adcu);


/******************************************************************************/
/*
    Var ID — generic config access
*/
/******************************************************************************/
typedef enum SinCos_ConfigId
{
    SIN_COS_CONFIG_MIN_ADCU,
    SIN_COS_CONFIG_MAX_ADCU,
    SIN_COS_CONFIG_MIN_MILLIV,
    SIN_COS_CONFIG_MAX_MILLIV,
    SIN_COS_CONFIG_ANGLE_OFFSET,
    SIN_COS_CONFIG_IS_CCW_POSITIVE,
    SIN_COS_CONFIG_ELECTRICAL_ROTATIONS_RATIO,
}
SinCos_ConfigId_T;

extern int SinCos_ConfigId_Get(const SinCos_State_T * p_state, SinCos_ConfigId_T id);
extern void SinCos_ConfigId_Set(SinCos_State_T * p_state, SinCos_ConfigId_T id, int value);
