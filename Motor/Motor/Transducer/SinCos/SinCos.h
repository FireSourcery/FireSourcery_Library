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
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef SIN_COS_H
#define SIN_COS_H

#include "Math/Linear/Linear_ADC.h"
#include "Math/Fixed/fract16.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct __attribute__((aligned(2U)))
{
    uint16_t Min_Adcu;
    uint16_t Max_Adcu;
    uint16_t Min_MilliV;
    uint16_t Max_MilliV;
    uint16_t ElectricalRotationsRatio; /* = PolePairs / CyclesPerRotation */
//    uint16_t CyclePerMechRotation;
    qangle16_t AngleOffet;
    bool IsCcwPositive;        /* Calibrates Ccw as positive */
}
SinCos_Config_T;

typedef const struct SinCos_Const
{
    const SinCos_Config_T * const P_CONFIG;
}
SinCos_Const_T;

typedef struct
{
    SinCos_Const_T CONST;
    SinCos_Config_T Config;
    Linear_T UnitsAngle;
    qangle16_t Angle; /* Sensor Output Angle, Mechanical Angle or proportional */
    // bool IsDirectionPositive;

    // qangle16_t DebugAPre;
    // qangle16_t DebugBPre;
    // qangle16_t DebugAPostMech;
    // qangle16_t DebugBPostMech;
    // qangle16_t DebugAPostElec;
    // qangle16_t DebugBPostElec;
}
SinCos_T;

#define SIN_COS_INIT(p_Config) { .CONST = { .P_CONFIG = p_Config, } }

/*
    Activate Adc outside module
*/
static inline qangle16_t _SinCos_CalcAngle(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
    qfrac16_t sin = Linear_ADC_Frac16(&p_sincos->UnitsAngle, sin_Adcu);
    qfrac16_t cos = Linear_ADC_Frac16(&p_sincos->UnitsAngle, cos_Adcu);
    qangle16_t angle = qfrac16_atan2(sin, cos);
    return angle;
}

/*
    Calibration set to return CCW as positive
*/
static inline qangle16_t SinCos_CaptureAngle(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu)
{
    qangle16_t angle = _SinCos_CalcAngle(p_sincos, sin_Adcu, cos_Adcu);
    angle = angle - p_sincos->Config.AngleOffet; /* move to sinCos calc */
    if(p_sincos->Config.IsCcwPositive == false) { angle = 0 - angle; };
    p_sincos->Angle = angle; //need counter to add offset if multiple cycles per rotation
    return angle;
}

static inline qangle16_t SinCos_GetMechanicalAngle(SinCos_T * p_sincos) { return p_sincos->Angle; }
/* effectively modulus angle max */
static inline qangle16_t SinCos_GetElectricalAngle(SinCos_T * p_sincos) { return (qangle16_t)((int32_t)p_sincos->Angle * p_sincos->Config.ElectricalRotationsRatio); }

/*
    CCW is positive
*/
// static inline void SinCos_SetDirectionCcw(SinCos_T * p_sincos)     { p_sincos->IsDirectionPositive = p_sincos->Config.IsCcwPositive; }
// static inline void SinCos_SetDirectionCw(SinCos_T * p_sincos)     { p_sincos->IsDirectionPositive = !p_sincos->Config.IsCcwPositive; }

/*
    Extern declarations
*/
extern void SinCos_Init(SinCos_T * p_sincos);
extern void SinCos_ResetUnitsAngle(SinCos_T * p_sincos);
extern void SinCos_SetAngleRatio(SinCos_T * p_sincos, uint16_t polePairs);
extern void SinCos_SetConfigAdc(SinCos_T * p_sincos, uint16_t min_Adcu, uint16_t max_Adcu, uint16_t min_mV, uint16_t max_mV);
extern void SinCos_SetConfigAdc_mV(SinCos_T * p_sincos, uint16_t adcVref_mV, uint16_t min_mV, uint16_t max_mV);
extern void SinCos_CalibrateAngleOffset(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);
extern void SinCos_CalibrateCcwPositive(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);
extern void SinCos_CalibrateA(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);
extern void SinCos_CalibrateB(SinCos_T * p_sincos, uint16_t sin_Adcu, uint16_t cos_Adcu);

#endif

