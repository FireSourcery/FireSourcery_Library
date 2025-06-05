#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Voltage.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Peripheral/Analog/AnalogReference.h"
#include "Linear_Voltage.h"

#include <stdint.h>
#include <stdbool.h>

/*  */
typedef const struct VDivider
{
    const uint32_t R1;
    const uint32_t R2;
    // const uint16_t VREF_MILLIV; /* If different than global */
}
VDivider_T;

#define V_DIVIDER_INIT(RefR1, RefR2) { .R1 = (RefR1), .R2 = (RefR2) }


/******************************************************************************/
/*!
*/
/******************************************************************************/
typedef enum VDivider_RefId
{
    VDIVIDER_REF_R1,
    VDIVIDER_REF_R2,
}
VDivider_RefId_T;

static inline int32_t _VDivider_RefId_Get(const VDivider_T * p_voltage, VDivider_RefId_T configId)
{
    int32_t value = 0;
    switch (configId)
    {
        case VDIVIDER_REF_R1: value = p_voltage->R1 / 10U;       break;
        case VDIVIDER_REF_R2: value = p_voltage->R2 / 10U;       break;
        default: break;
    }
    return value;
}


static inline int32_t VDivider_RefId_Get(const VDivider_T * p_voltage, VDivider_RefId_T configId) { return (p_voltage != NULL) ? _VDivider_RefId_Get(p_voltage, configId) : 0; }

// static inline void _VDivider_RefId_Set(const VDivider_T * p_voltage, VDivider_RefId_T configId, uint16_t value)
// {
//     switch (configId)
//     {
//         case VDIVIDER_REF_R1:   break;
//         case VDIVIDER_REF_R2:   break;
//         default: break;
//     }
// }

// static inline void VDivider_RefId_Set(const VDivider_T * p_voltage, VDivider_RefId_T configId, uint16_t value) { if (p_voltage != NULL) { _VDivider_RefId_Set(p_voltage, configId, value); } }

/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline void VDivider_ToLinear(const VDivider_T * p_voltage, Linear_T * p_linear)
{
    Linear_Voltage_Init(p_linear, p_voltage->R1, p_voltage->R2, ANALOG_REFERENCE.ADC_VREF_MILLIV, ANALOG_REFERENCE.ADC_BITS);
}

// static inline  void VDivider_InitUnitsRef_Fract16(const VDivider_T * p_voltage, Linear_T * p_const)

// static inline void VDivider_ToLinearAsChargeLevel(const VDivider_T * p_voltage, int32_t vZero_milliV, int32_t vRef_milliV, Linear_T * p_linear)
// {

// }

/******************************************************************************/
/*!
    Local Unit Conversion Wrapper
*/
/******************************************************************************/
// static inline int32_t Voltage_ScalarVOf(const Voltage_T * p_voltage, uint16_t adcu, uint16_t vScalar) { return Linear_Voltage_ScalarV(&p_voltage->Units, adcu, vScalar); }
// static inline int32_t Voltage_AdcuOfMilliV(const Voltage_T * p_voltage, uint32_t milliV)              { return Linear_Voltage_AdcuOfMilliV(&p_voltage->Units, milliV); }
// static inline int32_t Voltage_AdcuOfV(Voltage_T * p_voltage, uint16_t v)                              { return Linear_Voltage_AdcuOfV(&p_voltage->Units, v); }

