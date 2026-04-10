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
#include "Peripheral/Analog/Analog_Reference.h"
#include "Linear_Voltage.h"

#include <stdint.h>
#include <stdbool.h>

/*  */
typedef const struct VDivider
{
    uint32_t R1;
    uint32_t R2;
    // uint16_t VREF_MILLIV; /* If different than global */
}
VDivider_T;

#define V_DIVIDER_INIT(RefR1, RefR2) (VDivider_T) { .R1 = (RefR1), .R2 = (RefR2) }

/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline void VDivider_ToLinear(const VDivider_T * p_voltage, Linear_T * p_linear)
{
    Linear_Voltage_Init(p_linear, p_voltage->R1, p_voltage->R2, ANALOG_REFERENCE.ADC_VREF_MILLIV, ANALOG_REFERENCE.ADC_BITS);
}

/******************************************************************************/
/*!
    Local Unit Conversion Wrapper
*/
/******************************************************************************/
// static inline int32_t Voltage_ScalarVOf(const Voltage_T * p_voltage, uint16_t adcu, uint16_t vScalar) { return Linear_Voltage_ScalarV(&p_voltage->Units, adcu, vScalar); }
// static inline int32_t Voltage_AdcuOfMilliV(const Voltage_T * p_voltage, uint32_t milliV)              { return Linear_Voltage_AdcuOfMilliV(&p_voltage->Units, milliV); }
// static inline int32_t Voltage_AdcuOfV(Voltage_T * p_voltage, uint16_t v)                              { return Linear_Voltage_AdcuOfV(&p_voltage->Units, v); }

/******************************************************************************/
/*!
*/
/******************************************************************************/
typedef enum VDivider_ConfigId
{
    VDIVIDER_BOARD_R1,
    VDIVIDER_BOARD_R2,
}
VDivider_ConfigId_T;

static inline int32_t _VDivider_ConfigId_Get(const VDivider_T * p_voltage, VDivider_ConfigId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case VDIVIDER_BOARD_R1: value = p_voltage->R1 / 10U;       break;
        case VDIVIDER_BOARD_R2: value = p_voltage->R2 / 10U;       break;
        default: break;
    }
    return value;
}

static inline int32_t VDivider_ConfigId_Get(const VDivider_T * p_voltage, VDivider_ConfigId_T id)
{
    return (p_voltage != NULL) ? _VDivider_ConfigId_Get(p_voltage, id) : 0;
}