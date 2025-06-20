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
    @file   MotorAnalog.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "MotorAnalog.h"

/* Set before Motor_Init */
static uint16_t VSource_Fract16; /* VBus */
static uint32_t VSourceInvScalar;
// static uint16_t VSourceNominal_V;

void MotorAnalog_InitVSource_V(uint16_t vSource_V)
{
    // VSourceNominal_V = vSource_V;
    VSource_Fract16 = fract16(vSource_V, MotorAnalogRef_GetVMaxVolts());
}

/* Using the active read value */
void MotorAnalog_CaptureVSource_Adcu(uint16_t vSource_Adcu)
{
    VSource_Fract16 = vSource_Adcu << MOTOR_ANALOG_V_FRACT16_SHIFT;
    VSourceInvScalar = FRACT16_MAX * 65536U / VSource_Fract16; /* shift 16 as fract32 */
}

uint16_t MotorAnalog_GetVSource_Fract16(void)  { return VSource_Fract16; }
uint16_t MotorAnalog_GetVSource_V(void)        { return fract16_mul(VSource_Fract16, MotorAnalogRef_GetVMaxVolts()); }
uint16_t MotorAnalog_GetVSourceInvScalar(void) { return VSourceInvScalar; }
// uint16_t MotorAnalog_GetVSource_Adcu(void)     { return VSource_Fract16 >> MOTOR_ANALOG_V_FRACT16_SHIFT; }

// uint16_t MotorAnalog_GetVSourceNominal_V(void) { return VSourceNominal_V; }

// void InitUnitsVSource(void)
// {
//     Linear_Voltage_Init(&UnitsVSource_V, MOTOR_ANALOG_REFERENCE.V_ABC_R1, MOTOR_ANALOG_REFERENCE.V_ABC_R2, ANALOG_REFERENCE.ADC_VREF_MILLIV, ANALOG_REFERENCE.ADC_BITS);
// }


