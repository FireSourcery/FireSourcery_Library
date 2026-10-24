/*******************************************************************************/
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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file   Linear_Voltage.c
    @author FireSourcery
    @brief

*/
/*******************************************************************************/
#include "Linear_Voltage.h"

/******************************************************************************/
/*!
    @brief  Initialize Linear struct using Voltage Divider parameters.
            Use a constant shift instead of deriving it from max input, accounts for millivolt bounds.

    f(adcu) = voltage
    f16(adcu) = fract16
    invf(voltage) = adcu
    invf16(fract16)  = adcu

    VDIV = VIN*(R2/(R1+R2))
    DIV = (R2/(R1+R2))
    VADC = ADC_VREF/ADC_MAX

    ADC = VIN*DIV/VADC = VIN*(R2*ADC_MAX)/((R1+R2)*ADC_VREF)
    VIN = ADC*VADC/DIV = ADC*(ADC_VREF*(R1+R2))/(ADC_MAX*R2)
    VIN/ADC = VADC/DIV = ADC_VREF*(R1 + R2)/(ADC_MAX*R2)

    @param[in] line - Struct containing calculated intermediate values
    @param[in] r1 - R1 value expressed as a whole number, < 65536
    @param[in] r2 - R2 value expressed as a whole number, < 65536
    @param[in] adcBit - Number of ADC bits
    @param[in] adcVRef - ADC reference voltage
*/
/******************************************************************************/
void Linear_Voltage_Init(Linear_T * p_linear, uint32_t r1, uint32_t r2, uint16_t adcVRef_MilliV, uint8_t adcBits)
{
    p_linear->Slope             = (((uint64_t)adcVRef_MilliV * (r1 + r2)) << (LINEAR_VOLTAGE_SHIFT - adcBits)) / ((uint64_t)r2 * 1000U); /* (ADC_VREF*(R1 + R2) << 15)/(ADC_MAX*R2) */
    p_linear->SlopeShift        = LINEAR_VOLTAGE_SHIFT;
    p_linear->X0                = 0;
    p_linear->XDelta            = 1UL << adcBits;
}

// @param[in] vInRef - Scalar Fract16 reference
// void Linear_Voltage_Init (Linear_T * p_linear, uint32_t r1, uint32_t r2, uint16_t adcVRef_MilliV, uint8_t adcBits, uint16_t VInRef)
// {

// }