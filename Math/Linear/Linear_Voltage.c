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
    @version V0
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
    @param[in] vInRef - Fract16 reference
*/
/******************************************************************************/
void Linear_Voltage_Init(Linear_T * p_linear, uint32_t r1, uint32_t r2, uint16_t adcVRef_MilliV, uint8_t adcBits, uint16_t vInRef)
{
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
    p_linear->Slope             = (((uint64_t)adcVRef_MilliV * (r1 + r2)) << (LINEAR_VOLTAGE_SHIFT - adcBits)) / r2 / 1000U; /* (ADC_VREF*(R1 + R2) << 16)/(ADC_MAX*R2) */
    // p_linear->Slope             = math_muldiv64_unsigned((r1 + r2), math_muldiv64_unsigned(adcVRef_MilliV, ((uint32_t)1UL << LINEAR_VOLTAGE_SHIFT), ((uint32_t)1UL << adcBits)), r2 * 1000U); /* (ADC_VREF*(R1 + R2) << 16)/(ADC_MAX*R2) */
    p_linear->SlopeShift        = LINEAR_VOLTAGE_SHIFT;
    p_linear->InvSlope          = ((uint64_t)r2 << LINEAR_VOLTAGE_SHIFT) * 1000U / adcVRef_MilliV / (r1 + r2); /* (R2 << 16)/(ADC_VREF*(R1 + R2)) */
    p_linear->InvSlopeShift     = LINEAR_VOLTAGE_SHIFT - adcBits;
#elif defined (CONFIG_LINEAR_DIVIDE_NUMERICAL)
    p_linear->SlopeFactor       = (uint64_t)adcVRef_MilliV * (r1 + r2) / 1000U; /* (ADC_VREF*(R1+R2)) */
    p_linear->SlopeDivisor      = (((uint64_t)1U << adcBits) - 1U) * r2;       /* (ADC_MAX*R2) */
#endif
    p_linear->X0                = 0;
    p_linear->Y0                = 0;
    p_linear->YReference        = vInRef;
    p_linear->XReference        = linear_invf(p_linear->Slope, ((uint32_t)1U << LINEAR_VOLTAGE_SHIFT), 0, vInRef); /* Fract16 reference */
    p_linear->XDeltaRef         = p_linear->XReference - p_linear->X0;
    p_linear->YDeltaRef         = vInRef - p_linear->Y0;
}

