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
    @file   Linear_ADC.h
    @author FireSourcery
    @brief  Scale ADCU to provided reference value

*/
/******************************************************************************/
#ifndef LINEAR_ADC_H
#define LINEAR_ADC_H

#include "Linear_Q16.h"
#include <stdint.h>

#define VSAMPLE_OF_ADCU(Gain, VRef_MilliV, AdcBits, Adcu) ((VRef_MilliV * Adcu / AdcBits) / (Gain))
#define AMPS_OF_ADCU(Shunt, Gain, VRef_MilliV, AdcBits, Adcu) (VSAMPLE_OF_ADCU(Gain, VRef_MilliV, AdcBits, Adcu) / (Shunt))

/******************************************************************************/
/*!
    Init as Const
    f(adcuZero) = 0
    f([adcuZero - (2^adcBits - adcuZero)):adcuZero + (2^adcBits - adcuZero)]) = [-32768:32768]

    multiply +/- 1 and shift
*/
/******************************************************************************/
#define LINEAR_ADC_FRACT16_INIT(adcBits, adcuZero, isInverted)  \
{                                                       \
    .Slope              = isInverted ? -1 : 1,          \
    .SlopeShift         = 15U - adcuBits,               \
    .InvSlope           = isInverted ? -1 : 1,          \
    .InvSlopeShift      = 15U - adcuBits,               \
    .X0                 = adcuZero,                     \
    .XDelta             = (1L << adcBits) - adcuZero,   \
    .XReference         = (1L << adcBits),              \
    .Y0                 = 0,                            \
    .YDelta             = 32768 * (isInverted ? -1 : 1),    \
    .YReference         = 32768 * (isInverted ? -1 : 1),    \
}

/******************************************************************************/
/*!
    Units as Init
*/
/******************************************************************************/
static inline int16_t Linear_ADC_Normalize(const Linear_T * p_linear, uint16_t adcu)  { return linear_shift_f_x0(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, adcu); }
static inline uint16_t Linear_ADC_Of(const Linear_T * p_linear, int32_t normalized) { return linear_shift_invf_x0(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->X0, normalized); }

// static inline int16_t Linear_ADC_Fract16(const Linear_T * p_linear, uint16_t adcu)                  { return Linear_Q16_Fract(p_linear, adcu); }
// static inline uint16_t Linear_ADC_Percent16(const Linear_T * p_linear, uint16_t adcu)               { return Linear_Q16_Percent(p_linear, adcu); }
// static inline uint16_t Linear_ADC_Percent16_Abs(const Linear_T * p_linear, uint16_t adcu)           { return Linear_Q16_Percent_Abs(p_linear, adcu); }
// static inline uint16_t Linear_ADC_AdcuOfFract16(const Linear_T * p_linear, int32_t fract16)         { return Linear_Q16_InvFract(p_linear, fract16); }
// static inline uint16_t Linear_ADC_AdcuOfPercent16(const Linear_T * p_linear, uint32_t percent16)    { return Linear_Q16_InvPercent(p_linear, percent16); }


/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Linear_ADC_Init_ZeroToPeakMilliV(Linear_T * p_linear, uint16_t adcVRef_MilliV, uint16_t adcuMax, uint16_t zero_MilliV, uint16_t max_MilliV);

#endif

/* split general factor shift calc */
// Alternatively
// typedef struct Linear_ADC
// {
//     Linear_T Linear;
//     int32_t UnitsRef;
// }
// Linear_ADC_T;

// static inline int32_t Linear_ADC_CalcPhysical(const Linear_T * p_linear, uint16_t adcu) { return Linear_Q16_Units(p_linear, adcu); }
// static inline int32_t Linear_ADC_CalcPhysical_Fract16(const Linear_T * p_linear, uint16_t fract16) { return Linear_Q16_Units16(p_linear, fract16); }
// static inline int32_t Linear_ADC_CalcFract16_Physical(const Linear_T * p_linear, int32_t units) { return Linear_Q16_InvUnits16(p_linear, units); }
// static inline uint16_t Linear_ADC_CalcAdcu_Physical(const Linear_T * p_linear, int16_t units) { return Linear_Q16_InvUnits(p_linear, units); }

/******************************************************************************/

/******************************************************************************/

// linear_shift_fixed32
// linear_shift_invfixed32
// static inline int32_t linear_units_of_fixed(uint8_t nBits, int32_t y0_units, int32_t deltay_units, int32_t y_fixed32)
// {
//     return ((y_fixed32 * deltay_units) >> nBits) + y0_units;
// }

// static inline int32_t linear_fixed_of_units(uint8_t nBits, int32_t y0_units, int32_t deltay_units, int32_t y_units)
// {
//     return (((y_units - y0_units) << nBits) / deltay_units);
// }

/* y_fixed32 * (yref_units - y0_units) >> 16U + y0_units */
/* fixed32 to y_units */
// static inline int32_t linear_units_of_fixed(int32_t y0_units, int32_t deltay_units, int32_t y_fixed32)
// {
//     return ((y_fixed32 * deltay_units) / 65536) + y0_units;
// }

// /* (y_units - y0_units) << 16U / (yref_units - y0_units) */
// /* y_units to fixed32 */
// static inline int32_t linear_fixed_of_units(int32_t y0_units, int32_t deltay_units, int32_t y_units)
// {
//     return ((y_units - y0_units) * 65536 / deltay_units);
// }

// static inline int32_t Linear_Q16_Units16(const Linear_T * p_linear, int32_t y_fract16)
// {
//     return linear_units_of_fixed(p_linear->Y0, p_linear->YDelta, y_fract16);
// }

// static inline int32_t Linear_Q16_InvUnits16(const Linear_T * p_linear, int32_t y_units)
// {
//     return linear_fixed_of_units(p_linear->Y0, p_linear->YDelta, y_units);
// }

// /* x to fixed to y_units */
// static inline int32_t linear_m16_f(int32_t m16_shifted, uint8_t shift, int32_t x0, int32_t y0_units, int32_t deltay_units, int32_t x)
// {
//     return linear_units_of_fixed(deltay_units, y0_units, linear_shift_f_x0(m16_shifted, shift, x0, x));
// }

// /* y_units to fixed to x */
// static inline int32_t linear_m16_invf(int32_t invm16_shifted, uint8_t shift, int32_t x0, int32_t y0_units, int32_t deltay_units, int32_t y_units)
// {
//     return linear_shift_invf_x0(invm16_shifted, shift, x0, linear_fixed_of_units(deltay_units, y0_units, y_units));
// }

// /* User Units using YRef */
// static inline int32_t Linear_Q16_Units(const Linear_T * p_linear, int32_t x)
// {
//     return linear_m16_f(p_linear->Slope, p_linear->SlopeShift, p_linear->X0, p_linear->Y0, p_linear->YDelta, x);
// }

// /* Division limited to this function only */
// static inline int32_t Linear_Q16_InvUnits(const Linear_T * p_linear, int32_t y)
// {
//     return linear_m16_invf(p_linear->InvSlope, p_linear->InvSlopeShift, p_linear->X0, p_linear->Y0, p_linear->YDelta, y);
// }