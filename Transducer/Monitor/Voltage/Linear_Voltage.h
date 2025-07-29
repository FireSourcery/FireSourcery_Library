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
    @file   Linear_Voltage.h
    @author FireSourcery
    @brief     ADC to Voltage values conversion using voltage divider.
            Flexible V units with precision over Linear_ADC.

*/
/******************************************************************************/
#ifndef LINEAR_VOLTAGE_H
#define LINEAR_VOLTAGE_H

#include "Math/Linear/Linear.h"
#include <stdint.h>

#define LINEAR_VOLTAGE_SHIFT (15U)

/*

*/
#define LINEAR_VOLTAGE_OF(r1, r2, adcVRef, adcBits, adcu) ((double)(adcVRef) * (r1 + r2) * (adcu) / (r2) / (double)(1UL << adcBits))
#define _LINEAR_V_PER_ADCU(r1, r2, adcVRef, adcBits) (int32_t)(LINEAR_VOLTAGE_OF(r1, r2, adcVRef, adcBits, 1) * (double)(1UL << (LINEAR_VOLTAGE_SHIFT)))

#define LINEAR_ADCU_OF_V(r1, r2, adcVRef_V, adcBits, volts) ((double)(volts) * (r2) / (r1 + r2) / (adcVRef_V) * (double)(1UL << adcBits))
#define _LINEAR_ADCU_PER_V(r1, r2, adcVRef_V, adcBits) (int32_t)(LINEAR_ADCU_OF_V(r1, r2, adcVRef_V, adcBits, 1) * (double)(1UL << (LINEAR_VOLTAGE_SHIFT - adcBits)))

#define LINEAR_VOLTAGE_INIT(r1, r2, adcVRef, adcBits)                                                                   \
{                                                                                                                       \
    .Slope              = _LINEAR_V_PER_ADCU(r1, r2, adcVRef, adcBits, 1),                                              \
    .SlopeShift         = LINEAR_VOLTAGE_SHIFT,                                                                         \
    .InvSlope           = _LINEAR_ADCU_PER_V(r1, r2, adcVRef, adcBits, 1),                                              \
    .InvSlopeShift      = LINEAR_VOLTAGE_SHIFT - adcBits,                                                               \
    .Y0                 = 0U,                                                                                           \
    .X0                 = 0U,                                                                                           \
    .XDelta          = 1UL << adcBits,                                                                               \
    .YDelta          = (int32_t)(_LINEAR_V_PER_ADCU(r1, r2, adcVRef, adcBits) * (double)(1UL << adcBits)),           \
}


/******************************************************************************/
/*!
    @brief Calculate voltage from given ADC value

    @param[in] p_linear - struct containing calculated intermediate values
    @param[in] adcu - ADC value
    @return Calculated voltage
*/
/******************************************************************************/
static inline int32_t Linear_Voltage_Of(const Linear_T * p_linear, uint16_t adcu) { return Linear_Of(p_linear, adcu); }

/* Overflow scalar > 4096 */
static inline int32_t Linear_Voltage_MilliV(const Linear_T * p_linear, uint16_t adcu) { return Linear_Of(p_linear, adcu * 1000U); }

// static inline int32_t Linear_Voltage_ScalarV(const Linear_T * p_linear, uint16_t adcu, uint16_t scalar) { return Linear_Of_Scalar(p_linear, adcu, scalar); }

/******************************************************************************/
/*!
    @brief Calculate ADC value from given voltage

    @param[in] linear - struct containing calculated intermediate values
    @param[in] voltage - voltage
    @return Calculated ADC value
*/
/******************************************************************************/
static inline uint16_t Linear_Voltage_AdcuOfV(const Linear_T * p_linear, uint16_t volts) { return (uint16_t)Linear_InvOf(p_linear, volts); }

static inline uint16_t Linear_Voltage_AdcuOfMilliV(const Linear_T * p_linear, uint32_t milliV) { return (uint16_t)(Linear_InvOf(p_linear, milliV) / 1000U); }

// static inline uint16_t Linear_Voltage_AdcuOfScalarV(const Linear_T * p_linear, uint16_t scalarV, uint16_t scalar) { return (uint16_t)Linear_InvOf_Scalar(p_linear, scalarV, scalar); }

/* round up .5 volts */
static inline uint16_t Linear_Voltage_AdcuInputOfV_Input(const Linear_T * p_linear, uint16_t volts) { return Linear_InvOf_Round(p_linear, volts); }

static inline uint16_t Linear_Voltage_AdcuOfMilliV_Input(const Linear_T * p_linear, uint32_t milliV) { return Linear_InvOf_Round(p_linear, milliV) / 1000U; }

// /******************************************************************************/
// /*!
//     @brief Fraction as max ADCU value
//     using division for conversion, alternatively determine by init only
//     requires [vInRef] to be set
// */
// /******************************************************************************/
// /*!
//     @brief  results in Q0.16, where 65356 => [vInRef]
// */
// static inline uint16_t Linear_Voltage_Percent16OfAdcu(const Linear_T * p_linear, uint16_t adcu) { return _Linear_Percent16_Clamp(p_linear, adcu); }

// /*!
//     @brief  results in Q1.15 where 32,767 => [vInRef]
// */
// static inline int16_t Linear_Voltage_Fract16OfAdcu(const Linear_T * p_linear, uint16_t adcu) { return _Linear_Fract16(p_linear, adcu); }

// // static inline uint32_t Linear_Voltage_Charge_Fract16OfAdcu(const Linear_T * p_linear, uint16_t adcuZero, uint16_t adcu)
// // {
// //      return ((uint32_t)(adcu - adcuZero) * INT16_MAX) / (p_linear->XReference - adcuZero);
// // }

// static inline uint16_t Linear_Voltage_AdcuOfPercent16(const Linear_T * p_linear, uint16_t percent16) { return (uint16_t)_Linear_InvPercent16(p_linear, percent16); }
// static inline uint16_t Linear_Voltage_AdcuOfFract16(const Linear_T * p_linear, int16_t fract16) { return (uint16_t)_Linear_InvFract16(p_linear, fract16); }
// static inline int32_t Linear_Voltage_OfFract16(const Linear_T * p_linear, uint16_t fract16) { return Linear_Of(p_linear, Linear_Voltage_AdcuOfFract16(p_linear, fract16)); }


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
extern void Linear_Voltage_Init(Linear_T * p_linear, uint32_t r1, uint32_t r2, uint16_t adcVRef_MilliV, uint8_t adcBits);


#endif
