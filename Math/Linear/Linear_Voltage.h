/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     Linear_Voltage.h
    @author FireSourcery
    @brief     ADC to Voltage values conversion using voltage divider.
            Flexible V units with precision over Linear_ADC.
    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_VOLTAGE_H
#define LINEAR_VOLTAGE_H

#include "Linear.h"
#include <stdint.h>

#define LINEAR_VOLTAGE_SHIFT (15U)

/*
    Overflow: R2 > 65536
*/
#define LINEAR_VOLTAGE_INIT(r1, r2, adcBits, adcVRef_MilliV, vInRef)                                                     \
{                                                                                                                        \
    .Slope              = (((uint32_t)adcVRef_MilliV * (r1 + r2)) << (LINEAR_VOLTAGE_SHIFT - adcBits)) / r2 / 1000U,     \
    .SlopeShift         = LINEAR_VOLTAGE_SHIFT,                                                                            \
    .InvSlope           = ((int32_t)r2 << LINEAR_VOLTAGE_SHIFT) / adcVRef_MilliV * 1000U / (r1 + r2),                    \
    .InvSlopeShift      = LINEAR_VOLTAGE_SHIFT - adcBits,                                                                \
    .YOffset            = 0U,                                                                                             \
    .XOffset            = 0U,                                                                                             \
    .YReference         = vInRef,                                                                                         \
}

/******************************************************************************/
/*!
    @brief Calculate voltage from given ADC value

    @param[in] p_linear - struct containing calculated intermediate values
    @param[in] adcu - ADC value
    @return Calculated voltage
*/
/******************************************************************************/
static inline int32_t Linear_Voltage_CalcV(const Linear_T * p_linear, uint16_t adcu)
{
    return Linear_Function(p_linear, adcu);
}

static inline int32_t Linear_Voltage_CalcMilliV(const Linear_T * p_linear, uint16_t adcu)
{
    return Linear_Function_Scalar(p_linear, adcu, 1000U);
}

static inline int32_t Linear_Voltage_CalcScalarV(const Linear_T * p_linear, uint16_t adcu, uint16_t scalar)
{
    return Linear_Function_Scalar(p_linear, adcu, scalar);
}

static inline int32_t Linear_Voltage_CalcV_Frac16(const Linear_T * p_linear, uint16_t frac16)
{

}

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/*!
    @brief     results expressed in Q16.16, where 65356 => 100% of vInMax
*/
static inline int32_t Linear_Voltage_CalcFrac16(const Linear_T * p_linear, uint16_t adcu)
{
    return Linear_Function_Frac16(p_linear, adcu);
}

/*!
    @brief     results expressed in Q0.16, Saturated to 65535 max
*/
static inline uint16_t Linear_Voltage_CalcFracU16(const Linear_T * p_linear, uint16_t adcu)
{
    return Linear_Function_FracU16(p_linear, adcu);
}

/*!
    @brief     results expressed in Q1.15 where 32,767 => 100% of vInMax
*/
static inline int16_t Linear_Voltage_CalcFracS16(const Linear_T * p_linear, uint16_t adcu)
{
    return Linear_Function_FracS16(p_linear, adcu);
}

/******************************************************************************/
/*!
    @brief Calculate ADC value from given voltage

    @param[in] linear - struct containing calculated intermediate values
    @param[in] voltage - voltage
    @return Calculated ADC value
*/
/******************************************************************************/
static inline uint16_t Linear_Voltage_CalcAdcu_V(const Linear_T * p_linear, uint16_t volts)
{
    return (uint16_t)Linear_InvFunction(p_linear, volts);
}

static inline uint16_t Linear_Voltage_CalcAdcu_MilliV(const Linear_T * p_linear, uint32_t milliV)
{
    return (uint16_t)(Linear_InvFunction(p_linear, milliV) / 1000U);
}

static inline uint16_t Linear_Voltage_CalcAdcu_ScalarV(const Linear_T * p_linear, uint16_t scalarV, uint16_t scalar)
{
    return (uint16_t)Linear_InvFunction_Scalar(p_linear, scalarV, scalar); //todo
}

static inline uint16_t Linear_Voltage_CalcAdcu_Frac16(const Linear_T * p_linear, int32_t frac16)
{
    return (uint16_t)Linear_InvFunction_Frac16(p_linear, frac16);
}

/* Same as general Linear_Voltage_CalcAdcu_Frac16 */
static inline uint16_t Linear_Voltage_CalcAdcu_FracU16(const Linear_T * p_linear, uint16_t frac16)
{
    return (uint16_t)Linear_InvFunction_FracU16(p_linear, frac16);
}

/* frac16 in Q1.15 */
static inline uint16_t Linear_Voltage_CalcAdcu_FracS16(const Linear_T * p_linear, int16_t frac16)
{
    return (uint16_t)Linear_InvFunction_FracS16(p_linear, frac16);
}

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
extern void Linear_Voltage_Init(Linear_T * p_linear, uint16_t r1, uint16_t r2, uint8_t adcBits, uint16_t adcVRef_MilliV, uint16_t vInMax);
extern uint16_t Linear_Voltage_CalcAdcuInput_V(const Linear_T * p_linear, uint16_t volts);
extern uint16_t Linear_Voltage_CalcAdcuInput_MilliV(const Linear_T * p_linear, uint32_t milliV);

#endif
