/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
    @file 	Linear_Voltage.h
    @author FireSoucery
    @brief 	Voltage to ADC values conversion using voltage divider.
    		Calculates and holds ADCU voltage conversion ratios

    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_VOLTAGE_H
#define LINEAR_VOLTAGE_H

#include "Linear.h"

#include <stdint.h>

/*
 * R2 overflow > 65536
 */
#define LINEAR_VOLTAGE_CONFIG(r1, r2, adcVRef10, adcBits, vInMax) 							\
{																							\
	.SlopeFactor 				= (((int32_t)adcVRef10 * (r1 + r2)) << (16U - adcBits)) / r2 / 10U, 	\
	.SlopeDivisor_Shift 		= 16U,														\
	.SlopeDivisor 				= ((int32_t)r2 << 16U) / adcVRef10 * 10U / (r1 + r2),				\
	.SlopeFactor_Shift 			= 16U - adcBits,											\
	.YOffset 					= 0U, 														\
	.XOffset 					= 0U, 														\
	.YReference 				= vInMax - 0U, 												\
}

//#define LINEAR_VOLTAGE_CONFIG(r1, r2, adcVRef10, adcBits, vInMax) LINEAR_CONFIG(((int32_t)adcVRef10 * (r1 + r2) / 10 / r2), ((int32_t)1 << adcBits), 0, ((int32_t)vInMax * 1000))

/******************************************************************************/
/*!
	@brief Calculate voltage from given ADC value
	// (adcu*VREF*(R1+R2))/(ADC_MAX*R2);
	 *
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
	int32_t factor = adcu * p_linear->SlopeFactor;
	int32_t milliV;

	if(factor < INT32_MAX / 1000UL)
	{
		milliV = Linear_Function(p_linear, (uint32_t)adcu * 1000UL);
	}
	else if(factor < INT32_MAX / 100UL)
	{
		milliV = Linear_Function(p_linear, (uint32_t)adcu * 100UL) * 10U;
	}
	else if(factor < INT32_MAX / 10UL)
	{
		milliV = Linear_Function(p_linear, (uint32_t)adcu * 10UL) * 100U;
	}
	else
	{
		milliV = Linear_Function(p_linear, (uint32_t)adcu) * 1000U;
	}

	return milliV;
}

/*
 * Check Overflow range
 */
static inline int32_t Linear_Voltage_CalcScalarV(const Linear_T * p_linear, uint16_t adcu, uint16_t scalar)
{
	return Linear_Function(p_linear, (uint32_t)adcu * scalar);
}

/******************************************************************************/
/*!
	@brief 	results expressed in Q16.16  where 65356 => 100% of vInMax
 */
/******************************************************************************/
/*
 * unbounded where > 65536 is > 100%
 */
static inline int32_t Linear_Voltage_CalcFraction16(const Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Function_Fraction16(p_linear, adcu);
}

/******************************************************************************/
/*!
	@brief 	results expressed in Q0.16, Saturated to 65535 max
 */
/******************************************************************************/
static inline uint16_t Linear_Voltage_CalcFractionUnsigned16(const Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Function_FractionUnsigned16(p_linear, adcu);
}

/******************************************************************************/
/*!
	@brief 	results expressed in Q1.15 where 32,767 ~= 100%
 */
/******************************************************************************/
static inline int16_t Linear_Voltage_CalcFractionSigned16(const Linear_T * p_linear, uint16_t adcu)
{
	return Linear_Function_FractionSigned16(p_linear, adcu);
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

static inline uint16_t Linear_Voltage_CalcAdcu_Fraction16(const Linear_T * p_linear, int32_t fract16)
{
	return (uint16_t)Linear_InvFunction_Fraction16(p_linear, fract16);
}

/*
 * Same as general Linear_Voltage_CalcAdcu_Fraction16
 */
static inline uint16_t Linear_Voltage_CalcAdcu_FractionUnsigned16(const Linear_T * p_linear, uint16_t fract16)
{
	return (uint16_t)Linear_InvFunction_FractionUnsigned16(p_linear, fract16);
}

/*
 * fract16 in Q1.15
 */
static inline uint16_t Linear_Voltage_CalcAdcu_FractionSigned16(const Linear_T * p_linear, int16_t fract16)
{
	return (uint16_t)Linear_InvFunction_FractionSigned16(p_linear, fract16);
}

extern void Linear_Voltage_Init(Linear_T * p_linear, uint16_t r1, uint16_t r2, uint8_t adcVRef10, uint8_t adcBits, uint16_t vInMax);

#endif
