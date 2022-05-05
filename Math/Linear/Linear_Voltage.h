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
	@brief 	ADC to Voltage values conversion using voltage divider.
			Calculates and holds ADCU voltage conversion ratios. 
			Flexible voltage units.
			Frac16 init fixes precision to 1 voltage unit.
	@version V0
*/
/******************************************************************************/
#ifndef LINEAR_VOLTAGE_H
#define LINEAR_VOLTAGE_H

#include "Linear.h" 
#include <stdint.h>

/*
	Overflow: R2 > 65536
 */
// #define LINEAR_VOLTAGE_CONFIG(r1, r2, adcBits, adcVRef_MilliV, vInMax) 									\
// {																										\
// 	.Slope 				= (((int32_t) adcVRef_MilliV * (r1 + r2)) << (16U - adcBits)) / r2 / 1000U, 	\
// 	.SlopeShift 		= 16U,																			\
// 	.InvSlope  			= ((int32_t)r2 << 16U) / adcVRef_MilliV * 1000U / (r1 + r2),					\
// 	.InvSlopeShift 		= 16U - adcBits,																\
// 	.YOffset 			= 0U, 																			\
// 	.XOffset 			= 0U, 																			\
// 	.YReference 		= vInMax, 																		\
// }

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

/******************************************************************************/
/*!
	@brief 	results expressed in Q16.16, where 65356 => 100% of vInMax
*/
/******************************************************************************/ 
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
	@brief 	results expressed in Q1.15 where 32,767 => 100% of vInMax
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

static inline uint16_t Linear_Voltage_CalcAdcu_ScalarV(const Linear_T * p_linear, uint16_t scalarV, uint16_t scalar)
{
	// return (uint16_t)Linear_InvFunction_Scalar(p_linear, scalarV, scalar);
}


static inline uint16_t Linear_Voltage_CalcAdcu_Fraction16(const Linear_T * p_linear, int32_t fract16)
{
	return (uint16_t)Linear_InvFunction_Fraction16(p_linear, fract16);
}

/* Same as general Linear_Voltage_CalcAdcu_Fraction16 */
static inline uint16_t Linear_Voltage_CalcAdcu_FractionUnsigned16(const Linear_T * p_linear, uint16_t fract16)
{
	return (uint16_t)Linear_InvFunction_FractionUnsigned16(p_linear, fract16);
}

/* fract16 in Q1.15 */
static inline uint16_t Linear_Voltage_CalcAdcu_FractionSigned16(const Linear_T * p_linear, int16_t fract16)
{
	return (uint16_t)Linear_InvFunction_FractionSigned16(p_linear, fract16);
}

extern void Linear_Voltage_Init(Linear_T * p_linear, uint16_t r1, uint16_t r2, uint8_t adcBits, uint16_t adcVRef_MilliV, uint16_t vInMax); 
extern uint16_t Linear_Voltage_CalcAdcu_UserV(const Linear_T * p_linear, uint16_t volts);
extern uint16_t Linear_Voltage_CalcAdcu_UserMilliV(const Linear_T * p_linear, uint32_t milliV);

#endif
