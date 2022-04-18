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
    @file 	Linear_Speed.h
    @author FireSoucery
    @brief	Linear Frac16 calc without division
    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_SPEED_H
#define LINEAR_SPEED_H

#include "Linear.h"

static inline void Encoder_DeltaD_CaptureQuadrature(Encoder_T * p_encoder)
{


}
static inline int32_t Linear_Speed_Calc(const Linear_T * p_linear, int32_t angle, int32_t anglePrev, bool isDirectionPositive)
{
//
//	if (isDirectionPositive)
//	{
//
//	}
//		bool isIncrement;
//		bool isCounterIncrementDirectionPositive;
//
//		/*
//		 * Unsigned DeltaD capture
//		 */
//		if (HAL_Encoder_ReadTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER))
//		{
//			if(HAL_Encoder_ReadQuadratureCounterOverflowIncrement(p_encoder->CONFIG.P_HAL_ENCODER))
//			{
//				p_encoder->DeltaD = p_encoder->Params.CountsPerRevolution - p_encoder->TimerCounterSaved + counterValue;
//				isIncrement = true;
//			}
//			else if (HAL_Encoder_ReadQuadratureCounterOverflowDecrement(p_encoder->CONFIG.P_HAL_ENCODER)) //counter counts down, deltaD is negative
//			{
//				p_encoder->DeltaD = p_encoder->Params.CountsPerRevolution - counterValue + p_encoder->TimerCounterSaved;
//				isIncrement = false;
//			}
//
//			HAL_Encoder_ClearTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER);
//		}
//		else
//		{
//			if (counterValue > p_encoder->TimerCounterSaved)
//			{
//				p_encoder->DeltaD = counterValue - p_encoder->TimerCounterSaved;
//				isIncrement = true;
//			}
//			else //counter counts down, deltaD is negative
//			{
//				p_encoder->DeltaD = p_encoder->TimerCounterSaved - counterValue;
//				isIncrement = false;
//			}
//
//			//signed capture
//	//		p_encoder->DeltaD = counterValue - p_encoder->TimerCounterSaved;
//		}
//
//		p_encoder->TimerCounterSaved = counterValue;
//		p_encoder->AngularD = counterValue;
//		p_encoder->TotalT += 1U;
//
//
//
//		//static inline void Encoder_DeltaD_ReadQuadratureDirection(Encoder_T * p_encoder)
//		//{//#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
//		//	//	isCounterIncrementDirectionPositive = p_encoder->IsALeadBDirectionPositive;
//		//	//#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
//		//	//	isCounterIncrementDirectionPositive = !p_encoder->IsALeadBDirectionPositive;
//		//	//#endif
//		//	return HAL_Encoder_ReadQuadratureCounterDirection(p_encoder->CONFIG.P_HAL_ENCODER);
//		//}
//
//	#ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
//		isCounterIncrementDirectionPositive = p_encoder->Params.IsALeadBPositive;
//	#elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
//		isCounterIncrementDirectionPositive = !p_encoder->Params.IsALeadBPositive;
//	#endif
//
//		if ((isCounterIncrementDirectionPositive) && (isIncrement == true))
//	//	if ((p_encoder->Params.DirectionCalibration == ENCODER_DIRECTION_DIRECT) && (isIncrement == true)) //Positive DeltaD is positive direction
//		{
//			p_encoder->TotalD += p_encoder->DeltaD;
//		}
//		else
//		{
//			p_encoder->TotalD -= p_encoder->DeltaD;	//  deltaD is negative
//		}
//
//	return linear_f16_m16_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->XOffset, p_linear->YOffset, x);
}


//static inline int32_t Linear_Speed(const Linear_T * p_linear, int32_t x)
//{
//	return linear_f16_m16_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->XOffset, p_linear->YOffset, x);
//}
//
//static inline int32_t Linear_InvFrac16(const Linear_T * p_linear, int32_t y_frac16)
//{
//	return linear_invf16_invm16_shift(p_linear->SlopeDivisor, p_linear->SlopeFactor_Shift, p_linear->XOffset, p_linear->YOffset, y_frac16);
//}
//
//static inline int32_t Linear_Speed_CalcUnits(const Linear_T * p_linear, int32_t x)
//{
//	return linear_f_m16_shift(p_linear->SlopeFactor, p_linear->SlopeDivisor_Shift, p_linear->XOffset, p_linear->YOffset, p_linear->YReference, x);
//}
//
//static inline int32_t Linear_InvFrac16_CalcUnits(const Linear_T * p_linear, int32_t y)
//{
//	return linear_invf_invm16_shift(p_linear->SlopeDivisor, p_linear->SlopeFactor_Shift, p_linear->XOffset, p_linear->YOffset, p_linear->YReference, y);
//}
//
////todo common bound function
//static inline uint16_t Linear_Speed_Unsigned(const Linear_T * p_linear, int32_t x)
//{
//	int32_t frac16 = Linear_Speed(p_linear, x);
//
//	if 		(frac16 > 65535)	{frac16 = 65535;}
//	else if (frac16 < 0) 		{frac16 = 0;}
//
//	return (uint16_t)frac16;
//}
//
//static inline uint16_t Linear_Speed_Unsigned_Abs(const Linear_T * p_linear, int32_t x)
//{
//	int32_t frac16 = Linear_Speed(p_linear, x);
//
//	if 		(frac16 < 0)		{frac16 = 0 - frac16;}
//	else if (frac16 > 65535)	{frac16 = 65535;}
//
//	return (uint16_t)frac16;
//}
//
//static inline int32_t Linear_InvFrac16_Unsigned(const Linear_T * p_linear, uint16_t y_frac16)
//{
//	return Linear_InvFrac16(p_linear, y_frac16);
//}
//
//static inline int16_t Linear_Speed_Signed(const Linear_T * p_linear, int32_t x)
//{
//	int32_t frac16 = Linear_Speed(p_linear, x) / 2;
//
//	if 		(frac16 > 32767)	{frac16 = 32767;}
//	else if (frac16 < -32768) 	{frac16 = -32768;}
//
//	return (int16_t)frac16;
//}
//
//static inline int32_t Linear_InvFrac16_Signed(const Linear_T * p_linear, int16_t y_fracSigned16)
//{
//	return Linear_InvFrac16(p_linear, y_fracSigned16 * 2);
//}

#endif
