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
	@file  	Private.h
	@author FireSourcery
	@brief 	Encoder module private functions
	@version V0
 */
/******************************************************************************/
#ifndef PRIVATE_ENCODER_H
#define PRIVATE_ENCODER_H

#include "Encoder.h"
#include "HAL_Encoder.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
	@addtogroup	CaptureDelta
	@brief	Capture DeltaT or DeltaD between 2 samples.
			Either DeltaT or DeltaD is selected to be fixed.
			Used for all speed calculations.

			Filtering Delta is responsibility of caller.
 */
/*! @{ */
/******************************************************************************/

/*!
	@brief Private CaptureDelta Helper

	capture increasing
 */
//static inline void CaptureEncoderDeltaIncreasing(Encoder_T * p_encoder, volatile uint32_t * capturedest, *timercounter, max, prev)
//{
//	uint32_t timerCounterValue = HAL_Encoder_ReadTimerCounter(p_encoder->p_HAL_Encoder);
//
//	if (timerCounterValue < p_encoder->TimerCounterSaved) /* TimerCounter overflow */
//	{
//		*p_delta = p_encoder->TimerCounterMax - p_encoder->TimerCounterSaved + timerCounterValue + 1U;
//	}
//	else /* normal case */
//	{
//		*p_delta = timerCounterValue - p_encoder->TimerCounterSaved;
//	}
//
//	p_encoder->TimerCounterSaved = timerCounterValue;
//}


/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
