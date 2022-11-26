#ifndef ENCODER_ISR_H
#define ENCODER_ISR_H

#include "Encoder.h"
#include "Encoder_DeltaD.h"
#include "Encoder_DeltaT.h"

/******************************************************************************/
/*!
	ISRs
*/
/*! @{ */
/******************************************************************************/
/*
	Index Pin
*/
static inline void Encoder_OnIndex_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
	HAL_Encoder_ClearFlagIndex(p_encoder->CONFIG.P_HAL_ENCODER, 0);
	HAL_Encoder_ClearPinFlagPhaseA(p_encoder->CONFIG.P_HAL_ENCODER, 0);
	HAL_Encoder_ClearPinFlagPhaseB(p_encoder->CONFIG.P_HAL_ENCODER, 0);
	p_encoder->AngularD = 0U;
}

#if 	defined(CONFIG_ENCODER_HW_QUADRATURE_CAPABLE)
#elif 	defined(CONFIG_ENCODER_HW_CAPTURE_COUNT)
#elif 	defined(CONFIG_ENCODER_HW_CAPTURE_TIME)
#endif

/* Shared Index, A, B Thread */
static inline void Encoder_CaptureDeltaT_ISR(Encoder_T * p_encoder)
{
	if(HAL_Encoder_ReadPinFlagPhaseA(p_encoder->CONFIG.P_HAL_ENCODER, 0) == true)
	{
		HAL_Encoder_ClearPinFlagPhaseA(p_encoder->CONFIG.P_HAL_ENCODER, 0);
	}
	else if(HAL_Encoder_ReadPinFlagPhaseB(p_encoder->CONFIG.P_HAL_ENCODER, 0) == true)
	{
		HAL_Encoder_ClearPinFlagPhaseB(p_encoder->CONFIG.P_HAL_ENCODER, 0);
	}
	else if(HAL_Encoder_ReadPinFlagIndex(p_encoder->CONFIG.P_HAL_ENCODER, 0) == true)
	{
		Encoder_OnIndex_ISR(p_encoder);
	}

	Encoder_DeltaT_Capture(p_encoder);
}
/******************************************************************************/
/*! @} */
/******************************************************************************/
#endif