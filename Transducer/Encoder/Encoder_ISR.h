#ifndef ENCODER_ISR_H
#define ENCODER_ISR_H

#include "Encoder.h"
#include "Encoder_DeltaD.h"
#include "Encoder_DeltaT.h"

/******************************************************************************/
/*!
	ISRs
	For emulated capture D need ISR, or callback hook
*/
/*! @{ */
/******************************************************************************/

#if defined(CONFIG_ENCODER_HW_EMULATED)
/*
	Index Pin
*/
static inline void Encoder_OnIndex_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER_Z, 0U);
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID);
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID);
	p_encoder->AngularD = 0U;
}

static inline void Encoder_OnPhaseA_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID);
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER_A, 0U);
	_Encoder_CaptureAngularDIncreasing(p_encoder);
	//todo check freq for capture speed
// #if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
// 	_Encoder_CaptureAngularD_Quadrature(p_encoder);
// #endif
}

static inline void Encoder_OnPhaseB_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_ClearPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID);
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER_B, 0U);
	_Encoder_CaptureAngularDIncreasing(p_encoder);
// #if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
// 	_Encoder_CaptureAngularD_Quadrature(p_encoder);
// #endif
}

/* Shared A, B Thread */
static inline void Encoder_OnPhaseAB_ISR(Encoder_T * p_encoder)
{
	if 		(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID) == true) { Encoder_OnPhaseA_ISR(p_encoder); }
	else if	(HAL_Encoder_ReadPhaseFlag(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID) == true) { Encoder_OnPhaseB_ISR(p_encoder); }
}
#endif

/******************************************************************************/
/*! @} */
/******************************************************************************/
#endif