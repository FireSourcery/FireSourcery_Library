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
// static inline void Encoder_TimerCounterOverflow_ISR(Encoder_T * p_encoder)
// {
// 	HAL_Encoder_ClearTimerCounterOverflow(p_encoder->CONFIG.P_HAL_ENCODER);
// 	//set overflow direction?
// }

/*
	Index Pin
*/
static inline void Encoder_OnIndex_ISR(Encoder_T * p_encoder)
{
	HAL_Encoder_WriteTimerCounter(p_encoder->CONFIG.P_HAL_ENCODER, 0U);
	p_encoder->AngularD = 0U;
}

/* Shared Index, A, B Thread */
static inline void Encoder_ISR(Encoder_T * p_encoder)
{

	/* Capture DeltaD Mode */
	// if source == a or b
#if 	defined(CONFIG_ENCODER_HW_QUADRATURE_CAPABLE)
#elif 	defined(CONFIG_ENCODER_HW_CAPTURE_COUNT)
#elif 	defined(CONFIG_ENCODER_HW_CAPTURE_TIME)
	Encoder_DeltaT_Capture(p_encoder);
#endif

// if (getsource == index)
// {
// 	Encoder_OnIndex_ISR(p_encoder)
// }

}
/******************************************************************************/
/*! @} */
/******************************************************************************/
#endif