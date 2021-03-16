/*******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file 	FOC.h
    @author FireSoucery
    @brief  FOC math functions only. Wrapper for pure math functions.
    		 Only this modules call math_foc, math_svpwm
    @version V0
*/
/*******************************************************************************/
#ifndef FOC_H
#define FOC_H

#include "math_foc.h"
#include "math_svpwm.h"


typedef enum
{
	VECTOR_000 = 0,
	VECTOR_001_A = 0b001,
	VECTOR_011_INV_C = 0b011,
} FOC_VectorID_T;

typedef struct
{
//	/* Pointer mapped inputs, FOC read-only */
//	volatile const qfrac16_t * p_Ia;
//	volatile const qfrac16_t * p_Ib;
//	volatile const qfrac16_t * p_Ic;
//	volatile const qangle16_t * p_Theta;

	/* persistent after init */
	uint16_t PwmPeriod;

	qfrac16_t VectorMaxMagnitude;
	qfrac16_t VectorMaxD; /* default sqrt3 */

	/* Inputs */
	qfrac16_t Ia;
	qfrac16_t Ib;
	qfrac16_t Ic;
	//qangle16_t Theta;  // electrical angle
	qfrac16_t Sine; /* save for inverse park call */
	qfrac16_t Cosine;

	qfrac16_t Ialpha;
	qfrac16_t Ibeta;

	/* PID process/feedback variable, PID mapped */
	qfrac16_t Id;
	qfrac16_t Iq;

	/* PID control variable */
	/* Intermediate Input, outside may set directly */
	qfrac16_t Vd;
	qfrac16_t Vq;

	qfrac16_t Valpha;
	qfrac16_t Vbeta;

	/* Pointer mapped outputs */
	uint16_t * p_PwmA;
	uint16_t * p_PwmB;
	uint16_t * p_PwmC;

} FOC_T;

static inline void FOC_ProcClarkePark(FOC_T *  p_foc)
{
//	foc_clarke(&p_foc->Ialpha, &p_foc->Ibeta, *(p_foc->p_Ia), *(p_foc->p_Ib), *(p_foc->p_Ic));
//	foc_park_vector(&p_foc->Id, &p_foc->Iq, p_foc->Ialpha, p_foc->Ibeta, *(p_foc->p_Theta));
	foc_clarke(&p_foc->Ialpha, &p_foc->Ibeta, p_foc->Ia, p_foc->Ib, p_foc->Ic);
	foc_park_vector(&p_foc->Id, &p_foc->Iq, p_foc->Ialpha, p_foc->Ibeta, p_foc->Sine, p_foc->Cosine);
}

static inline void FOC_ProcInvParkInvClarkeSvpwm(FOC_T *  p_foc)
{
	foc_limitvector_dmax(&p_foc->Vd, &p_foc->Vq, p_foc->VectorMaxMagnitude, p_foc->VectorMaxD);

	foc_invpark_vector(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
	svpwm_midclamp(p_foc->p_PwmA, p_foc->p_PwmB, p_foc->p_PwmC, p_foc->PwmPeriod, p_foc->Valpha, p_foc->Vbeta);
}

static inline void FOC_SetVector(FOC_T * p_foc, qangle16_t theta)
{
	qfrac16_vector(&p_foc->Cosine, &p_foc->Sine, theta);
}

static inline void FOC_SetIa(FOC_T * p_foc, qfrac16_t ia)
{
	p_foc->Ia = ia;
}

static inline void FOC_SetIb(FOC_T * p_foc, qfrac16_t ib)
{
	p_foc->Ib = ib;
}

static inline void FOC_SetIc(FOC_T * p_foc, qfrac16_t ic)
{
	p_foc->Ic = ic;
}

static inline void FOC_SetVd(FOC_T * p_foc, qfrac16_t vd)
{
	p_foc->Vd = vd;
}

static inline void FOC_SetVq(FOC_T * p_foc, qfrac16_t vq)
{
	p_foc->Vq = vq;
}

void FOC_SetAlign(FOC_T * p_foc, qfrac16_t vd);
void FOC_SetZero(FOC_T * p_foc);

//void FOC_GetId(FOC_t * p_foc)
//{
//}
//
//void FOC_GetIq(FOC_t * p_foc)
//{
//}

//static inline void FOC_SetTheta(FOC_T *  p_foc, qangle16_t theta)
//{
//	p_foc->Theta = theta;
//}

#endif
