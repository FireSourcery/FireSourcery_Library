/*******************************************************************************/
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

#include "Math/Q/QFrac16.h"

typedef enum
{
	VECTOR_000 = 0U,
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

	/* Config */
	qfrac16_t VectorMaxMagnitude;
	qfrac16_t VectorMaxD; /* default sqrt1/3 */

	/* Inputs */
	 volatile qfrac16_t Ia;
	 volatile qfrac16_t Ib;
	 volatile qfrac16_t Ic;

	 volatile qangle16_t Theta;  // electrical angle
	 volatile qfrac16_t Sine; /* save for inverse park call */
	 volatile qfrac16_t Cosine;

	 /* calculated */
	 volatile qfrac16_t Ialpha;
	 volatile qfrac16_t Ibeta;

	/* PID process/feedback variable */
	 volatile qfrac16_t Id;
	 volatile qfrac16_t Iq;

	/* PID control variable */
	/* Intermediate Input to bypass current feedback */
	 volatile qfrac16_t Vd;
	 volatile qfrac16_t Vq;

	 volatile qfrac16_t Valpha;
	 volatile qfrac16_t Vbeta;

	 volatile uint16_t DutyA;
	 volatile uint16_t DutyB;
	 volatile uint16_t DutyC;

	/* Pointer mapped outputs */
//	uint16_t *p_PwmA;
//	uint16_t *p_PwmB;
//	uint16_t *p_PwmC;

} FOC_T;

static inline void FOC_ProcTheta(FOC_T * p_foc)
{
	qfrac16_vector(&p_foc->Cosine, &p_foc->Sine, p_foc->Theta);
}

static inline void FOC_ProcClarkePark(FOC_T *  p_foc)
{
	foc_clarke(&p_foc->Ialpha, &p_foc->Ibeta, p_foc->Ia, p_foc->Ib, p_foc->Ic);
	foc_park_vector(&p_foc->Id, &p_foc->Iq, p_foc->Ialpha, p_foc->Ibeta, p_foc->Sine, p_foc->Cosine);

//	foc_clarke(&p_foc->Ialpha, &p_foc->Ibeta, *(p_foc->p_Ia), *(p_foc->p_Ib), *(p_foc->p_Ic));
//	foc_park_vector(&p_foc->Id, &p_foc->Iq, p_foc->Ialpha, p_foc->Ibeta, *(p_foc->p_Theta));
}

static inline void FOC_ProcInvParkInvClarkeSvpwm(FOC_T *  p_foc)
{
	foc_limitvector_dmax(&p_foc->Vd, &p_foc->Vq, p_foc->VectorMaxMagnitude, p_foc->VectorMaxD);
	foc_invpark_vector(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
	svpwm_midclamp(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, p_foc->Valpha, p_foc->Vbeta);
}

static inline void FOC_ProcDirectThetaSvpwm(FOC_T *  p_foc)
{
//	qangle16_t phi = arctan(qfrac16_div(p_foc->Vq, p_foc->Vd));
//	uint16_t magnitude = mag(p_foc->Vd, p_foc->Vq);
//
//	svpwm_unipolar1(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, magnitude, p_foc->Theta, phi);

	//vq is positive
	svpwm_unipolar1(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, (uint16_t)(p_foc->Vq << 1U), p_foc->Theta, QANGLE16_90);
}


static inline void FOC_SetTheta(FOC_T * p_foc, qangle16_t theta){p_foc->Theta = theta;}
static inline void FOC_SetVector(FOC_T * p_foc, qangle16_t theta){qfrac16_vector(&p_foc->Cosine, &p_foc->Sine, theta);}
static inline void FOC_SetIa(FOC_T * p_foc, qfrac16_t ia){p_foc->Ia = ia;}
static inline void FOC_SetIb(FOC_T * p_foc, qfrac16_t ib){p_foc->Ib = ib;}
static inline void FOC_SetIc(FOC_T * p_foc, qfrac16_t ic){p_foc->Ic = ic;}
static inline void FOC_SetVd(FOC_T * p_foc, qfrac16_t vd){p_foc->Vd = vd;}
static inline void FOC_SetVq(FOC_T * p_foc, qfrac16_t vq){p_foc->Vq = vq;}
static inline uint16_t FOC_GetDutyA(FOC_T * p_foc){return p_foc->DutyA;}
static inline uint16_t FOC_GetDutyB(FOC_T * p_foc){return p_foc->DutyB;}
static inline uint16_t FOC_GetDutyC(FOC_T * p_foc){return p_foc->DutyC;}

extern void FOC_Init(FOC_T * p_foc);
extern void FOC_SetAlign(FOC_T * p_foc, qfrac16_t vd);
extern void FOC_SetZero(FOC_T * p_foc);

//void FOC_SetId(FOC_t * p_foc)
//{
//}
//
//void FOC_SetIq(FOC_t * p_foc)
//{
//}


#endif
