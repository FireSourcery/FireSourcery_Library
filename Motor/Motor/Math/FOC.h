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
    @file 	FOC.h
    @author FireSoucery
    @brief  FOC math functions only. Wrapper for pure math functions.
    		 Only this modules calls math_foc, math_svpwm
    @version V0
*/
/******************************************************************************/
#ifndef FOC_H
#define FOC_H

#include "math_foc.h"
#include "math_svpwm.h"

#include "Math/Q/QFrac16.h"

//typedef enum
//{
//	VECTOR_000 = 0U,
//	VECTOR_001_A = 0b001,
//	VECTOR_011_INV_C = 0b011,
//} FOC_VectorId_T;

typedef struct
{
	/* Params */
	qfrac16_t VectorMaxMagnitude;
	qfrac16_t VectorMaxD; /* default 1/sqrt3 */

	/* Inputs */
	qfrac16_t Ia;
	qfrac16_t Ib;
	qfrac16_t Ic;

//	qangle16_t Theta;
	qfrac16_t Sine; /* save for inverse park call */
	qfrac16_t Cosine;

	/* calculated */
	qfrac16_t Ialpha;
	qfrac16_t Ibeta;

	/* PID process/feedback variable */
	qfrac16_t Id;
	qfrac16_t Iq;

	/* PID control variable */
	/* Intermediate Input, bypass current feedback */
	qfrac16_t Vd;
	qfrac16_t Vq;

	qfrac16_t Valpha;
	qfrac16_t Vbeta;

	/* dutyA, dutyB, dutyC -> 16 bits, q0.16, always positive */
	uint16_t DutyA;
	uint16_t DutyB;
	uint16_t DutyC;
}
FOC_T;

static inline void FOC_ProcClarkePark(FOC_T * p_foc)
{
	foc_clarke(&p_foc->Ialpha, &p_foc->Ibeta, p_foc->Ia, p_foc->Ib, p_foc->Ic);
	foc_park_vector(&p_foc->Id, &p_foc->Iq, p_foc->Ialpha, p_foc->Ibeta, p_foc->Sine, p_foc->Cosine);

//	if (p_foc->Iq > -200 && p_foc->Iq < 200 ) {p_foc->Iq = 0;}
//	if (p_foc->Id > -200 && p_foc->Id < 200 ) {p_foc->Id = 0;}
}

static inline void FOC_ProcInvParkInvClarkeSvpwm(FOC_T * p_foc)
{
//	foc_circlelimit_dmax(&p_foc->Vd, &p_foc->Vq, p_foc->VectorMaxMagnitude, p_foc->VectorMaxD);
	foc_circlelimit(&p_foc->Vd, &p_foc->Vq, QFRAC16_MAX);
	foc_invpark_vector(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
	svpwm_midclamp(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, p_foc->Valpha, p_foc->Vbeta);

	p_foc->DutyA = p_foc->DutyA * 2U;
	p_foc->DutyB = p_foc->DutyB * 2U;
	p_foc->DutyC = p_foc->DutyC * 2U;
}

static inline void FOC_SetVector(FOC_T * p_foc, qangle16_t theta) {qfrac16_vector(&p_foc->Cosine, &p_foc->Sine, theta);}
static inline void FOC_SetIa(FOC_T * p_foc, qfrac16_t ia) {p_foc->Ia = ia;}
static inline void FOC_SetIb(FOC_T * p_foc, qfrac16_t ib) {p_foc->Ib = ib;}
static inline void FOC_SetIc(FOC_T * p_foc, qfrac16_t ic) {p_foc->Ic = ic;}
static inline void FOC_SetVReq(FOC_T * p_foc, qfrac16_t vq, qfrac16_t vd) {p_foc->Vq = vq; p_foc->Vd = vd;}
static inline void FOC_SetVd(FOC_T * p_foc, qfrac16_t vd) {p_foc->Vd = vd;}
static inline void FOC_SetVq(FOC_T * p_foc, qfrac16_t vq) {p_foc->Vq = vq;}

static inline uint16_t FOC_GetDutyA(FOC_T * p_foc) {return p_foc->DutyA;}
static inline uint16_t FOC_GetDutyB(FOC_T * p_foc) {return p_foc->DutyB;}
static inline uint16_t FOC_GetDutyC(FOC_T * p_foc) {return p_foc->DutyC;}
static inline qfrac16_t FOC_GetIa(FOC_T * p_foc) {return p_foc->Ia;}
static inline qfrac16_t FOC_GetIb(FOC_T * p_foc) {return p_foc->Ib;}
static inline qfrac16_t FOC_GetIc(FOC_T * p_foc) {return p_foc->Ic;}
static inline qfrac16_t FOC_GetId(FOC_T * p_foc) {return p_foc->Id;}
static inline qfrac16_t FOC_GetIq(FOC_T * p_foc) {return p_foc->Iq;}
static inline qfrac16_t FOC_GetVd(FOC_T * p_foc) {return p_foc->Vd;}
static inline qfrac16_t FOC_GetVq(FOC_T * p_foc) {return p_foc->Vq;}

extern void FOC_Init(FOC_T * p_foc);
extern void FOC_SetAlign(FOC_T * p_foc, qfrac16_t vd);
extern void FOC_Reset(FOC_T * p_foc);

#endif
//static inline void FOC_ProcThetaSvpwm(FOC_T *  p_foc)
//{
////	qangle16_t phi = arctan(qfrac16_div(p_foc->Vq, p_foc->Vd));
////	uint16_t magnitude = mag(p_foc->Vd, p_foc->Vq);
////
////	svpwm_unipolar1(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, magnitude, p_foc->Theta, phi);
//
//	//vq is positive
////	svpwm_unipolar1(&p_foc->DutyA, &p_foc->DutyB, &p_foc->DutyC, (uint16_t)(p_foc->Vq << 1U), p_foc->Theta, QANGLE16_90);
//}
