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
    @file 	FOC.c
    @author FireSoucery
    @brief  FOC
    @version V0
*/
/*******************************************************************************/
#include "FOC.h"

#include "Math/Q/QFrac16.h"

void FOC_Init(FOC_T * p_foc, uint16_t pwmPeroid, volatile uint16_t * p_pwmA, volatile uint16_t * p_pwmB, volatile uint16_t * p_pwmC)
{
	p_foc->p_PwmA = p_pwmA;
	p_foc->p_PwmB = p_pwmB;
	p_foc->p_PwmC = p_pwmC;

	p_foc->PwmPeriod = pwmPeroid;

	p_foc->VectorMaxMagnitude = QFRAC16_MAX;
	p_foc->VectorMaxD = QFRAC16_1_DIV_SQRT3;
}

void FOC_MapPwm(FOC_T * p_foc, volatile uint16_t * p_pwmA, volatile uint16_t * p_pwmB, volatile uint16_t * p_pwmC)
{
	p_foc->p_PwmA = p_pwmA;
	p_foc->p_PwmB = p_pwmB;
	p_foc->p_PwmC = p_pwmC;
}

void FOC_SetAlign(FOC_T * p_foc, qfrac16_t vd)
{
	p_foc->Vd = vd;
	p_foc->Vq = 0;
	p_foc->Sine = 0;
	p_foc->Cosine = QFRAC16_MAX;
	FOC_ProcInvParkInvClarkeSvpwm(p_foc);
}

void FOC_SetZero(FOC_T * p_foc)
{
	p_foc->Ia = 0;
	p_foc->Ib = 0;
	p_foc->Ic = 0;
//	p_foc->Ialpha;
//	p_foc->Ibeta;
//
//	p_foc->Vd;
//	p_foc->Vq;
//	p_foc->Valpha;
//	p_foc->Vbeta;


	p_foc-> Sine = 0; /* save for inverse park call */
	p_foc-> Cosine = 0;

	*p_foc->p_PwmA = 0; // = 1/2;
	*p_foc->p_PwmB = 0;
	*p_foc->p_PwmC = 0;  //

//	p_foc->IdReq = 0;
//	p_foc->IqReq = 0;
//
//	p_foc->dReq = 0;
//	p_foc->qReq = 0;

}


