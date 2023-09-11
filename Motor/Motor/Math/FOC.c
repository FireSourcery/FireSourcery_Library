/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   FOC.c
    @author FireSourcery
    @version V0
*/
/******************************************************************************/
#include "FOC.h"

void FOC_Init(FOC_T * p_foc)
{
    // p_foc->IdqMagnitudeMax = QFRAC16_MAX;
    // p_foc->IdMax = QFRAC16_1_DIV_SQRT3;
    // FOC_ClearState(p_foc);
}

// void FOC_SetVectorMax(FOC_T * p_foc, qfrac16_t dMax)
// {
//     p_foc->VectorMaxMagnitude = QFRAC16_MAX;
//     p_foc->VectorMaxD = dMax;
// }

/* Prep Align using input intensity */
void FOC_SetAlign(FOC_T * p_foc, qfrac16_t vd)
{
    p_foc->Vd = vd;
    p_foc->Vq = 0;
    p_foc->Sine = 0;
    p_foc->Cosine = QFRAC16_MAX;
    FOC_ProcInvParkInvClarkeSvpwm(p_foc);
}

void FOC_ZeroSvpwm(FOC_T * p_foc)
{
    p_foc->DutyA = 65536U / 2U;
    p_foc->DutyB = 65536U / 2U;
    p_foc->DutyC = 65536U / 2U;
}

void FOC_ClearControlState(FOC_T * p_foc)
{
    p_foc->Ia = 0;  /* ADC */
    p_foc->Ib = 0;
    p_foc->Ic = 0;
    p_foc->Id = 0;  /* Feedback */
    p_foc->Iq = 0;
    p_foc->DReq = 0; /* Req */
    p_foc->QReq = 0;
    FOC_ZeroSvpwm(p_foc);
}

void FOC_ClearObserveState(FOC_T * p_foc)
{
    p_foc->Va = 0;
    p_foc->Vb = 0;
    p_foc->Vc = 0;
}

// void FOC_ClearState(FOC_T * p_foc)
// {
//     p_foc->Ia = 0;  /* ADC */
//     p_foc->Ib = 0;
//     p_foc->Ic = 0;
//     p_foc->Id = 0;  /* Feedback */
//     p_foc->Iq = 0;
//     p_foc->DReq = 0; /* Req */
//     p_foc->QReq = 0;

//     p_foc->Sine = 0;
//     p_foc->Cosine = 0;
//     p_foc->Va = 0;
//     p_foc->Vb = 0;
//     p_foc->Vc = 0;
//     p_foc->Valpha = 0;
//     p_foc->Vbeta = 0;
//     p_foc->Vd = 0; /* Output/Bemf */
//     p_foc->Vq = 0;

//     FOC_ZeroSvpwm(p_foc);
// }

