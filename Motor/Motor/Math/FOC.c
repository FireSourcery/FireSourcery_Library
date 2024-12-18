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
    // p_foc->IdqMagnitudeMax = FRACT16_MAX;
    // p_foc->IdMax = FRACT16_1_DIV_SQRT3;
    // FOC_ClearState(p_foc);
}

/* Prep Align using input intensity */
void FOC_SetAlign(FOC_T * p_foc, fract16_t vd)
{
    p_foc->Vd = vd;
    p_foc->Vq = 0;
    p_foc->Sine = 0;
    p_foc->Cosine = FRACT16_MAX;
    FOC_ProcInvParkInvClarkeSvpwm(p_foc);
}

void FOC_ZeroSvpwm(FOC_T * p_foc)
{
    // p_foc->DutyA = FRACT16_MAX / 2;
    // p_foc->DutyB = FRACT16_MAX / 2;
    // p_foc->DutyC = FRACT16_MAX / 2;
    p_foc->Va = FRACT16_MAX / 2;
    p_foc->Vb = FRACT16_MAX / 2;
    p_foc->Vc = FRACT16_MAX / 2;
}

void FOC_ClearControlState(FOC_T * p_foc)
{
    p_foc->Ia = 0;  /* ADC */
    p_foc->Ib = 0;
    p_foc->Ic = 0;
    p_foc->Id = 0;  /* Feedback */
    p_foc->Iq = 0;
    p_foc->Ialpha = 0; /* User view Phase values */
    p_foc->Ibeta = 0;
    p_foc->ReqD = 0; /* Req */
    p_foc->ReqQ = 0;
//     p_foc->Va = 0; /* Clear in case of VSpeed selected over VBemf */
//     p_foc->Vb = 0;
//     p_foc->Vc = 0;
//     p_foc->Valpha = 0;
//     p_foc->Vbeta = 0;
//     p_foc->Vd = 0; /* Output/Bemf */
//     p_foc->Vq = 0;
    FOC_ZeroSvpwm(p_foc);
}

