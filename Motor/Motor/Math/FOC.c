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

*/
/******************************************************************************/
#include "FOC.h"


// void FOC_InitModulationFull(FOC_T * p_foc) { p_foc->Modulation = FRACT16_MAX; }
// void FOC_InitModulationLinear(FOC_T * p_foc) { p_foc->Modulation = FRACT16_SQRT3_DIV_2; }

void FOC_Init(FOC_T * p_foc)
{
    // p_foc->Modulation = FRACT16_MAX;
    // FOC_CaptureVBus(p_foc, vBus);
    /* or handle outside */
    // p_foc->VdLimit = fract16_mul(fract16_mul(vBus, FRACT16_1_DIV_SQRT3), p_foc->Modulation);
    // p_foc->VPhaseLimit = fract16_mul(fract16_mul(vBus, FRACT16_1_DIV_SQRT3), p_foc->Modulation);
}

/* Prep Align using input intensity */
void FOC_SetAlign(FOC_T * p_foc, fract16_t vd)
{
    p_foc->Vd = vd;
    p_foc->Vq = 0;
    p_foc->Sine = 0;
    p_foc->Cosine = FRACT16_MAX;
}

void FOC_ZeroSvpwm(FOC_T * p_foc)
{
    p_foc->DutyA = FRACT16_1_DIV_2;
    p_foc->DutyB = FRACT16_1_DIV_2;
    p_foc->DutyC = FRACT16_1_DIV_2;
}

void FOC_ClearCaptureState(FOC_T * p_foc)
{
    // p_foc->Ia = 0;  /* ADC */
    // p_foc->Ib = 0;
    // p_foc->Ic = 0;
    p_foc->Id = 0;  /* Feedback */
    p_foc->Iq = 0;
    p_foc->Ialpha = 0; /* User view Phase values */
    p_foc->Ibeta = 0;
    // p_foc->ReqD = 0; /* Req */
    // p_foc->ReqQ = 0;
    // p_foc->Va = 0;
    // p_foc->Vb = 0;
    // p_foc->Vc = 0;
//     p_foc->Valpha = 0;
//     p_foc->Vbeta = 0;
//     p_foc->Vd = 0;
//     p_foc->Vq = 0;
    // FOC_ZeroSvpwm(p_foc);
}


// bool FOC_ValidateInputs(const FOC_T * p_foc)
// {
//     /* Check for reasonable current values */
//     if (abs(p_foc->Ia) > FRACT16_MAX * 9 / 10) return false;
//     if (abs(p_foc->Ib) > FRACT16_MAX * 9 / 10) return false;
//     if (abs(p_foc->Ic) > FRACT16_MAX * 9 / 10) return false;

//     /* Validate Kirchhoff's current law: Ia + Ib + Ic ≈ 0 */
//     int32_t currentSum = (int32_t)p_foc->Ia + p_foc->Ib + p_foc->Ic;
//     if (abs(currentSum) > FRACT16_MAX / 20) return false;  /* 5% tolerance */

//     /* Check for reasonable voltage requests */
//     int32_t vMagSq = (int32_t)p_foc->Vd * p_foc->Vd + (int32_t)p_foc->Vq * p_foc->Vq;
//     if (vMagSq > (int32_t)FRACT16_MAX * FRACT16_MAX) return false;

//     return true;
// }

bool FOC_ValidateTheta(fract16_t sine, fract16_t cosine)
{
    /* Validate unit circle: sin²θ + cos²θ ≈ 1 */
    int32_t magnitudeSquared = (int32_t)sine * sine + (int32_t)cosine * cosine;
    int32_t expected = (int32_t)FRACT16_MAX * FRACT16_MAX;
    int32_t tolerance = expected / 20;  /* 5% tolerance */

    return abs(magnitudeSquared - expected) < tolerance;
}


