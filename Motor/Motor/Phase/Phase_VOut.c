/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Phase.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Phase_VOut.h"
#include "Math/Fixed/fract16.h"

void Phase_Init(Phase_VOut_T * p_phase)
{
    PWM_Module_Init(&p_phase->PWM_MODULE);
    PWM_Channel_Init(&p_phase->PWM_A);
    PWM_Channel_Init(&p_phase->PWM_B);
    PWM_Channel_Init(&p_phase->PWM_C);
    Pin_Output_Init(&p_phase->PIN_A);
    Pin_Output_Init(&p_phase->PIN_B);
    Pin_Output_Init(&p_phase->PIN_C);
    // p_phase->PolarMode = PHASE_MODE_UNIPOLAR_1;
}


/******************************************************************************/
/*
    3-Phase Align
*/
/******************************************************************************/
/* VAlign = (2/3) * duty * VBus. scales Ratio to 1/2 VBus */
static inline uint16_t _Phase_VAlignDutyOfRatio(uint16_t ratio) { return (uint32_t)ratio * 3 / 4; }

/* resolved against the live vbus state */
/* duty = (3/2) · VAlign / VBus  =  (3/4) · VAlign / (VBus/2) */
// static inline uint16_t _Phase_VDutyOfPu(uint16_t pu, uint32_t vBusInv_pu) { return math_min( (int32_t)vphase_pu * (int32_t)vBusInv_pu / FRACT16_SCALE * 3 / 2, FRACT16_MAX); }
/* 3/2 on the inverse holds precision at low VBus; the 64-bit product saturates where the int32 form overflows at VAlign >= 2 VBus */
static inline uint16_t _Phase_VAlignDutyOfPu(uint16_t pu, uint32_t vBusInv_pu) { return fract16_sat_positive(accum32_mul(pu, vBusInv_pu * 3U / 2U)); }

/*
    Duty only
*/
void Phase_Align(Phase_VOut_T * p_phase, Phase_Id_T id, uint16_t duty)
{
    Phase_Bitmask_T state = Phase_Bitmask(id);
    Phase_WriteDuty(p_phase, duty * state.A, duty * state.B, duty * state.C);
}

/* 1 as 1/2 vBus */
void Phase_Align_VScalar(Phase_VOut_T * p_phase, Phase_Id_T id, uint16_t scalar)
{
    Phase_Align(p_phase, id, _Phase_VAlignDutyOfRatio(scalar));
}

/* v in the pu base of VBus */
void Phase_Align_V(Phase_VOut_T * p_phase, Phase_Id_T id, uint32_t vBusInv_pu, uint16_t v_pu)
{
    Phase_Align(p_phase, id, _Phase_VAlignDutyOfPu(v_pu, vBusInv_pu));
}

/*
    Enable Ouput
*/
// void Phase_ActivateAlign(Phase_VOut_T * p_phase, Phase_Id_T id, uint16_t duty)
// {
//     Phase_Align(p_phase, id, duty);
//     Phase_ActivateOutput(p_phase);
// }

/*
    Jog with prev State
*/
/* valid after align output write only */
Phase_Id_T Phase_ReadAlign(Phase_VOut_T * p_phase) { return (Phase_Id_T)_Phase_ReadDutyAlign(p_phase).Bits; }
Phase_Id_T Phase_ReadAlignNext(Phase_VOut_T * p_phase) { return Phase_NextOf(Phase_ReadAlign(p_phase)); }
Phase_Id_T Phase_ReadAlignPrev(Phase_VOut_T * p_phase) { return Phase_PrevOf(Phase_ReadAlign(p_phase)); }

/* A towards B */
Phase_Id_T Phase_JogNext(Phase_VOut_T * p_phase, uint16_t duty)
{
    Phase_Id_T id = Phase_ReadAlignNext(p_phase);
    Phase_Align(p_phase, id, duty);
    return id;
}

Phase_Id_T Phase_JogPrev(Phase_VOut_T * p_phase, uint16_t duty)
{
    Phase_Id_T id = Phase_ReadAlignPrev(p_phase);
    Phase_Align(p_phase, id, duty);
    return id;
}

Phase_Id_T Phase_JogSigned(Phase_VOut_T * p_phase, int16_t dutySigned)
{
    Phase_Id_T id;
    if      (dutySigned > 0) { id = Phase_JogNext(p_phase, (uint16_t)dutySigned); }
    else if (dutySigned < 0) { id = Phase_JogPrev(p_phase, (uint16_t)(0 - dutySigned)); }
    else                     { id = Phase_ReadAlign(p_phase); } /* no change */
    return id;
}

// call check after update
// void _Phase_JogNext(Phase_VOut_T * p_phase, uint16_t duty) { Phase_Align(p_phase, Phase_NextOf(Phase_ReadAlign(p_phase)), duty); }
// void _Phase_JogPrev(Phase_VOut_T * p_phase, uint16_t duty) { Phase_Align(p_phase, Phase_PrevOf(Phase_ReadAlign(p_phase)), duty); }
// void _Phase_JogSigned(Phase_VOut_T * p_phase, int16_t dutySigned)
// {
//     if (dutySigned > 0) { _Phase_JogNext(p_phase, (uint16_t)dutySigned); }
//     else if (dutySigned < 0) { _Phase_JogPrev(p_phase, (uint16_t)(0 - dutySigned)); }
//     else { /* no change */ }
// }


/******************************************************************************/
/*! */
/******************************************************************************/

