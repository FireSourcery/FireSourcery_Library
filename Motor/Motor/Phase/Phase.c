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
#include "Phase.h"

void Phase_Init(const Phase_T * p_phase)
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
/*
    Duty only
*/
void Phase_Align(const Phase_T * p_phase, Phase_Id_T id, uint16_t duty)
{
    assert(id != PHASE_ID_ABC);
    const Phase_Bitmask_T state = Phase_Bitmask(id);
    Phase_WriteDuty(p_phase, duty * state.A, duty * state.B, duty * state.C);
}

/* 1 as 1/2 vBus */
void Phase_Align_VScalar(const Phase_T * p_phase, Phase_Id_T id, uint16_t scalar_fract16)
{
    Phase_Align(p_phase, id, (uint32_t)scalar_fract16 * 3 / 4);
}

/*
    Enable Ouput
*/
// void Phase_ActivateAlign(const Phase_T * p_phase, Phase_Id_T id, uint16_t duty)
// {
//     Phase_Align(p_phase, id, duty);
//     Phase_ActivateOutput(p_phase);
// }

/*
    Jog with prev State
*/
/* valid after align output write only */
Phase_Id_T Phase_ReadAlign(const Phase_T * p_phase) { return _Phase_ReadDutyState(p_phase).Bits; }
Phase_Id_T Phase_ReadAlignNext(const Phase_T * p_phase) { return Phase_NextOf(Phase_ReadAlign(p_phase)); }
Phase_Id_T Phase_ReadAlignPrev(const Phase_T * p_phase) { return Phase_PrevOf(Phase_ReadAlign(p_phase)); }

/* A towards B */
Phase_Id_T Phase_JogNext(const Phase_T * p_phase, uint16_t duty)
{
    Phase_Id_T id = Phase_ReadAlignNext(p_phase);
    Phase_Align(p_phase, id, duty);
    return id;
}

Phase_Id_T Phase_JogPrev(const Phase_T * p_phase, uint16_t duty)
{
    Phase_Id_T id = Phase_ReadAlignPrev(p_phase);
    Phase_Align(p_phase, id, duty);
    return id;
}

Phase_Id_T Phase_JogSigned(const Phase_T * p_phase, int16_t dutySigned)
{
    Phase_Id_T id;
    if      (dutySigned > 0) { id = Phase_JogNext(p_phase, (uint16_t)dutySigned); }
    else if (dutySigned < 0) { id = Phase_JogPrev(p_phase, (uint16_t)(0 - dutySigned)); }
    else                     { id = Phase_ReadAlign(p_phase); } /* no change */
    return id;
}

// call check after update
// void _Phase_JogNext(const Phase_T * p_phase, uint16_t duty) { Phase_Align(p_phase, Phase_NextOf(Phase_ReadAlign(p_phase)), duty); }
// void _Phase_JogPrev(const Phase_T * p_phase, uint16_t duty) { Phase_Align(p_phase, Phase_PrevOf(Phase_ReadAlign(p_phase)), duty); }
// void _Phase_JogSigned(const Phase_T * p_phase, int16_t dutySigned)
// {
//     if (dutySigned > 0) { _Phase_JogNext(p_phase, (uint16_t)dutySigned); }
//     else if (dutySigned < 0) { _Phase_JogPrev(p_phase, (uint16_t)(0 - dutySigned)); }
//     else { /* no change */ }
// }


/******************************************************************************/
/*! */
/******************************************************************************/

// typedef enum Motor_Var_Phase
// {
//     MOTOR_VAR_PHASE_OUTPUT,
//     MOTOR_VAR_PHASE_PIN_A,
//     MOTOR_VAR_PHASE_PWM_A,
// }
// Motor_Var_Phase_T;