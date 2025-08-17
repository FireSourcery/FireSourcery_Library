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
static void _Align_WriteDuty(const PWM_T * p_vDuty, const PWM_T * p_vGround1, const PWM_T * p_vGround2, uint16_t duty)
{
    PWM_WriteDuty(p_vDuty, duty);
    PWM_WriteDuty(p_vGround1, 0U);
    PWM_WriteDuty(p_vGround2, 0U);
}

static void _AlignInv_WriteDuty(const PWM_T * p_vInv, const PWM_T * p_vDuty1, const PWM_T * p_vDuty2, uint16_t duty)
{
    PWM_WriteDuty(p_vInv, 0U);
    PWM_WriteDuty(p_vDuty1, duty / 2U);
    PWM_WriteDuty(p_vDuty2, duty / 2U);
}

void _Phase_AlignA(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, duty, 0U, 0U); } /* _Align_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_B, &p_phase->PWM_C, duty);  */
void _Phase_AlignB(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, 0U, duty, 0U); }
void _Phase_AlignC(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, 0U, 0U, duty); }
void _Phase_AlignInvA(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, 0U, duty/2, duty/2); }
void _Phase_AlignInvB(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, duty/2, 0U, duty/2); }
void _Phase_AlignInvC(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, duty/2, duty/2, 0U); }


/* Duty Only */
void Phase_Align(const Phase_T * p_phase, Phase_Id_T id, uint16_t duty)
{
    switch (id)
    {
        case PHASE_ID_A:        _Phase_AlignA(p_phase, duty);           break;
        case PHASE_ID_INV_C:    _Phase_AlignInvC(p_phase, duty);        break;
        case PHASE_ID_B:        _Phase_AlignB(p_phase, duty);           break;
        case PHASE_ID_INV_A:    _Phase_AlignInvA(p_phase, duty);        break;
        case PHASE_ID_C:        _Phase_AlignC(p_phase, duty);           break;
        case PHASE_ID_INV_B:    _Phase_AlignInvB(p_phase, duty);        break;
        case PHASE_ID_0:        Phase_WriteDuty(p_phase, 0U, 0U, 0U);                     break;
        case PHASE_ID_ABC:      Phase_WriteDuty(p_phase, duty / 2, duty / 2, duty / 2);   break;
        default: break;
    }
}

/* 1 as 1/2 vBus */
void Phase_Align_VScalar(const Phase_T * p_phase, Phase_Id_T id, uint16_t scalar_fract16)
{
    Phase_Align(p_phase, id, scalar_fract16 * 3 / 4);
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
Phase_Id_T Phase_ReadAlign(const Phase_T * p_phase) { return _Phase_ReadDutyState(p_phase).Id; }
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
// void Phase_JogNext(const Phase_T * p_phase, uint16_t duty)
// {
//     Phase_Align(p_phase, Phase_NextOf(Phase_ReadAlign(p_phase)), duty);
// }

// void Phase_JogPrev(const Phase_T * p_phase, uint16_t duty)
// {
//     Phase_Align(p_phase, Phase_PrevOf(Phase_ReadAlign(p_phase)), duty);
// }

// void Phase_JogSigned(const Phase_T * p_phase, int16_t dutySigned)
// {
//     if      (dutySigned > 0) { Phase_JogNext(p_phase, (uint16_t)dutySigned); }
//     else if (dutySigned < 0) { Phase_JogPrev(p_phase, (uint16_t)(0 - dutySigned)); }
//     else                     { /* no change */ }
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