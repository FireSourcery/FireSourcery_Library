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
    @file   Phase.c
    @author FireSourcery
    @version V0
    @brief     Phase module conventional function definitions
*/
/******************************************************************************/
#include "Phase.h"

void Phase_Init(Phase_T * p_phase)
{
    PWM_Module_Init(&p_phase->PwmModule);
    PWM_Channel_Init(&p_phase->PwmA);
    PWM_Channel_Init(&p_phase->PwmB);
    PWM_Channel_Init(&p_phase->PwmC);
    Pin_Output_Init(&p_phase->PinA);
    Pin_Output_Init(&p_phase->PinB);
    Pin_Output_Init(&p_phase->PinC);
    p_phase->PolarMode = PHASE_MODE_UNIPOLAR_1;
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    Duty 100% == CONFIG_PWM_DUTY_MAX
*/
void Phase_WriteDuty(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
    PWM_WriteDuty(&p_phase->PwmA, pwmDutyA);
    PWM_WriteDuty(&p_phase->PwmB, pwmDutyB);
    PWM_WriteDuty(&p_phase->PwmC, pwmDutyC);
    _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

void Phase_WriteDuty_Fract16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
    PWM_WriteDuty_Fract16(&p_phase->PwmA, pwmDutyA);
    PWM_WriteDuty_Fract16(&p_phase->PwmB, pwmDutyB);
    PWM_WriteDuty_Fract16(&p_phase->PwmC, pwmDutyC);
    _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

void Phase_WriteDuty_Percent16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
    PWM_WriteDuty_Percent16(&p_phase->PwmA, pwmDutyA);
    PWM_WriteDuty_Percent16(&p_phase->PwmB, pwmDutyB);
    PWM_WriteDuty_Percent16(&p_phase->PwmC, pwmDutyC);
    _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

void Phase_ActivateOutput(const Phase_T * p_phase)
{
    _Phase_EnableA(p_phase);
    _Phase_EnableB(p_phase);
    _Phase_EnableC(p_phase);
    // _Phase_SyncPwmOnOff(p_phase);
    // _Phase_WriteState(p_phase, PHASE_ID_ABC);
}

void Phase_Float(const Phase_T * p_phase)
{
    _Phase_DisableA(p_phase);
    _Phase_DisableB(p_phase);
    _Phase_DisableC(p_phase);
    // _Phase_SyncPwmOnOff(p_phase);
    // _Phase_WriteState(p_phase, PHASE_ID_DISABLE);
}

/* NOT for Bipolar active. Enable all at 0 duty */
void Phase_Ground(const Phase_T * p_phase)
{
    assert(p_phase->PolarMode != PHASE_MODE_BIPOLAR);
    Phase_WriteDuty(p_phase, 0U, 0U, 0U);
    Phase_ActivateOutput(p_phase);
}

bool Phase_IsFloat(const Phase_T * p_phase)
{
    return (_Phase_ReadState(p_phase).Value == PHASE_ID_DISABLE);
}

bool Phase_IsGround(const Phase_T * p_phase)
{
    return ((_Phase_ReadState(p_phase).Value == PHASE_ID_ABC) && (_Phase_ReadDutyState(p_phase).Value == PHASE_ID_DISABLE));
}

Phase_Output_T Phase_ReadOutputState(const Phase_T * p_phase)
{
    Phase_Output_T state;
    if      (Phase_IsFloat(p_phase) == true)    { state = PHASE_OUTPUT_FLOAT; }
    else if (Phase_IsGround(p_phase) == true)   { state = PHASE_OUTPUT_GROUND; }
    else                                        { state = PHASE_OUTPUT_VPWM; }
    return state;
}

void Phase_ActivateOutputState(const Phase_T * p_phase, Phase_Output_T state)
{
    switch (state)
    {
        case PHASE_OUTPUT_FLOAT:    Phase_Float(p_phase); break;
        case PHASE_OUTPUT_GROUND:   Phase_Ground(p_phase); break;
        case PHASE_OUTPUT_VPWM:     Phase_ActivateOutput(p_phase); break; /* restore last active */
        // Phase_WriteDuty(p_phase, PWM_DUTY_MAX / 2, PWM_DUTY_MAX / 2, PWM_DUTY_MAX / 2);
        default: assert(false); break;
    }
}


/******************************************************************************/
/*
    3-Phase Align
*/
/******************************************************************************/
/*
    Duty only
*/
static inline void _Align_WriteDuty(const PWM_T * p_vDuty, const PWM_T * p_vGround1, const PWM_T * p_vGround2, uint16_t duty)
{
    PWM_WriteDuty(p_vDuty, duty);    PWM_WriteDuty(p_vGround1, 0U);    PWM_WriteDuty(p_vGround2, 0U);
}

static inline void _AlignInv_WriteDuty(const PWM_T * p_vInv, const PWM_T * p_vDuty1, const PWM_T * p_vDuty2, uint16_t duty)
{
    PWM_WriteDuty(p_vInv, 0U);    PWM_WriteDuty(p_vDuty1, duty / 2);    PWM_WriteDuty(p_vDuty2, duty / 2);
}

void _Phase_AlignA(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, duty, 0U, 0U); }
void _Phase_AlignB(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, 0U, duty, 0U); }
void _Phase_AlignC(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, 0U, 0U, duty); }
void _Phase_AlignInvA(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, 0U, duty/2, duty/2); }
void _Phase_AlignInvB(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, duty/2, 0U, duty/2); }
void _Phase_AlignInvC(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, duty/2, duty/2, 0U); }

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
        case PHASE_ID_DISABLE:  Phase_WriteDuty(p_phase, 0U, 0U, 0U);                     break;
        case PHASE_ID_ABC:      Phase_WriteDuty(p_phase, duty / 2, duty / 2, duty / 2);   break;
        default: break;
    }
}

/*
    Enable Ouput
*/
// void Phase_ActivateAlignA(const Phase_T * p_phase, uint16_t duty)     { _Phase_AlignA(p_phase, duty); Phase_ActivateOutput(p_phase); }
// void Phase_ActivateAlignB(const Phase_T * p_phase, uint16_t duty)     { _Phase_AlignB(p_phase, duty); Phase_ActivateOutput(p_phase); }
// void Phase_ActivateAlignC(const Phase_T * p_phase, uint16_t duty)     { _Phase_AlignC(p_phase, duty); Phase_ActivateOutput(p_phase); }
// void Phase_ActivateAlignInvA(const Phase_T * p_phase, uint16_t duty)  { _Phase_AlignInvA(p_phase, duty); Phase_ActivateOutput(p_phase); }
// void Phase_ActivateAlignInvB(const Phase_T * p_phase, uint16_t duty)  { _Phase_AlignInvB(p_phase, duty); Phase_ActivateOutput(p_phase); }
// void Phase_ActivateAlignInvC(const Phase_T * p_phase, uint16_t duty)  { _Phase_AlignInvC(p_phase, duty); Phase_ActivateOutput(p_phase); }

void Phase_ActivateAlign(const Phase_T * p_phase, Phase_Id_T id, uint16_t duty)
{
    Phase_Align(p_phase, id, duty);
    Phase_ActivateOutput(p_phase);
}

/*
    State
*/
Phase_Id_T Phase_ReadAlign(const Phase_T * p_phase) { return _Phase_ReadDutyState(p_phase).Value; }

Phase_Id_T Phase_ReadAlignNext(const Phase_T * p_phase) { return Phase_NextOf(Phase_ReadAlign(p_phase)); }

Phase_Id_T Phase_ReadAlignNextDirection(const Phase_T * p_phase, bool ccw)
{
    return (ccw == true) ? Phase_NextOf(Phase_ReadAlign(p_phase)) : Phase_PrevOf(Phase_ReadAlign(p_phase));
}

// Phase_Id_T Phase_JogDirection(const Phase_T * p_phase, int16_t dutySigned)
Phase_Id_T Phase_JogDirection(const Phase_T * p_phase, uint16_t duty, bool ccw)
{
    Phase_Id_T id = Phase_ReadAlignNextDirection(p_phase, ccw);
    Phase_Align(p_phase, id, duty);
    return id;
}
/******************************************************************************/
/*! */
/******************************************************************************/

/******************************************************************************/
/*!
    2-Phase Polar PWM
*/
/******************************************************************************/
/* Enable all at 0 duty */
void Phase_Polar_Ground(const Phase_T * p_phase)
{
    if(p_phase->PolarMode == PHASE_MODE_BIPOLAR)
    {
        Phase_Float(p_phase);
        PWM_DisableInvertPolarity(&p_phase->PwmA);
        PWM_DisableInvertPolarity(&p_phase->PwmB);
        PWM_DisableInvertPolarity(&p_phase->PwmC);
    }

    Phase_Ground(p_phase);
}

static void _Phase_ActivateOutputNotA(const Phase_T * p_phase) { _Phase_DisableA(p_phase); _Phase_EnableB(p_phase); _Phase_EnableC(p_phase);   }
static void _Phase_ActivateOutputNotB(const Phase_T * p_phase) { _Phase_EnableA(p_phase); _Phase_DisableB(p_phase); _Phase_EnableC(p_phase);   }
static void _Phase_ActivateOutputNotC(const Phase_T * p_phase) { _Phase_EnableA(p_phase); _Phase_EnableB(p_phase); _Phase_DisableC(p_phase);   }


/*
    For Bipolar Pwm
*/
// todo conditional _Phase_SyncPwmInvert
static void _Phase_ActivateInvertPolarityA(const Phase_T * p_phase) { PWM_EnableInvertPolarity(&p_phase->PwmA); PWM_DisableInvertPolarity(&p_phase->PwmB); PWM_DisableInvertPolarity(&p_phase->PwmC); _Phase_SyncPwmInvert(p_phase, PHASE_ID_A); }
static void _Phase_ActivateInvertPolarityB(const Phase_T * p_phase) { PWM_DisableInvertPolarity(&p_phase->PwmA); PWM_EnableInvertPolarity(&p_phase->PwmB); PWM_DisableInvertPolarity(&p_phase->PwmC); _Phase_SyncPwmInvert(p_phase, PHASE_ID_B); }
static void _Phase_ActivateInvertPolarityC(const Phase_T * p_phase) { PWM_DisableInvertPolarity(&p_phase->PwmA); PWM_DisableInvertPolarity(&p_phase->PwmB); PWM_EnableInvertPolarity(&p_phase->PwmC); _Phase_SyncPwmInvert(p_phase, PHASE_ID_C); }

/*
    Activate Switches Only
*/
void Phase_Polar_ActivateOutputAC(const Phase_T * p_phase) { if(p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
void Phase_Polar_ActivateOutputBC(const Phase_T * p_phase) { if(p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
void Phase_Polar_ActivateOutputBA(const Phase_T * p_phase) { if(p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotC(p_phase); }
void Phase_Polar_ActivateOutputCA(const Phase_T * p_phase) { if(p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
void Phase_Polar_ActivateOutputCB(const Phase_T * p_phase) { if(p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
void Phase_Polar_ActivateOutputAB(const Phase_T * p_phase) { if(p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotC(p_phase); }


void Phase_Unipolar_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId)
{
    switch (phaseId)
    {
        case PHASE_ID_0: break;
        case PHASE_ID_1_AC: _Phase_ActivateOutputNotB(p_phase); break; /*  _Phase_WriteState(p_phase, PHASE_ID_INV_B); break; */
        case PHASE_ID_4_CA: _Phase_ActivateOutputNotB(p_phase); break;
        case PHASE_ID_6_AB: _Phase_ActivateOutputNotC(p_phase); break;
        case PHASE_ID_3_BA: _Phase_ActivateOutputNotC(p_phase); break;
        case PHASE_ID_5_CB: _Phase_ActivateOutputNotA(p_phase); break;
        case PHASE_ID_2_BC: _Phase_ActivateOutputNotA(p_phase); break;
        case PHASE_ID_7: break;
        default: break;
    }
}

/*
    Activate Duty Only
*/
// void _Unipolar1_WriteDuty(const PWM_T * p_vDuty, const PWM_T * p_vGround, uint16_t duty) { PWM_WriteDuty(p_vDuty, duty); PWM_WriteDuty(p_vGround, 0U); }

void Phase_Unipolar1_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmA, duty); PWM_WriteDuty(&p_phase->PwmC, 0U); }
void Phase_Unipolar1_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmB, duty); PWM_WriteDuty(&p_phase->PwmC, 0U); }
void Phase_Unipolar1_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmB, duty); PWM_WriteDuty(&p_phase->PwmA, 0U); }
void Phase_Unipolar1_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmC, duty); PWM_WriteDuty(&p_phase->PwmA, 0U); }
void Phase_Unipolar1_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmC, duty); PWM_WriteDuty(&p_phase->PwmB, 0U); }
void Phase_Unipolar1_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmA, duty); PWM_WriteDuty(&p_phase->PwmB, 0U); }

void Phase_Unipolar1_WriteDuty(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
{
    switch (phaseId)
    {
        case PHASE_ID_0: break;
        case PHASE_ID_1_AC: Phase_Unipolar1_ActivateDutyAC(p_phase, duty); break;
        case PHASE_ID_2_BC: Phase_Unipolar1_ActivateDutyBC(p_phase, duty); break;
        case PHASE_ID_3_BA: Phase_Unipolar1_ActivateDutyBA(p_phase, duty); break;
        case PHASE_ID_4_CA: Phase_Unipolar1_ActivateDutyCA(p_phase, duty); break;
        case PHASE_ID_5_CB: Phase_Unipolar1_ActivateDutyCB(p_phase, duty); break;
        case PHASE_ID_6_AB: Phase_Unipolar1_ActivateDutyAB(p_phase, duty); break;
        case PHASE_ID_7: break;
        default: break;
    }
}


/*
    PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
    PwmNegative = PwmPeriodTotal/2 - PwmScalar/2
*/
void Phase_Unipolar2_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmC, duty); }
void Phase_Unipolar2_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmC, duty); }
void Phase_Unipolar2_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmA, duty); }
void Phase_Unipolar2_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmA, duty); }
void Phase_Unipolar2_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmB, duty); }
void Phase_Unipolar2_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmB, duty); }

/*

*/
// void Phase_Bipolar_ActivateOutputAC(const Phase_T * p_phase) { { Phase_Float(p_phase); _Phase_InvertPolarityC(p_phase, PHASE_ID_C); } _Phase_ActivateOutput(p_phase, PHASE_ID_INV_B); }
void Phase_Bipolar_ActivateOutputAC(const Phase_T * p_phase) { { Phase_Float(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
void Phase_Bipolar_ActivateOutputBC(const Phase_T * p_phase) { { Phase_Float(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
void Phase_Bipolar_ActivateOutputBA(const Phase_T * p_phase) { { Phase_Float(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotC(p_phase); }
void Phase_Bipolar_ActivateOutputCA(const Phase_T * p_phase) { { Phase_Float(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
void Phase_Bipolar_ActivateOutputCB(const Phase_T * p_phase) { { Phase_Float(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
void Phase_Bipolar_ActivateOutputAB(const Phase_T * p_phase) { { Phase_Float(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotC(p_phase); }

/*
    PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
    PwmNegative = PwmPeriodTotal/2 + PwmScalar/2, Inverse polarity
*/
void Phase_Bipolar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); }
void Phase_Bipolar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); }
void Phase_Bipolar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); }
void Phase_Bipolar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); }
void Phase_Bipolar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); }
void Phase_Bipolar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); }

// todo wrap layering
void Phase_Polar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PolarMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyAC(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyAC(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyAC(p_phase, duty);    break;
        default: break;
    }
     _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

void Phase_Polar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PolarMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyBC(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyBC(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyBC(p_phase, duty);    break;
        default: break;
    }
     _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

void Phase_Polar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PolarMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyBA(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyBA(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyBA(p_phase, duty);    break;
        default: break;
    }
     _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

void Phase_Polar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PolarMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyCA(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyCA(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyCA(p_phase, duty);    break;
        default: break;
    }
     _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

void Phase_Polar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PolarMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyCB(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyCB(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyCB(p_phase, duty);    break;
        default: break;
    }
     _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

void Phase_Polar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PolarMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyAB(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyAB(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyAB(p_phase, duty);    break;
        default: break;
    }
     _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

/*
    Activate Duty and Sets On/Off State
*/
void Phase_Polar_ActivateAC(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyAC(p_phase, duty); Phase_Polar_ActivateOutputAC(p_phase); }
void Phase_Polar_ActivateBC(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyBC(p_phase, duty); Phase_Polar_ActivateOutputBC(p_phase); }
void Phase_Polar_ActivateBA(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyBA(p_phase, duty); Phase_Polar_ActivateOutputBA(p_phase); }
void Phase_Polar_ActivateCA(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyCA(p_phase, duty); Phase_Polar_ActivateOutputCA(p_phase); }
void Phase_Polar_ActivateCB(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyCB(p_phase, duty); Phase_Polar_ActivateOutputCB(p_phase); }
void Phase_Polar_ActivateAB(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyAB(p_phase, duty); Phase_Polar_ActivateOutputAB(p_phase); }
/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*!

*/
/******************************************************************************/
void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Mode_T phaseMode)
{
    if(p_phase->PolarMode == PHASE_MODE_BIPOLAR) /* If current phase mode is Bipolar, disable */
    {
        Phase_Float(p_phase);
        PWM_DisableInvertPolarity(&p_phase->PwmA);
        PWM_DisableInvertPolarity(&p_phase->PwmB);
        PWM_DisableInvertPolarity(&p_phase->PwmC);
    }
    p_phase->PolarMode = phaseMode;
}

/*
    Commutation Phase Id handled by Phase module
    switch should be faster than CommutationTable[phaseID][p_phase->PolarMode]();
*/
void Phase_Polar_Activate(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
{
    switch(phaseId)
    {
        case PHASE_ID_0: break;
        case PHASE_ID_1_AC: Phase_Polar_ActivateAC(p_phase, duty); break;
        case PHASE_ID_2_BC: Phase_Polar_ActivateBC(p_phase, duty); break;
        case PHASE_ID_3_BA: Phase_Polar_ActivateBA(p_phase, duty); break;
        case PHASE_ID_4_CA: Phase_Polar_ActivateCA(p_phase, duty); break;
        case PHASE_ID_5_CB: Phase_Polar_ActivateCB(p_phase, duty); break;
        case PHASE_ID_6_AB: Phase_Polar_ActivateAB(p_phase, duty); break;
        case PHASE_ID_7: break;
        default: break;
    }
}

void Phase_Polar_ActivateDuty(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
{
    switch(phaseId)
    {
        case PHASE_ID_0: break;
        case PHASE_ID_1_AC: Phase_Polar_ActivateDutyAC(p_phase, duty); break;
        case PHASE_ID_2_BC: Phase_Polar_ActivateDutyBC(p_phase, duty); break;
        case PHASE_ID_3_BA: Phase_Polar_ActivateDutyBA(p_phase, duty); break;
        case PHASE_ID_4_CA: Phase_Polar_ActivateDutyCA(p_phase, duty); break;
        case PHASE_ID_5_CB: Phase_Polar_ActivateDutyCB(p_phase, duty); break;
        case PHASE_ID_6_AB: Phase_Polar_ActivateDutyAB(p_phase, duty); break;
        case PHASE_ID_7: break;
        default: break;
    }
}

void Phase_Polar_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId)
{
    switch(phaseId)
    {
        case PHASE_ID_0: break;
        case PHASE_ID_1_AC: Phase_Polar_ActivateOutputAC(p_phase); break;
        case PHASE_ID_2_BC: Phase_Polar_ActivateOutputBC(p_phase); break;
        case PHASE_ID_3_BA: Phase_Polar_ActivateOutputBA(p_phase); break;
        case PHASE_ID_4_CA: Phase_Polar_ActivateOutputCA(p_phase); break;
        case PHASE_ID_5_CB: Phase_Polar_ActivateOutputCB(p_phase); break;
        case PHASE_ID_6_AB: Phase_Polar_ActivateOutputAB(p_phase); break;
        case PHASE_ID_7: break;
        default: break;
    }

    // switch (p_phase->PolarMode)
    // {
    //     case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar_ActivateOutput(p_phase, phaseId);    break;
    //     case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar_ActivateOutput(p_phase, phaseId);    break;
    //     case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateOutput(p_phase, phaseId);    break;
    //     default: break;
    // }
}
