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
    @file   Phase_Polar.c
    @author FireSourcery

*/
/******************************************************************************/
#include "Phase_Polar.h"

/******************************************************************************/
/*!
    2-Phase Polar PWM
*/
/******************************************************************************/
// void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Polar_Mode_T phaseMode)
// {
//     if(p_phase->PolarMode == PHASE_MODE_BIPOLAR) /* If current phase mode is Bipolar, disable */
//     {
//         Phase_Float(p_phase);
//         PWM_DisableInvertPolarity(&p_phase->PWM_A);
//         PWM_DisableInvertPolarity(&p_phase->PWM_B);
//         PWM_DisableInvertPolarity(&p_phase->PWM_C);
//     }
//     p_phase->PolarMode = phaseMode;
// }

/* Enable all at 0 duty */
void Phase_Polar_Ground(const Phase_T * p_phase)
{
    // if (p_phase->PolarMode == PHASE_MODE_BIPOLAR)
    {
        Phase_Float(p_phase);
        PWM_DisableInvertPolarity(&p_phase->PWM_A);
        PWM_DisableInvertPolarity(&p_phase->PWM_B);
        PWM_DisableInvertPolarity(&p_phase->PWM_C);
    }

    Phase_ActivateOutputV0(p_phase);
}

static void _Phase_ActivateOutputNotA(const Phase_T * p_phase) { _Phase_DisableA(p_phase); _Phase_EnableB(p_phase); _Phase_EnableC(p_phase); }
static void _Phase_ActivateOutputNotB(const Phase_T * p_phase) { _Phase_EnableA(p_phase); _Phase_DisableB(p_phase); _Phase_EnableC(p_phase); }
static void _Phase_ActivateOutputNotC(const Phase_T * p_phase) { _Phase_EnableA(p_phase); _Phase_EnableB(p_phase); _Phase_DisableC(p_phase); }


/*
    For Bipolar Pwm
*/
// todo conditional _Phase_SyncPwmInvert
static void _Phase_ActivateInvertPolarityA(const Phase_T * p_phase) { PWM_EnableInvertPolarity(&p_phase->PWM_A); PWM_DisableInvertPolarity(&p_phase->PWM_B); PWM_DisableInvertPolarity(&p_phase->PWM_C); _Phase_SyncPwmInvert(p_phase, PHASE_ID_A); }
static void _Phase_ActivateInvertPolarityB(const Phase_T * p_phase) { PWM_DisableInvertPolarity(&p_phase->PWM_A); PWM_EnableInvertPolarity(&p_phase->PWM_B); PWM_DisableInvertPolarity(&p_phase->PWM_C); _Phase_SyncPwmInvert(p_phase, PHASE_ID_B); }
static void _Phase_ActivateInvertPolarityC(const Phase_T * p_phase) { PWM_DisableInvertPolarity(&p_phase->PWM_A); PWM_DisableInvertPolarity(&p_phase->PWM_B); PWM_EnableInvertPolarity(&p_phase->PWM_C); _Phase_SyncPwmInvert(p_phase, PHASE_ID_C); }

/*
    Activate Switches Only
*/
// void Phase_Polar_ActivateOutputAC(const Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
// void Phase_Polar_ActivateOutputBC(const Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
// void Phase_Polar_ActivateOutputBA(const Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotC(p_phase); }
// void Phase_Polar_ActivateOutputCA(const Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
// void Phase_Polar_ActivateOutputCB(const Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
// void Phase_Polar_ActivateOutputAB(const Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotC(p_phase); }


void Phase_Unipolar_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId)
{
    switch (phaseId)
    {
        case PHASE_ID_POLAR_0: break;
        case PHASE_ID_1_AC: _Phase_ActivateOutputNotB(p_phase); break; /*  _Phase_WriteState(p_phase, PHASE_ID_INV_B); break; */
        case PHASE_ID_4_CA: _Phase_ActivateOutputNotB(p_phase); break;
        case PHASE_ID_6_AB: _Phase_ActivateOutputNotC(p_phase); break;
        case PHASE_ID_3_BA: _Phase_ActivateOutputNotC(p_phase); break;
        case PHASE_ID_5_CB: _Phase_ActivateOutputNotA(p_phase); break;
        case PHASE_ID_2_BC: _Phase_ActivateOutputNotA(p_phase); break;
        case PHASE_ID_POLAR_7: break;
        default: break;
    }
}

/*
    Activate Duty Only
*/
static inline void _Phase_Unipolar1_WriteDuty(const PWM_T * p_vDuty, const PWM_T * p_vGround, uint16_t duty) { PWM_WriteDuty(p_vDuty, duty); PWM_WriteDuty(p_vGround, 0U); }
// void Phase_Unipolar1_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PWM_A, duty); PWM_WriteDuty(&p_phase->PWM_C, 0U); }
// void Phase_Unipolar1_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PWM_B, duty); PWM_WriteDuty(&p_phase->PWM_C, 0U); }
// void Phase_Unipolar1_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PWM_B, duty); PWM_WriteDuty(&p_phase->PWM_A, 0U); }
// void Phase_Unipolar1_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PWM_C, duty); PWM_WriteDuty(&p_phase->PWM_A, 0U); }
// void Phase_Unipolar1_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PWM_C, duty); PWM_WriteDuty(&p_phase->PWM_B, 0U); }
// void Phase_Unipolar1_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PWM_A, duty); PWM_WriteDuty(&p_phase->PWM_B, 0U); }

void Phase_Unipolar1_WriteDuty(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
{
    switch (phaseId)
    {
        case PHASE_ID_POLAR_0: break;
        case PHASE_ID_1_AC: _Phase_Unipolar1_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_C, duty); break;
        case PHASE_ID_2_BC: _Phase_Unipolar1_WriteDuty(&p_phase->PWM_B, &p_phase->PWM_C, duty); break;
        case PHASE_ID_3_BA: _Phase_Unipolar1_WriteDuty(&p_phase->PWM_B, &p_phase->PWM_A, duty); break;
        case PHASE_ID_4_CA: _Phase_Unipolar1_WriteDuty(&p_phase->PWM_C, &p_phase->PWM_A, duty); break;
        case PHASE_ID_5_CB: _Phase_Unipolar1_WriteDuty(&p_phase->PWM_C, &p_phase->PWM_B, duty); break;
        case PHASE_ID_6_AB: _Phase_Unipolar1_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_B, duty); break;
        // case PHASE_ID_1_AC: Phase_Unipolar1_ActivateDutyAC(p_phase, duty); break;
        // case PHASE_ID_2_BC: Phase_Unipolar1_ActivateDutyBC(p_phase, duty); break;
        // case PHASE_ID_3_BA: Phase_Unipolar1_ActivateDutyBA(p_phase, duty); break;
        // case PHASE_ID_4_CA: Phase_Unipolar1_ActivateDutyCA(p_phase, duty); break;
        // case PHASE_ID_5_CB: Phase_Unipolar1_ActivateDutyCB(p_phase, duty); break;
        // case PHASE_ID_6_AB: Phase_Unipolar1_ActivateDutyAB(p_phase, duty); break;
        case PHASE_ID_POLAR_7: break;
        default: break;
    }
}


/*
    PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
    PwmNegative = PwmPeriodTotal/2 - PwmScalar/2
*/
void Phase_Unipolar2_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_A, duty); PWM_WriteDutyMidMinus(&p_phase->PWM_C, duty); }
void Phase_Unipolar2_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_B, duty); PWM_WriteDutyMidMinus(&p_phase->PWM_C, duty); }
void Phase_Unipolar2_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_B, duty); PWM_WriteDutyMidMinus(&p_phase->PWM_A, duty); }
void Phase_Unipolar2_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_C, duty); PWM_WriteDutyMidMinus(&p_phase->PWM_A, duty); }
void Phase_Unipolar2_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_C, duty); PWM_WriteDutyMidMinus(&p_phase->PWM_B, duty); }
void Phase_Unipolar2_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_A, duty); PWM_WriteDutyMidMinus(&p_phase->PWM_B, duty); }

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
void Phase_Bipolar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_A, duty); PWM_WriteDutyMidPlus(&p_phase->PWM_C, duty); }
void Phase_Bipolar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_B, duty); PWM_WriteDutyMidPlus(&p_phase->PWM_C, duty); }
void Phase_Bipolar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_B, duty); PWM_WriteDutyMidPlus(&p_phase->PWM_A, duty); }
void Phase_Bipolar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_C, duty); PWM_WriteDutyMidPlus(&p_phase->PWM_A, duty); }
void Phase_Bipolar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_C, duty); PWM_WriteDutyMidPlus(&p_phase->PWM_B, duty); }
void Phase_Bipolar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDutyMidPlus(&p_phase->PWM_A, duty); PWM_WriteDutyMidPlus(&p_phase->PWM_B, duty); }


/******************************************************************************/
/*!

*/
/******************************************************************************/
// layering todo
// /*
//     Commutation Phase Bits handled by Phase module
//     switch should be faster than CommutationTable[phaseID][p_phase->PolarMode]();
// */
// void Phase_Polar_Activate(Phase_T * p_phase,  Phase_Polar_T phaseId, uint16_t duty)
// {
//     switch(phaseId)
//     {
//         case PHASE_ID_POLAR_0: break;
//         case PHASE_ID_1_AC: Phase_Polar_ActivateAC(p_phase, duty); break;
//         case PHASE_ID_2_BC: Phase_Polar_ActivateBC(p_phase, duty); break;
//         case PHASE_ID_3_BA: Phase_Polar_ActivateBA(p_phase, duty); break;
//         case PHASE_ID_4_CA: Phase_Polar_ActivateCA(p_phase, duty); break;
//         case PHASE_ID_5_CB: Phase_Polar_ActivateCB(p_phase, duty); break;
//         case PHASE_ID_6_AB: Phase_Polar_ActivateAB(p_phase, duty); break;
//         case PHASE_ID_POLAR_7: break;
//         default: break;
//     }
// }

// void Phase_Polar_ActivateDuty(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
// {
//     switch(phaseId)
//     {
//         case PHASE_ID_POLAR_0: break;
//         case PHASE_ID_1_AC: Phase_Polar_ActivateDutyAC(p_phase, duty); break;
//         case PHASE_ID_2_BC: Phase_Polar_ActivateDutyBC(p_phase, duty); break;
//         case PHASE_ID_3_BA: Phase_Polar_ActivateDutyBA(p_phase, duty); break;
//         case PHASE_ID_4_CA: Phase_Polar_ActivateDutyCA(p_phase, duty); break;
//         case PHASE_ID_5_CB: Phase_Polar_ActivateDutyCB(p_phase, duty); break;
//         case PHASE_ID_6_AB: Phase_Polar_ActivateDutyAB(p_phase, duty); break;
//         case PHASE_ID_POLAR_7: break;
//         default: break;
//     }
// }

// void Phase_Polar_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId)
// {
//     switch(phaseId)
//     {
//         case PHASE_ID_POLAR_0: break;
//         case PHASE_ID_1_AC: Phase_Polar_ActivateOutputAC(p_phase); break;
//         case PHASE_ID_2_BC: Phase_Polar_ActivateOutputBC(p_phase); break;
//         case PHASE_ID_3_BA: Phase_Polar_ActivateOutputBA(p_phase); break;
//         case PHASE_ID_4_CA: Phase_Polar_ActivateOutputCA(p_phase); break;
//         case PHASE_ID_5_CB: Phase_Polar_ActivateOutputCB(p_phase); break;
//         case PHASE_ID_6_AB: Phase_Polar_ActivateOutputAB(p_phase); break;
//         case PHASE_ID_POLAR_7: break;
//         default: break;
//     }

//     // switch (p_phase->PolarMode)
//     // {
//     //     case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar_ActivateOutput(p_phase, phaseId);    break;
//     //     case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar_ActivateOutput(p_phase, phaseId);    break;
//     //     case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateOutput(p_phase, phaseId);    break;
//     //     default: break;
//     // }
// }
// redo wrap layering
// void Phase_Polar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty)
// {
//     switch(p_phase->PolarMode)
//     {
//         case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyAC(p_phase, duty);    break;
//         case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyAC(p_phase, duty);    break;
//         case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyAC(p_phase, duty);    break;
//         default: break;
//     }
//      _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
// }

// void Phase_Polar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty)
// {
//     switch(p_phase->PolarMode)
//     {
//         case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyBC(p_phase, duty);    break;
//         case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyBC(p_phase, duty);    break;
//         case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyBC(p_phase, duty);    break;
//         default: break;
//     }
//      _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
// }

// void Phase_Polar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty)
// {
//     switch(p_phase->PolarMode)
//     {
//         case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyBA(p_phase, duty);    break;
//         case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyBA(p_phase, duty);    break;
//         case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyBA(p_phase, duty);    break;
//         default: break;
//     }
//      _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
// }

// void Phase_Polar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty)
// {
//     switch(p_phase->PolarMode)
//     {
//         case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyCA(p_phase, duty);    break;
//         case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyCA(p_phase, duty);    break;
//         case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyCA(p_phase, duty);    break;
//         default: break;
//     }
//      _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
// }

// void Phase_Polar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty)
// {
//     switch(p_phase->PolarMode)
//     {
//         case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyCB(p_phase, duty);    break;
//         case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyCB(p_phase, duty);    break;
//         case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyCB(p_phase, duty);    break;
//         default: break;
//     }
//      _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
// }

// void Phase_Polar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty)
// {
//     switch(p_phase->PolarMode)
//     {
//         case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyAB(p_phase, duty);    break;
//         case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyAB(p_phase, duty);    break;
//         case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyAB(p_phase, duty);    break;
//         default: break;
//     }
//      _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
// }

// /*
//     Activate Duty and Sets On/Off State
// */
// void Phase_Polar_ActivateAC(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyAC(p_phase, duty); Phase_Polar_ActivateOutputAC(p_phase); }
// void Phase_Polar_ActivateBC(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyBC(p_phase, duty); Phase_Polar_ActivateOutputBC(p_phase); }
// void Phase_Polar_ActivateBA(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyBA(p_phase, duty); Phase_Polar_ActivateOutputBA(p_phase); }
// void Phase_Polar_ActivateCA(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyCA(p_phase, duty); Phase_Polar_ActivateOutputCA(p_phase); }
// void Phase_Polar_ActivateCB(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyCB(p_phase, duty); Phase_Polar_ActivateOutputCB(p_phase); }
// void Phase_Polar_ActivateAB(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyAB(p_phase, duty); Phase_Polar_ActivateOutputAB(p_phase); }
/******************************************************************************/
/*! @} */
/******************************************************************************/