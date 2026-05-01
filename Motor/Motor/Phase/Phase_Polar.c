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
//         Phase_Deactivate(p_phase);
//         PWM_DisableInvertPolarity(&p_phase->PWM_A);
//         PWM_DisableInvertPolarity(&p_phase->PWM_B);
//         PWM_DisableInvertPolarity(&p_phase->PWM_C);
//     }
//     p_phase->PolarMode = phaseMode;
// }

/* Enable all at 0 duty */
void Phase_Polar_OutputV0(Phase_T * p_phase)
{
    // if (p_phase->PolarMode == PHASE_MODE_BIPOLAR)
    {
        Phase_Deactivate(p_phase);
        PWM_DisableInvertPolarity(&p_phase->PWM_A);
        PWM_DisableInvertPolarity(&p_phase->PWM_B);
        PWM_DisableInvertPolarity(&p_phase->PWM_C);
    }

    Phase_ActivateV0(p_phase);
}

/*
    For Bipolar Pwm
*/
static void _Phase_WriteInvertPol(PWM_T * p_invert, PWM_T * p_nonInv1, PWM_T * p_nonInv2) { PWM_DisableInvertPolarity(p_nonInv1); PWM_DisableInvertPolarity(p_nonInv2); PWM_EnableInvertPolarity(p_invert); }

// todo conditional _Phase_SyncPwmInvert
static void _Phase_ActivateInvertPolarityA(Phase_T * p_phase) { PWM_EnableInvertPolarity(&p_phase->PWM_A); PWM_DisableInvertPolarity(&p_phase->PWM_B); PWM_DisableInvertPolarity(&p_phase->PWM_C); _Phase_SyncPwmInvert(p_phase, PHASE_ID_A); }
static void _Phase_ActivateInvertPolarityB(Phase_T * p_phase) { PWM_DisableInvertPolarity(&p_phase->PWM_A); PWM_EnableInvertPolarity(&p_phase->PWM_B); PWM_DisableInvertPolarity(&p_phase->PWM_C); _Phase_SyncPwmInvert(p_phase, PHASE_ID_B); }
static void _Phase_ActivateInvertPolarityC(Phase_T * p_phase) { PWM_DisableInvertPolarity(&p_phase->PWM_A); PWM_DisableInvertPolarity(&p_phase->PWM_B); PWM_EnableInvertPolarity(&p_phase->PWM_C); _Phase_SyncPwmInvert(p_phase, PHASE_ID_C); }


/******************************************************************************/
/*!
    2-Phase Polar PWM
*/
/******************************************************************************/
void Phase_Unipolar_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId) { _Phase_WriteState(p_phase, phaseId); }

/*
    Activate Duty Only
*/
static inline void _Phase_Unipolar1_WriteDuty(const PWM_T * p_vDuty, const PWM_T * p_vGround, uint16_t duty) { PWM_WriteDuty(p_vDuty, duty); PWM_WriteDuty(p_vGround, 0U); }

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
        case PHASE_ID_POLAR_7: break;
        default: break;
    }
}

/******************************************************************************/
/******************************************************************************/
void Phase_Unipolar2_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId) { _Phase_WriteState(p_phase, phaseId); }

/*
    PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
    PwmNegative = PwmPeriodTotal/2 - PwmScalar/2
*/
static inline void _Phase_Unipolar2_WriteDuty(const PWM_T * p_vPlus, const PWM_T * p_vMinus, uint16_t duty) { PWM_WriteDutyMidPlus(p_vPlus, duty); PWM_WriteDutyMidMinus(p_vMinus, duty); }

/* optionally for table */
void Phase_Unipolar2_ActivateDutyAC(Phase_T * p_phase, uint16_t duty) { _Phase_Unipolar2_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_C, duty); }
void Phase_Unipolar2_ActivateDutyBC(Phase_T * p_phase, uint16_t duty) { _Phase_Unipolar2_WriteDuty(&p_phase->PWM_B, &p_phase->PWM_C, duty); }
void Phase_Unipolar2_ActivateDutyBA(Phase_T * p_phase, uint16_t duty) { _Phase_Unipolar2_WriteDuty(&p_phase->PWM_B, &p_phase->PWM_A, duty); }
void Phase_Unipolar2_ActivateDutyCA(Phase_T * p_phase, uint16_t duty) { _Phase_Unipolar2_WriteDuty(&p_phase->PWM_C, &p_phase->PWM_A, duty); }
void Phase_Unipolar2_ActivateDutyCB(Phase_T * p_phase, uint16_t duty) { _Phase_Unipolar2_WriteDuty(&p_phase->PWM_C, &p_phase->PWM_B, duty); }
void Phase_Unipolar2_ActivateDutyAB(Phase_T * p_phase, uint16_t duty) { _Phase_Unipolar2_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_B, duty); }

void Phase_Unipolar2_WriteDuty(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
{
    switch (phaseId)
    {
        case PHASE_ID_POLAR_0: break;
        case PHASE_ID_1_AC: _Phase_Unipolar2_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_C, duty); break;
        case PHASE_ID_2_BC: _Phase_Unipolar2_WriteDuty(&p_phase->PWM_B, &p_phase->PWM_C, duty); break;
        case PHASE_ID_3_BA: _Phase_Unipolar2_WriteDuty(&p_phase->PWM_B, &p_phase->PWM_A, duty); break;
        case PHASE_ID_4_CA: _Phase_Unipolar2_WriteDuty(&p_phase->PWM_C, &p_phase->PWM_A, duty); break;
        case PHASE_ID_5_CB: _Phase_Unipolar2_WriteDuty(&p_phase->PWM_C, &p_phase->PWM_B, duty); break;
        case PHASE_ID_6_AB: _Phase_Unipolar2_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_B, duty); break;
        case PHASE_ID_POLAR_7: break;
        default: break;
    }
}

/******************************************************************************/
/******************************************************************************/
static inline void _Phase_Bipolar_InvertPolarity(const Phase_T * p_phase, Phase_Polar_T id)
{
    switch (id)
    {
        case PHASE_ID_POLAR_0: break;
        case PHASE_ID_1_AC: _Phase_WriteInvertPol(&p_phase->PWM_C, &p_phase->PWM_A, &p_phase->PWM_B);  _Phase_SyncPwmInvert(p_phase, PHASE_ID_C); break;
        case PHASE_ID_2_BC: _Phase_WriteInvertPol(&p_phase->PWM_C, &p_phase->PWM_A, &p_phase->PWM_B);  _Phase_SyncPwmInvert(p_phase, PHASE_ID_C); break;
        case PHASE_ID_3_BA: _Phase_WriteInvertPol(&p_phase->PWM_A, &p_phase->PWM_B, &p_phase->PWM_C);  _Phase_SyncPwmInvert(p_phase, PHASE_ID_A); break;
        case PHASE_ID_4_CA: _Phase_WriteInvertPol(&p_phase->PWM_A, &p_phase->PWM_B, &p_phase->PWM_C);  _Phase_SyncPwmInvert(p_phase, PHASE_ID_A); break;
        case PHASE_ID_5_CB: _Phase_WriteInvertPol(&p_phase->PWM_B, &p_phase->PWM_A, &p_phase->PWM_C);  _Phase_SyncPwmInvert(p_phase, PHASE_ID_B); break;
        case PHASE_ID_6_AB: _Phase_WriteInvertPol(&p_phase->PWM_B, &p_phase->PWM_A, &p_phase->PWM_C);  _Phase_SyncPwmInvert(p_phase, PHASE_ID_B); break;
        case PHASE_ID_POLAR_7: break;
        default: break;
    }
}

void Phase_Bipolar_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId) { Phase_Deactivate(p_phase); _Phase_Bipolar_InvertPolarity(p_phase, phaseId);  _Phase_WriteState(p_phase, phaseId); }

// void Phase_Bipolar_ActivateOutputAC(Phase_T * p_phase) { { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
// void Phase_Bipolar_ActivateOutputBC(Phase_T * p_phase) { { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
// void Phase_Bipolar_ActivateOutputBA(Phase_T * p_phase) { { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotC(p_phase); }
// void Phase_Bipolar_ActivateOutputCA(Phase_T * p_phase) { { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
// void Phase_Bipolar_ActivateOutputCB(Phase_T * p_phase) { { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
// void Phase_Bipolar_ActivateOutputAB(Phase_T * p_phase) { { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotC(p_phase); }

/*
    PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
    PwmNegative = PwmPeriodTotal/2 + PwmScalar/2, Inverse polarity
*/
static inline void _Phase_Bipolar_WriteDuty(const PWM_T * p_vHigh, const PWM_T * p_vLow, uint16_t duty) { PWM_WriteDutyMidPlus(p_vHigh, duty); PWM_WriteDutyMidPlus(p_vLow, duty); }

void Phase_Bipolar_ActivateDutyAC(Phase_T * p_phase, uint16_t duty) { _Phase_Bipolar_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_C, duty); }
void Phase_Bipolar_ActivateDutyBC(Phase_T * p_phase, uint16_t duty) { _Phase_Bipolar_WriteDuty(&p_phase->PWM_B, &p_phase->PWM_C, duty); }
void Phase_Bipolar_ActivateDutyBA(Phase_T * p_phase, uint16_t duty) { _Phase_Bipolar_WriteDuty(&p_phase->PWM_B, &p_phase->PWM_A, duty); }
void Phase_Bipolar_ActivateDutyCA(Phase_T * p_phase, uint16_t duty) { _Phase_Bipolar_WriteDuty(&p_phase->PWM_C, &p_phase->PWM_A, duty); }
void Phase_Bipolar_ActivateDutyCB(Phase_T * p_phase, uint16_t duty) { _Phase_Bipolar_WriteDuty(&p_phase->PWM_C, &p_phase->PWM_B, duty); }
void Phase_Bipolar_ActivateDutyAB(Phase_T * p_phase, uint16_t duty) { _Phase_Bipolar_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_B, duty); }

void Phase_Bipolar_WriteDuty(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
{
    switch (phaseId)
    {
        case PHASE_ID_POLAR_0: break;
        case PHASE_ID_1_AC: _Phase_Bipolar_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_C, duty); break;
        case PHASE_ID_2_BC: _Phase_Bipolar_WriteDuty(&p_phase->PWM_B, &p_phase->PWM_C, duty); break;
        case PHASE_ID_3_BA: _Phase_Bipolar_WriteDuty(&p_phase->PWM_B, &p_phase->PWM_A, duty); break;
        case PHASE_ID_4_CA: _Phase_Bipolar_WriteDuty(&p_phase->PWM_C, &p_phase->PWM_A, duty); break;
        case PHASE_ID_5_CB: _Phase_Bipolar_WriteDuty(&p_phase->PWM_C, &p_phase->PWM_B, duty); break;
        case PHASE_ID_6_AB: _Phase_Bipolar_WriteDuty(&p_phase->PWM_A, &p_phase->PWM_B, duty); break;
        case PHASE_ID_POLAR_7: break;
        default: break;
    }
}


/******************************************************************************/
/*!

*/
/******************************************************************************/
// void Phase_Polar_ActivateDuty(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
// {
//     switch(p_phase->PolarMode)
//     {
//         case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_WriteDuty(p_phase, phaseId, duty); break;
//         case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_WriteDuty(p_phase, phaseId, duty); break;
//         case PHASE_MODE_BIPOLAR:    Phase_Bipolar_WriteDuty(p_phase, phaseId, duty);   break;
//         default: break;
//     }
//     _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
// }

// void Phase_Polar_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId)
// {
//     switch (p_phase->PolarMode)
//     {
//         case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar_ActivateOutput(p_phase, phaseId);    break;
//         case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar_ActivateOutput(p_phase, phaseId);    break;
//         case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateOutput(p_phase, phaseId);    break;
//         default: break;
//     }
// }

/*
    Activate Switches Only
*/
// void Phase_Polar_ActivateOutputAC(Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
// void Phase_Polar_ActivateOutputBC(Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
// void Phase_Polar_ActivateOutputBA(Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotC(p_phase); }
// void Phase_Polar_ActivateOutputCA(Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
// void Phase_Polar_ActivateOutputCB(Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
// void Phase_Polar_ActivateOutputAB(Phase_T * p_phase) { if (p_phase->PolarMode == PHASE_MODE_BIPOLAR) { Phase_Deactivate(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotC(p_phase); }

// void Phase_Polar_Activate(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
// {
// }

/******************************************************************************/
/*! @} */
/******************************************************************************/