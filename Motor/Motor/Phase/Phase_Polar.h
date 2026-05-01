
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
    @file   Phase.h
    @author FireSourcery

    @brief   Part of Phase. 2-Phase Polar implementation
        Treats each phase as a complementary PWM output.
        2-Phase: PWM positive side MOSFETs, ground side bottom MOSFET stays on.
        e.g. PhaseAB -> PWM phase A MOSFETs, phase B bottom MOSFET stays on.
*/
/******************************************************************************/
#ifndef PHASE_POLAR_H
#define PHASE_POLAR_H

#include "Phase.h"

/*
    2-Phase Active, Six-Step Commutation
*/
typedef enum Phase_Polar
{
    PHASE_ID_POLAR_0 = 0U,
    PHASE_ID_1_AC = PHASE_ID_INV_B,
    PHASE_ID_2_BC = PHASE_ID_A,
    PHASE_ID_3_BA = PHASE_ID_INV_C,
    PHASE_ID_4_CA = PHASE_ID_B,
    PHASE_ID_5_CB = PHASE_ID_INV_A,
    PHASE_ID_6_AB = PHASE_ID_C,
    PHASE_ID_POLAR_7 = 7U,
}
Phase_Polar_T;

/* PolarMode */
typedef enum Phase_Polar_Mode
{
    PHASE_MODE_UNIPOLAR_1,    /*!<   */
    PHASE_MODE_UNIPOLAR_2,    /*!<   */
    PHASE_MODE_BIPOLAR,       /*!<   */
}
Phase_Polar_Mode_T;


extern void Phase_Unipolar1_ActivateDutyAC(Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar1_ActivateDutyBC(Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar1_ActivateDutyBA(Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar1_ActivateDutyCA(Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar1_ActivateDutyCB(Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar1_ActivateDutyAB(Phase_T * p_phase, uint16_t duty);

extern void Phase_Unipolar2_ActivateDutyAC(Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar2_ActivateDutyBC(Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar2_ActivateDutyBA(Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar2_ActivateDutyCA(Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar2_ActivateDutyCB(Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar2_ActivateDutyAB(Phase_T * p_phase, uint16_t duty);

extern void Phase_Bipolar_ActivateDutyAC(Phase_T * p_phase, uint16_t duty);
extern void Phase_Bipolar_ActivateDutyBC(Phase_T * p_phase, uint16_t duty);
extern void Phase_Bipolar_ActivateDutyBA(Phase_T * p_phase, uint16_t duty);
extern void Phase_Bipolar_ActivateDutyCA(Phase_T * p_phase, uint16_t duty);
extern void Phase_Bipolar_ActivateDutyCB(Phase_T * p_phase, uint16_t duty);
extern void Phase_Bipolar_ActivateDutyAB(Phase_T * p_phase, uint16_t duty);

extern void Phase_Polar_Ground(Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputAC(Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputBC(Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputBA(Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputCA(Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputCB(Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputAB(Phase_T * p_phase);

extern void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Polar_Mode_T phaseMode);
extern void Phase_Polar_Activate(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty);
extern void Phase_Polar_ActivateDuty(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty);
extern void Phase_Polar_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId);

#endif