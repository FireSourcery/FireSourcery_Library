/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@file 	Phase.c
	@author FireSourcery
	@brief 	Phase module conventional function definitions
	@version V0
*/
/******************************************************************************/
#include "Phase.h"

void Phase_Init(Phase_T * p_phase)
{
	PWM_InitModule(&p_phase->PwmA);
	PWM_InitChannel(&p_phase->PwmA);
	PWM_InitChannel(&p_phase->PwmB);
	PWM_InitChannel(&p_phase->PwmC);
	p_phase->PhaseMode = PHASE_MODE_UNIPOLAR_1;
}

void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Mode_T phaseMode)
{
	if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR)
	{
		Phase_Float(p_phase);
		PWM_DisableInvertPolarity(&p_phase->PwmA);
		PWM_DisableInvertPolarity(&p_phase->PwmB);
		PWM_DisableInvertPolarity(&p_phase->PwmC);
	}
	p_phase->PhaseMode = phaseMode;
}

/*
	Commutation Phase Id handled by Phase module
	switch should be faster than CommutationTable[phaseID][p_phase->PhaseMode]();
*/
void Phase_Polar_Activate(Phase_T * p_phase, Phase_Id_T phaseId, uint16_t duty)
{
	switch(phaseId)
	{
		case PHASE_ID_0: break;
		case PHASE_ID_1_AC:	Phase_Polar_ActivateAC(p_phase, duty); break;
		case PHASE_ID_2_BC: Phase_Polar_ActivateBC(p_phase, duty); break;
		case PHASE_ID_3_BA: Phase_Polar_ActivateBA(p_phase, duty); break;
		case PHASE_ID_4_CA: Phase_Polar_ActivateCA(p_phase, duty); break;
		case PHASE_ID_5_CB: Phase_Polar_ActivateCB(p_phase, duty); break;
		case PHASE_ID_6_AB: Phase_Polar_ActivateAB(p_phase, duty); break;
		case PHASE_ID_7: break;
		default: break;
	}
}

void Phase_Polar_ActivateDuty(Phase_T * p_phase, Phase_Id_T phaseId, uint16_t duty)
{
	switch(phaseId)
	{
		case PHASE_ID_0: break;
		case PHASE_ID_1_AC:	Phase_Polar_ActivateDutyAC(p_phase, duty); break;
		case PHASE_ID_2_BC: Phase_Polar_ActivateDutyBC(p_phase, duty); break;
		case PHASE_ID_3_BA: Phase_Polar_ActivateDutyBA(p_phase, duty); break;
		case PHASE_ID_4_CA: Phase_Polar_ActivateDutyCA(p_phase, duty); break;
		case PHASE_ID_5_CB: Phase_Polar_ActivateDutyCB(p_phase, duty); break;
		case PHASE_ID_6_AB: Phase_Polar_ActivateDutyAB(p_phase, duty); break;
		case PHASE_ID_7: break;
		default: break;
	}
}

void Phase_Polar_ActivateSwitch(Phase_T * p_phase, Phase_Id_T phaseId)
{
	switch(phaseId)
	{
		case PHASE_ID_0: break;
		case PHASE_ID_1_AC:	Phase_Polar_ActivateSwitchAC(p_phase); break;
		case PHASE_ID_2_BC: Phase_Polar_ActivateSwitchBC(p_phase); break;
		case PHASE_ID_3_BA: Phase_Polar_ActivateSwitchBA(p_phase); break;
		case PHASE_ID_4_CA: Phase_Polar_ActivateSwitchCA(p_phase); break;
		case PHASE_ID_5_CB: Phase_Polar_ActivateSwitchCB(p_phase); break;
		case PHASE_ID_6_AB: Phase_Polar_ActivateSwitchAB(p_phase); break;
		case PHASE_ID_7: break;
		default: break;
	}
}
