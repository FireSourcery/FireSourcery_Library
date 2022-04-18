/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
	@author FireSoucery
	@brief 	Phase module conventional function definitions
	@version V0
*/
/******************************************************************************/
#include "Phase.h"

#include "Peripheral/PWM/PWM.h"

#include <stdint.h>
#include <stdbool.h>

void Phase_Init(Phase_T * p_phase)
{
	PWM_InitModule(&p_phase->PwmA);
	PWM_InitChannel(&p_phase->PwmA);
	PWM_InitChannel(&p_phase->PwmB);
	PWM_InitChannel(&p_phase->PwmC);
}

void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Mode_T phaseMode)
{
	p_phase->PhaseMode = phaseMode; /* if use swtich */

	PWM_DisableInvertPolarity(&p_phase->PwmA);
	PWM_DisableInvertPolarity(&p_phase->PwmB);
	PWM_DisableInvertPolarity(&p_phase->PwmC);

//	switch (phaseMode)
//	{
//	case PHASE_MODE_UNIPOLAR_1:
//	//	p_phase->ActivateAC = Phase_Unipolar_ActivateAC(); /* if use fp */
//	//	p_phase->ActivateAB = 0;
//	break;
//
//	case PHASE_MODE_UNIPOLAR_2:
//		break;
//	case PHASE_MODE_BIPOLAR:
//		break;
//	}

}


