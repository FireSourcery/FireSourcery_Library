/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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

#include "HAL.h"

#include <stdint.h>
#include <stdbool.h>

void Phase_Init
(
	Phase_T * p_phase,
	HAL_PWM_T * p_pwmA,
	HAL_PWM_T * p_pwmB,
	HAL_PWM_T * p_pwmC,
	uint16_t pwmPeroidMax,
	void (*onPhaseAB)(void * onPhaseData),
	void (*onPhaseAC)(void * onPhaseData),
	void (*onPhaseBC)(void * onPhaseData),
	void (*onPhaseBA)(void * onPhaseData),
	void (*onPhaseCA)(void * onPhaseData),
	void (*onPhaseCB)(void * onPhaseData)
)
{
	p_phase->p_PwmA = p_pwmA;
	p_phase->p_PwmB = p_pwmB;
	p_phase->p_PwmC = p_pwmC;

	p_phase->PwmTotalPeriod = pwmPeroidMax;

	p_phase->PhaseMode = PHASE_MODE_UNIPOLAR_1;

	p_phase->OnPhaseAB = onPhaseAB;
	p_phase->OnPhaseAC = onPhaseAC;
	p_phase->OnPhaseBC = onPhaseBC;
	p_phase->OnPhaseBA = onPhaseBA;
	p_phase->OnPhaseCA = onPhaseCA;
	p_phase->OnPhaseCB = onPhaseCB;
}

static inline void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Mode_T phaseMode)
{
	p_phase->PhaseMode = phaseMode; /* switch style*/

	Phase_ActuateInvertPolarity(p_phase, false, false, false);

	switch (phaseMode)
	{
	case PHASE_MODE_UNIPOLAR_1:
//	p_phase->array[1] = Phase_Unipolar_ActivateAC(); /* fp style*/
//	p_phase->array[2] = 0;
	break;

	case PHASE_MODE_UNIPOLAR_2:
		break;
	case PHASE_MODE_BIPOLAR:
		break;
	}

}

void Phase_EnableSinusoidalModulation(Phase_T * p_phase)
{
//	p_phase->SinusoidalModulation = true;
	Phase_SetState(p_phase, true, true, true);
	Phase_ActuatePeriod(p_phase, 0U, 0U, 0U);
	Phase_ActuateInvertPolarity(p_phase, false, false, false);
}

void Phase_DisableSinusoidalModulation(Phase_T * p_phase)
{
//	p_phase->SinusoidalModulation = false;
	Phase_SetState(p_phase, true, true, true);
	Phase_ActuatePeriod(p_phase, 0U, 0U, 0U);
	Phase_ActuateInvertPolarity(p_phase, false, false, false);
}











