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

#include "HAL_Phase.h"

#include <stdint.h>
#include <stdbool.h>

void Phase_Init
(
	Phase_T * p_phase,
#if  	defined(CONFIG_PHASE_HAL_PWM)
	const HAL_PWM_T * p_HAL_pwmA,
	const HAL_PWM_T * p_HAL_pwmB,
	const HAL_PWM_T * p_HAL_pwmC,
#elif 	defined(CONFIG_PHASE_HAL_PHASE)
	const HAL_Phase_T * p_HAL_Phase,
#endif
	uint16_t pwmPeroid_Ticks
//	void (*onPhaseAB)(void * onPhaseData),
//	void (*onPhaseAC)(void * onPhaseData),
//	void (*onPhaseBC)(void * onPhaseData),
//	void (*onPhaseBA)(void * onPhaseData),
//	void (*onPhaseCA)(void * onPhaseData),
//	void (*onPhaseCB)(void * onPhaseData)
)
{
#if  	defined(CONFIG_PHASE_HAL_PWM)
	p_phase->p_HAL_PwmA = p_HAL_pwmA;
	p_phase->p_HAL_PwmB = p_HAL_pwmB;
	p_phase->p_HAL_PwmC = p_HAL_pwmC;
#elif 	defined(CONFIG_PHASE_HAL_PHASE)
	p_phase->p_HAL_Phase = p_HAL_Phase;
	HAL_Phase_Init(p_HAL_Phase);
#endif

	p_phase->PwmPeriod_Ticks = pwmPeroid_Ticks;

	Phase_ActuateDutyCycle(p_phase, 0U, 0U, 0U);
	Phase_ActuateState(p_phase, false, false, false);
	Phase_ActuateInvertPolarity(p_phase, false, false, false);
	p_phase->PhaseMode = PHASE_MODE_UNIPOLAR_1;
	p_phase->DutyA_Ticks = 0U;
	p_phase->DutyB_Ticks = 0U;
	p_phase->DutyC_Ticks = 0U;
	p_phase->StateA = false;
	p_phase->StateB = false;
	p_phase->StateC = false;
	p_phase->DutyXY_Ticks = 0U;

//	p_phase->OnPhaseAB = onPhaseAB;
//	p_phase->OnPhaseAC = onPhaseAC;
//	p_phase->OnPhaseBC = onPhaseBC;
//	p_phase->OnPhaseBA = onPhaseBA;
//	p_phase->OnPhaseCA = onPhaseCA;
//	p_phase->OnPhaseCB = onPhaseCB;
}

void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Mode_T phaseMode)
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

//void Phase_EnableSinusoidalModulation(Phase_T * p_phase)
//{
////	p_phase->SinusoidalModulation = true;
////	p_phase->PhaseMode = SINUSOIDAL; /* switch style*/
//	Phase_SetState(p_phase, true, true, true);
//	Phase_ActuatePeriod(p_phase, 0U, 0U, 0U);
//	Phase_ActuateInvertPolarity(p_phase, false, false, false);
//}
//
//void Phase_DisableSinusoidalModulation(Phase_T * p_phase)
//{
////	p_phase->SinusoidalModulation = false;
//	Phase_SetState(p_phase, true, true, true);
//	Phase_ActuatePeriod(p_phase, 0U, 0U, 0U);
//	Phase_ActuateInvertPolarity(p_phase, false, false, false);
//}


