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


/*
 * 16-bit range, 512 domain table. 0-340, 341-511 == 0
 */

//const uint16_t MATH_SVPWM_SADDLE_120[170] =
//{
//	4390, 4997, 5607, 6222, 6841, 7464, 8090, 8720, 9354,
//	9992, 10633, 11277, 11925, 12575, 13229, 13886, 14545, 15207,
//	15872, 16540, 17210, 17882, 18557, 19233, 19912, 20593, 21275,
//	21959, 22645, 23333, 24022, 24712, 25403, 26096, 26789, 27484,
//	28179, 28875, 29571, 30268, 30965, 31663, 32361, 33059, 33757,
//	34454, 35152, 35849, 36545, 37241, 37937, 38631, 39325, 40018,
//	40709, 41399, 42089, 42776, 43462, 44147, 44830, 45511, 46190,
//	46867, 47542, 48214, 48885, 49553, 50218, 50881, 51541, 52198,
//	52852, 53503, 54151, 54796, 55438, 56076, 56710, 57341, 57968,
//	58592, 59211, 59827, 60438, 61045, 61312, 61508, 61699, 61886,
//	62069, 62247, 62421, 62590, 62755, 62915, 63071, 63222, 63368,
//	63510, 63647, 63779, 63907, 64030, 64149, 64262, 64371, 64475,
//	64575, 64669, 64759, 64843, 64923, 64999, 65069, 65134, 65195,
//	65250, 65301, 65347, 65387, 65423, 65454, 65480, 65501, 65517,
//	65528, 65535, 65536, 65532, 65523, 65510, 65491, 65468, 65439,
//	65406, 65368, 65324, 65276, 65223, 65165, 65102, 65034, 64962,
//	64884, 64802, 64714, 64622, 64525, 64424, 64317, 64206, 64090,
//	63969, 63844, 63714, 63579, 63439, 63295, 63147, 62993, 62835,
//	62673, 62506, 62335, 62159, 61978, 61793, 61604, 61411, 61213,
//};
