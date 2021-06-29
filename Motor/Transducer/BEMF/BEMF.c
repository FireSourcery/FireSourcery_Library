/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	BEMF.h
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#include "BEMF.h"

#include <stdint.h>
#include <stdbool.h>

void BEMF_Init
(
	BEMF_T * p_bemf,
	const uint32_t * p_timer,
	volatile const bemf_t * p_phaseA,
	volatile const bemf_t * p_phaseB,
	volatile const bemf_t * p_phaseC,
	volatile const bemf_t * p_vbus,
	BEMF_SampleMode_T mode
)
{
	p_bemf->p_Timer = p_timer;

	p_bemf->p_VPhaseA_ADCU = p_phaseA;
	p_bemf->p_VPhaseB_ADCU = p_phaseB;
	p_bemf->p_VPhaseC_ADCU = p_phaseC;
	p_bemf->p_VBus_ADCU = p_vbus;

	p_bemf->PhaseAdvanceTime = 0u;
	p_bemf->ZeroCrossingThreshold_ADCU = 0u;

 	p_bemf->SampleMode = mode;

	p_bemf->p_VPhaseObserve_ADCU = p_phaseA;
	p_bemf->PhaseObserveId = BEMF_PHASE_A;
	p_bemf->Mode = BEMF_MODE_PASSIVE;
}



void BEMF_SetSampleMode(BEMF_T * p_bemf, BEMF_SampleMode_T mode)
{
 	p_bemf->SampleMode = mode;

	switch(p_bemf->SampleMode)
	{
//	case BEMF_SAMPLE_MODE_PWM_BIPOLAR:		break;
	case BEMF_SAMPLE_MODE_PWM_ON:  p_bemf->ZeroCrossingThreshold_ADCU = 0; break;
//	case BEMF_SAMPLE_MODE_PWM_OFF:  		break;
	default:	break;
	}
}

void BEMF_SetObserveMode(BEMF_T * p_bemf, BEMF_Mode_T mode)
{
 	p_bemf->Mode = mode;
}

void BEMF_Reset(BEMF_T * p_bemf)
{
//	p_bemf->IsReliable = 0U;
	p_bemf->ZeroCrossingCounter = 0U;
	//map phase if module maps phase
}


//calc speed-> calc advance angle
void BEMF_SetAdvanceAngleTime(BEMF_T * p_bemf, uint16_t angle_frac16)
{
//	p_bemf->PhaseAdvanceTime = t*angle_frac16>>15;
}

//void BEMF_ResetTimer(BEMF_T * p_bemf)
//{
//	p_bemf->TimeCommutationStart = 0U;
//}
