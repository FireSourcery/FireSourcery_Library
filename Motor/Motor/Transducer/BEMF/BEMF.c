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
    @file 	BEMF.h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#include "BEMF.h"

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#include <stdint.h>
#include <stdbool.h>




 volatile uint16_t BemfDebug[300];
 volatile uint16_t BemfDebugIndex = 0;


 /*
 call on adc
 capture bemf with time stamp
  capture bemf sample only at end of adc
 proc zcd next pwm
  */
static inline void CaptureVPhaseA(BEMF_T * p_bemf)
{
//	CaptureEmf(p_bemf,  &p_bemf->AnalogResults[BEMF_ANALOG_CHANNEL_VA]);
}

static inline void CaptureVPhaseB(BEMF_T * p_bemf)
{
//	CaptureEmf(p_bemf,  &p_bemf->AnalogResults[BEMF_ANALOG_CHANNEL_VB]);
}

static inline void CaptureVPhaseC(BEMF_T * p_bemf)
{
//	CaptureEmf(p_bemf,  &p_bemf->AnalogResults[BEMF_ANALOG_CHANNEL_VC]);
}


//static const Analog_ConversionVirtualChannel_T CHANNELS_BEMF_A_REPEAT[] =
//{
//[0U] = {MOTOR_ANALOG_CHANNEL_VA, 	(Analog_OnComplete_T)&Motor_SixStep_CaptureBemfA},
//};
//
//const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_A_REPEAT =
//{
//.P_CHANNELS 	= CHANNELS_BEMF_A_REPEAT,
//.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_A_REPEAT)/sizeof(Analog_ConversionVirtualChannel_T),
//.ON_COMPLETE 	= 0U,
//.OPTIONS =		{.HwTriggerConversion = 0U, .ContinuousConversion = 1U, .CaptureLocalPeak = 1U },
//};

void BEMF_Init(BEMF_T * p_bemf)
{
	p_bemf->PhaseAdvanceTime = 0u;
	p_bemf->ZeroCrossingThreshold_ADCU = 0u;
	p_bemf->SampleMode = BEMF_SAMPLE_MODE_PWM_ON;
//	p_bemf->p_VPhaseObserve_ADCU = p_phaseA;
//	p_bemf->PhaseObserveId = BEMF_PHASE_A;
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
//	case BEMF_SAMPLE_MODE_PWM_ON_OFF:  		break;
	default:	break;
	}
}

void BEMF_SetObserveMode(BEMF_T * p_bemf, BEMF_Mode_T mode)
{
 	p_bemf->Mode = mode;
	p_bemf->ZeroCrossingCounter = 0U; //reset consecutive zcd counter
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

//void BEMF_Reset(BEMF_T * p_bemf)
//{
////	p_bemf->IsReliable = 0U;
//	p_bemf->ZeroCrossingCounter = 0U;
//	//map phase if module maps phase
//}
