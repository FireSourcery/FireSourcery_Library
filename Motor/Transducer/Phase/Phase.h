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
    @file 	Phase.h
    @author FireSoucery
    @brief  3-Phase PWM functions

  	Assumes each phase is a complementary PWM output.
	2-Phase settings: PWM positive side MOSFETs, ground side bottom MOSFET stays on.
	e.g. PhaseAB -> PWM phase A MOSFETs, phase B bottom MOSFET stays on.
	Implementation details; dead time, are delegated to user/HAL functions

	Only necessary for 2-Phase actuation. However all motor actuating will use this module for uniform interface.

    @version V0
*/
/**************************************************************************/
#ifndef PHASE_H
#define PHASE_H

#include "HAL.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

#define PHASE_DUTY_CYCLE_MAX (65536U)

typedef enum
{
	PHASE_MODE_UNIPOLAR_1,	/*!< PHASE_MODE_UNIPOLAR_1 */
	PHASE_MODE_UNIPOLAR_2,	/*!< PHASE_MODE_UNIPOLAR_2 */
	PHASE_MODE_BIPOLAR   	/*!< PHASE_MODE_BIPOLAR */
} Phase_Mode_T;

typedef enum
{
	PHASE_ID_0 = 0,
	PHASE_ID_1_AC = 1, //Phase AC
	PHASE_ID_2_BC = 2,
	PHASE_ID_3_BA = 3,
	PHASE_ID_4_CA = 4,
	PHASE_ID_5_CB = 5,
	PHASE_ID_6_AB = 6,
	PHASE_ID_7 = 7,
} Phase_Id_T;

typedef struct
{
#if  	defined(CONFIG_PHASE_HAL_PWM)
	const HAL_PWM_T * p_HAL_PwmA;
	const HAL_PWM_T * p_HAL_PwmB;
	const HAL_PWM_T * p_HAL_PwmC;
	#ifdef CONFIG_PHASE_HAL_EXTERNAL_SWITCH
		const HAL_Pin_T * p_HAL_PinSwitchA;
		const HAL_Pin_T * p_HAL_PinSwitchB;
		const HAL_Pin_T * p_HAL_PinSwitchC;
	#endif
#elif 	defined(CONFIG_PHASE_HAL_PHASE)
	const HAL_Phase_T * p_HAL_Phase;
#endif

	uint32_t PwmPeriod_Ticks;

	volatile Phase_Mode_T PhaseMode; //const PhaseMode[8];

	/*
	 * Buffered data
	 */
	volatile uint16_t DutyA_Ticks; /* Phase PWM peroid in ticks */
	volatile uint16_t DutyB_Ticks;
	volatile uint16_t DutyC_Ticks;

//	volatile uint16_t DutyA;
//	volatile uint16_t DutyB;
//	volatile uint16_t DutyC;

	volatile bool StateA; /* On/Off State */
	volatile bool StateB;
	volatile bool StateC;

	volatile uint32_t Duty_Ticks; /* 2-phase polar pwm mode scalar */

//	bool UseSinusoidalInterpolation;
//	uint32_t AngularSpeedTime; // (384*HallBaseTimerFreq/ISRFreq)
//	volatile  uint16_t Angle;
//	volatile  uint16_t AngleOffset;
//	volatile  uint32_t * HallTimerDelta;
//	volatile  uint32_t ISRCount;

//	void * p_OnPhaseData;
//	void (*OnPhaseAB)(void * onPhaseData); /*< User provides function e.g. measure current, start ADC*/
//	void (*OnPhaseAC)(void * onPhaseData);
//	void (*OnPhaseBC)(void * onPhaseData);
//	void (*OnPhaseBA)(void * onPhaseData);
//	void (*OnPhaseCA)(void * onPhaseData);
//	void (*OnPhaseCB)(void * onPhaseData);
} Phase_T;



/*
	Actuate arguments immediately
 */
/*
	Duty in ticks
 */
static inline void Phase_ActuateDutyCycle_Ticks(const Phase_T * p_phase, uint32_t pwmDutyA, uint32_t pwmDutyB, uint32_t pwmDutyC)
{
#if  	defined(CONFIG_PHASE_HAL_PWM)
	HAL_PWM_WriteDuty(p_phase->p_HAL_PwmA, p_phase->DutyA);
	HAL_PWM_WriteDuty(p_phase->p_HAL_PwmB, p_phase->DutyB);
	HAL_PWM_WriteDuty(p_phase->p_HAL_PwmC, p_phase->DutyC);
#elif 	defined(CONFIG_PHASE_HAL_PHASE)
	HAL_Phase_WriteDuty(p_phase->p_HAL_Phase, pwmDutyA, pwmDutyB, pwmDutyC);
#endif
}

/*
	where 65536 is 100% duty
 */
static inline void Phase_ActuateDutyCycle(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	Phase_ActuateDutyCycle_Ticks
	(
		p_phase,
		(uint32_t)pwmDutyA * (uint32_t)p_phase->PwmPeriod_Ticks / 65536U,
		(uint32_t)pwmDutyB * (uint32_t)p_phase->PwmPeriod_Ticks / 65536U,
		(uint32_t)pwmDutyC * (uint32_t)p_phase->PwmPeriod_Ticks / 65536U
	);
}

static inline void Phase_ActuateState(const Phase_T * p_phase, bool a, bool b, bool c)
{
#if  	defined(CONFIG_PHASE_HAL_PWM)
	HAL_PWM_WriteState(p_phase->p_HAL_PwmA, a);
	HAL_PWM_WriteState(p_phase->p_HAL_PwmB, b);
	HAL_PWM_WriteState(p_phase->p_HAL_PwmC, c);
	#ifdef CONFIG_PHASE_HAL_EXTERNAL_SWITCH
		HAL_Pin_WriteState(p_phase->p_HAL_PinSwitchA, a);
		HAL_Pin_WriteState(p_phase->p_HAL_PinSwitchB, b);
		HAL_Pin_WriteState(p_phase->p_HAL_PinSwitchC, c);
	#endif
#elif 	defined(CONFIG_PHASE_HAL_PHASE)
	HAL_Phase_WriteState(p_phase->p_HAL_Phase, a, b, c);
#endif
}

static inline void Phase_ActuateInvertPolarity(const Phase_T * p_phase, bool isInvA, bool isInvB, bool isInvC)
{
#if  	defined(CONFIG_PHASE_HAL_PWM)
	HAL_PWM_WriteInvertPolarity(p_phase->p_HAL_PwmA, isInvA);
	HAL_PWM_WriteInvertPolarity(p_phase->p_HAL_PwmB, isInvB);
	HAL_PWM_WriteInvertPolarity(p_phase->p_HAL_PwmC, isInvC);
#elif 	defined(CONFIG_PHASE_HAL_PHASE)
	HAL_Phase_WriteInvertPolarity(p_phase->p_HAL_Phase, isInvA, isInvB, isInvC);
#endif
}

/*
	Actuate buffered data
 */
static inline void Phase_Actuate(const Phase_T * p_phase)
{
#if  	defined(CONFIG_PHASE_HAL_PWM)
	HAL_PWM_WriteDuty(p_phase->p_HAL_PwmA, p_phase->DutyA);
	HAL_PWM_WriteDuty(p_phase->p_HAL_PwmB, p_phase->DutyB);
	HAL_PWM_WriteDuty(p_phase->p_HAL_PwmC, p_phase->DutyC);
	HAL_PWM_WriteState(p_phase->p_HAL_PwmA, a);
	HAL_PWM_WriteState(p_phase->p_HAL_PwmB, b);
	HAL_PWM_WriteState(p_phase->p_HAL_PwmC, c);
	#ifdef CONFIG_PHASE_HAL_EXTERNAL_SWITCH
		HAL_Pin_WriteState(p_phase->p_HAL_PinSwitchA, p_phase->StateA);
		HAL_Pin_WriteState(p_phase->p_HAL_PinSwitchB, p_phase->StateB);
		HAL_Pin_WriteState(p_phase->p_HAL_PinSwitchC, p_phase->StateC);
	#endif
#elif 	defined(CONFIG_PHASE_HAL_PHASE)
	HAL_Phase_WriteDuty(p_phase->p_HAL_Phase, p_phase->DutyA_Ticks, p_phase->DutyB_Ticks, p_phase->DutyC_Ticks);
	HAL_Phase_WriteState(p_phase->p_HAL_Phase, p_phase->StateA, p_phase->StateB, p_phase->StateC);
#endif
}

/*
	Set buffered data
	Duty cycle in 16 bits. e.g. 65536 == 100%
 */
static inline void Phase_SetDutyCyle(Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	p_phase->DutyA_Ticks = (uint32_t) pwmDutyA * (uint32_t) p_phase->PwmPeriod_Ticks / 65536U;
	p_phase->DutyB_Ticks = (uint32_t) pwmDutyB * (uint32_t) p_phase->PwmPeriod_Ticks / 65536U;
	p_phase->DutyC_Ticks = (uint32_t) pwmDutyC * (uint32_t) p_phase->PwmPeriod_Ticks / 65536U;
}

static inline void Phase_SetDutyCyle_15(Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	p_phase->DutyA_Ticks = (uint32_t) pwmDutyA * (uint32_t) p_phase->PwmPeriod_Ticks / 32768U;
	p_phase->DutyB_Ticks = (uint32_t) pwmDutyB * (uint32_t) p_phase->PwmPeriod_Ticks / 32768U;
	p_phase->DutyC_Ticks = (uint32_t) pwmDutyC * (uint32_t) p_phase->PwmPeriod_Ticks / 32768U;
}

static inline void Phase_SetState(Phase_T * p_phase, bool a, bool b, bool c)
{
	p_phase->StateA = a;
	p_phase->StateB = b;
	p_phase->StateC = c;
}

static inline void Phase_Float(const Phase_T *p_phase)
{
//	Phase_ActuateDutyCycle_Ticks(p_phase, 0U, 0U, 0U);
	Phase_ActuateState(p_phase, false, false, false);
}

static inline void Phase_Short(const Phase_T *p_phase)
{
	Phase_ActuateDutyCycle_Ticks(p_phase, 0U, 0U, 0U);
	Phase_ActuateState(p_phase, true, true, true);
}

/******************************************************************************/
/*!
	2-Phase Polar PWM
 */
/*! @{ */
/******************************************************************************/
static inline void Phase_Unipolar1_ActivateAC(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, p_phase->Duty_Ticks, 0U, 0U);}
static inline void Phase_Unipolar1_ActivateBC(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0U, p_phase->Duty_Ticks, 0U);}
static inline void Phase_Unipolar1_ActivateBA(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0U, p_phase->Duty_Ticks, 0U);}
static inline void Phase_Unipolar1_ActivateCA(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0U, 0U, p_phase->Duty_Ticks);}
static inline void Phase_Unipolar1_ActivateCB(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0U, 0U, p_phase->Duty_Ticks);}
static inline void Phase_Unipolar1_ActivateAB(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, p_phase->Duty_Ticks, 0U, 0U);}

/*
	PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
	PwmNegative = PwmPeriodTotal/2 - PwmScalar/2
 */
static inline void Phase_Unipolar2_ActivateAC(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, 0U, (p_phase->PwmPeriod_Ticks - p_phase->Duty_Ticks) / 2U);}
static inline void Phase_Unipolar2_ActivateBC(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, (p_phase->PwmPeriod_Ticks - p_phase->Duty_Ticks) / 2U);}
static inline void Phase_Unipolar2_ActivateBA(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, (p_phase->PwmPeriod_Ticks - p_phase->Duty_Ticks) / 2U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, 0U);}
static inline void Phase_Unipolar2_ActivateCA(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, (p_phase->PwmPeriod_Ticks - p_phase->Duty_Ticks) / 2U, 0U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U);}
static inline void Phase_Unipolar2_ActivateCB(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0U, (p_phase->PwmPeriod_Ticks - p_phase->Duty_Ticks) / 2U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U);}
static inline void Phase_Unipolar2_ActivateAB(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, (p_phase->PwmPeriod_Ticks - p_phase->Duty_Ticks) / 2U, 0U);}

/*
	PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
	PwmNegative = PwmPeriodTotal/2 + PwmScalar/2, Inverse polarity
 */
static inline void Phase_Bipolar_ActivateAC(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, 0U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U);}
static inline void Phase_Bipolar_ActivateBC(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U);}
static inline void Phase_Bipolar_ActivateBA(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, 0U);}
static inline void Phase_Bipolar_ActivateCA(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, 0U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U);}
static inline void Phase_Bipolar_ActivateCB(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U);}
static inline void Phase_Bipolar_ActivateAB(const Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, (p_phase->PwmPeriod_Ticks + p_phase->Duty_Ticks) / 2U, 0U);}


static inline void Phase_Polar_ActivateAC(const Phase_T * p_phase)
{
	//	if (waveform->SinusoidalModulation)
	//	{
	//		waveform->Angle = REFERENCE_ANGLE_PHASE_AC;
	//		waveform->ISRCount = 0;
	//		//if (PhaseMode == PHASE_MODE_BIPOLAR) EnablePWM(1, 0, 1);
	//	}
	switch (p_phase->PhaseMode)
	{
	case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateAC(p_phase);	break;
	case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateAC(p_phase);	break;
	case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateAC(p_phase);	Phase_ActuateInvertPolarity(p_phase, false, false, true); break;
	default: break;
	}

	Phase_ActuateState(p_phase, true, false, true);
//	waveform->OnPhaseAC();
}

static inline void Phase_Polar_ActivateBC(const Phase_T * p_phase)
{
	switch (p_phase->PhaseMode)
	{
	case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateBC(p_phase);	break;
	case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateBC(p_phase);	break;
	case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateBC(p_phase);	Phase_ActuateInvertPolarity(p_phase, false, false, true); break;
	default: break;
	}
	Phase_ActuateState(p_phase, false, true, true);
}

static inline void Phase_Polar_ActivateBA(const Phase_T * p_phase)
{
	switch (p_phase->PhaseMode)
	{
	case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateBA(p_phase);	break;
	case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateBA(p_phase);	break;
	case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateBA(p_phase);	Phase_ActuateInvertPolarity(p_phase, true, false, false); break;
	default: break;
	}
	Phase_ActuateState(p_phase, true, true, false);
}

static inline void Phase_Polar_ActivateCA(const Phase_T * p_phase)
{
	switch (p_phase->PhaseMode)
	{
	case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateCA(p_phase);	break;
	case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateCA(p_phase);	break;
	case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateCA(p_phase);	Phase_ActuateInvertPolarity(p_phase, true, false, false); break;
	default: break;
	}
	Phase_ActuateState(p_phase, true, false, true);
}

static inline void Phase_Polar_ActivateCB(const Phase_T * p_phase)
{
	switch (p_phase->PhaseMode)
	{
	case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateCB(p_phase);	break;
	case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateCB(p_phase);	break;
	case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateCB(p_phase);	Phase_ActuateInvertPolarity(p_phase, false, true, false); break;
	default: break;
	}
	Phase_ActuateState(p_phase, false, true, true);
}

static inline void Phase_Polar_ActivateAB(const Phase_T * p_phase)
{
//	if (waveform->SinusoidalModulation)
//	{
//		waveform->Angle = REFERENCE_ANGLE_PHASE_AB;
//		waveform->ISRCount = 0;
//	}
	switch (p_phase->PhaseMode)
	{
	case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateAB(p_phase);	break;
	case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateAB(p_phase);	break;
	case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateAB(p_phase);	Phase_ActuateInvertPolarity(p_phase, false, true, false); break;
	default: break;
	}

	Phase_ActuateState(p_phase, true, true, false);
//	p_phase->OnPhaseAB(p_phase->p_OnPhaseData);
}

static inline void Phase_Polar_SetDutyCyle(Phase_T * p_phase, uint16_t pwmDuty)
{
	p_phase->Duty_Ticks = (uint32_t)pwmDuty * (uint32_t)p_phase->PwmPeriod_Ticks / 65536U;
}

static inline void Phase_Polar_SetDutyCyle_Ticks(Phase_T * p_phase, uint32_t pwmTicks)
{
	p_phase->Duty_Ticks = pwmTicks;
}

//static inline void Phase_Polar_SetDutyCyle_NBits(Phase_T * p_phase, uint16_t pwmDuty)
//{
////	p_phase->Duty_Ticks = pwmDuty * p_phase->PwmPeriod_Ticks >> p_phase->NBits;
//}


/******************************************************************************/
/*! @} */
/******************************************************************************/
static inline void Phase_Polar_ActivatePhase(Phase_T * p_phase, Phase_Id_T phaseId)
{
//	CommutationTable[phaseID][p_phase->PhaseMode]();
	switch (phaseId)
	{
	case PHASE_ID_0: break;
	case PHASE_ID_1_AC:	Phase_Polar_ActivateAC(p_phase); break; //p_phase->OnPhaseAC;
	case PHASE_ID_2_BC: Phase_Polar_ActivateBC(p_phase); break;
	case PHASE_ID_3_BA: Phase_Polar_ActivateBA(p_phase); break;
	case PHASE_ID_4_CA: Phase_Polar_ActivateCA(p_phase); break;
	case PHASE_ID_5_CB: Phase_Polar_ActivateCB(p_phase); break;
	case PHASE_ID_6_AB: Phase_Polar_ActivateAB(p_phase); break;
	case PHASE_ID_7: break;
	default: break;
	}

	//	if (waveform->SinusoidalModulation)
	//	{
	//
	//		waveform->Index = 0;
	//	switch (phaseId)
	//	{
	//	case PHASE_ID_0: break;
	//	case PHASE_ID_1_AC:	waveform->Angle = REFERENCE_ANGLE_PHASE_AC; break;
	//	case PHASE_ID_2_BC: waveform->Angle = REFERENCE_ANGLE_PHASE_AC; break;
	//	case PHASE_ID_3_BA: waveform->Angle = REFERENCE_ANGLE_PHASE_AC; break;
	//	case PHASE_ID_4_CA: waveform->Angle = REFERENCE_ANGLE_PHASE_AC; break;
	//	case PHASE_ID_5_CB: waveform->Angle = REFERENCE_ANGLE_PHASE_AC; break;
	//	case PHASE_ID_6_AB: waveform->Angle = REFERENCE_ANGLE_PHASE_AC; break;
	//	case PHASE_ID_7: break;
	//	default: break;
	//	}
	//	}
}

static inline void Phase_Polar_ActivateAngle(Phase_T * p_phase, uint16_t theta)
{

}


static inline void Phase_ActivateA(Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, p_phase->Duty_Ticks, 0, 0);}
static inline void Phase_ActivateB(Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0, p_phase->Duty_Ticks, 0);}
static inline void Phase_ActivateC(Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0, 0, p_phase->Duty_Ticks);}
static inline void Phase_ActivateInvA(Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0, p_phase->Duty_Ticks, p_phase->Duty_Ticks);}
static inline void Phase_ActivateInvB(Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, p_phase->Duty_Ticks, 0, p_phase->Duty_Ticks);}
static inline void Phase_ActivateInvC(Phase_T * p_phase){Phase_ActuateDutyCycle_Ticks(p_phase, 0, p_phase->Duty_Ticks, p_phase->Duty_Ticks);}

extern void Phase_Init
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
);
extern void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Mode_T phaseMode);

#endif
