/**************************************************************************/
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

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	PHASE_MODE_UNIPOLAR_1,	/*!< PHASE_MODE_UNIPOLAR_1 */
	PHASE_MODE_UNIPOLAR_2,	/*!< PHASE_MODE_UNIPOLAR_2 */
	PHASE_MODE_BIPOLAR   	/*!< PHASE_MODE_BIPOLAR */
} Phase_Mode_T;

typedef struct
{
//	union
//	{
//		volatile void * p_TimerMap;
//		volatile void (* const (* pp_TimerMaps)[3]); /* if pwm using separate timers */
//	};
//
//	uint32_t Timer_Id_A;
//	uint32_t Timer_Id_B;
//	uint32_t Timer_Id_C;
	HAL_PWM_T * p_PwmA;
	HAL_PWM_T * p_PwmB;
	HAL_PWM_T * p_PwmC;

//	HAL_Pin_T PwmSwitchA;
//	HAL_Pin_T PwmSwitchB;
//	HAL_Pin_T PwmSwitchC;

	uint32_t PwmTotalPeriod; //	uint32_t PwmTotalPeriodHalf;

	volatile Phase_Mode_T PhaseMode; //const PhaseMode[8];

	/* Optionally set buffers */
	volatile uint16_t PeriodA; /* Phase PWM peroid in ticks */
	volatile uint16_t PeriodB;
	volatile uint16_t PeriodC;

	volatile bool StateA; /*  On/Off State */
	volatile bool StateB;
	volatile bool StateC;

	volatile uint32_t PeriodScalar; /* 2-phase polar pwm mode scalar */

//	bool UseSinusoidalInterpolation;
//	uint32_t AngularSpeedTime; // (384*HallBaseTimerFreq/ISRFreq)

//	volatile  uint16_t Angle;
//	volatile  uint16_t AngleOffset;
////	volatile  uint32_t PWM;
//	volatile  uint32_t * HallTimerDelta;
//	volatile  uint32_t ISRCount;

	void * OnPhaseData;

	void (*OnPhaseAB)(void * onPhaseData); /*< User provides function e.g. measure current, start ADC*/
	void (*OnPhaseAC)(void * onPhaseData);
	void (*OnPhaseBC)(void * onPhaseData);
	void (*OnPhaseBA)(void * onPhaseData);
	void (*OnPhaseCA)(void * onPhaseData);
	void (*OnPhaseCB)(void * onPhaseData);
} Phase_T;



/*
	Actuate buffered data
 */
static inline void Phase_Actuate(const Phase_T * p_phase)
{
	HAL_PWM_WritePeriod(p_phase->p_PwmA, p_phase->PeriodA);
	HAL_PWM_WritePeriod(p_phase->p_PwmB, p_phase->PeriodB);
	HAL_PWM_WritePeriod(p_phase->p_PwmC, p_phase->PeriodC);
	HAL_PWM_WriteState(p_phase->p_PwmA, p_phase->StateA);
	HAL_PWM_WriteState(p_phase->p_PwmB, p_phase->StateB);
	HAL_PWM_WriteState(p_phase->p_PwmC, p_phase->StateC);
}

/*
	Actuate arguments immediately
 */
static inline void Phase_ActuatePeriod(const Phase_T * p_phase, uint16_t pwmPeriodA, uint16_t pwmPeriodB, uint16_t pwmPeriodC)
{
	HAL_PWM_WritePeriod(p_phase->p_PwmA, pwmPeriodA);
	HAL_PWM_WritePeriod(p_phase->p_PwmB, pwmPeriodB);
	HAL_PWM_WritePeriod(p_phase->p_PwmC, pwmPeriodC);
}


static inline void Phase_ActuatePeriod_DutyCycle(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	HAL_PWM_WritePeriod(p_phase->p_PwmA, pwmDutyA * p_phase->PwmTotalPeriod / 65536U);
	HAL_PWM_WritePeriod(p_phase->p_PwmB, pwmDutyB * p_phase->PwmTotalPeriod / 65536U);
	HAL_PWM_WritePeriod(p_phase->p_PwmC, pwmDutyC * p_phase->PwmTotalPeriod / 65536U);
}

static inline void Phase_ActuateInvertPolarity(const Phase_T * p_phase, bool isInvA, bool isInvB, bool isInvC)
{
	HAL_PWM_WriteInvertPolarity(p_phase->p_PwmA, isInvA);
	HAL_PWM_WriteInvertPolarity(p_phase->p_PwmB, isInvB);
	HAL_PWM_WriteInvertPolarity(p_phase->p_PwmC, isInvB);
}

static inline void Phase_ActuateState(const Phase_T * p_phase, bool a, bool b, bool c)
{
//	HAL_Pin_WriteState(&p_phase->p_PwmSwitchA, a);
//	HAL_Pin_WriteState(&p_phase->p_PwmSwitchB, b);
//	HAL_Pin_WriteState(&p_phase->p_PwmSwitchC, c);

	HAL_PWM_WriteState(p_phase->p_PwmA, a);
	HAL_PWM_WriteState(p_phase->p_PwmB, b);
	HAL_PWM_WriteState(p_phase->p_PwmC, c);
}

/*
	Duty cycle express pwm period as percentage in 16 bits. e.g. 65536 == 100%
 */
static inline void Phase_SetDutyCyle(Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	p_phase->PeriodA = pwmDutyA * p_phase->PwmTotalPeriod / 65536U;
	p_phase->PeriodB = pwmDutyB * p_phase->PwmTotalPeriod / 65536U;
	p_phase->PeriodC = pwmDutyC * p_phase->PwmTotalPeriod / 65536U;
}

static inline void Phase_SetState(Phase_T * p_phase, bool a, bool b, bool c)
{
	p_phase->StateA = a;
	p_phase->StateB = b;
	p_phase->StateC = c;
}




//	void Phase_CommutatePhaseA(Phase_T * p_phase, uint16_t pwm)
//	{
//		p_phase->SetPWMVal(pwm, 0, 0);
//	}
//
//	void Phase_CommutatePhaseB(Phase_T * p_phase, uint16_t pwm)
//	{
//		p_phase->SetPWMVal(0, pwm, 0);
//	}
//
//	void Phase_CommutatePhaseC(Phase_T * p_phase, uint16_t pwm)
//	{
//		p_phase->SetPWMVal(0, 0, pwm);
//	}
//
//	void Phase_CommutatePhaseInvA(Phase_T * p_phase, uint16_t pwm) //avoid confusion with phaseCB
//	{
//		p_phase->SetPWMVal(0, pwm, pwm);
//	}
//
//	void Phase_CommutatePhaseInvB(Phase_T * p_phase, uint16_t pwm)
//	{
//		p_phase->SetPWMVal(pwm, 0, pwm);
//	}
//
//	void Phase_CommutatePhaseInvC(Phase_T * p_phase, uint16_t pwm)
//	{
//		p_phase->SetPWMVal(0, pwm, pwm);
//	}
//
//	void Phase_CommutatePhaseInvC(Phase_T * p_phase, uint16_t pwm)
//	{
//		p_phase->SetPWMVal(0, pwm, pwm);
//	}

/*
	2-Phase Polar PWM
 */

static inline void Phase_Unipolar1_ActivateAB(const Phase_T * p_phase)
{
	Phase_ActuatePeriod(p_phase, p_phase->PeriodScalar, 0U, 0U);
//	Phase_ActuatePeriod_DutyCycle(p_phase, p_phase->PeriodScalar, 0, 0);
}

//static inline void Phase_Unipolar1_ActivateAB_Duty(Phase_T * p_phase, uint16_t pwmDuty)
//{
//	Phase_ActuatePeriod(p_phase, (pwmDuty * p_phase->PwmTotalPeriod >> 16), 0, 0);
//}

/*
	PwmPeriodA = PwmTotalPeriod/2 + PwmScale/2
 */
static inline void Phase_Unipolar2_ActivateAB(const Phase_T * p_phase)
{
	Phase_ActuatePeriod(p_phase, (p_phase->PwmTotalPeriod + p_phase->PeriodScalar) / 2U, (p_phase->PwmTotalPeriod - p_phase->PeriodScalar) / 2U, 0U);
}

static inline void Phase_Bipolar_ActivateAB(const Phase_T * p_phase)
{
	Phase_ActuatePeriod(p_phase, (p_phase->PwmTotalPeriod + p_phase->PeriodScalar) / 2U, (p_phase->PwmTotalPeriod + p_phase->PeriodScalar) / 2U, 0U);
	Phase_ActuateInvertPolarity(p_phase, false, true, false);
}


static inline void Phase_Polar_SetPeriod(Phase_T * p_phase, uint16_t pwmPeriod)
{
	p_phase->PeriodScalar = pwmPeriod;
}

static inline void Phase_Polar_SetDutyCyle(Phase_T * p_phase, uint16_t pwmDuty)
{
	p_phase->PeriodScalar = pwmDuty * p_phase->PwmTotalPeriod / 65536U;
}

static inline void Phase_Polar_ActivateAB(const Phase_T * p_phase)
{
	switch (p_phase->PhaseMode)
	{
	case PHASE_MODE_UNIPOLAR_1:		Phase_Unipolar1_ActivateAB(p_phase);	break;
	case PHASE_MODE_UNIPOLAR_2:		Phase_Unipolar2_ActivateAB(p_phase);	break;
	case PHASE_MODE_BIPOLAR:		Phase_Bipolar_ActivateAB(p_phase);		break;
	default: break;
	}

	Phase_ActuateState(p_phase, true, true, false);
	p_phase->OnPhaseAB(p_phase->OnPhaseData);
}

static inline void Phase_Polar_ActivateAB_Duty(Phase_T * p_phase, uint16_t pwmDuty)
{
	Phase_Polar_SetDutyCyle(p_phase, pwmDuty);
	Phase_Polar_ActivateAB(p_phase);
}









extern void Phase_Init
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
);

#endif
