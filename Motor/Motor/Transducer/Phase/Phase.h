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
	@file 	Phase.h
	@author FireSourcery
	@brief  3-Phase PWM functions. Includes 2-Phase Polar implementation

	Treats each phase as a complementary PWM output.
	2-Phase settings: PWM positive side MOSFETs, ground side bottom MOSFET stays on.
	e.g. PhaseAB -> PWM phase A MOSFETs, phase B bottom MOSFET stays on.
	Implementation details; dead time, are delegated to user/HAL functions

	@version V0
*/
/******************************************************************************/
#ifndef PHASE_H
#define PHASE_H

#include "Config.h"
#include "Peripheral/PWM/PWM.h"
#include "Peripheral/Pin/Pin.h"
#include <stdint.h>

typedef enum Phase_Mode_Tag
{
	PHASE_MODE_UNIPOLAR_1,	/*!<   */
	PHASE_MODE_UNIPOLAR_2,	/*!<   */
	PHASE_MODE_BIPOLAR   	/*!<   */
}
Phase_Mode_T;

/* 2-Phase */
typedef enum Phase_Id_Tag
{
	PHASE_ID_0 = 0U,
	PHASE_ID_1_AC = 1U,
	PHASE_ID_2_BC = 2U,
	PHASE_ID_3_BA = 3U,
	PHASE_ID_4_CA = 4U,
	PHASE_ID_5_CB = 5U,
	PHASE_ID_6_AB = 6U,
	PHASE_ID_7 = 7U,
}
Phase_Id_T;

/* 3-Phase */
// typedef enum Phase_Id_Tag
// {
// 	PHASE_ID_0 = 0U,
// 	PHASE_ID_A = 1U,
// 	PHASE_ID_B = 2U,
// 	PHASE_ID_C = 3U,
// 	PHASE_ID_INV_A = 4U,
// 	PHASE_ID_INV_B = 5U,
// 	PHASE_ID_INV_C = 6U,
// 	PHASE_ID_7 = 7U,
// }
// Phase_Id_T;

typedef struct Phase_Tag
{
	PWM_T PwmA;
	PWM_T PwmB;
	PWM_T PwmC;
	//	PWM_Module_T PwmModule;
#ifdef CONFIG_PHASE_PIN_SWITCH
	Pin_T PinA;
	Pin_T PinB;
	Pin_T PinC;
#endif
	Phase_Mode_T PhaseMode;
}
Phase_T;

//p_PwmAHal, p_PwmBHal, p_PwmCHal
#define PHASE_INIT(p_PwmHal, PwmPeriodTicks, PwmAChannel, PwmBChannel, PwmCChannel, p_PinAHal, PinAId, p_PinBHal, PinBId, p_PinCHal, PinCId)	\
{																	\
	.PwmA = PWM_INIT(p_PwmHal, PwmPeriodTicks, PwmAChannel),		\
	.PwmB = PWM_INIT(p_PwmHal, PwmPeriodTicks, PwmBChannel),		\
	.PwmC = PWM_INIT(p_PwmHal, PwmPeriodTicks, PwmCChannel),		\
	.PinA = PIN_INIT(p_PinAHal, PinAId),							\
	.PinB = PIN_INIT(p_PinBHal, PinBId),							\
	.PinC = PIN_INIT(p_PinCHal, PinCId),							\
}

static inline void Phase_ClearInterrupt(const Phase_T * p_phase) 	{ PWM_ClearInterrupt(&p_phase->PwmA); }
static inline void Phase_DisableInterrupt(const Phase_T * p_phase) 	{ PWM_DisableInterrupt(&p_phase->PwmA); }
static inline void Phase_EnableInterrupt(const Phase_T * p_phase) 	{ PWM_EnableInterrupt(&p_phase->PwmA); }

/* Bit Reg operation need to be in single write for sync update  */
//Need to PWM_module or HAL_Phase with awareness of all pwm channels for Sync on writes to 1 register PWM_Module_Enable(&p_phase->Pwm_Module, enumnstate[0,0,0]);
static inline void _Phase_SyncPwmDuty(const Phase_T * p_phase)
{
	PWM_ActuateSync(&p_phase->PwmA);
}

/* Sync activation of Switch and Invert Polarity */
static inline void _Phase_SyncPwmSwitch(const Phase_T * p_phase)
{
#ifndef CONFIG_PHASE_PIN_SWITCH
	PWM_ActuateSync(&p_phase->PwmA); //PWM_Module_ActuateSync(&p_phase->PwmModule);
#endif
}

/******************************************************************************/
/*!
	2/3-Phase Polar common
*/
/*! @{ */
/******************************************************************************/
static inline void _Phase_EnablePwmSwitch(const PWM_T * p_pwm)
{
#ifndef CONFIG_PHASE_PIN_SWITCH
	PWM_Enable(p_pwm);
#endif
}

static inline void _Phase_DisablePwmSwitch(const PWM_T * p_pwm)
{
#ifndef CONFIG_PHASE_PIN_SWITCH
	PWM_Disable(p_pwm);
#endif
}

static inline void _Phase_EnablePinSwitch(const Pin_T * p_pin)
{
#ifdef CONFIG_PHASE_PIN_SWITCH
	Pin_Output_High(p_pin);
#endif
}

static inline void _Phase_DisablePinSwitch(const Pin_T * p_pin)
{
#ifdef CONFIG_PHASE_PIN_SWITCH
	Pin_Output_Low(p_pin);
#endif
}

static inline void _Phase_EnableA(const Phase_T * p_phase) { _Phase_EnablePwmSwitch(&p_phase->PwmA); _Phase_EnablePinSwitch(&p_phase->PinA); }
static inline void _Phase_EnableB(const Phase_T * p_phase) { _Phase_EnablePwmSwitch(&p_phase->PwmB); _Phase_EnablePinSwitch(&p_phase->PinB); }
static inline void _Phase_EnableC(const Phase_T * p_phase) { _Phase_EnablePwmSwitch(&p_phase->PwmC); _Phase_EnablePinSwitch(&p_phase->PinC); }
static inline void _Phase_DisableA(const Phase_T * p_phase) { _Phase_DisablePwmSwitch(&p_phase->PwmA); _Phase_DisablePinSwitch(&p_phase->PinA); }
static inline void _Phase_DisableB(const Phase_T * p_phase) { _Phase_DisablePwmSwitch(&p_phase->PwmB); _Phase_DisablePinSwitch(&p_phase->PinB); }
static inline void _Phase_DisableC(const Phase_T * p_phase) { _Phase_DisablePwmSwitch(&p_phase->PwmC); _Phase_DisablePinSwitch(&p_phase->PinC); }
/******************************************************************************/
/*! @} */
/******************************************************************************/

/*
	Duty 100% == CONFIG_PWM_DUTY_MAX
*/
static inline void Phase_ActivateDuty(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	PWM_ActuateDuty(&p_phase->PwmA, pwmDutyA);
	PWM_ActuateDuty(&p_phase->PwmB, pwmDutyB);
	PWM_ActuateDuty(&p_phase->PwmC, pwmDutyC);
	_Phase_SyncPwmDuty(p_phase);
}

static inline void Phase_ActuateDuty_Frac15(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	PWM_ActuateDuty_Frac15(&p_phase->PwmA, pwmDutyA);
	PWM_ActuateDuty_Frac15(&p_phase->PwmB, pwmDutyB);
	PWM_ActuateDuty_Frac15(&p_phase->PwmC, pwmDutyC);
	_Phase_SyncPwmDuty(p_phase);
}

static inline void Phase_ActuateDuty_Frac16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	PWM_ActuateDuty_Frac16(&p_phase->PwmA, pwmDutyA);
	PWM_ActuateDuty_Frac16(&p_phase->PwmB, pwmDutyB);
	PWM_ActuateDuty_Frac16(&p_phase->PwmC, pwmDutyC);
	_Phase_SyncPwmDuty(p_phase);
}

static inline void Phase_ActivateSwitchABC(const Phase_T * p_phase)
{
	_Phase_EnableA(p_phase);
	_Phase_EnableB(p_phase);
	_Phase_EnableC(p_phase);
	_Phase_SyncPwmSwitch(p_phase);
}

static inline void Phase_Float(const Phase_T * p_phase)
{
	_Phase_DisableA(p_phase);
	_Phase_DisableB(p_phase);
	_Phase_DisableC(p_phase);
	_Phase_SyncPwmSwitch(p_phase);
}

/* DO NOT USE if Bipolar active. Enable all at 0 duty */
static inline void Phase_Ground(const Phase_T * p_phase)
{
	Phase_ActivateDuty(p_phase, 0U, 0U, 0U);
	Phase_ActivateSwitchABC(p_phase);
}

/******************************************************************************/
/*
	3-Phase Polar
*/
/******************************************************************************/
static inline void Phase_Polar_ActivateA(const Phase_T * p_phase, uint16_t duty) 	{ Phase_ActivateDuty(p_phase, duty, 0U, 0U); Phase_ActivateSwitchABC(p_phase); }
static inline void Phase_Polar_ActivateB(const Phase_T * p_phase, uint16_t duty) 	{ Phase_ActivateDuty(p_phase, 0U, duty, 0U); Phase_ActivateSwitchABC(p_phase); }
static inline void Phase_Polar_ActivateC(const Phase_T * p_phase, uint16_t duty) 	{ Phase_ActivateDuty(p_phase, 0U, 0U, duty); Phase_ActivateSwitchABC(p_phase); }
static inline void Phase_Polar_ActivateInvA(const Phase_T * p_phase, uint16_t duty) { Phase_ActivateDuty(p_phase, 0U, duty, duty); Phase_ActivateSwitchABC(p_phase); }
static inline void Phase_Polar_ActivateInvB(const Phase_T * p_phase, uint16_t duty) { Phase_ActivateDuty(p_phase, duty, 0U, duty); Phase_ActivateSwitchABC(p_phase); }
static inline void Phase_Polar_ActivateInvC(const Phase_T * p_phase, uint16_t duty) { Phase_ActivateDuty(p_phase, duty, duty, 0U); Phase_ActivateSwitchABC(p_phase); }

/* Enable all at 0 duty */
static inline void Phase_Polar_Ground(const Phase_T * p_phase)
{
	if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR)
	{
		Phase_Float(p_phase);
		PWM_DisableInvertPolarity(&p_phase->PwmA);
		PWM_DisableInvertPolarity(&p_phase->PwmB);
		PWM_DisableInvertPolarity(&p_phase->PwmC);
	}

	Phase_Ground(p_phase);
}
/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
	2-Phase Polar PWM
	if Pwm is updated every cycle, no need to use buffered write
*/
/*! @{ */
/******************************************************************************/
static inline void _Phase_ActivateSwitchNotA(const Phase_T * p_phase) { _Phase_DisableA(p_phase); _Phase_EnableB(p_phase); _Phase_EnableC(p_phase); _Phase_SyncPwmSwitch(p_phase); }
static inline void _Phase_ActivateSwitchNotB(const Phase_T * p_phase) { _Phase_EnableA(p_phase); _Phase_DisableB(p_phase); _Phase_EnableC(p_phase); _Phase_SyncPwmSwitch(p_phase); }
static inline void _Phase_ActivateSwitchNotC(const Phase_T * p_phase) { _Phase_EnableA(p_phase); _Phase_EnableB(p_phase); _Phase_DisableC(p_phase); _Phase_SyncPwmSwitch(p_phase); }

/*
	For Bipolar Pwm
*/
static inline void _Phase_ActivateInvertPolarityA(const Phase_T * p_phase) { PWM_EnableInvertPolarity(&p_phase->PwmA); PWM_DisableInvertPolarity(&p_phase->PwmB); PWM_DisableInvertPolarity(&p_phase->PwmC); }
static inline void _Phase_ActivateInvertPolarityB(const Phase_T * p_phase) { PWM_DisableInvertPolarity(&p_phase->PwmA); PWM_EnableInvertPolarity(&p_phase->PwmB); PWM_DisableInvertPolarity(&p_phase->PwmC); }
static inline void _Phase_ActivateInvertPolarityC(const Phase_T * p_phase) { PWM_DisableInvertPolarity(&p_phase->PwmA); PWM_DisableInvertPolarity(&p_phase->PwmB); PWM_EnableInvertPolarity(&p_phase->PwmC); }

/*
	Activate Switches Only
*/
static inline void Phase_Polar_ActivateSwitchAC(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateSwitchNotB(p_phase); }
static inline void Phase_Polar_ActivateSwitchBC(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateSwitchNotA(p_phase); }
static inline void Phase_Polar_ActivateSwitchBA(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateSwitchNotC(p_phase); }
static inline void Phase_Polar_ActivateSwitchCA(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateSwitchNotB(p_phase); }
static inline void Phase_Polar_ActivateSwitchCB(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateSwitchNotA(p_phase); }
static inline void Phase_Polar_ActivateSwitchAB(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateSwitchNotC(p_phase); }

/*
	Activate Duty Only
*/
static inline void Phase_Unipolar1_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDuty(&p_phase->PwmA, duty); PWM_ActuateDuty(&p_phase->PwmC, 0U); }
static inline void Phase_Unipolar1_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDuty(&p_phase->PwmB, duty); PWM_ActuateDuty(&p_phase->PwmC, 0U); }
static inline void Phase_Unipolar1_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDuty(&p_phase->PwmB, duty); PWM_ActuateDuty(&p_phase->PwmA, 0U); }
static inline void Phase_Unipolar1_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDuty(&p_phase->PwmC, duty); PWM_ActuateDuty(&p_phase->PwmA, 0U); }
static inline void Phase_Unipolar1_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDuty(&p_phase->PwmC, duty); PWM_ActuateDuty(&p_phase->PwmB, 0U); }
static inline void Phase_Unipolar1_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDuty(&p_phase->PwmA, duty); PWM_ActuateDuty(&p_phase->PwmB, 0U); }

/*
	PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
	PwmNegative = PwmPeriodTotal/2 - PwmScalar/2
*/
static inline void Phase_Unipolar2_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmC, duty); }
static inline void Phase_Unipolar2_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmC, duty); }
static inline void Phase_Unipolar2_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmA, duty); }
static inline void Phase_Unipolar2_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmA, duty); }
static inline void Phase_Unipolar2_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmB, duty); }
static inline void Phase_Unipolar2_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmB, duty); }

/*
	PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
	PwmNegative = PwmPeriodTotal/2 + PwmScalar/2, Inverse polarity
*/
static inline void Phase_Bipolar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); }
static inline void Phase_Bipolar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); }
static inline void Phase_Bipolar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); }
static inline void Phase_Bipolar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); }
static inline void Phase_Bipolar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); }
static inline void Phase_Bipolar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); }

static inline void Phase_Polar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty)
{
	switch(p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyAC(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyAC(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyAC(p_phase, duty);	break;
		default: break;
	}
	_Phase_SyncPwmDuty(p_phase);
}

static inline void Phase_Polar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty)
{
	switch(p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyBC(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyBC(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyBC(p_phase, duty);	break;
		default: break;
	}
	_Phase_SyncPwmDuty(p_phase);
}

static inline void Phase_Polar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty)
{
	switch(p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyBA(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyBA(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyBA(p_phase, duty);	break;
		default: break;
	}
	_Phase_SyncPwmDuty(p_phase);
}

static inline void Phase_Polar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty)
{
	switch(p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyCA(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyCA(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyCA(p_phase, duty);	break;
		default: break;
	}
	_Phase_SyncPwmDuty(p_phase);
}

static inline void Phase_Polar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty)
{
	switch(p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyCB(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyCB(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyCB(p_phase, duty);	break;
		default: break;
	}
	_Phase_SyncPwmDuty(p_phase);
}

static inline void Phase_Polar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty)
{
	switch(p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyAB(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyAB(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyAB(p_phase, duty);	break;
		default: break;
	}
	_Phase_SyncPwmDuty(p_phase);
}

/*
	Activate Duty and Sets On/Off State
*/
static inline void Phase_Polar_ActivateAC(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyAC(p_phase, duty); Phase_Polar_ActivateSwitchAC(p_phase); }
static inline void Phase_Polar_ActivateBC(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyBC(p_phase, duty); Phase_Polar_ActivateSwitchBC(p_phase); }
static inline void Phase_Polar_ActivateBA(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyBA(p_phase, duty); Phase_Polar_ActivateSwitchBA(p_phase); }
static inline void Phase_Polar_ActivateCA(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyCA(p_phase, duty); Phase_Polar_ActivateSwitchCA(p_phase); }
static inline void Phase_Polar_ActivateCB(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyCB(p_phase, duty); Phase_Polar_ActivateSwitchCB(p_phase); }
static inline void Phase_Polar_ActivateAB(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyAB(p_phase, duty); Phase_Polar_ActivateSwitchAB(p_phase); }
/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*! Extern */
/******************************************************************************/
extern void Phase_Init(Phase_T * p_phase);
extern void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Mode_T phaseMode);
extern void Phase_Polar_Activate(Phase_T * p_phase, Phase_Id_T phaseId, uint16_t duty);
extern void Phase_Polar_ActivateDuty(Phase_T * p_phase, Phase_Id_T phaseId, uint16_t duty);
extern void Phase_Polar_ActivateSwitch(Phase_T * p_phase, Phase_Id_T phaseId);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif
