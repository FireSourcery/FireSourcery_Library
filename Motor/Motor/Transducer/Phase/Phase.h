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
    @file 	Phase.h
    @author FireSoucery
    @brief  3-Phase PWM functions

  	Assumes each phase is a complementary PWM output.
	2-Phase settings: PWM positive side MOSFETs, ground side bottom MOSFET stays on.
	e.g. PhaseAB -> PWM phase A MOSFETs, phase B bottom MOSFET stays on.
	Implementation details; dead time, are delegated to user/HAL functions

	Includes 2-Phase implementation

    @version V0
*/
/******************************************************************************/
#ifndef PHASE_H
#define PHASE_H

#include "Peripheral/PWM/PWM.h"
#include "Peripheral/Pin/Pin.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	PHASE_MODE_UNIPOLAR_1,	/*!< PHASE_MODE_UNIPOLAR_1 */
	PHASE_MODE_UNIPOLAR_2,	/*!< PHASE_MODE_UNIPOLAR_2 */
	PHASE_MODE_BIPOLAR   	/*!< PHASE_MODE_BIPOLAR */
}
Phase_Mode_T;

typedef enum
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

typedef struct
{
	//	#ifdef CONFIG_PHASE_EXTERNAL_SWITCH
	Pin_T OnOffA;
	Pin_T OnOffB;
	Pin_T OnOffC;
	//	#endif

// 	const Phase_Params_T * const P_PARAMS;
}
Phase_Config_T;

typedef struct
{
	Phase_Config_T CONFIG;

	PWM_T PwmA;
	PWM_T PwmB;
	PWM_T PwmC;

//	PWM3X_T Pwm3x;

	Phase_Mode_T PhaseMode;
}
Phase_T;

#define PHASE_CONFIG(p_OnOffA_Hal, OnOffA_Id, p_OnOffB_Hal, OnOffB_Id, p_OnOffC_Hal, OnOffC_Id, p_Pwm_Hal, Pwm_PeriodTicks, PwmA_Channel, PwmB_Channel, PwmC_Channel)	\
{																	\
	.CONFIG = 														\
	{																\
		.OnOffA = PIN_CONFIG(p_OnOffA_Hal, OnOffA_Id),				\
		.OnOffB = PIN_CONFIG(p_OnOffB_Hal, OnOffB_Id),				\
		.OnOffC = PIN_CONFIG(p_OnOffC_Hal, OnOffC_Id),				\
	},																\
	.PwmA = PWM_CONFIG(p_Pwm_Hal, Pwm_PeriodTicks, PwmA_Channel),	\
	.PwmB = PWM_CONFIG(p_Pwm_Hal, Pwm_PeriodTicks, PwmB_Channel),	\
	.PwmC = PWM_CONFIG(p_Pwm_Hal, Pwm_PeriodTicks, PwmC_Channel),	\
}


static inline void Phase_ClearInterrupt(const Phase_T * p_phase)
{
	PWM_ClearInterrupt(&p_phase->PwmA); //configurable clear all if needed
}

static inline void SyncPhasePwm(const Phase_T * p_phase)
{
	PWM_ActuateSync(&p_phase->PwmA); //configurable clear all if needed
//	PWM_Sync(&p_phase->PwmB);
//	PWM_Sync(&p_phase->PwmC);
}

static inline void EnablePhaseExternalSwitch(const Pin_T * p_pin)
{
//	#ifdef CONFIG_PHASE_EXTERNAL_SWITCH
	Pin_Output_On(p_pin);
//	#endif
}

static inline void DisablePhaseExternalSwitch(const Pin_T * p_pin)
{
//	#ifdef CONFIG_PHASE_EXTERNAL_SWITCH
	Pin_Output_Off(p_pin);
//	#endif
}

static inline void EnablePhaseA(const Phase_T * p_phase) 	{PWM_Enable(&p_phase->PwmA); 	EnablePhaseExternalSwitch(&p_phase->CONFIG.OnOffA);}
static inline void EnablePhaseB(const Phase_T * p_phase) 	{PWM_Enable(&p_phase->PwmB);	EnablePhaseExternalSwitch(&p_phase->CONFIG.OnOffB);}
static inline void EnablePhaseC(const Phase_T * p_phase) 	{PWM_Enable(&p_phase->PwmC);	EnablePhaseExternalSwitch(&p_phase->CONFIG.OnOffC);}
static inline void DisablePhaseA(const Phase_T * p_phase) 	{PWM_Disable(&p_phase->PwmA);	DisablePhaseExternalSwitch(&p_phase->CONFIG.OnOffA);}
static inline void DisablePhaseB(const Phase_T * p_phase) 	{PWM_Disable(&p_phase->PwmB);	DisablePhaseExternalSwitch(&p_phase->CONFIG.OnOffB);}
static inline void DisablePhaseC(const Phase_T * p_phase) 	{PWM_Disable(&p_phase->PwmC);	DisablePhaseExternalSwitch(&p_phase->CONFIG.OnOffC);}

/*
 * For SVPWM
	where CONFIG_PWM_DUTY_MAX is 100% duty
 */
static inline void Phase_ActivateDuty(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	PWM_ActuateDuty(&p_phase->PwmA, pwmDutyA);
	PWM_ActuateDuty(&p_phase->PwmB, pwmDutyB);
	PWM_ActuateDuty(&p_phase->PwmC, pwmDutyC);
	SyncPhasePwm(p_phase);
}

//static inline void Phase_ActuateDuty_Frac15(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
//{
//	PWM_ActuateDuty_Frac15(&p_phase->PwmA, pwmDutyA);
//	PWM_ActuateDuty_Frac15(&p_phase->PwmB, pwmDutyB);
//	PWM_ActuateDuty_Frac15(&p_phase->PwmC, pwmDutyC);
//	SyncPhasePwm(p_phase);
//}
//
//static inline void Phase_ActuateDuty_Frac16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
//{
//	PWM_ActuateDuty_Frac16(&p_phase->PwmA, pwmDutyA);
//	PWM_ActuateDuty_Frac16(&p_phase->PwmB, pwmDutyB);
//	PWM_ActuateDuty_Frac16(&p_phase->PwmC, pwmDutyC);
//	SyncPhasePwm(p_phase);
//}

/*
 *
 */
static inline void Phase_Float(const Phase_T * p_phase)
{
	DisablePhaseA(p_phase);
	DisablePhaseB(p_phase);
	DisablePhaseC(p_phase);
//	SyncPhasePwm(p_phase);  Need to change to PWM3 or HAL_Phase for Sync on writes to 1 register
	//	PWM_ActuateOnOff(&p_phase->Pwm3x, enumnstate[0,0,0]);
}

static inline void Phase_Ground(const Phase_T * p_phase)
{
	if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR)
	{
		Phase_Float(p_phase);
		PWM_DisableInvertPolarity(&p_phase->PwmA);
		PWM_DisableInvertPolarity(&p_phase->PwmB);
		PWM_DisableInvertPolarity(&p_phase->PwmC);
	}

	Phase_ActivateDuty(p_phase, 0U, 0U, 0U);
	EnablePhaseA(p_phase);
	EnablePhaseB(p_phase);
	EnablePhaseC(p_phase);
}

static inline void Phase_ActivateSwitchABC(const Phase_T * p_phase)
{
	EnablePhaseA(p_phase);
	EnablePhaseB(p_phase);
	EnablePhaseC(p_phase);
}

/******************************************************************************/
/*!
	2-Phase Polar PWM
	if Pwm is updated every cycle, no need to save calculated value
 */
/*! @{ */
/******************************************************************************/
/* Bit Reg operation need to be in single write for sync update  */
//static inline void EnablePhaseNotB(const Phase_T * p_phase)
//{
//	PWM_Enable(&p_phase->Pwm3x, 0b101);
//	SyncPhasePwm(p_phase);
//}

static inline void ActivatePhaseSwitchABC(const Phase_T * p_phase) 	{EnablePhaseA(p_phase); EnablePhaseB(p_phase); EnablePhaseC(p_phase); 	}
static inline void ActivatePhaseSwitchNotB(const Phase_T * p_phase) {EnablePhaseA(p_phase); DisablePhaseB(p_phase); EnablePhaseC(p_phase); 	}
static inline void ActivatePhaseSwitchNotA(const Phase_T * p_phase) {DisablePhaseA(p_phase); EnablePhaseB(p_phase); EnablePhaseC(p_phase); 	}
static inline void ActivatePhaseSwitchNotC(const Phase_T * p_phase) {EnablePhaseA(p_phase); EnablePhaseB(p_phase); DisablePhaseC(p_phase); 	}

static inline void ActivatePhaseSwitchInvertA(const Phase_T * p_phase) {PWM_EnableInvertPolarity(&p_phase->PwmA); PWM_DisableInvertPolarity(&p_phase->PwmB); PWM_DisableInvertPolarity(&p_phase->PwmC);}
static inline void ActivatePhaseSwitchInvertB(const Phase_T * p_phase) {PWM_DisableInvertPolarity(&p_phase->PwmA); PWM_EnableInvertPolarity(&p_phase->PwmB); PWM_DisableInvertPolarity(&p_phase->PwmC);}
static inline void ActivatePhaseSwitchInvertC(const Phase_T * p_phase) {PWM_DisableInvertPolarity(&p_phase->PwmA); PWM_DisableInvertPolarity(&p_phase->PwmB); PWM_EnableInvertPolarity(&p_phase->PwmC);}

static inline void Phase_Polar_ActivateSwitchAC(const Phase_T * p_phase) {if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) {Phase_Float(p_phase); ActivatePhaseSwitchInvertC(p_phase);} ActivatePhaseSwitchNotB(p_phase);}
static inline void Phase_Polar_ActivateSwitchBC(const Phase_T * p_phase) {if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) {Phase_Float(p_phase); ActivatePhaseSwitchInvertC(p_phase);} ActivatePhaseSwitchNotA(p_phase);}
static inline void Phase_Polar_ActivateSwitchBA(const Phase_T * p_phase) {if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) {Phase_Float(p_phase); ActivatePhaseSwitchInvertA(p_phase);} ActivatePhaseSwitchNotC(p_phase);}
static inline void Phase_Polar_ActivateSwitchCA(const Phase_T * p_phase) {if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) {Phase_Float(p_phase); ActivatePhaseSwitchInvertA(p_phase);} ActivatePhaseSwitchNotB(p_phase);}
static inline void Phase_Polar_ActivateSwitchCB(const Phase_T * p_phase) {if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) {Phase_Float(p_phase); ActivatePhaseSwitchInvertB(p_phase);} ActivatePhaseSwitchNotA(p_phase);}
static inline void Phase_Polar_ActivateSwitchAB(const Phase_T * p_phase) {if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) {Phase_Float(p_phase); ActivatePhaseSwitchInvertB(p_phase);} ActivatePhaseSwitchNotC(p_phase);}

static inline void Phase_Unipolar1_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDuty(&p_phase->PwmA, duty); PWM_ActuateDuty(&p_phase->PwmC, 0U); }
static inline void Phase_Unipolar1_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDuty(&p_phase->PwmB, duty); PWM_ActuateDuty(&p_phase->PwmC, 0U); }
static inline void Phase_Unipolar1_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDuty(&p_phase->PwmB, duty); PWM_ActuateDuty(&p_phase->PwmA, 0U); }
static inline void Phase_Unipolar1_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDuty(&p_phase->PwmC, duty); PWM_ActuateDuty(&p_phase->PwmA, 0U); }
static inline void Phase_Unipolar1_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDuty(&p_phase->PwmC, duty); PWM_ActuateDuty(&p_phase->PwmB, 0U); }
static inline void Phase_Unipolar1_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDuty(&p_phase->PwmA, duty); PWM_ActuateDuty(&p_phase->PwmB, 0U); }

/*
	PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
	PwmNegative = PwmPeriodTotal/2 - PwmScalar/2
 */
static inline void Phase_Unipolar2_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmC, duty); }
static inline void Phase_Unipolar2_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmC, duty); }
static inline void Phase_Unipolar2_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmA, duty); }
static inline void Phase_Unipolar2_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmA, duty); }
static inline void Phase_Unipolar2_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmB, duty); }
static inline void Phase_Unipolar2_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmB, duty); }

/*
	PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
	PwmNegative = PwmPeriodTotal/2 + PwmScalar/2, Inverse polarity
 */
static inline void Phase_Bipolar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); }
static inline void Phase_Bipolar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); }
static inline void Phase_Bipolar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); }
static inline void Phase_Bipolar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); }
static inline void Phase_Bipolar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); }
static inline void Phase_Bipolar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) {PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); }

static inline void Phase_Polar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty)
{
	switch (p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyAC(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyAC(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyAC(p_phase, duty); 	break;
		default: break;
	}
	SyncPhasePwm(p_phase);
}

static inline void Phase_Polar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty)
{
	switch (p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyBC(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyBC(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyBC(p_phase, duty); 	break;
		default: break;
	}
	SyncPhasePwm(p_phase);
}

static inline void Phase_Polar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty)
{
	switch (p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyBA(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyBA(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyBA(p_phase, duty);	break;
		default: break;
	}
	SyncPhasePwm(p_phase);
}

static inline void Phase_Polar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty)
{
	switch (p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyCA(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyCA(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyCA(p_phase, duty); 	break;
		default: break;
	}
	SyncPhasePwm(p_phase);
}

static inline void Phase_Polar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty)
{
	switch (p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyCB(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyCB(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyCB(p_phase, duty); 	break;
		default: break;
	}
	SyncPhasePwm(p_phase);
}

static inline void Phase_Polar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty)
{
	switch (p_phase->PhaseMode)
	{
		case PHASE_MODE_UNIPOLAR_1:	Phase_Unipolar1_ActivateDutyAB(p_phase, duty);	break;
		case PHASE_MODE_UNIPOLAR_2:	Phase_Unipolar2_ActivateDutyAB(p_phase, duty);	break;
		case PHASE_MODE_BIPOLAR:	Phase_Bipolar_ActivateDutyAB(p_phase, duty);	break;
		default: break;
	}
	SyncPhasePwm(p_phase);
}


/*
 * Activate Duty and Sets On/Off State
 */
static inline void Phase_Polar_ActivateAC(const Phase_T * p_phase, uint16_t duty) {Phase_Polar_ActivateDutyAC(p_phase, duty); Phase_Polar_ActivateSwitchAC(p_phase);}
static inline void Phase_Polar_ActivateBC(const Phase_T * p_phase, uint16_t duty) {Phase_Polar_ActivateDutyBC(p_phase, duty); Phase_Polar_ActivateSwitchBC(p_phase);}
static inline void Phase_Polar_ActivateBA(const Phase_T * p_phase, uint16_t duty) {Phase_Polar_ActivateDutyBA(p_phase, duty); Phase_Polar_ActivateSwitchBA(p_phase);}
static inline void Phase_Polar_ActivateCA(const Phase_T * p_phase, uint16_t duty) {Phase_Polar_ActivateDutyCA(p_phase, duty); Phase_Polar_ActivateSwitchCA(p_phase);}
static inline void Phase_Polar_ActivateCB(const Phase_T * p_phase, uint16_t duty) {Phase_Polar_ActivateDutyCB(p_phase, duty); Phase_Polar_ActivateSwitchCB(p_phase);}
static inline void Phase_Polar_ActivateAB(const Phase_T * p_phase, uint16_t duty) {Phase_Polar_ActivateDutyAB(p_phase, duty); Phase_Polar_ActivateSwitchAB(p_phase);}

static inline void Phase_Polar_ActivateA(const Phase_T * p_phase, uint16_t duty) 	{Phase_ActivateDuty(p_phase, duty, 0U, 0U); ActivatePhaseSwitchABC(p_phase);}
static inline void Phase_Polar_ActivateB(const Phase_T * p_phase, uint16_t duty) 	{Phase_ActivateDuty(p_phase, 0U, duty, 0U); ActivatePhaseSwitchABC(p_phase);}
static inline void Phase_Polar_ActivateC(const Phase_T * p_phase, uint16_t duty) 	{Phase_ActivateDuty(p_phase, 0U, 0U, duty); ActivatePhaseSwitchABC(p_phase);}
static inline void Phase_Polar_ActivateInvA(const Phase_T * p_phase, uint16_t duty) 	{Phase_ActivateDuty(p_phase, 0U, duty, duty); ActivatePhaseSwitchABC(p_phase);}
static inline void Phase_Polar_ActivateInvB(const Phase_T * p_phase, uint16_t duty) 	{Phase_ActivateDuty(p_phase, duty, 0U, duty); ActivatePhaseSwitchABC(p_phase);}
static inline void Phase_Polar_ActivateInvC(const Phase_T * p_phase, uint16_t duty) 	{Phase_ActivateDuty(p_phase, duty, duty, 0U); ActivatePhaseSwitchABC(p_phase);}

/******************************************************************************/
/*! @} */
/******************************************************************************/
/*
 * This module handles Phase Id
 */
static inline void Phase_Polar_Activate(Phase_T * p_phase, Phase_Id_T phaseId, uint16_t duty)
{
	switch (phaseId)
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
	//	CommutationTable[phaseID][p_phase->PhaseMode]();

	//	if (p_phase->SinusoidalModulation)
	//	{
	//
	//		p_phase->Index = 0;
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

static inline void Phase_Polar_ActivateDuty(Phase_T * p_phase, Phase_Id_T phaseId, uint16_t duty)
{
	switch (phaseId)
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

static inline void Phase_Polar_ActivateSwitch(Phase_T * p_phase, Phase_Id_T phaseId)
{
	switch (phaseId)
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



//extern const uint16_t MATH_SVPWM_SADDLE_120[170];
//
//static inline uint16_t svpwm_saddle120(qangle16_t theta)
//{
//	return MATH_SVPWM_SADDLE_120[(((uint16_t)theta) >> 7U)];
//}
//static inline uint16_t svpwm_saddle(qangle16_t theta)
//{
//	uint16_t saddle;
//
//	if ((uint16_t)theta < (uint16_t)QANGLE16_120)
//	{
//		saddle = svpwm_saddle120(theta);
//	}
//	else if ((uint16_t)theta < (uint16_t)QANGLE16_240)
//	{
//		saddle = svpwm_saddle120((QANGLE16_240 - 1U - theta));
//	}
//	else
//	{
//		saddle = 0U;
//	}
//
//	return saddle;
//}
//
///*
// *
// */
//static inline void svpwm_unipolar1(uint16_t * p_dutyA, uint16_t * p_dutyB, uint16_t * p_dutyC, uint16_t dutyScalar, qangle16_t rotorAngle, qangle16_t voltageLeadAngle)
//{
//	qangle16_t angleA;
//	qangle16_t angleB;
//	qangle16_t angleC;
//
//	uint16_t saddleA;
//	uint16_t saddleB;
//	uint16_t saddleC;
//
//	#define PHASE_SHIFT_A  QANGLE16_120
//	#define PHASE_SHIFT_B  QANGLE16_240
//	#define PHASE_SHIFT_C  0
//
//	angleA = PHASE_SHIFT_A + rotorAngle + voltageLeadAngle; // angle loops
//	angleB = PHASE_SHIFT_B + rotorAngle + voltageLeadAngle;
//	angleC = PHASE_SHIFT_C + rotorAngle + voltageLeadAngle;
//
//	saddleA = svpwm_saddle(angleA);
//	saddleB = svpwm_saddle(angleB);
//	saddleC = svpwm_saddle(angleC);
//
//	//saddle and duty are NOT qfrac16
//	*p_dutyA = ((uint32_t)saddleA * (uint32_t)dutyScalar) / 65536U;
//	*p_dutyB = ((uint32_t)saddleB * (uint32_t)dutyScalar) / 65536U;
//	*p_dutyC = ((uint32_t)saddleC * (uint32_t)dutyScalar) / 65536U;
//}

/*
 * simpler, single vector angle compared to FOC, Vq only
 */
//static inline void ActivatePhaseVoltageAngle(Phase_T * p_phase, uint16_t theta)
//{
	/* theta = 0 => Va Max*/

//	#define PHASE_SHIFT_A  (65536U/2U)
//	#define PHASE_SHIFT_B  (65536U/2U - 65536U/3U)
//	#define PHASE_SHIFT_C  (65536U/2U + 65536U/3U)
//
//	uint16_t angleA = PHASE_SHIFT_A + theta; // angle loops
//	uint16_t angleB = PHASE_SHIFT_B + theta;
//	uint16_t angleC = PHASE_SHIFT_C + theta;

	//if 384 table
//	uint16_t angleAIndex = (uint32_t)angleA * 3U / 512U;
//	uint16_t angleBIndex = (uint32_t)angleB * 3U / 512U;
//	uint16_t angleCIndex = (uint32_t)angleC * 3U / 512U;
//}

//static inline void Phase_Polar_ActivateEstimateAngle(Phase_T * p_phase, uint16_t hallAngle, uint16_t anglespeedtime)
//{
//  angleA = PHASE_SHIFT_A + rotorAngle + voltageLeadAngle; // angle loops
//	angleB = PHASE_SHIFT_B + rotorAngle + voltageLeadAngle;
//	angleC = PHASE_SHIFT_C + rotorAngle + voltageLeadAngle;
//}

extern void Phase_Init(Phase_T * p_phase);
extern void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Mode_T phaseMode);

#endif
