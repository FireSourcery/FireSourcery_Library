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
	@file 	PWM.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef PWM_H
#define PWM_H

#include "HAL_PWM.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

//typedef struct
//{
//	HAL_PWM_T * const P_HAL_PWM;
//	const uint32_t PERIOD_TICKS;
//}
//PWM_Module_T;

//typedef struct
//{
//	HAL_PWM_Channel_T * const P_HAL;
//	const uint32_t CHANNEL_ID;
//}
//PWM_Channel_T;

typedef const struct PWM_Config_Tag
{
	HAL_PWM_T * const P_HAL_PWM;
	const uint32_t CHANNEL_ID;
	const uint32_t PERIOD_TICKS;
	//	PWM_Module_T * P_MODULE;
}
PWM_Config_T;

typedef struct PWM_Tag
{
	const PWM_Config_T CONFIG;
	/*
		Buffered data
	*/
	//	uint32_t Duty_Ticks;
	//	bool IsOn;
	//	bool IsInvertPolarity;
}
PWM_T;

#define PWM_CONFIG(p_Hal, Peroid_Ticks, Channel) 												\
{																								\
	.CONFIG = {.P_HAL_PWM = p_Hal, .PERIOD_TICKS = Peroid_Ticks, .CHANNEL_ID = Channel},		\
}

static inline void PWM_ClearInterrupt(const PWM_T * p_pwm)
{
	HAL_PWM_ClearInterrupt(p_pwm->CONFIG.P_HAL_PWM);
}

static inline uint32_t CalcPwmDutyTicks(const PWM_T * p_pwm, uint32_t duty)
{
	return p_pwm->CONFIG.PERIOD_TICKS * duty / CONFIG_PWM_DUTY_MAX;
}

/*
	Actuate arguments immediately
*/
static inline void PWM_ActuateDuty_Ticks(const PWM_T * p_pwm, uint32_t pwmDuty_Ticks)
{
	HAL_PWM_WriteDuty(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID, pwmDuty_Ticks);
}

/*
	where PWM_DUTY_CYCLE_MAX is 100% duty
*/
static inline void PWM_ActuateDuty(const PWM_T * p_pwm, uint32_t pwmDuty)
{
	PWM_ActuateDuty_Ticks(p_pwm, CalcPwmDutyTicks(p_pwm, pwmDuty));
}

static inline void PWM_ActuateDutyMidPlus(const PWM_T * p_pwm, uint32_t pwmDuty)
{
	uint32_t ticks = (p_pwm->CONFIG.PERIOD_TICKS + CalcPwmDutyTicks(p_pwm, pwmDuty)) / 2U;
	PWM_ActuateDuty_Ticks(p_pwm, ticks);
}

static inline void PWM_ActuateDutyMidMinus(const PWM_T * p_pwm, uint32_t pwmDuty)
{
	uint32_t ticks = (p_pwm->CONFIG.PERIOD_TICKS - CalcPwmDutyTicks(p_pwm, pwmDuty)) / 2U;
	PWM_ActuateDuty_Ticks(p_pwm, ticks);
}

static inline void PWM_ActuateDuty_Frac16(const PWM_T * p_pwm, uint16_t pwmDuty)
{
	PWM_ActuateDuty_Ticks(p_pwm, pwmDuty * p_pwm->CONFIG.PERIOD_TICKS >> 16U);
}

static inline void PWM_ActuateDuty_Frac15(const PWM_T * p_pwm, uint16_t pwmDuty)
{
	PWM_ActuateDuty_Ticks(p_pwm, pwmDuty * p_pwm->CONFIG.PERIOD_TICKS >> 15U);
}

/*
	If multiple pwms share a register. may interfere with sync
*/
static inline void PWM_Enable(const PWM_T * p_pwm)
{
	HAL_PWM_EnableOutput(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID);
}

static inline void PWM_Disable(const PWM_T * p_pwm)
{
	HAL_PWM_DisableOutput(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID);
}

static inline void PWM_EnableInvertPolarity(const PWM_T * p_pwm)
{
	HAL_PWM_EnableInvertPolarity(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID);
}

static inline void PWM_DisableInvertPolarity(const PWM_T * p_pwm)
{
	HAL_PWM_DisableInvertPolarity(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID);
}

static inline void PWM_ActuateSync(const PWM_T * p_pwm)
{
	HAL_PWM_Sync(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID);
}

extern void PWM_Init(PWM_T * p_pwm);
extern void PWM_InitChannel(PWM_T * p_pwm);
extern void PWM_InitModule(PWM_T * p_pwm);

#endif

/*
	Use buffered data
*/
 //static inline void PWM_SetDuty(PWM_T * p_pwm, uint32_t pwmDuty)
 //{
 //	p_pwm->Duty_Ticks = CalcPwmDutyTicks(p_pwm, pwmDuty);
 //}
 //
 //static inline void PWM_SetDuty_Frac16(PWM_T * p_pwm, uint16_t pwmDuty)
 //{
 //	p_pwm->Duty_Ticks = pwmDuty * p_pwm->CONFIG.PERIOD_TICKS >> 16U;
 //}
 //
 //static inline void PWM_SetDuty_Frac15(PWM_T * p_pwm, uint16_t pwmDuty)
 //{
 //	p_pwm->Duty_Ticks = pwmDuty * p_pwm->CONFIG.PERIOD_TICKS >> 15U;
 //}
 //
 //static inline void PWM_SetEnable(PWM_T * p_pwm)
 //{
 //	p_pwm->IsOn = true;
 //}
 //
 //static inline void PWM_SetDisable(PWM_T * p_pwm)
 //{
 //	p_pwm->IsOn = false;
 //}
 //
 //static inline void PWM_ActuateDutyThis(const PWM_T * p_pwm)
 //{
 //	HAL_PWM_WriteDuty(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID, p_pwm->Duty_Ticks);
 //}
 //
 //static inline void PWM_ActuateOnOffThis(const PWM_T * p_pwm)
 //{
 //	(p_pwm->IsOn == true) ? PWM_Enable(p_pwm->CONFIG.P_HAL_PWM) : PWM_Disable(p_pwm->CONFIG.P_HAL_PWM);
 //}
 //
 //static inline void PWM_ActuateInvertPolarityThis(const PWM_T * p_pwm)
 //{
 //	(p_pwm->IsInvertPolarity == true) ? PWM_EnableInvertPolarity(p_pwm->CONFIG.P_HAL_PWM) : PWM_DisableInvertPolarity(p_pwm->CONFIG.P_HAL_PWM);
 //}
 //
 //static inline void PWM_ActuateThis(const PWM_T * p_pwm)
 //{
 //	PWM_ActuateDutyThis(p_pwm);
 //	PWM_ActuateOnOffThis(p_pwm);
 //	PWM_ActuateInvertPolarityThis(p_pwm);
 //	PWM_ActuateSync(p_pwm); //user call sync in other cases
 //}