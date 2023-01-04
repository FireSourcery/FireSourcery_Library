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
	@file 	PWM.h
	@author FireSourcery
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

typedef const struct PWM_Config_Tag
{
	// HAL_PWM_Module_T * const P_HAL_PWM_MODULE;
	HAL_PWM_T * const P_HAL_PWM;
	const uint32_t CHANNEL_ID;
	const uint32_t PERIOD_TICKS;
}
PWM_Config_T;

typedef struct PWM_Tag
{
	const PWM_Config_T CONFIG;
}
PWM_T;

#define PWM_INIT(p_Hal, Peroid_Ticks, Channel) 												\
{																							\
	.CONFIG = { .P_HAL_PWM = p_Hal, .PERIOD_TICKS = Peroid_Ticks, .CHANNEL_ID = Channel },	\
}

#define PWM_DUTY16 (65536U)

/*
	Channel
*/
static inline uint32_t _PWM_CalcPwmDutyTicks(const PWM_T * p_pwm, uint32_t duty) { return p_pwm->CONFIG.PERIOD_TICKS * duty / PWM_DUTY16; }

/*
	Actuate arguments immediately, unless use sync is enabled
*/
static inline void PWM_ActuateDuty_Ticks(const PWM_T * p_pwm, uint32_t pwmDuty_Ticks) { HAL_PWM_WriteDuty(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID, pwmDuty_Ticks); }

/* where CONFIG_PWM_DUTY_MAX is 100% duty */
static inline void PWM_ActuateDuty(const PWM_T * p_pwm, uint32_t pwmDuty) { PWM_ActuateDuty_Ticks(p_pwm, _PWM_CalcPwmDutyTicks(p_pwm, pwmDuty)); }

static inline void PWM_ActuateDutyMidPlus(const PWM_T * p_pwm, uint32_t pwmDuty)
{
	PWM_ActuateDuty_Ticks(p_pwm, (p_pwm->CONFIG.PERIOD_TICKS + _PWM_CalcPwmDutyTicks(p_pwm, pwmDuty)) / 2U);
}

static inline void PWM_ActuateDutyMidMinus(const PWM_T * p_pwm, uint32_t pwmDuty)
{
	PWM_ActuateDuty_Ticks(p_pwm, (p_pwm->CONFIG.PERIOD_TICKS - _PWM_CalcPwmDutyTicks(p_pwm, pwmDuty)) / 2U);
}

static inline void PWM_ActuateDuty_Frac16(const PWM_T * p_pwm, uint16_t pwmDuty16) { PWM_ActuateDuty_Ticks(p_pwm, (uint32_t)pwmDuty16 * p_pwm->CONFIG.PERIOD_TICKS >> 16U); }
static inline void PWM_ActuateDuty_Frac15(const PWM_T * p_pwm, uint16_t pwmDuty15) { PWM_ActuateDuty_Ticks(p_pwm, (uint32_t)pwmDuty15 * p_pwm->CONFIG.PERIOD_TICKS >> 15U); }

static inline void PWM_Enable(const PWM_T * p_pwm) 					{ HAL_PWM_EnableOutput(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID); }
static inline void PWM_Disable(const PWM_T * p_pwm) 				{ HAL_PWM_DisableOutput(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID); }
static inline void PWM_EnableInvertPolarity(const PWM_T * p_pwm) 	{ HAL_PWM_EnableInvertPolarity(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID); }
static inline void PWM_DisableInvertPolarity(const PWM_T * p_pwm) 	{ HAL_PWM_DisableInvertPolarity(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID); }

/*
	Module
*/
/* Shared interrupt */
static inline void PWM_ClearInterrupt(const PWM_T * p_pwm) 		{ HAL_PWM_ClearInterrupt(p_pwm->CONFIG.P_HAL_PWM); }
static inline void PWM_DisableInterrupt(const PWM_T * p_pwm) 	{ HAL_PWM_DisableInterrupt(p_pwm->CONFIG.P_HAL_PWM); }
static inline void PWM_EnableInterrupt(const PWM_T * p_pwm) 	{ HAL_PWM_EnableInterrupt(p_pwm->CONFIG.P_HAL_PWM); }
/* If multiple PWMs share a register. may interfere with sync */
static inline void PWM_ActuateSync(const PWM_T * p_pwm) 		{ HAL_PWM_SyncModule(p_pwm->CONFIG.P_HAL_PWM); }

/*
	Extern
*/
extern void PWM_Init(PWM_T * p_pwm);
extern void PWM_InitChannel(PWM_T * p_pwm);
extern void PWM_InitModule(PWM_T * p_pwm);

#endif

/*
	Use buffered data
*/

/*
	Buffered data
*/
//	uint32_t Duty_Ticks;
//	bool IsOn;
//	bool IsInvertPolarity;

 //static inline void PWM_SetDuty(PWM_T * p_pwm, uint32_t pwmDuty)
 //{
 //	p_pwm->Duty_Ticks = _PWM_CalcPwmDutyTicks(p_pwm, pwmDuty);
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

/* Although PWM_Module encompass many PWM Channels, treat as a subset of functions */
// typedef const struct PWM_Module_Config_Tag
// {
// 	HAL_PWM_T * const P_HAL_PWM;
// }
// PWM_Module_Config_T;
// typedef struct
// {
// 	const PWM_Module_Config_T CONFIG;
// 	uint32_t SyncBuffer;
// }
// PWM_Module_T;

// static inline void PWM_Module_ClearInterrupt(const PWM_Module_T * p_pwm) 		{ HAL_PWM_ClearInterrupt(p_pwm->CONFIG.P_HAL_PWM); }
// static inline void PWM_Module_DisableInterrupt(const PWM_Module_T * p_pwm) 		{ HAL_PWM_DisableInterrupt(p_pwm->CONFIG.P_HAL_PWM); }
// static inline void PWM_Module_EnableInterrupt(const PWM_Module_T * p_pwm) 		{ HAL_PWM_EnableInterrupt(p_pwm->CONFIG.P_HAL_PWM); }

// // static inline void PWM_SetEnable(PWM_T * p_pwm) { p_pwm->IsOn = true; }
// // static inline void PWM_SetDisable(PWM_T * p_pwm) { p_pwm->IsOn = false; }
// static inline void PWM_Module_EnableChannel(PWM_Module_T * p_pwm, const PWM_T * p_channel) 			{ p_pwm->SyncBuffer |= p_channel->CONFIG.CHANNEL_MASK; }
// static inline void PWM_Module_DisableChannel(PWM_Module_T * p_pwm, const PWM_T * p_channel) 		{ p_pwm->SyncBuffer &= ~p_channel->CONFIG.CHANNEL_MASK; }
// static inline void PWM_Module_EnableInvertPolarity(PWM_Module_T * p_pwm, const PWM_T * p_channel) 	{ p_pwm->SyncBuffer |= p_channel->CONFIG.CHANNEL_MASK; }
// static inline void PWM_Module_DisableInvertPolarity(PWM_Module_T * p_pwm, const PWM_T * p_channel) 	{ p_pwm->SyncBuffer &= ~p_channel->CONFIG.CHANNEL_MASK; }

// static inline void PWM_Module_ClearSync(PWM_Module_T * p_pwm) 						{ p_pwm->SyncBuffer = 0U; }
// static inline void PWM_Module_EnableChannels(const PWM_Module_T * p_pwm) 			{ HAL_PWM_EnableOutput(p_pwm->CONFIG.P_HAL_PWM, p_pwm->SyncBuffer); }
// static inline void PWM_Module_DisableChannels(const PWM_Module_T * p_pwm) 			{ HAL_PWM_DisableOutput(p_pwm->CONFIG.P_HAL_PWM, p_pwm->SyncBuffer); }
// static inline void PWM_Module_EnableInvertPolarity(const PWM_Module_T * p_pwm) 		{ HAL_PWM_EnableInvertPolarity(p_pwm->CONFIG.P_HAL_PWM, p_pwm->SyncBuffer); }
// static inline void PWM_Module_DisableInvertPolarity(const PWM_Module_T * p_pwm) 	{ HAL_PWM_DisableInvertPolarity(p_pwm->CONFIG.P_HAL_PWM, p_pwm->SyncBuffer); }
// static inline void PWM_Module_ActuateSync(const PWM_Module_T * p_pwm) 				{ HAL_PWM_SyncModule(p_pwm->CONFIG.P_HAL_PWM); }


// static inline void PWM_Module_EnableChannels(const PWM_Module_T * p_pwm, const PWM_T * pp_channels, uint8_t channelCount)
// {
// 	uint32_t buffer;
// 	for(uint8_t iChannel = 0U; iChannel < channelCount; iChannel++)
// 	{
// 		buffer |= pp_channels[channelCount].CONFIG.CHANNEL_MASK;
// 	}
// 	HAL_PWM_EnableOutput(p_pwm->CONFIG.P_HAL_PWM, buffer);
// }
// static inline void PWM_Module_ActuateSync(const PWM_Module_T * p_pwm) { HAL_PWM_SyncModule(p_pwm->CONFIG.P_HAL_PWM); }