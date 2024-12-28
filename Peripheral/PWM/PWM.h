/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   PWM.h
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

typedef const struct PWM_Const
{
    // HAL_PWM_Module_T * const P_HAL_PWM_MODULE;
    HAL_PWM_T * const P_HAL_PWM;
    const uint32_t CHANNEL_ID;
    const uint32_t PERIOD_TICKS;
}
PWM_Const_T;

typedef struct PWM
{
    const PWM_Const_T CONST;
}
PWM_T;

#define PWM_INIT(p_Hal, Peroid_Ticks, Channel)  { .CONST = { .P_HAL_PWM = p_Hal, .PERIOD_TICKS = Peroid_Ticks, .CHANNEL_ID = Channel }, }
#define PWM_MODULE_INIT(p_Hal, Peroid_Ticks)    { .CONST = { .P_HAL_PWM = p_Hal, .PERIOD_TICKS = Peroid_Ticks }, }

#ifndef PWM_DUTY_MAX
    #define PWM_DUTY_MAX (65536U)
#endif
/*
    Channel
*/
static inline uint32_t _PWM_DutyTicksOf(const PWM_T * p_pwm, uint32_t duty) { return p_pwm->CONST.PERIOD_TICKS * duty / PWM_DUTY_MAX; }

/*
    Actuate arguments immediately, unless use sync is enabled
*/
static inline void PWM_ActuateDuty_Ticks(const PWM_T * p_pwm, uint32_t pwmDuty_Ticks) { HAL_PWM_WriteDuty(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID, pwmDuty_Ticks); }

/* where PWM_DUTY_MAX is 100% duty */
static inline void PWM_ActuateDuty(const PWM_T * p_pwm, uint32_t pwmDuty)           { PWM_ActuateDuty_Ticks(p_pwm, _PWM_DutyTicksOf(p_pwm, pwmDuty)); }
static inline void PWM_ActuateDutyMidPlus(const PWM_T * p_pwm, uint32_t pwmDuty)    { PWM_ActuateDuty_Ticks(p_pwm, (p_pwm->CONST.PERIOD_TICKS + _PWM_DutyTicksOf(p_pwm, pwmDuty)) / 2U); }
static inline void PWM_ActuateDutyMidMinus(const PWM_T * p_pwm, uint32_t pwmDuty)   { PWM_ActuateDuty_Ticks(p_pwm, (p_pwm->CONST.PERIOD_TICKS - _PWM_DutyTicksOf(p_pwm, pwmDuty)) / 2U); }

static inline void PWM_ActuateDuty_Percent16(const PWM_T * p_pwm, uint16_t percent16)   { PWM_ActuateDuty_Ticks(p_pwm, (uint32_t)percent16 * p_pwm->CONST.PERIOD_TICKS >> 16U); }
static inline void PWM_ActuateDuty_Fract16(const PWM_T * p_pwm, uint16_t fract16)       { PWM_ActuateDuty_Ticks(p_pwm, (uint32_t)fract16 * p_pwm->CONST.PERIOD_TICKS >> 15U); }

static inline void PWM_Enable(const PWM_T * p_pwm)                  { HAL_PWM_EnableOutput(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID); }
static inline void PWM_Disable(const PWM_T * p_pwm)                 { HAL_PWM_DisableOutput(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID); }
static inline void PWM_EnableInvertPolarity(const PWM_T * p_pwm)    { HAL_PWM_EnableInvertPolarity(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID); }
static inline void PWM_DisableInvertPolarity(const PWM_T * p_pwm)   { HAL_PWM_DisableInvertPolarity(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID); }

/*
    Module
*/
/* Shared interrupt */
static inline void PWM_ClearInterrupt(const PWM_T * p_pwm)      { HAL_PWM_ClearInterrupt(p_pwm->CONST.P_HAL_PWM); }
static inline void PWM_DisableInterrupt(const PWM_T * p_pwm)    { HAL_PWM_DisableInterrupt(p_pwm->CONST.P_HAL_PWM); }
static inline void PWM_EnableInterrupt(const PWM_T * p_pwm)     { HAL_PWM_EnableInterrupt(p_pwm->CONST.P_HAL_PWM); }
/* If multiple PWMs share a register. may interfere with sync. todo buffered module sync */
static inline void PWM_ActuateSync(const PWM_T * p_pwm)         { HAL_PWM_SyncModule(p_pwm->CONST.P_HAL_PWM); }

/*
    Extern
*/
extern void PWM_Init(PWM_T * p_pwm);
extern void PWM_InitChannel(PWM_T * p_pwm);
extern void PWM_InitModule(PWM_T * p_pwm);

#endif

