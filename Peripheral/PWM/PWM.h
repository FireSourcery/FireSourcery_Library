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

/******************************************************************************/
/*!
    @brief  Implementation of PWM Module and Channel
            With Channel as primary interface, and Module as shared component.
*/
/******************************************************************************/

typedef struct PWM_Module
{
    const struct
    {
        HAL_PWM_Module_T * const P_HAL_PWM_MODULE;
        const uint32_t PERIOD_TICKS;
        const uint32_t * P_CHANNELS; /* Map index to ChannelId/Mask */
        // const uint8_t CHANNELS_COUNT;
        // const uint32_t CHANNELS_MASK; /* combined */
        // const uint32_t * P_STATE;
    };
}
PWM_Module_T;

#define PWM_MODULE_INIT(p_Hal, Peroid_Ticks, ...) \
{ \
    .P_HAL_PWM_MODULE   = p_Hal,                                                        \
    .PERIOD_TICKS       = Peroid_Ticks,                                                 \
    .P_CHANNELS         = (const uint32_t[]){ __VA_ARGS__ },                            \
}
// .CHANNELS_COUNT     = sizeof((const uint32_t[]){ __VA_ARGS__ }) / sizeof(uint32_t), \

typedef const struct PWM_Const
{
    HAL_PWM_T * const P_HAL_PWM;
    const uint32_t CHANNEL_ID;
    const uint32_t PERIOD_TICKS;
    // PWM_Module_T * const P_PWM_MODULE;
}
PWM_Const_T;

typedef struct PWM
{
    const PWM_Const_T CONST;
}
PWM_T;

#define PWM_INIT(p_Hal, Peroid_Ticks, Channel) { .CONST = { .P_HAL_PWM = p_Hal, .PERIOD_TICKS = Peroid_Ticks, .CHANNEL_ID = Channel }, }

#ifndef PWM_DUTY_MAX
    #define PWM_DUTY_MAX (32768U)
#endif

/* where PWM_DUTY_MAX is 100% duty */
static inline uint32_t _PWM_DutyTicksOf(const PWM_T * p_pwm, uint32_t duty) { return p_pwm->CONST.PERIOD_TICKS * duty / PWM_DUTY_MAX; }

static inline uint32_t _PWM_TicksOfPercent16(const PWM_T * p_pwm, uint16_t percent16) { return p_pwm->CONST.PERIOD_TICKS * percent16 >> 16U; }
static inline uint32_t _PWM_TicksOfFract16(const PWM_T * p_pwm, uint16_t fract16) { return p_pwm->CONST.PERIOD_TICKS * fract16 >> 15U; }

/*
    Channel
*/
/*
    Actuate arguments immediately, unless use sync is enabled
*/
static inline uint32_t PWM_ReadDuty_Ticks(const PWM_T * p_pwm) { return HAL_PWM_ReadDuty(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID); }
static inline void PWM_WriteDuty_Ticks(const PWM_T * p_pwm, uint32_t duty_Ticks) { HAL_PWM_WriteDuty(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID, duty_Ticks); }

static inline void PWM_WriteDuty_Percent16(const PWM_T * p_pwm, uint16_t percent16)   { PWM_WriteDuty_Ticks(p_pwm, _PWM_TicksOfPercent16(p_pwm, percent16)); }
static inline void PWM_WriteDuty_Fract16(const PWM_T * p_pwm, uint16_t fract16)       { PWM_WriteDuty_Ticks(p_pwm, _PWM_TicksOfFract16(p_pwm, fract16)); }

static inline void PWM_WriteDuty(const PWM_T * p_pwm, uint32_t duty)             { PWM_WriteDuty_Ticks(p_pwm, _PWM_DutyTicksOf(p_pwm, duty)); }
static inline void PWM_ActuateDutyMidPlus(const PWM_T * p_pwm, uint32_t duty)    { PWM_WriteDuty_Ticks(p_pwm, (p_pwm->CONST.PERIOD_TICKS + _PWM_DutyTicksOf(p_pwm, duty)) / 2U); }
static inline void PWM_ActuateDutyMidMinus(const PWM_T * p_pwm, uint32_t duty)   { PWM_WriteDuty_Ticks(p_pwm, (p_pwm->CONST.PERIOD_TICKS - _PWM_DutyTicksOf(p_pwm, duty)) / 2U); }

static inline void PWM_Enable(const PWM_T * p_pwm)                  { HAL_PWM_EnableOutput(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID); }
static inline void PWM_Disable(const PWM_T * p_pwm)                 { HAL_PWM_DisableOutput(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID); }
static inline void PWM_EnableInvertPolarity(const PWM_T * p_pwm)    { HAL_PWM_EnableInvertPolarity(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID); }
static inline void PWM_DisableInvertPolarity(const PWM_T * p_pwm)   { HAL_PWM_DisableInvertPolarity(p_pwm->CONST.P_HAL_PWM, p_pwm->CONST.CHANNEL_ID); }

static inline uint32_t _PWM_ChannelMaskOf(const PWM_T * p_pwm, bool isOn) { return isOn ? p_pwm->CONST.CHANNEL_ID : 0U; }

/*
    Module
*/
/* Shared interrupt */
static inline void PWM_ClearInterrupt(const PWM_Module_T * p_pwm) { HAL_PWM_ClearInterrupt(p_pwm->P_HAL_PWM_MODULE); }
static inline void PWM_DisableInterrupt(const PWM_Module_T * p_pwm) { HAL_PWM_DisableInterrupt(p_pwm->P_HAL_PWM_MODULE); }
static inline void PWM_EnableInterrupt(const PWM_Module_T * p_pwm) { HAL_PWM_EnableInterrupt(p_pwm->P_HAL_PWM_MODULE); }

/*
    Sync update buffered values via Module.
*/
/*
    actuates buffered values. all configured values.
*/
static inline void PWM_ActuateSync(const PWM_Module_T * p_pwm) { HAL_PWM_SyncModule(p_pwm->P_HAL_PWM_MODULE); }

/*
    individual buffers, if collective configuration is not supported.
*/
static inline void _PWM_Module_WriteSyncDuty(const PWM_Module_T * p_pwm, uint32_t channelMask) { HAL_PWM_SyncModuleDuty(p_pwm->P_HAL_PWM_MODULE, channelMask); }
static inline void _PWM_Module_WriteSyncInvert(const PWM_Module_T * p_pwm, uint32_t channelMask) { HAL_PWM_SyncModuleInvert(p_pwm->P_HAL_PWM_MODULE, channelMask); }
static inline void _PWM_Module_WriteSyncOnOff(const PWM_Module_T * p_pwm, uint32_t channelMask) { HAL_PWM_SyncModuleOutputState(p_pwm->P_HAL_PWM_MODULE, channelMask); }

// static inline uint32_t _PWM_Module_ChannelMaskOf(const PWM_Module_T * p_pwm, uint8_t channelBits)
// {
//     uint32_t mask = 0;
//     for (uint8_t index = 0; index < p_pwm->CHANNELS_COUNT; index++)
//     {
//         if (channelBits & (1U << index)) { mask |= p_pwm->P_CHANNELS[index]; }
//     }
//     return mask;
// }

// static inline void PWM_Module_WriteSyncDuty(const PWM_Module_T * p_pwm, uint8_t channels)
// {
//     _PWM_Module_WriteSyncDuty(p_pwm, _PWM_Module_ChannelMaskOf(p_pwm, channels));
// }

// static inline void PWM_Module_WriteSyncInvert(const PWM_Module_T * p_pwm, uint8_t channels)
// {
//     _PWM_Module_WriteSyncInvert(p_pwm, _PWM_Module_ChannelMaskOf(p_pwm, channels));
// }

// static inline void PWM_Module_WriteSyncOnOff(const PWM_Module_T * p_pwm, uint8_t channels)
// {
//     _PWM_Module_WriteSyncOnOff(p_pwm, _PWM_Module_ChannelMaskOf(p_pwm, channels));
// }


/*
    Extern
*/
extern void PWM_Init(PWM_T * p_pwm);
extern void PWM_Channel_Init(PWM_T * p_pwm);
extern void PWM_Module_Init(PWM_Module_T * p_pwm);

#endif

