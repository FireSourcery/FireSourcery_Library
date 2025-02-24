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
    @file     HAL_PWM.h
    @author
    @brief      PWM HAL for KE0x
    @version    V0
*/
/******************************************************************************/
#ifndef HAL_PWM_PLATFORM_H
#define HAL_PWM_PLATFORM_H

#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef HAL_PWM_CLOCK_SOURCE_FREQ
#define HAL_PWM_CLOCK_SOURCE_FREQ (CPU_FREQ / 2U) /* Divided 2 for center align mode */
#endif

typedef FTM_Type HAL_PWM_T;
typedef FTM_Type HAL_PWM_Module_T;

/*
    Channel
*/
static inline void HAL_PWM_WriteDuty(HAL_PWM_T * p_hal, uint32_t channel, uint32_t duty)    { p_hal->CONTROLS[channel].CnV = duty; }
static inline uint32_t HAL_PWM_ReadDuty(HAL_PWM_T * p_hal, uint32_t channel)                { return p_hal->CONTROLS[channel].CnV; }
static inline void HAL_PWM_EnableOutput(HAL_PWM_T * p_hal, uint32_t channel)                { p_hal->OUTMASK &= ~(1UL << channel); }
static inline void HAL_PWM_DisableOutput(HAL_PWM_T * p_hal, uint32_t channel)               { p_hal->OUTMASK |= (1UL << channel); }
static inline bool HAL_PWM_ReadOutputState(HAL_PWM_T * p_hal, uint32_t channel)             { p_hal->OUTMASK & (1UL << channel); }
static inline void HAL_PWM_EnableInvertPolarity(HAL_PWM_T * p_hal, uint32_t channel)        { p_hal->POL |= (1UL << channel); }
static inline void HAL_PWM_DisableInvertPolarity(HAL_PWM_T * p_hal, uint32_t channel)       { p_hal->POL &= ~(1UL << channel); }

static inline void HAL_PWM_EnableSoftwareControl(HAL_PWM_T * p_hal, uint32_t channel)   { p_hal->SWOCTRL |= 1UL << channel; }
static inline void HAL_PWM_DisableSoftwareControl(HAL_PWM_T * p_hal, uint32_t channel)  { p_hal->SWOCTRL &= ~(1UL << channel); }
static inline void HAL_PWM_WriteHigh(HAL_PWM_T * p_hal, uint32_t channel)               { p_hal->SWOCTRL |= (1UL << (channel + FTM_SWOCTRL_CH0OCV_SHIFT)); }
static inline void HAL_PWM_WriteLow(HAL_PWM_T * p_hal, uint32_t channel)                { p_hal->SWOCTRL &= ~(1UL << (channel + FTM_SWOCTRL_CH0OCV_SHIFT)); }

static inline void HAL_PWM_InitChannel(HAL_PWM_T * p_hal, uint32_t channel)
{
    p_hal->CONTROLS[channel].CnSC = FTM_CnSC_ELSB_MASK; /* High-true pulses (clear Output on match-up) */
    HAL_PWM_EnableOutput(p_hal, channel);
}

/*
    Module Common
    HAL as module wide control
*/
/* Read-after-write sequence to guarantee required serialization of memory operations */
static inline void HAL_PWM_ClearInterrupt(HAL_PWM_Module_T * p_hal)    { p_hal->SC &= ~FTM_SC_TOF_MASK; p_hal->SC; }
static inline void HAL_PWM_DisableInterrupt(HAL_PWM_Module_T * p_hal)  { p_hal->SC &= ~FTM_SC_TOIE_MASK; }
static inline void HAL_PWM_EnableInterrupt(HAL_PWM_Module_T * p_hal)   { p_hal->SC |= FTM_SC_TOIE_MASK; }

/* Sync update buffered values, via config select in init. */
static inline void HAL_PWM_SyncModule(HAL_PWM_Module_T * p_hal) { p_hal->SYNC |= FTM_SYNC_SWSYNC_MASK; }

/* update buffer and actuate sync update */
/* Duty may write channels individually then sync */
static inline void HAL_PWM_SyncModuleDuty(HAL_PWM_Module_T * p_hal, uint32_t channels) { (void)channels; p_hal->SYNC |= FTM_SYNC_SWSYNC_MASK; }
/* Write full register only, if sync is enabled */
static inline void HAL_PWM_SyncModuleInvert(HAL_PWM_Module_T * p_hal, uint32_t channels)        { /* p_hal->POL |= channels; */ }
static inline void HAL_PWM_SyncModuleOutputState(HAL_PWM_Module_T * p_hal, uint32_t channels)   { /* p_hal->OUTMASK &= ~channels; */ }

/* Caller accounts for factor of 2 in up down mode */
static inline void HAL_PWM_InitModulePeriod(HAL_PWM_Module_T * p_hal, uint32_t ticks)
{
    p_hal->SC &= ~FTM_SC_CLKS_MASK;
    p_hal->MOD = FTM_MOD_MOD(ticks);
    p_hal->SC |= FTM_SC_CLKS(0x01U);
}

// static inline void HAL_PWM_InitModuleDuty_Freq(HAL_PWM_Module_T * p_hal, uint32_t freq)
static inline void HAL_PWM_InitModuleFreq(HAL_PWM_Module_T * p_hal, uint32_t freq)
{
    p_hal->SC &= ~FTM_SC_CLKS_MASK;
    p_hal->MOD = FTM_MOD_MOD(HAL_PWM_CLOCK_SOURCE_FREQ / freq);
    p_hal->SC |= FTM_SC_CLKS(0x01U);
}

/*
    Init Module
    Center aligned mode by default
    To use sync with shared channel registers, buffer value, HAL_WriteModule register channel values simultaneously
*/
static inline void HAL_PWM_InitModule(HAL_PWM_Module_T * p_hal)
{
    p_hal->MODE         = FTM_MODE_WPDIS_MASK | FTM_MODE_FTMEN_MASK; /* Enable FTM mode */
    p_hal->COMBINE      = FTM_COMBINE_SYNCEN0_MASK | FTM_COMBINE_SYNCEN1_MASK | FTM_COMBINE_SYNCEN2_MASK | FTM_COMBINE_SYNCEN3_MASK;    /* All channel sync set to enable. */
    p_hal->SYNC         = FTM_SYNC_SWSYNC_MASK | FTM_SYNC_CNTMIN_MASK | FTM_SYNC_CNTMAX_MASK; // | FTM_SYNC_SYNCHOM_MASK;
    p_hal->SYNCONF      = FTM_SYNCONF_SYNCMODE_MASK | FTM_SYNCONF_SWWRBUF_MASK;
        /* Sync of channels other than duty cannot be written with iterative |= */
            // | FTM_SYNCONF_SWRSTCNT_MASK //| FTM_SYNCONF_INVC_MASK  // | FTM_SYNCONF_SWOC_MASK | FTM_SYNCONF_SWINVC_MASK | FTM_SYNCONF_SWOM_MASK| FTM_SYNCONF_SWSOC_MASK;;
    p_hal->OUTMASK      = FTM_OUTMASK_CH0OM_MASK | FTM_OUTMASK_CH1OM_MASK | FTM_OUTMASK_CH2OM_MASK | FTM_OUTMASK_CH3OM_MASK | FTM_OUTMASK_CH4OM_MASK | FTM_OUTMASK_CH5OM_MASK | FTM_OUTMASK_CH6OM_MASK | FTM_OUTMASK_CH7OM_MASK;
    p_hal->EXTTRIG      = FTM_EXTTRIG_INITTRIGEN_MASK;
    p_hal->CONF         = FTM_CONF_BDMMODE(0x03U);
    p_hal->SC           = FTM_SC_CPWMS_MASK | FTM_SC_TOIE_MASK;
    p_hal->SC           |= FTM_SC_CLKS(0x01U);
}

#endif
