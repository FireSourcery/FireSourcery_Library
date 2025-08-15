#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   SysTime.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>

#if defined(CPU_FREQ)
#else
#error "SysTime - Undefined CPU_FREQ"
#endif

#ifdef SYSTIME_SYSTICK
#else
#error "SysTime - Undefined SYSTIME_HAL"
#endif

#ifdef SYSTIME_SYSTICK
#define SYST_CSR    (*((uint32_t *)0xE000E010))        /* SysTick Control and Status Register */
#define SYST_RVR    (*((uint32_t *)0xE000E014))        /* SysTick Reload Value Register */
#define SYST_CVR    (*((uint32_t *)0xE000E018))        /* SysTick Current Value Register */
#define SYST_CALIB  (*((uint32_t *)0xE000E01C))        /* SysTick Calibration Value Register */
#define SCB_SHPR3   (*((uint32_t *)0xE000ED20))        /* System Handler Priority Register 3 */

#define SYST_CSR_ENABLE_MASK     0x1U
#define SYST_CSR_ENABLE_SHIFT    0U
#define SYST_CSR_ENABLE_WIDTH    1U
#define SYST_CSR_ENABLE(x)       (((uint32_t)(((uint32_t)(x))<<SYST_CSR_ENABLE_SHIFT))&SYST_CSR_ENABLE_MASK)
#define SYST_CSR_TICKINT_MASK    0x2U
#define SYST_CSR_TICKINT_SHIFT   1U
#define SYST_CSR_TICKINT_WIDTH   1U
#define SYST_CSR_TICKINT(x)      (((uint32_t)(((uint32_t)(x))<<SYST_CSR_TICKINT_SHIFT))&SYST_CSR_TICKINT_MASK)
#define SYST_CSR_CLKSOURCE_MASK  0x4U
#define SCB_SHPR3_PRI_15_MASK    0xFF000000U
#define SCB_SHPR3_PRI_15_SHIFT   24U
#define SCB_SHPR3_PRI_15_WIDTH   8U
#define SCB_SHPR3_PRI_15(x)      (((uint32_t)(((uint32_t)(x))<<SCB_SHPR3_PRI_15_SHIFT))&SCB_SHPR3_PRI_15_MASK)

#define SYST_COUNT() (SYST_CVR)
#endif

#if defined(SYSTIME_SYSTICK) && defined(CPU_FREQ)
/* SYST_CVR ticks down */
static inline uint32_t SysTime_GetTicks(void) { return (uint32_t)((CPU_FREQ / 1000U - 1U) - SYST_CVR); }
#endif

#define MILLIS_TIMER_FREQ (1000U)

extern volatile uint32_t SysTime_Millis;

/*
    App maps to 1ms ISR
*/
static inline void SysTime_CaptureMillis_ISR(void)  { SysTime_Millis++; }

static inline uint32_t SysTime_GetMillis(void)      { return SysTime_Millis; }
static inline uint32_t Millis(void)                 { return SysTime_GetMillis(); }

static inline void SysTime_ZeroMillis(void)         { SysTime_Millis = 0U; }

/*
    Tick/Micros
*/
#define MICROS_PER_TICK_SHIFT 16U
#define MICROS_PER_TICK ((uint32_t)(((uint64_t)1000000ULL << MICROS_PER_TICK_SHIFT) / CPU_FREQ))

/* us within current ms = ticksElapsed * (1e6 / CPU_FREQ). Wraps every 1000 */
static inline uint32_t SysTime_GetMicros(void) { return ((SysTime_GetTicks() * MICROS_PER_TICK) >> MICROS_PER_TICK_SHIFT); }

static inline uint32_t Micros(void) { return SysTime_GetMicros(); }

extern void SysTime_Init(void);
