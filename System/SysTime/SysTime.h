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
	@file 	SysTime.h
	@author FireSoucery
	@brief  Init SysTick as System time, 1ms,  reference
	@version V0
*/
/******************************************************************************/
#ifndef SYSTIME_H
#define SYSTIME_H

#include "Config.h"
#include <stdint.h>

#ifdef CONFIG_SYSTIME_SYSTICK
#define SYST_CSR 	(*((uint32_t *)0xE000E010))		/* SysTick Control and Status Register */
#define SYST_RVR 	(*((uint32_t *)0xE000E014))		/* SysTick Reload Value Register */
#define SYST_CVR 	(*((uint32_t *)0xE000E018))		/* SysTick Current Value Register */
#define SYST_CALIB 	(*((uint32_t *)0xE000E01C))		/* SysTick Calibration Value Register */
#define SCB_SHPR3 	(*((uint32_t *)0xE000ED20))		/* System Handler Priority Register 3 */

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

/*
	MISRA Violation
*/
extern volatile uint32_t SysTime_Millis;

/*
	User Map to 1ms ISR
*/
static inline void SysTime_CaptureMillis_ISR(void) 	{ SysTime_Millis++; }
static inline uint32_t SysTime_GetMillis(void) 		{ return SysTime_Millis; }
static inline uint32_t Millis(void)  				{ return SysTime_GetMillis(); }

#if defined(CONFIG_SYSTIME_SYSTICK) && defined(CPU_FREQ)
static inline uint32_t SysTime_GetMicros(void)
{
	volatile uint32_t micros = (CPU_FREQ / 1000U - 1U) - (SYST_CVR / (CPU_FREQ / 1000000U)); /* SYST_CVR ticks down */
	return SysTime_Millis * 1000U + micros;
}

static inline uint32_t Micros(void) { return SysTime_GetMicros(); }
#endif

extern void SysTime_Init(void);

#endif /* SYSTIME_H */
