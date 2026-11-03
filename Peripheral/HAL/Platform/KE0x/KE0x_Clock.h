#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   HAL_KE0x.h
    @author FireSourcery
    @brief  Platform init, called by the Board. Clock, register access.
*/
/******************************************************************************/
#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    SIM. Core and timer at ICSOUTCLK, bus at half.
    The same values as the SDK's safe dividers, so written once, ahead of the ICS switch.
*/
/******************************************************************************/
static inline void KE0x_SIM_InitClockDivs(void)
{
#if     defined(KE06Z4_SERIES)
    SIM->CLKDIV = SIM_CLKDIV_OUTDIV1(0U) | SIM_CLKDIV_OUTDIV2(1U) | SIM_CLKDIV_OUTDIV3(0U);
#elif   defined(KE02Z4_SERIES)
    SIM->BUSDIV = SIM_BUSDIV_BUSDIV(1U);
#endif
}

/* Core, bus, and timer at ICSOUTCLK. Written after the ICS switch */
static inline void KE0x_SIM_InitClockDivsUnity(void)
{
#if     defined(KE06Z4_SERIES)
    SIM->CLKDIV = SIM_CLKDIV_OUTDIV1(0U) | SIM_CLKDIV_OUTDIV2(0U) | SIM_CLKDIV_OUTDIV3(0U);
#elif   defined(KE02Z4_SERIES)
    SIM->BUSDIV = SIM_BUSDIV_BUSDIV(0U);
#endif
}

/******************************************************************************/
/*
    OSC. Crystal, high gain, high range, 4 MHz and up.
*/
/******************************************************************************/
static inline void KE0x_OSC_InitHighGain(void)
{
    OSC->CR = OSC_CR_OSCEN_MASK | OSC_CR_OSCOS_MASK | OSC_CR_HGO_MASK | OSC_CR_RANGE_MASK;
    while ((OSC->CR & OSC_CR_OSCINIT_MASK) == 0U) {}
}

/******************************************************************************/
/*
    ICS. The FLL multiplies its reference, 31.25 to 39.0625 kHz: KE06 by 1280, KE02 by 1024.
*/
/******************************************************************************/
/* FLL reference, the slow internal reference clock */
static inline void KE0x_ICS_SelectFllRefInternal(void)
{
    ICS->C1 |= ICS_C1_IREFS_MASK;
    while ((ICS->S & ICS_S_IREFST_MASK) == 0U) {}
}

/* FLL reference, the OSC divided by (32 << rDiv) at high range. 10 MHz, rDiv 3: 39.0625 kHz */
static inline void KE0x_ICS_SelectFllRefExternal(uint8_t rDiv)
{
    ICS->C1 = (ICS->C1 & ~(ICS_C1_RDIV_MASK | ICS_C1_IREFS_MASK)) | ICS_C1_RDIV(rDiv);
    while ((ICS->S & ICS_S_IREFST_MASK) != 0U) {}
}

/* ICSOUTCLK, the FLL divided by (1 << bDiv). Set after the reference settles, per the SDK's note on IREFST */
static inline void KE0x_ICS_SelectOutFll(uint8_t bDiv)
{
    ICS->C1 &= ~ICS_C1_CLKS_MASK;
    ICS->C2 = (ICS->C2 & ~ICS_C2_BDIV_MASK) | ICS_C2_BDIV(bDiv);
    while ((ICS->S & ICS_S_CLKST_MASK) != 0U) {}
}

/* Then clears the loss of lock flag, write 1 to clear */
static inline void KE0x_ICS_WaitFllLock(void)
{
    while ((ICS->S & ICS_S_LOCK_MASK) == 0U) {}
    ICS->S = ICS_S_LOLS_MASK;
}

/* ICSIRCLK off, in run and stop */
static inline void KE0x_ICS_DisableIrclk(void)
{
    ICS->C1 &= ~(ICS_C1_IRCLKEN_MASK | ICS_C1_IREFSTEN_MASK);
}

/******************************************************************************/
/*
    Boot clock. Core at the FLL, bus at half.
*/
/******************************************************************************/
/* FEE, FLL Engaged External. KE06 rDiv 3: crystal x 5 */
static inline void _KE0x_InitClockExternal(uint8_t rDiv)
{
    KE0x_SIM_InitClockDivs();
    KE0x_OSC_InitHighGain();
    KE0x_ICS_SelectFllRefExternal(rDiv);
    KE0x_ICS_SelectOutFll(0U);
    KE0x_ICS_WaitFllLock();
}

static inline void KE0x_InitClockExternal(void)
{
    _KE0x_InitClockExternal(3U);
}

/* FEI, FLL Engaged Internal. KE06 37.5 kHz: 48 MHz */
static inline void KE0x_InitClockInternal(void)
{
    KE0x_SIM_InitClockDivs();
    KE0x_ICS_SelectFllRefInternal();
    KE0x_ICS_SelectOutFll(0U);
    KE0x_ICS_WaitFllLock();
}


/******************************************************************************/
/*
    Reset clock, HAL_ResetClock's state. FEI, ICSOUTCLK at the FLL / 2, core and bus at ICSOUTCLK. KE06: 24 MHz.
    The bus stays at half until ICSOUTCLK is down to 24 MHz.
*/
/******************************************************************************/
static inline void KE0x_InitClockReset(void)
{
    KE0x_SIM_InitClockDivs();
    KE0x_ICS_SelectFllRefInternal();
    KE0x_ICS_SelectOutFll(1U);
    KE0x_ICS_WaitFllLock();
    KE0x_ICS_DisableIrclk();
    KE0x_SIM_InitClockDivsUnity();
}

/******************************************************************************/
/*
    Boot clock, the calls inlined
*/
/******************************************************************************/
// static inline void KE0x_InitClockExternal_Flat(uint8_t rDiv)
// {
//     /* SIM. Core and timer at ICSOUTCLK, bus at half */
// #if     defined(KE06Z4_SERIES)
//     SIM->CLKDIV = SIM_CLKDIV_OUTDIV1(0U) | SIM_CLKDIV_OUTDIV2(1U) | SIM_CLKDIV_OUTDIV3(0U);
// #elif   defined(KE02Z4_SERIES)
//     SIM->BUSDIV = SIM_BUSDIV_BUSDIV(1U);
// #endif

//     /* OSC. Crystal, high gain, high range */
//     OSC->CR = OSC_CR_OSCEN_MASK | OSC_CR_OSCOS_MASK | OSC_CR_HGO_MASK | OSC_CR_RANGE_MASK;
//     while ((OSC->CR & OSC_CR_OSCINIT_MASK) == 0U) {}

//     /* FLL reference, the OSC divided by (32 << rDiv) */
//     ICS->C1 = (ICS->C1 & ~(ICS_C1_RDIV_MASK | ICS_C1_IREFS_MASK)) | ICS_C1_RDIV(rDiv);
//     while ((ICS->S & ICS_S_IREFST_MASK) != 0U) {}

//     /* ICSOUTCLK, the FLL divided by 1 */
//     ICS->C1 &= ~ICS_C1_CLKS_MASK;
//     ICS->C2 = (ICS->C2 & ~ICS_C2_BDIV_MASK) | ICS_C2_BDIV(0U);
//     while ((ICS->S & ICS_S_CLKST_MASK) != 0U) {}

//     /* FLL lock, then clear the loss of lock flag */
//     while ((ICS->S & ICS_S_LOCK_MASK) == 0U) {}
//     ICS->S = ICS_S_LOLS_MASK;
// }

// static inline void KE0x_InitClockInternal_Flat(void)
// {
//     /* SIM. Core and timer at ICSOUTCLK, bus at half */
// #if     defined(KE06Z4_SERIES)
//     SIM->CLKDIV = SIM_CLKDIV_OUTDIV1(0U) | SIM_CLKDIV_OUTDIV2(1U) | SIM_CLKDIV_OUTDIV3(0U);
// #elif   defined(KE02Z4_SERIES)
//     SIM->BUSDIV = SIM_BUSDIV_BUSDIV(1U);
// #endif

//     /* FLL reference, the slow internal reference clock */
//     ICS->C1 |= ICS_C1_IREFS_MASK;
//     while ((ICS->S & ICS_S_IREFST_MASK) == 0U) {}

//     /* ICSOUTCLK, the FLL divided by 1 */
//     ICS->C1 &= ~ICS_C1_CLKS_MASK;
//     ICS->C2 = (ICS->C2 & ~ICS_C2_BDIV_MASK) | ICS_C2_BDIV(0U);
//     while ((ICS->S & ICS_S_CLKST_MASK) != 0U) {}

//     /* FLL lock, then clear the loss of lock flag */
//     while ((ICS->S & ICS_S_LOCK_MASK) == 0U) {}
//     ICS->S = ICS_S_LOLS_MASK;
// }