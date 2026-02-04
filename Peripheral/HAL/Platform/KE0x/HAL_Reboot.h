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
    @file    .h
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#ifndef HAL_REBOOT_PLATFORM_H
#define HAL_REBOOT_PLATFORM_H

#include "KE0x.h"
#include "fsl_clock.h"

#include <stdint.h>
#include <stdbool.h>

static inline void HAL_ResetClock()
{
    const ics_config_t icsConfig_BOARD_BootClockRUN =
    {
        .icsMode = kICS_ModeFEI,                  /* FEI - FLL Engaged Internal */
        .irClkEnableMode = 0,                     /* ICSIRCLK disabled */
        .bDiv = 0x1U,                             /* Bus clock divider: divided by 2 */
        .rDiv = 0x0U,                             /* FLL external reference clock divider: divided by 1 */
    };
    const sim_clock_config_t simConfig_BOARD_BootClockRUN =
    {
        .outDiv1 = 0x0U,                          /* DIV1 clock divider: divided by 1 */
        .outDiv2 = 0x0U,                          /* DIV2 clock divider: divided by 1 */
        .outDiv3 = 0x0U,                          /* DIV3 clock divider: divided by 1 */
        .busClkPrescaler = 0x0U,                  /* bus clock optional prescaler */
    };
    const osc_config_t oscConfig_BOARD_BootClockRUN =
    {
        .freq = 0U,                               /* Oscillator frequency: 0Hz */
        .workMode = 0,                            /* Use external clock */
        .enableMode = 0,                          /* Disable external reference clock */
    };

    /* Set the system clock dividers in SIM to safe value. */
    CLOCK_SetSimSafeDivs();
    /* Set ICS to FEI mode. */
    CLOCK_BootToFeiMode(icsConfig_BOARD_BootClockRUN.bDiv);
    /* Configure the Internal Reference clock (ICSIRCLK). */
    CLOCK_SetInternalRefClkConfig(icsConfig_BOARD_BootClockRUN.irClkEnableMode);
    /* Set the clock configuration in SIM module. */
    CLOCK_SetSimConfig(&simConfig_BOARD_BootClockRUN);
}

static inline void HAL_Reboot()
{
    __disable_irq();

    for (uint8_t iReg = 0U; iReg < 1; iReg++) { NVIC->ICER[iReg] = 0xFFFFFFFF; }
    for (uint8_t iReg = 0U; iReg < 1; iReg++) { NVIC->ICPR[iReg] = 0xFFFFFFFF; }

    SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;

// #ifdef HAL_REBOOT_RESET_CLOCK
    HAL_ResetClock();
// #endif

    NVIC_SystemReset();
}

#endif