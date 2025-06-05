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
    @file   SysTime.c
    @author FireSourcery
    @brief  SysTick Wrapper

*/
/******************************************************************************/
#include "SysTime.h"

volatile uint32_t SysTime_Millis = 0U;

// static void (*Yield)(void);

/* Blocking delay */
void SysTime_Delay(uint32_t ms, void (*yield)(void))
{
    uint32_t start = SysTime_Millis;
    uint32_t time = SysTime_Millis;

    while(SysTime_Millis < start + ms)
    {
        if(yield != 0U) { yield(); }
        while(SysTime_Millis <= time);
        time = SysTime_Millis;
    }
}

// void SysTime_SetDelayYield(void (*fp)(void)) { Yield = fp; }

/* App init must set priority */
void SysTime_Init(void)
{
#ifdef CONFIG_SYSTIME_SYSTICK
    // SysTick_Config(ticks)
    SYST_RVR = (CPU_FREQ / 1000U) - 1U;
    SYST_CVR = 0U;
    SYST_CSR = SYST_CSR_CLKSOURCE_MASK | SYST_CSR_TICKINT_MASK | SYST_CSR_ENABLE_MASK;
#endif
}
