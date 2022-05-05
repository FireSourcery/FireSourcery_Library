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
	@file 	SysTime.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "SysTime.h"

volatile uint32_t SysTime_Millis = 0U;

static void (*Yield)(void);

/* blocking delay */
void SysTime_Delay(uint32_t ms)
{
	uint32_t start = SysTime_Millis;
	uint32_t time = SysTime_Millis;

	while(SysTime_Millis < start + ms)
	{
		if(Yield != 0U)
		{
			Yield();
		}
		while(SysTime_Millis <= time);
		time = SysTime_Millis;
	}
}

void SysTime_SetDelayYield(void (*fp)(void))
{
	Yield = fp;
}

#ifdef CONFIG_SYSTIME_SYSTICK 
void SysTime_Init(void)
{
	SYST_RVR = (CONFIG_SYSTIME_CPU_FREQ / 1000U) - 1U;
	SYST_CVR = 0U;
	SYST_CSR = SYST_CSR_CLKSOURCE_MASK | SYST_CSR_TICKINT_MASK | SYST_CSR_ENABLE_MASK;
	SCB_SHPR3 = (SCB_SHPR3 & SCB_SHPR3_PRI_15_MASK) | (SCB_SHPR3_PRI_15(CONFIG_SYSTIME_SYSTICK_PRIORITY)); /* Priority => PRIORITY & 0xF0 >> 4, lower is higher */
} 
#endif
