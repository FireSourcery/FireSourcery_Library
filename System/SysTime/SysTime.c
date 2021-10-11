/*
 * SysTime.c
 *
 * Set up millisecond counter
 * Default use SysTick timer.
 *
 */
#include "SysTime.h"

#include <stdint.h>

volatile uint32_t SysTime_Millis = 0U;
	
//void (*SysTime_OnTick)(void);
void (*SysTime_Yield)(void);

void SysTime_Delay(uint32_t ms) //blocking delay
{
	uint32_t start = SysTime_Millis;
	uint32_t time = SysTime_Millis;

	while (SysTime_Millis < start + ms)
	{
		if (SysTime_Yield != 0U)
		{
			SysTime_Yield();
		}
		while (SysTime_Millis <= time);
		time = SysTime_Millis;
	}
}

void SysTime_MapYield(void (*fp)(void))
{
	SysTime_Yield = fp;
}

//void SysTime_MapOnTick(void (*fp)(void))
//{
//	SysTime_OnTick = fp;
//}

volatile const uint32_t * SysTime_GetPtr(void)
{
	return &SysTime_Millis;
}

#ifdef CONFIG_SYSTIME_SYSTICK
	#ifdef CONFIG_SYSTIME_CPU_FREQ
//priority = p & 0xF0 > 4, lower is higher
void SysTime_Init(uint8_t priority)
{
	SYST_RVR = (CONFIG_SYSTIME_CPU_FREQ / 1000U) - 1U;
	SYST_CVR = 0U;
	SYST_CSR = SysTick_CSR_CLKSOURCE_MASK | SysTick_CSR_TICKINT_MASK | SysTick_CSR_ENABLE_MASK;
	SCB_SHPR3 = (SCB_SHPR3 & SCB_SHPR3_PRI_15_MASK) | (SCB_SHPR3_PRI_15(priority));
}
	#else
void SysTime_Init(uint32_t freqCpu, uint8_t priority)
{
	SYST_RVR = (freqCpu / 1000U) - 1U;
	SYST_CVR = 0U;
	SYST_CSR = SysTick_CSR_CLKSOURCE_MASK | SysTick_CSR_TICKINT_MASK | SysTick_CSR_ENABLE_MASK;
	SCB_SHPR3 = (SCB_SHPR3 & SCB_SHPR3_PRI_15_MASK) | (SCB_SHPR3_PRI_15(priority));
}
	#endif
#endif
