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

static void (*Yield)(void);

void SysTime_Delay(uint32_t ms) //blocking delay
{
	uint32_t start = SysTime_Millis;
	uint32_t time = SysTime_Millis;

	while (SysTime_Millis < start + ms)
	{
		if (Yield != 0U)
		{
			Yield();
		}
		while (SysTime_Millis <= time);
		time = SysTime_Millis;
	}
}

void SysTime_SetDelayYield(void (*fp)(void))
{
	Yield = fp;
}

#ifdef CONFIG_SYSTIME_SYSTICK

	#ifdef CONFIG_SYSTIME_SYSTICK_PRIOTIY
	#else
		#define CONFIG_SYSTIME_SYSTICK_PRIOTIY 0x80U // Priority 8  //priority => pri & 0xF0 >> 4, lower is higher
	#endif

	#ifdef CONFIG_SYSTIME_CPU_FREQ
	#else
		#error "SysTime - Undefined CONFIG_SYSTIME_CPU_FREQ"
	#endif

void SysTime_Init(void)
{
	SYST_RVR = (CONFIG_SYSTIME_CPU_FREQ / 1000U) - 1U;
	SYST_CVR = 0U;
	SYST_CSR = SYST_CSR_CLKSOURCE_MASK | SYST_CSR_TICKINT_MASK | SYST_CSR_ENABLE_MASK;
	SCB_SHPR3 = (SCB_SHPR3 & SCB_SHPR3_PRI_15_MASK) | (SCB_SHPR3_PRI_15(CONFIG_SYSTIME_SYSTICK_PRIOTIY));
}

#endif
