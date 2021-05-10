/*
 * Millis.c
 *
 * Set up millisecond counter, using SysTick timer.
 *
 */
#include "Millis.h"

#include <stdint.h>

volatile uint32_t Millis_TickCount = 0;
	
//void (*Millis_OnTick)(void);
void (*Millis_Yield)(void);

void Delay(uint32_t ms) //blocking delay
{
	uint32_t start = Millis_TickCount;
	uint32_t elapsed = 1U;

	while (Millis_TickCount < start + ms)
	{
		if (Millis_Yield) Millis_Yield();
		while (Millis_TickCount < start + elapsed);
		elapsed++;
	}
}

void Millis_MapYield(void (*fp)(void))
{
	Millis_Yield = fp;
}

//void Millis_MapOnTick(void (*fp)(void))
//{
//	Millis_OnTick = fp;
//}

#ifdef CONFIG_MILLIS_MCU_ARM
// KE/KEA PE priority 2 -> PRI_15 = 0x80
#ifdef CPU_FREQ
void Millis_Init(uint8_t priority)
{
	SYST_RVR = (CPU_FREQ / 1000U) - 1U;
	SYST_CVR = 0U;
	SYST_CSR = SysTick_CSR_CLKSOURCE_MASK | SysTick_CSR_TICKINT_MASK | SysTick_CSR_ENABLE_MASK;
	SCB_SHPR3 = (uint32_t) ((SCB_SHPR3 & (uint32_t) ~(uint32_t) (SCB_SHPR3_PRI_15(0xFF))) | (uint32_t) (SCB_SHPR3_PRI_15(priority)));
}
#else
void Millis_Init(uint32_t freqCpu, uint8_t priority)
{
	SYST_RVR = (freqCpu / 1000U) - 1U;
	SYST_CVR = 0U;
	SYST_CSR = SysTick_CSR_CLKSOURCE_MASK | SysTick_CSR_TICKINT_MASK | SysTick_CSR_ENABLE_MASK;
	SCB_SHPR3 = (uint32_t) ((SCB_SHPR3 & (uint32_t) ~(uint32_t) (SCB_SHPR3_PRI_15(0xFF))) | (uint32_t) (SCB_SHPR3_PRI_15(priority)));
}
#endif
#endif
