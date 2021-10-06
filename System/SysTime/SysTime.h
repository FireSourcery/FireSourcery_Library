#include <stdint.h>

#ifndef SYSTIME_H
#define SYSTIME_H

#ifdef CONFIG_SYSTIME_SYSTICK
#define SYST_CSR 	(*((uint32_t *)0xE000E010))		// SysTick Control and Status Register
#define SYST_RVR 	(*((uint32_t *)0xE000E014))		// SysTick Reload Value Register
#define SYST_CVR 	(*((uint32_t *)0xE000E018))		// SysTick Current Value Register
#define SYST_CALIB 	(*((uint32_t *)0xE000E01C))		// SysTick Calibration Value Register
#define SCB_SHPR3 	(*((uint32_t *)0xE000ED20))		// System Handler Priority Register 3

#define SysTick_CSR_ENABLE_MASK                  0x1u
#define SysTick_CSR_ENABLE_SHIFT                 0
#define SysTick_CSR_ENABLE_WIDTH                 1
#define SysTick_CSR_ENABLE(x)                    (((uint32_t)(((uint32_t)(x))<<SysTick_CSR_ENABLE_SHIFT))&SysTick_CSR_ENABLE_MASK)
#define SysTick_CSR_TICKINT_MASK                 0x2u
#define SysTick_CSR_TICKINT_SHIFT                1
#define SysTick_CSR_TICKINT_WIDTH                1
#define SysTick_CSR_TICKINT(x)                   (((uint32_t)(((uint32_t)(x))<<SysTick_CSR_TICKINT_SHIFT))&SysTick_CSR_TICKINT_MASK)
#define SysTick_CSR_CLKSOURCE_MASK               0x4u
#define SCB_SHPR3_PRI_15_MASK                    0xFF000000u
#define SCB_SHPR3_PRI_15_SHIFT                   24
#define SCB_SHPR3_PRI_15_WIDTH                   8
#define SCB_SHPR3_PRI_15(x)                      (((uint32_t)(((uint32_t)(x))<<SCB_SHPR3_PRI_15_SHIFT))&SCB_SHPR3_PRI_15_MASK)

#define SYSTICK_COUNT() (SYST_CVR)
#endif

/*
 * MISRA Violation
 */
extern volatile uint32_t SysTime_Millis;

/*
 *  User Map to 1ms ISR
 */
static inline void SysTime_CaptureMillis_ISR(void)
{
	SysTime_Millis++;
//	if (Millis_OnTick) Millis_OnTick();
}

static inline uint32_t SysTime_GetMillis(void)
{
	return SysTime_Millis;
}

static inline uint32_t Millis(void)
{
	return SysTime_GetMillis();
}

#if defined(CONFIG_SYSTIME_SYSTICK) && defined(CONFIG_SYSTIME_CPU_FREQ)

static inline uint32_t SysTime_GetMicros(void)
{
	volatile uint32_t micros = (CONFIG_SYSTIME_CPU_FREQ / 1000U - 1U) - (SYST_CVR / (CONFIG_SYSTIME_CPU_FREQ / 1000000U)); //SYST_CVR ticks down
	return SysTime_Millis * 1000U + micros;
}

static inline uint32_t Micros(void)
{
	return SysTime_GetMicros();
}

#endif

#ifdef CONFIG_SYSTIME_SYSTICK
	#ifdef CONFIG_SYSTIME_CPU_FREQ
void SysTime_Init(uint8_t priority);
	#else
void SysTime_Init(uint32_t freqCpu, uint8_t priority);
	#endif
#endif

#endif /* SYSTIME_H */
