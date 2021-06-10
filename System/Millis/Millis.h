#include <stdint.h>

#ifndef MILLIS_H
#define MILLIS_H

#ifdef CONFIG_MILLIS_MCU_ARM
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
extern volatile uint32_t Millis_Count;

/*
 *  User Map to 1ms ISR
 */
static inline void Millis_Tick_IO(void)
{
	Millis_Count++;
//	if (Millis_OnTick) Millis_OnTick();
}

static inline uint32_t Millis(void)
{
	return Millis_Count;
}

#if defined(CONFIG_MILLIS_MCU_ARM) && defined(CPU_FREQ)
static inline uint32_t Micros(void)
{
	uint32_t micros = (CPU_FREQ / 1000U - 1U) - (SYST_CVR / (CPU_FREQ / 1000000U)); //SYST_CVR ticks down
	return Millis_Count * 1000U + micros;
}
#endif


#ifdef CONFIG_MILLIS_MCU_ARM
#ifdef CPU_FREQ
void Millis_Init(uint8_t priority);
#else
void Millis_Init(uint32_t freqCpu, uint8_t priority);
#endif
#endif

#endif /* MILLIS_H */
