#ifndef CONFIG_HAL_PERIPHERAL_H
#define CONFIG_HAL_PERIPHERAL_H

#if defined(KE06Z4_SERIES) || (KE02Z4_SERIES) /* From NXP headers */

#else
    #error "No valid CPU defined!"
#endif

// #ifndef HAL_ENCODER_CLOCK_SOURCE_FREQ
// #define HAL_ENCODER_CLOCK_SOURCE_FREQ CPU_FREQ
// #endif

// #ifndef HAL_PWM_CLOCK_SOURCE_FREQ
// #define HAL_PWM_CLOCK_SOURCE_FREQ CPU_FREQ
// #endif

// #ifndef HAL_SERIAL_CLOCK_SOURCE_FREQ
// #define HAL_SERIAL_CLOCK_SOURCE_FREQ (CPU_FREQ / 2U)
// #endif

#endif