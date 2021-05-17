#ifndef HAL_PIN_PLATFORM_H
#define HAL_PIN_PLATFORM_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
	GPIO_Type * p_GpioBase;
	uint32_t GpioPinMask;
} HAL_Pin_T;

static inline void HAL_Pin_WriteState(const HAL_Pin_T * p_pin, bool isOn)
{
	isOn ? (p_pin->p_GpioBase->PDOR |= p_pin->GpioPinMask) : (p_pin->p_GpioBase->PDOR &= ~(p_pin->GpioPinMask));
}

static inline bool HAL_Pin_ReadState(const HAL_Pin_T * p_pin)
{
	return ((p_pin->p_GpioBase->PDIR | p_pin->GpioPinMask) != 0U) ? true : false;
}

#endif
