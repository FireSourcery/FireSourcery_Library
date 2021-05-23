#ifndef BLINKY_H_
#define BLINKY_H_

#include <stdint.h>
#include <stdbool.h>

#include "Peripheral/Pin/Pin.h"

typedef struct
{
	HAL_Pin_T * p_HAL_Pin;
	volatile bool IsOn;

//	Thread_Timer_T ThreadTimer;
	uint32_t OnTime;
	uint32_t OffTime;
} Blinky_T;


#endif
