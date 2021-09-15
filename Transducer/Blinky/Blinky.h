#ifndef BLINKY_H_
#define BLINKY_H_

#include <stdint.h>
#include <stdbool.h>

//#include "Peripheral/Pin/Pin.h"
#include "HAL_Pin.h"

#include "System/Thread/Thread.h"


typedef struct
{
	HAL_Pin_T * p_HAL_Pin;

	uint32_t ProcFreq; //outer thread freq

	volatile bool IsOn;

	Thread_T ThreadTimer;
	uint32_t OnTime;
	uint32_t OffTime;

	//default settings for when timing parameters are not specified
} Blinky_T;


#endif
