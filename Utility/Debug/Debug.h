#ifndef DEBUG_H
#define DEBUG_H

#include "System/SysTime/SysTime.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	volatile uint32_t Debug_TimeRef;
	volatile uint32_t Debug_Delta[10U];
	volatile uint32_t Debug_PeriodRef[10U];
} Debug_T;

extern volatile Debug_T g_Debug;

static inline Debug_CaptureRef()
{
	g_Debug.Debug_TimeRef = Micros();
}

static uint32_t Debug_CaptureElapsed(uint8_t index)
{
	uint32_t micros = Micros() - g_Debug.Debug_TimeRef;
	g_Debug.Debug_Delta[index] = micros;
	return micros;
}

static inline Debug_CapturePeriod(uint8_t index)
{
	volatile uint32_t micros = Micros();

	g_Debug.Debug_Delta[index] = micros - g_Debug.Debug_PeriodRef[index];
	g_Debug.Debug_PeriodRef[index] = micros;
}


#endif
