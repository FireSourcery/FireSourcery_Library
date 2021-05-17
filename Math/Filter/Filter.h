#include "Math/Q/QFrac16.h"

typedef struct
{
	uint32_t * Buffer;
	uint8_t Index;
	uint8_t BufferSize;
//	bool IsBufferFull;

	int16_t Coeffcient;
	int32_t Accumulator;
}Filter_T;


