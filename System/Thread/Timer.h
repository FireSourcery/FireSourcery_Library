//for elapsed time
typedef struct
{
	const volatile uint32_t * p_Counter;
	uint32_t CounterFreq;
	bool IsOneShot;

	volatile uint32_t CounterPrev; 	// Last update time stamp
	volatile uint32_t Period;		// Number of timer ticks between updates
} Timer_T;
