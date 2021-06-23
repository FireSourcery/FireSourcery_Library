//for elapsed time
typedef struct
{
	const volatile uint32_t * p_Counter;
	uint32_t CounterFreq;

	uint32_t Period;			// Number of timer ticks between updates
	bool IsOneShot;

	volatile uint32_t CounterPrev; 	// Last update time stamp
	volatile bool IsEnabled;		// Enable or disable

} Timer_T;
