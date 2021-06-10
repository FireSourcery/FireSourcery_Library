

typedef struct
{
	//UI - change to 1 per all motor?
	Debounce_T PinBrake;
	Debounce_T PinThrottle;
	Debounce_T PinForward;
	Debounce_T PinReverse;
	volatile bool InputSwitchBrake;
	volatile bool InputSwitchThrottle;
	volatile bool InputSwitchForward;
	volatile bool InputSwitchReverse;
	volatile bool InputSwitchNeutral;
	volatile uint16_t InputValueThrottle;
	volatile uint16_t InputValueBrake;

} MotorUser_T;

