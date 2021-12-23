

typedef const struct
{
	//	const Pin_T PIN_METER;
	//	const Pin_T PIN_COIL;
}
MotAux_Config_T;

typedef struct
{
	MotAux_Config_T  CONFIG;
	Debounce_T PinDIn; //configurable input
	bool InputSwitchDIn;
}
MotAux_T;

//#define MOT_AUX_CONFIG(p_Millis, p_Brake_PinHal, Brake_PinId, p_Throttle_PinHal, Throttle_PinId, p_Forward_PinHal, Forward_PinId, p_Reverse_PinHal, Reverse_PinId, p_ThrottleAdcu, p_BrakeAdcu)	\
//{																\
//	.CONFIG =													\
//	{															\
//		.P_THROTTLE_ADCU =		p_ThrottleAdcu,					\
//		.P_BRAKE_ADCU =			p_BrakeAdcu, 					\
//	},															\
//	.PinMeter 	=  (p_Brake_PinHal, 		Brake_PinId, 		p_Millis),  	\
//	.PinCoil 	=  (p_Throttle_PinHal, 	Throttle_PinId, 	p_Millis),  	\
//}
//.HAL_PIN_METER 	= {.P_GPIO_BASE = MTR_OUT_PORT,		.GPIO_PIN_MASK = (uint32_t)1U << MTR_OUT_PIN,	},
//.HAL_PIN_COIL	 	= {.P_GPIO_BASE = COIL_OUT_PORT,	.GPIO_PIN_MASK = (uint32_t)1U << COIL_OUT_PIN,	},
//.HAL_PIN_AUX1	 	= {.P_GPIO_BASE = ECO_IN_PORT,		.GPIO_PIN_MASK = (uint32_t)1U << ECO_IN_PIN,	},
