//Analog Board Sensors

//per controller monitor

typedef const struct
{
	volatile const adc_t * const P_VBUS_ADCU;
	volatile const adc_t * const P_VACC_ADCU;
	volatile const adc_t * const P_VSENSE_ADCU;
	volatile const adc_t * const P_HEAT_PCB_ADCU;
	volatile const adc_t * const P_HEAT_MOSFETS_H_ADCU;
	volatile const adc_t * const P_HEAT_MOSFETS_L_ADCU;
//	volatile const adc_t * const P_THROTTLE_ADCU;
//	volatile const adc_t * const P_BRAKE_ADCU;
}
MotAnalogMonitor_AdcMap_T;

////		.HAL_PIN_METER 		= {.P_GPIO_BASE = MTR_OUT_PORT,		.GPIO_PIN_MASK = (uint32_t)1U << MTR_OUT_PIN,	},
////		.HAL_PIN_COIL	 	= {.P_GPIO_BASE = COIL_OUT_PORT,	.GPIO_PIN_MASK = (uint32_t)1U << COIL_OUT_PIN,	},
////		.HAL_PIN_AUX1	 	= {.P_GPIO_BASE = ECO_IN_PORT,		.GPIO_PIN_MASK = (uint32_t)1U << ECO_IN_PIN,	},

typedef const struct
{
	const volatile uint16_t VBUS_LIMIT_HIGH_V;
	const volatile uint16_t VBUS_LIMIT_LOW_V;

	const volatile uint8_t HEAT_PCB_HIGH_SHUTDOWN_C;
	const volatile uint8_t HEAT_PCB_HIGH_THRESHOLD_C;


	const volatile adc_t VBUS_LIMIT_HIGH_ADCU;
	const volatile adc_t VBUS_LIMIT_LOW_ADCU;
	const volatile adc_t HEAT_PCB_SHUTDOWN_HIGH_ADCU;
	const volatile adc_t HEAT_PCB_THRESHOLD_HIGH_ADCU;
	const volatile adc_t HEAT_MOSFETS_H_SHUTDOWN_HIGH_ADCU;
	const volatile adc_t HEAT_MOSFETS_H_THRESHOLD_HIGH_ADCU;
	const volatile adc_t HEAT_MOSFETS_L_SHUTDOWN_HIGH_ADCU;
	const volatile adc_t HEAT_MOSFETS_L_THRESHOLD_HIGH_ADCU;
}
MotAnalogMonitor_Params_T;


typedef struct
{
	const MotAnalogMonitor_AdcMap_T ADC_MAP;
	const MotAnalogMonitor_Params_T * P_PARAMS;

}
MotAnalogMonitor_T;
