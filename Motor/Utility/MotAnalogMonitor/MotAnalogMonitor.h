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
	volatile const adc_t * const P_THROTTLE_ADCU;
	volatile const adc_t * const P_BRAKE_ADCU;
}
AnalogMonitor_AdcMap_T;

////		.HAL_PIN_METER 		= {.P_GPIO_BASE = MTR_OUT_PORT,		.GPIO_PIN_MASK = (uint32_t)1U << MTR_OUT_PIN,	},
////		.HAL_PIN_COIL	 	= {.P_GPIO_BASE = COIL_OUT_PORT,	.GPIO_PIN_MASK = (uint32_t)1U << COIL_OUT_PIN,	},
////		.HAL_PIN_AUX1	 	= {.P_GPIO_BASE = ECO_IN_PORT,		.GPIO_PIN_MASK = (uint32_t)1U << ECO_IN_PIN,	},

typedef struct
{
	AnalogMonitor_AdcMap_T * p_AdcMap;


}
AnalogMonitor_T;
