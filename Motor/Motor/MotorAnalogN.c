#include "Peripheral/Analog/Analog.h"

#include <stdint.h>
#include <stdbool.h>


/* if Analog_T abstracted to controls all adc.(nfixed and nmultimuxed modes). Analog_T   can be controlled within motor module */
//	Analog_T MotorAnalog;

// Analog_T scale 1 per motor, still need shared queue for adc retrurn. and solve with activate overwrite bewteen virtual analogs

//1 analog nadc mode per all motors to share queue,  set active conversion pin map prior to start
//analog per adc hw, hal layer


/*!
	@brief Virtual channel identifiers, index into arrays containing ADC channel data
 */
#define MOTOR_ANALOG_ADC_CHANNEL_COUNT 			14U
typedef enum
{
	/* BEMF sensorless */
	MOTOR_ANALOG_CHANNEL_VBUS, 	/* V battery, V in */
	MOTOR_ANALOG_CHANNEL_VA,
	MOTOR_ANALOG_CHANNEL_VB,
	MOTOR_ANALOG_CHANNEL_VC,

	/* FOC */
	MOTOR_ANALOG_CHANNEL_IA,
	MOTOR_ANALOG_CHANNEL_IB,
	MOTOR_ANALOG_CHANNEL_IC,

	/* Temperature */
	MOTOR_ANALOG_CHANNEL_HEAT_MOTOR,
	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS_HIGH,
	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS_LOW,
	MOTOR_ANALOG_CHANNEL_HEAT_PCB,

	/* Error checking */
	MOTOR_ANALOG_CHANNEL_VACC,		/* V accessories */
	MOTOR_ANALOG_CHANNEL_VSENSE,	/* V analog sensors */

	/* analog sensor input */
//	MOTOR_ANALOG_CHANNEL_THROTTLE,
//	MOTOR_ANALOG_CHANNEL_BRAKE,
} MotorAnalog_Channel_T;

#define CONVERSION_QUEUE_BUFFER_SIZE (8U)
static Analog_ConversionActive_T ConversionQueue[ANALOG_CONVERSION_QUEUE_BUFFER_SIZE];


//void MotorAnalog_Init(Motor_T * p_motor, uint32_t * p_ChannelPinMap,  uint32_t * p_ChannelAdcMap)
//{
//	Analog_Init
//	(
//		&p_motor->Analog,
//		&p_motor->p_Init->P_HAL_ADCS,
//		10U, //pin count
//		p_ChannelPinMap,
//		p_ChannelAdcMap,
////		sizeof(MOTOR_ANALOG_CHANNEL_PINS)/sizeof(uint32_t),
////		MOTOR_ANALOG_CHANNEL_PINS,
//		&p_motor->AnalogChannelResults,
//		p_ConversionQueue,
//		CONVERSION_QUEUE_BUFFER_SIZE
//	);
//}

/*******************************************************************************/
/*!
    @brief  Conversions
*/
/*******************************************************************************/

/*******************************************************************************/
/*!
    @brief  Conversion
*/
/*******************************************************************************/
/*!
	@brief FOC mode ADC channels to be sampled

	Sample all channels sequentially on PWM trigger
	Measure Ia, Ib, Ic first, so there is longer time for FOC calculations
	remaining channels will continue processing as FOC calculations are performed
 */
static const Analog_ConversionChannel_T CHANNELS_IABC[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_IA, Motor_FOC_ProcIa_IO},
	[1U] = {MOTOR_ANALOG_CHANNEL_IB, Motor_FOC_ProcIb_IO},
	[2U] = {MOTOR_ANALOG_CHANNEL_IC, Motor_FOC_ProcIc_IO},
};

static const Analog_Conversion_T CONVERSION_IABC =
{
	.P_CHANNELS 	= CHANNELS_IABC,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_IABC)/sizeof(Analog_ConversionChannel_T),
	.ON_COMPLETE 	= (void (*)(volatile void *)) &Motor_FOC_ProcCurrentFeedback,
	.OPTIONS =
	{
		.UseConfig = 1U,
		.UseHwTriggerPerConversion = 1U,
		.UseInterrrupt = 1U,
	},
};

void MotorAnalog_ActivateIabc(Analog_T * p_analog, Motor_T * p_motor)
{
//	Analog_ActivateConversion(&MotorAnalog, &CONVERSION_IABC, p_motor);
}

/*******************************************************************************/
/*!
    @brief  Conversion
*/
/*******************************************************************************/
static const Analog_ConversionChannel_T CHANNELS_FOC_REMAINDER[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_VA, 			0U},
	[1U] = {MOTOR_ANALOG_CHANNEL_VB, 			0U},
	[2U] = {MOTOR_ANALOG_CHANNEL_VC, 			0U},
	[3U] = {MOTOR_ANALOG_CHANNEL_VBUS, 			0U},
	[4U] = {MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS, 	0U},
};

static const Analog_Conversion_T CONVERSION_FOC_REMAINDER =
{
	.P_CHANNELS 	= CHANNELS_FOC_REMAINDER,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_FOC_REMAINDER)/sizeof(Analog_ConversionChannel_T),
	.OPTIONS =
	{
		.UseConfig = 1U,
		.UseInterrrupt = 1U,
	},
};

void MotorAnalog_EnqueueFocRemainder(Analog_T * p_analog, Motor_T * p_motor)
{
//	Analog_EnqueueConversion(&MotorAnalog, &CONVERSION_FOC_REMAINDER);
}


/*******************************************************************************/
/*!
    @brief  Conversion
*/
/*******************************************************************************/
static const Analog_ConversionChannel_T CHANNELS_BEMF_A[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_VBUS, 		0U},
	[1U] = {MOTOR_ANALOG_CHANNEL_IB, 	Motor_CaptureIBusIb_IO},		//capture as bus current
	[2U] = {MOTOR_ANALOG_CHANNEL_VA, 	Motor_SixStep_CaptureBemfA_IO},//capture as emf for zcd
//	[3] = {MOTOR_ANALOG_CHANNEL_IC,
//	[4] = {MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
};

static const Analog_Conversion_T CONVERSION_BEMF_A =
{
	.P_CHANNELS 			= CHANNELS_BEMF_A,
	.CHANNEL_COUNT 			= sizeof(CHANNELS_BEMF_A)/sizeof(Analog_Conversion_T),
	.ON_COMPLETE = 0U,//MotorAnalog_LoadBemfA, //re arm for next pwm pulse on complete, if phase bemf called once per commutation
	.OPTIONS =
	{
		.UseConfig = 1U,
		.UseHwTriggerPerConversion = 1U,
		.UseInterrrupt = 1U,
	},
};

void MotorAnalog_LoadBemfA(Analog_T * p_analog, Motor_T * p_motor)
{
	Analog_ActivateConversion(&MotorAnalog, &CONVERSION_BEMF_A);
}

/*******************************************************************************/
/*!
    @brief  Conversion
*/
/*******************************************************************************/
static const Analog_ConversionChannel_T CHANNELS_BEMF_B[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_VBUS, 	0U},
	[2U] = {MOTOR_ANALOG_CHANNEL_IC, 	Motor_CaptureIBusIc_IO},
	[1U] = {MOTOR_ANALOG_CHANNEL_VB, 	Motor_SixStep_CaptureBemfB_IO},
//	MOTOR_ANALOG_CHANNEL_IA,
//	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
};

static const Analog_Conversion_T CONVERSION_BEMF_B =
{
	.P_CHANNELS 			= CHANNELS_BEMF_B,
	.CHANNEL_COUNT 			= sizeof(CHANNELS_BEMF_B)/sizeof(Analog_ConversionChannel_T),
	.ON_COMPLETE 	= 0U,//MotorAnalog_LoadBemfB,
	.OPTIONS =
	{
		.UseConfig = 1U,
		.UseHwTriggerPerConversion = 1U,
		.UseInterrrupt = 1U,
	},
};

void MotorAnalog_LoadBemfB(Analog_T * p_analog, Motor_T * p_motor)
{
//	Analog_ActivateConversion(&MotorAnalog, &CONVERSION_BEMF_B);
}

/*******************************************************************************/
/*!
    @brief  Conversion
*/
/*******************************************************************************/
static const Analog_ConversionChannel_T CHANNELS_BEMF_C[] =
{
	[0U] = {MOTOR_ANALOG_CHANNEL_VBUS, 	0U},
	[1U] = {MOTOR_ANALOG_CHANNEL_IA, 	Motor_CaptureIBusIa_IO},
	[2U] = {MOTOR_ANALOG_CHANNEL_VC, 	Motor_SixStep_CaptureBemfC_IO},
//	MOTOR_ANALOG_CHANNEL_IB,
//	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
};

static const Analog_Conversion_T CONVERSION_BEMF_C =
{
	.P_CHANNELS 	= CHANNELS_BEMF_C,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_C)/sizeof(Analog_ConversionChannel_T),
	.ON_COMPLETE 	= 0U,//MotorAnalog_LoadBemfC,
	.OPTIONS =
	{
		.UseConfig = 1U,
		.UseHwTriggerPerConversion = 1U,
		.UseInterrrupt = 1U,
	},
};

void MotorAnalog_LoadBemfC(Analog_T * p_analog, Motor_T * p_motor)
{
//	Analog_ActivateConversion(&MotorAnalog, &CONVERSION_BEMF_C);
}


/*******************************************************************************/
/*!
    @brief  Conversion
*/
/*******************************************************************************/
//static const Analog_ConversionChannel_T CHANNELS_BEMF_REMAINDER[] =
//{
//
//	[4U] = {MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS, 	0U},
//};
//
//static const Analog_Conversion_T CONVERSION_BEMF_REMAINDER =
//{
//	.P_CHANNELS 	= CHANNELS_BEMF_REMAINDER,
//	.CHANNEL_COUNT 	= sizeof(CHANNELS_BEMF_REMAINDER)/sizeof(Analog_ConversionChannel_T),
//	.OPTIONS =
//	{
//		.UseConfig = 1U,
//		.UseInterrrupt = 1U,
//	},
//};

void MotorAnalog_EnqueueBemfRemainder()
{

}


/*******************************************************************************/
/*!
    @brief  Conversion
*/
/*******************************************************************************/
static const Analog_ConversionChannel_T CHANNELS_MONITOR[] =
{
	{MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS, 0U},
	{MOTOR_ANALOG_CHANNEL_HEAT_PCB, 	0U},
	{MOTOR_ANALOG_CHANNEL_HEAT_MOTOR, 	0U},
};

const Analog_Conversion_T CONVERSION_MONITOR =
{
	.P_CHANNELS 	= CHANNELS_MONITOR,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_MONITOR)/sizeof(Analog_ConversionChannel_T),
	.ON_COMPLETE 	= 0U,
	.OPTIONS =
	{
		.UseConfig = 1U,
		.UseInterrrupt = 1U,
	},
};

void MotorAnalog_EnqueueMonitor()
{
//	Analog_Conversion_T * saveConversion = MotorAnalog.p_ActiveConversion;
//
//	Analog_ActivateConversion(&MotorAnalog, &CONVERSION_MONITOR); //todo return to load bemf
//	Analog_EnqueueConversion(&MotorAnalog, saveConversion); //todo return to load bemf
}


/*******************************************************************************/
/*!
    @brief  Conversion when motor is stoped
*/
/*******************************************************************************/
static const Analog_ConversionChannel_T CHANNELS_IDLE[] =
{
	{MOTOR_ANALOG_CHANNEL_VBUS, 	0U},
	{MOTOR_ANALOG_CHANNEL_VA, 		Motor_SixStep_CaptureBemfA_IO},
	{MOTOR_ANALOG_CHANNEL_VB, 		Motor_SixStep_CaptureBemfB_IO},
	{MOTOR_ANALOG_CHANNEL_VC, 		Motor_SixStep_CaptureBemfC_IO},
	{MOTOR_ANALOG_CHANNEL_IA, 		Motor_CaptureIBusIa_IO},
	{MOTOR_ANALOG_CHANNEL_IB, 		Motor_CaptureIBusIb_IO},
	{MOTOR_ANALOG_CHANNEL_IC, 		Motor_CaptureIBusIc_IO},
	{MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS, 	0U},
	{MOTOR_ANALOG_CHANNEL_HEAT_PCB, 		0U},
	{MOTOR_ANALOG_CHANNEL_HEAT_MOTOR, 		0U},
};


const Analog_Conversion_T CONVERSION_IDLE =
{
	.P_CHANNELS 	= CHANNELS_IDLE,
	.CHANNEL_COUNT 	= sizeof(CHANNELS_IDLE)/sizeof(Analog_ConversionChannel_T),
	.ON_COMPLETE 	= 0U,
	.OPTIONS =
	{
		.UseConfig = 1U,
		.UseInterrrupt = 1U,
	},
};


void MotorAnalog_EnqueueIdle()
{
	Analog_EnqueueConversion(&MotorAnalog, &CONVERSION_IDLE);
//	Analog_DequeueConversion(&MotorAnalog);
}

static const Analog_ConversionChannel_T CHANNELS_USER[] =
{
//	MOTOR_ANALOG_CHANNEL_BRAKE,
//	MOTOR_ANALOG_CHANNEL_THROTTLE,


//	MOTOR_ANALOG_CHANNEL_VACC,
//	MOTOR_ANALOG_CHANNEL_VSENSE,
};

static Analog_Conversion_T CONVERSION_USER =
{
	.P_CHANNELS = CHANNELS_USER,
	.CHANNEL_COUNT = sizeof(CHANNELS_USER)/sizeof(Analog_ConversionChannel_T),
	.OPTIONS =
	{
		.UseConfig = 1U,
		.UseInterrrupt = 1U,
	},
	.p_OnCompleteChannels = 0,
	.ON_COMPLETE = 0,
	.p_OnCompleteUserData = 0,
};

void MotorAnalog_EnqueueUser()
{
	Analog_EnqueueConversion(&MotorAnalog, &CONVERSION_USER);
//	Analog_DequeueConversion(&MotorAnalog);	//starts chain if no conversion is currently active
}

