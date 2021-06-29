//typedef enum
//{
//	/* BEMF sensorless */
//	MOTOR_ANALOG_CHANNEL_VBUS, 	/* V battery, V in */
//	MOTOR_ANALOG_CHANNEL_VA,
//	MOTOR_ANALOG_CHANNEL_VB,
//	MOTOR_ANALOG_CHANNEL_VC,
//
//	/* FOC */
//	MOTOR_ANALOG_CHANNEL_IA,
//	MOTOR_ANALOG_CHANNEL_IB,
//	MOTOR_ANALOG_CHANNEL_IC,
//
//	/* Temperature */
//	MOTOR_ANALOG_CHANNEL_HEAT_MOTOR,
//	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
//
//	MOTOR_ANALOG_CHANNEL_HEAT_PCB,
//
//	/* Error checking */
//	MOTOR_ANALOG_CHANNEL_VACC,		/* V accessories */
//	MOTOR_ANALOG_CHANNEL_VSENSE,	/* V analog sensors */
//
//	/* analog sensor input */
////	MOTOR_ANALOG_CHANNEL_THROTTLE,
////	MOTOR_ANALOG_CHANNEL_BRAKE,
//} Motor_AnalogChannel_T;
//
///*******************************************************************************/
///*!
//    @brief  Conversion
//*/
///*******************************************************************************/
//static const uint8_t ANALOG_CONVERSION_CHANNELS_BEMF_A[] =
//{
//	[0] = MOTOR_ANALOG_CHANNEL_VBUS,
//	[1] = MOTOR_ANALOG_CHANNEL_VA,
//
//	[2] = MOTOR_ANALOG_CHANNEL_IB,
//	[3] = MOTOR_ANALOG_CHANNEL_IC
//};
//
//static void (* ANALOG_CONVERSION_FUNCTIONS_BEMF_A[])(Motor_T *) =
//{
//	[0] = 0,
//	[1] = Motor_SixStep_ProcBemf_IO,
//	[2] = 0,
//	[3] = 0,
//	[4] = 0,
//};
//
//const Analog_Conversion_T ANALOG_CONVERSION_BEMF_A =
//{
//	.Config =
//	{
//		.UseConfig = 1U,
//		.UseHwTriggerPerConversion = 1U,
//		.UseInterrrupt = 1U,
//	},
//	.ChannelCount = sizeof(ANALOG_CONVERSION_CHANNELS_BEMF_A)/sizeof(uint8_t),
//	.p_VirtualChannels 	= ANALOG_CONVERSION_CHANNELS_BEMF_A,
//	.p_OnCompleteChannels = ANALOG_CONVERSION_FUNCTIONS_BEMF_A,
//	.OnCompleteConversion = 0U,
//	.p_OnCompleteUserData = 0U,
//};
//
//void MotorAnalog_ActivateConversionBemfA(Motor_T * p_motor)
//{
//	Analog_ActivateConversion(&p_motor->Analog, &ANALOG_1_CONVERSION_BEMF_A);
////	Analog_EnqueueConversion(&p_motor->Analog, &ANALOG_1_CONVERSION_BEMF_A);
//}
//
//
//
//#define MOTOR_ANALOG_CONVERSION_QUEUE_BUFFER_SIZE (4U)
//static const Analog_Conversion_T * (p_ConversionQueue[ANALOG_CONVERSION_QUEUE_BUFFER_SIZE]);
//
//void MotorAnalog_Init(Motor_T * p_motor, uint32_t * p_ChannelPinMap,  uint32_t * p_ChannelAdcMap)
//{
//	Analog_Init
//	(
//		&p_motor->Analog,
//		&p_motor->p_Init->P_HAL_ADCS,
//		10U, //pin count
//		p_ChannelPinMap,
//		p_ChannelAdcMap,
////		sizeof(ANALOG_1_CHANNEL_PINS)/sizeof(uint32_t),
////		ANALOG_1_CHANNEL_PINS,
//		&p_motor->AnalogChannelResults,
//		p_ConversionQueue,
//		ANALOG_1_CONVERSION_QUEUE_BUFFER_SIZE
//	);
//}
