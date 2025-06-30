// typedef enum Motor_Analog_Channel
// {
// #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
//     MOTOR_ANALOG_CHANNEL_SIN,
//     MOTOR_ANALOG_CHANNEL_COS,
// #endif
// }
// Motor_Analog_Channel_T;

// typedef union MotorAnalog_Conversions
// {
//     struct
//     {
//     #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
//         const Analog_Conversion_T CONVERSION_SIN;
//         const Analog_Conversion_T CONVERSION_COS;
//     #endif
//     };
// }
// MotorAnalog_Conversions_T;

// void Motor_MarkAnalog_Thread(Motor_State_T * p_motor)
// {
// #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE) || defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
//     if (p_motor->Config.SensorMode == ROTOR_SENSOR_ID_SIN_COS)
//     {
//         Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
//         Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
//     }
//     // RotorSensor_MarkAnalog(&p_motor->Sensor);
// #endif
// }