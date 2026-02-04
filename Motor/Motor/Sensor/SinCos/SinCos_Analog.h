// typedef enum Motor_Analog_Channel
// {
// #if defined(MOTOR_SENSOR_SIN_COS_ENABLE)
//     PHASE_ANALOG_CHANNEL_SIN,
//     PHASE_ANALOG_CHANNEL_COS,
// #endif
// }
// Motor_Analog_Channel_T;

// typedef union Phase_Input_Conversions
// {
//     struct
//     {
//     #if defined(MOTOR_SENSOR_SIN_COS_ENABLE)
//         const Analog_Conversion_T CONVERSION_SIN;
//         const Analog_Conversion_T CONVERSION_COS;
//     #endif
//     };
// }
// Phase_Input_Conversions_T;

// void Motor_MarkAnalog_Thread(Motor_State_T * p_motor)
// {
// #if defined(MOTOR_SENSOR_SIN_COS_ENABLE) || defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
//     if (p_motor->Config.SensorMode == ROTOR_SENSOR_ID_SIN_COS)
//     {
//         Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
//         Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
//     }
//     // RotorSensor_MarkAnalog(&p_motor->Sensor);
// #endif
// }