// void Motor_MarkAnalog_Thread(Motor_T * p_motor)
// {
// #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
//     if (p_motor->Config.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
//     {
//         Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
//         Analog_MarkConversion(&p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
//     }
//     // MotorSensor_MarkAnalog(&p_motor->Sensor);
// #endif
// }