// sensor interface



/* SinCos, Mechanical Rotation Sensor */
// void Motor_ResetUnitsAngleSpeed_ElecControl(Motor_T * p_motor)
// {
//     // Linear_Speed_InitElectricalAngleRpm(&p_motor->UnitsAngleRpm, MOTOR_CONTROL_FREQ, 16U, p_motor->Config.PolePairs, Motor_GetSpeedVRef_Rpm(p_motor));
// }

// void Motor_ResetUnitsAngleSpeed_Mech(Motor_T * p_motor)
// {
//     // Linear_Speed_InitAngleRpm(&p_motor->UnitsAngleRpm, 1000U, 16U, Motor_GetSpeedVRef_Rpm(p_motor));
// }


/* For SinCos, Sensorless, when not using Encoder module */
// static inline int32_t PollAngleSpeed(Motor_T * p_motor, angle16_t speedAngle)
// {
//     int32_t speedDelta = speedAngle - p_motor->SpeedAngle; /* loops if no overflow past 1 full cycle */
//     int32_t speedFeedback_Fract16 = (p_motor->Speed_Fract16 + Linear_Speed_CalcAngleRpmFract16(&p_motor->UnitsAngleRpm, speedDelta)) / 2;
//     p_motor->SpeedAngle = speedAngle; /* mechanical angle */
//     return speedFeedback_Fract16;
// }
