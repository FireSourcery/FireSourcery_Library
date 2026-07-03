// /******************************************************************************/
// /*
//     Shared Pin channel case
// */
// /******************************************************************************/
// /* Optionally use Hall ISR */
// static inline void Motor_HallEncoderA_ISR(Motor_T * p_dev)
// {
// #ifdef MOTOR_SENSOR_ENCODER_ENABLE
//     Encoder_OnPhaseA_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
// #endif
// #if defined(MOTOR_HALL_MODE_ISR)
//     if (p_dev->P_MOTOR->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
// #endif
// }

// static inline void Motor_HallEncoderB_ISR(Motor_T * p_dev)
// {
// #ifdef MOTOR_SENSOR_ENCODER_ENABLE
//     Encoder_OnPhaseB_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
// #endif
// #if defined(MOTOR_HALL_MODE_ISR)
//     if (p_dev->P_MOTOR->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
// #endif
// }

// static inline void Motor_HallEncoderAB_ISR(Motor_T * p_dev)
// {
// #ifdef MOTOR_SENSOR_ENCODER_ENABLE
//     Encoder_OnPhaseAB_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
// #endif
// #if defined(MOTOR_HALL_MODE_ISR)
//     if (p_dev->P_MOTOR->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
// #endif
// }

// static inline void Motor_HallEncoderCZ_ISR(Motor_T * p_dev)
// {
//     switch (p_dev->P_MOTOR->Config.SensorMode)
//     {
//     #ifdef MOTOR_SENSOR_ENCODER_ENABLE
//         case ROTOR_SENSOR_ID_ENCODER:
//             Encoder_OnIndex_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
//             break;
//         #endif

//         #ifdef MOTOR_HALL_MODE_ISR
//         case ROTOR_SENSOR_ID_HALL:
//             Encoder_OnPhaseC_Hall_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
//             Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL);
//             break;
//         #endif
//         default: break;
//     }
// }
