/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Motor_Sensor.c
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "Motor_Sensor.h"
#include "../Motor.h"


void Motor_Sensor_ResetUnits(Motor_State_T * p_motor)
{
    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
            // Motor_ResetUnitsHallEncoder_ElSpeed(p_motor);
            // Motor_ResetUnitsHallEncoder_MechSpeed(p_motor);
            // Encoder_SetUnitsHall_MechSpeed(&p_motor->p_ActiveSensor, Motor_GetSpeedRatedRef_Rpm(p_motor), p_motor->Config.PolePairs);
            // Hall_MotorSensor_InitConfig(const Hall_MotorSensor_T * p_sensor, uint16_t speedRef_Rpm, uint8_t polePairs);
            break;
        case MOTOR_SENSOR_MODE_ENCODER:
            // Motor_ResetUnitsEncoder(p_motor);
            break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:
            // Motor_ResetUnitsSinCos(&p_motor->SinCos);
            // Motor_ResetUnitsAngleSpeed_Mech(&p_motor);
            break;
        #endif
        #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:
            // Motor_ResetUnitsAngleSpeed_ElecControl(p_motor);
            break;
        #endif
        default:
            break;
    }
}




// static inline int32_t Motor_Sensor_Config_Get(const Motor_T * p_motor, MotorSensor_Id_T sensor, uint8_t configId)
// {
//     const MotorSensor_Table_T * p_table = &p_motor->SENSOR_TABLE;
//     switch (sensor)
//     {
//         case MOTOR_SENSOR_MODE_HALL: return Motor_Hall_Config_Get(&p_table->HALL.HALL, configId);
//         // case MOTOR_SENSOR_MODE_ENCODER: return Encoder_MotorSensor_Config_Get(&p_table->ENCODER, configId);
//         // case MOTOR_SENSOR_MODE_SIN_COS: return SinCos_MotorSensor_Config_Get(&p_table->SinCos, configId);
//         // case MOTOR_SENSOR_MODE_SENSORLESS: return Sensorless_MotorSensor_Config_Get(&p_table->Sensorless, configId);
//         default: return 0; // or some default value
//     }
// }


/******************************************************************************/
/*!
    Position Sensor Feedback - Speed, Angle
*/
/******************************************************************************/
/* Re init peripheral registers */

// void Motor_InitSensor(const Motor_T * p_motor)
// {
//     // MotorSensor_Init(&p_motor->Sensor Interface);

//     switch (p_motor->P_ACTIVE->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_HALL:
//             Hall_Init(&p_motor->HALL);
//         #if     defined(CONFIG_MOTOR_HALL_MODE_POLLING) || !defined(CONFIG_MOTOR_HALL_MODE_ISR)
//             Encoder_ModeDT_Init_Polling(&p_motor->ENCODER);
//         #elif   defined(CONFIG_MOTOR_HALL_MODE_ISR)
//             Encoder_InitInterrupts_ABC(&p_motor->ENCODER);
//         #endif
//             break;
//         case MOTOR_SENSOR_MODE_ENCODER:
//             Encoder_ModeDT_Init_InterruptQuadrature(&p_motor->ENCODER);
//             Encoder_EnableQuadratureMode(&p_motor->ENCODER);
//             break;
//         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SIN_COS:
//             SinCos_Init(&p_motor->SinCos);
//             break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS:
//             break;
//         #endif
//         default:
//             break;
//     }
//     Motor_ResetUnitsSensor(p_motor);
// }

// bool Motor_VerifySensorCalibration(const Motor_T * p_motor)
// {
//     bool isValid = true;
//     switch (p_motor->P_ACTIVE->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_HALL:
//             // if (Hall_IsStateValid(&p_motor->HALL.P_STATE) == false) { isValid = false; }
//             if (Hall_IsTableValid(&p_motor->HALL.P_STATE) == false) { isValid = false; }
//             break;
//         case MOTOR_SENSOR_MODE_ENCODER:
//             break;
//         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SIN_COS:
//             break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS:
//             break;
//         #endif
//         default:
//             break;
//     }
//     return isValid;
// }

/* Control Freq */
// angle16_t Motor_PollSensorAngle(const Motor_T * p_motor)
// {
//     angle16_t electricalAngle; /* [0, 65535] maps to negative portions of angle16_t */

//     switch (p_motor->P_ACTIVE->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_HALL:
//         #if defined(CONFIG_MOTOR_HALL_MODE_POLLING)
//             if (Hall_PollCaptureSensors(&p_motor->HALL) == true)
//             {
//                 Encoder_SinglePhase_CapturePulse(&p_motor->ENCODER); // Encoder_CaptureCount
//                 Hall_CaptureAngle(&p_motor->HALL);
//                 // Hall_CaptureDirection(&p_motor->HALL); /* on  direction diff */
//             }
//         #endif
//             electricalAngle = Hall_GetAngle16(&p_motor->HALL);
//             electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->ENCODER);
//             /* handle direction reset */
//             break;

//         case MOTOR_SENSOR_MODE_ENCODER:
//             electricalAngle = Encoder_GetPartitionAngle(&p_motor->ENCODER);
//             electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->ENCODER);
//             break;

//         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SIN_COS:
//             SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
//             electricalAngle = SinCos_GetElectricalAngle(&p_motor->SinCos);
//             //todo group
//             AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
//             AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
//             break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS:
//             //todo observer
//             electricalAngle = 0;
//             p_motor->FeedbackMode.OpenLoop = 1U;
//             p_motor->FeedbackMode.OpenLoop = 1U;
//             p_motor->StateFlags.SensorFeedback = 0U;
//             p_motor->StateFlags.SensorFeedback = 0U;
//             break;
//         #endif
//         default: electricalAngle = 0; break;
//     }

//     return electricalAngle;
// }

// angle16_t Motor_CaptureAngle(const Motor_T * p_motor)
// {
//     p_motor->P_ACTIVE->ElectricalAngle = Motor_PollSensorAngle(p_motor);
//     return p_motor->P_ACTIVE->ElectricalAngle;
// }

// angle16_t Motor_GetMechanicalAngle(const Motor_T * p_motor)
// {
//     // angle16_t angle;
//     // switch (p_motor->P_ACTIVE->Config.SensorMode)
//     // {
//     //     case MOTOR_SENSOR_MODE_HALL:    angle = Encoder_GetAngle(&p_motor->ENCODER.P_STATE);    break;
//     //     case MOTOR_SENSOR_MODE_ENCODER: angle = Encoder_GetAngle(&p_motor->ENCODER.P_STATE);    break;
//     //     #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//     //     case MOTOR_SENSOR_MODE_SIN_COS: angle = SinCos_GetMechanicalAngle(&p_motor->SinCos);     break;
//     //     #endif
//     //     #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
//     //     case MOTOR_SENSOR_MODE_SENSORLESS: angle = 0; break;
//     //     #endif
//     //     default: angle = 0; break;
//     // }
//     // return angle;
// }

// /*!
//     @return Fract16 Q1.15 unsaturated
// */
// int32_t Motor_PollSensorSpeed(const Motor_T * p_motor)
// {
//     int32_t speed;
//     switch (p_motor->P_ACTIVE->Config.SensorMode)
//     {
//         /* Using assigned direction */
//         case MOTOR_SENSOR_MODE_HALL:
//             Encoder_ModeDT_CaptureVelocity(&p_motor->ENCODER);
//             p_motor->P_ACTIVE->AngularSpeed_DegPerCycle = Encoder_ModeDT_CapturePollingAngle(&p_motor->ENCODER);
//             speed = Encoder_ModeDT_GetScalarVelocity(&p_motor->ENCODER);
//             break;
//         case MOTOR_SENSOR_MODE_ENCODER:
//             Encoder_ModeDT_CaptureVelocity(&p_motor->ENCODER);
//             speed = Encoder_ModeDT_GetScalarVelocity(&p_motor->ENCODER);
//             break;

//         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SIN_COS: speed = PollAngleSpeed(p_motor, SinCos_GetMechanicalAngle(&p_motor->SinCos));    break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS: break;
//         #endif
//         default: speed = 0; break;
//     }
//     return speed;
//     // return p_motor->AngularSpeed_DegPerCycle;
// }

// bool Motor_PollCaptureSpeed(const Motor_T * p_motor)
// {
//     bool isCaptureSpeed = Timer_Periodic_Poll(&p_motor->P_ACTIVE->SpeedTimer);
//     if (isCaptureSpeed == true) { p_motor->P_ACTIVE->Speed_Fract16 = (Motor_PollSensorSpeed(p_motor) + p_motor->P_ACTIVE->Speed_Fract16) / 2; }
//     return isCaptureSpeed;
// }

// void Motor_ZeroSensor(const Motor_T * p_motor)
// {
//     switch (p_motor->P_ACTIVE->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_HALL:
//             Hall_SetInitial(&p_motor->HALL);
//             Encoder_ModeDT_SetInitial(&p_motor->ENCODER);
//             break;
//         case MOTOR_SENSOR_MODE_ENCODER:
//             Encoder_ModeDT_SetInitial(&p_motor->ENCODER);
//             break;
//         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SIN_COS:
//             break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS:
//             Motor_SetPositionFeedback(p_motor, 0U);
//             break;
//         #endif
//         default: break;
//     }
// }

// /* From Stop and after Align */
// bool _Motor_IsSensorAvailable(const Motor_T * p_motor)
// {
//     bool isAvailable;
//     switch (p_motor->P_ACTIVE->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_HALL:    isAvailable = true;         break;
//         case MOTOR_SENSOR_MODE_ENCODER: isAvailable = Encoder_IsAligned(p_motor->ENCODER.P_STATE);    break;
//         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SIN_COS:     isAvailable = true;     break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS:  isAvailable = false;    break;
//         #endif
//         default: isAvailable = false; break;
//     }
//     return isAvailable;
// }

// static inline bool _Motor_IsOpenLoop(const Motor_T * p_motor)
// {
//     // #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE)  || defined(CONFIG_MOTOR_DEBUG_ENABLE)
//     return (p_motor->P_ACTIVE->FeedbackMode.OpenLoop == 1U);
//     // #else
//     //     (void)p_motor; return false;
//     // #endif
// }

// inline bool Motor_IsClosedLoop(const Motor_T * p_motor)
// {
//     return ((_Motor_IsSensorAvailable(p_motor) == true) && (_Motor_IsOpenLoop(p_motor) == false));
// }

// // Motor_Direction_T Motor_GetDirection(const Motor_T * p_motor)
// // {
// //     switch (p_motor->P_ACTIVE->Config.SensorMode)
// //     {
// //         case MOTOR_SENSOR_MODE_HALL:
// //             break;
// //         case MOTOR_SENSOR_MODE_ENCODER:
// //             break;
// //         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
// //         case MOTOR_SENSOR_MODE_SIN_COS:
// //             break;
// //         #endif
// //         #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
// //         case MOTOR_SENSOR_MODE_SENSORLESS:
// //             break;
// //         #endif
// //         default:
// //             break;
// //     }
// // }


// /*
//     Sensor Direction
// */
// void Motor_SetSensorDirection(const Motor_T * p_motor, Motor_Direction_T direction)
// {
//     switch (p_motor->P_ACTIVE->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_HALL:
//             Hall_SetDirection(p_motor->HALL.P_STATE, (Hall_Direction_T)direction);
//             Encoder_SinglePhase_SetDirection(&p_motor->ENCODER, direction);  /* interpolate as +/- */
//             break;
//         case MOTOR_SENSOR_MODE_ENCODER:
//             break;
//         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS:  break;
//         #endif
//         default: break;
//     }
// }

/*
    todo as electrical speed
*/

/*
    Reset Sensors, propagate Motor Config to sensor module independent params,
    Sync to motor module params
*/
// void Motor_ResetUnitsHallEncoder_MechSpeed(const Motor_T * p_motor)
// {
//     p_motor->ENCODER.Config.IsQuadratureCaptureEnabled = false;

//     if (p_motor->Config.PolePairs * 6U != p_motor->ENCODER.Config.CountsPerRevolution)
//         { Encoder_SetCountsPerRevolution(&p_motor->ENCODER, p_motor->Config.PolePairs * 6U); }

//     if (Motor_GetSpeedRatedRef_Rpm(p_motor) != p_motor->ENCODER.Config.ScalarSpeedRef_Rpm)
//         { Encoder_SetScalarSpeedRef(&p_motor->ENCODER, Motor_GetSpeedRatedRef_Rpm(p_motor)); }

//     /* Set for electrical cycle */
//     if (p_motor->Config.PolePairs != p_motor->ENCODER.Config.PartitionsPerRevolution)
//         { Encoder_SetPartitionsPerRevolution(&p_motor->ENCODER, p_motor->Config.PolePairs); }
// }

// void Motor_ResetUnitsHallEncoder_ElSpeed(const Motor_T * p_motor)
// {
//     p_motor->ENCODER.Config.IsQuadratureCaptureEnabled = false;
//     if (p_motor->ENCODER.Config.CountsPerRevolution != 6U) { Encoder_SetCountsPerRevolution(&p_motor->ENCODER, 6U); }
//     if (Motor_GetSpeedRatedRef_ERpm(p_motor) != p_motor->ENCODER.Config.ScalarSpeedRef_Rpm)
//         { Encoder_SetScalarSpeedRef(&p_motor->ENCODER, Motor_GetSpeedRatedRef_ERpm(p_motor)); }

//     Encoder_SetPartitionsPerRevolution(&p_motor->ENCODER, 1);
// }


// void Motor_ResetUnitsEncoder(const Motor_T * p_motor)
// {
//     // if (Motor_GetSpeedRatedRef_Rpm(p_motor) != p_motor->ENCODER.Config.ScalarSpeedRef_Rpm)
//     // {
//     //     Encoder_SetScalarSpeedRef(&p_motor->ENCODER, Motor_GetSpeedRatedRef_Rpm(p_motor));
//     // }
//     // if (p_motor->Config.PolePairs != p_motor->ENCODER.Config.PartitionsPerRevolution) /* Set for electrical cycle */
//     // {
//     //     Encoder_SetPartitionsPerRevolution(&p_motor->ENCODER, p_motor->Config.PolePairs);
//     // }
//     // if(p_motor->Config.GearRatioOutput != p_motor->ENCODER.Config.GearRatioOutput) ||
//     // {
//     //     Encoder_SetSurfaceRatio(&p_motor->ENCODER, p_motor->Config.GearRatio);
//     // }
// }

// void Motor_ResetUnitsSensor(const Motor_T * p_motor)
// {
//     switch (p_motor->P_ACTIVE->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_HALL:
//             // Motor_ResetUnitsHallEncoder_ElSpeed(p_motor);
//             // Motor_ResetUnitsHallEncoder_MechSpeed(p_motor);
//             Encoder_SetUnitsHall_MechSpeed(&p_motor->ENCODER, Motor_GetSpeedRatedRef_Rpm(p_motor->P_ACTIVE), p_motor->P_ACTIVE->Config.PolePairs);
//             break;
//         case MOTOR_SENSOR_MODE_ENCODER:
//             // Motor_ResetUnitsEncoder(p_motor);
//             break;
//         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SIN_COS:
//             // Motor_ResetUnitsSinCos(&p_motor->SinCos);
//             // Motor_ResetUnitsAngleSpeed_Mech(&p_motor);
//             break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS:
//             // Motor_ResetUnitsAngleSpeed_ElecControl(p_motor);
//             break;
//         #endif
//         default:
//             break;
//     }
// }

