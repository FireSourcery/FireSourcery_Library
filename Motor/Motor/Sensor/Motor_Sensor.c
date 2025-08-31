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
#include "../Motor_StateMachine.h"



static inline RotorSensor_T * Sensor(const Motor_T * p_motor) { return RotorSensor_Of(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR_STATE->Config.SensorMode); }

/******************************************************************************/
/*
    Collective Calibration access
    requires Motor StateMachine Context with substate
    cannot move to Sensor Interface
*/
/******************************************************************************/
/*!
    @param[in] RotorSensor_Id_T as varId. 1 less layer of nesting. handle in calling module.

    // varId as sensorId, varValue as cmdId
*/
void Motor_Sensor_CalibrationCmd_Call(const Motor_T * p_motor, RotorSensor_Id_T varId, int varValue)
{
    if (p_motor == NULL) return;
    if (!Motor_StateMachine_IsConfig(p_motor)) return;
    // If the sensor is not the selected [p_ActiveSensor], do not calibrate.
    if (!RotorSensor_Validate(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR_STATE->p_ActiveSensor, varId)) return;
    if (p_motor->P_MOTOR_STATE->Config.SensorMode != varId) return;

    // RotorSensor_Calibrate(p_motor->P_MOTOR_STATE->p_ActiveSensor, p_motor, varId, varValue);
    switch (varId)
    {
        case ROTOR_SENSOR_ID_HALL:
            (void)varValue; // unused
            assert(p_motor->P_MOTOR_STATE->p_ActiveSensor == (RotorSensor_T *)&p_motor->SENSOR_TABLE.HALL);
            assert(p_motor->P_MOTOR_STATE->Config.SensorMode == ROTOR_SENSOR_ID_HALL);
            Motor_Hall_Calibrate(p_motor);
            break;
        case ROTOR_SENSOR_ID_ENCODER:
            // switch (varValue)
            // {

            // }
            // Encoder_Calibrate(&p_motor->SENSOR_TABLE.ENCODER.ENCODER);
            break;
        #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
        case ROTOR_SENSOR_ID_SIN_COS:
            // SinCos_Calibrate(&p_motor->SENSOR_TABLE.SIN_COS.SIN_COS);
            break;
        #endif
        #if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
        case ROTOR_SENSOR_ID_SENSORLESS:
            // Sensorless_Calibrate(&p_motor->SENSOR_TABLE.SENSORLESS.SENSORLESS);
            break;
        #endif
        default: break;
    }
}





/******************************************************************************/
/*!
    Position Sensor Feedback - Speed, Angle
*/
/******************************************************************************/



/* Control Freq */
// angle16_t Motor_PollSensorAngle(const Motor_T * p_motor)
// {
//     angle16_t electricalAngle; /* [0, 65535] maps to negative portions of angle16_t */

//     switch (p_motor->P_MOTOR_STATE->Config.SensorMode)
//     {
//         case ROTOR_SENSOR_ID_HALL:


//             break;

//         case ROTOR_SENSOR_ID_ENCODER:
//             electricalAngle = Encoder_GetPartitionAngle(&p_motor->ENCODER);
//             electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->ENCODER);
//             break;

//         #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
//         case ROTOR_SENSOR_ID_SIN_COS:
//             SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
//             electricalAngle = SinCos_GetElectricalAngle(&p_motor->SinCos);
//             //todo group
//             AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
//             AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
//             break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
//         case ROTOR_SENSOR_ID_SENSORLESS:
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



// angle16_t Motor_GetMechanicalAngle(const Motor_T * p_motor)
// {
    // angle16_t angle;
    // switch (p_motor->P_MOTOR_STATE->Config.SensorMode)
    // {
    //     case ROTOR_SENSOR_ID_HALL:    angle = Encoder_GetAngle(&p_motor->ENCODER.P_STATE);    break;
    //     case ROTOR_SENSOR_ID_ENCODER: angle = Encoder_GetAngle(&p_motor->ENCODER.P_STATE);    break;
    //     #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
    //     case ROTOR_SENSOR_ID_SIN_COS: angle = SinCos_GetMechanicalAngle(&p_motor->SinCos);     break;
    //     #endif
    //     #if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
    //     case ROTOR_SENSOR_ID_SENSORLESS: angle = 0; break;
    //     #endif
    //     default: angle = 0; break;
    // }
    // return angle;
// }

// /*!
//     @return Fract16 Q1.15 unsaturated
// */
// int32_t Motor_PollSensorSpeed(const Motor_T * p_motor)
// {
//     int32_t speed;
//     switch (p_motor->P_MOTOR_STATE->Config.SensorMode)
//     {
//         /* Using assigned direction */
//         case ROTOR_SENSOR_ID_HALL:
//             Encoder_ModeDT_CaptureFreqD(&p_motor->ENCODER);
//             p_motor->P_MOTOR_STATE->ElectricalSpeed_DegPerCycle = Encoder_ModeDT_CapturePollingDelta(&p_motor->ENCODER);
//             speed = Encoder_ModeDT_GetScalarVelocity(&p_motor->ENCODER);
//             break;
//         case ROTOR_SENSOR_ID_ENCODER:
//             Encoder_ModeDT_CaptureFreqD(&p_motor->ENCODER);
//             speed = Encoder_ModeDT_GetScalarVelocity(&p_motor->ENCODER);
//             break;

//         #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
//         case ROTOR_SENSOR_ID_SIN_COS: speed = PollAngleSpeed(p_motor, SinCos_GetMechanicalAngle(&p_motor->SinCos));    break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
//         case ROTOR_SENSOR_ID_SENSORLESS: break;
//         #endif
//         default: speed = 0; break;
//     }
//     return speed;
//     // return p_motor->ElectricalSpeed_DegPerCycle;
// }





// static inline bool _Motor_IsOpenLoop(const Motor_T * p_motor)
// {
//     // #if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE)  || defined(CONFIG_MOTOR_DEBUG_ENABLE)
//     return (p_motor->P_MOTOR_STATE->FeedbackMode.OpenLoop == 1U);
//     // #else
//     //     (void)p_motor; return false;
//     // #endif
// }

// inline bool Motor_IsClosedLoop(const Motor_T * p_motor)
// {
//     return ((_Motor_IsSensorAvailable(p_motor) == true) && (_Motor_IsOpenLoop(p_motor) == false));
// }







// #if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
// extern int16_t Motor_User_GetGroundSpeed_Mph(Motor_State_T * p_motor);
// extern void Motor_User_SetGroundSpeed_Kmh(Motor_State_T * p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
// extern void Motor_User_SetGroundSpeed_Mph(Motor_State_T * p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
// #endif


// /******************************************************************************/
// /*
//     Ground Speed
// */
// /******************************************************************************/
// #if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
// int16_t Motor_User_GetGroundSpeed_Kmh(Motor_State_T * p_motor)
// {
//     int16_t speed;

//     switch (p_motor->Config.SensorMode)
//     {
//         case ROTOR_SENSOR_ID_HALL:        speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);    break;
//         case ROTOR_SENSOR_ID_ENCODER:     speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);    break;
//         #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
//         case ROTOR_SENSOR_ID_SIN_COS:     speed = Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
//         case ROTOR_SENSOR_ID_SENSORLESS:     speed = 0;     break;
//         #endif
//         default:                             speed = 0;     break;
//     }

//     return speed;
// }

// int16_t Motor_User_GetGroundSpeed_Mph(Motor_State_T * p_motor)
// {
//     int16_t speed;

//     switch (p_motor->Config.SensorMode)
//     {
//         case ROTOR_SENSOR_ID_HALL:         speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);    break;
//         case ROTOR_SENSOR_ID_ENCODER:     speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);    break;
//         #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
//         case ROTOR_SENSOR_ID_SIN_COS:     speed = Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
//         case ROTOR_SENSOR_ID_SENSORLESS:     speed = 0;     break;
//         #endif
//         default:                             speed = 0;     break;
//     }

//     return speed;
// }

// void Motor_User_SetGroundSpeed_Kmh(Motor_State_T * p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
// {
//     switch (p_motor->Config.SensorMode)
//     {
//         case ROTOR_SENSOR_ID_HALL:        Encoder_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
//         case ROTOR_SENSOR_ID_ENCODER:     Encoder_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
//         #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
//         case ROTOR_SENSOR_ID_SIN_COS:     Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
//         case ROTOR_SENSOR_ID_SENSORLESS:     speed = 0;     break;
//         #endif
//         default:     break;
//     }
// }

// void Motor_User_SetGroundSpeed_Mph(Motor_State_T * p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
// {
//     switch (p_motor->Config.SensorMode)
//     {
//         case ROTOR_SENSOR_ID_HALL:         Encoder_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
//         case ROTOR_SENSOR_ID_ENCODER:     Encoder_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
//         #if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
//         case ROTOR_SENSOR_ID_SIN_COS:      Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
//         case ROTOR_SENSOR_ID_SENSORLESS:     speed = 0;     break;
//         #endif
//         default:                             break;
//     }
// }
// #endif