/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Motor_HallEncoder.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../../Motor.h"



/******************************************************************************/
/*
    Shared Pin channel case
*/
/******************************************************************************/
/* Optionally use Hall ISR */
static inline void Motor_HallEncoderA_ISR(Motor_T * p_dev)
{
#ifdef MOTOR_SENSOR_ENCODER_ENABLE
    Encoder_OnPhaseA_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
#endif
#if defined(MOTOR_HALL_MODE_ISR)
    if (p_dev->P_MOTOR->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderB_ISR(Motor_T * p_dev)
{
#ifdef MOTOR_SENSOR_ENCODER_ENABLE
    Encoder_OnPhaseB_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
#endif
#if defined(MOTOR_HALL_MODE_ISR)
    if (p_dev->P_MOTOR->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderAB_ISR(Motor_T * p_dev)
{
#ifdef MOTOR_SENSOR_ENCODER_ENABLE
    Encoder_OnPhaseAB_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
#endif
#if defined(MOTOR_HALL_MODE_ISR)
    if (p_dev->P_MOTOR->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderCZ_ISR(Motor_T * p_dev)
{
    switch (p_dev->P_MOTOR->Config.SensorMode)
    {
    #ifdef MOTOR_SENSOR_ENCODER_ENABLE
        case ROTOR_SENSOR_ID_ENCODER:
            Encoder_OnIndex_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
            break;
        #endif

        #ifdef MOTOR_HALL_MODE_ISR
        case ROTOR_SENSOR_ID_HALL:
            Encoder_OnPhaseC_Hall_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
            Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL);
            break;
        #endif
        default: break;
    }
}
