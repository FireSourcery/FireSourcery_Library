/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Encoder_RotorSensor.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Encoder_Sensor.h"


// const Encoder_RotorSensor_T * OfContext(const RotorSensor_T * p_sensor) { return (Encoder_RotorSensor_T *)p_sensor; }

void Encoder_RotorSensor_Init(const Encoder_RotorSensor_T * p_sensor)
{
//     Encoder_RotorSensor_T * p_this = (Encoder_RotorSensor_T *)p_sensor; // Safe cast

//     Encoder_Init(&p_this->HALL);
//     Encoder_ModeDT_Init_Polling(p_this->P_ENCODER);
// #if     defined(CONFIG_MOTOR_HALL_MODE_ISR)
//     Encoder_InitInterrupts_ABC(p_this->P_ENCODER);
// #endif
//     // Encoder_SetUnitsEncoder_MechSpeed(p_this->P_ENCODER, );

//     Encoder_ModeDT_SetInitial(p_this->P_ENCODER);
}


void Encoder_RotorSensor_CaptureAngle(const RotorSensor_T * p_sensor)
{
    // Encoder_RotorSensor_T * p_this = (Encoder_RotorSensor_T *)p_sensor;
    // p_sensor->P_STATE->ElectricalAngle = Encoder_GetAngle16(&p_this->HALL) + Encoder_ModeDT_InterpolateAngularDisplacement(p_this->P_ENCODER);
}

void Encoder_RotorSensor_CaptureSpeed(const RotorSensor_T * p_sensor)
{
//     Encoder_RotorSensor_T * p_this = (Encoder_RotorSensor_T *)p_sensor;
//     Encoder_ModeDT_CaptureVelocity(p_this->P_ENCODER);
//     p_sensor->P_STATE->ElectricalSpeed_DegPerCycle = Encoder_ModeDT_CapturePollingAngle(p_this->P_ENCODER);
//     p_sensor->P_STATE->Speed_Fract16 = Encoder_ModeDT_GetScalarVelocity(p_this->P_ENCODER);
// }

// uint16_t Encoder_RotorSensor_CaptureState(const RotorSensor_T * p_sensor)
// {
}

bool Encoder_RotorSensor_VerifyCalibration(const RotorSensor_T * p_sensor)
{
    // Encoder_RotorSensor_T * p_this = (Encoder_RotorSensor_T *)p_sensor;
    // return Encoder_IsTableValid(p_this->HALL.P_STATE);
}





const RotorSensor_VTable_T ENCODER_VTABLE =
{
    .INIT = (RotorSensor_Proc_T)Encoder_RotorSensor_Init,
    .CAPTURE_ANGLE = (RotorSensor_Proc_T)Encoder_RotorSensor_CaptureAngle,
    .CAPTURE_SPEED = (RotorSensor_Proc_T)Encoder_RotorSensor_CaptureSpeed,
    .VERIFY_CALIBRATION = (RotorSensor_Test_T)Encoder_RotorSensor_VerifyCalibration,
    // .SetInitial = (RotorSensor_Proc_T)Encoder_SetInitial,
    // .GetElectricalAngle = (RotorSensor_Angle_T)Encoder_GetElectricalAngle,
    // .GetMechanicalAngle = (RotorSensor_Angle_T)Encoder_GetMechanicalAngle,
};
