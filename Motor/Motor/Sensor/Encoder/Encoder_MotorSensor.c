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
    @file   Encoder_MotorSensor.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Encoder_MotorSensor.h"

const MotorSensor_VTable_T HALL_VTABLE =
{
    .INIT = (MotorSensor_Proc_T)Encoder_MotorSensor_Init,
    .CAPTURE_ANGLE = (MotorSensor_Capture_T)Encoder_MotorSensor_CaptureAngle,
    .CAPTURE_SPEED = (MotorSensor_Capture_T)Encoder_MotorSensor_CaptureSpeed,
    .VERIFY_CALIBRATION = (MotorSensor_Test_T)Encoder_MotorSensor_VerifyCalibration,
    // .SetInitial = (MotorSensor_Proc_T)Encoder_SetInitial,
    // .GetElectricalAngle = (MotorSensor_Angle_T)Encoder_GetElectricalAngle,
    // .GetMechanicalAngle = (MotorSensor_Angle_T)Encoder_GetMechanicalAngle,
};


// const Encoder_MotorSensor_T * OfContext(const MotorSensor_T * p_sensor) { return (Encoder_MotorSensor_T *)p_sensor; }

void Encoder_MotorSensor_Init(const Encoder_MotorSensor_T * p_sensor)
{
//     Encoder_MotorSensor_T * p_this = (Encoder_MotorSensor_T *)p_sensor; // Safe cast

//     Encoder_Init(&p_this->HALL);
//     Encoder_ModeDT_Init_Polling(p_this->P_ENCODER);
// #if     defined(CONFIG_MOTOR_HALL_MODE_ISR)
//     Encoder_InitInterrupts_ABC(p_this->P_ENCODER);
// #endif
//     // Encoder_SetUnitsEncoder_MechSpeed(p_this->P_ENCODER, );

//     Encoder_ModeDT_SetInitial(p_this->P_ENCODER);
}


void Encoder_MotorSensor_CaptureAngle(const MotorSensor_T * p_sensor)
{
    // Encoder_MotorSensor_T * p_this = (Encoder_MotorSensor_T *)p_sensor;
    // p_sensor->P_STATE->ElectricalAngle = Encoder_GetAngle16(&p_this->HALL) + Encoder_ModeDT_InterpolateAngularDisplacement(p_this->P_ENCODER);
}

void Encoder_MotorSensor_CaptureSpeed(const MotorSensor_T * p_sensor)
{
//     Encoder_MotorSensor_T * p_this = (Encoder_MotorSensor_T *)p_sensor;
//     Encoder_ModeDT_CaptureVelocity(p_this->P_ENCODER);
//     p_sensor->P_STATE->AngularSpeed_DegPerCycle = Encoder_ModeDT_CapturePollingAngle(p_this->P_ENCODER);
//     p_sensor->P_STATE->Speed_Fract16 = Encoder_ModeDT_GetScalarVelocity(p_this->P_ENCODER);
// }

// uint16_t Encoder_MotorSensor_CaptureState(const MotorSensor_T * p_sensor)
// {

}

bool Encoder_MotorSensor_VerifyCalibration(const MotorSensor_T * p_sensor)
{
    // Encoder_MotorSensor_T * p_this = (Encoder_MotorSensor_T *)p_sensor;
    // return Encoder_IsTableValid(p_this->HALL.P_STATE);
}

// void Encoder_SetUnitsEncoder_ElSpeed(Encoder_State_T * p_encoder, uint32_t speedRef_Rpm)
// {
//     p_encoder->Config.IsQuadratureCaptureEnabled = false;
//     if (p_encoder->Config.ScalarSpeedRef_Rpm != speedRef_Rpm) { Encoder_SetScalarSpeedRef(p_encoder, speedRef_Rpm); }
//     if (p_encoder->Config.CountsPerRevolution != 6U) { Encoder_SetCountsPerRevolution(p_encoder, 6U); }
//     if (p_encoder->Config.PartitionsPerRevolution != 1U) { Encoder_SetPartitionsPerRevolution(p_encoder, 1U); }
// }

// void Encoder_SetUnitsEncoder_MechSpeed(Encoder_State_T * p_encoder, uint16_t speedRef_Rpm, uint8_t polePairs)
// {
//     p_encoder->Config.IsQuadratureCaptureEnabled = false;
//     if (p_encoder->Config.ScalarSpeedRef_Rpm != speedRef_Rpm) { Encoder_SetScalarSpeedRef(p_encoder, speedRef_Rpm); }
//     if (p_encoder->Config.CountsPerRevolution != polePairs * 6U) { Encoder_SetCountsPerRevolution(p_encoder, polePairs * 6U); }
//     if (p_encoder->Config.PartitionsPerRevolution != polePairs) { Encoder_SetPartitionsPerRevolution(p_encoder, polePairs); }     /* Set for electrical cycle */
// }


void Encoder_MotorSensor_InitConfig(const MotorSensor_T * p_sensor, uint16_t speedRef_Rpm, uint8_t polePairs)
{
    // Encoder_SetUnitsEncoder_MechSpeed(p_sensor, speedRef_Rpm, polePairs);
}