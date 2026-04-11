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

static void Encoder_RotorSensor_Init(const Encoder_RotorSensor_T * p_sensor)
{
//     Encoder_RotorSensor_T * p_this = (Encoder_RotorSensor_T *)p_sensor; // Safe cast
// Encoder_ModeDT_Init_InterruptQuadrature(&p_motor->ENCODER);
// Encoder_EnableQuadratureMode(&p_motor->ENCODER);
//     Encoder_ModeDT_SetInitial(p_this->P_ENCODER);
//     Encoder_ModeDT_Init_InterruptQuadrature(p_encoder);
//     Encoder_EnableQuadratureMode(p_encoder);
//     // Motor_ResetUnitsSensor(p_motor);
}


static void Encoder_RotorSensor_CaptureAngle(const RotorSensor_T * p_sensor)
{
    // Encoder_RotorSensor_T * p_this = (Encoder_RotorSensor_T *)p_sensor;
    // p_sensor->P_STATE->ElectricalAngle = Encoder_GetAngle16(&p_this->HALL) + Encoder_ModeDT_InterpolateAngularDisplacement(p_this->P_ENCODER);
    // Encoder_GetAngle_Scalar(&p_motor->ENCODER, p_motor->Config.PolePairs); /* ElectricalAngle => MechanicalAngle * PolePairs */
    // electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->ENCODER);
}

static void Encoder_RotorSensor_CaptureSpeed(const Encoder_RotorSensor_T * p_sensor)
{
    RotorSensor_State_T * p_state = p_sensor->BASE.P_STATE;
    Encoder_ModeDT_CaptureFreqD(&p_sensor->ENCODER);
    // p_state->ElectricalSpeed_DegPerCycle = Encoder_ModeDT_ResolveInterpolation(p_sensor->ENCODER.P_STATE);
    // p_state->Speed_Fract16 = Encoder_ModeDT_GetScalarVelocity(p_sensor->ENCODER.P_STATE);
    // Encoder_ModeDT_CaptureScalarVelocity(&p_motor->ENCODER) / 2;
}


static bool Encoder_RotorSensor_VerifyCalibration(const Encoder_RotorSensor_T * p_sensor)
{
    // Encoder_RotorSensor_T * p_this = (Encoder_RotorSensor_T *)p_sensor;
    // return Encoder_IsTableValid(p_this->HALL.P_STATE);
}

static void Encoder_RotorSensor_ZeroSensor(const Encoder_RotorSensor_T * p_sensor)
{
    Encoder_ModeDT_SetInitial(&p_sensor->ENCODER);
}

/* From Stop and after Align */
static bool Encoder_RotorSensor_IsSensorAvailable(const Encoder_RotorSensor_T * p_sensor)
{
    return Encoder_IsAligned(p_sensor->ENCODER.P_STATE);
}

// angle resolution 65536/cpr
// el angle per tick = 65536/cpr * polepairs
// counts per electrical revolution = cpr/polepairs
static void Encoder_RotorSensor_InitFrom(const Encoder_RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    p_sensor->ENCODER.P_STATE->Config.ScalarSpeedRef_Rpm = p_config->SpeedTypeMax_Rpm;
    Encoder_ModeDT_InitValuesFrom(&p_sensor->ENCODER, &p_sensor->ENCODER.P_STATE->Config);
}


const RotorSensor_VTable_T ENCODER_VTABLE =
{
    .INIT = (RotorSensor_Proc_T)Encoder_RotorSensor_Init,
    .INIT_UNITS_FROM = (RotorSensor_InitFrom_T)Encoder_RotorSensor_InitFrom,
    .CAPTURE_ANGLE = (RotorSensor_Proc_T)Encoder_RotorSensor_CaptureAngle,
    .CAPTURE_SPEED = (RotorSensor_Proc_T)Encoder_RotorSensor_CaptureSpeed,
    .VERIFY_CALIBRATION = (RotorSensor_Test_T)Encoder_RotorSensor_VerifyCalibration,
    .IS_FEEDBACK_AVAILABLE = (RotorSensor_Test_T)Encoder_RotorSensor_IsSensorAvailable,
    .ZERO_INITIAL = (RotorSensor_Proc_T)Encoder_RotorSensor_ZeroSensor,
};






