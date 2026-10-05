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


static void Encoder_RotorSensor_Init(const Encoder_RotorSensor_T * p_sensor)
{
    Encoder_MT_Init_InterruptQuadrature(&p_sensor->ENCODER);
    Encoder_EnableQuadratureMode( p_sensor->ENCODER .P_STATE);
}

/*
    20kHz. Count resolution, no interpolation between counts.
    Mechanical in the calibrated direction, d-axis frame. Electrical valid when Encoder_IsAligned
*/
static void Encoder_RotorSensor_CaptureAngle(const Encoder_RotorSensor_T * p_sensor)
{
    p_sensor->BASE.P_STATE->MechanicalAngle = Encoder_GetAngle(&p_sensor->ENCODER) * Encoder_GetDirectionRef(p_sensor->ENCODER.P_STATE);
    Angle_SetAngle(&p_sensor->BASE.P_STATE->AngleSpeed, _RotorSensor_ElectricalAngleOf(p_sensor->BASE.P_STATE, p_sensor->BASE.P_STATE->MechanicalAngle));
}

static void Encoder_RotorSensor_CaptureSpeed(const Encoder_RotorSensor_T * p_sensor)
{
    RotorSensor_State_T * p_state = p_sensor->BASE.P_STATE;
    Encoder_MT_CaptureFreqD(&p_sensor->ENCODER);
    Encoder_MT_ResolveInterpolation(&p_sensor->ENCODER);
    p_state->Speed_Pu = Encoder_MT_GetSpeed_Pu(p_sensor->ENCODER.P_STATE);
    Angle_CaptureSpeed_Pu(&p_state->AngleSpeed, &p_state->UnitRef.SpeedPuRef, p_state->Speed_Pu);
    /* Promote on any index edge, not only during a homing sweep - an ordinary open loop start up reaches Z too */
    Encoder_PollIndexCapture(p_sensor->ENCODER.P_STATE);
}


static bool Encoder_RotorSensor_VerifyCalibration(const Encoder_RotorSensor_T * p_sensor)
{
    (void)p_sensor;
    // return Encoder_IsTableValid(p_this->HALL.P_STATE);
    return true;
}

/*
    Resets pulse accumulation and speed state only. Reached on a direction change, where counting
    stays continuous - the position reference survives. Encoder_ClearPositionRef is for a real
    discontinuity: fault, sensor loss, power on.
*/
static void Encoder_RotorSensor_ZeroSensor(const Encoder_RotorSensor_T * p_sensor)
{
    Encoder_MT_SetInitial(&p_sensor->ENCODER);
}

/* Commutation needs the electrical datum only - ALIGNED or better */
static bool Encoder_RotorSensor_IsSensorAvailable(const Encoder_RotorSensor_T * p_sensor)
{
    return Encoder_IsAligned(p_sensor->ENCODER.P_STATE);
}

// angle resolution 65536/cpr
// el angle per tick = 65536/cpr * polepairs
// counts per electrical revolution = cpr/polepairs
/* The encoder revolution is mechanical: base AngleFreqBase / PolePairs */
static void Encoder_RotorSensor_InitUnits(const Encoder_RotorSensor_T * p_sensor)
{
    const RotorSensor_UnitRef_T * p_unitRef = &p_sensor->BASE.P_STATE->UnitRef;
    p_sensor->ENCODER.P_STATE->Config.AngleFreqBase = p_unitRef->AngleFreqBase / p_unitRef->PolePairs;
    Encoder_MT_InitUnits(&p_sensor->ENCODER);
}


const RotorSensor_VTable_T ENCODER_VTABLE =
{
    .INIT = (RotorSensor_Proc_T)Encoder_RotorSensor_Init,
    .INIT_UNITS = (RotorSensor_Proc_T)Encoder_RotorSensor_InitUnits,
    .CAPTURE_ANGLE = (RotorSensor_Proc_T)Encoder_RotorSensor_CaptureAngle,
    .CAPTURE_SPEED = (RotorSensor_Proc_T)Encoder_RotorSensor_CaptureSpeed,
    .VERIFY_CALIBRATION = (RotorSensor_Test_T)Encoder_RotorSensor_VerifyCalibration,
    .IS_FEEDBACK_AVAILABLE = (RotorSensor_Test_T)Encoder_RotorSensor_IsSensorAvailable,
    .ZERO_INITIAL = (RotorSensor_Proc_T)Encoder_RotorSensor_ZeroSensor,
};






