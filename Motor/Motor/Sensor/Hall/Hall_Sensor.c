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
    @file   Hall_RotorSensor.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Hall_Sensor.h"

static void Hall_RotorSensor_Init(const Hall_RotorSensor_T * p_sensor)
{
    Hall_Init(&p_sensor->HALL);
#if defined(MOTOR_HALL_MODE_ISR)
    Encoder_ModeDT_Init_InterruptAbc(p_sensor->P_ENCODER);
#else
    Encoder_ModeDT_Init_Polling(p_sensor->P_ENCODER);
#endif
}

static void Hall_RotorSensor_CaptureAngle(const Hall_RotorSensor_T * p_sensor)
{
    RotorSensor_State_T * p_state = p_sensor->BASE.P_STATE;
    Angle_T * p_angle = RotorSensor_GetAngleState(&p_sensor->BASE);

// #if defined(ROTOR_HALL_MODE_POLLING)
    if (Hall_PollCaptureSensors(&p_sensor->HALL) == true)
    {
        Encoder_SinglePhase_CapturePulse(p_sensor->P_ENCODER); // for speed and interpolation todo move to angle

        p_state->Direction = Hall_CaptureDirection(p_sensor->HALL.P_STATE);
        Angle_CaptureAngle(p_angle, Hall_CaptureAngle(p_sensor->HALL.P_STATE));
        Encoder_SinglePhase_SetDirection(p_sensor->P_ENCODER->P_STATE, p_state->Direction); /* interpolate as +/-, and on get speed */

        // Angle_CaptureAngle(p_angle, Hall_GetSensorAngle(p_sensor->HALL.P_STATE));

        // module handle edge. caller handle count error
        // if (p_state->Direction != Hall_GetSensorDirection(p_sensor->HALL.P_STATE))
        // {
        //     p_sensor->P_ENCODER->P_STATE->PollingAngleDelta = 0;
        //     p_sensor->P_ENCODER->P_STATE->InterpolateAngleSum = 0;
        //     p_state->Direction = Hall_GetDirection(p_sensor->HALL.P_STATE); // p_state->Direction as common feedback direction, alternatively use speed
        // }
    }
    else
// #endif
    {
        // Angle_T todo
        p_angle->Angle = Hall_GetAngle16(p_sensor->HALL.P_STATE) + Encoder_ModeDT_InterpolateAngularDisplacement(p_sensor->P_ENCODER);
        // p_state-> Angle = p_angle->Angle  + delta // may over accumulate
        // if capture mech angle
        p_state->MechanicalAngle += p_sensor->P_ENCODER->P_STATE->PollingAngleDelta * p_sensor->P_ENCODER->P_STATE->Config.PartitionsPerRevolution;
    }
}

static void Hall_RotorSensor_CaptureSpeed(const Hall_RotorSensor_T * p_sensor)
{
    Angle_T * p_angle = RotorSensor_GetAngleState(&p_sensor->BASE);
    Encoder_ModeDT_CaptureFreqD(p_sensor->P_ENCODER);
    Encoder_ModeDT_CapturePollingDelta(p_sensor->P_ENCODER->P_STATE);
    // p_angle->Delta = Encoder_ModeDT_CapturePollingDelta(p_sensor->P_ENCODER->P_STATE);  /* interpolate with Angle_T todo. use 1ms sample */
    Angle_CaptureSpeed_Fract16(p_angle, Encoder_ModeDT_GetScalarVelocity(p_sensor->P_ENCODER->P_STATE));
}


static bool Hall_RotorSensor_IsFeedbackAvailable(const Hall_RotorSensor_T * p_sensor) { return true; }

static int Hall_RotorSensor_GetDirection(const Hall_RotorSensor_T * p_sensor) { return Hall_GetSensorDirection(p_sensor->HALL.P_STATE); }

// set direction comp
// alternatively change to capture on set and this can be empty
static void Hall_RotorSensor_SetDirection(const Hall_RotorSensor_T * p_sensor, int direction)
{
    // p_sensor->BASE.P_STATE->Direction = direction;
    // Hall_SetDirection(p_sensor->HALL.P_STATE, (Hall_Direction_T)direction);
    // Encoder_SinglePhase_SetDirection(p_sensor->P_ENCODER->P_STATE, direction);  /* interpolate as +/-, and on get speed */
}

static void Hall_RotorSensor_ZeroInitial(const Hall_RotorSensor_T * p_sensor)
{
    Hall_ZeroInitial(&p_sensor->HALL);
    Encoder_ModeDT_SetInitial(p_sensor->P_ENCODER);
}

static bool Hall_RotorSensor_VerifyCalibration(const Hall_RotorSensor_T * p_sensor) { return Hall_IsTableValid(p_sensor->HALL.P_STATE); }

/*!
    Hall sensors as speed encoder.
    Config ScalarSpeed
    CPR = PolePairs*6   => GetSpeed => mechanical speed
    CPR = 6             => GetSpeed => electrical speed
*/
void Hall_RotorSensor_InitUnits_MechSpeed(const Hall_RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    Encoder_State_T * p_encoder = p_sensor->P_ENCODER->P_STATE;
    p_encoder->Config.IsQuadratureCaptureEnabled = false;

    Encoder_SetCountsPerRevolution(p_encoder, p_config->PolePairs * 6U);
    Encoder_SetPartitionsPerRevolution(p_encoder, p_config->PolePairs);  /* Set for electrical cycle */
    Encoder_SetScalarSpeedRef(p_encoder, p_config->SpeedTypeMax_Rpm); /* mech rpm */
}

void Hall_RotorSensor_InitUnits_ElSpeed(const Hall_RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    Encoder_State_T * p_encoder = p_sensor->P_ENCODER->P_STATE;
    p_encoder->Config.IsQuadratureCaptureEnabled = false;

    Encoder_SetCountsPerRevolution(p_encoder, 6U);
    Encoder_SetPartitionsPerRevolution(p_encoder, 1U);
    // rpm_of_angle(MOTOR_CONTROL_FREQ, p_motor->Config.SpeedRated_DegPerCycle) / p_motor->Config.PolePairs
    // Encoder_SetScalarSpeedRef(p_encoder,   ); // ERpm
}

/*
    Interface VTable
*/
const RotorSensor_VTable_T HALL_VTABLE =
{
    .INIT = (RotorSensor_Proc_T)Hall_RotorSensor_Init,
    .INIT_UNITS_FROM = (RotorSensor_InitFrom_T)Hall_RotorSensor_InitUnits_MechSpeed,
    // .INIT_UNITS_FROM = (RotorSensor_InitFrom_T)Hall_RotorSensor_InitUnits_ElSpeed,
    .CAPTURE_ANGLE = (RotorSensor_Proc_T)Hall_RotorSensor_CaptureAngle,
    .CAPTURE_SPEED = (RotorSensor_Proc_T)Hall_RotorSensor_CaptureSpeed,
    .IS_FEEDBACK_AVAILABLE = (RotorSensor_Test_T)Hall_RotorSensor_IsFeedbackAvailable,
    .SET_DIRECTION = (RotorSensor_Set_T)Hall_RotorSensor_SetDirection,
    // .GET_DIRECTION = (RotorSensor_Get_T)Hall_RotorSensor_GetDirection,
    .ZERO_INITIAL = (RotorSensor_Proc_T)Hall_RotorSensor_ZeroInitial,
    .VERIFY_CALIBRATION = (RotorSensor_Test_T)Hall_RotorSensor_VerifyCalibration,
};


