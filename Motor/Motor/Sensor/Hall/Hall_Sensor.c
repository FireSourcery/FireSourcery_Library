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
    Angle_T * p_angle = &p_state->AngleSpeed;

    if (Hall_PollCaptureSensors(&p_sensor->HALL) == true) /* Electrical Cycle, generally > 1ms */
    {
        Encoder_CaptureCount(p_sensor->P_ENCODER, Hall_CaptureDirection(p_sensor->HALL.P_STATE)); /* captured signed count */
        Angle_CaptureAngle(p_angle, Hall_CaptureAngle(p_sensor->HALL.P_STATE));
        Angle_ZeroInterpolation(p_angle); /* Reset interpolation on every edge */

        /*
            Direction mismatch: sensor direction != feedback direction from speed last capture.
            Speed capture should filter this out. Zero interpolation to prevent extrapolation in wrong direction.
        */
        if (Hall_GetDirection(p_sensor->HALL.P_STATE) != RotorSensor_GetFeedbackDirection(&p_sensor->BASE))
        {
            p_sensor->P_ENCODER->P_STATE->PollingAngleDelta = 0;
            p_angle->Interpolation.Delta = 0;
        }
    }
    else /* 20Khz */
    {
        /* Interpolate angle between Hall edges */
        p_angle->Angle = Hall_GetAngle16(p_sensor->HALL.P_STATE) + Angle_Interpolate(p_angle);

        /* Optionally accumulate mechanical angle */
        p_state->MechanicalAngle += p_sensor->P_ENCODER->P_STATE->PollingAngleDelta * p_sensor->P_ENCODER->P_STATE->Config.PartitionsPerRevolution;

        // p_angle->Angle = Hall_GetAngle16(p_sensor->HALL.P_STATE) + Encoder_ModeDT_InterpolateAngle(p_sensor->P_ENCODER);
        // p_state->MechanicalAngle += p_angle->Delta * p_sensor->BASE.P_STATE->Config.PolePairs; /* accumulate mech angle from electrical angle delta */
    }
}

/* 1ms */
static void Hall_RotorSensor_CaptureSpeed(const Hall_RotorSensor_T * p_sensor)
{
    Angle_T * p_angle = &p_sensor->BASE.P_STATE->AngleSpeed; /* use AngleSpeed for capture, and GetAngle can select angle/speed/other result from AngleSpeed struct */
    Encoder_ModeDT_CaptureFreqD(p_sensor->P_ENCODER);
    // Encoder_ModeDT_CapturePollingDelta(p_sensor->P_ENCODER->P_STATE);
    Angle_CaptureSpeed_Fract16(p_angle, Encoder_ModeDT_GetScalarSpeed(p_sensor->P_ENCODER->P_STATE)); /* FreqD captured with direction */
    Angle_CaptureInterpolationDelta(p_angle); /* Captures Delta as   */

}


static bool Hall_RotorSensor_IsFeedbackAvailable(const Hall_RotorSensor_T * p_sensor) { return true; }


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

    /* RotorSensor calls Angle_InitSpeedRef */

    /* Init interpolation limit from sector angle */
    Angle_InitInterpolation(&p_sensor->BASE.P_STATE->AngleSpeed, ANGLE16_PER_REVOLUTION / 6);

    // Angle_CounterConfig_T calib =
    // {
    //     .CountsPerRevolution = p_config->PolePairs * 6U,
    //     .PartitionsPerRevolution = p_config->PolePairs,  /* Set for electrical cycle */
    //     .ScalarSpeedRef_Rpm = p_config->SpeedTypeMax_Rpm, /* mech rpm */
    // };
    // Angle_Counter_InitRef(p_sensor->P_COUNTER_REF, &calib, p_sensor->P_PULSE->TIMER_FREQ, p_sensor->P_PULSE->SAMPLE_FREQ, p_sensor->POLLING_FREQ);
    // Angle_Counter_InitSpeedFract(p_sensor->P_COUNTER_REF, &calib, p_sensor->P_PULSE->TIMER_FREQ, p_sensor->P_PULSE->SAMPLE_FREQ, p_sensor->POLLING_FREQ);
}

// void Hall_RotorSensor_InitUnits_ElSpeed(const Hall_RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config)
// {
//     Encoder_State_T * p_encoder = p_sensor->P_ENCODER->P_STATE;
//     p_encoder->Config.IsQuadratureCaptureEnabled = false;

//     Encoder_SetCountsPerRevolution(p_encoder, 6U);
//     Encoder_SetPartitionsPerRevolution(p_encoder, 1U);
//     // rpm_of_angle(MOTOR_CONTROL_FREQ, p_motor->Config.SpeedRated_DegPerCycle) / p_motor->Config.PolePairs
//     // Encoder_SetScalarSpeedRef(p_encoder,   ); // ERpm
// }

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
    // .SET_DIRECTION = (RotorSensor_Set_T)Hall_RotorSensor_SetDirection,
    // .GET_DIRECTION = (RotorSensor_Get_T)Hall_RotorSensor_GetDirection,
    .ZERO_INITIAL = (RotorSensor_Proc_T)Hall_RotorSensor_ZeroInitial,
    .VERIFY_CALIBRATION = (RotorSensor_Test_T)Hall_RotorSensor_VerifyCalibration,
};


