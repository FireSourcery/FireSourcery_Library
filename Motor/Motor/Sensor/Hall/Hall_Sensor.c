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
    Encoder_ModeDT_Init_Polling(p_sensor->P_ENCODER);
#if     defined(MOTOR_HALL_MODE_ISR)
    Encoder_InitInterrupts_ABC(p_sensor->P_ENCODER);
#endif
    // Encoder_SetUnitsHall_MechSpeed(p_sensor->P_ENCODER, );
    Encoder_ModeDT_SetInitial(p_sensor->P_ENCODER);
}

static void Hall_RotorSensor_CaptureAngle(const Hall_RotorSensor_T * p_sensor)
{
    RotorSensor_State_T * p_state = p_sensor->MOTOR_SENSOR.P_STATE;
    Angle_T * p_angle = RotorSensor_GetAngleState(&p_sensor->MOTOR_SENSOR);

// #if defined(ROTOR_HALL_MODE_POLLING)
    if (Hall_PollCaptureSensors(&p_sensor->HALL) == true)
    {
        Encoder_SinglePhase_CapturePulse(p_sensor->P_ENCODER);
        // _Encoder_CaptureCount(p_encoder->P_STATE, 1);
        // Encoder_DeltaT_CaptureExtended(p_encoder);
        // Encoder_ZeroInterpolateAngle(p_encoder->P_STATE)

        Angle_CaptureAngle(p_angle, Hall_CaptureAngle(p_sensor->HALL.P_STATE));  // p_angle->ElectricalAngle = Hall_CaptureAngle(p_sensor->HALL.P_STATE);

        //
        // Hall_CaptureDirection(p_sensor->HALL.P_STATE);
        // if ( sign(speed) != Hall_GetDirection(p_sensor->HALL.P_STATE))
        // if (p_state->Direction != Hall_GetDirection(p_sensor->HALL.P_STATE))
        // {
        //     // stop interpolation
        //     // Hall_SetDirection(p_sensor->HALL.P_STATE, 0); /* with Hall_CaptureDirection, it will reset the next cycle */
        //     p_sensor->P_ENCODER->P_STATE->PollingAngleDelta = 0;
        //     p_sensor->P_ENCODER->P_STATE->InterpolateAngleSum = 0;

        //     // count towards fault
        //     p_state->DirectionErrorCount++;
        //     p_state->Direction = Hall_GetDirection(p_sensor->HALL.P_STATE); // p_state->Direction as common feedback direction, alternatively use speed
        // }
    }
    else
// #endif
    {
        // todo with Interpolate Angle
        p_angle->Angle = Hall_GetAngle16(p_sensor->HALL.P_STATE) + Encoder_ModeDT_InterpolateAngularDisplacement(p_sensor->P_ENCODER);
        // p_state->ElectricalAngle = Hall_GetAngle16(p_sensor->HALL.P_STATE) + Encoder_ModeDT_InterpolateAngularDisplacement(p_sensor->P_ENCODER);

        // if capture mech anagle
        p_state->MechanicalAngle += p_sensor->P_ENCODER->P_STATE->PollingAngleDelta * p_sensor->P_ENCODER->P_STATE->Config.PartitionsPerRevolution;
    }
}

static void Hall_RotorSensor_CaptureSpeed(const Hall_RotorSensor_T * p_sensor)
{
    RotorSensor_State_T * p_state = p_sensor->MOTOR_SENSOR.P_STATE;
    Angle_T * p_angle = RotorSensor_GetAngleState(&p_sensor->MOTOR_SENSOR);

    // Encoder_ModeDT_CaptureFreqD(p_sensor->P_ENCODER);
    // p_state->ElectricalSpeed_DegPerCycle = Encoder_ModeDT_CapturePollingDelta(p_sensor->P_ENCODER->P_STATE);
    // p_angle->Speed_Fract16 = (Encoder_ModeDT_GetScalarVelocity(p_sensor->P_ENCODER->P_STATE) + p_state->Speed_Fract16) / 2;
    Encoder_ModeDT_CaptureFreqD(p_sensor->P_ENCODER);
    Angle_CaptureDelta(p_angle, Encoder_ModeDT_CapturePollingDelta(p_sensor->P_ENCODER->P_STATE));
    Angle_CaptureSpeed_Fract16(p_angle, Encoder_ModeDT_GetScalarVelocity(p_sensor->P_ENCODER->P_STATE));
}


static bool Hall_RotorSensor_IsFeedbackAvailable(const Hall_RotorSensor_T * p_sensor) { return true; }

// static int Hall_RotorSensor_GetDirection(const Hall_RotorSensor_T * p_sensor) { return Hall_GetDirection(p_sensor->HALL.P_STATE); }

static void Hall_RotorSensor_SetDirection(const Hall_RotorSensor_T * p_sensor, int direction)
{
    p_sensor->MOTOR_SENSOR.P_STATE->Direction = direction;
    Hall_SetDirection(p_sensor->HALL.P_STATE, (Hall_Direction_T)direction);
    Encoder_SinglePhase_SetDirection(p_sensor->P_ENCODER->P_STATE, direction);  /* interpolate as +/-, and on get speed */
}

static void Hall_RotorSensor_SetInitial(const Hall_RotorSensor_T * p_sensor)
{
    Hall_ZeroInitial(&p_sensor->HALL);
    Encoder_ModeDT_SetInitial(p_sensor->P_ENCODER);
    // Hall_RotorSensor_SetDirection(p_sensor, p_sensor->MOTOR_SENSOR.P_STATE->Direction);
}

static bool Hall_RotorSensor_VerifyCalibration(const Hall_RotorSensor_T * p_sensor) { return Hall_IsTableValid(p_sensor->HALL.P_STATE); }

/*!
    Hall sensors as speed encoder.
    Config ScalarSpeed
    CPR = PolePairs*6   => GetSpeed => mechanical speed
    CPR = 6             => GetSpeed => electrical speed
*/
void Hall_RotorSensor_InitUnits_ElSpeed(const Hall_RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    Encoder_State_T * p_encoder = p_sensor->P_ENCODER->P_STATE;
    p_encoder->Config.IsQuadratureCaptureEnabled = false;

    Encoder_SetCountsPerRevolution(p_encoder, 6U);
    Encoder_SetPartitionsPerRevolution(p_encoder, 1U);
    // rpm_of_angle(MOTOR_CONTROL_FREQ, p_motor->Config.SpeedRated_DegPerCycle) / p_motor->Config.PolePairs
    // Encoder_SetScalarSpeedRef(p_encoder,   ); // ERpm

    // if (p_encoder->Config.ScalarSpeedRef_Rpm != speedRef_ERpm) { Encoder_SetScalarSpeedRef(p_encoder, speedRef_ERpm); }
    // if (p_encoder->Config.CountsPerRevolution != 6U) { Encoder_SetCountsPerRevolution(p_encoder, 6U); }
    // if (p_encoder->Config.PartitionsPerRevolution != 1U) { Encoder_SetPartitionsPerRevolution(p_encoder, 1U); }
}

void Hall_RotorSensor_InitUnits_MechSpeed(const Hall_RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    Encoder_State_T * p_encoder = p_sensor->P_ENCODER->P_STATE;
    p_encoder->Config.IsQuadratureCaptureEnabled = false;

    Encoder_SetCountsPerRevolution(p_encoder, p_config->PolePairs * 6U);
    Encoder_SetPartitionsPerRevolution(p_encoder, p_config->PolePairs);  /* Set for electrical cycle */
    Encoder_SetScalarSpeedRef(p_encoder, p_config->MechSpeedRated_Rpm); /* mech rpm */

    // if (p_encoder->Config.ScalarSpeedRef_Rpm != speedRef_Rpm) { Encoder_SetScalarSpeedRef(p_encoder, speedRef_Rpm); }
    // if (p_encoder->Config.CountsPerRevolution != polePairs * 6U) { Encoder_SetCountsPerRevolution(p_encoder, polePairs * 6U); }
    // if (p_encoder->Config.PartitionsPerRevolution != polePairs) { Encoder_SetPartitionsPerRevolution(p_encoder, polePairs); } /* Set for electrical cycle */
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
    .ZERO_INITIAL = (RotorSensor_Proc_T)Hall_RotorSensor_SetInitial,
    .VERIFY_CALIBRATION = (RotorSensor_Test_T)Hall_RotorSensor_VerifyCalibration,
};


