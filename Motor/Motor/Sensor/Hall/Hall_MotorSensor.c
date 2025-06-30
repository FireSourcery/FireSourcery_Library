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
    @file   Hall_MotorSensor.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Hall_MotorSensor.h"



static void Hall_MotorSensor_Init(const Hall_MotorSensor_T * p_sensor)
{
    Hall_Init(&p_sensor->HALL);
    Encoder_ModeDT_Init_Polling(p_sensor->P_ENCODER);
#if     defined(CONFIG_MOTOR_HALL_MODE_ISR)
    Encoder_InitInterrupts_ABC(p_sensor->P_ENCODER);
#endif
    // Encoder_SetUnitsHall_MechSpeed(p_sensor->P_ENCODER, );

    Encoder_ModeDT_SetInitial(p_sensor->P_ENCODER);
}

static void Hall_MotorSensor_CaptureAngle(const Hall_MotorSensor_T * p_sensor)
{
    MotorSensor_State_T * p_state = p_sensor->MOTOR_SENSOR.P_STATE;

    if (Hall_PollCaptureSensors(&p_sensor->HALL) == true)
    {
        Encoder_SinglePhase_CapturePulse(p_sensor->P_ENCODER); // Encoder_CaptureCount
        p_state->ElectricalAngle = Hall_CaptureAngle(p_sensor->HALL.P_STATE);
        p_state->Direction = Hall_GetDirection(p_sensor->HALL.P_STATE);

        /* on direction diff */

    }
    else
    {
        p_state->ElectricalAngle = Hall_GetAngle16(p_sensor->HALL.P_STATE) + Encoder_ModeDT_InterpolateAngularDisplacement(p_sensor->P_ENCODER);

        // if capture mech anagle
        p_state->MechanicalAngle += p_sensor->P_ENCODER->P_STATE->PollingAngleDelta * p_sensor->P_ENCODER->P_STATE->Config.PartitionsPerRevolution;
    }
}

static void Hall_MotorSensor_CaptureSpeed(const Hall_MotorSensor_T * p_sensor)
{
    MotorSensor_State_T * p_state = p_sensor->MOTOR_SENSOR.P_STATE;

    Encoder_ModeDT_CaptureVelocity(p_sensor->P_ENCODER);
    p_state->ElectricalSpeed_DegPerCycle = Encoder_ModeDT_CapturePollingAngle(p_sensor->P_ENCODER->P_STATE);
    // p_state->Speed_Fract16 = Encoder_ModeDT_GetScalarVelocity(p_sensor->P_ENCODER->P_STATE);
    p_state->Speed_Fract16 = (Encoder_ModeDT_GetScalarVelocity(p_sensor->P_ENCODER->P_STATE) + p_state->Speed_Fract16) / 2;
}

// uint16_t Hall_MotorSensor_CaptureState(const MotorSensor_T * p_sensor)
// {

// }

static bool Hall_MotorSensor_IsFeedbackAvailable(const Hall_MotorSensor_T * p_sensor) { return true; }

static bool Hall_MotorSensor_VerifyCalibration(const Hall_MotorSensor_T * p_sensor) { return Hall_IsTableValid(p_sensor->HALL.P_STATE); }

static int Hall_MotorSensor_GetDirection(const Hall_MotorSensor_T * p_sensor) { return Hall_GetDirection(p_sensor->HALL.P_STATE); }

static void Hall_MotorSensor_SetDirection(const Hall_MotorSensor_T * p_sensor, int direction)
{
    Hall_SetDirection(p_sensor->HALL.P_STATE, (Hall_Direction_T)direction);
    Encoder_SinglePhase_SetDirection(p_sensor->P_ENCODER->P_STATE, direction);  /* interpolate as +/- */
}

static void Hall_MotorSensor_SetInitial(const Hall_MotorSensor_T * p_sensor)
{
    Hall_SetInitial(&p_sensor->HALL);
    Encoder_ModeDT_SetInitial(p_sensor->P_ENCODER);
    // Hall_MotorSensor_SetDirection(p_sensor, p_sensor->MOTOR_SENSOR.P_STATE->Direction);
}

/*!
    Hall sensors as speed encoder.
    CPR = PolePairs*6   => GetSpeed => mechanical speed
    CPR = 6             => GetSpeed => electrical speed
*/
void Hall_MotorSensor_InitUnits_ElSpeed(const Hall_MotorSensor_T * p_sensor, const MotorSensor_Config_T * p_config)
{
    Encoder_State_T * p_encoder = p_sensor->P_ENCODER->P_STATE;
    p_encoder->Config.IsQuadratureCaptureEnabled = false;

    // speed_rpm_of_angle16(MOTOR_CONTROL_FREQ, p_motor->Config.SpeedRated_DegPerCycle) / p_motor->Config.PolePairs
    Encoder_SetScalarSpeedRef(p_encoder, p_config->ElSpeedRated_DegPerCycle); // ERpm
    Encoder_SetCountsPerRevolution(p_encoder, 6U);
    Encoder_SetPartitionsPerRevolution(p_encoder, 1U);

    // if (p_encoder->Config.ScalarSpeedRef_Rpm != speedRef_ERpm) { Encoder_SetScalarSpeedRef(p_encoder, speedRef_ERpm); }
    // if (p_encoder->Config.CountsPerRevolution != 6U) { Encoder_SetCountsPerRevolution(p_encoder, 6U); }
    // if (p_encoder->Config.PartitionsPerRevolution != 1U) { Encoder_SetPartitionsPerRevolution(p_encoder, 1U); }
}

void Hall_MotorSensor_InitUnits_MechSpeed(const Hall_MotorSensor_T * p_sensor, const MotorSensor_Config_T * p_config)
{
    Encoder_State_T * p_encoder = p_sensor->P_ENCODER->P_STATE;
    p_encoder->Config.IsQuadratureCaptureEnabled = false;

    Encoder_SetScalarSpeedRef(p_encoder, p_config->MechSpeedRated_Rpm); /* mech rpm */
    Encoder_SetCountsPerRevolution(p_encoder, p_config->PolePairs * 6U);
    Encoder_SetPartitionsPerRevolution(p_encoder, p_config->PolePairs);  /* Set for electrical cycle */

    // if (p_encoder->Config.ScalarSpeedRef_Rpm != speedRef_Rpm) { Encoder_SetScalarSpeedRef(p_encoder, speedRef_Rpm); }
    // if (p_encoder->Config.CountsPerRevolution != polePairs * 6U) { Encoder_SetCountsPerRevolution(p_encoder, polePairs * 6U); }
    // if (p_encoder->Config.PartitionsPerRevolution != polePairs) { Encoder_SetPartitionsPerRevolution(p_encoder, polePairs); } /* Set for electrical cycle */
}


/*
    Interface VTable
*/
const MotorSensor_VTable_T HALL_VTABLE =
{
    .INIT = (MotorSensor_Proc_T)Hall_MotorSensor_Init,
    .CAPTURE_ANGLE = (MotorSensor_Proc_T)Hall_MotorSensor_CaptureAngle,
    .CAPTURE_SPEED = (MotorSensor_Proc_T)Hall_MotorSensor_CaptureSpeed,
    .IS_FEEDBACK_AVAILABLE = (MotorSensor_Test_T)Hall_MotorSensor_IsFeedbackAvailable,
    .SET_DIRECTION = (MotorSensor_Set_T)Hall_MotorSensor_SetDirection,
    .GET_DIRECTION = (MotorSensor_Get_T)Hall_MotorSensor_GetDirection,
    // .SetInitial = (MotorSensor_Proc_T)Hall_SetInitial,
    .INIT_UNITS_FROM = (MotorSensor_InitFrom_T)Hall_MotorSensor_InitUnits_MechSpeed,
    // .INIT_UNITS_FROM = (MotorSensor_InitFrom_T)Hall_MotorSensor_InitUnits_ElSpeed,
    .VERIFY_CALIBRATION = (MotorSensor_Test_T)Hall_MotorSensor_VerifyCalibration,
};


