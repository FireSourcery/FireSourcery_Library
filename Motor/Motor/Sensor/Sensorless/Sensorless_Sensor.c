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
    @file   Sensorless_Sensor.c
    @author FireSourcery
*/
/******************************************************************************/
#include "Sensorless_Sensor.h"


/* Project observer state into RotorSensor_State. Idempotent — safe to call
   from CAPTURE_ANGLE / CAPTURE_SPEED whether or not Step ran this tick. */
// static void publish_to_rotor_state(const Sensorless_Sensor_T * p_sensor)
// {
//     RotorSensor_State_T * p_rotor = p_sensor->BASE.P_STATE;
//     angle16_t theta = FOC_Sensorless_GetAngle(p_sensor->P_OBSERVER);
//     angle16_t delta = FOC_Sensorless_GetDelta(p_sensor->P_OBSERVER);
//     Angle_CaptureAngle(&p_rotor->AngleSpeed, theta);
//     Angle_CaptureDelta(&p_rotor->AngleSpeed, delta);
// }

/* Push-driven step. */
void Sensorless_Sensor_Step(const Sensorless_Sensor_T * p_sensor, fract16_t i_alpha, fract16_t i_beta)
{
    RotorSensor_State_T * p_rotor = p_sensor->BASE.P_STATE;
    FOC_Sensorless_Step(p_sensor->P_OBSERVER, i_alpha, i_beta);
    Angle_CaptureAngle(&p_rotor->AngleSpeed, FOC_Sensorless_GetAngle(p_sensor->P_OBSERVER));
    Angle_CaptureDelta(&p_rotor->AngleSpeed, FOC_Sensorless_GetDelta(p_sensor->P_OBSERVER));
}

/******************************************************************************/
/*!
    Vtable methods
*/
/******************************************************************************/
static void Sensorless_Sensor_Init(const Sensorless_Sensor_T * p_sensor)
{
    FOC_Sensorless_Init(p_sensor->P_OBSERVER);
}

/* Idempotent publish. The actual observer step is driven by Sensorless_Sensor_Step. */
static void Sensorless_Sensor_CaptureAngle(const Sensorless_Sensor_T * p_sensor)
{
    RotorSensor_State_T * p_rotor = p_sensor->BASE.P_STATE;
    Angle_CaptureAngle(&p_rotor->AngleSpeed, FOC_Sensorless_GetAngle(p_sensor->P_OBSERVER));
    Angle_CaptureDelta(&p_rotor->AngleSpeed, FOC_Sensorless_GetDelta(p_sensor->P_OBSERVER));
}

/* Project observer ω̂ → fract16 speed via the configured SpeedFractRef.
   Assumes CAPTURE_ANGLE already published Delta into AngleSpeed this tick. */
static void Sensorless_Sensor_CaptureSpeed(const Sensorless_Sensor_T * p_sensor)
{
    RotorSensor_State_T * p_rotor = p_sensor->BASE.P_STATE;
    p_rotor->Speed_Fract16 = Angle_ResolveSpeed_Fract16(&p_rotor->AngleSpeed, &p_rotor->SpeedFractRef);
}

static bool Sensorless_Sensor_IsFeedbackAvailable(const Sensorless_Sensor_T * p_sensor)
{
    return FOC_Sensorless_IsLocked(p_sensor->P_OBSERVER);
}

static void Sensorless_Sensor_ZeroInitial(const Sensorless_Sensor_T * p_sensor)
{
    FOC_Sensorless_ResetState(p_sensor->P_OBSERVER);
}

static bool Sensorless_Sensor_VerifyCalibration(const Sensorless_Sensor_T * p_sensor)
{
    /* Calibrated params are well-formed if Ls_pu and Psi_pu are non-zero
       (Rs_pu may legitimately be zero on perfectly-superconducting stators). */
    const FOC_SensorlessConfig_T * c = &p_sensor->P_OBSERVER->Config;
    return (c->Ls_pu != 0) && (c->Psi_pu != 0);
}

static void Sensorless_Sensor_InitFrom(const Sensorless_Sensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    (void)p_sensor;
    (void)p_config;
}


const RotorSensor_VTable_T SENSORLESS_SENSOR_VTABLE =
{
    .INIT                  = (RotorSensor_Proc_T)Sensorless_Sensor_Init,
    .INIT_UNITS_FROM       = (RotorSensor_InitFrom_T)Sensorless_Sensor_InitFrom,
    .CAPTURE_ANGLE         = (RotorSensor_Proc_T)Sensorless_Sensor_CaptureAngle,
    .CAPTURE_SPEED         = (RotorSensor_Proc_T)Sensorless_Sensor_CaptureSpeed,
    .IS_FEEDBACK_AVAILABLE = (RotorSensor_Test_T)Sensorless_Sensor_IsFeedbackAvailable,
    .ZERO_INITIAL          = (RotorSensor_Proc_T)Sensorless_Sensor_ZeroInitial,
    .VERIFY_CALIBRATION    = (RotorSensor_Test_T)Sensorless_Sensor_VerifyCalibration,
};
