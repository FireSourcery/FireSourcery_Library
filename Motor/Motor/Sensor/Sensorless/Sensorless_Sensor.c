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



/******************************************************************************/
/*!
    Vtable methods
*/
/******************************************************************************/
/* PID/lifecycle init. Config is owned by FocSensorless and populated by
   Motor.c::Init via FOC_Sensorless_Init(&FocSensorless, &Config.SensorlessConfig);
   here we only (re)bind the PID and reset per-tick state. */
static void Sensorless_Sensor_Init(const Sensorless_Sensor_T * p_sensor)
{
    FOC_Sensorless_Init(p_sensor->P_OBSERVER, NULL);
}

/* Idempotent publish — Step already mirrors observer state into the FOC_Sensorless
   AngleSpeed; here we forward it to the RotorSensor common interface state. */
static void Sensorless_Sensor_CaptureAngle(const Sensorless_Sensor_T * p_sensor)
{
    RotorSensor_State_T * p_rotor = p_sensor->BASE.P_STATE;
    p_rotor->AngleSpeed.Angle = FOC_Sensorless_GetAngle(p_sensor->P_OBSERVER);
    p_rotor->AngleSpeed.Delta = FOC_Sensorless_GetDelta(p_sensor->P_OBSERVER);
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

/* Observer is "calibrated" if the tuning gains are populated. K_smo == 0
   means K·sat(·) produces no forcing, and LpfCoef == 0 freezes the EMF
   estimate — either alone leaves the observer non-functional. */
static bool Sensorless_Sensor_VerifyCalibration(const Sensorless_Sensor_T * p_sensor)
{
    const FOC_SensorlessConfig_T * c = &p_sensor->P_OBSERVER->Config;
    return (c->K_smo != 0) && (c->LpfCoef != 0);
}

static void Sensorless_Sensor_InitFrom(const Sensorless_Sensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    // FOC_Sensorless_Init(&p_motor->FocSensorless, &p_motor->Config.SensorlessConfig);
    // FOC_Sensorless_InitG(&p_motor->FocSensorless, smo_g_pu_rpm(MOTOR_I_LOOP_FREQ, Motor_GetSpeedTypeMax_Rpm(&p_motor->Config.SpeedRating), p_motor->Config.SpeedRating.PolePairs, (p_motor->Foc.Config.Electrical.Ld + p_motor->Foc.Config.Electrical.Lq) / 2));
    // // FOC_Sensorless_InitG(&p_motor->FocSensorless, smo_g_pu_of_angle(MOTOR_I_LOOP_FREQ, Motor_GetSpeedTypeMax_Angle(&p_motor->Config.SpeedRating), (p_motor->Config.ElectricalParams_Pu.Ld + p_motor->Config.ElectricalParams_Pu.Lq) / 2));
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
