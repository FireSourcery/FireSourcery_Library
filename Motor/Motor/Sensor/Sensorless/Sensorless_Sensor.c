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
/* Tuning from NVM, and per-tick state cleared. The winding is not tuning, see [EmfObserver_Config_T] */
static void Sensorless_Sensor_Init(Sensorless_Sensor_T * p_sensor)
{
    EmfObserver_Init(p_sensor->P_OBSERVER, p_sensor->P_NVM_CONFIG);
}

/* Idempotent publish — Proc already advances θ̂, ω̂ in the observer's [AnglePll_T];
   here we forward it to the RotorSensor common interface state. */
static void Sensorless_Sensor_CaptureAngle(Sensorless_Sensor_T * p_sensor)
{
    RotorSensor_State_T * p_rotor = p_sensor->BASE.P_STATE;
    p_rotor->AngleSpeed.Angle = p_sensor->P_OBSERVER->Pll.Base.Angle;
    _RotorSensor_CaptureDelta(p_rotor, p_sensor->P_OBSERVER->Pll.Base.Delta);
}

/* Project observer ω̂ → fract16 speed via the configured SpeedPuRef.
   Assumes CAPTURE_ANGLE already published Delta into AngleSpeed this tick. */
static void Sensorless_Sensor_CaptureSpeed(Sensorless_Sensor_T * p_sensor)
{
    RotorSensor_State_T * p_rotor = p_sensor->BASE.P_STATE;
    // p_rotor->Speed_Pu = Angle_ResolveSpeed_Pu(&p_rotor->AngleSpeed, &p_rotor->UnitRef.SpeedPuRef);
    _RotorSensor_CaptureSpeedPu(p_rotor, Angle_ResolveSpeed_Pu(&p_rotor->AngleSpeed, &p_rotor->UnitRef.SpeedPuRef));
}

static bool Sensorless_Sensor_IsFeedbackAvailable(Sensorless_Sensor_T * p_sensor)
{
    return EmfObserver_IsLocked(p_sensor->P_OBSERVER);
}

static void Sensorless_Sensor_ZeroInitial(Sensorless_Sensor_T * p_sensor)
{
    EmfObserver_Reset(p_sensor->P_OBSERVER);
}

/* Observer is "calibrated" if the tuning gains and the plant are populated. K_smo == 0
   means K·sat(·) produces no forcing, LpfCoef == 0 freezes the EMF
   estimate, and G == 0 (no plant) holds î — each alone leaves the observer non-functional. */
static bool Sensorless_Sensor_VerifyCalibration(Sensorless_Sensor_T * p_sensor)
{
    const EmfObserver_Config_T * p_config = &p_sensor->P_OBSERVER->Config;
    return (p_config->K_smo != 0) && (p_config->LpfCoef != 0) && (p_sensor->P_OBSERVER->Stator.Winding.G != 0);
}

/* No sensor-local unit state: the observer works entirely in the motor's per-unit
   basis. UnitRef.SpeedPuRef — the only field CAPTURE_SPEED reads — is held by the
   RotorSensor base, and the winding needs the motor's electrical model. */
static void Sensorless_Sensor_InitUnits(Sensorless_Sensor_T * p_sensor)
{
    (void)p_sensor;
}


const RotorSensor_VTable_T SENSORLESS_SENSOR_VTABLE =
{
    .INIT                  = (RotorSensor_Proc_T)Sensorless_Sensor_Init,
    .INIT_UNITS            = (RotorSensor_Proc_T)Sensorless_Sensor_InitUnits,
    .CAPTURE_ANGLE         = (RotorSensor_Proc_T)Sensorless_Sensor_CaptureAngle,
    .CAPTURE_SPEED         = (RotorSensor_Proc_T)Sensorless_Sensor_CaptureSpeed,
    .IS_FEEDBACK_AVAILABLE = (RotorSensor_Test_T)Sensorless_Sensor_IsFeedbackAvailable,
    .ZERO_INITIAL          = (RotorSensor_Proc_T)Sensorless_Sensor_ZeroInitial,
    .VERIFY_CALIBRATION    = (RotorSensor_Test_T)Sensorless_Sensor_VerifyCalibration,
};
