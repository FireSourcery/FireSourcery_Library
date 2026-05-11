#pragma once

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
    @file   Sensorless_Sensor.h
    @author FireSourcery
    @brief  RotorSensor adapter wrapping FOC_Sensorless as the angle source.

    Differs from Hall/Encoder/SinCos: the underlying engine is push-driven (it
    needs i_αβ from FOC, not a peripheral). The integration layer therefore
    invokes Sensorless_Sensor_Step(p, i_α, i_β) explicitly each control tick;
    the vtable's CAPTURE_ANGLE/CAPTURE_SPEED only publish already-computed
    observer state into RotorSensor_State.
*/
/******************************************************************************/
#include "../RotorSensor.h"
#include "../../Math/FOC_Sensorless.h"


typedef const struct Sensorless_Sensor
{
    const RotorSensor_T BASE;
    FOC_Sensorless_T * P_OBSERVER;
}
Sensorless_Sensor_T;

extern const RotorSensor_VTable_T SENSORLESS_SENSOR_VTABLE;

#define SENSORLESS_SENSOR_INIT(p_RotorState, p_ObserverState, p_ObserverConfig) (Sensorless_Sensor_T) \
{                                                                           \
    .BASE = ROTOR_SENSOR_INIT(&SENSORLESS_SENSOR_VTABLE, (p_RotorState)),   \
    .P_OBSERVER = &(FOC_Sensorless_T){0}, \
}


/******************************************************************************/
/*!
    Push-driven entrypoint — call once per control tick after Clarke on i_abc.
    Runs the observer step, then publishes θ̂ and ω̂

    Caller pushes v_αβ for next cycle via Sensorless_Sensor_CaptureVoltage
    (typically right after foc_inv_clarke_park / SVPWM).
*/
/******************************************************************************/
/* Push-driven step. */
static inline void Sensorless_Sensor_Step(Sensorless_Sensor_T * p_sensor, fract16_t i_alpha, fract16_t i_beta)
{
    RotorSensor_State_T * p_rotor = p_sensor->BASE.P_STATE;
    FOC_Sensorless_Step(p_sensor->P_OBSERVER, i_alpha, i_beta);
    /* push to common interface state */
    p_rotor->AngleSpeed.Angle = p_sensor->P_OBSERVER->AngleSpeed.Angle;
    p_rotor->AngleSpeed.Delta = p_sensor->P_OBSERVER->AngleSpeed.Delta;
    // FOC_Sensorless_Step(p_sensor->P_OBSERVER, &p_rotor->AngleSpeed.Angle, i_alpha, i_beta);
}

static inline void Sensorless_Sensor_CaptureVoltage(Sensorless_Sensor_T * p_sensor, fract16_t v_alpha, fract16_t v_beta)
{
    FOC_Sensorless_CaptureVoltage(p_sensor->P_OBSERVER, v_alpha, v_beta);
}

/* Bumpless transfer from open-loop ramp into closed-loop observer tracking. */
static inline void Sensorless_Sensor_SeedAngle(Sensorless_Sensor_T * p_sensor, angle16_t theta, angle16_t delta)
{
    FOC_Sensorless_SeedAngle(p_sensor->P_OBSERVER, theta, delta);
}

static inline const FOC_Sensorless_T * Sensorless_Sensor_GetObserver(Sensorless_Sensor_T * p_sensor)
{
    return p_sensor->P_OBSERVER;
}
