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
    needs i_αβ and v_αβ from FOC, not a peripheral). The integration layer
    invokes Sensorless_Sensor_Step(p, p_foc) once per control tick (reads
    Iαβ from FOC, observer's own VαβPrev) and Sensorless_Sensor_CaptureVoltage
    after InvPark. The vtable's CAPTURE_ANGLE/CAPTURE_SPEED publish observer
    state into RotorSensor_State idempotently.
*/
/******************************************************************************/
#include "../RotorSensor.h"
#include "../../Math/FOC_Sensorless.h"
#include "../../Math/FOC.h"


typedef const struct Sensorless_Sensor
{
    const RotorSensor_T BASE;
    FOC_Sensorless_T * P_OBSERVER;
}
Sensorless_Sensor_T;

extern const RotorSensor_VTable_T SENSORLESS_SENSOR_VTABLE;

#define SENSORLESS_SENSOR_INIT(p_RotorState, p_Observer) (Sensorless_Sensor_T) \
{                                                                           \
    .BASE = ROTOR_SENSOR_INIT(&SENSORLESS_SENSOR_VTABLE, (p_RotorState)),   \
    .P_OBSERVER = (p_Observer),                                             \
}


/******************************************************************************/
/*!
    Push-driven entrypoint — call once per control tick after Clarke on i_abc
    (i.e. after FOC_CaptureIabc / FOC_ProcClarkePark populates p_foc->Iαβ).
    Observer reads i from FOC and v from its own VαβPrev (stored last tick).

    Caller pushes v_αβ for next cycle via Sensorless_Sensor_CaptureVoltage
    (after FOC_ProcInvClarkePark / SVPWM commit).
*/
/******************************************************************************/
/* Push-driven step. */
static inline void Sensorless_Sensor_Step(const Sensorless_Sensor_T * p_sensor, const FOC_T * p_foc)
{
    FOC_Sensorless_Step(p_foc, p_sensor->P_OBSERVER);
}

static inline void Sensorless_Sensor_CaptureVoltage(const Sensorless_Sensor_T * p_sensor, fract16_t v_alpha, fract16_t v_beta)
{
    FOC_Sensorless_CaptureVoltage(p_sensor->P_OBSERVER, v_alpha, v_beta);
}

/* Bumpless transfer from open-loop ramp into closed-loop observer tracking. */
static inline void Sensorless_Sensor_SeedAngle(const Sensorless_Sensor_T * p_sensor, angle16_t theta, angle16_t delta)
{
    FOC_Sensorless_SeedAngle(p_sensor->P_OBSERVER, theta, delta);
}

static inline const FOC_Sensorless_T * Sensorless_Sensor_GetObserver(const Sensorless_Sensor_T * p_sensor)
{
    return p_sensor->P_OBSERVER;
}
