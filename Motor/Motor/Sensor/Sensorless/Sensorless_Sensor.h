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
    invokes Sensorless_Sensor_Proc(p, p_foc) once per control tick, after the
    FOC pipeline has captured i_abc and written v_αβ (i.e. after
    Motor_FOC_AngleControl). The vtable's CAPTURE_ANGLE/CAPTURE_SPEED publish
    observer state into RotorSensor_State idempotently.

    Only the sensorless start-up substates (Motor_Sensorless.c) drive Proc;
    RotorSensor_CaptureAngle alone would publish a frozen θ̂.
*/
/******************************************************************************/
#include "../RotorSensor.h"
#include "../../Math/FOC_Sensorless.h"
#include "../../Math/FOC.h"


typedef const struct Sensorless_Sensor
{
    const RotorSensor_T BASE;
    FOC_Sensorless_T * P_OBSERVER;
    const FOC_SensorlessConfig_T * P_NVM_CONFIG;
}
Sensorless_Sensor_T;

/* Observer live state — file-scope static duration, held by the const table entry. */
#define SENSORLESS_OBSERVER_ALLOC() (&(FOC_Sensorless_T){0})

extern const RotorSensor_VTable_T SENSORLESS_SENSOR_VTABLE;

#define SENSORLESS_SENSOR_INIT(p_RotorState, p_Observer, p_NvmConfig) (Sensorless_Sensor_T) \
{                                                                           \
    .BASE = ROTOR_SENSOR_INIT(&SENSORLESS_SENSOR_VTABLE, (p_RotorState)),   \
    .P_OBSERVER = (p_Observer),                                             \
    .P_NVM_CONFIG = (p_NvmConfig),                                          \
}


/******************************************************************************/
/*!
    Push-driven entrypoints.

    Step reads i_αβ from FOC (captured this tick) and v_αβ from the observer's
    own store (applied last tick). CaptureVoltage then stores the v_αβ this tick
    commits, for the next Step. Order is load-bearing — CaptureVoltage before
    Step would feed the observer a v that has not been applied yet.
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

/*
    One control tick of the observer. Call after the FOC proc has run Clarke on
    the sampled i_abc and InvClarkePark onto v_αβ — a single Motor_FOC_AngleControl
    spans both, so both halves land here.
*/
static inline void Sensorless_Sensor_Proc(const Sensorless_Sensor_T * p_sensor, const FOC_T * p_foc)
{
    Sensorless_Sensor_Step(p_sensor, p_foc);
    Sensorless_Sensor_CaptureVoltage(p_sensor, p_foc->Valpha, p_foc->Vbeta);
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
