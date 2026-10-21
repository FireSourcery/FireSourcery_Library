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
    invokes Sensorless_Sensor_Proc(p, p_context) once per control tick, after the
    FOC proc (i.e. after Motor_FOC_AngleControl). Proc captures the period into the
    observer's αβ image [Stator_AlphaBeta_T] and steps it. The vtable's CAPTURE_ANGLE/CAPTURE_SPEED publish
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
    Stator_AlphaBeta_T * P_STATOR;
    // Observer_T * P_OBSERVER; // spill from Stator_AlphaBeta_T
    const FOC_SensorlessConfig_T * P_NVM_CONFIG;
}
Sensorless_Sensor_T;

/* Observer live state — file-scope static duration, held by the const table entry. */
#define SENSORLESS_OBSERVER_ALLOC() (&(FOC_Sensorless_T){})

extern const RotorSensor_VTable_T SENSORLESS_SENSOR_VTABLE;

#define SENSORLESS_SENSOR_INIT(p_RotorState, p_Observer, p_NvmConfig) (Sensorless_Sensor_T) \
{                                                                           \
    .BASE = ROTOR_SENSOR_INIT(&SENSORLESS_SENSOR_VTABLE, (p_RotorState)),   \
    .P_OBSERVER = (p_Observer),                                             \
    .P_NVM_CONFIG = (p_NvmConfig),                                          \
}

