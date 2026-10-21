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
    @brief  RotorSensor reading the rotor through the stator [EmfObserver_T].

    Differs from Hall/Encoder/SinCos: the transducer is push-driven, it needs i_αβ and v_αβ from FOC, not a peripheral.
    The integration layer invokes Sensorless_Sensor_Proc once per control tick, after the FOC proc. The vtable's
    CAPTURE_ANGLE/CAPTURE_SPEED publish the observer's θ̂, ω̂ into RotorSensor_State idempotently.

    Only the sensorless start-up substates (Motor_Sensorless.c) drive Proc; RotorSensor_CaptureAngle alone would publish a frozen θ̂.
*/
/******************************************************************************/
#include "EmfObserver.h"
#include "../RotorSensor.h"
#include "../../Math/FOC.h"


typedef const struct Sensorless_Sensor
{
    const RotorSensor_T BASE;
    EmfObserver_T * P_OBSERVER;
    const EmfObserver_Config_T * P_NVM_CONFIG;
}
Sensorless_Sensor_T;

extern const RotorSensor_VTable_T SENSORLESS_SENSOR_VTABLE;

/* Observer state of static duration, held by the const table entry */
#define SENSORLESS_SENSOR_INIT(p_RotorState, p_NvmConfig) (Sensorless_Sensor_T)    \
{                                                                                   \
    .BASE           = ROTOR_SENSOR_INIT(&SENSORLESS_SENSOR_VTABLE, (p_RotorState)), \
    .P_OBSERVER     = &(EmfObserver_T){},                                           \
    .P_NVM_CONFIG   = (p_NvmConfig),                                                \
}


/******************************************************************************/
/*!
    Push-driven entry
*/
/******************************************************************************/
/*
    One control tick, after the FOC proc: the period its sample closes and the command it made, from the αβ it carries.
*/
static inline void Sensorless_Sensor_Proc(Sensorless_Sensor_T * p_sensor, const FOC_T * p_foc)
{
    EmfObserver_Capture(p_sensor->P_OBSERVER, p_foc->Ialpha, p_foc->Ibeta, p_foc->Valpha, p_foc->Vbeta);
    EmfObserver_StepSmo(p_sensor->P_OBSERVER);
}
