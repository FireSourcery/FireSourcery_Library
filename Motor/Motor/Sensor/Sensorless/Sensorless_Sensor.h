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

    Differs from Hall/Encoder/SinCos: the correction is push-driven, it needs the phase sample and the voltage held across
    its period, not a peripheral. As Hall interpolates between edges, CAPTURE_ANGLE predicts θ̂ += ω̂ every control tick and
    publishes it [AnglePll_Predict]. Once per sample, after the write, the observer's states push the FOC's take of the sample and the held
    voltage [Motor_Sensorless.c] → EmfObserver_Eemf_ProcHeld, which corrects ω̂ for the next prediction.
    The FOC Parks the sample at the θ̂ the tick published, the frame the correction Parks it in. Every state reads the same θ̂.
*/
/******************************************************************************/
#include "../../Model/EmfObserver.h"
#include "../RotorSensor.h"
#include "../../Math/FOC.h"
#include "../../Motor_Electrical.h"


typedef const struct Sensorless_Sensor
{
    const RotorSensor_T BASE;
    EmfObserver_T * P_OBSERVER;
    FOC_T * P_FOC; /* Common */
    const Motor_Electrical_T * P_ELECTRICAL;    /* The motor's model, the winding's source [INIT_UNITS] */
    const EmfObserver_Config_T * P_NVM_CONFIG;
}
Sensorless_Sensor_T;

extern const RotorSensor_VTable_T SENSORLESS_SENSOR_VTABLE;

/* Observer state of static duration, held by the const table entry */
#define SENSORLESS_SENSOR_INIT(p_RotorState, p_Electrical, p_NvmConfig) (Sensorless_Sensor_T)   \
{                                                                                               \
    .BASE           = ROTOR_SENSOR_INIT(&SENSORLESS_SENSOR_VTABLE, (p_RotorState)),             \
    .P_OBSERVER     = &(EmfObserver_T){},                                                       \
    .P_ELECTRICAL   = (p_Electrical),                                                           \
    .P_NVM_CONFIG   = (p_NvmConfig),                                                            \
}

// #define SENSORLESS_SENSOR_INIT(p_RotorState, p_NvmConfig) (Sensorless_Sensor_T)
// {
//     .BASE           = ROTOR_SENSOR_INIT(&SENSORLESS_SENSOR_VTABLE, (p_RotorState)),
//     .P_OBSERVER     = &(EmfObserver_T){},
//     .P_NVM_CONFIG   = (p_NvmConfig),
// }


/******************************************************************************/
/*!
    Push-driven entry
*/
/******************************************************************************/
