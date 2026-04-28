#pragma once

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
    @file   Hall_RotorSensor.h
    @author FireSourcery
    @brief  Implement the RotorSensor interface for Hall sensors.
            Composes: PulseEncoder_T (PulseTimer + AngleCounter bundle) + Hall_T (sensor decode).
*/
/******************************************************************************/
#include "../RotorSensor.h"
#include "Hall.h"
#include "Transducer/Pulse/PulseEncoder.h"
#include "Math/Angle/AngleCounter.h"

typedef const struct Hall_RotorSensor
{
    RotorSensor_T BASE;       /* P_STATE->AngleSpeed is the final output interface */
    Hall_T HALL;
    PulseEncoder_T PULSE;     /* PulseTimer + AngleCounter bundle (edge timing + count/freq/interp) */
    uint32_t POLLING_FREQ;    /* Control loop frequency [Hz] for angle delta conversion */
}
Hall_RotorSensor_T;

extern const RotorSensor_VTable_T HALL_VTABLE;

#define HALL_ROTOR_SENSOR_INIT(HallStruct, PulseStruct, PollingFreq, p_State) (Hall_RotorSensor_T) \
{                                                                                               \
    .BASE           = ROTOR_SENSOR_INIT(&HALL_VTABLE, p_State),                                 \
    .HALL           = (HallStruct),                                                             \
    .PULSE          = (PulseStruct),                                                            \
    .POLLING_FREQ   = (PollingFreq),                                                            \
}
