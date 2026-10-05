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
            Composes: Hall_T (sensor decode), PulseTimer_T (edge timing) and AngleCounter_T (count/freq/interp) as collaborators.
*/
/******************************************************************************/
#include "../RotorSensor.h"
#include "Hall.h"
#include "Transducer/Pulse/PulseTimer_Counter.h"
#include "Math/Angle/AngleCounter.h"

typedef const struct Hall_RotorSensor
{
    RotorSensor_T BASE;         /* P_STATE->AngleSpeed is the final output interface */
    Hall_T HALL;
    PulseTimer_T TIMER;         /* Edge timing */
    AngleCounter_T * P_COUNTER; /* Count, FreqD, interpolation via Base */
    uint32_t POLLING_FREQ;      /* Control loop frequency [Hz] for angle delta conversion */
}
Hall_RotorSensor_T;

// extern const RotorSensor_VTable_T HALL_VTABLE;

#define HALL_ROTOR_SENSOR_INIT(HallStruct, TimerStruct, PollingFreq, p_State) (Hall_RotorSensor_T) \
{                                                                                               \
    .BASE           = ROTOR_SENSOR_INIT(&HALL_VTABLE, p_State),                                 \
    .HALL           = (HallStruct),                                                             \
    .TIMER          = (TimerStruct),                                                            \
    .P_COUNTER      = ANGLE_COUNTER_ALLOC(),                                                    \
    .POLLING_FREQ   = (PollingFreq),                                                            \
}


static void Hall_RotorSensor_Init(Hall_RotorSensor_T * p_sensor)
{
    Hall_Init(&p_sensor->HALL);
    PulseTimer_Init(&p_sensor->TIMER);
    PulseTimer_SetExtendedWatchStop_Millis(&p_sensor->TIMER, 500U);
}

/*
    20kHz - Capture angle.
    Base.Angle is the full-rotation accumulator: snapped to the sector boundary
    on each Hall edge; interpolated toward the next-sector endpoint between edges.
*/
static void Hall_RotorSensor_CaptureAngle(Hall_RotorSensor_T * p_sensor)
{
    if (Hall_PollCaptureSensors(&p_sensor->HALL) == true) /* 1/6 Electrical Cycle, typically > 1ms */
    {
        PulseTimer_CaptureCount(&p_sensor->TIMER, p_sensor->P_COUNTER, Hall_ResolveDirection(p_sensor->HALL.P_STATE));
        AngleCounter_SetAngle(p_sensor->P_COUNTER, Hall_ResolveAngle(p_sensor->HALL.P_STATE)); /* Snap Base.Angle to exact sector boundary — self-corrects interpolation drift each edge */
        AngleCounter_SetLimitWindow(p_sensor->P_COUNTER, ANGLE16_PER_REVOLUTION / 6); /* Clamp interpolation to the next sector endpoint (60° ahead in sign(Delta)) */
    }
    else /* 20kHz, interpolate toward next sector boundary using Base.Delta */
    {
        AngleCounter_Interpolate(p_sensor->P_COUNTER);
    }

    p_sensor->BASE.P_STATE->AngleSpeed.Angle = p_sensor->P_COUNTER->Base.Angle;
}

/*
    1ms - Capture speed. FreqD drives Base.Delta for interpolation.
*/
static void Hall_RotorSensor_CaptureSpeed(Hall_RotorSensor_T * p_sensor)
{
    PulseTimer_CaptureFreq(&p_sensor->TIMER, p_sensor->P_COUNTER); /* Gradual decay when no pulses */
    /* Propagate FreqD for interpolation */
    AngleCounter_ResolveAngleDelta(p_sensor->P_COUNTER);
    /* Write speed to output interface */
    p_sensor->BASE.P_STATE->Speed_Pu = (AngleCounter_GetSpeed_Pu(p_sensor->P_COUNTER) + p_sensor->BASE.P_STATE->Speed_Pu) / 2;
    p_sensor->BASE.P_STATE->AngleSpeed.Delta = p_sensor->P_COUNTER->Base.Delta / 2 + p_sensor->BASE.P_STATE->AngleSpeed.Delta / 2;
}

static bool Hall_RotorSensor_IsFeedbackAvailable(Hall_RotorSensor_T * p_sensor) { (void)p_sensor; return true; }


static void Hall_RotorSensor_ZeroInitial(Hall_RotorSensor_T * p_sensor)
{
    Hall_ZeroInitial(&p_sensor->HALL);
    AngleCounter_ZeroCount(p_sensor->P_COUNTER);
    Angle_ZeroCaptureState(&p_sensor->P_COUNTER->Base);
    PulseTimer_SetInitial(&p_sensor->TIMER);
}

static bool Hall_RotorSensor_VerifyCalibration(Hall_RotorSensor_T * p_sensor) { return Hall_IsCalibrationTableValid(p_sensor->HALL.P_STATE); }

/*!
    The counter revolution is electrical: 6 edges per electrical cycle, against the electrical base.
*/
static void Hall_RotorSensor_InitUnits_ElSpeed(Hall_RotorSensor_T * p_sensor)
{
    AngleCounter_Config_T config =
    {
        .CountsPerRevolution = 6U,
        .PollingFreq = p_sensor->POLLING_FREQ,
        .AngleFreqBase = p_sensor->BASE.P_STATE->UnitRef.AngleFreqBase,
    };
    AngleCounter_InitFrom(p_sensor->P_COUNTER, &config);
}

/*
    Interface VTable
*/
static const RotorSensor_VTable_T HALL_VTABLE =
{
    .INIT = (RotorSensor_Proc_T)Hall_RotorSensor_Init,
    .INIT_UNITS = (RotorSensor_Proc_T)Hall_RotorSensor_InitUnits_ElSpeed,
    .CAPTURE_ANGLE = (RotorSensor_Proc_T)Hall_RotorSensor_CaptureAngle,
    .CAPTURE_SPEED = (RotorSensor_Proc_T)Hall_RotorSensor_CaptureSpeed,
    .IS_FEEDBACK_AVAILABLE = (RotorSensor_Test_T)Hall_RotorSensor_IsFeedbackAvailable,
    .ZERO_INITIAL = (RotorSensor_Proc_T)Hall_RotorSensor_ZeroInitial,
    .VERIFY_CALIBRATION = (RotorSensor_Test_T)Hall_RotorSensor_VerifyCalibration,
};


/*!
    Hall sensors as speed encoder.
    CPR = PolePairs*6   => GetSpeed => mechanical speed
*/
static void Hall_RotorSensor_InitUnits_MechSpeed(Hall_RotorSensor_T * p_sensor)
{
    const RotorSensor_UnitRef_T * p_unitRef = &p_sensor->BASE.P_STATE->UnitRef;
    AngleCounter_Config_T config =
    {
        .CountsPerRevolution = 6U * p_unitRef->PolePairs, /* Mechanical CPR for speed/RPM */
        .PollingFreq = p_sensor->POLLING_FREQ,
        .AngleFreqBase = p_unitRef->AngleFreqBase / p_unitRef->PolePairs,
    };
    AngleCounter_InitFrom(p_sensor->P_COUNTER, &config);
    /* Override angle delta factor for electrical interpolation: 6 edges per electrical cycle */
    p_sensor->P_COUNTER->UnitRef.AngleDt32PerCount = angle_dt32_per_count_cpr(p_sensor->POLLING_FREQ, 6U);
}