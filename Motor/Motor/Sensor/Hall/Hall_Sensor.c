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
    @file   Hall_RotorSensor.c
    @author FireSourcery
    @brief  Hall sensor RotorSensor implementation.
            AngleCounter_T for counter + frequency + interpolation (via Angle_T Base),
            PulseTimer for Timer HAL, Hall_T for sensor decode.
            AngleSpeed in RotorSensor_State_T is the final output interface.
*/
/******************************************************************************/
#include "Hall_Sensor.h"
#include "Math/Angle/Angle.h"

static void Hall_RotorSensor_Init(const Hall_RotorSensor_T * p_sensor)
{
    Hall_Init(&p_sensor->HALL);
    PulseTimer_Init(&p_sensor->TIMER);
    PulseTimer_SetExtendedWatchStop_Millis(&p_sensor->TIMER, 1000U);
}

/*
    20kHz - Capture angle and interpolate between Hall edges.
    AngleCounter.Base accumulates interpolation. On edge, reset and snapshot.
*/
static void Hall_RotorSensor_CaptureAngle(const Hall_RotorSensor_T * p_sensor)
{
    RotorSensor_State_T * p_state = p_sensor->BASE.P_STATE;
    AngleCounter_T * p_counter = p_sensor->P_COUNTER;

    if (Hall_PollCaptureSensors(&p_sensor->HALL) == true) /* 1/6 Electrical Cycle, typically > 1ms */
    {
        PulseTimer_CaptureEdge(&p_sensor->TIMER);
        AngleCounter_CaptureCount(p_counter, Hall_ResolveDirection(p_sensor->HALL.P_STATE));
        // AngleCounter_SetAngle(p_counter, Hall_ResolveAngle(p_sensor->HALL.P_STATE));
        // AngleCounter_InitLimit(p_sensor->P_COUNTER, Hall_ResolveAngle(p_sensor->HALL.P_STATE) + ANGLE16_PER_REVOLUTION / 6);

        Hall_ResolveAngle(p_sensor->HALL.P_STATE); /* for Hall_GetAngle16 */
        AngleCounter_ZeroAngle(p_counter); /* Reset interpolation on edge */
    }
    else /* 20kHz, Interpolate angle between Hall edges */
    {
        AngleCounter_Interpolate(p_counter);
    }


    /* Write final angle to output interface: sector angle + interpolation offset */
    p_state->AngleSpeed.Angle = ((int32_t)Hall_GetAngle16(p_sensor->HALL.P_STATE) << ANGLE32_SHIFT) + p_counter->Base.Angle;

    /* altnernatively use p_counter->Base.Angle accumated state */
    // p_state->AngleSpeed.Angle = p_counter->Base.Angle;
}

/*
    1ms - Capture speed. FreqD drives Base.Delta for interpolation.
*/
static void Hall_RotorSensor_CaptureSpeed(const Hall_RotorSensor_T * p_sensor)
{
    RotorSensor_State_T * p_state = p_sensor->BASE.P_STATE;
    AngleCounter_T * p_counter = p_sensor->P_COUNTER;

    if (PulseTimer_IsExtendedStop(&p_sensor->TIMER) == false) { AngleCounter_CaptureFreqD(p_counter, PulseTimer_CaptureSampleTk(&p_sensor->TIMER)); }
    else { p_counter->FreqD = 0; }
    // if (PulseTimer_IsExtendedStop(&p_sensor->TIMER) == false) { AngleCounter_CaptureFreq(p_counter, PulseTimer_CaptureSampleTk_Freq(&p_sensor->TIMER)); }
    // else { p_counter->FreqD = 0; }

    /* Propagate FreqD for interpolation */
    AngleCounter_ResolveInterpolationDelta(p_counter);

    /* Write speed to output interface */
    p_state->Speed_Fract16 = (AngleCounter_GetSpeed_Fract16(p_counter) + p_state->Speed_Fract16) / 2;
    // p_state->AngleSpeed.Delta = p_counter->Base.Delta;
    // Angle_CaptureSpeed_Fract16(&p_state->AngleSpeed, &p_state->SpeedFractRef, p_state->Speed_Fract16);
}


static bool Hall_RotorSensor_IsFeedbackAvailable(const Hall_RotorSensor_T * p_sensor) { (void)p_sensor; return true; }


static void Hall_RotorSensor_ZeroInitial(const Hall_RotorSensor_T * p_sensor)
{
    Hall_ZeroInitial(&p_sensor->HALL);
    AngleCounter_Zero(p_sensor->P_COUNTER);
    Angle_ZeroCaptureState(&p_sensor->P_COUNTER->Base);
    PulseTimer_SetInitial(&p_sensor->TIMER);
}

static bool Hall_RotorSensor_VerifyCalibration(const Hall_RotorSensor_T * p_sensor) { return Hall_IsTableValid(p_sensor->HALL.P_STATE); }

/*!
    Hall sensors as speed encoder.
    CPR = PolePairs*6   => GetSpeed => mechanical speed
*/
static void Hall_RotorSensor_InitUnits_MechSpeed(const Hall_RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    AngleCounter_Config_T config =
    {
        .CountsPerRevolution = 6U * p_config->PolePairs, /* Mechanical CPR for speed/RPM */
        .TimerFreq = p_sensor->TIMER.TIMER_FREQ,
        .SampleFreq = p_sensor->TIMER.SAMPLE_FREQ,
        .PollingFreq = p_sensor->POLLING_FREQ,
        .FractSpeedRef_Rpm = p_config->SpeedTypeMax_Rpm,
    };
    AngleCounter_InitFrom(p_sensor->P_COUNTER, &config);
    /* Override angle delta factor for electrical interpolation: 6 edges per electrical cycle */
    p_sensor->P_COUNTER->Ref.AngleSpeed32PerCount = angle32_speed_per_count_cpr(p_sensor->POLLING_FREQ, 6U);

    /* Init interpolation limit from electrical sector angle (60 deg per Hall sector) */
    AngleCounter_InitLimit(p_sensor->P_COUNTER, ANGLE16_PER_REVOLUTION / 6);
}

/*
    Interface VTable
*/
const RotorSensor_VTable_T HALL_VTABLE =
{
    .INIT = (RotorSensor_Proc_T)Hall_RotorSensor_Init,
    .INIT_UNITS_FROM = (RotorSensor_InitFrom_T)Hall_RotorSensor_InitUnits_MechSpeed,
    .CAPTURE_ANGLE = (RotorSensor_Proc_T)Hall_RotorSensor_CaptureAngle,
    .CAPTURE_SPEED = (RotorSensor_Proc_T)Hall_RotorSensor_CaptureSpeed,
    .IS_FEEDBACK_AVAILABLE = (RotorSensor_Test_T)Hall_RotorSensor_IsFeedbackAvailable,
    .ZERO_INITIAL = (RotorSensor_Proc_T)Hall_RotorSensor_ZeroInitial,
    .VERIFY_CALIBRATION = (RotorSensor_Test_T)Hall_RotorSensor_VerifyCalibration,
};

