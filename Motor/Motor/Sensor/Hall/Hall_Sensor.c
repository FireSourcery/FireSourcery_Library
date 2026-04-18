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
            PulseEncoder_T bundles PulseTimer (edge timing) + AngleCounter (count/freq/interp via Angle_T Base).
            AngleSpeed in RotorSensor_State_T is the final output interface.
*/
/******************************************************************************/
#include "Hall_Sensor.h"
#include "Math/Angle/Angle.h"

static void Hall_RotorSensor_Init(const Hall_RotorSensor_T * p_sensor)
{
    Hall_Init(&p_sensor->HALL);
    PulseEncoder_Init(&p_sensor->PULSE);
    PulseTimer_SetExtendedWatchStop_Millis(&p_sensor->PULSE.TIMER, 1000U);
}

/*
    20kHz - Capture angle.
    Base.Angle is the full-rotation accumulator: snapped to the sector boundary
    on each Hall edge; interpolated toward the next-sector endpoint between edges.
*/
static void Hall_RotorSensor_CaptureAngle(const Hall_RotorSensor_T * p_sensor)
{
    if (Hall_PollCaptureSensors(&p_sensor->HALL) == true) /* 1/6 Electrical Cycle, typically > 1ms */
    {
        PulseEncoder_CaptureEdge(&p_sensor->PULSE, Hall_ResolveDirection(p_sensor->HALL.P_STATE));
        PulseEncoder_SetAngle(&p_sensor->PULSE, Hall_ResolveAngle(p_sensor->HALL.P_STATE)); /* Snap Base.Angle to exact sector boundary — self-corrects interpolation drift each edge */
        PulseEncoder_SetLimitWindow(&p_sensor->PULSE, ANGLE16_PER_REVOLUTION / 6); /* Clamp interpolation to the next sector endpoint (60° ahead in sign(Delta)) */
    }
    else /* 20kHz, interpolate toward next sector boundary using Base.Delta */
    {
        PulseEncoder_Interpolate(&p_sensor->PULSE);
    }

    p_sensor->BASE.P_STATE->AngleSpeed.Angle = p_sensor->PULSE.P_STATE->Counter.Base.Angle;
}

/*
    1ms - Capture speed. FreqD drives Base.Delta for interpolation.
*/
static void Hall_RotorSensor_CaptureSpeed(const Hall_RotorSensor_T * p_sensor)
{
    if (PulseEncoder_IsExtendedStop(&p_sensor->PULSE) == false) { PulseEncoder_CaptureFreq(&p_sensor->PULSE); }
    else { p_sensor->PULSE.P_STATE->Counter.FreqD = 0; }

    /* Propagate FreqD for interpolation */
    PulseEncoder_ResolveAngleDelta(&p_sensor->PULSE);
    /* Write speed to output interface */
    p_sensor->BASE.P_STATE->Speed_Fract16 = (PulseEncoder_GetSpeed_Fract16(&p_sensor->PULSE) + p_sensor->BASE.P_STATE->Speed_Fract16) / 2;
}


static bool Hall_RotorSensor_IsFeedbackAvailable(const Hall_RotorSensor_T * p_sensor) { (void)p_sensor; return true; }


static void Hall_RotorSensor_ZeroInitial(const Hall_RotorSensor_T * p_sensor)
{
    Hall_ZeroInitial(&p_sensor->HALL);
    AngleCounter_Zero(&p_sensor->PULSE.P_STATE->Counter);
    Angle_ZeroCaptureState(&p_sensor->PULSE.P_STATE->Counter.Base);
    PulseEncoder_SetInitial(&p_sensor->PULSE);
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
        .PollingFreq = p_sensor->POLLING_FREQ,
        .FractSpeedRef_Rpm = p_config->SpeedTypeMax_Rpm,
    };
    AngleCounter_InitFrom(&p_sensor->PULSE.P_STATE->Counter, &config);
    /* Override angle delta factor for electrical interpolation: 6 edges per electrical cycle */
    p_sensor->PULSE.P_STATE->Counter.Ref.AngleSpeed32PerCount = angle32_speed_per_count_cpr(p_sensor->POLLING_FREQ, 6U);
}

static void Hall_RotorSensor_InitUnits_Elpeed(const Hall_RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    AngleCounter_Config_T config =
    {
        .CountsPerRevolution = 6U,
        .PollingFreq = p_sensor->POLLING_FREQ,
        .FractSpeedRef_Rpm = p_config->SpeedTypeMax_Rpm * p_config->PolePairs,
    };
    AngleCounter_InitFrom(&p_sensor->PULSE.P_STATE->Counter, &config);
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
