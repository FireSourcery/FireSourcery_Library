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
    @file   Threshold.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/*
    Interface for configuration types
    Handle Configs. Bridge to Hysteresis with Setpoint/Resetpoint pairs
*/
// typedef int32_t threshold_t;

/*
    2-value active at run time
*/
typedef struct Threshold
{
    int32_t Setpoint;    /* Activation threshold (trip point) */
    int32_t Resetpoint;  /* Deactivation threshold (reset point) */
    // bool IsEnabled; /* Per level disable */
}
Threshold_T;

/*
    Config formats
*/
/*
    Hysteresis band for this level
    Setpoint +/- Deadband
*/
typedef struct Threshold_Level
{
    int32_t Setpoint;
    uint32_t Deadband;
}
Threshold_Level_T;

/* { Magnitude, Deadband } Threshold_Symmetric_T */

/* simplified Dual direction init using band */
/*
    Threshold_Window, OperatingLimits

    High => Setpoint + Deadband
    Low  => Setpoint - Deadband
*/
typedef struct Threshold_Range
{
    int32_t LimitHigh;
    int32_t LimitLow;
    uint32_t Deadband;     /* Applies to both directions */
    // int32_t Bandwitdh;
}
Threshold_Range_T;

static inline Threshold_T Threshold_FromRangeHigh(Threshold_Range_T range) { return (Threshold_T) { .Setpoint = range.LimitHigh, .Resetpoint = range.LimitHigh - range.Deadband }; }
static inline Threshold_T Threshold_FromRangeLow(Threshold_Range_T range) { return (Threshold_T) { .Setpoint = range.LimitLow, .Resetpoint = range.LimitLow + range.Deadband }; }
static inline Threshold_T Threshold_FromLevelHigh(Threshold_Level_T level) { return (Threshold_T) { .Setpoint = level.Setpoint, .Resetpoint = level.Setpoint - level.Deadband }; }
static inline Threshold_T Threshold_FromLevelLow(Threshold_Level_T level) { return (Threshold_T) { .Setpoint = level.Setpoint, .Resetpoint = level.Setpoint + level.Deadband }; }


static inline Threshold_T Threshold_InitSymmetric(int32_t center_point, uint32_t deadband_width)
{
    int32_t half_band = deadband_width / 2;
    return (Threshold_T) { .Setpoint = center_point + half_band, .Resetpoint = center_point - half_band };
}
