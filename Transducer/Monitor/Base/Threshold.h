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
*/

/*
    2-value active at run time
*/
// typedef struct Threshold_Band
typedef struct Threshold_Setpoint
{
    int32_t Setpoint;    /* Activation threshold (trip point) */
    int32_t Resetpoint;  /* Deactivation threshold (reset point) */
    // optionally
    // bool IsEnabled; /* Per level disable */
}
Threshold_Setpoint_T;

/*
    Config formats
*/
/* Hysteresis band for this level */
typedef struct Threshold_Level
{
    int32_t Setpoint;
    uint32_t Deadband;
}
Threshold_Level_T;

/* simplified Dual direction init using band */
/* Threshold_Window, OperatingLimits */
typedef struct Threshold_Range
{
    int32_t LimitHigh;
    int32_t LimitLow;
    uint32_t Deadband;     /* Applies to both directions */
    // int32_t Bandwitdh;
}
Threshold_Range_T;