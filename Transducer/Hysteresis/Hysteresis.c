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
    @file   Hysteresis.c
    @author FireSourcery
    @brief  Reusable hysteresis behavior implementation
*/
/******************************************************************************/
#include "Hysteresis.h"

// void Hysteresis_InitFromThresholdLevel(Hysteresis_T * p_hyst, Threshold_Level_T * p_level)
// {
// }

// void Hysteresis_InitFromRangeAsHigh(Hysteresis_T * p_hyst, Threshold_Range_T * p_range)
// {
// }


/*!
    @brief  Initialize hysteresis with explicit setpoint and resetpoint
            (setpoint == resetpoint) => Hysteresis region is skipped.
*/
void Hysteresis_InitThresholds(Hysteresis_T * p_hyst, int32_t setpoint, int32_t resetpoint)
{
    p_hyst->Setpoint = setpoint;
    p_hyst->Resetpoint = resetpoint;

    // p_hyst->IsActiveLow = (setpoint < resetpoint);
    // p_hyst->Direction = (setpoint - resetpoint); /* 0 for disable */

    p_hyst->Output = resetpoint;  /* Start at inactive state (resetpoint is always inactive) */
    // p_hyst->OutputStatePrev = false;
    p_hyst->OutputState = false;
}


/*

*/
void Hysteresis_InitAsActiveHigh(Hysteresis_T * p_hyst, int32_t setpoint, int32_t deadband_width)
{
    Hysteresis_InitThresholds(p_hyst, setpoint, setpoint - deadband_width);
}

/* Low alarm: setpoint is below reset */
void Hysteresis_InitAsActiveLow(Hysteresis_T * p_hyst, int32_t setpoint, int32_t deadband_width)
{
    Hysteresis_InitThresholds(p_hyst, setpoint, setpoint + deadband_width);
}

/*!
    @brief  Initialize symmetric hysteresis around a center point
            Active high
*/
// void Hysteresis_InitSymmetric(Hysteresis_T * p_hyst, int32_t center_point, int32_t deadband_width)
// {
//     int32_t half_band = deadband_width / 2;
//     Hysteresis_InitThresholds(p_hyst, center_point + half_band, center_point - half_band);
// }
