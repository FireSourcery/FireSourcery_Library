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
    @file   Phase_Calibration.h
    @author FireSourcery
    @brief  Global "Static" Const, for all Motor instances
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"


/******************************************************************************/
/*!
    Phase Calibration and Scaling References
*/
/******************************************************************************/
typedef const struct Phase_Calibration
{
    volatile uint16_t V_MAX_VOLTS;         /* Calibration Max. Unit conversion reference. Compile time derived */
    volatile uint16_t I_MAX_AMPS;          /* Calibration Max. Unit conversion reference. Compile time derived */

    volatile uint16_t V_RATED_FRACT16;
    volatile uint16_t I_RATED_PEAK_FRACT16;
}
Phase_Calibration_T;

/* Define in Main App */
/* run-time overwrite or compile time def. */
extern const Phase_Calibration_T PHASE_CALIBRATION;

/* Getter interface. */
static inline uint16_t Phase_Calibration_GetIMaxAmps(void) { return PHASE_CALIBRATION.I_MAX_AMPS; }
static inline uint16_t Phase_Calibration_GetVMaxVolts(void) { return PHASE_CALIBRATION.V_MAX_VOLTS; }
static inline uint16_t Phase_Calibration_GetVRated_Fract16(void) { return PHASE_CALIBRATION.V_RATED_FRACT16; }
static inline uint16_t Phase_Calibration_GetIRatedPeak_Fract16(void) { return PHASE_CALIBRATION.I_RATED_PEAK_FRACT16; }

static inline uint16_t Phase_Calibration_GetVRated_V(void) { return Phase_Calibration_GetVRated_Fract16() * Phase_Calibration_GetVMaxVolts() / 32768; }
static inline int16_t Phase_Calibration_GetIRatedPeak_Amps(void) { return Phase_Calibration_GetIRatedPeak_Fract16() * Phase_Calibration_GetIMaxAmps() / 32768; }

static inline bool _Phase_Calibration_IsLoaded(uint16_t value) { return ((value != 0U) && (value != 0xFFFFU)); }

static bool Phase_Calibration_IsLoaded(void)
{
    return
    (
        _Phase_Calibration_IsLoaded(Phase_Calibration_GetVMaxVolts()) &&
        _Phase_Calibration_IsLoaded(Phase_Calibration_GetIMaxAmps()) &&
        _Phase_Calibration_IsLoaded(Phase_Calibration_GetVRated_Fract16()) &&
        _Phase_Calibration_IsLoaded(Phase_Calibration_GetIRatedPeak_Fract16())
    );
}

/*
    Local unit conversions
*/
static inline accum32_t Phase_I_Fract16OfAmps(int16_t amps) { return amps * INT16_MAX / Phase_Calibration_GetIMaxAmps(); }
static inline int16_t   Phase_I_AmpsOfFract16(accum32_t fract16) { return fract16 * Phase_Calibration_GetIMaxAmps() / 32768; }
static inline accum32_t Phase_V_Fract16OfVolts(int16_t volts) { return volts * INT16_MAX / Phase_Calibration_GetVMaxVolts(); }
static inline int16_t   Phase_V_VoltsOfFract16(accum32_t fract16) { return fract16 * Phase_Calibration_GetVMaxVolts() / 32768; }
static inline accum32_t Phase_Power_VAOfFract16(accum32_t fract16) { return fract16 * Phase_Calibration_GetIMaxAmps() * Phase_Calibration_GetVMaxVolts() / 32768; }
