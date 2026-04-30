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
/* Phase_Board */
typedef const struct Phase_Calibration
{
    /* Sensor/Type/Calibration Max. Unit conversion reference. Compile time defined. Optionally allow runtime overwrite */
    volatile uint16_t V_MAX_VOLTS;
    volatile uint16_t I_MAX_AMPS;

    /* Optionally include units */
    volatile uint16_t V_RATED_FRACT16;
    volatile uint16_t I_RATED_PEAK_FRACT16;
}
Phase_Calibration_T;
//todo volatile in getter instead

/* Define in Main App */
/* run-time overwrite or compile time def. */
extern const Phase_Calibration_T PHASE_CALIBRATION;

#if !defined(PHASE_V_TYPE_MAX_VOLTS) && !defined(PHASE_I_TYPE_MAX_AMPS)
#define PHASE_V_TYPE_MAX_VOLTS PHASE_CALIBRATION.V_MAX_VOLTS
#define PHASE_I_TYPE_MAX_AMPS PHASE_CALIBRATION.I_MAX_AMPS
#define PHASE_V_FRACT16(volts, Max) FRACT16((float)volts / Max)
#define PHASE_I_FRACT16(amps, Max) FRACT16((float)amps / Max)
#else /* Compile time def only */
#define PHASE_V_FRACT16(volts) FRACT16((float)volts / PHASE_V_TYPE_MAX_VOLTS)
#define PHASE_I_FRACT16(amps) FRACT16((float)amps / PHASE_I_TYPE_MAX_AMPS)
#endif

/* Getter interface. */
static inline uint16_t Phase_Calibration_GetIMaxAmps(void) { return PHASE_CALIBRATION.I_MAX_AMPS; }
static inline uint16_t Phase_Calibration_GetVMaxVolts(void) { return PHASE_CALIBRATION.V_MAX_VOLTS; }
static inline uint16_t Phase_Calibration_GetVRated_Fract16(void) { return PHASE_CALIBRATION.V_RATED_FRACT16; }
static inline uint16_t Phase_Calibration_GetIRatedPeak_Fract16(void) { return PHASE_CALIBRATION.I_RATED_PEAK_FRACT16; }
static inline uint16_t Phase_Calibration_GetVRated_V(void) { return Phase_Calibration_GetVRated_Fract16() * Phase_Calibration_GetVMaxVolts() / 32768; }
static inline int16_t Phase_Calibration_GetIRatedPeak_Amps(void) { return Phase_Calibration_GetIRatedPeak_Fract16() * Phase_Calibration_GetIMaxAmps() / 32768; }
static inline int16_t Phase_Calibration_GetIRatedRms_Amps(void) { return Phase_Calibration_GetIRatedPeak_Fract16() * Phase_Calibration_GetIMaxAmps() / FRACT16_SQRT2; }


/******************************************************************************/

/******************************************************************************/
static inline bool _Phase_Calibration_IsValid(uint16_t value) { return ((value != 0U) && (value != 0xFFFFU)); }

static bool Phase_Calibration_IsValid(void)
{
    return
    (
        _Phase_Calibration_IsValid(Phase_Calibration_GetVMaxVolts()) &&
        _Phase_Calibration_IsValid(Phase_Calibration_GetIMaxAmps()) &&
        _Phase_Calibration_IsValid(Phase_Calibration_GetVRated_Fract16()) &&
        _Phase_Calibration_IsValid(Phase_Calibration_GetIRatedPeak_Fract16())
    );
}


/******************************************************************************/
/*
    Local unit conversions
*/
/******************************************************************************/
static inline accum32_t Phase_I_Fract16OfAmps(int16_t amps) { return amps * INT16_MAX / Phase_Calibration_GetIMaxAmps(); }
static inline int16_t   Phase_I_AmpsOfFract16(accum32_t fract16) { return fract16 * Phase_Calibration_GetIMaxAmps() / 32768; }
static inline accum32_t Phase_V_Fract16OfVolts(int16_t volts) { return volts * INT16_MAX / Phase_Calibration_GetVMaxVolts(); }
static inline int16_t   Phase_V_VoltsOfFract16(accum32_t fract16) { return fract16 * Phase_Calibration_GetVMaxVolts() / 32768; }
static inline accum32_t Phase_Power_VAOfFract16(accum32_t fract16) { return fract16 * Phase_Calibration_GetIMaxAmps() * Phase_Calibration_GetVMaxVolts() / 32768; }

/*
    Resistance / Inductance conversions
    R_REF = V_MAX_VOLTS / I_MAX_AMPS [Ohm]
    Rs_Fract16 represents (R_Ohm / R_REF) in fract16.
    Rs_MilliOhms = Rs_Fract16 * V_MAX_VOLTS * 1000 / (I_MAX_AMPS * 32768)
    L conversions not fract16-based; inductance lives in uH directly. See KL*_Fract16 in Motor_Config_T for in-loop use.
*/
static inline accum32_t Phase_R_Fract16OfMilliOhms(uint16_t milliOhms)
{
    return ((accum32_t)milliOhms * Phase_Calibration_GetIMaxAmps() * INT16_MAX) / ((accum32_t)Phase_Calibration_GetVMaxVolts() * 1000);
}

static inline uint16_t Phase_R_MilliOhmsOfFract16(accum32_t fract16)
{
    return ((accum32_t)fract16 * Phase_Calibration_GetVMaxVolts() * 1000) / ((accum32_t)Phase_Calibration_GetIMaxAmps() * 32768);
}
