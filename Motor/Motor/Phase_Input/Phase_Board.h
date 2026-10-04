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
    @file   Phase_Board.h
    @author FireSourcery
    @brief  Global "Static" Const, for all Motor instances
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"
#include "../Math/motor_electrical_math.h"


/******************************************************************************/
/*!
    Phase Board and Scaling References
*/
/******************************************************************************/
/* Phase_Board */
typedef const struct Phase_Board
{
    /* Sensor/Type/Calibration Max. Unit conversion reference. Compile time defined. Optionally allow runtime overwrite */
    /* Type max using sensor saturation. alternatively runtime select */
    volatile uint16_t V_MAX_VOLTS;
    volatile uint16_t I_MAX_AMPS;

    /* Limits for config input */
    /* Optionally include si units */
    volatile uint16_t V_RATED_PU;
    volatile uint16_t I_RATED_PEAK_PU;
    volatile uint16_t I_RATED_FW_PU;
    volatile uint16_t _RESV[3U];
}
Phase_Board_T;

/* Define in Main App */
/* run-time overwrite or compile time def. */
extern const Phase_Board_T PHASE_BOARD;

#if !defined(PHASE_V_TYPE_MAX_VOLTS) && !defined(PHASE_I_TYPE_MAX_AMPS)
#define PHASE_V_TYPE_MAX_VOLTS PHASE_BOARD.V_MAX_VOLTS
#define PHASE_I_TYPE_MAX_AMPS PHASE_BOARD.I_MAX_AMPS
#else /* Compile time def only */
#define PHASE_V_PU(volts) FRACT16((float)volts / PHASE_V_TYPE_MAX_VOLTS)
#define PHASE_I_PU(amps) FRACT16((float)amps / PHASE_I_TYPE_MAX_AMPS)
#endif

static inline uint16_t Phase_IMaxAmps(void) { return PHASE_BOARD.I_MAX_AMPS; }
static inline uint16_t Phase_VMaxVolts(void) { return PHASE_BOARD.V_MAX_VOLTS; }
/* Keep virtual getters in case structure changes */
static inline uint16_t Phase_VRated_Pu(void) { return PHASE_BOARD.V_RATED_PU; }
static inline uint16_t Phase_IRatedPeak_Pu(void) { return PHASE_BOARD.I_RATED_PEAK_PU; }
static inline uint16_t Phase_IRatedFw_Pu(void) { return PHASE_BOARD.I_RATED_FW_PU; }
static inline uint16_t Phase_VRated_Volts(void) { return Phase_VRated_Pu() * Phase_VMaxVolts() / 32768; }
static inline int16_t Phase_IRatedPeak_Amps(void) { return Phase_IRatedPeak_Pu() * Phase_IMaxAmps() / 32768; }
static inline int16_t Phase_IRatedRms_Amps(void) { return Phase_IRatedPeak_Pu() * Phase_IMaxAmps() / FRACT16_SQRT2; }


/******************************************************************************/

/******************************************************************************/
static inline bool _Phase_Board_IsValid(uint16_t value) { return ((value != 0U) && (value != 0xFFFFU)); }
static inline bool _Phase_Board_IsValidRated(uint16_t value) { return ((value != 0U) && (value <= FRACT16_SCALE)); } /* (0, 1.0] of the base */

static inline bool Phase_Board_IsValid(void)
{
    return
    (
        _Phase_Board_IsValid(Phase_VMaxVolts()) &&
        _Phase_Board_IsValid(Phase_IMaxAmps()) &&
        _Phase_Board_IsValidRated(Phase_VRated_Pu()) &&
        _Phase_Board_IsValidRated(Phase_IRatedPeak_Pu()) &&
        (Phase_IRatedFw_Pu() < Phase_IRatedPeak_Pu()) /* Id within the rated vector, leaving Iq > 0. 0 disables FW */
    );
}


/******************************************************************************/
/*
    Local unit conversions
*/
/******************************************************************************/
static inline accum32_t Phase_I_PuOfAmps(int16_t amps) { return amps * INT16_MAX / Phase_IMaxAmps(); }
static inline int16_t   Phase_I_AmpsOfPu(accum32_t fract16) { return fract16 * Phase_IMaxAmps() / 32768; }
static inline accum32_t Phase_V_PuOfVolts(int16_t volts) { return volts * INT16_MAX / Phase_VMaxVolts(); }
static inline int16_t   Phase_V_VoltsOfPu(accum32_t fract16) { return fract16 * Phase_VMaxVolts() / 32768; }
static inline accum32_t Phase_Power_VoltAmpsOfPu(accum32_t fract16) { return fract16 * Phase_IMaxAmps() * Phase_VMaxVolts() / 32768; }

/*
    Resistance Ref
    R_REF = V_MAX_VOLTS / I_MAX_AMPS [Ohm]
    Rs_Pu represents (R_Ohm / R_REF) in fract16.
    Rs_MilliOhms = Rs_Pu * V_MAX_VOLTS * 1000 / (I_MAX_AMPS * 32768)
*/
static inline accum32_t Phase_R_PuOfMilliOhms(uint16_t milliOhms)
{
    return ((accum32_t)milliOhms * Phase_IMaxAmps() * INT16_MAX) / ((accum32_t)Phase_VMaxVolts() * 1000);
}

static inline uint16_t Phase_R_MilliOhmsOfPu(accum32_t fract16)
{
    return ((accum32_t)fract16 * Phase_VMaxVolts() * 1000) / ((accum32_t)Phase_IMaxAmps() * 32768);
}

/* τ_base = 1 / ω_base */
static inline accum32_t Phase_L_PuTauOfMicroHenries(angle_freq_t speedBase, uint16_t microHenries)
{
    return l_pu_of_h(Phase_VMaxVolts(), Phase_IMaxAmps(), speedBase, microHenries, 1000000UL);
}

static inline accum32_t Phase_L_PuTickOfMicroHenries(uint32_t fs, uint16_t microHenries)
{
    return l_pu_tick_of_h(fs, Phase_VMaxVolts(), Phase_IMaxAmps(), microHenries, 1000000UL);
}