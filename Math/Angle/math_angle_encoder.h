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
    @file   math_angle_encoder.h
    @author FireSourcery
    @brief  angle math using cpr
*/
/******************************************************************************/
#include "../Fixed/fract16.h"

typedef uint32_t angle32_t; // shifted 16
// #define ANGLE32_PER_REVOLUTION (UINT32_C(0x100000000)) /* 2^32 */
#define ANGLE_ACCUM_SHIFT (16U)
#define ANGLE_SPEED_SHIFT (15U)

/******************************************************************************/
/*!
    Unit Calculation Functions - Pure Functions
*/
/******************************************************************************/
/*
    angle_encoder_unit
*/
/*
    Factor Unit
*/
// 16 fraction bits
static inline uint32_t angle_encoder_unit_shifted(uint32_t counts_per_revolution) { return UINT32_MAX / counts_per_revolution + 1U; } /* +1 to round up */

/* counterD_Ticks < counts_per_revolution */
static inline uint32_t angle_of_encoder_count(uint32_t angle_unit_shifted, uint32_t counterD_Ticks) { return ((counterD_Ticks * angle_unit_shifted) >> 16U); }
static inline uint32_t encoder_count_of_angle(uint32_t angle_unit_shifted, uint16_t angle16) { return (angle16 << 16U) / angle_unit_shifted; }

/* delta */
/* counterD_Ticks < 65536 */
static inline uint32_t encoder_count_of_angle_cpr(uint32_t counts_per_revolution, uint32_t angle16) { return ((angle16 * counts_per_revolution) / ANGLE16_PER_REVOLUTION); }
static inline uint32_t angle_of_count_cpr(uint32_t counts_per_revolution, uint32_t counterD_Ticks) { return ((counterD_Ticks * ANGLE16_PER_REVOLUTION) / counts_per_revolution); }


// uint32_t speed_fract16_unit_shifted(uint32_t UnitTime_Freq, uint32_t CountsPerRevolution, uint32_t ScalarSpeedRef_Rpm)
// {
//     return (((uint64_t)32768U * 60U * UnitTime_Freq) << 15U) / (CountsPerRevolution * ScalarSpeedRef_Rpm);
// }

// 15 fraction bits
// static inline accum32_t angle_speed_fract16_unit_shifted_rpm(uint32_t UnitTime_Freq, uint32_t angle32_per_count, uint32_t speed_ref_rpm)
// {
//     return ((uint64_t)angle32_per_count * 60U * UnitTime_Freq) / (2U * speed_ref_rpm);
// }

// static inline accum32_t angle_speed_digital_unit_shifted(uint32_t UnitTime_Freq, uint32_t angle32_per_count, uint32_t PartitionsPerRevolution)
// {
//     return ((uint64_t)angle32_per_count * PartitionsPerRevolution) / UnitTime_Freq;
// }

/*
   Pulse Encoder
*/
// static inline int32_t angle_encoder_freq_speed_unit(uint32_t pollingFreq, uint32_t countsPerRevolution, uint32_t countFreq)

// static inline int32_t angle_speed_of_encoder_freq(uint32_t pollingFreq, uint32_t countsPerRevolution, uint32_t countFreq)
// static inline int32_t angle_speed_of_encoder_freq(uint32_t pollingFreq, uint32_t angle_unit_shifted, uint32_t countFreq)

// static inline void Angle_Encoder_CaptureFreqD(Angle_T * p_angle, uint32_t freqD)
// {
//     return freqD * p_angle->ScalarSpeedUnitPerCount >> p_angle->ScalarSpeedUnitShift;
// }
// static inline void Angle_CaptureInterpolateDelta(Angle_T * p_angle, accum32_t speed_fract16)
// {
     // p_angle->Speed = angle_of_speed_fract16(p_angle->UnitRef.SpeedRated, speed_fract16);
// }

/*
    Units
*/
/**
 * @brief Calculate angle conversion factor from encoder counts to degrees
 * @param counts_per_revolution Number of encoder counts per full revolution
 * @return Angle conversion factor (degrees per count, scaled by ENCODER_ANGLE_SHIFT)
 */
// static angle32 angle_unit_factor(uint16_t counts_per_revolution) { return ANGLE16_PER_REVOLUTION / counts_per_revolution; }

/**
 * @brief Calculate scalar speed conversion factor
 * @param counts_per_revolution Number of encoder counts per revolution
 * @param speed_ref_rpm Reference speed in RPM for scaling
 * @param unit_time_freq Timer frequency in Hz
 * @param shift_bits Number of bits to shift for fixed-point precision
 * @return Speed conversion factor
 */
static uint64_t scalar_speed_unit_factor(uint16_t counts_per_revolution, uint16_t speed_ref_rpm, uint32_t unit_time_freq, uint8_t shift_bits)
{
    const uint32_t FRACT16_HALF = 32768U;
    const uint32_t SECONDS_PER_MINUTE = 60U;
    return ((uint64_t)FRACT16_HALF * SECONDS_PER_MINUTE * unit_time_freq << shift_bits) / (uint64_t)counts_per_revolution * speed_ref_rpm;
}

/**
 * @brief Calculate angular speed conversion factor (degrees per second)
 * @param counts_per_revolution Number of encoder counts per revolution
 * @param unit_time_freq Timer frequency in Hz
 * @return Angular speed conversion factor
 */
static uint64_t angular_speed_unit_factor(uint16_t counts_per_revolution, uint32_t unit_time_freq)
{
    return (uint64_t)ANGLE16_PER_REVOLUTION * unit_time_freq / counts_per_revolution;
}

/**
 * @brief Calculate polling angle conversion factor
 * @param angle_unit_factor Base angle conversion factor
 * @param partitions_per_revolution Number of partitions per revolution
 * @param polling_frequency Polling frequency in Hz
 * @return Polling angle conversion factor
 */
static uint32_t polling_angle_unit_factor(uint32_t angle_unit_factor, uint16_t partitions_per_revolution, uint32_t polling_frequency)
{
    return angle_unit_factor * partitions_per_revolution / polling_frequency;
}

/**
 * @brief Calculate interpolation angle limit
 * @param angle_unit_factor Base angle conversion factor
 * @param partitions_per_revolution Number of partitions per revolution
 * @return Interpolation angle limit
 */
static uint32_t interpolation_angle_limit(uint32_t angle_unit_factor, uint16_t partitions_per_revolution)
{
    return angle_unit_factor * partitions_per_revolution;
}

/**
 * @brief Calculate linear speed conversion factor
 * @param counts_per_revolution Number of encoder counts per revolution
 * @param unit_time_freq Timer frequency in Hz
 * @param surface_diameter Surface diameter in millimeters
 * @param gear_ratio_input Input gear ratio
 * @param gear_ratio_output Output gear ratio
 * @return Linear speed conversion factor
 */
static uint64_t linear_speed_unit_factor(uint16_t counts_per_revolution, uint32_t unit_time_freq, uint32_t surface_diameter, uint32_t gear_ratio_input, uint32_t gear_ratio_output)
{
    // assert(counts_per_revolution == 0);
    // assert(gear_ratio_output == 0);
    const uint32_t PI_SCALED = 314U;  // π * 100 for fixed-point
    const uint32_t SCALE_FACTOR = 100U;

    uint64_t numerator = (uint64_t)unit_time_freq * gear_ratio_input * surface_diameter * PI_SCALED;
    uint64_t denominator = (uint64_t)counts_per_revolution * gear_ratio_output * SCALE_FACTOR;

    return numerator / denominator;
}

/**
 * @brief Calculate maximum delta for speed calculations
 * @param speed_ref_rpm Reference speed in RPM
 * @param counts_per_revolution Number of encoder counts per revolution
 * @param unit_time_freq Timer frequency in Hz
 * @return Maximum delta value
 */
static uint32_t max_delta(uint16_t counts_per_revolution, uint16_t speed_ref_rpm, uint32_t unit_time_freq)
{
    const uint32_t SECONDS_PER_MINUTE = 60U;
    const uint32_t SAFETY_FACTOR = 2U;
    return speed_ref_rpm * SAFETY_FACTOR * counts_per_revolution / (SECONDS_PER_MINUTE * unit_time_freq);
}