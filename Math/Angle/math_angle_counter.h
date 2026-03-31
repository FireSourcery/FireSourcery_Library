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

typedef uint32_t angle32_t;
#define ANGLE_EXT_SHIFT (16U)
#define ANGLE_SPEED_SHIFT (15U)

/******************************************************************************/
/*!
    Unit Calculation Functions - Pure Functions
*/
/******************************************************************************/

/*!
    @brief Encoder Counter
*/
/* count: [CountsPerRevolution:65536/2] */
static inline uint32_t angle_of_count_cpr(uint32_t cpr, uint32_t count) { return ((count * ANGLE16_PER_REVOLUTION) / cpr); }
static inline uint32_t count_of_angle_cpr(uint32_t cpr, uint32_t angle16) { return ((angle16 * cpr) / ANGLE16_PER_REVOLUTION); }

/* counter [0:CountsPerRevolution], wrapped counter value */
static inline uint32_t angle_of_counter(uint32_t angle32PerCount, uint32_t counter) { return ((counter * angle32PerCount) >> ANGLE_EXT_SHIFT); }

/*
    Units Config
*/

/*
    Angle Accum Unit
    Angle = Counts * [(DEGREES << SHIFT) / CountsPerRevolution] >> SHIFT
*/
/* UINT32_MAX ~= ANGLE_PER_REVOLUTION << ANGLE_EXT_SHIFT */
static inline uint32_t angle32_per_count(uint32_t counts_per_revolution) { return UINT32_MAX / counts_per_revolution + 1U; } /* +1 to round up */
/* lossless for counts is pow2 */
static inline uint32_t angle_per_count(uint32_t counts_per_revolution) { return ANGLE16_PER_REVOLUTION / counts_per_revolution; }


/*
    Speed
*/
/*
    Speed Fract16 = Speed_Rpm * FRACT16_MAX / ScalarSpeedRef_Rpm
    Speed Fract16 => DeltaD * [UnitTime_Freq * FRACT16_MAX * 60 / (CountsPerRevolution * ScalarSpeedRef_Rpm)] / DeltaT
    Speed Fract16 => [DeltaD / DeltaT] * [UnitTime_Freq] * [FRACT16_MAX * 60 / (CountsPerRevolution * ScalarSpeedRef_Rpm)]

    e.g.
    Shift = 1:
    UnitTime_Freq = 625000, CountsPerRevolution = 60,
        ScalarSpeedRef_Rpm = 2500 => 16,384,000
        ScalarSpeedRef_Rpm = 10000 => 4,095,937.5
    UnitTime_Freq = 1000, CountsPerRevolution = 8192,
        ScalarSpeedRef_Rpm = 5000 => 96
    CountsPerRevolution = 24, ScalarSpeedRef_Rpm = 4000 =>
           671,088 <=> 40 << Shift

    FreqD = Speed_Rpm * CountsPerRevolution / 60
*/
static inline uint32_t rpm_fract16_per_count(uint32_t UnitTime_Freq, uint32_t CountsPerRevolution, uint32_t rpmMax)
{
    return (((uint64_t)32768U * 60U * UnitTime_Freq) << 15U) / (CountsPerRevolution * rpmMax);
}

// static inline uint32_t speed_fract16_per_count(uint32_t angle32_speed_per_count, uint32_t rpmMax)
// {
// }

/*
    Polling Angle Unit
    Angle = (Count/Time) * [(DEGREES / CountsPerRevolution) / POLLING_FREQ]
        AngularSpeed / POLLING_FREQ

    Alternatively at run time
        FreqD * [DEGREES / CountsPerRevolution] / POLLING_FREQ
*/
/*
    FreqD * [(DEGREES << SHIFT) / CountsPerRevolution / POLLING_FREQ] >> SHIFT
    shift EXT_SHIFT => FreqD < cpr * pollingFreq => counts per poll < cpr
*/
static uint32_t angle32_speed_per_count_cpr(uint32_t polling_frequency, uint32_t cpr) { return angle32_per_count(cpr) / polling_frequency; }

static uint32_t angle32_speed_per_count(uint32_t polling_frequency, uint32_t angle32_per_count) { return angle32_per_count / polling_frequency; }


// /*
//         AngleIndex * [DEGREES * TIMER_FREQ / CountsPerRevolution / POLLING_FREQ] / DeltaT
// */
// static uint32_t angle_per_poll_time_index(uint32_t angle_unit_factor,  uint32_t polling_frequency)
// {
    //     /* altneratively TIMER_FREQ / POLLING_FREQ << Shift */
    //     p_encoder->UnitPollingAngle = math_muldiv64_unsigned(ENCODER_ANGLE_DEGREES * p_encoder->Config.PartitionsPerRevolution, p_const->TIMER_FREQ, p_const->POLLING_FREQ * p_encoder->Config.CountsPerRevolution);
    //     p_encoder->InterpolateAngleLimit = ENCODER_ANGLE_DEGREES * p_encoder->Config.PartitionsPerRevolution / p_encoder->Config.CountsPerRevolution;
// }

/******************************************************************************/
/*
    FreqD - Pulse Frequency [counts/sec]
    Pure conversions between FreqD and standard representations.
    Parallel to math_angle_speed.h angle16/rpm conversions.
*/
/******************************************************************************/
/*
    FreqD [counts/sec] <=> RPM
    FreqD = RPM * CPR / 60
    RPM = FreqD * 60 / CPR
*/
#define COUNTER_FREQ_OF_RPM(cpr, rpm)     ((int64_t)(rpm) * (cpr) / 60)
#define COUNTER_RPM_OF_FREQ(cpr, freqD)   ((int64_t)(freqD) * 60 / (cpr))

static inline int32_t rpm_of_counter_freq(int32_t freqD, uint16_t countsPerRevolution) { return (int64_t)freqD * 60 / countsPerRevolution; }
static inline int32_t counter_freq_of_rpm(int32_t rpm, uint16_t countsPerRevolution) { return (int64_t)rpm * countsPerRevolution / 60; }

/*
    FreqD [counts/sec] <=> angle16 per poll cycle
    angle16_per_poll = FreqD * ANGLE16_PER_REVOLUTION / (CPR * pollingFreq)
*/
static inline int32_t angle_speed_of_counter_freq(uint32_t pollingFreq, uint16_t countsPerRevolution, int32_t freqD) { return (int64_t)freqD * ANGLE16_PER_REVOLUTION / ((int64_t)countsPerRevolution * pollingFreq); }
static inline int32_t counter_freq_of_angle_speed(uint32_t pollingFreq, uint16_t countsPerRevolution, int32_t angle16_per_poll) { return (int64_t)angle16_per_poll * countsPerRevolution * pollingFreq / ANGLE16_PER_REVOLUTION; }

/*
    Counts in per second (~1000x of SamplePeriod)
    Core ModeDT formula: FreqD = DeltaD * TimerFreq / PeriodT
*/
static inline int32_t counter_freq(int32_t deltaD, uint32_t timerFreq, uint32_t periodT) { return deltaD * (timerFreq / periodT); }



/******************************************************************************/
/*
    Addtional
*/
/******************************************************************************/
/*
    per timer mode
    Max = 2 * ScalarSpeedRef_Rpm * CountsPerRevolution / (60 * UnitTime_Freq)
*/
// static uint32_t max_delta(uint16_t counts_per_revolution, uint16_t speed_ref_rpm, uint32_t unit_time_freq)
// {
//     const uint32_t SECONDS_PER_MINUTE = 60U;
//     const uint32_t SAFETY_FACTOR = 2U;
//     return speed_ref_rpm * SAFETY_FACTOR * counts_per_revolution / (SECONDS_PER_MINUTE * unit_time_freq);
// }


// /*
//     Angle/S - Direct to per second.
//         same as RPS normalized to [0:65536]

//     AngleSpeed = DeltaD * [DEGREES * UnitTime_Freq / CountsPerRevolution] / DeltaT
//             <=> [AngleAccumPerCount * UnitTime_Freq >> SHIFT]

//     DeltaAngle[DEGREES]         = DegreesPerRevolution / CountsPerRevolution * DeltaD
//     AngularSpeed[DEGREES/s]     = DegreesPerRevolution / CountsPerRevolution * DeltaD * UnitTime_Freq[Hz] / DeltaT[TimerTicks]
//     Revolutions[N]              = DeltaD / CountsPerRevolution
//     RotationalSpeed[N/s]        = DeltaD / CountsPerRevolution * UnitTime_Freq[Hz] / DeltaT[TimerTicks]

//     e.g. DEGREES_BITS = 16,
//         UnitAngularSpeed = 160,000          : UnitTime_Freq = 20000, CountsPerRevolution = 8192
//         UnitAngularSpeed = 131,072          : UnitTime_Freq = 20000, CountsPerRevolution = 10000
//         UnitAngularSpeed = 8,000            : UnitTime_Freq = 1000, CountsPerRevolution = 8192
//         UnitAngularSpeed = 819,200,000      : UnitTime_Freq = 750000, CountsPerRevolution = 60

// */
// static uint64_t angle_speed_per_count(uint32_t unit_time_freq, uint16_t counts_per_revolution) { return (uint64_t)ANGLE16_PER_REVOLUTION * unit_time_freq / counts_per_revolution; }

// /* 15 fraction bits */
// static inline accum32_t angle32_speed_fract16_per_rpm(uint32_t UnitTime_Freq, uint32_t angle32_per_count, uint32_t speed_ref_rpm)
// {
//     return ((uint64_t)angle32_per_count * 60U * UnitTime_Freq) / (2U * speed_ref_rpm);
// }

// static inline accum32_t angle32_speed_(uint32_t UnitTime_Freq, uint32_t angle32_per_count)
// {
//     return ((uint64_t)angle32_per_count) / UnitTime_Freq;
// }

// static uint64_t linear_speed_unit_factor(uint16_t counts_per_revolution, uint32_t unit_time_freq, uint32_t surface_diameter, uint32_t gear_ratio_input, uint32_t gear_ratio_output)
// {
//     // assert(counts_per_revolution == 0);
//     // assert(gear_ratio_output == 0);
//     const uint32_t PI_SCALED = 314U;  // π * 100 for fixed-point
//     const uint32_t SCALE_FACTOR = 100U;

//     uint64_t numerator = (uint64_t)unit_time_freq * gear_ratio_input * surface_diameter * PI_SCALED;
//     uint64_t denominator = (uint64_t)counts_per_revolution * gear_ratio_output * SCALE_FACTOR;

//     return numerator / denominator;
// }
