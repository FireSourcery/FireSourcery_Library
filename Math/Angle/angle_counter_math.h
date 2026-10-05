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
    @file   angle_counter_math.h
    @author FireSourcery
    @brief  angle math using cpr. Counter forms to the [angle_speed_math.h] canonical forms.

        count                       position, cpr counts per revolution
        count_freq [count/s]        FreqD. ΔD per sample at sample_freq, or per ΔT ticks of timer_freq
        angle_freq [angle16/s]      = count_freq · 65536 / cpr, of the counter revolution
        angle_dt   [angle16/Ts]     = angle_freq / Fs, Fs the polling freq

    [Hz], [rpm] are views at the user boundary.
*/
/******************************************************************************/
#include "angle_speed_math.h"
#include "../Fixed/fract16.h"

#define ANGLE_EXT_SHIFT (16)

/******************************************************************************/
/*!
    Unit Calculation Functions - Pure Functions
*/
/******************************************************************************/
/*!
    @brief Encoder Counter
*/
/* count: [CountsPerRevolution:65536/2] */
static inline uint32_t angle_of_count(uint32_t cpr, uint32_t count) { return ((count * ANGLE16_PER_REVOLUTION) / cpr); }
static inline uint32_t count_of_angle(uint32_t cpr, uint32_t angle16) { return ((angle16 * cpr) / ANGLE16_PER_REVOLUTION); }


/*
    Angle Accum Unit
    Angle = Counts * [(DEGREES << SHIFT) / CountsPerRevolution] >> SHIFT
*/
/* UINT32_MAX ~= ANGLE_PER_REVOLUTION << ANGLE_EXT_SHIFT */
static inline uint32_t angle32_per_count(uint32_t counts_per_revolution) { return UINT32_MAX / counts_per_revolution + 1U; } /* +1 to round up */
/* lossless for counts is pow2 */
// static inline uint32_t angle_per_count(uint32_t counts_per_revolution) { return ANGLE16_PER_REVOLUTION / counts_per_revolution; }

/* counter [0:CountsPerRevolution], wrapped counter value */
static inline uint32_t angle_of_counter(uint32_t angle32PerCount, uint32_t counter) { return ((counter * angle32PerCount) >> ANGLE_EXT_SHIFT); }

/* counter [0:CountsPerRevolution], wrapped counter value */
static inline uint32_t angle_counter_wrapped(uint32_t max, uint32_t prev, uint32_t count) { return (count < prev) ? (max + 1U + count - prev) : (count - prev); }


/******************************************************************************/
/*
    Speed — count_freq to the canonical forms
*/
/******************************************************************************/
/*
    Angle Freq [angle16/s]
        angle_freq = (count / s) · [65536 / cpr]

    Angle Dt, Polling Unit [angle16/Ts], Fs the polling freq
        angle_dt = (count / s) · [(65536 / cpr) / Fs]

    FreqD at runtime, as angle_dt << ANGLE_EXT_SHIFT, the [Angle_T] Delta form:
        Delta = FreqD · [(65536 << ANGLE_EXT_SHIFT) / cpr / Fs] = FreqD · AngleDt32PerCount
        Delta < 2^31 => |FreqD| < cpr · Fs / 2 => counts per poll < cpr / 2
    Alternatively at run time, without the factor truncation
        angle_dt = FreqD · [65536 / cpr] / Fs
*/
static inline angle_freq_t angle_freq_of_count_freq(uint32_t cpr, int32_t count_freq) { return (int64_t)count_freq * ANGLE16_PER_REVOLUTION / cpr; }
static inline int32_t count_freq_of_angle_freq(uint32_t cpr, angle_freq_t freq) { return (int64_t)freq * cpr / ANGLE16_PER_REVOLUTION; }

static inline int32_t angle_dt_of_count_freq(uint32_t fs, uint32_t cpr, int32_t count_freq) { return (int64_t)count_freq * ANGLE16_PER_REVOLUTION / ((int64_t)cpr * fs); }
static inline int32_t count_freq_of_angle_dt(uint32_t fs, uint32_t cpr, angle_dt_t angle_dt) { return (int64_t)angle_dt * cpr * fs / ANGLE16_PER_REVOLUTION; }


/******************************************************************************/
/*
    Runtime factors, precomputed
*/
/******************************************************************************/
/*
    AngleDt32PerCount = [(65536 << ANGLE_EXT_SHIFT) / cpr / Fs], truncated
        e.g. Fs = 20000: cpr 6 => 35,791 (35,791.4), cpr 1024 => 209 (209.7), cpr 8192 => 26 (26.2)
*/
static inline uint32_t angle_dt32_per_count_cpr(uint32_t fs, uint16_t cpr) { return angle32_per_count(cpr) / fs; }
static inline uint32_t angle_dt32_per_count(uint32_t fs, uint32_t angle32_per_count) { return angle32_per_count / fs; }

/*
    Speed Pu [angle_freq / base], base [angle16/s] of the counter revolution
        speed_pu = ΔD · [sample_freq · 65536 / (cpr · base)]                    ΔD per sample
        speed_pu = [ΔD / ΔT] · [timer_freq · 65536 / (cpr · base)]              ΔD over ΔT timer ticks
        speed_pu = FreqD · [65536 / (cpr · base)]                               FreqD = ΔD · sample_freq, sample_freq = 1

    FreqD at runtime, Q30 factor for precision, >> 15 at use:
        Speed_Pu [Q15] = FreqD · [2^30 · 65536 / (cpr · base)] >> 15 = FreqD · SpeedPuPerCount >> 15
        [0:base] => [0:2^30], ~INT32_MAX / 2. Overflows past 2.0 pu
        Factor fits uint32 for cpr · base > 2^14 · sample_freq. e.g. sample_freq = 1000: cpr · base_rpm > 15,000

    e.g. base 14100 rpm = 15,400,960 [angle16/s]
        cpr 24 (Hall, 4 pole pairs) => 190,379, FreqD 5640 at 1.0
        cpr 1024                    => 4,462, FreqD 240,640 at 1.0
*/
static inline uint32_t angle_speed_pu32_per_count(uint32_t sample_freq, uint32_t cpr, angle_freq_t base) { return ((uint64_t)(FRACT16_SCALE << 15) * ANGLE16_PER_REVOLUTION * sample_freq) / ((uint64_t)cpr * base); }

// /* Generic base, args scaled by caller. speed_pu = ΔD · [sample_freq / cpr], 1.0 at 1 rev/s. e.g. rpm base: (sample_freq · 60, cpr · base_rpm) */
// static inline uint32_t accum32_per_count(uint32_t sample_freq, uint32_t cpr) { return ((uint64_t)(FRACT16_SCALE << 15) * sample_freq) / cpr; }

// /* speed_pu = ΔD · [sample_freq · 60 / (cpr · base_rpm)] */
// static inline uint32_t rpm_accum32_per_count(uint32_t sample_freq, uint32_t cpr, uint32_t base_rpm) { return ((uint64_t)(FRACT16_SCALE << 15) * SECONDS_PER_MINUTE * sample_freq) / ((uint64_t)cpr * base_rpm); }


/******************************************************************************/
/*!
    @brief  Per second, user boundary SI views of count_freq
*/
/******************************************************************************/
/*
    Angle/S - Direct to per second. [angle_freq_t], RPS normalized to [0:65536]
        angle_freq [angle16/s]  : 65536 / cpr · ΔD · freq [Hz] / ΔT
        cps [rev/s]             : ΔD / cpr · freq [Hz] / ΔT
        rpm                     : ΔD / cpr · freq [Hz] · 60 / ΔT
    freq is timer_freq over ΔT timer ticks, or sample_freq with ΔT = 1. FreqD = ΔD · freq / ΔT

    angle_freq = ΔD · [65536 · freq / cpr] / ΔT
            <=> ΔD · [angle32_per_count · freq >> ANGLE_EXT_SHIFT] / ΔT

    e.g. [65536 · freq / cpr]
        160,000         : freq = 20000, cpr = 8192
        131,072         : freq = 20000, cpr = 10000
        8,000           : freq = 1000, cpr = 8192
        819,200,000     : freq = 750000, cpr = 60
*/
/*
    FreqD [count/s] <=> rpm
        FreqD = rpm · cpr / 60
        rpm = FreqD · 60 / cpr
*/
static inline int32_t cps_of_count_freq(uint16_t cpr, int32_t count_freq) { return count_freq / (int32_t)cpr; }
static inline int32_t rpm_of_count_freq(uint16_t cpr, int32_t count_freq) { return (int64_t)count_freq * SECONDS_PER_MINUTE / cpr; }
static inline int32_t count_freq_of_rpm(uint16_t cpr, int32_t rpm) { return (int64_t)rpm * cpr / SECONDS_PER_MINUTE; }

// static inline int32_t cps_of_count(uint32_t sampleFreq, uint32_t cpr, int32_t deltaD) { return (int64_t)deltaD * sampleFreq / cpr; }
// static inline int32_t count_of_cps(uint32_t sampleFreq, uint32_t cpr, int32_t cps) { return (int64_t)cps * cpr / sampleFreq; }
// static inline int32_t rpm_of_count(uint32_t sampleFreq, uint32_t cpr, int32_t deltaD) { return (int64_t)deltaD * sampleFreq * SECONDS_PER_MINUTE / cpr; }
// static inline int32_t count_of_rpm(uint32_t sampleFreq, uint32_t cpr, int32_t rpm) { return (int64_t)rpm * cpr / ((int64_t)sampleFreq * SECONDS_PER_MINUTE); }


/******************************************************************************/
/*
    Specialized capture mode polling angle interpolation
    ΔD counts over ΔT ticks of the timer_freq clock
*/
/******************************************************************************/
/*
    angle_dt = ΔD [count] / ΔT [tick] · timer_freq [tick/s] · [65536 / cpr] / Fs
             = ΔD · [65536 · timer_freq / Fs / cpr] / ΔT

    Angle    = AngleIndex · angle_dt
             = AngleIndex · [65536 / cpr · timer_freq / Fs] / ΔT                 ΔD = 1
    AngleIndex [0:InterpolationCount]
*/
/*!
    Estimate Angle each control cycle in between encoder counts

    Only when POLLING_FREQ > PulseFreq, i.e. 0 encoder counts per poll, polls per encoder count > 1
    e.g. High res break even point
        8192 CountsPerRevolution, 20Khz POLLING_FREQ => 146 RPM
*/
/*
    InterpolationCount - numbers of Polls per encoder count, per DeltaT Capture, AngleIndex max
    POLLING_FREQ/EncoderPulseFreq == POLLING_FREQ / (TIMER_FREQ / DeltaT);
*/
static inline angle_freq_t angle_freq_of_count_ticks(uint32_t timer_freq, uint32_t cpr, int32_t deltaD, uint32_t deltaT) { return (int64_t)deltaD * ANGLE16_PER_REVOLUTION * timer_freq / ((int64_t)cpr * deltaT); }
static inline int32_t angle_dt_of_count_ticks(uint32_t fs, uint32_t timer_freq, uint32_t cpr, int32_t deltaD, uint32_t deltaT) { return (int64_t)deltaD * ANGLE16_PER_REVOLUTION * timer_freq / ((int64_t)fs * cpr * deltaT); }

/* [65536 · timer_freq / Fs / cpr]. angle_dt = ΔD · factor / ΔT, ΔT the sampleTk spanning ΔD > 1 counts at large cpr */
static inline uint32_t angle_dt_ticks_per_count(uint32_t fs, uint32_t timer_freq, uint16_t cpr) { return (uint64_t)ANGLE16_PER_REVOLUTION * timer_freq / ((uint64_t)fs * cpr); }
// static inline uint32_t polling_count_of_delta_t(uint32_t polling_freq, uint32_t timer_freq, uint16_t cpr, uint32_t delta_t) { return (uint64_t)polling_freq * delta_t /timer_freq; }



/*
 */
// static inline uint32_t count_of_linear_speed(uint32_t sampleFreq, uint32_t cpr, uint32_t speed_UnitsPerSecond)
// static inline uint32_t linear_speed_of_count(uint32_t sampleFreq, uint32_t cpr, uint32_t deltaD_Ticks)
// static uint64_t linear_speed_unit_factor(uint16_t counts_per_revolution, uint32_t unit_time_freq, uint32_t surface_diameter, uint32_t gear_ratio_input, uint32_t gear_ratio_output)
// {
//     uint64_t numerator = (uint64_t)unit_time_freq * gear_ratio_input * surface_diameter * PI_SCALED;
//     uint64_t denominator = (uint64_t)counts_per_revolution * gear_ratio_output * SCALE_FACTOR;
//     return numerator / denominator;
// }

// static inline uint32_t Encoder_GroundSpeedOf_Mph(UnitSurfaceSpeed, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
// {
//     return deltaD_Ticks *  UnitSurfaceSpeed * 60U * 60U / (deltaT_Ticks * 1609344U);
// }

// static inline uint32_t Encoder_GroundSpeedOf_Kmh(UnitSurfaceSpeed, uint32_t deltaD_Ticks, uint32_t deltaT_Ticks)
// {
//     return deltaD_Ticks *  UnitSurfaceSpeed * 60U * 60U / (deltaT_Ticks * 1000000U);
// }

