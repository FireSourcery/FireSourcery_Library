
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
    @file   AngleCounter.h
    @author FireSourcery
    @brief  Wrapping counter for Angle
            Stateful counter/pulse math for angle and speed estimation.
            Companion to Angle_T — using square wave encoders,
            Soft counter accumulation + mixed DeltaD/DeltaT (M/T) frequency estimation.
*/
/******************************************************************************/
#include "angle_counter_math.h"
#include "Angle.h"

/******************************************************************************/
/*
    Counter Config/Ref - Unit Conversion
*/
/******************************************************************************/
/* Runtime Ref - computed from calib + freq constants */
typedef struct AngleCounter_Ref
{
    /* wrapping angle */
    // angle16_t AnglePerCount;     /* AngleD Unit */
    uint32_t Angle32PerCount;       /* [(UINT32_MAX+1)/CountsPerRevolution] */
    uint32_t AngleDt32PerCount;     /* angle_dt32 per count_freq, at PollingFreq */
    uint32_t SpeedPuPerCount;       /* speed_pu per count_freq, Q30 */

    /* physical units handle with /cpr */
    uint16_t CountsPerRevolution;   /* Counter counts per mechanical revolution */
}
AngleCounter_Ref_T;


/******************************************************************************/
/*
    Counter State - Accumulation and M/T
*/
/******************************************************************************/
typedef struct AngleCounter
{
    Angle_T Base;           /* Holds accumulated angle */
    int32_t CounterD;       /* Signed displacement counter. Cleared for DeltaD */
    int32_t DeltaD;         /* Intermediate interface. */
    int32_t FreqD;          /* count_freq [count/s]. DeltaD over 1 second */
    AngleCounter_Ref_T UnitRef; /* Runtime unit conversion */
}
AngleCounter_T;

#define ANGLE_COUNTER_ALLOC() (&(AngleCounter_T){})


static inline Angle_T * AngleCounter_Angle(AngleCounter_T * p_counter) { return &p_counter->Base; }


/******************************************************************************/
/*
    Integrator from pulse count
    Count Accumulation
*/
/******************************************************************************/
/*!
    On pulse edge. Accumulates signed count.
    @param[in] sign { -1, 0, +1 } direction of this edge
*/
static inline void _AngleCounter_CaptureCount(AngleCounter_T * p_counter, int sign)
{
    p_counter->CounterD += sign;
}

static inline void AngleCounter_CaptureCount(AngleCounter_T * p_counter, int sign)
{
    p_counter->CounterD += sign;
    p_counter->Base.Angle += sign * p_counter->UnitRef.Angle32PerCount;
}

static inline void AngleCounter_ZeroCount(AngleCounter_T * p_counter) { p_counter->CounterD = 0; }

/*
    Directly set angle on sensor snapshot.
*/
static inline void AngleCounter_SetAngle(AngleCounter_T * p_counter, angle16_t angle) { Angle_SetAngle(&p_counter->Base, angle); }

static inline void AngleCounter_ZeroAngle(AngleCounter_T * p_counter) { Angle_ZeroAngle(&p_counter->Base); }

/* Optional seperate ResolveCounterDelta */
static inline int32_t AngleCounter_CaptureDeltaD(AngleCounter_T * p_counter)
{
    int32_t deltaD = p_counter->CounterD;
    p_counter->CounterD = 0;
    return deltaD;
}

/******************************************************************************/
/*
    M/T Frequency Estimation - call at SampleFreq (~1kHz)
    Samples DeltaD from CounterD, computes PeriodT from DeltaTh, runs M/T.

    @param[in] sampleTkFreq  timer_freq / sampleTk, from PulseTimer
*/
/******************************************************************************/
static inline void AngleCounter_CaptureFreq(AngleCounter_T * p_counter, uint32_t sampleTkFreq)
{
    int32_t deltaD = AngleCounter_CaptureDeltaD(p_counter);

    if (sampleTkFreq != 0) /* else bad sample */
    {
        /* sampleTkFreq is DeltaT Freq or accumulating SampleT Freq, returns 0 when < 1 pulse per second */
        p_counter->FreqD = ((deltaD != 0) ? deltaD : math_sign(p_counter->FreqD)) * (int32_t)sampleTkFreq;
    }
}

/******************************************************************************/
/*
    Bridge From Counter to Angle — FreqD drives Base.Delta for interpolation
*/
/******************************************************************************/
/*
    Propagate FreqD into Base.Delta as shifted Q16.16 angle increment per poll cycle.
    which lands directly in the tracker's shifted Delta.
    angle_dt32 = count_freq · AngleDt32PerCount
*/
static inline angle16_t AngleCounter_ResolveAngleDelta(AngleCounter_T * p_counter)
{
    p_counter->Base.Delta = (int32_t)p_counter->UnitRef.AngleDt32PerCount * p_counter->FreqD;
    return p_counter->Base.Delta >> ANGLE_EXT_SHIFT;
}

// static inline void AngleCounter_ResolveSpeed(AngleCounter_T * p_counter, uint32_t sampleTkFreq)
// {
//     AngleCounter_CaptureFreq(p_counter, sampleTkFreq);
//     AngleCounter_ResolveAngleDelta(p_counter);
// }

/*
    Angle_T Base forwarding — interpolation interface
*/
static inline angle16_t AngleCounter_Interpolate(AngleCounter_T * p_counter) { return Angle_Interpolate(&p_counter->Base); }

/* Without updating angle state */
static inline void AngleCounter_SetLimitWindow(AngleCounter_T * p_counter, uangle16_t width_angle16) { Angle_SetLimitWindow(&p_counter->Base, width_angle16); }
static inline void AngleCounter_SetLimits(AngleCounter_T * p_counter, angle16_t lower, angle16_t upper) { Angle_SetLimits(&p_counter->Base, lower, upper); }
static inline void AngleCounter_InitLimits(AngleCounter_T * p_counter, angle16_t limit_angle16) { Angle_InitLimits(&p_counter->Base, limit_angle16); }

/******************************************************************************/
/*
    Query
*/
/******************************************************************************/
static inline angle16_t AngleCounter_GetAngleDelta(AngleCounter_T * p_counter) { return Angle_Delta(&p_counter->Base); }
static inline int32_t AngleCounter_GetSpeed_Pu(AngleCounter_T * p_counter) { return (p_counter->FreqD * (int32_t)p_counter->UnitRef.SpeedPuPerCount >> 15); }

/* FreqD-based, of the counter revolution, using stored CountsPerRevolution */
static inline angle_freq_t AngleCounter_GetAngleFreq(const AngleCounter_T * p_counter) { return angle_freq_of_count_freq(p_counter->UnitRef.CountsPerRevolution, p_counter->FreqD); }
static inline int32_t AngleCounter_GetRpm(const AngleCounter_T * p_counter) { return rpm_of_count_freq(p_counter->UnitRef.CountsPerRevolution, p_counter->FreqD); }
static inline int32_t AngleCounter_GetCps(const AngleCounter_T * p_counter) { return cps_of_count_freq(p_counter->UnitRef.CountsPerRevolution, p_counter->FreqD); }

static inline int32_t AngleCounter_GetFreqD(const AngleCounter_T * p_counter) { return p_counter->FreqD; }
// static inline int32_t AngleCounter_GetDeltaD(const AngleCounter_T * p_counter) { return p_counter->DeltaD; }


/******************************************************************************/
/*
    Init / Reset
*/
/******************************************************************************/


/******************************************************************************/
/*
    Counter Ref Init - Compute runtime units from calibration
    The caller owns the inputs and builds the complete Ref. No Config: nothing here is persisted.
*/
/******************************************************************************/
/* sample_freq 1: FreqD is per second, runtime (timerFreq / periodTk) */
static inline AngleCounter_Ref_T AngleCounter_Ref(uint32_t pollingFreq, uint16_t countsPerRevolution, angle_freq_t angleFreqBase)
{
    return (AngleCounter_Ref_T)
    {
        .Angle32PerCount = angle32_per_count(countsPerRevolution),
        .AngleDt32PerCount = angle_dt32_per_count_cpr(pollingFreq, countsPerRevolution),
        .SpeedPuPerCount = (angleFreqBase != 0) ? angle_speed_pu32_per_count(1U, countsPerRevolution, angleFreqBase) : 0U, /* Base 0 is unset */
        .CountsPerRevolution = countsPerRevolution,
    };
}

static inline void AngleCounter_InitFrom(AngleCounter_T * p_counter, AngleCounter_Ref_T unitRef)
{
    p_counter->UnitRef = unitRef;
    AngleCounter_ZeroCount(p_counter);
}




