
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
    @file   Angle_Counter.h
    @author FireSourcery
    @brief  Stateful counter/pulse math for angle and speed estimation.
            Pure math — no HAL dependencies. Timer values passed as parameters.
            Parallel to Angle.h (which wraps math_angle_speed.h).
*/
/******************************************************************************/
#include "math_angle_counter.h"
#include "Angle.h"

/******************************************************************************/
/*
    Counter Config/Ref - Unit Conversion
    Follows Angle_SpeedFractRef_T / Angle_SpeedFractCalib_T pattern.
*/
/******************************************************************************/
/* Runtime Ref - computed from calib + freq constants */
typedef struct Angle_CounterRef
{
    angle16_t AnglePerCount;        /* AngleD Unit */
    uint32_t Angle32PerCount;       /* AngleD Unit */
    uint32_t AngleSpeed32PerCount;  /* AngleSpeed Unit at PollingFreq */
    uint32_t SpeedFractPerCount;    /* FractSpeed Unit */

    uint32_t SamplePeriod;          /* Timer ticks per sample period: TimerFreq / SampleFreq */
    uint32_t TimerFreq;             /* Timer frequency [Hz] */
}
Angle_CounterRef_T;

/******************************************************************************/
/*
    Counter State - Accumulation and ModeDT
*/
/******************************************************************************/
typedef struct Angle_Counter
{
    Angle_T Base;  /* */

    /* Count Accumulation */
    int32_t CounterD;       /* Signed pulse counter. Accumulated +1/-1 from edges */
    int32_t CounterPrev;    /* Previous counter for DeltaD */
    uint32_t Angle32;

    /* DeltaD/DeltaT Capture */
    int32_t DeltaD;         /* Count delta between samples */
    uint32_t PeriodT;       /* ModeDT period in timer ticks */

    /* ModeDT Speed */
    uint32_t DeltaTh;       /* ModeDT helper: timer value at last DeltaD capture */
    int32_t FreqD;          /* Pulse frequency [Hz]. DeltaD over 1 second */

    Angle_CounterRef_T Ref; /* Runtime unit conversion */
    // uint16_t CountsPerRevolution;       /* Counter counts per mechanical revolution */

    // Angle_CounterConfig_T hold config for runtime updates?

/* Timer domain constants - stored at init for FreqD computation */
    // uint32_t SamplePeriod;          /* Timer ticks per sample period: TimerFreq / SampleFreq */
    // uint32_t TimerFreq;             /* Timer frequency [Hz] */
}
Angle_Counter_T;

// typedef struct Angle_CounterTimerCalib
// {
//     uint32_t SampleFreq;              /* Sample frequency [Hz] */
//     // uint32_t SamplePeriod;         /* Timer ticks per sample period: TimerFreq / SampleFreq */
//     uint32_t TimerFreq;               /* Timer frequency [Hz] */
//     uint32_t PollingFreq;             /* Polling frequency [Hz] */
// }
// Angle_CounterTimerCalib_T;

/* Config Data Interface */
typedef struct Angle_CounterConfig
{
    uint16_t CountsPerRevolution;       /* Counter counts per mechanical revolution */
    // uint16_t PartitionsPerRevolution;   /* Optionally module handle */
    uint32_t TimerFreq;               /* Timer frequency [Hz] */
    uint32_t SampleFreq;              /* Sample frequency [Hz] */
    // uint32_t SamplePeriod;            /* Timer ticks per sample period: TimerFreq / SampleFreq */

    // Angle_SpeedFractCalib_T SpeedFractCalib;  base config
    uint32_t PollingFreq;             /* Polling frequency [Hz] */
    uint16_t FractSpeedRef_Rpm;        /* Reference speed for Fract16 normalization */

    //or split timer calib
}
Angle_CounterConfig_T;


/******************************************************************************/
/*
    Count Accumulation
*/
/******************************************************************************/
/*!
    On pulse edge. Accumulates signed count.
    @param[in] sign { -1, 0, +1 } direction of this edge
*/
static inline void Angle_Counter_CaptureCount(Angle_Counter_T * p_counter, int sign)
{
    p_counter->CounterD += sign;
}

/* DeltaD capture - call at SampleFreq */
static inline void Angle_Counter_CaptureDeltaD(Angle_Counter_T * p_counter)
{
    p_counter->DeltaD = p_counter->CounterD - p_counter->CounterPrev;
    p_counter->CounterPrev = p_counter->CounterD;
}

static inline void Angle_Counter_CapturePeriodT(Angle_Counter_T * p_counter, uint32_t deltaT)
{
    p_counter->PeriodT = deltaT;
    p_counter->DeltaTh = 0;
}


/******************************************************************************/
/*
    ModeDT - Mixed DeltaD/DeltaT Frequency Estimation
    Pure math — timer values passed as parameters, no HAL calls.
    Call at SampleFreq (~1kHz).

    @param[in] deltaTh  Current timer reading. 0 indicates timer overflow.
    @param[in] ref      Runtime constants (SamplePeriod, TimerFreq)

    Caller is responsible for:
        1. Calling Angle_Counter_CaptureDeltaD() prior
        2. Reading the timer value from HAL and passing it
        3. Passing 0 for deltaTh when timer overflow occurred
*/
/******************************************************************************/
static inline void Angle_Counter_CaptureFreqD(Angle_Counter_T * p_counter, uint32_t deltaD, uint32_t deltaTh)
{
    const uint32_t timerFreq = ;
    const uint32_t samplePeriod; /* in Timer ticks */ /* periodTs = 1 / SAMPLE_FREQ */

    uint32_t deltaTh;
    uint32_t periodTk;

    // Encoder_DeltaD_Capture(p_counter);

    if (p_counter->DeltaD == 0)
    {
        /*  No pulses this sample. Same FreqD until next pulse. Accumulate DeltaTh on overflow */
        p_counter->DeltaTh = (deltaTh == 0) ? (p_counter->DeltaTh + samplePeriod) : deltaTh;
    }
    else
    {
        /* Overflow is > samplePeriod. DeltaD == 0 occurs prior. */
        p_counter->DeltaTh = deltaTh;
        periodTk = samplePeriod + (p_counter->DeltaTh - deltaTh);
        if (periodTk > samplePeriod / 2)
        {
            p_counter->PeriodT = periodTk;
            p_counter->FreqD = p_counter->DeltaD * (timerFreq / periodTk);
        }

        p_counter->DeltaTh = deltaTh;
    }
}

/* Wrapper: CaptureDeltaD + CaptureFreqD combined */
// static inline void Angle_Counter_CaptureSpeed(Angle_Counter_T * p_counter,   uint32_t deltaTh)
// {
//     Angle_Counter_CaptureDeltaD(p_counter);
//     Angle_Counter_CaptureFreqD(p_counter, p_ref, deltaTh);
// }


/******************************************************************************/
/*
    Counter Speed Conversions - Using precomputed Ref units
    FreqD from Angle_Counter_T, unit conversion via Angle_CounterRef_T
*/
/******************************************************************************/
/*
    FreqD => using precomputed unit factor
*/
static inline int32_t _speed_fract16_of_counter_freq(uint32_t unitScalarSpeed, int32_t freqD) { return freqD * unitScalarSpeed >> ANGLE_EXT_SHIFT; }
static inline uint32_t _angle_speed_of_counter_freq(uint32_t unitPollingAngle, int32_t freqD) { return freqD * unitPollingAngle >> ANGLE_EXT_SHIFT; }

/*
    For interpolation: angle increment per poll cycle
*/
static inline uint32_t Angle_Counter_ResolveInterpolationDelta(Angle_Counter_T * p_counter)
{
    return p_counter->Base.Delta = _angle_speed_of_counter_freq(p_counter->Ref.Angle32PerCount, p_counter->FreqD);
    Angle_ResolveInterpolationDelta(&p_counter->Base);
    return p_counter->Base.Delta;
}

static inline uint32_t _Angle_Counter_ResolveSpeed(Angle_Counter_T * p_counter)
{
    p_counter->Base.Speed_Fract16 = _speed_fract16_of_counter_freq(p_counter->Ref.SpeedFractPerCount, p_counter->FreqD);
    return p_counter->Base.Speed_Fract16;
}

static inline uint32_t Angle_Counter_ResolveSpeed(Angle_Counter_T * p_counter)
{
    Angle_ResolveInterpolationDelta(&p_counter->Base);
    return _Angle_Counter_ResolveSpeed(p_counter);
}

/******************************************************************************/
/*
    Query
*/
/******************************************************************************/
static inline int32_t Angle_Counter_GetFreqD(const Angle_Counter_T * p_counter) { return p_counter->FreqD; }
static inline int32_t Angle_Counter_GetDeltaD(const Angle_Counter_T * p_counter) { return p_counter->DeltaD; }

/******************************************************************************/
/*
    Init / Reset
*/
/******************************************************************************/
static inline void Angle_Counter_Zero(Angle_Counter_T * p_counter)
{
    p_counter->CounterD = 0;
    p_counter->CounterPrev = 0;
    p_counter->DeltaD = 0;
    p_counter->DeltaTh = 0;
    p_counter->FreqD = 0;
    p_counter->PeriodT = 0;
}

/******************************************************************************/
/*
    Counter Ref Init - Compute runtime units from calibration
*/
/******************************************************************************/
static inline Angle_CounterRef_T * Angle_CounterRef_Init(Angle_CounterRef_T * p_ref, const Angle_CounterConfig_T * p_config)
{
    p_ref->AnglePerCount = angle_per_count(p_config->CountsPerRevolution);
    p_ref->Angle32PerCount = angle32_per_count(p_config->CountsPerRevolution);
    p_ref->AngleSpeed32PerCount = angle32_speed_per_count(p_config->PollingFreq, p_ref->Angle32PerCount);
    p_ref->SpeedFractPerCount = rpm_fract16_per_count(p_config->TimerFreq, p_config->CountsPerRevolution, p_config->FractSpeedRef_Rpm);
    p_ref->SamplePeriod = p_config->TimerFreq / p_config->SampleFreq;
    p_ref->TimerFreq = p_config->TimerFreq;
    return p_ref;
}



// /* FreqD => ScalarSpeed fract16 */
// static inline int32_t Angle_CounterRef_FractSpeed(const Angle_CounterRef_T * p_ref, int32_t freqD)
// {
//     // return counter_scalar_speed(freqD, p_ref->UnitFractSpeed, p_ref->UnitFractSpeedShift);
// }

// /* FreqD => signed scalar velocity with direction compensation */
// static inline int32_t Angle_CounterRef_FractVelocity(const Angle_CounterRef_T * p_ref, int32_t signedFreqD)
// {
//     // return counter_scalar_speed(signedFreqD, p_ref->UnitFractSpeed, p_ref->UnitFractSpeedShift);
// }

// /* FreqD => polling angle delta (shifted). For interpolation */
// static inline uint32_t Angle_CounterRef_PollingDelta(const Angle_CounterRef_T * p_ref, int32_t freqD)
// {
//     // return counter_polling_delta(freqD, p_ref->UnitPollingAngle);
// }

static inline void Angle_Counter_InitFrom(Angle_Counter_T * p_angle, const Angle_CounterConfig_T * p_config)
{
    Angle_CounterRef_Init(&p_angle->Ref, p_config);
    Angle_InitSpeedRef_Rpm(&p_angle->Base, p_config->PollingFreq, p_config->FractSpeedRef_Rpm);
    Angle_Counter_Zero(p_angle);
}

/******************************************************************************/
/*!
*/
/******************************************************************************/


