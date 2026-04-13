
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
    @brief  Stateful counter/pulse math for angle and speed estimation.
            Companion to Angle_T — using square wave encoders,
            Soft counter accumulation + mixed DeltaD/DeltaT (ModeDT) frequency estimation.
*/
/******************************************************************************/
#include "angle_counter_fn.h"
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
    angle16_t AnglePerCount;        /* AngleD Unit */
    uint32_t Angle32PerCount;       /*!< [(UINT32_MAX+1)/CountsPerRevolution] => Angle = PulseCounter * UnitAngleD >> DEGREES_SHIFT */

    /* positive only signed for umambigous multiply with deltaD */
    int32_t AngleSpeed32PerCount;  /* AngleSpeed Unit at PollingFreq */
    int32_t SpeedFractPerCount;    /* FractSpeed Unit */

    uint32_t SamplePeriod;          /* Timer ticks per sample period: TimerFreq / SampleFreq */
    uint32_t TimerFreq;             /* Timer frequency [Hz] */
    uint16_t CountsPerRevolution;   /* Counter counts per mechanical revolution */

    /*
        Unit conversion. derived on init from NvMem Config
    */
    // physical units handle with /cpr
    // uint32_t UnitLinearD;                   /*!< Linear D unit conversion factor. Units per TimerCounter tick, using Capture DeltaD (DeltaT = 1). Units per DeltaT capture, using Capture DeltaT (DeltaD = 1).*/
    // uint32_t UnitAngularSpeed;              /*!< [(1 << DEGREES_BITS) * UnitTime_Freq / CountsPerRevolution] => AngularSpeed = DeltaD * UnitAngularSpeed / DeltaT */
    // uint32_t UnitSurfaceSpeed;              /*!< [UnitD * UnitTime_Freq] => Speed = DeltaD * UnitSpeed / DeltaT */
}
AngleCounter_Ref_T;

static inline int32_t _speed_fract16_of_counter_freq(uint32_t speed_per_count, int32_t freqD) { return (freqD * (int32_t)speed_per_count >> 15); }
static inline int32_t _angle_speed_of_counter_freq(uint32_t angle32_per_count, int32_t freqD) { return (freqD * (int32_t)angle32_per_count >> ANGLE_EXT_SHIFT); }

/******************************************************************************/
/*
    Counter State - Accumulation and ModeDT
*/
/******************************************************************************/
typedef struct AngleCounter
{
    Angle_T Base;

    int32_t CounterD;       /* Signed pulse counter. Accumulated +1/-1 from edges */
    // int32_t CounterPrev; /* Previous counter value, if counter must be preserved */
    // int32_t DeltaD;
    int32_t FreqD;          /* Pulse frequency [Hz]. DeltaD over 1 second */

    AngleCounter_Ref_T Ref; /* Runtime unit conversion */
}
AngleCounter_T;


/* Units conversion */
typedef struct AngleCounter_Config
{
    uint16_t CountsPerRevolution;       /* Counter counts per mechanical revolution */
    uint32_t TimerFreq;                 /* Timer frequency [Hz] */
    uint32_t SampleFreq;                /* Sample frequency [Hz] */
    // Angle_SpeedFractCalib_T SpeedFractCalib;  base config
    uint32_t PollingFreq;               /* Polling frequency [Hz] */
    uint16_t FractSpeedRef_Rpm;         /* Reference speed for Fract16 normalization */
}
AngleCounter_Config_T;


/******************************************************************************/
/*
    Count Accumulation
*/
/******************************************************************************/
/*!
    On pulse edge. Accumulates signed count.
    @param[in] sign { -1, 0, +1 } direction of this edge
*/
static inline void AngleCounter_CaptureCount(AngleCounter_T * p_counter, int sign)
{
    p_counter->CounterD += sign;
}

static inline void AngleCounter_CaptureCountWrap(AngleCounter_T * p_counter, int sign)
{
    p_counter->CounterD += sign;
    p_counter->Base.Angle += sign * p_counter->Ref.Angle32PerCount;
}

/* Optional seperate capture */
// static inline int32_t AngleCounter_ResolveDeltaD(AngleCounter_T * p_counter)
// {
//     p_counter->DeltaD = p_counter->CounterD;
//     p_counter->CounterD = 0;
//     return p_counter->DeltaD;
// }

/******************************************************************************/
/*
    ModeDT Frequency Estimation - call at SampleFreq (~1kHz)
    Samples DeltaD from CounterD, computes PeriodT from DeltaTh, runs ModeDT.

    @param[in] periodTk  Timer reading from PulseTimer
*/
/******************************************************************************/
static inline void AngleCounter_CaptureFreqD(AngleCounter_T * p_freq, uint32_t periodTk)
{
    const uint32_t samplePeriod = p_freq->Ref.SamplePeriod;
    const uint32_t timerFreq = p_freq->Ref.TimerFreq;

    int32_t deltaD = p_freq->CounterD;
    p_freq->CounterD = 0;

    /* Same FreqD/speed until next pulse. or clear */
    // if (deltaD == 0) { p_freq->FreqD = 0;  /* assert(periodTk == 0) */ }
    /* Overflow is > samplePeriod. DeltaD == 0 occurs prior. */
    // else if (periodTk > samplePeriod / 2) { p_freq->FreqD = deltaD * (int32_t)(timerFreq / periodTk); }

    if ((deltaD != 0) && (periodTk > samplePeriod / 2)){ p_freq->FreqD = deltaD * (int32_t)(timerFreq / periodTk); }
}

/******************************************************************************/
/*
    Bridge From Counter to Angle — FreqD drives Base.Delta for interpolation
*/
/******************************************************************************/
static inline int32_t _AngleCounter_GetDelta(AngleCounter_T * p_counter) { return _angle_speed_of_counter_freq(p_counter->Ref.AngleSpeed32PerCount, p_counter->FreqD); }

/*
    Propagate FreqD into Base.Delta as shifted Q16.16 angle increment per poll cycle.
    _angle_speed_of_counter_freq produces a value in ANGLE_EXT_SHIFT domain,
    which lands directly in the tracker's shifted Delta.
*/
static inline angle16_t AngleCounter_ResolveInterpolationDelta(AngleCounter_T * p_counter)
{
    p_counter->Base.Delta = _AngleCounter_GetDelta(p_counter);
    return p_counter->Base.Delta >> ANGLE_EXT_SHIFT;
}

/*
    Angle_T Base forwarding — interpolation interface
*/
static inline angle16_t AngleCounter_Interpolate(AngleCounter_T * p_counter) { return Angle_Interpolate(&p_counter->Base); }
static inline void AngleCounter_ZeroAngle(AngleCounter_T * p_counter) { Angle_ZeroAngle(&p_counter->Base); }
static inline void AngleCounter_InitLimit(AngleCounter_T * p_counter, angle16_t limit_angle16) { Angle_InitLimit(&p_counter->Base, limit_angle16); }

/*
    Query — FreqD conversions
*/
static inline angle16_t AngleCounter_GetDelta(AngleCounter_T * p_counter) { return _AngleCounter_GetDelta(p_counter) >> ANGLE_EXT_SHIFT; }
static inline int32_t AngleCounter_GetSpeed_Fract16(AngleCounter_T * p_counter) { return _speed_fract16_of_counter_freq(p_counter->Ref.SpeedFractPerCount, p_counter->FreqD); }

/******************************************************************************/
/*
    Query
*/
/******************************************************************************/
/* FreqD-based RPM/RPS using stored CountsPerRevolution */
static inline int32_t AngleCounter_GetRpm(const AngleCounter_T * p_counter) { return rpm_of_count_freq(p_counter->Ref.CountsPerRevolution, p_counter->FreqD); }
static inline int32_t AngleCounter_GetCps(const AngleCounter_T * p_counter) { return cps_of_count_freq(p_counter->Ref.CountsPerRevolution, p_counter->FreqD); }

static inline int32_t AngleCounter_GetFreqD(const AngleCounter_T * p_counter) { return p_counter->FreqD; }
/* Counter Delta */
// static inline int32_t AngleCounter_GetDeltaD(const AngleCounter_T * p_counter) { return p_counter->DeltaD; }

/******************************************************************************/
/*
    Init / Reset
*/
/******************************************************************************/
static inline void AngleCounter_Zero(AngleCounter_T * p_counter)
{
    p_counter->CounterD = 0;
}

/******************************************************************************/
/*
    Counter Ref Init - Compute runtime units from calibration
*/
/******************************************************************************/
static inline void AngleCounter_Ref_Init(AngleCounter_Ref_T * p_ref, const AngleCounter_Config_T * p_config)
{
    p_ref->AnglePerCount = angle_per_count(p_config->CountsPerRevolution);
    p_ref->Angle32PerCount = angle32_per_count(p_config->CountsPerRevolution);
    p_ref->AngleSpeed32PerCount = angle32_speed_per_count(p_config->PollingFreq, p_ref->Angle32PerCount);
    p_ref->SamplePeriod = p_config->TimerFreq / p_config->SampleFreq;
    p_ref->TimerFreq = p_config->TimerFreq;
    p_ref->CountsPerRevolution = p_config->CountsPerRevolution;
    p_ref->SpeedFractPerCount = rpm_accum32_per_count(1, p_config->CountsPerRevolution, p_config->FractSpeedRef_Rpm); /* For FreqD for now, or split */
}


static inline void AngleCounter_InitFrom(AngleCounter_T * p_angle, const AngleCounter_Config_T * p_config)
{
    AngleCounter_Ref_Init(&p_angle->Ref, p_config);
    AngleCounter_Zero(p_angle);
    // Angle_InitSpeedRef_Rpm(&p_angle->Base, p_config->PollingFreq, p_config->FractSpeedRef_Rpm);
}

/******************************************************************************/
/*!
*/
/******************************************************************************/

