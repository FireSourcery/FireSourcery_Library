
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
#include "angle_counter_fn.h"
#include "Angle.h"

/******************************************************************************/
/*
    Counter Config/Ref - Unit Conversion
*/
/******************************************************************************/
/* Runtime Ref - computed from calib + freq constants */
typedef struct Angle_CounterRef
{
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
    // int32_t CounterPrev;    /* Previous counter for DeltaD */
    uint32_t Angle32;

    /* Speed Counter */
    /* DeltaD/DeltaT Capture */
    int32_t DeltaD;         /* Counter counts (of distance) between 2 samples. Units in raw counter ticks */
    uint32_t PeriodT;       /* Timer counts between 2 pulse counts. Units in raw timer ticks */
    uint32_t DeltaTh;       /* ModeDT helper: timer value at last DeltaD capture */
    int32_t FreqD;          /* Pulse frequency [Hz]. DeltaD over 1 second */

    Angle_CounterRef_T Ref; /* Runtime unit conversion */
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

/* Timer domain constants - stored at init for FreqD computation */
/* Config Data Interface */
typedef struct Angle_CounterConfig
{
    uint16_t CountsPerRevolution;       /* Counter counts per mechanical revolution */
    uint32_t TimerFreq;                 /* Timer frequency [Hz] */
    uint32_t SampleFreq;                /* Sample frequency [Hz] */
    // Angle_SpeedFractCalib_T SpeedFractCalib;  base config
    uint32_t PollingFreq;               /* Polling frequency [Hz] */
    uint16_t FractSpeedRef_Rpm;         /* Reference speed for Fract16 normalization */

    //or split timer calib
    // uint32_t SamplePeriod;            /* Timer ticks per sample period: TimerFreq / SampleFreq */
    // uint16_t PartitionsPerRevolution;   /* Optionally module handle */
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
    p_counter->Angle32 += sign * p_counter->Ref.Angle32PerCount;
}


// static inline void Angle_Counter_CapturePeriodT(Angle_Counter_T * p_counter, uint32_t deltaT)
// {
//     p_counter->PeriodT = deltaT;
//     p_counter->DeltaTh = 0;
// }

// /* DeltaD capture - call at SampleFreq */
// // /* static inline void Angle_Counter_CaptureDeltaD(Angle_Counter_T * p_counter)
// // {
// //     p_counter->DeltaD = p_counter->CounterD - p_counter->CounterPrev;
// //     p_counter->CounterPrev = p_counter->CounterD;
// // } */
// static inline void Angle_Counter_CaptureDeltaD(Angle_Counter_T * p_counter)
// {
//     p_counter->DeltaD = p_counter->CounterD;
//     p_counter->CounterD = 0;
// }

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
// static inline void Angle_Counter_CaptureFreqD(Angle_Counter_T * p_counter, uint32_t deltaD, uint32_t deltaTh)
// {
//     const uint32_t timerFreq = p_counter->Ref.TimerFreq;
//     const uint32_t samplePeriod = p_counter->Ref.SamplePeriod; /* in Timer ticks */ /* periodTs = 1 / SAMPLE_FREQ */

//     uint32_t periodTk;

//     // Encoder_DeltaD_Capture(p_counter);

//     if (p_counter->DeltaD == 0)
//     {
//         /*  No pulses this sample. Same FreqD until next pulse. Accumulate DeltaTh on overflow */
//         p_counter->DeltaTh = (deltaTh == 0) ? (p_counter->DeltaTh + samplePeriod) : deltaTh;
//     }
//     else
//     {
//         /* Overflow is > samplePeriod. DeltaD == 0 occurs prior. */
//         periodTk = samplePeriod + (p_counter->DeltaTh - deltaTh);
//         if (periodTk > samplePeriod / 2)
//         {
//             p_counter->PeriodT = periodTk;
//             p_counter->FreqD = p_counter->DeltaD * (timerFreq / periodTk);
//         }

//         p_counter->DeltaTh = deltaTh;
//     }
// }

/* Wrapper: CaptureDeltaD + CaptureFreqD combined */
// static inline void Angle_Counter_CaptureSpeed(Angle_Counter_T * p_counter,   uint32_t deltaTh)
// {
//     Angle_Counter_CaptureDeltaD(p_counter);
//     Angle_Counter_CaptureFreqD(p_counter, p_ref, deltaTh);
// }


/******************************************************************************/
/*
    Bridge From Counter to Angle
*/
/******************************************************************************/
/*
    FreqD => using precomputed unit factor
*/
static inline int32_t _speed_fract16_of_counter_freq(uint32_t speed_per_count, int32_t freqD) { return (freqD * (int32_t)speed_per_count >> 15); }
static inline int32_t _angle_speed_of_counter_freq(uint32_t angle32_per_count, int32_t freqD) { return (freqD * (int32_t)angle32_per_count >> ANGLE_EXT_SHIFT); }

/*
    Angle Delta: angle increment per poll cycle
*/
static inline int32_t Angle_Counter_GetDelta(Angle_Counter_T * p_counter) { return _angle_speed_of_counter_freq(p_counter->Ref.AngleSpeed32PerCount, p_counter->FreqD); }
static inline int32_t Angle_Counter_GetSpeed(Angle_Counter_T * p_counter) { return _speed_fract16_of_counter_freq(p_counter->Ref.SpeedFractPerCount, p_counter->FreqD); }

static inline void Angle_Counter_ResolveInterpolationDelta(Angle_Counter_T * p_counter)
{
    p_counter->Base.Delta = _angle_speed_of_counter_freq(p_counter->Ref.AngleSpeed32PerCount, p_counter->FreqD);
    Angle_ResolveInterpolationDelta(&p_counter->Base);
}

static inline int32_t Angle_Counter_ResolveSpeed(Angle_Counter_T * p_counter)
{
    p_counter->Base.Speed_Fract16 = _speed_fract16_of_counter_freq(p_counter->Ref.SpeedFractPerCount, p_counter->FreqD);
    return p_counter->Base.Speed_Fract16;
}

/******************************************************************************/
/*
    Query
*/
/******************************************************************************/
static inline int32_t Angle_Counter_GetFreqD(const Angle_Counter_T * p_counter) { return p_counter->FreqD; }
/* Counter Delta */
static inline int32_t Angle_Counter_GetDeltaD(const Angle_Counter_T * p_counter) { return p_counter->DeltaD; }

/* FreqD-based RPM/RPS using stored CountsPerRevolution */
static inline int32_t Angle_Counter_GetRpm(const Angle_Counter_T * p_counter) { return rpm_of_count_freq(p_counter->Ref.CountsPerRevolution, p_counter->FreqD); }
static inline int32_t Angle_Counter_GetCps(const Angle_Counter_T * p_counter) { return cps_of_count_freq(p_counter->Ref.CountsPerRevolution, p_counter->FreqD); }

/******************************************************************************/
/*
    Init / Reset
*/
/******************************************************************************/
static inline void Angle_Counter_Zero(Angle_Counter_T * p_counter)
{
    p_counter->CounterD = 0;
    // p_counter->CounterPrev = 0;
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
    p_ref->SamplePeriod = p_config->TimerFreq / p_config->SampleFreq;
    p_ref->TimerFreq = p_config->TimerFreq;
    p_ref->CountsPerRevolution = p_config->CountsPerRevolution;
    p_ref->SpeedFractPerCount = rpm_accum32_per_count(1, p_config->CountsPerRevolution, p_config->FractSpeedRef_Rpm); /* For FreqD for now, or split */
    return p_ref;
}


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


