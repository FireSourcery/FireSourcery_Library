
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

/******************************************************************************/
/*
    Counter Config/Ref - Unit Conversion
    Follows Angle_SpeedFractRef_T / Angle_SpeedFractCalib_T pattern.
*/
/******************************************************************************/
/* Calibration Config - stored/NVM */
typedef struct Angle_CounterConfig
{
    uint16_t CountsPerRevolution;       /* Counter counts per mechanical revolution */
    // uint16_t PartitionsPerRevolution;   /* Electrical partitions. e.g. PolePairs for Hall */
    // uint16_t ScalarSpeedRef_Rpm;        /* Reference speed for Fract16 normalization */

    uint32_t SampleTime;            /* Timer ticks per sample period: TimerFreq / SampleFreq */
    uint32_t TimerFreq;             /* Timer frequency [Hz] */
}
Angle_CounterConfig_T;

/* Runtime Ref - computed from calib + freq constants */
typedef struct Angle_CounterRef
{
    // uint32_t UnitAngleD;            /* angle32 per count: UINT32_MAX / CountsPerRevolution + 1 */
    // uint32_t UnitPollingAngle;      /* angle32 per count per poll cycle. For interpolation delta */
    // uint32_t InterpolateAngleLimit; /* angle32 clamp per partition */
    // uint32_t UnitScalarSpeed;       /* Scalar speed conversion factor */
    // uint8_t UnitScalarSpeedShift;

    angle16_t AnglePerCount;
    uint32_t AngleAccumPerCount;
    uint32_t AngleAccumPerPoll;
    uint32_t SpeedFractPerCount;
    /* Timer domain constants - stored at init for FreqD computation */
    uint32_t SampleTime;            /* Timer ticks per sample period: TimerFreq / SampleFreq */
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
    /* Count Accumulation */
    int32_t CounterD;       /* Signed pulse counter. Accumulated +1/-1 from edges */
    int32_t CounterPrev;    /* Previous counter for DeltaD */

    /* DeltaD/DeltaT Capture */
    int32_t DeltaD;         /* Count delta between samples */

    /* ModeDT Speed */
    uint32_t DeltaTh;       /* ModeDT helper: timer value at last DeltaD capture */
    int32_t FreqD;          /* Pulse frequency [Hz]. DeltaD over 1 second */
    uint32_t PeriodT;       /* ModeDT period in timer ticks */
}
Angle_Counter_T;

/******************************************************************************/
/*
    Count Accumulation
*/
/******************************************************************************/
/*
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
    @param[in] ref      Runtime constants (SampleTime, TimerFreq)

    Caller is responsible for:
        1. Calling Angle_Counter_CaptureDeltaD() prior
        2. Reading the timer value from HAL and passing it
        3. Passing 0 for deltaTh when timer overflow occurred
*/
/******************************************************************************/
static inline void Angle_Counter_CaptureFreqD(Angle_Counter_T * p_counter, uint32_t deltaTh, uint32_t deltaTh)
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
static inline void Angle_Counter_CaptureSpeed(Angle_Counter_T * p_counter, const Angle_CounterRef_T * p_ref, uint32_t deltaTh)
{
    Angle_Counter_CaptureDeltaD(p_counter);
    Angle_Counter_CaptureFreqD(p_counter, p_ref, deltaTh);
}

/******************************************************************************/
/*
    Counter Speed Conversions - Using precomputed Ref units
    FreqD from Angle_Counter_T, unit conversion via Angle_CounterRef_T
*/
/******************************************************************************/
/* FreqD => ScalarSpeed fract16 */
static inline int32_t Angle_Counter_ScalarSpeed(const Angle_CounterRef_T * p_ref, int32_t freqD)
{
    // return counter_scalar_speed(freqD, p_ref->UnitScalarSpeed, p_ref->UnitScalarSpeedShift);
}

/* FreqD => signed scalar velocity with direction compensation */
static inline int32_t Angle_Counter_ScalarVelocity(const Angle_CounterRef_T * p_ref, int32_t signedFreqD)
{
    // return counter_scalar_speed(signedFreqD, p_ref->UnitScalarSpeed, p_ref->UnitScalarSpeedShift);
}

/* FreqD => polling angle delta (shifted). For interpolation */
static inline uint32_t Angle_Counter_PollingDelta(const Angle_CounterRef_T * p_ref, int32_t freqD)
{
    // return counter_polling_delta(freqD, p_ref->UnitPollingAngle);
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
static inline void Angle_Counter_InitRef(Angle_CounterRef_T * p_ref, const Angle_CounterConfig_T * p_config, uint32_t timerFreq, uint32_t sampleFreq, uint32_t pollingFreq)
{
    // p_ref->UnitAngleD = angle_encoder_unit_shifted(p_config->CountsPerRevolution);
    // p_ref->UnitPollingAngle = polling_angle_unit_factor(p_ref->UnitAngleD, p_config->PartitionsPerRevolution, pollingFreq);
    // p_ref->InterpolateAngleLimit = interpolation_angle_limit(p_ref->UnitAngleD, p_config->PartitionsPerRevolution);
    // p_ref->UnitScalarSpeedShift = 14U;
    // p_ref->UnitScalarSpeed = scalar_speed_unit_factor(p_config->CountsPerRevolution, p_config->ScalarSpeedRef_Rpm, sampleFreq, p_ref->UnitScalarSpeedShift);
    p_ref->SampleTime = timerFreq / sampleFreq;
    p_ref->TimerFreq = timerFreq;
}




/******************************************************************************/
/*!
    Config Units
    todo split module
    Units
*/
/******************************************************************************/


