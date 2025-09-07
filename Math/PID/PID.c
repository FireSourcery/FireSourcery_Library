/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   PID.c
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#include "PID.h"
#include <string.h>
#include <assert.h>

static void ResetGains(PID_T * p_pid)
{
    PID_SetKp_Fixed32(p_pid, p_pid->Config.Kp_Fixed32);
    PID_SetKi_Fixed32(p_pid, p_pid->Config.Ki_Fixed32);
    PID_SetKd_Fixed32(p_pid, p_pid->Config.Kd_Fixed32);
}

void PID_InitFrom(PID_T * p_pid, const PID_Config_T * p_config)
{
    if (p_config != NULL) { memcpy(&p_pid->Config, p_config, sizeof(PID_Config_T)); }
    ResetGains(p_pid);
    PID_SetOutputLimits(p_pid, INT16_MIN, INT16_MAX);
    PID_Reset(p_pid);
}

/*!
    dynamic output limits
    update synchronous with proc
*/
void PID_CaptureOutputLimits(PID_T * p_pid, int16_t min, int16_t max)
{
    assert(max > min);
    p_pid->OutputMin = min;
    p_pid->OutputMax = max;
}

static inline int16_t GetIntegral(const PID_T * p_pid) { return (int16_t)(p_pid->IntegralAccum >> 15); }
static inline void SetIntegral(PID_T * p_pid, int16_t integral) { p_pid->IntegralAccum = ((int32_t)integral << 15); }

/*!
    Conventional parallel PID calculation
    @return control = (Kp * error) + (Ki * error * SampleTime + IntegralPrev) + (Kd * (error - ErrorPrev) / SampleTime)

    integral [-32768:32767] << 15
    error [-32768:32767] << 15
    for (INT32_MAX / 2) + (INT32_MAX / 2) without saturated add
    cannot be the case both integral and error are -32768
*/
static inline int32_t CalcPI(PID_T * p_pid, int16_t error)
{
    int32_t proportional, integral, integralAccum, integralMin, integralMax, output;

    proportional = ((int32_t)p_pid->PropGain * error) >> p_pid->PropGainShift; /* Includes 15 shift */

    /* Dynamic Clamp */
    integralMin = math_min(p_pid->OutputMin - proportional, 0);
    integralMax = math_max(p_pid->OutputMax - proportional, 0);

    /*
        Store as Integral ("integrate" then sum). Allows compute time gain adjustment.
            Alternatively, store as Riemann Sum. (Ki * ErrorSum * SampleTime)
        Forward rectangular approximation.
    */
    // assert(abs(p_pid->IntegralAccum) < INT32_MAX / 2);
    // assert(abs((p_pid->IntegralGain * error) >> p_pid->IntegralGainShift) < INT32_MAX / 2);
    integralAccum = p_pid->IntegralAccum + (((int32_t)p_pid->IntegralGain * error) >> p_pid->IntegralGainShift); /* Excludes 15 shift */
    p_pid->IntegralAccum = math_clamp(integralAccum, integralMin << 15, integralMax << 15);
    integral = p_pid->IntegralAccum >> 15;

    p_pid->ErrorPrev = error;

    return proportional + integral;
}

/*!
    @param[in] setpoint [-32768:32767] with over saturation
    @param[in] feedback [-32768:32767] with over saturation

    inputs upto 2x over saturation.
    error must be within [-32768:32767]

    e.g setpoint 32767, feedback 40000 => ok
        setpoint 32767, feedback -2 => overflow
*/
int16_t PID_ProcPI(PID_T * p_pid, int32_t feedback, int32_t setpoint)
{
    // error = math_clamp(setpoint - feedback, INT16_MIN, INT16_MAX);
    // assert(math_is_in_range(setpoint - feedback, INT16_MIN, INT16_MAX));
    p_pid->Output = math_clamp(CalcPI(p_pid, setpoint - feedback), p_pid->OutputMin, p_pid->OutputMax);
    return p_pid->Output;
}

// int32_t PID_ProcPID(PID_T * p_pid, int32_t feedback, int32_t setpoint)
// {
//     int32_t error = setpoint - feedback;
//     int32_t pi = CalcPI(p_pid, error);
//     int32_t derivative = (p_pid->KdFactorFreq * (error - p_pid->ErrorPrev) / p_pid->Config.KdDivisor);

//     p_pid->ErrorPrev = error;
//     p_pid->Output = math_clamp(pi + derivative, p_pid->OutputMin, p_pid->OutputMax);
//     return p_pid->Output;
// }

/*
    Compute-Time State
*/
void PID_Reset(PID_T * p_pid)
{
    p_pid->IntegralAccum = 0;
    p_pid->ErrorPrev = 0;
    p_pid->Output = 0;
}

static inline void SetOutputState(PID_T * p_pid, int16_t output)
{
    SetIntegral(p_pid, output);
    p_pid->Output = output;
    p_pid->ErrorPrev = 0;
}

/* allow int32_t input to clamp in 1 step */
void PID_SetOutputState(PID_T * p_pid, int16_t output)
{
    SetOutputState(p_pid, (int16_t)math_clamp(output, p_pid->OutputMin, p_pid->OutputMax));
}

void PID_SetOutputLimits(PID_T * p_pid, int16_t min, int16_t max)
{
    if (max > min)
    {
        p_pid->OutputMin = min;
        p_pid->OutputMax = max;
        // SetIntegral(p_pid, math_clamp(GetIntegral(p_pid), p_pid->OutputMin, p_pid->OutputMax));
        PID_SetOutputState(p_pid, GetIntegral(p_pid)); /* Reset integral with limits */
    }
}

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
/*
    Select Gain and Shift without overflow
    [Error Max] = 32,767 => Gain < 32,767
*/

/*!
    Proportional(k) = Kp * error(k)
                    = [kp_Fixed32 >> N] * error(k)

    Gain = kp_Fixed32 >> M
    GainShift = N - M

    [kp_Fixed32 >> N] = [PropGain >> PropGainShift], includes shift N
    PropGain = [kp_Fixed32 >> N << PropGainShift]
    [kp_Fixed32 >> N << PropGainShift] * [Error Max] < INT32_MAX
    PropGainShift < log2f(INT32_MAX / [Error Max] << N / kp_Fixed32)

    as Q17.15
    kp_Fixed32 = 32767, 1.0F => M = 0, Gain = (32,767, 15 - 0)
    kp_Fixed32 = [511 << 15], 511.0F => M = 9, Gain = (32,704, 15 - 9)

    @param[in] kp_Fixed32 Q17.15, 32767 => 1.0F
*/
static void SetKp_Fixed32(PID_T * p_pid, uint32_t kp_Fixed32)
{
    // p_pid->PropGainShift = math_limit_upper(30 - fixed_log2(kp_Fixed32), 15); /* discards negative shift */
    // p_pid->PropGain = kp_Fixed32 >> (15 - p_pid->PropGainShift);

    uint8_t m = math_limit_lower(fixed_bit_width(kp_Fixed32) - 15, 0); /* discards negative shift */
    p_pid->PropGainShift = 15 - m;
    p_pid->PropGain = kp_Fixed32 >> m;

    assert((kp_Fixed32 >> (15 - p_pid->PropGainShift)) < INT16_MAX);
}

/*!
    Integral(k) = Ki * error(k) / SampleFreq + Integral(k-1)
                = [ki_Fixed32 >> N / SampleFreq] * error(k) + Integral(k-1)

    Kp_Msb16 = kp_Fixed32 >> M

    Gain = ki_Fixed32 >> M / SampleFreq >> S
    GainShift = - S - M, excludes shift N

    [ki_Fixed32 / SampleFreq] = [IntegralGain >> IntegralGainShift], excludes shift N
    IntegralGain = [ki_Fixed32 / SampleFreq << IntegralGainShift]
    [ki_Fixed32 / SampleFreq << IntegralGainShift] * [Error Max] < [INT32_MAX / 2]
    IntegralGainShift < log2f([INT32_MAX / 2] / [Error Max] * SampleFreq / ki_Fixed32)

    SampleFreq = 10000:
    ki_Fixed32 = 32768 * 200 = 6,553,600 => (20,971, 5)

    ki_Fixed32 = 32768 * 200 = 6,553,600 => (20,971, 5) 21,473,648
    m = 8 , gain = 25,600, 14 - 8 = 6

    SampleFreq = 20000:
    ki_Fixed32 = 65536 * 255 = 16,711,680 => (26,843, 4)
    ki_Fixed32 = 256 => Gain = (26,843, 20)
    ki_Fixed32 = 65536 => Gain = (26,843, 12)

    @param[in] ki_Fixed32 Q17.15, 32767 => 1.0F
*/
static void SetKi_Fixed32(PID_T * p_pid, uint32_t ki_Fixed32)
{
    assert(ki_Fixed32 < INT32_MAX / 2);
    assert(p_pid->Config.SampleFreq != 0);

    p_pid->IntegralGainShift = math_limit_lower(15 - fixed_bit_width(ki_Fixed32 / p_pid->Config.SampleFreq), 0); /* left shift only */
    p_pid->IntegralGain = (ki_Fixed32 << p_pid->IntegralGainShift) / p_pid->Config.SampleFreq;

    assert((((int64_t)ki_Fixed32 << p_pid->IntegralGainShift) / p_pid->Config.SampleFreq) < INT16_MAX);
}


void PID_SetFreq(PID_T * p_pid, uint16_t sampleFreq)
{
    p_pid->Config.SampleFreq = sampleFreq;
    SetKi_Fixed32(p_pid, p_pid->Config.Ki_Fixed32);
    // SetKd_Fixed32(p_pid, p_pid->Config.Kd_Fixed32);
}

void PID_SetKp_Fixed32(PID_T * p_pid, int32_t kp_Fixed32)
{
    p_pid->Config.Kp_Fixed32 = kp_Fixed32;
    SetKp_Fixed32(p_pid, kp_Fixed32);
}

void PID_SetKi_Fixed32(PID_T * p_pid, int32_t ki_Fixed32)
{
    p_pid->Config.Ki_Fixed32 = ki_Fixed32;
    SetKi_Fixed32(p_pid, ki_Fixed32);
}

void PID_SetKd_Fixed32(PID_T * p_pid, int32_t kd_Fixed32)
{
    p_pid->Config.Kd_Fixed32 = kd_Fixed32; /* todo */
}

/*!
    @param[in] kp_Fixed16 [0:INT16_MAX] Q9.7
    Unsigned for resolution
*/

void PID_SetKp_Fixed16(PID_T * p_pid, uint16_t kp_Fixed16) { PID_SetKp_Fixed32(p_pid, (uint32_t)kp_Fixed16 << 8); }

void PID_SetKi_Fixed16(PID_T * p_pid, uint16_t ki_Fixed16) { PID_SetKi_Fixed32(p_pid, (uint32_t)ki_Fixed16 << 8); }

void PID_SetKd_Fixed16(PID_T * p_pid, uint16_t kd_Fixed16) { PID_SetKd_Fixed32(p_pid, (uint32_t)kd_Fixed16 << 8); }

