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

// static void ResetGains(PID_T * p_pid)
// {
//     PID_SetKp_Fixed32(p_pid, p_pid->Config.Kp_Fixed32);
//     PID_SetKi_Fixed32(p_pid, p_pid->Config.Ki_Fixed32);
//     PID_SetKd_Fixed32(p_pid, p_pid->Config.Kd_Fixed32);
// }

void PID_InitFrom(PID_T * p_pid, const PID_Config_T * p_config)
{
    if (p_config != NULL) { p_pid->Config = *p_config; }
    PID_SetKp_Fixed32(p_pid, p_pid->Config.Kp);
    PID_SetKi_Fixed32(p_pid, p_pid->Config.Ki);
    PID_SetKd_Fixed32(p_pid, p_pid->Config.Kd);
    PID_SetOutputLimits(p_pid, INT16_MIN, INT16_MAX);
    PID_Reset(p_pid);
}


static inline int16_t GetIntegral(const PID_T * p_pid) { return (int16_t)(p_pid->IntegralAccum >> 15); }
static inline void SetIntegral(PID_T * p_pid, int16_t integral) { p_pid->IntegralAccum = ((int32_t)integral << 15); }

/* passing limits for internal state */
// int16_t PID_ProcAntiWindupPI(PID_T * p_pid, int16_t limitLow, int16_t limitHigh, int16_t error)
// {
//     int32_t proportional = ((int32_t)p_pid->PropGain * error) >> p_pid->PropGainShift; /* Includes 15 shift */

//     /* Dynamic Clamp */
//     int32_t integralMin = limitLow - proportional;
//     int32_t integralMax = limitHigh - proportional;

//     int32_t integralAccum = p_pid->IntegralAccum + (((int32_t)p_pid->IntegralGain * error) >> p_pid->IntegralGainShift); /* Excludes 15 shift */
//     p_pid->IntegralAccum = math_clamp(integralAccum, integralMin << 15, integralMax << 15);
//     int32_t integral = p_pid->IntegralAccum >> 15;

//     p_pid->ErrorPrev = error;

//     /* Optionally output limits as persistent saturation limits */
//     p_pid->Output = math_clamp(proportional + integral, p_pid->OutputMin, p_pid->OutputMax);
//     return p_pid->Output;
// }

/*!
    Conventional parallel PID calculation
    @return control = (Kp * error) + (Ki * error * SampleTime + IntegralPrev) + (Kd * (error - ErrorPrev) / SampleTime)

    integralAccum [-32768:32767] << 15
    (KiPerTick.mantissa * 65535) >> KiPerTick.shift < INT32_MAX / 2: integralAccum + increment without saturation
*/
static inline int32_t CalcPI(PID_T * p_pid, int32_t error)
{
    int32_t proportional = fract16e_mul(p_pid->Kp, error); /* Includes 15 shift */

    /*
        Dynamic Clamp, tracks the band.
        [OutputMin - proportional, OutputMax - proportional] moves the integral with the band, saturation releases without unwinding.
        PID_CaptureOutputLimits relies on it, e.g. the Vq band shrinking under field weakening. Anchored at the accumulator, the integral is left outside the band.
        Saturated to the accumulator range ahead of the shift.
    */
    // int32_t integralMin = math_min(p_pid->IntegralAccum, math_clamp(p_pid->OutputMin - proportional, p_pid->OutputMin, p_pid->OutputMax) << 15);
    // int32_t integralMax = math_max(p_pid->IntegralAccum, math_clamp(p_pid->OutputMax - proportional, p_pid->OutputMin, p_pid->OutputMax) << 15);

    // int32_t integralMin = math_clamp(p_pid->OutputMin - proportional, p_pid->OutputMin, p_pid->OutputMax) << 15;
    // int32_t integralMax = math_clamp(p_pid->OutputMax - proportional, p_pid->OutputMin, p_pid->OutputMax) << 15;
    // int32_t integralMin = math_min(p_pid->OutputMin - proportional, p_pid->OutputMax) << 15;
    // int32_t integralMax = math_max(p_pid->OutputMax - proportional, p_pid->OutputMin) << 15;
    int32_t integralMin = math_clamp(p_pid->OutputMin - proportional, INT16_MIN, p_pid->OutputMax) << 15;
    int32_t integralMax = math_clamp(p_pid->OutputMax - proportional, p_pid->OutputMin, INT16_MAX) << 15;

    /*
        Store as Integral ("integrate" then sum). Allows compute time gain adjustment.
            Alternatively, store as Riemann Sum. (Ki * ErrorSum * SampleTime)
        Forward rectangular approximation.
    */
    int32_t integralAccum = p_pid->IntegralAccum + fract16e_mul(p_pid->KiPerTick, error); /* Excludes 15 shift */
    p_pid->IntegralAccum = math_clamp(integralAccum, integralMin, integralMax);
    int32_t integral = p_pid->IntegralAccum >> 15;

    p_pid->ErrorPrev = error;

    return proportional + integral;
}


/*!
    dynamic output limits
    update synchronous with proc
*/
void PID_CaptureOutputLimits(PID_T * p_pid, int16_t min, int16_t max)
{
    assert(max >= min);
    if (max >= min) /* allow 0 width band */
    {
        p_pid->OutputMin = min;
        p_pid->OutputMax = max;
    }
}

/*!
    @param[in] setpoint [-32768:32767]
    @param[in] feedback [-32768:32767]
*/
int16_t PID_ProcPI(PID_T * p_pid, int16_t feedback, int16_t setpoint)
{
    p_pid->Output = math_clamp(CalcPI(p_pid, (int32_t)setpoint - (int32_t)feedback), p_pid->OutputMin, p_pid->OutputMax);
    // p_pid->Output = PID_ProcAntiWindupPI(p_pid, p_pid->OutputMin, p_pid->OutputMax, (int32_t)setpoint - feedback);
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

 void _PID_SetOutputState(PID_T * p_pid, int16_t output)
 {
    assert(output >= p_pid->OutputMin && output <= p_pid->OutputMax);
    SetIntegral(p_pid, output);
    p_pid->Output = output;
}

/* allow int32_t input to clamp in 1 step */
void PID_SetOutputState(PID_T * p_pid, int16_t output)
{
    _PID_SetOutputState(p_pid, (int16_t)math_clamp(output, p_pid->OutputMin, p_pid->OutputMax));
}

/* Persistent limits, the integral moves into range. PID_CaptureOutputLimits leaves it to the dynamic clamp */
void PID_SetOutputLimits(PID_T * p_pid, int16_t min, int16_t max)
{
    if (max >= min) /* allow 0 width band */
    {
        p_pid->OutputMin = min;
        p_pid->OutputMax = max;
        p_pid->IntegralAccum = math_clamp(p_pid->IntegralAccum, (int32_t)min << 15, (int32_t)max << 15);
    }
}

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
/*
    Select mantissa and shift without overflow
    [Error Max] = 65,535, setpoint - feedback
*/

/*!
    Proportional(k) = Kp * error(k)
                    = [kp_Fixed32 >> N] * error(k)

    [kp_Fixed32 >> N] = Kp = mantissa >> shift, includes shift N
        mantissa = kp_Fixed32 >> M, M integer bits of kp_Fixed32
        shift = N - M

    mantissa * [Error Max] < INT32_MAX: |mantissa| < 2^15

    as Q17.15
    kp_Fixed32 = 32767, 1.0F => M = 0, Kp = (32,767, 15 - 0)
    kp_Fixed32 = [511 << 15], 511.0F => M = 9, Kp = (32,704, 15 - 9)

    @param[in] kp_Fixed32 Q17.15, 32767 => 1.0F
*/
static void SetKp_Fixed32(PID_T * p_pid, uint32_t kp_Fixed32) { p_pid->Kp = fract16e_of(kp_Fixed32); }

/*!
    Integral(k) = Ki * error(k) / SampleFreq + Integral(k-1)
                = [ki_Fixed32 >> N / SampleFreq] * error(k) + Integral(k-1)

    [ki_Fixed32 / SampleFreq] = KiPerTick = mantissa >> shift, excludes shift N
        IntegralAccum holds Integral << N

    mantissa * [Error Max] < INT32_MAX: |mantissa| < 2^15
    [ki_Fixed32 / SampleFreq] * [Error Max] < [INT32_MAX / 2]: the increment adds to IntegralAccum without saturation

    SampleFreq = 10000:
    ki_Fixed32 = 32768 * 200 = 6,553,600 => (20,971, 5)

    SampleFreq = 20000:
    ki_Fixed32 = 65536 * 255 = 16,711,680 => (26,738, 5)
    ki_Fixed32 = 65536 => (26,843, 13)
    ki_Fixed32 = 256 => (26,843, 21)

    @param[in] ki_Fixed32 Q17.15, 32767 => 1.0F
*/
static void SetKi_Fixed32(PID_T * p_pid, uint32_t ki_Fixed32)
{
    assert(math_is_in_range(p_pid->Config.SampleFreq, 1, UINT16_MAX));
    p_pid->KiPerTick = fract16e_of_ratio(ki_Fixed32, p_pid->Config.SampleFreq);
    assert(fract16e_mul(p_pid->KiPerTick, UINT16_MAX) < INT32_MAX / 2); /* [Error Max] */
}


void PID_SetFreq(PID_T * p_pid, uint16_t sampleFreq)
{
    p_pid->Config.SampleFreq = sampleFreq;
    SetKi_Fixed32(p_pid, p_pid->Config.Ki);
    // SetKd_Fixed32(p_pid, p_pid->Config.Kd_Fixed32);
}

void PID_SetKp_Fixed32(PID_T * p_pid, int32_t kp_Fixed32)
{
    p_pid->Config.Kp = kp_Fixed32;
    SetKp_Fixed32(p_pid, kp_Fixed32);
}

void PID_SetKi_Fixed32(PID_T * p_pid, int32_t ki_Fixed32)
{
    p_pid->Config.Ki = ki_Fixed32;
    SetKi_Fixed32(p_pid, ki_Fixed32);
}

void PID_SetKd_Fixed32(PID_T * p_pid, int32_t kd_Fixed32)
{
    p_pid->Config.Kd = kd_Fixed32; /* todo */
}

/*!
    @param[in] kp_Fixed16 [0:INT16_MAX] Q9.7
    Unsigned for resolution
*/

void PID_SetKp_Fixed16(PID_T * p_pid, uint16_t kp_Fixed16) { PID_SetKp_Fixed32(p_pid, (uint32_t)kp_Fixed16 << 8); }

void PID_SetKi_Fixed16(PID_T * p_pid, uint16_t ki_Fixed16) { PID_SetKi_Fixed32(p_pid, (uint32_t)ki_Fixed16 << 8); }

void PID_SetKd_Fixed16(PID_T * p_pid, uint16_t kd_Fixed16) { PID_SetKd_Fixed32(p_pid, (uint32_t)kd_Fixed16 << 8); }

