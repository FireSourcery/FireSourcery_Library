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
    @version V0
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

void PID_Init(PID_T * p_pid)
{
    if (p_pid->CONST.P_CONFIG != NULL) { memcpy(&p_pid->Config, p_pid->CONST.P_CONFIG, sizeof(PID_Config_T)); }
    ResetGains(p_pid);
    PID_SetOutputLimits(p_pid, INT16_MIN, INT16_MAX);
    PID_Reset(p_pid);

    // PID_Init(p_pid, p_pid->CONST.P_CONFIG);
}

// void PID_Init(PID_T * p_pid, PID_Config_T * p_config)
// {
//     memcpy(&p_pid->Config, p_config, sizeof(PID_Config_T));
//     ResetGains(p_pid);
//     PID_SetOutputLimits(p_pid, INT16_MIN, INT16_MAX);
//     PID_Reset(p_pid);
// }

void PID_SetConfig(PID_T * p_pid, PID_Config_T * p_config)
{
    memcpy(&p_pid->Config, p_config, sizeof(PID_Config_T));
    ResetGains(p_pid);
}

static inline int16_t GetIntegral(const PID_T * p_pid) { return (p_pid->IntegralAccum >> 15); }
static inline void SetIntegral(PID_T * p_pid, int16_t integral) { p_pid->IntegralAccum = ((int32_t)integral << 15); }

/*!
    Conventional parallel PID calculation
    @return control = (Kp * error) + (Ki * error * SampleTime + IntegralPrev) + (Kd * (error - ErrorPrev) / SampleTime)

    integral [-32767:32767]
*/
static inline int32_t CalcPI(PID_T * p_pid, int32_t error)
{
    int32_t proportional, integral, integralAccum, integralMin, integralMax, output;

    // error = math_clamp(error, INT16_MIN, INT16_MAX);

    proportional = (p_pid->PropGain * error) >> p_pid->PropGainShift; /* Includes 15 shift */

    /* Dynamic Clamp */
    integralMin = math_min(p_pid->OutputMin - proportional, 0);
    integralMax = math_max(p_pid->OutputMax - proportional, 0);

    /*
        Store as Integral ("integrate" then sum). Allows compute time gain adjustment.
            Alternatively, store as Riemann Sum. (Ki * ErrorSum * SampleTime)
        Forward rectangular approximation.
    */

    // p_pid->IntegralAccum += (p_pid->IntegralGain * error) >> p_pid->IntegralGainShift; /* Excludes 15 shift */
    // p_pid->IntegralAccum = math_clamp(p_pid->IntegralAccum, integralMin << 15, integralMax << 15);
    // integral = p_pid->IntegralAccum >> 15;

    // assert(p_pid->IntegralAccum < INT32_MAX / 2);
    // assert(((p_pid->IntegralGain * error) >> p_pid->IntegralGainShift) < INT32_MAX / 2);

    integralAccum = p_pid->IntegralAccum + ((p_pid->IntegralGain * error) >> p_pid->IntegralGainShift); /* Excludes 15 shift */
    integral = math_clamp(integralAccum >> 15, integralMin, integralMax);
    p_pid->IntegralAccum = (integral == (integralAccum >> 15)) ? integralAccum : (integral << 15);

    p_pid->ErrorPrev = error;

    return proportional + integral;
}

/*!
    @param[in] setpoint [-32768:32767] with over saturation
    @param[in] feedback [-32768:32767] with over saturation
*/
int16_t PID_ProcPI(PID_T * p_pid, int32_t feedback, int32_t setpoint)
{
    p_pid->Output = math_clamp(CalcPI(p_pid, setpoint - feedback), p_pid->OutputMin, p_pid->OutputMax);
    return p_pid->Output;
}

int16_t _PID_ProcPI(PID_T * p_pid, int32_t feedback, int32_t setpoint)
{
    p_pid->Output = CalcPI(p_pid, setpoint - feedback);
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

/* allow int32_t input to clamp in 1 step */
void PID_SetOutputState(PID_T * p_pid, int32_t state)
{
    SetIntegral(p_pid, math_clamp(state, p_pid->OutputMin, p_pid->OutputMax));
    p_pid->Output = GetIntegral(p_pid); /* passed value after clamp */
    p_pid->ErrorPrev = 0;
}

/*!
    @param[in] [-32767:32767]
*/
void PID_SetOutputLimits(PID_T * p_pid, int16_t min, int16_t max)
{
    if (max > min)
    {
        p_pid->OutputMin = min;
        p_pid->OutputMax = max;
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
    [Error Max] = 32,767 * 2 => [Gain Max] = 32,767
*/

/*!
    Proportional(k) = Kp * error(k)
                    = [kp_Fixed32 >> N] * error(k)

    [kp_Fixed32 >> N] = [PropGain >> PropGainShift], includes shift N
    PropGain = [kp_Fixed32 << PropGainShift >> N]
    [kp_Fixed32 << PropGainShift >> N] * [Error Max] < INT32_MAX / 2
    PropGainShift <= log2(INT32_MAX / 2 / [Error Max] << N / kp_Fixed32)

    as Q17.15
    kp_Fixed32 = 32767, 1.0F => RShift 0, Gain = (32,767, 15 - 0)
    kp_Fixed32 = 65536, 2.0F => RShift 2, Gain = (16,384, 15 - 2)
    kp_Fixed32 = [511.0F = 511 < 15] => RShift 9, Gain = (~32,767, 15 - 9)

    @param[in] kp_Fixed32 Q17.15, 32767 => 1.0F
*/
static void SetKp_Fixed32(PID_T * p_pid, uint32_t kp_Fixed32)
{
    /*
        As additional shift of fract16
        (p_pid->PropGain * error) >> (15 - p_pid->PropGainShift)
    */
    // p_pid->PropGainShift = math_limit_lower(fixed_log2(kp_Fixed32) + 1 - 15, 0);
    // p_pid->PropGain = kp_Fixed32 >> p_pid->PropGainShift;

    /*
        (p_pid->PropGain * error) >> (p_pid->PropGainShift)
    */
    p_pid->PropGainShift = math_limit_upper(fixed_log2(INT32_MAX / 2 / kp_Fixed32), 15); /* discard negative shift */
    p_pid->PropGain = kp_Fixed32 >> (15 - p_pid->PropGainShift);

    assert(kp_Fixed32 >> (15 - p_pid->PropGainShift) < INT16_MAX);
}

/*!
    Integral(k) = Ki * error(k) / SampleFreq + Integral(k-1)
                = [ki_Fixed32 >> N / SampleFreq] * error(k) + Integral(k-1)

    [ki_Fixed32 / SampleFreq] = [IntegralGain >> IntegralGainShift], excludes shift N
    IntegralGain = [ki_Fixed32 / SampleFreq << IntegralGainShift]
    [ki_Fixed32 / SampleFreq << IntegralGainShift] * [Error Max] < [INT32_MAX / 2],
    IntegralGainShift <= log2([INT32_MAX / 2] / [Error Max] * SampleFreq / ki_Fixed32)

    SampleFreq = 10000:
    ki_Fixed32 = 32768 * 200 = 6,553,600 => (20,971, 5)

    SampleFreq = 20000:
    ki_Fixed32 = 65536 * 255 = 16,711,680 => (26,843, 4)
    ki_Fixed32 = 256 => Gain = (26,843, 20)
    ki_Fixed32 = 65536 => Gain = (26,843, 12)

    @param[in] ki_Fixed32 Q17.15, 32767 => 1.0F
*/
static void SetKi_Fixed32(PID_T * p_pid, uint32_t ki_Fixed32)
{
    /*
        As additional shift of fract16
        (p_pid->PropGain * error) >> (15 + p_pid->PropGainShift)

        error 32767 => INT32_MAX / 2
    */
    // p_pid->IntegralGainShift = fixed_log2((INT32_MAX / 2 / 65536) * p_pid->Config.SampleFreq / ki_Fixed32);
    // p_pid->IntegralGain = (ki_Fixed32 << p_pid->IntegralGainShift) / p_pid->Config.SampleFreq;

    p_pid->IntegralGainShift = math_limit_lower(15 - (fixed_log2(ki_Fixed32 / p_pid->Config.SampleFreq) + 1), 0);
    p_pid->IntegralGain = (ki_Fixed32 << p_pid->IntegralGainShift) / p_pid->Config.SampleFreq;

    assert(((ki_Fixed32 << p_pid->IntegralGainShift) / p_pid->Config.SampleFreq) < INT16_MAX);
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
*/

void PID_SetKp_Fixed16(PID_T * p_pid, uint16_t kp_Fixed16) { PID_SetKp_Fixed32(p_pid, (uint32_t)kp_Fixed16 << 8); }

void PID_SetKi_Fixed16(PID_T * p_pid, uint16_t ki_Fixed16) { PID_SetKi_Fixed32(p_pid, (uint32_t)ki_Fixed16 << 8); }

void PID_SetKd_Fixed16(PID_T * p_pid, uint16_t kd_Fixed16) { PID_SetKd_Fixed32(p_pid, (uint32_t)kd_Fixed16 << 8); }

