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

static void ResetGains(PID_T * p_pid)
{
    PID_SetKp_Fixed32(p_pid, p_pid->Config.Kp_Fixed32);
    PID_SetKi_Fixed32(p_pid, p_pid->Config.Ki_Fixed32);
    // PID_SetKd_Fixed32(p_pid, p_pid->Config.Kd_Fixed32);
}

void PID_Init(PID_T * p_pid)
{
    if (p_pid->CONST.P_CONFIG != NULL) { memcpy(&p_pid->Config, p_pid->CONST.P_CONFIG, sizeof(PID_Config_T)); }
    ResetGains(p_pid);
    PID_SetOutputLimits(p_pid, INT16_MIN, INT16_MAX);
    PID_Reset(p_pid);
}

// void PID_Init(PID_T * p_pid, PID_Config_T * p_config)
// {
//     memcpy(&p_pid->Config, p_config, sizeof(PID_Config_T));
//     ResetGains(p_pid);
//     PID_SetOutputLimits(p_pid, INT16_MIN, INT16_MAX);
//     PID_Reset(p_pid);
// }

static inline int16_t GetIntegral(PID_T * p_pid) { return (p_pid->Integral32 >> 16); }
static inline void SetIntegral(PID_T * p_pid, int16_t integral) { p_pid->Integral32 = ((int32_t)integral << 16); }

/*!
    Conventional parallel PID calculation
    @return control = (Kp * error) + (Ki * error * SampleTime + IntegralPrev) + (Kd * (error - ErrorPrev) / SampleTime)
*/
static inline int32_t CalcPI(PID_T * p_pid, int32_t error)
{
    int32_t proportional, integral32, integral32Part, integral, integralMin, integralMax;

    proportional = (p_pid->Config.PropGain * error) >> p_pid->Config.PropGainShift; /* Inclusive of 16 shift */

    /*
        Store as Integral ("integrate" then sum). Allows compute time gain adjustment.
            Alternatively, store as Riemann Sum. (Ki * ErrorSum * SampleTime)
    */
    /* Forward rectangular approximation. */
    integral32Part = (p_pid->Config.IntegralGain * error) >> p_pid->Config.IntegralGainShift; /* Exclusive of 16 shift */
    integral32 = math_add_sat(p_pid->Integral32, integral32Part);
    integral = integral32 >> 16;

    /* Dynamic Clamp */
    integralMin = math_min(p_pid->OutputMin - proportional, 0);
    integralMax = math_max(p_pid->OutputMax - proportional, 0);

    /*
        Clamp integral to prevent windup.
        Determine integral storage, Integral32, using [integral]
        if integral and error are in opposite directions, some synchronization issue has occurred, reset stored integral.
    */
    if      (integral > integralMax) { integral = integralMax; if (error < 0) { SetIntegral(p_pid, integralMax); } }
    else if (integral < integralMin) { integral = integralMin; if (error > 0) { SetIntegral(p_pid, integralMin); } }
    else                             { p_pid->Integral32 = integral32; }

    return proportional + integral;
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

/*!
    -65535 < (setpoint - feedback) < 65535
    @param[in] setpoint [-65536:65535]
    @param[in] feedback [-65536:65535]
*/
int32_t PID_ProcPI(PID_T * p_pid, int32_t feedback, int32_t setpoint)
{
    p_pid->Output = math_clamp(CalcPI(p_pid, setpoint - feedback), p_pid->OutputMin, p_pid->OutputMax);
    return p_pid->Output;
}

/*
    Compute-Time Variables Set
*/
void PID_Reset(PID_T * p_pid)
{
    p_pid->Integral32 = 0;
    p_pid->ErrorPrev = 0;
}

void PID_SetIntegral(PID_T * p_pid, int32_t integral)
{
    SetIntegral(p_pid, math_clamp(integral, p_pid->OutputMin, p_pid->OutputMax));
}

void PID_SetOutputState(PID_T * p_pid, int32_t integral)
{
    PID_SetIntegral(p_pid, integral);
    // p_pid->ErrorPrev = 0;
    p_pid->Output = GetIntegral(p_pid);
}

void PID_SetOutputLimits(PID_T * p_pid, int32_t min, int32_t max)
{
    if(max > min)
    {
        p_pid->OutputMin = min;
        p_pid->OutputMax = max;
        PID_SetIntegral(p_pid, GetIntegral(p_pid)); /* Reset integral with limits */
    }
}

/*
    Persistent Config Set
*/
void PID_SetFreq(PID_T * p_pid, uint16_t sampleFreq)
{
    int32_t ki;

    if (sampleFreq > 0U)
    {
        p_pid->Config.SampleFreq = sampleFreq; /* store fixed value, loop this may lose precision */
        ki = PID_GetKi_Fixed32(p_pid);
        PID_SetKi_Fixed32(p_pid, ki);
    }
}

/******************************************************************************/
/*
    Select Gain and Shift without overflow
    [Error Max] = (32,767 - (-32,768)) = 65535 => [Gain Max] = 32,767
*/
/******************************************************************************/

/*!
    Proportional(k) = Kp * error(k)
                    = kp_Fixed32 * error(k) >> 16

    kp_Fixed32 >> 16 = PropGain >> PropGainShift, inclusive of shift 16
    PropGain = kp_Fixed32 << PropGainShift >> 16
    [kp_Fixed32 << PropGainShift >> 16] * [Error Max] < INT32_MAX
    PropGainShift <= log2(INT32_MAX / 65536 * 65536 / kp_Fixed32)

    kp_Fixed32 = 65536 => RShift 2, Gain = (16,384, 14)
    kp_Fixed32 = 65535 => RShift 1, Gain = (32,767, 15)
    kp_Fixed32 = 32768 => RShift 1, Gain = (16,384, 15)
    kp_Fixed32 = 32767 => RShift 0, Gain = (32,767, 16)

    @param[in] kp_Fixed32 [0:INT32_MAX], Q16.16, 65536 => 1
*/
void PID_SetKp_Fixed32(PID_T * p_pid, uint32_t kp_Fixed32)
{
    p_pid->Config.PropGainShift = fixed_log2(INT32_MAX / kp_Fixed32);
    p_pid->Config.PropGain = kp_Fixed32 >> (16U - p_pid->Config.PropGainShift);
}

int32_t PID_GetKp_Fixed32(const PID_T * p_pid) { return (int32_t)p_pid->Config.PropGain << (16U - p_pid->Config.PropGainShift); }

/*!
    Integral(k) = Ki * error(k) / SampleFreq + Integral(k-1)
                = ki_Fixed32 * error(k) / SampleFreq >> 16 + Integral(k-1)

    ki_Fixed32 / SampleFreq = IntegralGain >> IntegralGainShift, exclusive of shift 16
    IntegralGain = ki_Fixed32 / SampleFreq << IntegralGainShift
    [ki_Fixed32 / SampleFreq << IntegralGainShift] * [Error Max] < INT32_MAX
    IntegralGainShift <= log2(INT32_MAX / 65536 * SampleFreq / ki_Fixed32)

    kp_Fixed32 = 65536, SampleFreq = 20000 => RShift 3, Gain = (26,843, 13)
*/
void PID_SetKi_Fixed32(PID_T * p_pid, uint32_t ki_Fixed32)
{
    p_pid->Config.IntegralGainShift = fixed_log2((INT32_MAX >> 16) * p_pid->Config.SampleFreq / ki_Fixed32);
    p_pid->Config.IntegralGain = (ki_Fixed32 << p_pid->Config.IntegralGainShift) / p_pid->Config.SampleFreq;
}

int32_t PID_GetKi_Fixed32(const PID_T * p_pid) { return (int32_t)p_pid->Config.IntegralGain * p_pid->Config.SampleFreq >> p_pid->Config.IntegralGainShift; }


/*!
    @param[in] kp_Fixed16 [0:INT16_MAX] Q8.8 256 => 1
*/
void PID_SetKp_Fixed16(PID_T * p_pid, uint16_t kp_Fixed16) { PID_SetKp_Fixed32(p_pid, (uint32_t)kp_Fixed16 << 8); }

int16_t PID_GetKp_Fixed16(const PID_T * p_pid) { return PID_GetKp_Fixed32(p_pid) >> 8; }

/*!
*/
void PID_SetKi_Fixed16(PID_T * p_pid, uint16_t ki_Fixed16) { PID_SetKi_Fixed32(p_pid, (uint32_t)ki_Fixed16 << 8); }

int16_t PID_GetKi_Fixed16(const PID_T * p_pid) { return PID_GetKi_Fixed32(p_pid) >> 8; }


/*!
    todo
*/
// void PID_SetKd_Fixed16(PID_T * p_pid, uint16_t kd_Fixed16) {}

// int16_t PID_GetKd_Fixed16(PID_T * p_pid) { return 0U; }