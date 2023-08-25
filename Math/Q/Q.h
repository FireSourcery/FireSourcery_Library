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
    @file   Q.h
    @author FireSourcery
    @brief     Q fixed point math operations
    @version V0
*/
/******************************************************************************/
#ifndef Q_MATH_H
#define Q_MATH_H

#include <stdint.h>

extern uint16_t q_sqrt(int32_t x);
extern uint8_t q_log2(uint32_t num);
extern uint8_t q_log2_ceiling(uint32_t num);
extern uint8_t q_log2_round(uint32_t num);
extern uint32_t q_pow2_round(uint32_t num);
extern uint8_t q_maxshift_signed(int32_t num);
extern uint8_t q_maxshift_unsigned(uint32_t positiveNum);

// typedef struct Q_Tag
// {
//     int16_t Factor;
//     int8_t Shift;
// }
// Q_T;
/*!
    Integral(k) = Ki * error(k) / SampleFreq + Integral(k-1)
                = ki_Fixed32 * error(k) / SampleFreq >> 16 + Integral(k-1)

    ki_Fixed32 / SampleFreq = IntegralGain >> IntegralGainShift, exclusive of shift 16
    IntegralGain = ki_Fixed32 / SampleFreq << IntegralGainShift
    [ki_Fixed32 / SampleFreq << IntegralGainShift] * [Error Max] < INT32_MAX
    IntegralGainShift <= log2(INT32_MAX / 65536 * SampleFreq / ki_Fixed32)

    kp_Fixed32 = 65536, SampleFreq = 20000 => RShift 3, Gain = (26,843, 13)
*/
// void PID_SetKi_Fixed32(PID_T * p_pid, uint32_t ki_Fixed32)
// {
//     p_pid->Params.IntegralGainShift = q_log2((uint32_t)32767U * p_pid->Params.SampleFreq / ki_Fixed32);
//     p_pid->Params.IntegralGain = (ki_Fixed32 << p_pid->Params.IntegralGainShift) / p_pid->Params.SampleFreq;
// }

// int32_t PID_GetKi_Fixed32(PID_T * p_pid) { return p_pid->Params.IntegralGain * p_pid->Params.SampleFreq >> p_pid->Params.IntegralGainShift; }


// /*!
//     @param[in] kp_Fixed16 [0:INT16_MAX] Q8.8 256 => 1
// */
// void PID_SetKp_Fixed16(PID_T * p_pid, uint16_t kp_Fixed16) { PID_SetKp_Fixed32(p_pid, (uint32_t)kp_Fixed16 << 8); }

// int32_t PID_GetKp_Fixed16(PID_T * p_pid) { return PID_GetKp_Fixed32(p_pid) >> 8; }

#endif
