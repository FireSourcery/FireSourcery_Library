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
    @file   PID.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

#include "Math/math_general.h"
#include "Math/Fixed/fixed.h"

typedef enum PID_Mode
{
    PID_MODE_PI,
    PID_MODE_PID,
}
PID_Mode_T;

typedef struct PID_Config
{
    PID_Mode_T Mode;
    uint32_t SampleFreq;
    int32_t Kp_Fixed32;
    int32_t Ki_Fixed32;
    int32_t Kd_Fixed32;
}
PID_Config_T;

typedef const struct PID_Const
{
    const PID_Config_T * const P_CONFIG;
}
PID_Const_T;

typedef struct PID
{
    const PID_Const_T CONST;
    PID_Config_T Config;

    /*
        Run-time coefficients. Store as Fixed32.
        Reapeating derived getter/setter may result in loss of precision
    */
    int16_t PropGain;
    int8_t PropGainShift;
    int16_t IntegralGain;
    int8_t IntegralGainShift;

    int32_t Integral32; /* Shifted 16 */
    int32_t ErrorPrev;
    int16_t OutputMin; /* -32768 Min */
    int16_t OutputMax; /* 32767 Max */
    int16_t Output; /* Polling use */
}
PID_T;

#define PID_INIT(p_Config) { .CONST = { .P_CONFIG = p_Config, } }

static inline int16_t PID_GetOutput(PID_T * p_pid) { return p_pid->Output; }

static inline uint32_t PID_GetSampleFreq(const PID_T * p_pid) { return p_pid->Config.SampleFreq; }
static inline int32_t PID_GetKp_Fixed32(const PID_T * p_pid) { return p_pid->Config.Kp_Fixed32; }
static inline int32_t PID_GetKi_Fixed32(const PID_T * p_pid) { return p_pid->Config.Ki_Fixed32; }
static inline int32_t PID_GetKd_Fixed32(const PID_T * p_pid) { return p_pid->Config.Kd_Fixed32; }
static inline int16_t PID_GetKp_Fixed16(const PID_T * p_pid) { return PID_GetKp_Fixed32(p_pid) >> 8; }
static inline int16_t PID_GetKi_Fixed16(const PID_T * p_pid) { return PID_GetKi_Fixed32(p_pid) >> 8; }
static inline int16_t PID_GetKd_Fixed16(const PID_T * p_pid) { return PID_GetKd_Fixed32(p_pid) >> 8; }


/******************************************************************************/
/*!
*/
/******************************************************************************/
extern void PID_Init(PID_T * p_pid);
extern int32_t PID_ProcPI(PID_T * p_pid, int32_t feedback, int32_t setpoint);
extern void PID_Reset(PID_T * p_pid);
extern void PID_SetIntegral(PID_T * p_pid, int32_t integral);
extern void PID_SetOutputState(PID_T * p_pid, int32_t integral);
extern void PID_SetOutputLimits(PID_T * p_pid, int32_t min, int32_t max);
extern void PID_SetFreq(PID_T * p_pid, uint16_t sampleFreq);
extern void PID_SetKp_Fixed32(PID_T * p_pid, uint32_t kp_Fixed32);
extern void PID_SetKi_Fixed32(PID_T * p_pid, uint32_t ki_Fixed32);
extern void PID_SetKd_Fixed32(PID_T * p_pid, uint32_t kd_Fixed32);
extern void PID_SetKp_Fixed16(PID_T * p_pid, uint16_t kp_Fixed16);
extern void PID_SetKi_Fixed16(PID_T * p_pid, uint16_t ki_Fixed16);
extern void PID_SetKd_Fixed16(PID_T * p_pid, uint16_t kd_Fixed16);

#endif /* PID_H */