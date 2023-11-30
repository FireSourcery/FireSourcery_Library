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
#include "Math/Q/Q.h"

typedef enum PID_Mode
{
    PID_MODE_PI,
    PID_MODE_PID,
}
PID_Mode_T;

typedef struct PID_Params
{
    PID_Mode_T Mode;
    uint32_t SampleFreq;

    //temp
    int32_t Kp_Fixed32;
    int32_t Ki_Fixed32;
    int32_t Kd_Fixed32;

    int16_t PropGain;
    int8_t PropGainShift;
    int16_t IntegralGain;
    int8_t IntegralGainShift;
}
PID_Params_T;

typedef const struct PID_Config
{
    const PID_Params_T * const P_PARAMS;
}
PID_Config_T;

typedef struct PID
{
    const PID_Config_T CONFIG;
    PID_Params_T Params;
    int32_t Integral32; /* Q16.16, Shifted 16 */
    int32_t ErrorPrev;
    int16_t OutputMin; /* -32768 Min */
    int16_t OutputMax; /* 32767 Max */
    int16_t Output;
}
PID_T;

#define PID_INIT(p_Params) { .CONFIG = { .P_PARAMS = p_Params, } }

static inline int16_t PID_GetOutput(PID_T * p_pid)          { return p_pid->Output; }
// static inline int32_t PID_GetKpParam_Fixed32(PID_T * p_pid) { return p_pid->Params.Kp_Fixed32; }
// static inline int32_t PID_GetKiParam_Fixed32(PID_T * p_pid) { return p_pid->Params.Ki_Fixed32; }
// static inline int32_t PID_GetKdParam_Fixed32(PID_T * p_pid) { return p_pid->Params.Kd_Fixed32; }
static inline uint32_t PID_GetSampleFreq(PID_T * p_pid)     { return p_pid->Params.SampleFreq; }

/******************************************************************************/
/*!
*/
/******************************************************************************/
extern void PID_Init(PID_T * p_pid);
extern int32_t PID_ProcPI(PID_T *p_pid, int32_t setpoint, int32_t feedback);
extern void PID_Reset(PID_T * p_pid);
extern void PID_SetIntegral(PID_T * p_pid, int16_t integral);
extern void PID_SetOutputState(PID_T * p_pid, int16_t integral);
extern void PID_SetOutputLimits(PID_T * p_pid, int16_t min, int16_t max);
extern void PID_SetFreq(PID_T * p_pid, uint32_t sampleFreq);
extern void PID_SetKp_Fixed32(PID_T * p_pid, uint32_t kp_Fixed32);
extern void PID_SetKi_Fixed32(PID_T * p_pid, uint32_t ki_Fixed32);
extern void PID_SetKd_Fixed32(PID_T * p_pid, uint32_t kd_Fixed32);
extern int32_t PID_GetKp_Fixed32(PID_T * p_pid);
extern int32_t PID_GetKi_Fixed32(PID_T * p_pid);
extern int32_t PID_GetKd_Fixed32(PID_T * p_pid);
extern void PID_SetKp_Fixed16(PID_T * p_pid, uint16_t kp_Fixed16);
extern void PID_SetKi_Fixed16(PID_T * p_pid, uint16_t ki_Fixed16);
extern void PID_SetKd_Fixed16(PID_T * p_pid, uint16_t kd_Fixed16);
extern int16_t PID_GetKp_Fixed16(PID_T * p_pid);
extern int16_t PID_GetKi_Fixed16(PID_T * p_pid);
extern int16_t PID_GetKd_Fixed16(PID_T * p_pid);

#endif /* PID_H */