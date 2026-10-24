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
    @file   PID.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "Math/math_general.h"
#include "Math/Fixed/fract16e.h"

typedef enum PID_Mode
{
    PID_MODE_PI,
    PID_MODE_PID,
}
PID_Mode_T;

/* Tuning Coefficients */
typedef struct PID_Config
{
    PID_Mode_T Mode;
    uint32_t SampleFreq;
    accum32_t Kp;
    accum32_t Ki;   /* [1/s] */
    accum32_t Kd;   /* [s] */
}
PID_Config_T;

typedef struct PID
{
    /*  */
    int32_t IntegralAccum; /* Shifted 15 */
    int32_t ErrorPrev;     /* setpoint - feedback, [-65535:65535] */
    int16_t OutputMin; /* -32768 Min */
    int16_t OutputMax; /* 32767 Max */
    int16_t Output;

    /*
        Run-time gains, derived from Config.
        Config holds the Q17.15 source, a readback through the derived form may lose precision
    */
    fract16e_t Kp;
    fract16e_t KiPerTick;       /* Ki / SampleFreq, into IntegralAccum */

    PID_Config_T Config; /* hold for runtime storage update */
}
PID_T;

static inline int16_t PID_GetOutput(const PID_T * p_pid) { return p_pid->Output; }
static inline int16_t PID_GetIntegral(const PID_T * p_pid) { return (p_pid->IntegralAccum >> 15); }

static inline bool PID_IsSaturated(const PID_T * p_pid) { return (p_pid->Output == p_pid->OutputMin) || (p_pid->Output == p_pid->OutputMax); }
// static inline bool PID_IsSaturated(const PID_T * p_pid) { return math_clamp(p_pid->Output, p_pid->OutputMin, p_pid->OutputMax) != p_pid->Output; }


/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
/* External storage */
static inline uint32_t _PID_GetSampleFreq(const PID_Config_T * p_config) { return p_config->SampleFreq; }
// static inline int32_t _PID_GetKp_Fixed32(const PID_Config_T * p_config) { return p_config->Kp_Fixed32; }
// static inline int32_t _PID_GetKi_Fixed32(const PID_Config_T * p_config) { return p_config->Ki_Fixed32; }
// static inline int32_t _PID_GetKd_Fixed32(const PID_Config_T * p_config) { return p_config->Kd_Fixed32; }


static inline int32_t _PID_GetKp_Fixed16(const PID_Config_T * p_config) { return p_config->Kp >> 8; }
static inline int32_t _PID_GetKi_Fixed16(const PID_Config_T * p_config) { return p_config->Ki >> 8; }
static inline int32_t _PID_GetKd_Fixed16(const PID_Config_T * p_config) { return p_config->Kd >> 8; }
static inline void _PID_SetKp_Fixed16(PID_Config_T * p_config, int32_t value) { p_config->Kp = value << 8; }
static inline void _PID_SetKi_Fixed16(PID_Config_T * p_config, int32_t value) { p_config->Ki = value << 8; }
static inline void _PID_SetKd_Fixed16(PID_Config_T * p_config, int32_t value) { p_config->Kd = value << 8; }

/*
*/
static inline uint32_t PID_GetSampleFreq(const PID_T * p_pid) { return p_pid->Config.SampleFreq; }

/* Q17.15 */
/* Accum32 */
static inline int32_t PID_GetKp_Fixed32(const PID_T * p_pid) { return p_pid->Config.Kp; }
static inline int32_t PID_GetKi_Fixed32(const PID_T * p_pid) { return p_pid->Config.Ki; }
static inline int32_t PID_GetKd_Fixed32(const PID_T * p_pid) { return p_pid->Config.Kd; }

/* Q9.7 */
/* Accum16 */
static inline uint16_t PID_GetKp_Fixed16(const PID_T * p_pid) { return PID_GetKp_Fixed32(p_pid) >> 8; }
static inline uint16_t PID_GetKi_Fixed16(const PID_T * p_pid) { return PID_GetKi_Fixed32(p_pid) >> 8; }
static inline uint16_t PID_GetKd_Fixed16(const PID_T * p_pid) { return PID_GetKd_Fixed32(p_pid) >> 8; }

/* Diagnostics */
static inline int32_t _PID_GetKp_Runtime(const PID_T * p_pid) { return accum32_of_fract16e(p_pid->Kp); }
static inline int32_t _PID_GetKi_Runtime(const PID_T * p_pid) { return fract16e_mul(p_pid->KiPerTick, p_pid->Config.SampleFreq); }

static inline int16_t _PID_GetKp_Fixed16_Runtime(const PID_T * p_pid) { return _PID_GetKp_Runtime(p_pid) >> 8; }
static inline int16_t _PID_GetKi_Fixed16_Runtime(const PID_T * p_pid) { return _PID_GetKi_Runtime(p_pid) >> 8; }

/******************************************************************************/
/*!
*/
/******************************************************************************/
extern void PID_InitFrom(PID_T * p_pid, const PID_Config_T * p_config);
// extern void PID_Init(PID_T * p_pid);
extern void PID_CaptureOutputLimits(PID_T * p_pid, int16_t min, int16_t max);
extern int16_t PID_ProcPI(PID_T * p_pid, int16_t feedback, int16_t setpoint);
extern void PID_Reset(PID_T * p_pid);
extern void _PID_SetOutputState(PID_T * p_pid, int16_t output);
extern void PID_SetOutputState(PID_T * p_pid, int16_t output);
extern void PID_SetOutputLimits(PID_T * p_pid, int16_t min, int16_t max);
// extern void PID_SetFreq(PID_T * p_pid, uint16_t sampleFreq);
extern void PID_SetKp_Fixed32(PID_T * p_pid, int32_t kp_Fixed32);
extern void PID_SetKi_Fixed32(PID_T * p_pid, int32_t ki_Fixed32);
extern void PID_SetKd_Fixed32(PID_T * p_pid, int32_t kd_Fixed32);
extern void PID_SetKp_Fixed16(PID_T * p_pid, uint16_t kp_Fixed16);
extern void PID_SetKi_Fixed16(PID_T * p_pid, uint16_t ki_Fixed16);
extern void PID_SetKd_Fixed16(PID_T * p_pid, uint16_t kd_Fixed16);



// #if      defined(PID_K_INT)
// #elif    defined(PID_K_FLOAT)
// #else
//     #define PID_K_INT
// #endif
