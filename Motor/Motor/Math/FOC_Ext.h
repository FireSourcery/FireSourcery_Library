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
    @file   FOC_Ext.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
// #include "FOC.h"

// // Add motor health monitoring
// typedef struct FOC_Diagnostics
// {
//     uint32_t overCurrentCount;
//     uint32_t overVoltageCount;
//     uint32_t commutationErrorCount;
//     uint32_t sensorErrorCount;
//     ufract16_t maxCurrent;
//     ufract16_t avgPower;
//     fract16_t temperatureEst;
// }
// FOC_Diagnostics_T;

// static inline void FOC_UpdateDiagnostics(FOC_T * p_foc, FOC_Diagnostics_T * p_diag)
// {
//     ufract16_t iMag = FOC_GetIMagnitude(p_foc);
//     ufract16_t vMag = FOC_GetVMagnitude(p_foc);

//     /* Track maximum current */
//     if (iMag > p_diag->maxCurrent)
//     {
//         p_diag->maxCurrent = iMag;
//     }

//     /* Count fault conditions */
//     if (iMag > MOTOR_CURRENT_LIMIT)
//     {
//         p_diag->overCurrentCount++;
//     }

//     if (vMag > MOTOR_VOLTAGE_LIMIT)
//     {
//         p_diag->overVoltageCount++;
//     }

//     /* Estimate temperature from I²R losses */
//     accum32_t power = FOC_GetTotalPower(p_foc);
//     p_diag->avgPower = (p_diag->avgPower * 15 + power) >> 4;  /* IIR filter */
// }

// // Add harmonic analysis for motor condition monitoring
// static inline void FOC_AnalyzeCurrentHarmonics(const FOC_T * p_foc, uint16_t * p_thd)
// {
//     /* Total Harmonic Distortion estimation */
//     ufract16_t fundamental = FOC_GetIMagnitude(p_foc);

//     /* Estimate harmonics from current ripple */
//     fract16_t ripple_a = p_foc->Ia - (p_foc->Ialpha * FRACT16_COS_0 + p_foc->Ibeta * FRACT16_SIN_0);
//     /* Add similar calculations for other phases */

//     if (fundamental > 0)
//     {
//         *p_thd = (abs(ripple_a) << 15) / fundamental;
//     }
// }