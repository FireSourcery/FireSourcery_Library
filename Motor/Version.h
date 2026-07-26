#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Version.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>

/* Library Software Version */
#define MOTOR_LIBRARY_VERSION_OPT       0U
#define MOTOR_LIBRARY_VERSION_MAJOR     0U
#define MOTOR_LIBRARY_VERSION_MINOR     1U
#define MOTOR_LIBRARY_VERSION_FIX       0U
#define MOTOR_LIBRARY_VERSION           ((MOTOR_LIBRARY_VERSION_OPT << 24U) | (MOTOR_LIBRARY_VERSION_MAJOR << 16U) | (MOTOR_LIBRARY_VERSION_MINOR << 8U) | (MOTOR_LIBRARY_VERSION_FIX))

/* Precompile Flags */
// typedef union Version_Flags
// {
//     struct
//     {
//         uint16_t FieldWeakening : 1U;
//         uint16_t OpenLoop : 1U;
//         uint16_t ElectricalCalibration : 1U;
//     };
//     // uint32_t Value;
// }
// Version_Flags_T;


// static const Version_Flags_T MOTOR_LIBRARY_VERSION_FLAGS =
// {
//     .FieldWeakening = MOTOR_FOC_FIELD_WEAKENING_ENABLE,
//     .OpenLoop = MOTOR_FOC_OPEN_LOOP_ENABLE,
//     .ElectricalCalibration = MOTOR_FOC_ELECTRICAL_CALIBRATION_ENABLE,
// };
