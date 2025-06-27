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
    @file   .h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Math/PID/PID.h"
#include "Math/Ramp/Ramp.h"
#include "Math/Fixed/fract16.h"

#include <stdint.h>
#include <stdbool.h>

// typedef struct AngleSpeed
// {

// }
// AngleSpeed_T;

// typedef struct
// {
//     AngleSpeed_T AngleSpeed;
//     Ramp_T SpeedRamp;  /* Speed Ramp */
//     PID_T SpeedPid;  /* Speed PID */

//     // uint16_t SpeedLimitForward_Fract16; /* May over saturate */
//     // uint16_t SpeedLimitReverse_Fract16;
// }
// AngleSpeed_Feedback_T;


// static inline void AngleSpeed(AngleSpeed_T *    )

/* AngleSpeed OuterLoop */
// static inline void AngleSpeed_ProcFeedback(AngleSpeed_T *  , PID_T *  )