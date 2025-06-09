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
    @file   FOC_Feedback.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "FOC.h"
#include "Math/PID/PID.h"
#include "Math/Ramp/Ramp.h"

typedef struct FOC_Feedback
{
    FOC_T Foc;
    Ramp_T TorqueRamp;
    PID_T PidIq;
    PID_T PidId;
}
FOC_Feedback_T;

// static inline void FOC_ProcIFeedback(FOC_T * p_foc, PID_T * p_pidId, PID_T * p_pidIq)
// {
//     p_foc->Vq = PID_ProcPI(p_pidIq, p_foc->Iq, p_foc->ReqQ);
//     p_foc->Vd = PID_ProcPI(p_pidId, p_foc->Id, p_foc->ReqD);
// }

// static inline void Motor_FOC_WriteDuty(const Phase_T * p_phase, const FOC_T * p_foc)
// {
//     Phase_WriteDuty_Fract16(p_phase, FOC_GetDutyA(p_foc), FOC_GetDutyB(p_foc), FOC_GetDutyC(p_foc));
// }
