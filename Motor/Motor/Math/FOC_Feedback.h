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

// typedef struct FOC_Feedback
// {
//     FOC_T Foc;
//     Ramp_T TorqueRamp;
//     PID_T PidIq;
//     PID_T PidId;
// }
// FOC_Feedback_T;

// static inline void FOC_Feedback_Proc(FOC_T * p_foc, PID_T * PidId, PID_T * PidIq)
// {
//     p_foc->Vd = PID_ProcPI(PidId, p_foc->Id, p_foc->ReqD);
//     p_foc->Vq = PID_ProcPI(PidIq, p_foc->Iq, p_foc->ReqQ);
// }

// static inline void FOC_Feedback_Proc(FOC_T * p_foc, PID_T * PidId, PID_T * PidIq, fract16_t idReq, fract16_t iqReq)
// {
//     p_foc->Vd = PID_ProcPI(PidId, p_foc->Id, idReq);
//     p_foc->Vq = PID_ProcPI(PidIq, p_foc->Iq, iqReq);
// }

// static inline void FOC_Feedback_ProcId(FOC_T * p_foc, PID_T * p_pidId, fract16_t idReq)
// {
//     p_foc->Vd = PID_ProcPI(p_pidId, p_foc->Id, idReq);
// }

// void FOC_MatchFeedbackState(FOC_T * p_foc, PID_T * PidId, PID_T * PidIq)
// {
//     // int32_t vq = GetVSpeed_Fract16(p_motor) / 2; // match without ad sampling
//     int16_t vq = FOC_GetVMagnitude(&p_foc) * p_foc->Direction;
//     int16_t qReq;

        // PID_SetOutputState(&p_foc->PidIq, vq);
        // PID_SetOutputState(&p_foc->PidId, 0);
        // qReq = FOC_GetIq(&p_foc);
        // MatchSpeedTorqueState(p_foc, qReq);