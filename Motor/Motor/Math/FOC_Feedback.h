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
#include "../Phase/Phase.h"
#include "Math/PID/PID.h"
#include "Math/Ramp/Ramp.h"



/* From Iabc to Idq */
// static inline bool _FOC_CaptureIabc(FOC_T * p_foc, Phase_Data_T * p_phaseData)
// {
//     if (p_phaseData->Flags.Bits == PHASE_ID_ABC)  /* alternatively use batch callback */
//     {
//         FOC_ProcClarkePark(p_foc, p_phaseData->Values.A, p_phaseData->Values.B, p_phaseData->Values.C);
//         p_phaseData->Flags.Bits = PHASE_ID_0; /* Clear capture flag after processing */
//         return true;
//     }
//     else
//     {
//         return false; /* No new data captured */
//     }
// }
// /* FeedbackState */
// static inline void  _FOC_CaptureFeedback(FOC_T * p_foc, Phase_Data_T * p_phaseData, angle16_t angle)
// {
//     FOC_SetTheta(p_foc, angle);
//     FOC_ProcClarkePark(p_foc, p_phaseData->Values.A, p_phaseData->Values.B, p_phaseData->Values.C);
// }

// /* Angle and Phase data set */
// /* limit req or Torque Ramp hold output limtis */
// static inline void  _FOC_TorqueControl(FOC_T * p_foc, PID_T * p_pidId, PID_T * p_pidIq, uint32_t vbus, Ramp_T * p_torqueRamp, fract16_t torqueReq)
// {
//     FOC_SetVd(p_foc, PID_ProcPI(p_pidId, FOC_Id(p_foc), 0));
//     FOC_SetVq(p_foc, PID_ProcPI(p_pidIq, FOC_Iq(p_foc), Ramp_ProcNextOf(p_torqueRamp, torqueReq)));
//     /* the combine output state can still grow outside of circle limit. limit after proc may still have windup */ /* propagate if limited.  */
//     if (FOC_ProcVectorLimit(p_foc, vbus) == true)
//     {
//         PID_SetOutputLimits(p_pidIq, (FOC_Vq(p_foc) < 0) * FOC_Vq(p_foc),  (FOC_Vq(p_foc) > 0) * FOC_Vq(p_foc));
//     }
// }

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

