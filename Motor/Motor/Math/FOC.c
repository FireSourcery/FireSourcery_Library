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
    @file   FOC.c
    @author FireSourcery

*/
/******************************************************************************/
#include "FOC.h"




/* Caller paas function pointer or band  */
// static inline void _FOC_ProcIFeedback(FOC_T * p_foc, ufract16_t vBus, angle16_t angleSpeed, int16_t idReq, int16_t iqReq, interval_t vqBand)
// {
//     ufract16_t vPhaseLimit = fract16_mul(vBus, FRACT16_1_DIV_2);
//     int32_t vd_ff = foc_vd_ff(fract16_mul(angleSpeed, p_foc->Electrical.Lq), p_foc->Iq); /* may exceeed INT16_MAX range */
//     int32_t vq_ff = foc_vq_ff(fract16_mul(angleSpeed, p_foc->Electrical.Ld), fract16_mul(angleSpeed, p_foc->Electrical.Psi), p_foc->Id);

//     /* Vd_PI ∈ [−Vlim − Vd_ff, Vlim​ − Vd_ff] */
//     PID_CaptureOutputLimits(&p_foc->PidId, fract16_sat(-vPhaseLimit - vd_ff), fract16_sat(vPhaseLimit - vd_ff)); /* vPhaseLimit < (INT16_MAX / 2) */
//     p_foc->Vd = math_clamp(PID_ProcPI(&p_foc->PidId, p_foc->Id, idReq) + vd_ff, -vPhaseLimit, vPhaseLimit);

//     /* Vq_PI ∈ [−Vcircle − Vq_ff, Vcircle − Vq_ff] */
//     ufract16_t vqCircleLimit = foc_vq_circle_limit(vPhaseLimit, p_foc->Vd);  /* VqCircleLimit < vPhaseLimit < (INT16_MAX / 2) */
//     interval_t vqLimit = interval_intersect(vqBand, interval_symmetric(0, vqCircleLimit));
//     PID_CaptureOutputLimits(&p_foc->PidIq, fract16_sat(vqLimit.low - vq_ff), fract16_sat(vqLimit.high - vq_ff));
//     p_foc->Vq = math_clamp(PID_ProcPI(&p_foc->PidIq, p_foc->Iq, iqReq) + vq_ff, vqLimit.low, vqLimit.high); /* lands back in vqBand */
// }

// static inline void _FOC_ProcIFeedback_Symmetric(FOC_T * p_foc, ufract16_t vBus, angle16_t angleSpeed, int16_t idReq, int16_t iqReq)
// {
//     ufract16_t vPhaseLimit = fract16_mul(vBus, FRACT16_1_DIV_2);
//     int32_t vd_ff = foc_vd_ff(fract16_mul(angleSpeed, p_foc->Electrical.Lq), p_foc->Iq); /* may exceeed INT16_MAX range */
//     int32_t vq_ff = foc_vq_ff(fract16_mul(angleSpeed, p_foc->Electrical.Ld), fract16_mul(angleSpeed, p_foc->Electrical.Psi), p_foc->Id);
//     /* Vd_PI ∈ [−Vlim − Vd_ff, Vlim​ − Vd_ff] */
//     PID_CaptureOutputLimits(&p_foc->PidId, fract16_sat(-vPhaseLimit - vd_ff), fract16_sat(vPhaseLimit - vd_ff)); /* vPhaseLimit < (INT16_MAX / 2) */
//     p_foc->Vd = math_clamp(PID_ProcPI(&p_foc->PidId, p_foc->Id, idReq) + vd_ff, -vPhaseLimit, vPhaseLimit);
//     /* Vq_PI ∈ [−Vcircle − Vq_ff, Vcircle − Vq_ff] */
//     ufract16_t vqCircleLimit = foc_vq_circle_limit(vPhaseLimit, p_foc->Vd);  /* VqCircleLimit < vPhaseLimit < (INT16_MAX / 2) */
//     PID_CaptureOutputLimits(&p_foc->PidIq, fract16_sat(-vqCircleLimit - vq_ff), fract16_sat(vqCircleLimit - vq_ff));
//     p_foc->Vq = math_clamp(PID_ProcPI(&p_foc->PidIq, p_foc->Iq, iqReq) + vq_ff, -vqCircleLimit, vqCircleLimit); /* lands back in vqBand */
// }


/* direction keyed */
// static inline void FOC_ProcIFeedback_AlignedDecouple(FOC_T * p_foc, ufract16_t vBus, sign_t direction, angle16_t angleSpeed, int16_t idReq, int16_t iqReq)
// {
//     ufract16_t vPhaseLimit = fract16_mul(vBus, FRACT16_1_DIV_2);
//     int32_t vd_ff = foc_vd_ff(fract16_mul(angleSpeed, p_foc->Electrical.Lq), p_foc->Iq); /* may exceeed INT16_MAX range */
//     int32_t vq_ff = foc_vq_ff(fract16_mul(angleSpeed, p_foc->Electrical.Ld), fract16_mul(angleSpeed, p_foc->Electrical.Psi), p_foc->Id);

//     /* Vd_PI ∈ [−Vlim − Vd_ff, Vlim​ − Vd_ff] */
//     PID_CaptureOutputLimits(&p_foc->PidId, fract16_sat(-vPhaseLimit - vd_ff), fract16_sat(vPhaseLimit - vd_ff)); /* vPhaseLimit < (INT16_MAX / 2) */
//     p_foc->Vd = math_clamp(PID_ProcPI(&p_foc->PidId, p_foc->Id, idReq) + vd_ff, -vPhaseLimit, vPhaseLimit);

//     /* Vq_PI ∈ [−Vcircle − Vq_ff, Vcircle − Vq_ff] */
//     interval_t vqBand = interval_of_sign(direction, _FOC_VqCircleLimit(p_foc, vPhaseLimit));  /* VqCircleLimit < vPhaseLimit < (INT16_MAX / 2) */
//     // vqBand = _FOC_VqBandOf(_FOC_VqCircleLimit(p_foc, vPhaseLimit), omega_psi, vBus / 8);
//     PID_CaptureOutputLimits(&p_foc->PidIq, fract16_sat(vqBand.low - vq_ff), fract16_sat(vqBand.high - vq_ff));
//     p_foc->Vq = math_clamp(PID_ProcPI(&p_foc->PidIq, p_foc->Iq, iqReq) + vq_ff, vqBand.low, vqBand.high); /* lands back in vqBand */
// }


// static inline void FOC_ProcIFeedback(FOC_T * p_foc, ufract16_t vBus, sign_t direction, int16_t idReq, int16_t iqReq)
// {
// #if defined(MOTOR_DECOUPLE_ENABLE)
//     FOC_ProcIFeedback_Decouple(p_foc, vBus, direction, p_foc->Electrical.Psi, idReq, iqReq);
// #else
//     FOC_ProcIFeedback_Base(p_foc, vBus, direction, idReq, iqReq);
// #endif
// }