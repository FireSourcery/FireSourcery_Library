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
#include "FOC.h"
#include "motor_electrical_math.h"
#include "../Phase/Phase_VOut.h"
#include "Math/PID/PID.h"
#include "Math/Ramp/Ramp.h"

/******************************************************************************/
/*!
    Collaborator Pattern
*/
/******************************************************************************/
/* From Iabc to Idq */
/* optional wrap AB selection */
__attribute__((noinline)) static bool FOC_CaptureIData(FOC_T * p_foc, volatile Phase_Data_T * p_iabc)
{
    if (p_iabc->Flags.Bits == PHASE_ID_ABC)  /* alternatively use batch callback */
    {
        FOC_ProcClarkePark(p_foc, p_iabc->Values.A, p_iabc->Values.B, p_iabc->Values.C);
        p_iabc->Flags.Bits = PHASE_ID_0; /* Clear capture flag after processing */
        return true;
    }
    return false; /* No new data captured */
}

// caller handle state marker
static inline void _FOC_CaptureIabc(FOC_T * p_foc, const volatile Phase_Triplet_T * p_iabc)
{
    FOC_ProcClarkePark(p_foc, p_iabc->A, p_iabc->B, p_iabc->C);
}

/* As [FOC_CaptureIData]: true on a fresh set, then clears its flag */
static inline bool FOC_CaptureVBemfData(FOC_T * p_foc, volatile Phase_Data_T * p_input)
{
    if (p_input->Flags.Bits == PHASE_ID_ABC)
    {
        FOC_ProcVBemfClarkePark(p_foc, p_input->Values.A, p_input->Values.B, p_input->Values.C);
        p_input->Flags.Bits = PHASE_ID_0;
        return true;
    }
    return false;
}


static inline Phase_Triplet_T FOC_Vabc(const FOC_T * p_foc) { return (Phase_Triplet_T) { .A = p_foc->Va, .B = p_foc->Vb, .C = p_foc->Vc, }; }

static inline Phase_Triplet_T FOC_GetVOut(const FOC_T * p_foc)
{
    struct foc_abc vOut = foc_inv_clarke_park(p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
    return (Phase_Triplet_T) { .A = fract16_sat(vOut.a), .B = fract16_sat(vOut.b), .C = fract16_sat(vOut.c), };
}

// static inline Phase_Triplet_T FOC_ProcIFeedbackVOut(FOC_T * p_foc, ufract16_t vBus, angle16_t angle, int16_t dReq, int16_t qReq)
// {
//     FOC_ProcIFeedback(p_foc, vBus, dReq, qReq);
//     return FOC_GetVOut(p_foc);
// }


#include "Motor/Motor/Model/EmfObserver.h"
/*
    One control tick, after the FOC proc
*/
// static inline void FOC_Sensorless_Proc(const FOC_T * p_foc, EmfObserver_T * p_observer)
// {
//     EmfObserver_AlphaBeta_Proc(p_observer, p_foc->Ialpha, p_foc->Ibeta, p_foc->Valpha, p_foc->Vbeta);
// }

/*
    Operating point. One sample of the dq state. Every condition below reads the sample, not the live FOC fields.
*/
// typedef struct Motor_OperatingPoint
// {
//     accum32_t Speed;    /* ω, speed pu, Ccw positive */
//     fract16_t Id, Iq;
//     fract16_t Vd, Vq;
// }
// Motor_OperatingPoint_T;



/******************************************************************************/
/*!

*/
/******************************************************************************/
/* Freq_Base [angle16/s] */
#define FOC_ELECTRICAL_FROM_SI(Freq_Base, V_Base, I_Base, Ld_uH, Lq_uH, Rs_mOhm, Psi_uWb) (FOC_Electrical_T) \
{ \
    .Ld  = ACCUM32(MOTOR_L_PU(Freq_Base, V_Base, I_Base, (Ld_uH) * 1.0E-6F)), \
    .Lq  = ACCUM32(MOTOR_L_PU(Freq_Base, V_Base, I_Base, (Lq_uH) * 1.0E-6F)), \
    .Rs  = ACCUM32(MOTOR_R_PU_OF_MOHM(V_Base, I_Base, Rs_mOhm)), \
    .Psi = ACCUM32(MOTOR_PSI_PU(Freq_Base, V_Base, (Psi_uWb) * 1.0E-6F)), \
}



/******************************************************************************/
/*!

*/
/******************************************************************************/

/* For interior PM motors, add MTPA calculation:
Id_opt = -λ_pm/(2*Ld) + sqrt((λ_pm/(2*Ld))² + Iq²)
*/
// Id_opt = -λ_pm/(2·Ld) + sqrt((λ_pm/(2·Ld))² + Iq²)
// Id_opt = -λ_pm / (2·(Lq - Ld)) + sqrt((λ_pm / (2·(Lq - Ld)))² + Iq²)
// // Add maximum torque per amp (MTPA) calculation
// static inline void FOC_CalcMTPA(FOC_T * p_foc, fract16_t iqReq)
// {
//     /* Simplified MTPA for surface-mounted PM motors */
//     p_foc->ReqD = 0;  /* Id = 0 for SMPM motors */
//     p_foc->ReqQ = iqReq;
// }

// Add six-step commutation for high-speed operation
// static inline void FOC_ProcSixStep(FOC_T * p_foc, angle16_t angle, fract16_t magnitude)
// {
//     Motor_SectorId_T sector = FOC_GetSectorId(angle);

//     /* Six-step commutation pattern */
//     switch (sector)
//     {
//         case MOTOR_SECTOR_ID_1:  /* A+, B- */
//             p_foc->Va = magnitude;  p_foc->Vb = -magnitude; p_foc->Vc = 0; break;
//         case MOTOR_SECTOR_ID_2:  /* A+, C- */
//             p_foc->Va = magnitude;  p_foc->Vb = 0; p_foc->Vc = -magnitude; break;
//         case MOTOR_SECTOR_ID_3:  /* B+, C- */
//             p_foc->Va = 0; p_foc->Vb = magnitude;  p_foc->Vc = -magnitude; break;
//         case MOTOR_SECTOR_ID_4:  /* B+, A- */
//             p_foc->Va = -magnitude; p_foc->Vb = magnitude;  p_foc->Vc = 0; break;
//         case MOTOR_SECTOR_ID_5:  /* C+, A- */
//             p_foc->Va = -magnitude; p_foc->Vb = 0; p_foc->Vc = magnitude; break;
//         case MOTOR_SECTOR_ID_6:  /* C+, B- */
//             p_foc->Va = 0; p_foc->Vb = -magnitude; p_foc->Vc = magnitude; break;
//     }
// }

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


