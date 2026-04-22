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
/******************************************************************************/
/*!
*/
/******************************************************************************/

/* with Scaled DutyAlpha, DutyBeta */
/*
    Normalized by a factor of sqrt(3)/2,
        alpha 1/sqrt(3) => a = .5
        alphaDuty 1.15 => a = 1.0
*/
// static inline void FOC_ProcInvParkInvClarkeSvpwm(FOC_T * p_foc)
// {
//     foc_inv_park_vector(&p_foc->Valpha, &p_foc->Vbeta, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
//     svpwm_midclamp(&p_foc->Va, &p_foc->Vb, &p_foc->Vc, _FOC_DutyOfV(p_foc, p_foc->Valpha), _FOC_DutyOfV(p_foc, p_foc->Vbeta));
// }


/* Store as max modulation, derive V_Fract16 by vbus */
/* V_Fract16 when Vd Vq are stored as normalized scalar values */
// #ifdef FOC_V_AS_SCALAR
// // static inline ufract16_t FOC_Va(const FOC_T * p_foc) { return _FOC_VOfVDuty(p_foc, p_foc->Va); }
// // static inline ufract16_t FOC_Vb(const FOC_T * p_foc) { return _FOC_VOfVDuty(p_foc, p_foc->Vb); }
// // static inline ufract16_t FOC_Vc(const FOC_T * p_foc) { return _FOC_VOfVDuty(p_foc, p_foc->Vc); }

// // static inline ufract16_t FOC_GetValpha(const FOC_T * p_foc) { return _FOC_VOfVDuty(p_foc, p_foc->Valpha); }
// // static inline ufract16_t FOC_GetVMagnitude(const FOC_T * p_foc) { return fract16_vector_magnitude(FOC_GetValpha(p_foc), FOC_GetVbeta(p_foc)); }

// // static inline void _FOC_SetVBemfA(FOC_T * p_foc, int32_t vBusInv_fract32, ufract16_t va) { p_foc->Va = _FOC_VDutyOfV_Inv32(vBusInv_fract32, va); }
// // static inline void _FOC_SetVBemfB(FOC_T * p_foc, int32_t vBusInv_fract32, ufract16_t vb) { p_foc->Vb = _FOC_VDutyOfV_Inv32(vBusInv_fract32, vb); }
// // static inline void _FOC_SetVBemfC(FOC_T * p_foc, int32_t vBusInv_fract32, ufract16_t vc) { p_foc->Vc = _FOC_VDutyOfV_Inv32(vBusInv_fract32, vc); }
// #else



/* Estimate efficiency based on I²R losses */
// static inline ufract16_t FOC_GetEfficiency(const FOC_T * p_foc, fract16_t rPhase)
// {
//     accum32_t activePower = FOC_GetActivePower(p_foc);
//     if (activePower <= 0) return 0;

//     ufract16_t iMag = FOC_GetIMagnitude(p_foc);
//     accum32_t resistiveLoss = fract16_mul(fract16_mul(iMag, iMag), rPhase) * 3;

//     return (activePower << 15) / (activePower + resistiveLoss);
// }


// Add field weakening support
// static inline fract16_t FOC_CalcFieldWeakeningId(const FOC_T * p_foc, fract16_t vBus, fract16_t speed)
// {
//     /* Calculate Id for field weakening at high speeds */
//     fract16_t vLimit = fract16_mul(vBus, FRACT16_1_DIV_SQRT3);
//     fract16_t vBemfEst = fract16_mul(speed, MOTOR_KE_CONSTANT);  /* Requires motor constant */

//     if (vBemfEst > vLimit)
//     {
//         /* Need field weakening */
//         return -(vBemfEst - vLimit);  /* Negative Id for field weakening */
//     }

//     return 0;  /* No field weakening needed */
// }

// // Add maximum torque per amp (MTPA) calculation
// static inline void FOC_CalcMTPA(FOC_T * p_foc, fract16_t iqReq)
// {
//     /* Simplified MTPA for surface-mounted PM motors */
//     p_foc->ReqD = 0;  /* Id = 0 for SMPM motors */
//     p_foc->ReqQ = iqReq;

//     /* For interior PM motors, add MTPA calculation:
//      * Id_opt = -λ_pm/(2*Ld) + sqrt((λ_pm/(2*Ld))² + Iq²)
//      */
// }

// // Add six-step commutation for high-speed operation
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



/* Direct DQ0 Transform - replaces Clarke + Park */
// static inline void FOC_ProcDQ0Transform(FOC_T * p_foc)
// {
//     foc_dq_transform(&p_foc->Id, &p_foc->Iq, p_foc->Ia, p_foc->Ib, p_foc->Ic, p_foc->Sine, p_foc->Cosine);
// }

// /* Direct DQ0 Transform for two-phase sensing (assumes Ic = -(Ia + Ib)) */
// static inline void FOC_ProcDQ0Transform_AB(FOC_T * p_foc)
// {
//     fract16_t ic = -(p_foc->Ia + p_foc->Ib);  /* Calculate third phase */
//     foc_dq_transform(&p_foc->Id, &p_foc->Iq, p_foc->Ia, p_foc->Ib, ic, p_foc->Sine, p_foc->Cosine);
// }

// /* Inverse DQ0 Transform - replaces Inverse Park + Inverse Clarke */
// static inline void FOC_ProcInvDQ0Transform(FOC_T * p_foc)
// {
//     foc_inv_dq_transform(&p_foc->Va, &p_foc->Vb, &p_foc->Vc, p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
// }

// /* Updated output processing using direct DQ0 transform */
// static inline void FOC_ProcOutputV_VBusInv_DQ0(FOC_T * p_foc, uint32_t vBusInv_fract32)
// {
//     FOC_ProcInvDQ0Transform(p_foc);
//     FOC_ProcSvpwm(p_foc, vBusInv_fract32);
// }

// static inline void FOC_ProcOutputV_VBus_DQ0(FOC_T * p_foc, ufract16_t vBus_fract16)
// {
//     FOC_ProcOutputV_VBusInv_DQ0(p_foc, ((uint32_t)FRACT16_MAX * 65536U / vBus_fract16));
// }

// /* VBemf processing using direct DQ0 transform */
// static inline void FOC_ProcVBemfDQ0Transform(FOC_T * p_foc)
// {
//     foc_dq_transform(&p_foc->Vd, &p_foc->Vq, p_foc->Va, p_foc->Vb, p_foc->Vc, p_foc->Sine, p_foc->Cosine);
// }


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