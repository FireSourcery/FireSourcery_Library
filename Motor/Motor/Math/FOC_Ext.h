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
#include "../Phase/Phase_VOut.h"
#include "Math/PID/PID.h"
#include "Math/Ramp/Ramp.h"

/******************************************************************************/
/*!
    Collaborator Pattern
*/
/******************************************************************************/
/* From Iabc to Idq */
static inline bool FOC_CaptureIabc(FOC_T * p_foc, volatile Phase_Data_T * p_phaseData)
{
    if (p_phaseData->Flags.Bits == PHASE_ID_ABC)  /* alternatively use batch callback */
    {
        FOC_ProcClarkePark(p_foc, p_phaseData->Values.A, p_phaseData->Values.B, p_phaseData->Values.C);
        p_phaseData->Flags.Bits = PHASE_ID_0; /* Clear capture flag after processing */
        return true;
    }
    else
    {
        return false; /* No new data captured */
    }
}

/* FeedbackState */
// static inline void  _FOC_CaptureFeedback(FOC_T * p_foc, Phase_Data_T * p_phaseData, angle16_t angle)
// {
//     FOC_SetTheta(p_foc, angle);
//     FOC_CaptureIabc(p_foc, p_phaseData);
// }
// static inline void  _FOC_ProcIFeedbackLoop(FOC_T * p_foc, Phase_Data_T * p_phaseData, angle16_t angle, ufract16_t vBus,  int16_t dReq, int16_t qReq)
// {
//     FOC_SetTheta(p_foc, angle);
//     FOC_CaptureIabc(p_foc, p_phaseData);
//     FOC_ProcIFeedback(p_foc, vBus, direction, dReq, qReq);
//     FOC_ProcOutputDuty(p_foc, Phase_VBus_Inv_Fract32());
// }

static inline void FOC_CaptureVBemf(FOC_T * p_foc, volatile Phase_Data_T * p_input)
{
    if (p_input->Flags.Bits == PHASE_ID_ABC)
    {
        FOC_ProcVBemfClarkePark(p_foc, p_input->Values.A, p_input->Values.B, p_input->Values.C);
        p_input->Flags.Bits = PHASE_ID_0;
    }
}

// static inline void FOC_ProcCaptureVBemf(FOC_T * p_foc, Phase_Data_T * p_input, angle16_t theta)
// {
//     FOC_SetTheta(p_foc, theta);
//     if (p_input->Flags.Bits == PHASE_ID_ABC)
//     {
//         FOC_ProcVBemfClarkePark(p_foc, p_input->Values.A, p_input->Values.B, p_input->Values.C);
//         p_input->Flags.Bits = PHASE_ID_0;
//     }
// }

static inline Phase_Triplet_T FOC_Vabc(const FOC_T * p_foc) { return (Phase_Triplet_T) { .A = p_foc->Va, .B = p_foc->Vb, .C = p_foc->Vc, }; }


static inline Phase_Triplet_T FOC_ProcVOut(const FOC_T * p_foc)
{
    struct foc_abc vOut = foc_inv_clarke_park(p_foc->Vd, p_foc->Vq, p_foc->Sine, p_foc->Cosine);
    return (Phase_Triplet_T) { .A = vOut.a, .B = vOut.b, .C = vOut.c, };
}


// // FOC_Electrical_T FOC_Electrical_FromSi_Kv(FOC_Electrical_T * p_electrical, Motor_ElectricalSpeedRating_T kv, uint16_t vBus_V)
// {
//     p_config->ElectricalParams_Pu.Psi = psi_pu_rpm_of_kv(Phase_Calibration_GetVMaxVolts(), Motor_GetSpeedTypeMax_Rpm(&p_config->SpeedRating) / 2, p_config->SpeedRating.Kv);
// }

static inline FOC_Electrical_T FOC_Electrical_FromSi(FOC_Electrical_T * p_electrical, uint32_t eRadsBase)
{
    return (FOC_Electrical_T)
    {
        .Ld = l_pu_rads_of_h(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), eRadsBase, p_electrical->Ld, 1000000UL),
        .Lq = l_pu_rads_of_h(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), eRadsBase, p_electrical->Lq, 1000000UL),
        .Rs = rs_pu_of_mohm(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_electrical->Rs),
        .Psi = psi_pu_rads_of_wb(Phase_Calibration_GetVMaxVolts(), eRadsBase, p_electrical->Psi, 1000000UL),
    };
}


/******************************************************************************/
/*!

*/
/******************************************************************************/
// static inline FOC_DecouplingCoeff_T Motor_InitDecouplingCoeffs(ld, lq, kv)
// {
//     return (FOC_DecouplingCoeff_T)
//     {
//         .Ld_Fract16 = l_fract16_of_uh(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_config->Ld_MicroHenries),
//         .Lq_Fract16 = l_fract16_of_uh(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_config->Lq_MicroHenries),
//         .Psi_Fract16 =   ke_vfract16_per_angle16(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), p_config->Kv)
//     };
// }


// typedef enum FOC_Quadrant
// {
//     FOC_QUADRANT_0 = 0,    /* Zero / Idle */
//     FOC_QUADRANT_FORWARD_MOTORING   = 1,    /* Q1: +Vq, +Iq */
//     FOC_QUADRANT_FORWARD_GENERATING = 2,    /* Q2: +Vq, -Iq */
//     FOC_QUADRANT_REVERSE_MOTORING   = 3,    /* Q3: -Vq, -Iq */
//     FOC_QUADRANT_REVERSE_GENERATING = 4,    /* Q4: -Vq, +Iq */
// }
// FOC_Quadrant_T;

// static inline FOC_Quadrant_T FOC_GetQuadrant(const FOC_T * p_foc)
// {
//     if (p_foc->Iq == 0 && p_foc->Vq == 0) { return FOC_QUADRANT_0; }
//     if (p_foc->Iq > 0)  { return (p_foc->Vq >= 0) ? FOC_QUADRANT_FORWARD_MOTORING : FOC_QUADRANT_REVERSE_GENERATING; }
//     else                { return (p_foc->Vq <= 0) ? FOC_QUADRANT_REVERSE_MOTORING : FOC_QUADRANT_FORWARD_GENERATING; }
// }


/******************************************************************************/
/*!

*/
/******************************************************************************/
/* Estimate efficiency based on I²R losses */
// static inline ufract16_t FOC_GetEfficiency(const FOC_T * p_foc, fract16_t rPhase)
// {
//     accum32_t activePower = FOC_GetActivePower(p_foc);
//     if (activePower <= 0) return 0;

//     ufract16_t iMag = FOC_GetIMagnitude(p_foc);
//     accum32_t resistiveLoss = fract16_mul(fract16_mul(iMag, iMag), rPhase) * 3;

//     return (activePower << 15) / (activePower + resistiveLoss);
// }




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

/*
    Debug
*/
// bool FOC_ValidateInputs(const FOC_T * p_foc)
// {
//     /* Check for reasonable current values */
//     if (abs(p_foc->Ia) > FRACT16_MAX * 9 / 10) return false;
//     if (abs(p_foc->Ib) > FRACT16_MAX * 9 / 10) return false;
//     if (abs(p_foc->Ic) > FRACT16_MAX * 9 / 10) return false;

//     /* Validate Kirchhoff's current law: Ia + Ib + Ic ≈ 0 */
//     int32_t currentSum = (int32_t)p_foc->Ia + p_foc->Ib + p_foc->Ic;
//     if (abs(currentSum) > FRACT16_MAX / 20) return false;  /* 5% tolerance */

//     /* Check for reasonable voltage requests */
//     int32_t vMagSq = (int32_t)p_foc->Vd * p_foc->Vd + (int32_t)p_foc->Vq * p_foc->Vq;
//     if (vMagSq > (int32_t)FRACT16_MAX * FRACT16_MAX) return false;

//     return true;
// }

// bool FOC_ValidateTheta(fract16_t sine, fract16_t cosine)
// {
//     /* Validate unit circle: sin²θ + cos²θ ≈ 1 */
//     int32_t magnitudeSquared = (int32_t)sine * sine + (int32_t)cosine * cosine;
//     int32_t expected = (int32_t)FRACT16_MAX * FRACT16_MAX;
//     int32_t tolerance = expected / 20;  /* 5% tolerance */

//     return abs(magnitudeSquared - expected) < tolerance;
// }
