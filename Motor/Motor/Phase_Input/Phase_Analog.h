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
    @file   Phase_Analog.h
    @author FireSourcery
    @brief  Analog component for every Phase_Input module
*/
/******************************************************************************/
#include "Phase_Input.h"
#include "../Phase/Phase_Types.h"
#include "Peripheral/ADC/ADC_Conversion.h"
#include "Peripheral/ADC/ADC_Reference.h"
#include "Math/Fixed/fract16.h"


/******************************************************************************/
/*
    ADC Sensor Calibration
*/
/******************************************************************************/
/*
    Common Calibration Max as preprocessor macros
*/
#ifndef PHASE_ANALOG_V_MAX_ADCU
#define PHASE_ANALOG_V_MAX_ADCU (4096U)
#define PHASE_ANALOG_V_PU_PER_ADCU (1L << 3U)
#endif

#ifndef PHASE_ANALOG_I_MAX_ADCU
#define PHASE_ANALOG_I_MAX_ADCU (2048U)
#define PHASE_ANALOG_I_PU_PER_ADCU (1L << 4U)
#endif

#ifndef PHASE_ANALOG_I_POLARITY
#ifdef PHASE_ANALOG_I_SENSOR_INVERT
#define PHASE_ANALOG_I_POLARITY (-1)
#else
#define PHASE_ANALOG_I_POLARITY (1)
#endif
#endif

/*
    PHASE_ANALOG_I_SENSOR_LOW_SIDE
    The I sensor reads a phase only while its low-side switch conducts: a low-side shunt, or the switch's R_DS(on).
    The analog thread converts the 2 phases of the longest low-side windows [Motor_Analog_MarkI], the capture solves the third.
*/

static inline fract16_t Phase_Analog_VPuOf(uint16_t adcu) { return adcu * PHASE_ANALOG_V_PU_PER_ADCU; }
static inline fract16_t Phase_Analog_IPuOf(uint16_t zero, uint16_t adcu) { return ((int16_t)adcu - zero) * (PHASE_ANALOG_I_PU_PER_ADCU * PHASE_ANALOG_I_POLARITY); }

/******************************************************************************/
/*!

*/
/******************************************************************************/
#define PHASE_ANALOG_V_MAX_VOLTS(VRef_mV, R1, R2) ((((VRef_mV)*((R1) +(R2))) + ((R2)*1000U/2U)) / ((R2)*1000U))
/* (VRef_mV/2 * 1/1000) / (R_shunt_uOhm * 1/1000000 * Gain) */
#define PHASE_ANALOG_I_MAX_AMPS(VRef_mV, Shunt_uOhm, Gain) (((500U*(VRef_mV)) + ((Shunt_uOhm)*(Gain)/2U)) / ((Shunt_uOhm)*(Gain)))
// #define I_MAX_AMPS_(VRef_mV, Shunt_uOhm, Gain)  ((1000U * (VRef_mV)) / ((Shunt_uOhm) * (Gain)))

/*
    optionally store as base ref
*/
typedef const struct Phase_AnalogBoard
{
    volatile uint32_t V_PHASE_R1;
    volatile uint32_t V_PHASE_R2;
    volatile uint16_t I_PHASE_R_BASE;
    volatile uint16_t I_PHASE_R_MOSFETS;
    volatile uint16_t I_PHASE_R_SHUNT;      /* uOhm */
    volatile uint16_t I_PHASE_GAIN;         /* x10 */
}
Phase_AnalogBoard_T;

extern const Phase_AnalogBoard_T PHASE_ANALOG_BOARD;

static inline uint16_t Phase_Analog_VMax(void) { return PHASE_ANALOG_V_MAX_VOLTS(ADC_REFERENCE.ADC_VREF_MILLIV, PHASE_ANALOG_BOARD.V_PHASE_R1, PHASE_ANALOG_BOARD.V_PHASE_R2); }
static inline uint16_t Phase_Analog_IMax(void) { return PHASE_ANALOG_I_MAX_AMPS(ADC_REFERENCE.ADC_VREF_MILLIV, PHASE_ANALOG_BOARD.I_PHASE_R_SHUNT, PHASE_ANALOG_BOARD.I_PHASE_GAIN); }


/******************************************************************************/
/*!

*/
/******************************************************************************/
typedef const struct Phase_Analog
{
    ADC_Conversion_T VA;
    ADC_Conversion_T VB;
    ADC_Conversion_T VC;
    ADC_Conversion_T IA;
    ADC_Conversion_T IB;
    ADC_Conversion_T IC;
    // Phase_Triplet_T * P_IZERO_REFS_ADCU;
}
Phase_Analog_T;


static void Phase_Analog_MarkVabc(Phase_Analog_T * p_analog)
{
    ADC_Conversion_Mark(&p_analog->VA);
    ADC_Conversion_Mark(&p_analog->VB);
    ADC_Conversion_Mark(&p_analog->VC);
}

static void Phase_Analog_MarkIabc(Phase_Analog_T * p_analog)
{
    ADC_Conversion_Mark(&p_analog->IA);
    ADC_Conversion_Mark(&p_analog->IB);
    ADC_Conversion_Mark(&p_analog->IC);
}

static inline void Phase_Analog_MarkI(Phase_Analog_T * p_analog, Phase_Bitmask_T phases)
{
    if (phases.A) { ADC_Conversion_Mark(&p_analog->IA); }
    if (phases.B) { ADC_Conversion_Mark(&p_analog->IB); }
    if (phases.C) { ADC_Conversion_Mark(&p_analog->IC); }
}

/*
    alternatively, directly on register state
*/
static void Phase_Analog_Mark(const Phase_Analog_T * p_analog, Phase_Bitmask_T state)
{
    ADC_Conversion_Mark((state.A) ? &p_analog->IA : &p_analog->VA);
    ADC_Conversion_Mark((state.B) ? &p_analog->IB : &p_analog->VB);
    ADC_Conversion_Mark((state.C) ? &p_analog->IC : &p_analog->VC);
}

// #if defined(PHASE_ANALOG_I_SENSOR_LOW_SIDE)
// static inline void Motor_Analog_MarkI(Motor_T * p_motor) { Phase_Analog_MarkI(&p_motor->PHASE_ANALOG, Phase_Bitmask(Phase_Inverse(Phase_ReadDutyMax(&p_motor->PHASE)))); }
// #else
// static inline void Motor_Analog_MarkI(Motor_T * p_motor) { Motor_Analog_MarkIabc(p_motor); }
// #endif


/*
    Analog Part for Input
    Capture on [Phase_Input_T]
*/

static inline void Phase_Analog_CaptureV(volatile Phase_Input_T * p_phase, Phase_Index_T channel, adc_result_t adcu)
{
    // assert(adcu <= PHASE_ANALOG_V_MAX_ADCU);
    Phase_Capture(&p_phase->V, channel, Phase_Analog_VPuOf(adcu));
}

static inline void Phase_Analog_CaptureVa(volatile Phase_Input_T * p_phase, adc_result_t adcu) { Phase_Analog_CaptureV(p_phase, PHASE_INDEX_A, adcu); }
static inline void Phase_Analog_CaptureVb(volatile Phase_Input_T * p_phase, adc_result_t adcu) { Phase_Analog_CaptureV(p_phase, PHASE_INDEX_B, adcu); }
static inline void Phase_Analog_CaptureVc(volatile Phase_Input_T * p_phase, adc_result_t adcu) { Phase_Analog_CaptureV(p_phase, PHASE_INDEX_C, adcu); }


/*
    A three-wire load carries i_a + i_b + i_c = 0, so 2 phases determine the third.
    Completes the capture on its second phase. A third conversion, where marked, replaces the solved value with its own.
    Out of line: each channel's capture calls it.
*/
__attribute__((noinline)) static void _Phase_Analog_SolveI(volatile Phase_Data_T * p_i)
{
    switch (p_i->Flags.Bits)
    {
        case PHASE_ID_INV_A: p_i->Values.A = fract16_sat(0 - ((int32_t)p_i->Values.B + p_i->Values.C)); break;
        case PHASE_ID_INV_B: p_i->Values.B = fract16_sat(0 - ((int32_t)p_i->Values.C + p_i->Values.A)); break;
        case PHASE_ID_INV_C: p_i->Values.C = fract16_sat(0 - ((int32_t)p_i->Values.A + p_i->Values.B)); break;
        default: return;
    }
    p_i->Flags.Bits = PHASE_ID_ABC;
}

static inline void Phase_Analog_CaptureI(volatile Phase_Input_T * p_phase, const Phase_Triplet_T * p_zeros, Phase_Index_T channel, adc_result_t adcu)
{
    // assert(adcu <= PHASE_ANALOG_I_MAX_ADCU);
    Phase_Capture(&p_phase->I, channel, Phase_Analog_IPuOf(p_zeros->Values[channel], adcu));
#if defined(PHASE_ANALOG_I_SENSOR_LOW_SIDE)
    _Phase_Analog_SolveI(&p_phase->I);
#endif
}

static inline void Phase_Analog_CaptureIa(volatile Phase_Input_T * p_phase, const Phase_Triplet_T * p_zeros, adc_result_t adcu) { Phase_Analog_CaptureI(p_phase, p_zeros, PHASE_INDEX_A, adcu); }
static inline void Phase_Analog_CaptureIb(volatile Phase_Input_T * p_phase, const Phase_Triplet_T * p_zeros, adc_result_t adcu) { Phase_Analog_CaptureI(p_phase, p_zeros, PHASE_INDEX_B, adcu); }
static inline void Phase_Analog_CaptureIc(volatile Phase_Input_T * p_phase, const Phase_Triplet_T * p_zeros, adc_result_t adcu) { Phase_Analog_CaptureI(p_phase, p_zeros, PHASE_INDEX_C, adcu); }


static inline Phase_Data_T Phase_Iabc_PuOfAdcu(const Phase_Triplet_T * p_zeros, adc_result_t a, adc_result_t b, adc_result_t c)
{
    return (Phase_Data_T)
    {
        .Values =
        {
            .A = Phase_Analog_IPuOf(p_zeros->Values[PHASE_INDEX_A], a),
            .B = Phase_Analog_IPuOf(p_zeros->Values[PHASE_INDEX_B], b),
            .C = Phase_Analog_IPuOf(p_zeros->Values[PHASE_INDEX_C], c),
        },
        .Flags.Bits = PHASE_ID_ABC,
    };
}

/*
    One batch's I set: [Phase_Data_T] of ADCU, Flags the phases converted, the others' values stale.
    Low-side sensing converts 2 of 3 [Motor_Analog_MarkI], and the third is solved.
*/
static inline void Phase_Analog_CaptureIBatch(volatile Phase_Input_T * p_phase, const Phase_Triplet_T * p_zeros, Phase_Data_T adcu)
{
    p_phase->I = Phase_Data(Phase_Analog_IPuOf(p_zeros->A, adcu.A), Phase_Analog_IPuOf(p_zeros->B, adcu.B), Phase_Analog_IPuOf(p_zeros->C, adcu.C), Phase_IdOf(adcu.Flags));
#if defined(PHASE_ANALOG_I_SENSOR_LOW_SIDE)
    _Phase_Analog_SolveI(&p_phase->I);
#endif
}

static inline Phase_Data_T Phase_Vabc_PuOfAdcu(adc_result_t a, adc_result_t b, adc_result_t c)
{
    return (Phase_Data_T)
    {
        .Values =
        {
            .A = Phase_Analog_VPuOf(a),
            .B = Phase_Analog_VPuOf(b),
            .C = Phase_Analog_VPuOf(c),
        },
        .Flags.Bits = PHASE_ID_ABC,
    };
}



// /*!
//     @brief Virtual channel identifiers
// */
// typedef enum Phase_AnalogChannel
// {
//     PHASE_ADC_CHANNEL_VA,
//     PHASE_ADC_CHANNEL_VB,
//     PHASE_ADC_CHANNEL_VC,
//     PHASE_ADC_CHANNEL_IA,
//     PHASE_ADC_CHANNEL_IB,
//     PHASE_ADC_CHANNEL_IC,
// }
// Phase_AnalogChannel_T;

