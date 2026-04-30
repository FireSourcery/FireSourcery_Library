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
#include "Phase_Calibration.h"
#include "Phase_Input.h"
#include "../Phase/Phase_Types.h"
#include "Peripheral/Analog/Analog.h"

/*
    Common Calibration Max as preprocessor macros
*/
#ifndef PHASE_ANALOG_V_MAX_ADCU
#define PHASE_ANALOG_V_MAX_ADCU (4096U)
#define PHASE_ANALOG_V_FRACT16_SHIFT (3U)
#define PHASE_ANALOG_V_FRACT16_FACTOR (1L << PHASE_ANALOG_V_FRACT16_SHIFT)
#endif

#ifndef PHASE_ANALOG_I_MAX_ADCU
#define PHASE_ANALOG_I_MAX_ADCU (2048U)
#define PHASE_ANALOG_I_FRACT16_SHIFT (4U)
#define PHASE_ANALOG_I_FRACT16_FACTOR (1L << PHASE_ANALOG_I_FRACT16_SHIFT)
#endif


#ifndef PHASE_ANALOG_I_POLARITY
#ifdef PHASE_ANALOG_I_SENSOR_INVERT
#define PHASE_ANALOG_I_POLARITY (-1)
#else
#define PHASE_ANALOG_I_POLARITY (1)
#endif
#endif

static inline fract16_t Phase_Analog_VFract16Of(uint16_t adcu) { return adcu * PHASE_ANALOG_V_FRACT16_FACTOR; }
static inline fract16_t Phase_Analog_IFract16Of(uint16_t zero, uint16_t adcu) { return ((int16_t)adcu - zero) * (PHASE_ANALOG_I_FRACT16_FACTOR * PHASE_ANALOG_I_POLARITY); }

/******************************************************************************/
/*
    ADC Sensor Calibration
    optionally store as base ref
*/
/******************************************************************************/
typedef const struct Phase_AnalogCalibration
{
    volatile uint32_t V_PHASE_R1;
    volatile uint32_t V_PHASE_R2;

    volatile uint16_t I_PHASE_R_BASE;      /* mOhm*1000 */
    volatile uint16_t I_PHASE_R_MOSFETS;   /* mOhm*1000 */
    volatile uint16_t I_PHASE_GAIN;        /* x10 */

    volatile uint16_t V_RATED;             /* VSource Limit */
    volatile uint16_t I_RATED_RMS;         /* */
}
Phase_AnalogCalibration_T;

extern const Phase_AnalogCalibration_T PHASE_ANALOG_CALIBRATION;

#define PHASE_ANALOG_V_MAX_VOLTS(VRef_mV, R1, R2) (((VRef_mV) * ((R1) + (R2))) + ((R2)*1000U/2U) / ((R2) * 1000U))
/* (VRef_mV/2 * 1/1000) / (R_shunt_uOhm * 1/1000000 * Gain) */
#define PHASE_ANALOG_I_MAX_AMPS(VRef_mV, Shunt_uOhm, Gain) ((500U * (VRef_mV)) + ((Shunt_uOhm)*(Gain)/2U) / ((Shunt_uOhm) * (Gain)))
// #define I_MAX_AMPS_(VRef_mV, Shunt_uOhm, Gain)  ((1000U * (VRef_mV)) / ((Shunt_uOhm) * (Gain)))

// static inline uint16_t Phase_AnalogCalibration_GetVMax(void) { return PHASE_ANALOG_V_MAX_VOLTS(ANALOG_REFERENCE.ADC_VREF_MILLIV, PHASE_ANALOG_CALIBRATION.V_PHASE_R1, PHASE_ANALOG_CALIBRATION.V_PHASE_R2); }

// handled in calibration
static inline uint16_t Phase_AnalogCalibration_GetVRated(void) { return PHASE_ANALOG_CALIBRATION.V_RATED; }
static inline uint16_t Phase_AnalogCalibration_GetIRatedRms(void) { return PHASE_ANALOG_CALIBRATION.I_RATED_RMS; }

/*
    Run-time conversion
*/
static inline uint16_t Phase_AnalogCalibration_GetVRated_Fract16(void) { return fract16(Phase_AnalogCalibration_GetVRated(), Phase_Calibration_GetVMaxVolts()); }
static inline uint16_t Phase_AnalogCalibration_GetIRatedPeak_Fract16(void) { return (uint32_t)Phase_AnalogCalibration_GetIRatedRms() * FRACT16_SQRT2 / Phase_Calibration_GetIMaxAmps(); }


/******************************************************************************/
/*!

*/
/******************************************************************************/
typedef const struct Phase_Analog
{
    Analog_Conversion_T VA;
    Analog_Conversion_T VB;
    Analog_Conversion_T VC;
    Analog_Conversion_T IA;
    Analog_Conversion_T IB;
    Analog_Conversion_T IC;
    // Phase_Triplet_T * P_IZERO_REFS_ADCU;
}
Phase_Analog_T;

/*
    Pass Struct and Index
*/
#define PHASE_ANALOG_INIT(AdcVa, IndexVa, AdcVb, IndexVb, AdcVc, IndexVc, AdcIa, IndexIa, AdcIb, IndexIb, AdcIc, IndexIc) \
{ \
    .VA = ANALOG_CONVERSION_INIT_FROM(AdcVa, IndexVa), \
    .VB = ANALOG_CONVERSION_INIT_FROM(AdcVb, IndexVb), \
    .VC = ANALOG_CONVERSION_INIT_FROM(AdcVc, IndexVc), \
    .IA = ANALOG_CONVERSION_INIT_FROM(AdcIa, IndexIa), \
    .IB = ANALOG_CONVERSION_INIT_FROM(AdcIb, IndexIb), \
    .IC = ANALOG_CONVERSION_INIT_FROM(AdcIc, IndexIc), \
}

static void Phase_Analog_MarkVabc(Phase_Analog_T * p_analog)
{
    Analog_Conversion_Mark(&p_analog->VA);
    Analog_Conversion_Mark(&p_analog->VB);
    Analog_Conversion_Mark(&p_analog->VC);
}

static void Phase_Analog_MarkIabc(Phase_Analog_T * p_analog)
{
    Analog_Conversion_Mark(&p_analog->IA);
    Analog_Conversion_Mark(&p_analog->IB);
    Analog_Conversion_Mark(&p_analog->IC);
}

/*
    alternatively, directly on register state
*/
static void Phase_Analog_Mark(const Phase_Analog_T * p_analog, Phase_Bitmask_T state)
{
    Analog_Conversion_Mark((state.A) ? &p_analog->IA : &p_analog->VA);
    Analog_Conversion_Mark((state.B) ? &p_analog->IB : &p_analog->VB);
    Analog_Conversion_Mark((state.C) ? &p_analog->IC : &p_analog->VC);
}


/*
    Analog Part for Input
    Capture on [Phase_Input]
*/
static inline void _Phase_CaptureVAdcu(volatile Phase_Triplet_T * p_triplet, volatile Phase_Bitmask_T * p_bits, Phase_Index_T channel, adc_result_t adcu)
{
    // assert(adcu <= PHASE_ANALOG_V_MAX_ADCU);
    _Phase_Capture(p_triplet, p_bits, channel, Phase_Analog_VFract16Of(adcu));
}

static inline void Phase_Analog_CaptureVa(volatile Phase_Input_T * p_phase, adc_result_t adcu) { _Phase_CaptureVAdcu(&p_phase->Vabc, &p_phase->VFlags, PHASE_INDEX_A, adcu); }
static inline void Phase_Analog_CaptureVb(volatile Phase_Input_T * p_phase, adc_result_t adcu) { _Phase_CaptureVAdcu(&p_phase->Vabc, &p_phase->VFlags, PHASE_INDEX_B, adcu); }
static inline void Phase_Analog_CaptureVc(volatile Phase_Input_T * p_phase, adc_result_t adcu) { _Phase_CaptureVAdcu(&p_phase->Vabc, &p_phase->VFlags, PHASE_INDEX_C, adcu); }

static inline void _Phase_CaptureIAdcu(volatile Phase_Triplet_T * p_triplet, volatile Phase_Bitmask_T * p_bits, const Phase_Triplet_T * p_zeroRefs, Phase_Index_T channel, adc_result_t adcu)
{
    // assert(adcu <= PHASE_ANALOG_I_MAX_ADCU);
    _Phase_Capture(p_triplet, p_bits, channel, Phase_Analog_IFract16Of(p_zeroRefs->Values[channel], adcu));
}

static inline void Phase_Analog_CaptureIa(volatile Phase_Input_T * p_phase, const Phase_Triplet_T * p_zeroRefs, adc_result_t adcu) { _Phase_CaptureIAdcu(&p_phase->Iabc, &p_phase->IFlags, p_zeroRefs, PHASE_INDEX_A, adcu); }
static inline void Phase_Analog_CaptureIb(volatile Phase_Input_T * p_phase, const Phase_Triplet_T * p_zeroRefs, adc_result_t adcu) { _Phase_CaptureIAdcu(&p_phase->Iabc, &p_phase->IFlags, p_zeroRefs, PHASE_INDEX_B, adcu); }
static inline void Phase_Analog_CaptureIc(volatile Phase_Input_T * p_phase, const Phase_Triplet_T * p_zeroRefs, adc_result_t adcu) { _Phase_CaptureIAdcu(&p_phase->Iabc, &p_phase->IFlags, p_zeroRefs, PHASE_INDEX_C, adcu); }



// /*!
//     @brief Virtual channel identifiers
// */
// typedef enum Phase_AnalogChannel
// {
//     PHASE_ANALOG_CHANNEL_VA,
//     PHASE_ANALOG_CHANNEL_VB,
//     PHASE_ANALOG_CHANNEL_VC,
//     PHASE_ANALOG_CHANNEL_IA,
//     PHASE_ANALOG_CHANNEL_IB,
//     PHASE_ANALOG_CHANNEL_IC,
// }
// Phase_AnalogChannel_T;

// #if     defined(PHASE_ANALOG_I_SENSORS_AB)
// #elif   defined(PHASE_ANALOG_I_SENSORS_ABC)
// #else
// #define PHASE_ANALOG_I_SENSORS_ABC
// #endif