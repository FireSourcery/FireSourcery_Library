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
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Phase_Calibration.h"
#include "Phase_Input.h" /* Analog component for every module */
// #include "Phase_VBus.h"
#include "../Phase/Phase_Types.h"
#include "../Phase/Phase.h"
#include "Peripheral/Analog/Analog.h"

/*
    Common Calibration Max as preprocessor macros
*/
#ifndef PHASE_ANALOG_V_MAX_ADCU
#define PHASE_ANALOG_V_MAX_ADCU (4096U)
#define PHASE_ANALOG_V_FRACT16_SHIFT (3U)
#define PHASE_ANALOG_V_FRACT16_ADCU_SCALAR (1L << 3)
#endif

#ifndef PHASE_ANALOG_I_MAX_ADCU
#define PHASE_ANALOG_I_MAX_ADCU (2048U)
#define PHASE_ANALOG_I_FRACT16_SHIFT (4U)
#define PHASE_ANALOG_I_FRACT16_ADCU_SCALAR (1L << 4)
#endif

#ifdef PHASE_ANALOG_I_SENSOR_INVERT
#if   (PHASE_ANALOG_I_SENSOR_INVERT == false)
#define PHASE_ANALOG_I_SENSOR_INVERT_FACTOR (1)
#elif (PHASE_ANALOG_I_SENSOR_INVERT == true)
#define PHASE_ANALOG_I_SENSOR_INVERT_FACTOR (-1)
#endif
#else
#define PHASE_ANALOG_I_SENSOR_INVERT (false)
#define PHASE_ANALOG_I_SENSOR_INVERT_FACTOR (1)
#endif

// #define PHASE_ANALOG_I_SENSOR_INVERT_FACTOR ((PHASE_ANALOG_I_SENSOR_INVERT) ? -1 : 1)

// #if     defined(PHASE_ANALOG_I_SENSORS_AB)
// #elif   defined(PHASE_ANALOG_I_SENSORS_ABC)
// #else
// #define PHASE_ANALOG_I_SENSORS_ABC
// #endif

// #if     defined(PHASE_ANALOG_V_SENSORS_ISOLATED)
// #elif   defined(PHASE_ANALOG_V_SENSORS_ANALOG)
// #else
// #define PHASE_ANALOG_V_SENSORS_ANALOG
// #endif

static inline fract16_t Phase_Analog_VFract16Of(uint16_t adcu) { return adcu * PHASE_ANALOG_V_FRACT16_ADCU_SCALAR; }
static inline fract16_t Phase_Analog_IFract16Of(uint16_t zero, uint16_t adcu) { return ((int16_t)adcu - zero) * (PHASE_ANALOG_I_FRACT16_ADCU_SCALAR * PHASE_ANALOG_I_SENSOR_INVERT_FACTOR); }

/******************************************************************************/
/*
    ADC Ref Sensor Calibration
    optionally store as base ref
    Phase_Analog_Calibration_T
*/
/******************************************************************************/
typedef const struct Phase_AnalogSensor
{
    volatile uint32_t V_PHASE_R1;
    volatile uint32_t V_PHASE_R2;

    volatile uint16_t I_PHASE_R_BASE;      /* mOhm*1000 */
    volatile uint16_t I_PHASE_R_MOSFETS;   /* mOhm*1000 */
    volatile uint16_t I_PHASE_GAIN;        /* x10 */

    volatile uint16_t V_RATED;             /* VSource Limit */
    volatile uint16_t I_RATED_RMS;         /* */
}
Phase_AnalogSensor_T;

extern const Phase_AnalogSensor_T PHASE_ANALOG_SENSOR_REF;

/* Init */
// #define PHASE_ANALOG_I_MAX_INIT(Shunt_UOhm, Gain_10, VRef_MilliV) ((uint16_t)((1000.0F * VRef_MilliV * PHASE_ANALOG_I_MAX_ADCU / 4096) / ((Shunt) * (Gain) / 10.0F)))

static inline uint16_t Phase_AnalogSensor_GetVRated(void) { return PHASE_ANALOG_SENSOR_REF.V_RATED; }
static inline uint16_t Phase_AnalogSensor_GetIRatedRms(void) { return PHASE_ANALOG_SENSOR_REF.I_RATED_RMS; }

/*
    Run-time conversion
*/
static inline uint16_t Phase_AnalogSensor_GetVRated_Fract16(void) { return fract16(Phase_AnalogSensor_GetVRated(), Phase_Calibration_GetVMaxVolts()); }
static inline uint16_t Phase_AnalogSensor_GetIRatedPeak_Fract16(void) { return (uint32_t)Phase_AnalogSensor_GetIRatedRms() * FRACT16_SQRT2 / Phase_Calibration_GetIMaxAmps(); }

// void InitUnitsVSource(void)
// {
//     Linear_Voltage_Init(&UnitsVSource_V, PHASE_CALIBRATION.V_ABC_R1, PHASE_CALIBRATION.V_ABC_R2, ANALOG_REFERENCE.ADC_VREF_MILLIV, ANALOG_REFERENCE.ADC_BITS);
// }

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


/* Global "Static" Const, for all Motor instances */
// extern Analog_Conversion_T PHASE_ANALOG_VBUS;


/*
    alternatively, directly on register state
*/
// static void Phase_Analog_MarkConversions_Thread(const Phase_T * p_phase, const Phase_Analog_T * p_analog)
// {
//     Phase_Bitmask_T state = _Phase_ReadState(p_phase);
//     Analog_Conversion_Mark((state.A) ? &p_analog->IA : &p_analog->VA);
//     Analog_Conversion_Mark((state.B) ? &p_analog->IB : &p_analog->VB);
//     Analog_Conversion_Mark((state.C) ? &p_analog->IC : &p_analog->VC);
// }

static void Phase_Analog_MarkVabc(Phase_Analog_T * p_context)
{
// #if defined(PHASE_V_SENSORS_ANALOG)
    Analog_Conversion_Mark(&p_context->VA);
    Analog_Conversion_Mark(&p_context->VB);
    Analog_Conversion_Mark(&p_context->VC);
// #else
//     (void)p_context;
// #endif
}

static void Phase_Analog_MarkIabc(Phase_Analog_T * p_context)
{
    Analog_Conversion_Mark(&p_context->IA);
    Analog_Conversion_Mark(&p_context->IB);
// #if defined(PHASE_I_SENSORS_ABC)
    Analog_Conversion_Mark(&p_context->IC);
// #endif
}


// static inline void _Phase_ApplyAveraging(volatile int16_t * p_value, int16_t newValue) { *p_value = (*p_value + newValue) / 2; }

/*
    Capture on [Phase_Input]
*/
static inline void _Phase_Capture(volatile Phase_Triplet_T * p_triplet, volatile Phase_Bitmask_T * p_bits, Phase_Index_T channel, fract16_t newValue)
{
    p_triplet->Values[channel] = ((int32_t)p_triplet->Values[channel] + newValue) / 2;
    p_bits->Bits |= (1U << channel);
}

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

/*

*/
// static inline void Phase_Analog_CaptureVBus(uint16_t vSource_Adcu)
// {
//     Phase_VBus_CaptureFract16(Phase_Analog_VFract16Of(vSource_Adcu));
// }


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
