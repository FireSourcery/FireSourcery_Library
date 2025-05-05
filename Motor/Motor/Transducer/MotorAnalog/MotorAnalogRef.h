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
    @file   MotorAnalogRef.h
    @author FireSourcery
    @version V0
    @brief  Global Static, for all Motor instances
*/
/******************************************************************************/
#ifndef MOTOR_ANALOG_REFERENCE_H
#define MOTOR_ANALOG_REFERENCE_H

#include "Peripheral/Analog/AnalogReference.h"
#include "Math/Fixed/fract16.h"

/*
    Common as preprocessor macros
*/
#ifndef MOTOR_ANALOG_V_MAX_ADCU
#define MOTOR_ANALOG_V_MAX_ADCU (4096U) /* Calibration Max */
#define MOTOR_ANALOG_V_FRACT16_SHIFT (3U)
#endif

#ifndef MOTOR_ANALOG_I_MAX_ADCU
#define MOTOR_ANALOG_I_MAX_ADCU (2048U) /* Calibration Max */
#define MOTOR_ANALOG_I_FRACT16_SHIFT (4U)
#endif

#ifndef MOTOR_ANALOG_I_SENSOR_INVERT
#define MOTOR_ANALOG_I_SENSOR_INVERT false
#endif

/* ADCU Factor */
#define MOTOR_ANALOG_V_FRACT16_ADCU_SCALAR (1L << MOTOR_ANALOG_V_FRACT16_SHIFT)
#define MOTOR_ANALOG_I_FRACT16_ADCU_SCALAR ((1L << MOTOR_ANALOG_I_FRACT16_SHIFT) * (MOTOR_ANALOG_I_SENSOR_INVERT ? -1 : 1))

static inline accum32_t MotorAnalogRef_V_Fract16(int16_t adcu) { return adcu * MOTOR_ANALOG_V_FRACT16_ADCU_SCALAR; }
static inline accum32_t MotorAnalogRef_I_Fract16(int16_t adcu) { return adcu * MOTOR_ANALOG_I_FRACT16_ADCU_SCALAR; }
// static inline fract16_t MotorAnalogRef_I_Fract16_Zero(uint16_t zero, uint16_t adcu) { return adcu * MOTOR_ANALOG_I_FRACT16_ADCU_SCALAR; }

/******************************************************************************/
/*
    Global "Static" Const.
*/
/******************************************************************************/
/* Sensor Calibration */
typedef const struct MotorAnalogRef_Board
{
    const uint32_t V_PHASE_R1;
    const uint32_t V_PHASE_R2;

    const uint16_t I_PHASE_R_BASE;
    const uint16_t I_PHASE_R_MOSFETS;   /* mOhm*1000 */
    const uint16_t I_PHASE_GAIN;        /* x10 */

    const uint16_t V_RATED;             /* VSource Limit */
    const uint16_t I_RATED_RMS;         /* */
}
MotorAnalogRef_Board_T;

extern const MotorAnalogRef_Board_T MOTOR_ANALOG_REFERENCE_BOARD; /* Optionally */

static inline uint16_t MotorAnalogRef_GetVRated(void) { return MOTOR_ANALOG_REFERENCE_BOARD.V_RATED; }
static inline uint16_t MotorAnalogRef_GetIRatedRms(void) { return MOTOR_ANALOG_REFERENCE_BOARD.I_RATED_RMS; }

/*
    Run-time
*/
// static inline uint16_t MotorAnalogRef_GetVRated_Fract16(void) { return fract16(MotorAnalogRef_GetVRated(), MotorAnalogRef_GetVMaxVolts()); }
// static inline uint16_t MotorAnalogRef_GetIRatedPeak_Fract16(void) { return (uint32_t)MotorAnalogRef_GetIRatedRms() * FRACT16_SQRT2 / MotorAnalogRef_GetIMaxAmps(); }

/* applies to external sensor */
typedef const struct MotorAnalogReference
{
    const uint16_t V_MAX_VOLTS;         /* Calibration Max. Unit conversion reference. Compile time derived */
    const uint16_t I_MAX_AMPS;          /* Calibration Max. Unit conversion reference. Compile time derived */

    const uint16_t V_RATED_FRACT16;
    const uint16_t I_RATED_PEAK_FRACT16;
}
MotorAnalogRef_T;

/* Init */
// #define MOTOR_ANALOG_I_MAX_INIT(Shunt_UOhm, Gain_10, VRef_MilliV) ((uint16_t)((1000.0F * VRef_MilliV * MOTOR_ANALOG_I_MAX_ADCU / 4096) / ((Shunt) * (Gain) / 10.0F)))

/* Define in Main App */
/* extern const allows run-time overwrite */
extern const MotorAnalogRef_T MOTOR_ANALOG_REFERENCE;

/* Getter interface. */
static inline uint16_t MotorAnalogRef_GetIMaxAmps(void) { return MOTOR_ANALOG_REFERENCE.I_MAX_AMPS; }
static inline uint16_t MotorAnalogRef_GetVMaxVolts(void) { return MOTOR_ANALOG_REFERENCE.V_MAX_VOLTS; }

static inline uint16_t MotorAnalogRef_GetVRated_Fract16(void) { return MOTOR_ANALOG_REFERENCE.V_RATED_FRACT16; }
static inline uint16_t MotorAnalogRef_GetIRatedPeak_Fract16(void) { return MOTOR_ANALOG_REFERENCE.I_RATED_PEAK_FRACT16; }
static inline int16_t MotorAnalogRef_GetVRated_Adcu(void) { return MotorAnalogRef_GetVRated_Fract16() / MOTOR_ANALOG_V_FRACT16_ADCU_SCALAR; }
static inline int16_t MotorAnalogRef_GetIRatedPeak_Adcu(void){ return MotorAnalogRef_GetIRatedPeak_Fract16() / MOTOR_ANALOG_I_FRACT16_ADCU_SCALAR; }

static inline uint16_t MotorAnalogRef_ILimitOf(uint16_t i_fract16) { return math_min(MotorAnalogRef_GetIRatedPeak_Fract16(), i_fract16); }
static inline uint16_t MotorAnalogRef_VLimitOf(uint16_t v_fract16) { return math_min(MotorAnalogRef_GetVRated_Fract16(), v_fract16); }

static inline bool _MotorAnalogRef_IsLoaded(uint16_t Value) { return (Value != 0U && Value != 0xFFFFU); }
static inline bool MotorAnalogRef_IsLoaded(void)
{
    return
    (
        _MotorAnalogRef_IsLoaded(MotorAnalogRef_GetVMaxVolts()) &&
        _MotorAnalogRef_IsLoaded(MotorAnalogRef_GetIMaxAmps()) &&
        _MotorAnalogRef_IsLoaded(MotorAnalogRef_GetVRated_Fract16()) &&
        _MotorAnalogRef_IsLoaded(MotorAnalogRef_GetIRatedPeak_Fract16())
    );
}

/* static in .c */
extern void MotorAnalogRef_InitVSource_V(uint16_t vSource_V);
extern void MotorAnalogRef_CaptureVSource_Adcu(uint16_t vSource_Adcu);
extern uint16_t MotorAnalogRef_GetVSource_Fract16(void);
extern uint16_t MotorAnalogRef_GetVSource_Adcu(void);
extern uint16_t MotorAnalogRef_GetVSource_V(void);

extern uint16_t MotorAnalogRef_GetVSourceInvScalar(void);

// extern uint16_t MotorAnalogRef_GetVNominal_V(void);



#endif
