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
    @file   RotorSensor.h
    @author FireSourcery
    @brief  Motor Sensor Generic Interface. Submodule for Sensors.
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"
#include "Math/Angle/Angle.h"
#include "Math/Angle/Angle_SpeedPu.h"

#include <stdint.h>
#include <stdbool.h>


/*
    Rotor Angle Sensor
    Implement as conventional interface pattern, Signature context as interface type.
*/
struct RotorSensor;

typedef void(*RotorSensor_Proc_T)(const struct RotorSensor * p_sensor);
typedef bool(*RotorSensor_Test_T)(const struct RotorSensor * p_sensor);
typedef int (*RotorSensor_Get_T)(const struct RotorSensor * p_sensor);
typedef void(*RotorSensor_Set_T)(const struct RotorSensor * p_sensor, int value);


/*
    Interface/Operations
    No Null check. Must be fully defined, use empty function.
*/
typedef const struct RotorSensor_VTable
{
    RotorSensor_Proc_T CAPTURE_ANGLE;   /* ~20Khz */
    RotorSensor_Proc_T CAPTURE_SPEED;   /* ~1Khz */
    RotorSensor_Test_T IS_FEEDBACK_AVAILABLE; /* From Stop and after Align */
    RotorSensor_Proc_T ZERO_INITIAL;

    /* Config */
    RotorSensor_Proc_T INIT; /* Re init peripheral registers */
    RotorSensor_Proc_T INIT_UNITS; /* Sensor units from P_STATE->UnitRef */
    RotorSensor_Test_T VERIFY_CALIBRATION;
}
RotorSensor_VTable_T;


/*
    [RotorSensor_UnitRef_T]
    Filled by the motor. RotorSensor_InitUnitsFrom derives the sensor base units.
*/
typedef struct RotorSensor_UnitRef
{
    uint32_t PollingFreq;           /* Control loop frequency, AngleSpeed.Delta per cycle */
    uint8_t PolePairs;              /* Mechanical to electrical */
    angle_freq_t AngleFreqBase;     /* Speed base, electrical */
}
RotorSensor_UnitRef_T;

/*
    [Angle_T] Wrap
*/
typedef struct RotorSensor_State
{
    Angle_T AngleSpeed;     /* Electrical angle and speed state. */

    /* Derived from [RotorSensor_UnitRef_T] */
    struct
    {
        uint8_t PolePairs;              /* Mechanical to electrical */
        angle_freq_t AngleFreqBase;     /* Speed base, electrical. Exact, for sensors deriving their own factors */
        Angle_SpeedPuRef_T SpeedPuRef;  /* AngleDtBase at the control Fs, AngleSpeed.Delta per control cycle */
    }
    UnitRef;

    accum32_t Speed_Pu;
    angle16_t MechanicalAngle; /* optionally supported */
}
RotorSensor_State_T;

/*
    [RotorSensor_T]
    Instance/Entry
    Base class. No backpointer to container. Use as first member in container.
*/
typedef const struct RotorSensor
{
    RotorSensor_VTable_T * P_VTABLE;
    RotorSensor_State_T * P_STATE;
    // TimerT_T TIMER;
}
RotorSensor_T;

#define ROTOR_SENSOR_INIT(p_VTable, p_State) { .P_VTABLE = p_VTable, .P_STATE = p_State, }
#define ROTOR_SENSOR_ALLOC(p_VTable) ROTOR_SENSOR_INIT(p_VTable, &(RotorSensor_State_T){0})

/******************************************************************************/
/*
    Empty VTable for unimplemented sensors.
    Use in place of NULL pointer checks.
*/
/******************************************************************************/
extern const RotorSensor_VTable_T MOTOR_SENSOR_VTABLE_EMPTY;

#define ROTOR_SENSOR_INIT_AS_EMPTY(p_State) ROTOR_SENSOR_INIT(&MOTOR_SENSOR_VTABLE_EMPTY, p_State)

/******************************************************************************/
/*!
    Private / Protected
*/
/******************************************************************************/
static void _RotorSensor_Reset(RotorSensor_State_T * p_state)
{
    Angle_ZeroCaptureState(&p_state->AngleSpeed);
    p_state->Speed_Pu = 0;
}

/* θ_e = PolePairs · θ_m. For sensors reporting a mechanical angle */
static inline angle16_t _RotorSensor_ElectricalAngleOf(const RotorSensor_State_T * p_state, angle16_t mechanical) { return mechanical * p_state->UnitRef.PolePairs; }

/* Both views of one measurement: Δθ for the electrical model, Speed_Pu for the control loop. Rolling two-sample filter */
static inline void _RotorSensor_CaptureDelta(RotorSensor_State_T * p_state, int32_t delta) { p_state->AngleSpeed.Delta = delta / 2 + p_state->AngleSpeed.Delta / 2; }
/* separate for FreqD sensors */
static inline void _RotorSensor_CaptureSpeedPu(RotorSensor_State_T * p_state, accum32_t speed_pu) { p_state->Speed_Pu = fract16_sat((speed_pu + p_state->Speed_Pu) / 2); }


/******************************************************************************/
/*!
    Base class
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Public Interface / Virtual Functions
*/
/******************************************************************************/
static inline void RotorSensor_Init(RotorSensor_T * p_sensor)
{
    p_sensor->P_VTABLE->INIT(p_sensor);
    _RotorSensor_Reset(p_sensor->P_STATE);
}

/* OnControlLoop */
static inline void RotorSensor_CaptureAngle(RotorSensor_T * p_sensor) { p_sensor->P_VTABLE->CAPTURE_ANGLE(p_sensor); }
/* OnSpeedLoop */
static inline void RotorSensor_CaptureSpeed(RotorSensor_T * p_sensor) { p_sensor->P_VTABLE->CAPTURE_SPEED(p_sensor); }
/* OnStart */
static inline void RotorSensor_ZeroInitial(RotorSensor_T * p_sensor) { p_sensor->P_VTABLE->ZERO_INITIAL(p_sensor); /* p_sensor->P_STATE->DirectionErrorCount = 0; */ }

static inline bool RotorSensor_IsFeedbackAvailable(RotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->IS_FEEDBACK_AVAILABLE(p_sensor); }

/*
    Config
*/
static inline bool RotorSensor_VerifyCalibration(RotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->VERIFY_CALIBRATION(p_sensor); }

static inline void RotorSensor_InitUnitsFrom(RotorSensor_T * p_sensor, RotorSensor_UnitRef_T unitRef)
{
    p_sensor->P_STATE->UnitRef.PolePairs = unitRef.PolePairs;
    p_sensor->P_STATE->UnitRef.AngleFreqBase = unitRef.AngleFreqBase;
    p_sensor->P_STATE->UnitRef.SpeedPuRef = Angle_SpeedPuRef_FromFreq(unitRef.PollingFreq, unitRef.AngleFreqBase);
    p_sensor->P_VTABLE->INIT_UNITS(p_sensor);
}


/******************************************************************************/
/*!
    Query Functions
*/
/******************************************************************************/
/* Electrical Angle State. Substitute getters */
static inline const Angle_T * RotorSensor_GetAngleState(RotorSensor_T * p_sensor) { return &p_sensor->P_STATE->AngleSpeed; }

/* Angle Feedback. Shared E-Cycle edge detect, User output */
static inline angle16_t RotorSensor_GetElectricalAngle(RotorSensor_T * p_sensor) { return Angle_Value(&p_sensor->P_STATE->AngleSpeed); }
// ElectricalDeltaAngle, DigitalSpeed [Degrees Per ControlCycle]
/* Electrical Speed,  < 32768 by SpeedRated */
static inline angle16_t RotorSensor_GetElectricalDelta(RotorSensor_T * p_sensor) { return Angle_Delta(&p_sensor->P_STATE->AngleSpeed); }
/* fract16 [-32767:32767]*2 Speed Feedback Variable. -/+ => virtual CW/CCW */
static inline int32_t RotorSensor_GetSpeed_Pu(RotorSensor_T * p_sensor) { return p_sensor->P_STATE->Speed_Pu; }
/* Speed sampled over 1ms */
static inline sign_t RotorSensor_GetFeedbackDirection(RotorSensor_T * p_sensor) { return math_sign(RotorSensor_GetSpeed_Pu(p_sensor)); }

static inline angle16_t RotorSensor_GetMechanicalAngle(RotorSensor_T * p_sensor) { return p_sensor->P_STATE->MechanicalAngle; }




/******************************************************************************/
/*!
    Id interface
*/
/******************************************************************************/
/*
    Rotor Angle/Speed State + Feedback
    Read-Only, RealTime
*/
typedef enum Motor_Var_Rotor
{
    MOTOR_VAR_ROTOR_ELECTRICAL_ANGLE,   /* in digital degrees */
    MOTOR_VAR_ROTOR_ELECTRICAL_DELTA,   /* Internal Ccw/Cw */
    MOTOR_VAR_ROTOR_SPEED_FEEDBACK,     /* Internal Ccw/Cw */
    MOTOR_VAR_ROTOR_MECHANICAL_ANGLE,   /* if supported */
    MOTOR_VAR_ROTOR_DIRECTION,          /* 1:Ccw, -1:Cw, 0:Stop  */
}
Motor_Var_Rotor_T;

static int _Motor_Var_Rotor_Get(RotorSensor_T * p_sensor, Motor_Var_Rotor_T varId)
{
    switch (varId)
    {
        case MOTOR_VAR_ROTOR_ELECTRICAL_ANGLE:   return RotorSensor_GetElectricalAngle(p_sensor);
        case MOTOR_VAR_ROTOR_ELECTRICAL_DELTA:   return RotorSensor_GetElectricalDelta(p_sensor);
        case MOTOR_VAR_ROTOR_SPEED_FEEDBACK:     return RotorSensor_GetSpeed_Pu(p_sensor);
        case MOTOR_VAR_ROTOR_MECHANICAL_ANGLE:   return RotorSensor_GetMechanicalAngle(p_sensor);
        case MOTOR_VAR_ROTOR_DIRECTION:          return RotorSensor_GetFeedbackDirection(p_sensor);
        default: return 0;
    }
}

// int _Motor_Var_Rotor_Get(Motor_T * p_motor, Motor_Var_Rotor_T varId);

// #ifndef ROTOR_SENSOR_POLLING_FREQ
// #define ROTOR_SENSOR_POLLING_FREQ (20000U)
// #endif

/* Assign virtual sign convention */
/* A -> B as positive/CCW */
// typedef enum RotorSensor_Direction
// {
//     MOTOR_DIRECTION_CW = -1,
//     MOTOR_DIRECTION_NULL = 0,
//     MOTOR_DIRECTION_CCW = 1,
// }
// RotorSensor_Direction_T;