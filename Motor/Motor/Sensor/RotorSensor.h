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

#include <stdint.h>
#include <stdbool.h>

// #ifndef ROTOR_SENSOR_POLLING_FREQ
// #define ROTOR_SENSOR_POLLING_FREQ (20000U)
// #endif

/*
    Rotor Angle Sensor
    Implement as conventional interface pattern, Signature context as interface type.
*/
struct RotorSensor;
struct RotorSensor_Config;

typedef void(*RotorSensor_Proc_T)(const struct RotorSensor * p_sensor);
typedef bool(*RotorSensor_Test_T)(const struct RotorSensor * p_sensor);
typedef int (*RotorSensor_Get_T)(const struct RotorSensor * p_sensor);
typedef void(*RotorSensor_Set_T)(const struct RotorSensor * p_sensor, int value);
typedef void(*RotorSensor_InitFrom_T)(const struct RotorSensor * p_sensor, const struct RotorSensor_Config * p_config);

/* Assign virtual sign convention */
/* A -> B as positive/CCW */
// typedef enum RotorSensor_Direction
// {
//     MOTOR_DIRECTION_CW = -1,
//     MOTOR_DIRECTION_NULL = 0,
//     MOTOR_DIRECTION_CCW = 1,
// }
// RotorSensor_Direction_T;

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
    RotorSensor_InitFrom_T INIT_UNITS_FROM;
    RotorSensor_Test_T VERIFY_CALIBRATION;
}
RotorSensor_VTable_T;


/*
    [RotorSensor_Units_T]/[RotorSensor_Calibration_T]
*/
typedef struct RotorSensor_Config
{
    uint8_t PolePairs;              /* Motor Pole Pairs. Config Mech/Electrical conversion */

    /* Config scalar speed. Caller derive  */
    uint16_t SpeedTypeMax_Rpm; /* mechanical */
    uint16_t SpeedTypeMax_DegPerCycle; /* electrical */

    // uint16_t MismatchLimit;

    // optionally move
    // Motor_Direction_T DirectionForward; /* CCW/CW Assigned positive direction */
    // uint8_t PolePairs;                  /* Motor Pole Pairs. Use to derive Mech/Electrical speed calibration */
    // uint16_t Kv;                        /* [RpmPerVolt] Motor Constant. Use to derive SpeedVRef. Optionally sets SpeedRated */
    // uint16_t SpeedRated_Rpm;            /* [Rpm] for same units as kv. Speed at nominal VSource. Clamp or scale limits. Derives Angle and Fract16 */
}
RotorSensor_Config_T;

/*
    [Angle_T] Wrap
*/
typedef struct RotorSensor_State
{
    Angle_T AngleSpeed;     /* Electrical angle and speed state. */
    Angle_SpeedFractRef_T SpeedFractRef;
    accum32_t Speed_Fract16;
    angle16_t MechanicalAngle;
    // RotorSensor_Config_T Config;
}
RotorSensor_State_T;

// static inline angle16_t _RotorSensor_GetElectricalAngle(const RotorSensor_State_T * p_state) { return p_state->AngleSpeed.Angle; }
// static inline angle16_t _RotorSensor_GetElectricalDelta(const RotorSensor_State_T * p_state) { return p_state->AngleSpeed.Delta; }
// static inline int32_t _RotorSensor_GetSpeed_Fract16(const RotorSensor_State_T * p_state) { return p_state->AngleSpeed.Speed_Fract16; }
// static inline int32_t _RotorSensor_GetDirection(const RotorSensor_State_T * p_state) { return p_state->Direction; }
// static inline angle16_t _RotorSensor_GetMechanicalAngle(const RotorSensor_State_T * p_state) { return p_state->MechanicalAngle; }


/*
    [RotorSensor_T]
    Instance/Entry
    No backpointer to container. Use as first member in container.
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
    Private
*/
/******************************************************************************/
static void _RotorSensor_Reset(RotorSensor_State_T * p_state)
{
    Angle_ZeroCaptureState(&p_state->AngleSpeed);
    p_state->Speed_Fract16 = 0;
}


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
static inline void RotorSensor_Init(const RotorSensor_T * p_sensor)
{
    p_sensor->P_VTABLE->INIT(p_sensor);
    _RotorSensor_Reset(p_sensor->P_STATE);
}

/* OnControlLoop */
static inline void RotorSensor_CaptureAngle(const RotorSensor_T * p_sensor) { p_sensor->P_VTABLE->CAPTURE_ANGLE(p_sensor); }
/* OnSpeedLoop */
static inline void RotorSensor_CaptureSpeed(const RotorSensor_T * p_sensor) { p_sensor->P_VTABLE->CAPTURE_SPEED(p_sensor); }
/* OnStart */
static inline void RotorSensor_ZeroInitial(const RotorSensor_T * p_sensor) { p_sensor->P_VTABLE->ZERO_INITIAL(p_sensor); /* p_sensor->P_STATE->DirectionErrorCount = 0; */ }

static inline bool RotorSensor_IsFeedbackAvailable(const RotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->IS_FEEDBACK_AVAILABLE(p_sensor); }

/*
    Config
*/
static inline bool RotorSensor_VerifyCalibration(const RotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->VERIFY_CALIBRATION(p_sensor); }

static inline void RotorSensor_InitUnitsFrom(const RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    p_sensor->P_STATE->SpeedFractRef = ANGLE_SPEED_FRACT_REF(p_config->SpeedTypeMax_DegPerCycle);
    p_sensor->P_VTABLE->INIT_UNITS_FROM(p_sensor, p_config);
}


/******************************************************************************/
/*!
    Query Functions
*/
/******************************************************************************/
/* Electrical Angle State. Subsitute getters */
static inline const Angle_T * RotorSensor_GetAngleState(const RotorSensor_T * p_sensor) { return &p_sensor->P_STATE->AngleSpeed; }

/* Angle Feedback. Shared E-Cycle edge detect, User output */
static inline angle16_t RotorSensor_GetElectricalAngle(const RotorSensor_T * p_sensor) { return Angle_Value(&p_sensor->P_STATE->AngleSpeed); }
// ElectricalDeltaAngle, DigitalSpeed [Degrees Per ControlCycle]
/* Electrical Speed,  < 32768 by SpeedRated */
static inline angle16_t RotorSensor_GetElectricalDelta(const RotorSensor_T * p_sensor) { return Angle_Delta(&p_sensor->P_STATE->AngleSpeed); }
/* fract16 [-32767:32767]*2 Speed Feedback Variable. -/+ => virtual CW/CCW */
static inline int32_t RotorSensor_GetSpeed_Fract16(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->Speed_Fract16; }
/* Speed sampled over 1ms */
static inline sign_t RotorSensor_GetFeedbackDirection(const RotorSensor_T * p_sensor) { return math_sign(RotorSensor_GetSpeed_Fract16(p_sensor)); }

static inline angle16_t RotorSensor_GetMechanicalAngle(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->MechanicalAngle; }


#ifndef ROTOR_DIRECTION_SPEED_THRESHOLD_FRACT16
#define ROTOR_DIRECTION_SPEED_THRESHOLD_FRACT16 (((int32_t)INT16_MAX * 2) / 64) /* ~2% */
#endif
static inline bool RotorSensor_IsSpeedReliable(const RotorSensor_T * p_sensor) { return math_abs(RotorSensor_GetSpeed_Fract16(p_sensor)) > ROTOR_DIRECTION_SPEED_THRESHOLD_FRACT16; }

static inline sign_t RotorSensor_GetEffectiveFeedbackDirection(const RotorSensor_T * p_sensor) { return (RotorSensor_IsSpeedReliable(p_sensor) ? RotorSensor_GetFeedbackDirection(p_sensor) : 0); }


/* set the direction compensation */
/* Optionally force sensor direction detection. handle internal */
// static inline void RotorSensor_SetDirectionComp(const RotorSensor_T * p_sensor, int direction) { p_sensor->P_VTABLE->SET_DIRECTION(p_sensor, direction); }

