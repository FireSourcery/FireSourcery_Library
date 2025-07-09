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
    @brief  Motor Sensor Generic Interface

*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"

#include <stdint.h>
#include <stdbool.h>


/*
    Rotor Angle Sensor
    Implement as conventional interface pattern, Signiture as interface type.
*/
struct RotorSensor;
struct RotorSensor_Config;

typedef void(*RotorSensor_Proc_T)(const struct RotorSensor * p_sensor);
typedef bool(*RotorSensor_Test_T)(const struct RotorSensor * p_sensor);

typedef int (*RotorSensor_Get_T)(const struct RotorSensor * p_sensor);
typedef void(*RotorSensor_Set_T)(const struct RotorSensor * p_sensor, int value);

typedef void(*RotorSensor_InitFrom_T)(const struct RotorSensor * p_sensor, const struct RotorSensor_Config * p_config);

/* alternatively */
// typedef void(*RotorSensor_Capture_T)(const struct RotorSensor * p_sensor, AngleSpeed_T * p_state);
// typedef int(*RotorSensor_Capture_T)(const struct RotorSensor * p_sensor);
/* inline vtable */
// typedef void(*RotorSensor_InstanceProc_T)(void);

/*
    Interface/Operations
    No Null check. Must be fully defined, use empty funcction.
*/
typedef const struct RotorSensor_VTable
{
    RotorSensor_Proc_T INIT;
    RotorSensor_Proc_T CAPTURE_ANGLE;
    RotorSensor_Proc_T CAPTURE_SPEED;
    RotorSensor_Test_T IS_FEEDBACK_AVAILABLE;
    // RotorSensor_Proc_T ZERO;
    RotorSensor_Get_T GET_DIRECTION;
    RotorSensor_Set_T SET_DIRECTION; // SET_CAPTURE_DIRECTION;

    /* Config */
    RotorSensor_InitFrom_T INIT_UNITS_FROM;
    RotorSensor_Test_T VERIFY_CALIBRATION;
}
RotorSensor_VTable_T;


/*
    [RotorSensor_Config_T]
    [RotorSensor_Units_T]
*/
typedef struct RotorSensor_Config
{
    uint8_t PolePairs;                      /* Motor Pole Pairs. Config Mech/Electrical conversion */
    uint16_t ElSpeedRated_DegPerCycle;      /* ~Speed at nominal VSource. Config scalar speed.
                                                as electrical speed in degrees per cycle.
                                                mech speed >= VSource*Kv */
    uint16_t MechSpeedRated_Rpm;
}
RotorSensor_Config_T;

/*
    [AngleSpeed_T]
*/
/* altneratively move with feedbackstate */
typedef struct RotorSensor_State
{
    angle16_t ElectricalAngle;  /* Angle Feedback. Shared E-Cycle edge detect, User output */
    angle16_t ElectricalSpeed_DegPerCycle;  /* Electrical Speed, ElectricalDeltaAngle. < 32768 by SpeedRated */
                                            // ElectricalDeltaAngle, DigitalSpeed [Degrees Per ControlCycle]

    int32_t Speed_Fract16;  /* fract16 [-32767:32767]*2 Speed Feedback Variable. -/+ => virtual CW/CCW */
    // int32_t SpeedPrev_Fract16; /* common filter */

    angle16_t MechanicalAngle;
    int Direction;
    // Units
}
RotorSensor_State_T;

/* Position */
static inline int32_t _RotorSensor_GetElecticalAngle(const RotorSensor_State_T * p_state) { return p_state->ElectricalAngle; }
static inline int32_t _RotorSensor_GetMechanicalAngle(const RotorSensor_State_T * p_state) { return p_state->MechanicalAngle; }

/* Speed Angle - Displacement of Electrical Angle at CONTROL_FREQ */
static inline int32_t _RotorSensor_GetElectricalSpeed(const RotorSensor_State_T * p_state) { return p_state->ElectricalSpeed_DegPerCycle; }
// static inline int32_t _RotorSensor_GetElectricalSpeed_RadPerS(const RotorSensor_State_T * p_state) { }

static inline int32_t _RotorSensor_GetSpeed_Fract16(const RotorSensor_State_T * p_state) { return p_state->Speed_Fract16; }
// static inline int32_t _RotorSensor_GetRpm(const RotorSensor_State_T * p_state) {}
static inline int32_t _RotorSensor_GetDirection(const RotorSensor_State_T * p_state) { return p_state->Direction; }

static inline void _RotorSensor_Reset(RotorSensor_State_T * p_state)
{
    p_state->ElectricalAngle = 0;
    p_state->ElectricalSpeed_DegPerCycle = 0;
    p_state->Speed_Fract16 = 0;
    p_state->MechanicalAngle = 0;
    p_state->Direction = 0;
}

/*
    [RotorSensor_T]
    Instance/Entry
    No backpointer to container. Use as first member in container.
*/
typedef const struct RotorSensor
{
    const RotorSensor_VTable_T * P_VTABLE;
    RotorSensor_State_T * P_STATE; // AngleSpeed_T * const P_STATE;
}
RotorSensor_T;

#define MOTOR_SENSOR_INIT(p_VTable, p_State) { .P_VTABLE = p_VTable, .P_STATE = p_State, }
#define MOTOR_SENSOR_ALLOC(p_VTable) MOTOR_SENSOR_INIT(p_VTable, &(RotorSensor_State_T){0})

/******************************************************************************/
/*
    Empty VTable for unimplemented sensors.
    Use to avoid NULL pointer checks.
*/
/******************************************************************************/
extern const RotorSensor_VTable_T MOTOR_SENSOR_VTABLE_EMPTY;

#define MOTOR_SENSOR_INIT_AS_EMPTY(p_State) MOTOR_SENSOR_INIT(&MOTOR_SENSOR_VTABLE_EMPTY, p_State)


/******************************************************************************/
/*!
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
// void RotorSensor_ZeroState(const Motor_T * p_motor);
static inline void RotorSensor_SetDirection(const RotorSensor_T * p_sensor, int direction) { p_sensor->P_VTABLE->SET_DIRECTION(p_sensor, direction); }
// static inline int RotorSensor_GetDirection(const RotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->GET_DIRECTION(p_sensor); }
// static inline void RotorSensor_SetDirection(const RotorSensor_T * p_sensor, int direction) { p_sensor->P_STATE->Direction = direction; }

static inline bool RotorSensor_IsFeedbackAvailable(const RotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->IS_FEEDBACK_AVAILABLE(p_sensor); }

/*
    Config
*/
static inline bool RotorSensor_VerifyCalibration(const RotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->VERIFY_CALIBRATION(p_sensor); }
static inline void RotorSensor_InitUnitsFrom(const RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config) { p_sensor->P_VTABLE->INIT_UNITS_FROM(p_sensor, p_config); }

/******************************************************************************/
/*!
    Query Functions
*/
/******************************************************************************/
static inline angle16_t RotorSensor_PollAngle(const RotorSensor_T * p_sensor)
{
    RotorSensor_CaptureAngle(p_sensor);
    return _RotorSensor_GetElecticalAngle(p_sensor->P_STATE);
}

static inline angle16_t RotorSensor_PollSpeed(const RotorSensor_T * p_sensor)
{
    RotorSensor_CaptureSpeed(p_sensor);
    return _RotorSensor_GetElectricalSpeed(p_sensor->P_STATE);
}

static inline int32_t RotorSensor_GetElecticalAngle(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->ElectricalAngle; }
static inline int32_t RotorSensor_GetElectricalAngleSpeed(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->ElectricalSpeed_DegPerCycle; }

static inline int32_t RotorSensor_GetSpeed_Fract16(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->Speed_Fract16; }
// static inline int32_t RotorSensor_GetSpeed_Fract16(const RotorSensor_T * p_sensor) { return speed_fract16_of_angle16(p_sensor->P_STATE->ElectricalSpeed_DegPerCycle, p_sensor->P_STATE->P_STATE->Config.ElSpeedRated_DegPerCycle); }

static inline int32_t RotorSensor_GetMechanicalAngle(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->MechanicalAngle; }
static inline int RotorSensor_GetDirection(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->Direction; }
// static inline int RotorSensor_GetDirection(const RotorSensor_T * p_sensor) { return math_sign(p_sensor->P_STATE->ElectricalSpeed_DegPerCycle); }


