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
    @file   MotorSensor.h
    @author FireSourcery
    @brief  Generic Sensor Interface
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"

// #include "Type/Mux.h"
#include <stdint.h>
#include <stdbool.h>


/*
    Implement as conventional interface pattern, Signiture as interface type.
*/
struct MotorSensor;
// typedef angle16_t(*MotorSensor_Capture_T)(const struct MotorSensor * p_sensor);
typedef void(*MotorSensor_Proc_T)(const struct MotorSensor * p_sensor);
typedef bool(*MotorSensor_Test_T)(const struct MotorSensor * p_sensor);

typedef int (*MotorSensor_Get_T)(const struct MotorSensor * p_sensor);
typedef void(*MotorSensor_Set_T)(const struct MotorSensor * p_sensor, int value);

/* inline vtable */
// typedef void(*MotorSensor_InlineProc_T)(void);

/*
    Interface/Operations
    No Null check. Must be fully defined, use empty funcction.
*/
typedef const struct MotorSensor_VTable
{
    MotorSensor_Proc_T INIT;
    MotorSensor_Proc_T CAPTURE_ANGLE;
    MotorSensor_Proc_T CAPTURE_SPEED;
    MotorSensor_Test_T IS_FEEDBACK_AVAILABLE;
    MotorSensor_Proc_T RESET_UNITS;
    MotorSensor_Test_T VERIFY_CALIBRATION;

    // MotorSensor_Proc_T ZERO;
    MotorSensor_Get_T GET_DIRECTION;
    MotorSensor_Set_T SET_DIRECTION;
}
MotorSensor_VTable_T;

/*
    Empty VTable for unimplemented sensors.
    Use to avoid NULL pointer checks.
*/
extern const MotorSensor_VTable_T MOTOR_SENSOR_VTABLE_EMPTY;

/*
    [MotorSensor_State_T]
*/
/* altneratively move with feedbackstate */
typedef struct MotorSensor_State
{
    angle16_t ElectricalAngle;  /* Angle Feedback. Shared E-Cycle edge detect, User output */
    int32_t AngularSpeed_DegPerCycle; /* Electrical Delta Angle */
    // angle16_t ElectricalAngleSpeed;
    // angle16_t ElectricalAnglePerCycle;
    int32_t Speed_Fract16;      /* fract16 [-32767:32767]*2 Speed Feedback Variable. -/+ => virtual CW/CCW */

    angle16_t MechanicalAngle;
    int Direction;
    // Units
}
MotorSensor_State_T;

/*
    Instance
    No backpointer to container. Use as first member in container.
*/
typedef const struct MotorSensor
{
    MotorSensor_VTable_T * const P_VTABLE;
    MotorSensor_State_T * const P_STATE;
    // SpeedAngle_T * const P_STATE;
}
MotorSensor_T;

#define MOTOR_SENSOR_INIT(p_VTable, p_State) { .P_VTABLE = p_VTable, .P_STATE = p_State, }
#define MOTOR_SENSOR_ALLOC(p_VTable) MOTOR_SENSOR_INIT(p_VTable, &(MotorSensor_State_T){0})

#define MOTOR_SENSOR_INIT_AS_EMPTY(p_State) { .P_VTABLE = &MOTOR_SENSOR_VTABLE_EMPTY, .P_STATE = p_State, }


/*
    VarId for Generic Common Values
*/
typedef enum MotorSensor_VarId
{
    MOTOR_SENSOR_VAR_ELECTRICAL_ANGLE,
    MOTOR_SENSOR_VAR_MECHANICAL_ANGLE,
    MOTOR_SENSOR_VAR_ELECTRICAL_SPEED_DEGREES,
    MOTOR_SENSOR_VAR_DIRECTION,
    // MOTOR_SENSOR_VAR_ELECTRICAL_SPEED_RADS,
    // MOTOR_SENSOR_VAR_MECHANICAL_SPEED_RPM,
}
MotorSensor_VarId_T;

static inline void MotorSensor_Init(MotorSensor_T * p_sensor)
{
    // assert(p_sensor == p_sensor->P_BASE);
    p_sensor->P_VTABLE->INIT(p_sensor);
}

/* OnControlLoop */
static inline void MotorSensor_CaptureAngle(const MotorSensor_T * p_sensor) { p_sensor->P_VTABLE->CAPTURE_ANGLE(p_sensor); }
/* OnSpeedLoop */
static inline void MotorSensor_CaptureSpeed(const MotorSensor_T * p_sensor) { p_sensor->P_VTABLE->CAPTURE_SPEED(p_sensor); }

static inline bool MotorSensor_VerifyCalibration(const MotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->VERIFY_CALIBRATION(p_sensor); }
static inline bool MotorSensor_IsFeedbackAvailable(const MotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->IS_FEEDBACK_AVAILABLE(p_sensor); }
static inline void MotorSensor_ResetUnits(const MotorSensor_T * p_sensor) { p_sensor->P_VTABLE->RESET_UNITS(p_sensor); }

static inline void MotorSensor_SetDirection(const MotorSensor_T * p_sensor, int direction) { p_sensor->P_VTABLE->SET_DIRECTION(p_sensor, direction); }
static inline int MotorSensor_GetDirection(const MotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->GET_DIRECTION(p_sensor); }

// static inline void MotorSensor_SetDirection(const MotorSensor_T * p_sensor, int direction) { p_sensor->P_STATE->Direction = direction; }
// static inline int MotorSensor_GetDirection(const MotorSensor_T * p_sensor) { return p_sensor->P_STATE->Direction; }


/* Position */
static inline int32_t MotorSensor_GetElecticalAngle(const MotorSensor_State_T * p_state) { return p_state->ElectricalAngle; }
static inline int32_t MotorSensor_GetMechanicalAngle(const MotorSensor_State_T * p_state) { return p_state->MechanicalAngle; }
/* Displacement of Electrical Angle at CONTROL_FREQ */
static inline int32_t MotorSensor_GetElectricalAngleSpeed(const MotorSensor_State_T * p_state) { return p_state->AngularSpeed_DegPerCycle; }
// static inline int32_t MotorSensor_GetElectricalAnglePerCycle(const MotorSensor_State_T * p_state) { return p_state->AngularSpeed_DegPerCycle; }

static inline int32_t MotorSensor_GetSpeed_Fract16(const MotorSensor_State_T * p_state) { return p_state->Speed_Fract16; }


// static inline int32_t MotorSensor_GetElecticalAngle(const MotorSensor_T * p_sensor) { return p_sensor->P_STATE->ElectricalAngle; }
// static inline int32_t MotorSensor_GetMechanicalAngle(const MotorSensor_T * p_sensor) { return p_sensor->P_STATE->MechanicalAngle; }
// /* Displacement of Electrical Angle at CONTROL_FREQ */
// static inline int32_t MotorSensor_GetElectricalAngleSpeed(const MotorSensor_T * p_sensor) { return p_sensor->P_STATE->AngularSpeed_DegPerCycle; }
// // static inline int32_t MotorSensor_GetElectricalAnglePerCycle(const MotorSensor_T * p_sensor) { return p_sensor->P_STATE-> AngularSpeed_DegPerCycle; }
// static inline int32_t MotorSensor_GetSpeed_Fract16(const MotorSensor_T * p_sensor) { return p_sensor->P_STATE->Speed_Fract16; }

// static inline int32_t MotorSensor_GetRpm(const MotorSensor_State_T * p_state) {}


// static inline angle16_t MotorSensor_PollAngle(MotorSensor_T * p_sensor)
// {
//     MotorSensor_CaptureAngle(p_sensor);
//     return MotorSensor_GetElecticalAngle(p_sensor);
// }

// static inline angle16_t MotorSensor_PollSpeed(MotorSensor_T * p_sensor)
// {
//     MotorSensor_CaptureSpeed(p_sensor);
//     return MotorSensor_GetControlDeltaAngle(p_sensor);
// }

/* OnStart */
// void MotorSensor_ZeroState(const Motor_T * p_motor);




