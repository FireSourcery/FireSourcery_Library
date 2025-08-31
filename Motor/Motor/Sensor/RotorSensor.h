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
#include "Math/Angle/Angle.h"

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
// typedef void(*RotorSensor_Capture_T)(const struct RotorSensor * p_sensor, Angle_T * p_state);
// typedef int(*RotorSensor_Capture_T)(const struct RotorSensor * p_sensor);
/* inline vtable */
// typedef void(*RotorSensor_InstanceProc_T)(void);

// typedef enum RotorSensor_Direction
// {
//     MOTOR_DIRECTION_CW = -1,
//     MOTOR_DIRECTION_NULL = 0,
//     MOTOR_DIRECTION_CCW = 1,
// }
// RotorSensor_Direction_T;

/*
    Interface/Operations
    No Null check. Must be fully defined, use empty funcction.
*/
typedef const struct RotorSensor_VTable
{
    RotorSensor_Proc_T CAPTURE_ANGLE;   /* ~20Khz */
    RotorSensor_Proc_T CAPTURE_SPEED;   /* ~1Khz */
    RotorSensor_Test_T IS_FEEDBACK_AVAILABLE; /* From Stop and after Align */
    RotorSensor_Proc_T ZERO_INITIAL;
    // RotorSensor_Get_T GET_DIRECTION;
    RotorSensor_Set_T SET_DIRECTION; // SET_CAPTURE_DIRECTION;

    /* Config */
    RotorSensor_Proc_T INIT; /* Re init peripheral registers */
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

    /*
        ~Speed at nominal VSource. Config scalar speed. Caller derive for simplicity.
    */
    uint16_t ElSpeedRated_DegPerCycle;
    uint16_t MechSpeedRated_Rpm;
}
RotorSensor_Config_T;

/*
    [Angle_T] Wrap
*/
typedef struct RotorSensor_State
{
    /* Angle and Speed State */
    /* Electrical Angle Per Control Cycle */
    /* AngleSpeed.Speed - Angle Displacement of Electrical Angle at CONTROL_FREQ */
    Angle_T AngleSpeed;

    angle16_t MechanicalAngle;
    int Direction; /* RotorSensor_Direction. Feedback Direction. Alternatively use speed */
    int DirectionErrorCount;
}
RotorSensor_State_T;

/*
    [RotorSensor_T]
    Instance/Entry
    No backpointer to container. Use as first member in container.
*/
typedef const struct RotorSensor
{
    const RotorSensor_VTable_T * P_VTABLE;
    RotorSensor_State_T * P_STATE;
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
    // p_state->ElectricalAngle = 0;
    // p_state->ElectricalSpeed_DegPerCycle = 0;
    // p_state->Speed_Fract16 = 0;
    // p_state->MechanicalAngle = 0;
    // p_state->Direction = 0;
    p_state->AngleSpeed.Angle = 0;
    p_state->AngleSpeed.Speed = 0;
    p_state->AngleSpeed.Speed_Fract16 = 0;
    // p_state->Direction = 0;
}

/* Getters in case implementation changes */
/* Position */
static inline angle16_t _RotorSensor_GetElecticalAngle(const RotorSensor_State_T * p_state) { return p_state->AngleSpeed.Angle; }
/* Speed Angle - Displacement of Electrical Angle at CONTROL_FREQ */
static inline angle16_t _RotorSensor_GetElectricalSpeed(const RotorSensor_State_T * p_state) { return p_state->AngleSpeed.Speed; }
static inline int32_t _RotorSensor_GetSpeed_Fract16(const RotorSensor_State_T * p_state) { return p_state->AngleSpeed.Speed_Fract16; }
// static inline int32_t RotorSensor_GetSpeed_Fract16(const RotorSensor_State_T * p_state) { return speed_fract16_of_angle16(p_state->ElectricalSpeed_DegPerCycle, p_state->Config.ElSpeedRated_DegPerCycle); }

static inline int32_t _RotorSensor_GetDirection(const RotorSensor_State_T * p_state) { return p_state->Direction; }
// static inline int32_t _RotorSensor_GetSensorDirection(const RotorSensor_State_T * p_state) { return math_sign(p_state->ElectricalSpeed_DegPerCycle); }

// static inline int32_t _RotorSensor_GetElectricalSpeed_RadPerS(const RotorSensor_State_T * p_state) { }
// static inline int32_t _RotorSensor_GetRpm(const RotorSensor_State_T * p_state) {}
static inline angle16_t _RotorSensor_GetMechanicalAngle(const RotorSensor_State_T * p_state) { return p_state->MechanicalAngle; }

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
static inline void RotorSensor_ZeroInitial(const RotorSensor_T * p_sensor) { p_sensor->P_VTABLE->ZERO_INITIAL(p_sensor); p_sensor->P_STATE->DirectionErrorCount = 0; }

/* Optionally for sensor without built-in direction detection */
static inline void RotorSensor_SetDirection(const RotorSensor_T * p_sensor, int direction) { p_sensor->P_VTABLE->SET_DIRECTION(p_sensor, direction); }
// static inline void RotorSensor_SetDirection(const RotorSensor_T * p_sensor, int direction) { p_sensor->P_STATE->Direction = direction; }

// static inline int RotorSensor_GetDirection(const RotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->GET_DIRECTION(p_sensor); }

static inline bool RotorSensor_IsFeedbackAvailable(const RotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->IS_FEEDBACK_AVAILABLE(p_sensor); }

/*
    Config
*/
static inline bool RotorSensor_VerifyCalibration(const RotorSensor_T * p_sensor) { return p_sensor->P_VTABLE->VERIFY_CALIBRATION(p_sensor); }
static inline void RotorSensor_InitUnitsFrom(const RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config) { p_sensor->P_VTABLE->INIT_UNITS_FROM(p_sensor, p_config); }

/******************************************************************************/
/*!
    Base class
*/
/******************************************************************************/
// static inline void _Motor_SetFeedforwardDelta(Motor_State_T * p_motor, angle16_t elSpeed_degPerCycle)
// {
//     p_motor->SensorState.ElectricalAngle += elSpeed_degPerCycle;
//     p_motor->SensorState.ElectricalSpeed_DegPerCycle = elSpeed_degPerCycle;
//     p_motor->SensorState.Speed_Fract16 = speed_fract16_of_angle16_rpm(MOTOR_CONTROL_FREQ, Motor_GetSpeedRatedRef_ERpm(p_motor), elSpeed_degPerCycle);
//     p_motor->SensorState.MechanicalAngle += (elSpeed_degPerCycle / p_motor->Config.PolePairs);
// }

/******************************************************************************/
/*!
    Combined
*/
/******************************************************************************/
// static inline angle16_t RotorSensor_PollAngle(const RotorSensor_T * p_sensor)
// {
//     RotorSensor_CaptureAngle(p_sensor);
//     return _RotorSensor_GetElecticalAngle(p_sensor->P_STATE);
// }

// static inline angle16_t RotorSensor_PollSpeed(const RotorSensor_T * p_sensor)
// {
//     RotorSensor_CaptureSpeed(p_sensor);
//     return _RotorSensor_GetElectricalSpeed(p_sensor->P_STATE);
// }

/******************************************************************************/
/*!
    Query Functions
*/
/******************************************************************************/
static inline Angle_T * RotorSensor_GetAngleState(const RotorSensor_T * p_sensor) { return &p_sensor->P_STATE->AngleSpeed; }

 /* Angle Feedback. Shared E-Cycle edge detect, User output */
static inline angle16_t RotorSensor_GetElectricalAngle(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->AngleSpeed.Angle; }

// ElectricalDeltaAngle, DigitalSpeed [Degrees Per ControlCycle]
/* Electrical Speed, ElectricalDeltaAngle. < 32768 by SpeedRated */
static inline angle16_t RotorSensor_GetElectricalSpeed(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->AngleSpeed.Speed; }

/* fract16 [-32767:32767]*2 Speed Feedback Variable. -/+ => virtual CW/CCW */
static inline int32_t RotorSensor_GetSpeed_Fract16(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->AngleSpeed.Speed_Fract16; }
// static inline int32_t RotorSensor_GetSpeed_Fract16(const RotorSensor_T * p_sensor) { return speed_fract16_of_angle16(p_sensor->P_STATE->ElectricalSpeed_DegPerCycle, p_sensor->P_STATE->P_STATE->Config.ElSpeedRated_DegPerCycle); }

static inline int RotorSensor_GetDirection(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->Direction; }
// static inline int RotorSensor_GetDirection(const RotorSensor_T * p_sensor) { return math_sign(RotorSensor_GetElectricalSpeed(p_sensor)); }

static inline angle16_t RotorSensor_GetMechanicalAngle(const RotorSensor_T * p_sensor) { return p_sensor->P_STATE->MechanicalAngle; }


/******************************************************************************/
/*!
    Angle Speed Wrap
*/
/******************************************************************************/
// move to sensor/angle
// static inline void _Motor_SetFeedforwardDelta(Motor_State_T * p_motor, angle16_t elSpeed_degPerCycle)
// {
//     p_motor->SensorState.ElectricalAngle += elSpeed_degPerCycle;
//     p_motor->SensorState.ElectricalSpeed_DegPerCycle = elSpeed_degPerCycle;
//     p_motor->SensorState.MechanicalAngle += (elSpeed_degPerCycle / p_motor->Config.PolePairs);
//     p_motor->SensorState.Speed_Fract16 = speed_fract16_of_angle16_rpm(MOTOR_CONTROL_FREQ, Motor_GetSpeedRatedRef_ERpm(p_motor), elSpeed_degPerCycle);
// }

// static inline void Motor_SetElAngleFeedforward(Motor_State_T * p_motor, angle16_t angle)
// {
//     _Motor_SetFeedforwardDelta(p_motor, angle - p_motor->SensorState.ElectricalAngle);
// }

// static inline void Motor_SetElSpeedFeedforward(Motor_State_T * p_motor, angle16_t elSpeed_degPerCycle)
// {
//     _Motor_SetFeedforwardDelta(p_motor, elSpeed_degPerCycle);
// }

// static inline void Motor_SetMechAngleFeedforward(Motor_State_T * p_motor, angle16_t angle)
// {
//     Motor_SetElAngleFeedforward(p_motor, angle * p_motor->Config.PolePairs);
// }

