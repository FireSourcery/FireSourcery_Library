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
    @file   Motor_Drive.h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "Motor.h"

#include <stdint.h>
#include <stdbool.h>



/*
    Q1(fwd, motor)  = 0b11
    Q2(fwd, gen)    = 0b10
    Q3(rev, motor)  = 0b00
    Q4(rev, gen)    = 0b01
*/
typedef struct
{
    uint8_t Motoring : 1;
    uint8_t Forward  : 1;
}
Motor_QuadrantSign_T;
// typedef union Motor_Quadrant
// {
//     struct { uint8_t Forward : 1; uint8_t Motoring : 1; }  ;
//     struct { uint8_t SpeedNeg : 1; uint8_t TorqueNeg : 1; }  ;
//     uint8_t Bits : 2;
// }
// Motor_Quadrant_T;

static inline Motor_QuadrantSign_T Motor_QuadrantSign(int16_t speed, int16_t torque) { return (Motor_QuadrantSign_T) { .Forward = (speed >= 0), .Motoring = (torque * speed >= 0) }; }
// static inline Motor_Axis_T Motor_FromQuadrant(Motor_Axis_T m, Motor_QuadrantSign_T q)
// {
//     int16_t speed = q.Forward ? (int16_t)m.Speed : -(int16_t)m.Speed;
//     int16_t torque = ((!q.Forward) ^ (!q.Motoring)) ? -(int16_t)m.Torque : (int16_t)m.Torque;
//     return (Motor_Axis_T) { speed, torque };
// }

typedef struct { int16_t Speed; int16_t Torque; } Motor_Axis_T;  /* fract16 */
// typedef struct { int16_t Speed; int16_t Torque; } Motor_Operating_T;
static inline int32_t _Motor_InstantPower(Motor_Axis_T feedback) { return (int32_t)feedback.Speed * feedback.Torque; }
static inline int32_t _Motor_IsMotoring(Motor_Axis_T feedback) { return _Motor_InstantPower(feedback) > 0; }


static inline int32_t Motor_QuadrantSignCmd(Motor_QuadrantSign_T sign, uint16_t cmd) {}


/* mask sign bit, motoring 0b01, plugging 0b01 */
typedef enum Motor_QuadrantId
{
    MOTOR_OPERATING_REVERSE_PLUGGING = -3,    /* Q4: +Vq, +Iq, -Speed - Vq opposes back-EMF */
    MOTOR_OPERATING_REVERSE_REGEN    = -2,    /* Q4: -Vq, +Iq, -Speed - VBemf < Vq < 0 */
    MOTOR_OPERATING_REVERSE_MOTORING = -1,    /* Q3: -Vq, -Iq, -Speed */
    MOTOR_OPERATING_IDLE             = 0,
    MOTOR_OPERATING_FORWARD_MOTORING = 1,    /* Q1: +Vq, +Iq, +Speed */
    MOTOR_OPERATING_FORWARD_REGEN    = 2,    /* Q2: +Vq, -Iq, +Speed - VBemf > Vq > 0 */
    MOTOR_OPERATING_FORWARD_PLUGGING = 3,    /* Q2: -Vq, -Iq, +Speed - Vq opposes back-EMF */
}
Motor_QuadrantId_T;


static inline Motor_QuadrantId_T Motor_QuadrantId(int16_t speed, fract16_t iq, fract16_t vq)
{
    if (speed == 0) { return MOTOR_OPERATING_IDLE; }

    int32_t is_generating = ((int32_t)iq * speed < 0);     /* 1 if Iq opposes speed */
    int32_t is_plugging = ((int32_t)vq * speed < 0);     /* 1 if Vq opposes speed */

    /* state magnitude: 1=Motoring, 2=Regen, 3=Plugging */
    /* sign encodes direction. bits [0:1] encode torque */
    return (Motor_QuadrantId_T)(math_sign(speed) * (1 + is_generating + is_plugging));
}


// sign_t Motor_Direction(Motor_QuadrantId_T quadrant) { return (quadrant > MOTOR_OPERATING_IDLE) - (quadrant < MOTOR_OPERATING_IDLE); }
// sign_t Motor_Mech(Motor_QuadrantId_T quadrant)  /* sign(Iq*speed):  */
// sign_t Motor_Power(Motor_QuadrantId_T quadrant)  /* sign(Vq*Iq):  */


// static inline bool Motor_IsForward(Motor_QuadrantId_T s) { return (s > MOTOR_OPERATING_IDLE); }
// static inline bool Motor_IsReverse(Motor_QuadrantId_T s) { return (s < MOTOR_OPERATING_IDLE); }
// static inline bool Motor_IsActive(Motor_QuadrantId_T s) { return (s != MOTOR_OPERATING_IDLE); }
// static inline bool Motor_IsMotoring(Motor_QuadrantId_T s)  { return abs(s) == 1; }
// static inline bool Motor_IsGenerating(Motor_QuadrantId_T s){ return abs(s) >= 2; }
// static inline bool Motor_IsPlugging(Motor_QuadrantId_T s)  { return abs(s) == 3; }

 typedef struct
{
    uint16_t Cmd        : 14; /* Abs value [0:16383] */
    uint16_t Generating : 1; /*  */
    uint16_t Reverse    : 1; /* User */
}
Motor_QuadrantCmd_T;

typedef struct
{
    Motor_FeedbackMode_T FeedbackMode;
    Motor_QuadrantCmd_T Cmd;
}
Motor_DriveCmd_T;

static inline int16_t Motor_QuadrantCmd_AsUser(Motor_QuadrantCmd_T cmd) { return (cmd.Reverse ? -cmd.Cmd : cmd.Cmd); }
/*   + aligned with rotation, - opposing (regen/brake) */
static inline int16_t Motor_QuadrantCmd_AsMotoring(Motor_QuadrantCmd_T cmd, Motor_Direction_T motor)
{
    return (cmd.Generating ? -cmd.Cmd : cmd.Cmd);
}

  /*    case Motoring:    return v * Motor_GetRunDirection(p);
        case Generati:  return -v * Motor_GetRunDirection(p); */

/* value << 1 */
// static inline int16_t Motor_QuadrantCmd_AsMotoring(Motor_QuadrantCmd_T cmd) { return (cmd.Reverse ^ cmd.Generating) ? -(int16_t)cmd.Cmd : (int16_t)cmd.Cmd; }

// typedef enum Motor_DriveDirection
// {
//     MOTOR_DRIVE_GENERATING = -1,
//     MOTOR_DRIVE_NULL = 0,
//     MOTOR_DRIVE_MOTORING = 1,
// }
// Motor_DriveDirection_T;


// void Motor_ApplyDriveCmd(const Motor_T * p, Motor_DriveCmd_T cmd)
// {
//     Motor_QuadrantSign_T runtime = Motor_QuadrantSign(Motor_GetSpeed(p), Motor_GetIq(p));
//     int16_t signed_cmd = Motor_QuadrantSign_AsMotoring(runtime, cmd.Cmd.Cmd);   /* or AsSpeed, per FeedbackMode */
//     Motor_ApplyFeedbackCmd(p, cmd.FeedbackMode, signed_cmd);
// }