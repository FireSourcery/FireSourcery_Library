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
    @file   Motor_FOC.h
    @author FireSourcery
    @brief  Motor FOC submodule. FOC control functions.

*/
/******************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

// #include "Sensor/RotorSensor.h"

#include "Motor.h"
#include "Motor_Debug.h"

#include "Math/FOC.h"


/******************************************************************************/
/*
    +/- Sign indicates absolute virtual direction, CW/CCW. NOT along/against direction selected.
    A->B virtual CCW as Positive.
    Iq sign is rotation direction, NOT relative to Vq direction.

    CCW +Vq +Iq => Forward Motoring Q1
    CCW +Vq -Iq => Forward Regen Q2, VBemf > Vq > 0
    CCW -Vq -Iq => Forward Plugging Q2

    CW -Vq -Iq => Reverse Motoring Q3
    CW -Vq +Iq => Reverse Regen Q4, VBemf < Vq < 0
    CW +Vq +Iq => Reverse Plugging Q4
*/
/******************************************************************************/


/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline void _Motor_FOC_WriteDuty(const Phase_T * p_phase, const FOC_T * p_foc)
{
    Phase_WriteDuty_Fract16(p_phase, FOC_DutyA(p_foc), FOC_DutyB(p_foc), FOC_DutyC(p_foc));
}

static inline void Motor_FOC_WriteDuty(const Motor_T * p_motor) { _Motor_FOC_WriteDuty(&p_motor->PHASE, &p_motor->P_MOTOR_STATE->Foc); }

/******************************************************************************/
/*!

*/
/******************************************************************************/
/* return int for function pointer casting */
static inline motor_value_t Motor_FOC_GetIPhase_Fract16(const Motor_State_T * p_motor)             { return FOC_GetIPhase(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetVPhase_Fract16(const Motor_State_T * p_motor)             { return FOC_GetVPhase(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetIPhase_UFract16(const Motor_State_T * p_motor)            { return FOC_GetIMagnitude(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetVPhase_UFract16(const Motor_State_T * p_motor)            { return FOC_GetVMagnitude(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetElectricalPower_UFract16(const Motor_State_T * p_motor)   { return FOC_GetPower(&p_motor->Foc); }

/* vq is expected to be the same sign as p_motor->Direction */

/* I limit in [Direction] selected */
static inline uint16_t Motor_FOC_GetILimit(const Motor_State_T * p_motor) { return (FOC_IsMotoringCmd(&p_motor->Foc) ? p_motor->ILimitMotoring_Fract16 : p_motor->ILimitGenerating_Fract16); }

static inline bool Motor_FOC_IsIqLimitReached(const Motor_State_T * p_motor) { return math_is_in_range(FOC_Iq(&p_motor->Foc), _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor)); }
static inline bool Motor_FOC_IsILimitReached(const Motor_State_T * p_motor) { return (FOC_GetIMagnitude(&p_motor->Foc) > Motor_FOC_GetILimit(p_motor)); }
// static inline bool _Motor_FOC_IsIqLimitReached(const Motor_State_T * p_motor) { return (abs(FOC_Iq(&p_motor->Foc)) > Motor_FOC_GetILimit(p_motor)); }


static inline bool Motor_FOC_IsMotoring(const Motor_State_T * p_motor) { return (FOC_Iq(&p_motor->Foc) * (int32_t)Motor_GetSpeedFeedback(p_motor) > 0); }
static inline bool Motor_FOC_IsGenerating(const Motor_State_T * p_motor) { return (FOC_Iq(&p_motor->Foc) * (int32_t)Motor_GetSpeedFeedback(p_motor) < 0); }

/*
    Plugging: applied Vq opposes back-EMF direction (speed sign)
    Speed sign gives true rotor direction regardless of Vq/Iq
*/
/* Vq and Speed have opposite signs - applied voltage opposes rotor back-EMF */
static inline bool Motor_FOC_IsPlugging(const Motor_State_T * p_motor) { return (FOC_Vq(&p_motor->Foc) * (int32_t)Motor_GetSpeedFeedback(p_motor) < 0); }
/*
    Regen: Iq opposes Vq direction (generating), but Vq aligns with speed
*/
/* Generating  AND Vq aligns with speed - true regeneration */
static inline bool Motor_FOC_IsRegen(const Motor_State_T * p_motor) { return (FOC_Vq(&p_motor->Foc) * FOC_Iq(&p_motor->Foc) < 0) && !Motor_FOC_IsPlugging(p_motor); }
// static inline bool Motor_FOC_IsRegen(const Motor_State_T * p_motor) { return (Motor_GetSpeedFeedback(p_motor) * FOC_Iq(&p_motor->Foc) < 0) && !Motor_FOC_IsPlugging(p_motor); }

/* mask sign bit, motoring 0b01, plugging 0b01 */
typedef enum Motor_FOC_OperatingState
{
    MOTOR_FOC_OPERATING_REVERSE_PLUGGING = -3,    /* Q4: +Vq, +Iq, -Speed - Vq opposes back-EMF */
    MOTOR_FOC_OPERATING_REVERSE_REGEN    = -2,    /* Q4: -Vq, +Iq, -Speed - VBemf < Vq < 0 */
    MOTOR_FOC_OPERATING_REVERSE_MOTORING = -1,    /* Q3: -Vq, -Iq, -Speed */
    MOTOR_FOC_OPERATING_IDLE             = 0,
    MOTOR_FOC_OPERATING_FORWARD_MOTORING = 1,    /* Q1: +Vq, +Iq, +Speed */
    MOTOR_FOC_OPERATING_FORWARD_REGEN    = 2,    /* Q2: +Vq, -Iq, +Speed - VBemf > Vq > 0 */
    MOTOR_FOC_OPERATING_FORWARD_PLUGGING = 3,    /* Q2: -Vq, -Iq, +Speed - Vq opposes back-EMF */
}
Motor_FOC_OperatingState_T;

static inline Motor_FOC_OperatingState_T Motor_FOC_OperatingState(int16_t speed, fract16_t vq, fract16_t iq)
{
    if (speed == 0) { return MOTOR_FOC_OPERATING_IDLE; }

    int32_t is_generating = ((int32_t)iq * speed < 0);     /* 1 if Iq opposes speed */
    int32_t is_plugging = ((int32_t)vq * speed < 0);     /* 1 if Vq opposes speed */

    /* state magnitude: 1=Motoring, 2=Regen, 3=Plugging */
    /* sign encodes direction. bits [0:1] encode torque */
    return (Motor_FOC_OperatingState_T)(math_sign(speed) * (1 + is_generating + is_plugging));
}

// static inline bool Motor_FOC_StateIsForward(Motor_FOC_OperatingState_T s) { return (s > MOTOR_FOC_OPERATING_IDLE); }
// static inline bool Motor_FOC_StateIsReverse(Motor_FOC_OperatingState_T s) { return (s < MOTOR_FOC_OPERATING_IDLE); }
// static inline bool Motor_FOC_StateIsActive(Motor_FOC_OperatingState_T s) { return (s != MOTOR_FOC_OPERATING_IDLE); }
// static inline bool Motor_FOC_StateIsMotoring(Motor_FOC_OperatingState_T s)  { return abs(s) == 1; }
// static inline bool Motor_FOC_StateIsGenerating(Motor_FOC_OperatingState_T s){ return abs(s) >= 2; }
// static inline bool Motor_FOC_StateIsPlugging(Motor_FOC_OperatingState_T s)  { return abs(s) == 3; }


/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_FOC_ProcAngleControl(Motor_State_T * p_motor);
extern void Motor_FOC_ProcCaptureAngleVBemf(Motor_State_T * p_motor);
extern void Motor_FOC_AngleControl(Motor_State_T * p_motor, angle16_t angle, fract16_t dReq, fract16_t qReq);
extern void Motor_FOC_ProcAngleFeedforwardV(Motor_State_T * p_motor, angle16_t angle, fract16_t vd, fract16_t vq);

void Motor_FOC_ProcTorqueReq(Motor_State_T * p_motor, fract16_t dReq, fract16_t qReq);

extern void Motor_FOC_ClearFeedbackState(Motor_State_T * p_motor);
extern void Motor_FOC_MatchIVState(Motor_State_T * p_motor);
extern void Motor_FOC_MatchFeedbackState(Motor_State_T * p_motor);

extern void Motor_FOC_StartAlignCmd(Motor_State_T * p_motor);
extern void Motor_FOC_ProcAlignCmd(Motor_State_T * p_motor);
extern void Motor_FOC_StartStartUpAlign(Motor_State_T * p_motor);
extern void Motor_FOC_ProcStartUpAlign(Motor_State_T * p_motor);
extern void Motor_FOC_StartAlignValidate(Motor_State_T * p_motor);

extern void Motor_FOC_StartOpenLoop(Motor_State_T * p_motor);
extern void Motor_FOC_ProcOpenLoop(Motor_State_T * p_motor);

extern void Motor_FOC_SetDirection(Motor_State_T * p_motor, Motor_Direction_T direction);
extern void Motor_FOC_SetDirectionForward(Motor_State_T * p_motor);

#ifdef MOTOR_EXTERN_CONTROL_ENABLE
extern void Motor_ExternControl(Motor_State_T * p_motor);
#endif


#endif
