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
#include "Analog/MotorAnalog.h"

#include "Motor.h"
#include "Motor_Debug.h"

#include "Math/FOC.h"


/******************************************************************************/
/*
    +/- Sign indicates absolute direction, CW/CCW. NOT along/against direction selected.
    Positive is virtual CCW.
    B and Beta are virtual CCW of A/Alpha.
    Iq sign is relative to rotor direction, NOT Vq direction.

    CCW +Vq +Iq => Forward Motoring Q1
    CCW +Vq -Iq => Forward Regen Q4
    CCW -Vq -Iq => Forward Plugging

    CW -Vq -Iq => Reverse Motoring Q2
    CW -Vq +Iq => Reverse Regen Q3
    CW +Vq +Iq => Reverse Plugging
*/
/******************************************************************************/


/******************************************************************************/
/*!
*/
/******************************************************************************/
// static inline void _Motor_FOC_ProcIFeedback(FOC_T * p_foc, PID_T * p_pidId, PID_T * p_pidIq)
// {
//     p_foc->Vq = PID_ProcPI(p_pidIq, p_foc->Iq, p_foc->ReqQ);
//     p_foc->Vd = PID_ProcPI(p_pidId, p_foc->Id, p_foc->ReqD);
// }

static inline void _Motor_FOC_WriteDuty(const Phase_T * p_phase, const FOC_T * p_foc)
{
    Phase_WriteDuty_Fract16(p_phase, FOC_GetDutyA(p_foc), FOC_GetDutyB(p_foc), FOC_GetDutyC(p_foc));
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

static inline bool Motor_FOC_IsMotoring(const Motor_State_T * p_motor) { return FOC_IsMotoring(&p_motor->Foc); }
static inline bool Motor_FOC_IsGenerating(const Motor_State_T * p_motor) { return FOC_IsGenerating(&p_motor->Foc); }

/* I limit in [Direction] selected */
static inline uint16_t Motor_FOC_GetILimit(const Motor_State_T * p_motor) { return (Motor_FOC_IsMotoring(p_motor) ? p_motor->ILimitMotoring_Fract16 : p_motor->ILimitGenerating_Fract16); }
// static inline bool Motor_FOC_IsILimitReached(const Motor_State_T * p_motor) { return (FOC_GetIMagnitude(&p_motor->Foc) > Motor_FOC_GetILimit(p_motor)); }
static inline bool Motor_FOC_IsILimitReached(const Motor_State_T * p_motor) { return (abs(FOC_GetIq(&p_motor->Foc)) > Motor_FOC_GetILimit(p_motor)); }



/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
// extern void Motor_FOC_WriteDuty(const Motor_T * p_motor);
// extern void Motor_FOC_ActivateOutput(const Motor_T * p_motor);
// extern void Motor_FOC_ActivateOutputZero(const Motor_T * p_motor);

extern void Motor_FOC_ProcAngleControl(Motor_State_T * p_motor);
extern void Motor_FOC_ProcCaptureAngleVBemf(Motor_State_T * p_motor);
extern void Motor_FOC_ProcAngleFeedforward(Motor_State_T * p_motor, angle16_t angle, fract16_t dReq, fract16_t qReq);

extern void Motor_FOC_ProcAngleFeedforwardV(Motor_State_T * p_motor, angle16_t angle, fract16_t vd, fract16_t vq);
extern void Motor_FOC_ClearFeedbackState(Motor_State_T * p_motor);
extern void Motor_FOC_MatchFeedbackState(Motor_State_T * p_motor);

extern void Motor_FOC_CaptureIabc(Motor_State_T * p_motor);

extern void Motor_FOC_StartAlignCmd(Motor_State_T * p_motor);
extern void Motor_FOC_ProcAlignCmd(Motor_State_T * p_motor);
extern void Motor_FOC_StartStartUpAlign(Motor_State_T * p_motor);
extern void Motor_FOC_ProcStartUpAlign(Motor_State_T * p_motor);
extern void Motor_FOC_StartAlignValidate(Motor_State_T * p_motor);

extern void Motor_FOC_StartOpenLoop(Motor_State_T * p_motor);
extern void Motor_FOC_ProcOpenLoop(Motor_State_T * p_motor);

extern void Motor_FOC_SetDirection(Motor_State_T * p_motor, Motor_Direction_T direction);
extern void Motor_FOC_SetDirection_Cast(Motor_State_T * p_motor, int direction);
extern void Motor_FOC_SetDirectionForward(Motor_State_T * p_motor);

#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
extern void Motor_ExternControl(Motor_State_T * p_motor);
#endif


#endif
