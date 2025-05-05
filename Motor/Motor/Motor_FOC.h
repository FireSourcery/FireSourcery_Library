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
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "Transducer/MotorSensor/Motor_Sensor.h"

#include "Motor.h"
#include "Motor_Analog.h"
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
/*
    Handle Filter
*/
static inline void Motor_FOC_CaptureIa(Motor_T * p_motor) { FOC_SetIa(&p_motor->Foc, ((int32_t)Motor_Analog_GetIa_Fract16(p_motor) + FOC_GetIa(&p_motor->Foc)) / 2); }
static inline void Motor_FOC_CaptureIb(Motor_T * p_motor) { FOC_SetIb(&p_motor->Foc, ((int32_t)Motor_Analog_GetIb_Fract16(p_motor) + FOC_GetIb(&p_motor->Foc)) / 2); }
static inline void Motor_FOC_CaptureIc(Motor_T * p_motor) { FOC_SetIc(&p_motor->Foc, ((int32_t)Motor_Analog_GetIc_Fract16(p_motor) + FOC_GetIc(&p_motor->Foc)) / 2); }
static inline void Motor_FOC_CaptureVa(Motor_T * p_motor) { FOC_SetVBemfA(&p_motor->Foc, ((int32_t)Motor_Analog_GetVa_Fract16(p_motor) + FOC_GetVBemfA(&p_motor->Foc)) / 2); }
static inline void Motor_FOC_CaptureVb(Motor_T * p_motor) { FOC_SetVBemfB(&p_motor->Foc, ((int32_t)Motor_Analog_GetVb_Fract16(p_motor) + FOC_GetVBemfB(&p_motor->Foc)) / 2); }
static inline void Motor_FOC_CaptureVc(Motor_T * p_motor) { FOC_SetVBemfC(&p_motor->Foc, ((int32_t)Motor_Analog_GetVc_Fract16(p_motor) + FOC_GetVBemfC(&p_motor->Foc)) / 2); }


/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline motor_value_t Motor_FOC_GetIPhase_Fract16(const Motor_T * p_motor)            { return FOC_GetIPhase(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetVPhase_Fract16(const Motor_T * p_motor)            { return FOC_GetVPhase(&p_motor->Foc); }

/* return int32 for function pointer casting compatibility */
static inline motor_value_t Motor_FOC_GetIPhase_UFract16(const Motor_T * p_motor)            { return FOC_GetIMagnitude(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetVPhase_UFract16(const Motor_T * p_motor)            { return FOC_GetVMagnitude(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetElectricalPower_UFract16(const Motor_T * p_motor)   { return FOC_GetPower(&p_motor->Foc); }

static inline bool Motor_FOC_IsMotoring(const Motor_T * p_motor) { return FOC_IsMotoring(&p_motor->Foc); }
static inline bool Motor_FOC_IsGenerating(const Motor_T * p_motor) { return FOC_IsGenerating(&p_motor->Foc); }

static inline uint16_t Motor_FOC_GetILimit(const Motor_T * p_motor) { return (Motor_FOC_IsMotoring(p_motor) ? p_motor->ILimitMotoring_Fract16 : p_motor->ILimitGenerating_Fract16); }
// static inline bool Motor_FOC_IsILimitReached(const Motor_T * p_motor) { return (FOC_GetIMagnitude(&p_motor->Foc) > Motor_FOC_GetILimit(p_motor)); }
static inline bool Motor_FOC_IsILimitReached(const Motor_T * p_motor) { return (FOC_GetIq(&p_motor->Foc) > Motor_FOC_GetILimit(p_motor)); }

// static inline int16_t Motor_FOC_VReqILimit(const Motor_T * p_motor, int16_t req)
// {
//     return math_feedback_scalar(req, Motor_FOC_GetILimit(p_motor), math_abs(FOC_GetIq(&p_motor->Foc)));
//     //     uint16_t scalar = FRACT16_1_OVERSAT;
//     //     /* fract16_div always return positive < 1 */
//     //     if      (p_motor->Foc.Iq < _Motor_GetILimitCw(p_motor)) { scalar = fract16_div(_Motor_GetILimitCw(p_motor), p_motor->Foc.Iq); }
//     //     else if (p_motor->Foc.Iq > _Motor_GetILimitCcw(p_motor)) { scalar = fract16_div(_Motor_GetILimitCcw(p_motor), p_motor->Foc.Iq); }
//     //     return scalar;
// }



/******************************************************************************/
/*!

*/
/******************************************************************************/

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_FOC_ProcAngleControl(Motor_T * p_motor);
extern void Motor_FOC_ProcCaptureAngleVBemf(Motor_T * p_motor);
extern void Motor_FOC_ProcAngleFeedforward(Motor_T * p_motor, angle16_t angle, fract16_t dReq, fract16_t qReq);
extern void Motor_FOC_SetAngleFeedforward(Motor_T * p_motor, angle16_t angle, fract16_t dReq, fract16_t qReq);

extern void Motor_FOC_ActivateOutputZero(Motor_T * p_motor);
extern void Motor_FOC_ActivateOutput(Motor_T * p_motor);
extern void Motor_FOC_ActivateAngle(Motor_T * p_motor, angle16_t angle, fract16_t vd, fract16_t vq);
extern void Motor_FOC_ClearFeedbackState(Motor_T * p_motor);
extern void Motor_FOC_MatchFeedbackState(Motor_T * p_motor);

extern void Motor_FOC_CaptureIabc(Motor_T * p_motor);

extern void Motor_FOC_StartAlignCmd(Motor_T * p_motor);
extern void Motor_FOC_ProcAlignCmd(Motor_T * p_motor);
extern void Motor_FOC_StartStartUpAlign(Motor_T * p_motor);
extern void Motor_FOC_ProcStartUpAlign(Motor_T * p_motor);
extern void Motor_FOC_StartAlignValidate(Motor_T * p_motor);

extern void Motor_FOC_StartOpenLoop(Motor_T * p_motor);
extern void Motor_FOC_ProcOpenLoop(Motor_T * p_motor);

extern void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_FOC_SetDirection_Cast(Motor_T * p_motor, int direction);
extern void Motor_FOC_SetDirectionForward(Motor_T * p_motor);

#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
extern void Motor_ExternControl(Motor_T * p_motor);
#endif


#endif
