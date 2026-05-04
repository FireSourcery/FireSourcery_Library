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
static inline void _Motor_FOC_WriteDuty(Phase_T * p_phase, const FOC_T * p_foc) { Phase_WriteDuty_Fract16(p_phase, FOC_DutyA(p_foc), FOC_DutyB(p_foc), FOC_DutyC(p_foc)); }
static inline void Motor_FOC_WriteDuty(Motor_T * p_motor) { _Motor_FOC_WriteDuty(&p_motor->PHASE, &p_motor->P_MOTOR->Foc); }

/******************************************************************************/
/*!

*/
/******************************************************************************/
/* return int for function pointer casting */
static inline motor_value_t Motor_FOC_GetIPhase_Fract16(const Motor_State_T * p_motor)             { return FOC_GetIPhase(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetVPhase_Fract16(const Motor_State_T * p_motor)             { return FOC_GetVPhase(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetIPhase_UFract16(const Motor_State_T * p_motor)            { return FOC_GetIMagnitude(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetVPhase_UFract16(const Motor_State_T * p_motor)            { return FOC_GetVMagnitude(&p_motor->Foc); }
static inline motor_value_t Motor_FOC_GetElectricalPower_UFract16(const Motor_State_T * p_motor)   { return FOC_GetActivePower(&p_motor->Foc); }

/*

*/
static inline bool Motor_FOC_IsMotoring(const Motor_State_T * p_motor) { return (FOC_Iq(&p_motor->Foc) * (int32_t)Motor_GetSpeedFeedback(p_motor) > 0); }
static inline bool Motor_FOC_IsGenerating(const Motor_State_T * p_motor) { return (FOC_Iq(&p_motor->Foc) * (int32_t)Motor_GetSpeedFeedback(p_motor) < 0); }
static inline bool _Motor_FOC_IsRegen(const Motor_State_T * p_motor) { return (FOC_Vq(&p_motor->Foc) * FOC_Iq(&p_motor->Foc) < 0); }

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
static inline bool Motor_FOC_IsRegen(const Motor_State_T * p_motor) { return (Motor_FOC_IsGenerating(p_motor) && !Motor_FOC_IsPlugging(p_motor)); }



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

extern void Motor_FOC_SetDirection(Motor_T * p_dev, Motor_Direction_T direction);
extern void Motor_FOC_SetDirectionForward(Motor_State_T * p_motor);

#ifdef MOTOR_EXTERN_CONTROL_ENABLE
extern void Motor_ExternControl(Motor_State_T * p_motor);
#endif


#endif
