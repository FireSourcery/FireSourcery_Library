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
    Map to Motor Analog Conversions
    Convert current from ADCU to fract16
    where 32767 is fully saturated current sensor
*/
/******************************************************************************/
static inline void Motor_FOC_CaptureIa(Motor_T * p_motor, uint16_t adcu)
{
    fract16_t iPhase = ((int32_t)Linear_ADC_Fract16(&p_motor->UnitsIa, adcu) + FOC_GetIa(&p_motor->Foc)) / 2;
    FOC_SetIa(&p_motor->Foc, iPhase);
}

static inline void Motor_FOC_CaptureIb(Motor_T * p_motor, uint16_t adcu)
{
    fract16_t iPhase = ((int32_t)Linear_ADC_Fract16(&p_motor->UnitsIb, adcu) + FOC_GetIb(&p_motor->Foc)) / 2;
    FOC_SetIb(&p_motor->Foc, iPhase);
}

static inline void Motor_FOC_CaptureIc(Motor_T * p_motor, uint16_t adcu)
{
    fract16_t iPhase = ((int32_t)Linear_ADC_Fract16(&p_motor->UnitsIc, adcu) + FOC_GetIc(&p_motor->Foc)) / 2;
    FOC_SetIc(&p_motor->Foc, iPhase);
}

static inline void Motor_FOC_CaptureVa(Motor_T * p_motor, uint16_t adcu)
{
    ufract16_t vPhase = ((int32_t)Linear_ADC_Fract16(&p_motor->UnitsVabc, adcu) + FOC_GetVBemfA(&p_motor->Foc)) / 2;
    FOC_SetVBemfA(&p_motor->Foc, vPhase);
}

static inline void Motor_FOC_CaptureVb(Motor_T * p_motor, uint16_t adcu)
{
    ufract16_t vPhase = ((int32_t)Linear_ADC_Fract16(&p_motor->UnitsVabc, adcu) + FOC_GetVBemfB(&p_motor->Foc)) / 2;
    FOC_SetVBemfB(&p_motor->Foc, vPhase);
}

static inline void Motor_FOC_CaptureVc(Motor_T * p_motor, uint16_t adcu)
{
    ufract16_t vPhase = ((int32_t)Linear_ADC_Fract16(&p_motor->UnitsVabc, adcu) + FOC_GetVBemfC(&p_motor->Foc)) / 2;
    FOC_SetVBemfC(&p_motor->Foc, vPhase);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline int32_t Motor_FOC_GetIPhase_Fract16(const Motor_T * p_motor)            { return FOC_GetIPhase(&p_motor->Foc); }
static inline int32_t Motor_FOC_GetVPhase_Fract16(const Motor_T * p_motor)            { return FOC_GetVPhase(&p_motor->Foc); }

/* return int32 for function pointer casting compatibility */
static inline int32_t Motor_FOC_GetIPhase_UFract16(const Motor_T * p_motor)            { return FOC_GetIMagnitude(&p_motor->Foc); }
static inline int32_t Motor_FOC_GetVPhase_UFract16(const Motor_T * p_motor)            { return FOC_GetVMagnitude(&p_motor->Foc); }
static inline int32_t Motor_FOC_GetElectricalPower_UFract16(const Motor_T * p_motor)   { return FOC_GetPower(&p_motor->Foc); }

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

extern void Motor_FOC_ActivateOutputZero(Motor_T * p_motor);
extern void Motor_FOC_ActivateOutput(Motor_T * p_motor);
extern void Motor_FOC_ActivateAngle(Motor_T * p_motor, angle16_t angle, fract16_t vq, fract16_t vd);
extern void Motor_FOC_ClearFeedbackState(Motor_T * p_motor);
extern void Motor_FOC_MatchFeedbackState(Motor_T * p_motor);

extern void Motor_FOC_StartAlign(Motor_T * p_motor);
extern void Motor_FOC_ProcAlign(Motor_T * p_motor);
extern void Motor_FOC_StartAlignValidate(Motor_T * p_motor);
extern void Motor_FOC_StartOpenLoop(Motor_T * p_motor);
extern void Motor_FOC_ProcOpenLoop(Motor_T * p_motor);

extern void Motor_FOC_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_FOC_SetDirectionCw(Motor_T * p_motor);
extern void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_FOC_SetDirection_Cast(Motor_T * p_motor, uint8_t direction);
extern void Motor_FOC_SetDirectionForward(Motor_T * p_motor);

#ifdef CONFIG_MOTOR_EXTERN_CONTROL_ENABLE
extern void Motor_ExternControl(Motor_T * p_motor);
#endif


#endif
