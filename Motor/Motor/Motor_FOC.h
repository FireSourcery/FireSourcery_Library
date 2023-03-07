/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery / The Firebrand Forge Inc

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
    @file     Motor_FOC.h
    @author FireSourcery
    @brief  Motor FOC submodule. FOC control functions.
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include "Motor.h"

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
    Convert current from ADCU to QFrac
*/
/******************************************************************************/
static inline void Motor_FOC_CaptureIa(Motor_T * p_motor)
{
    qfrac16_t iPhase = ((int32_t)Linear_ADC_CalcFracS16(&p_motor->UnitsIa, p_motor->AnalogResults.Ia_Adcu) + FOC_GetIa(&p_motor->Foc)) / 2;
    FOC_SetIa(&p_motor->Foc, iPhase);
}

static inline void Motor_FOC_CaptureIb(Motor_T * p_motor)
{
    qfrac16_t iPhase = ((int32_t)Linear_ADC_CalcFracS16(&p_motor->UnitsIb, p_motor->AnalogResults.Ib_Adcu) + FOC_GetIb(&p_motor->Foc)) / 2;
    FOC_SetIb(&p_motor->Foc, iPhase);
}

static inline void Motor_FOC_CaptureIc(Motor_T * p_motor)
{
    qfrac16_t iPhase = ((int32_t)Linear_ADC_CalcFracS16(&p_motor->UnitsIc, p_motor->AnalogResults.Ic_Adcu) + FOC_GetIc(&p_motor->Foc)) / 2;
    FOC_SetIc(&p_motor->Foc, iPhase);
}

static inline void Motor_FOC_CaptureVa(Motor_T * p_motor)
{
    qfrac16_t vPhase = ((int32_t)Linear_Voltage_CalcFracS16(&p_motor->UnitsVabc, p_motor->AnalogResults.Va_Adcu) + FOC_GetVBemfA(&p_motor->Foc)) / 2;
    FOC_SetVBemfA(&p_motor->Foc, vPhase);
}

static inline void Motor_FOC_CaptureVb(Motor_T * p_motor)
{
    qfrac16_t vPhase = ((int32_t)Linear_Voltage_CalcFracS16(&p_motor->UnitsVabc, p_motor->AnalogResults.Vb_Adcu) + FOC_GetVBemfB(&p_motor->Foc)) / 2;
    FOC_SetVBemfB(&p_motor->Foc, vPhase);
}

static inline void Motor_FOC_CaptureVc(Motor_T * p_motor)
{
    qfrac16_t vPhase = ((int32_t)Linear_Voltage_CalcFracS16(&p_motor->UnitsVabc, p_motor->AnalogResults.Vc_Adcu) + FOC_GetVBemfC(&p_motor->Foc)) / 2;
    FOC_SetVBemfC(&p_motor->Foc, vPhase);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline int32_t Motor_FOC_GetIPhase_FracS16(Motor_T * p_motor)               { return FOC_GetIPhase_Signed(&p_motor->Foc); }
static inline int32_t Motor_FOC_GetVPhase_FracS16(Motor_T * p_motor)               { return FOC_GetVPhase_Signed(&p_motor->Foc); }
/* return int32 for function pointer casting compatibility  */
/* FracS16Abs in Frac16 without int16 sign bit */
static inline int32_t Motor_FOC_GetIPhase_FracS16Abs(Motor_T * p_motor)            { return FOC_GetIPhase(&p_motor->Foc); }
static inline int32_t Motor_FOC_GetVPhase_FracS16Abs(Motor_T * p_motor)            { return FOC_GetVPhase(&p_motor->Foc); }
static inline int32_t Motor_FOC_GetElectricalPower_FracS16Abs(Motor_T * p_motor)   { return FOC_GetPower(&p_motor->Foc); }

/******************************************************************************/
/*!
    Check limits on set
*/
/******************************************************************************/
static inline bool Motor_FOC_CheckIOverThreshold(Motor_T * p_motor)
{
    return !math_isbound(FOC_GetIq(&p_motor->Foc), (int32_t)p_motor->ILimitCw_FracS16 * 7 / 8, (int32_t)p_motor->ILimitCcw_FracS16 * 7 / 8);
}

/* Clear State for SetFeedbackMatch */
static inline void Motor_FOC_ClearState(Motor_T * p_motor)
{
    FOC_ClearState(&p_motor->Foc);
}

/* From FreeWheel State, match to speed, overwrites VBemfClarke */
static inline void Motor_FOC_SetVSpeed(Motor_T * p_motor)
{
    FOC_SetVq(&p_motor->Foc, Linear_Function_FracS16(&p_motor->UnitsVSpeed, p_motor->Speed_FracS16));
    FOC_SetVd(&p_motor->Foc, 0);
}

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_FOC_ProcAngleControl(Motor_T * p_motor);
extern void Motor_FOC_ProcAngleVBemf(Motor_T * p_motor);

extern void Motor_FOC_StartAlign(Motor_T * p_motor);
extern void Motor_FOC_ProcAlign(Motor_T * p_motor);
extern void Motor_FOC_StartAlignValidate(Motor_T * p_motor);
extern void Motor_FOC_StartOpenLoop(Motor_T * p_motor);
extern void Motor_FOC_ProcOpenLoop(Motor_T * p_motor);

extern void Motor_FOC_ActivateOutput(Motor_T * p_motor);
extern void Motor_FOC_ActivateAngle(Motor_T * p_motor, qangle16_t angle, qfrac16_t vq, qfrac16_t vd);
extern void Motor_FOC_ProcFeedbackMatch(Motor_T * p_motor);

extern void Motor_FOC_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_FOC_SetDirectionCw(Motor_T * p_motor);
extern void Motor_FOC_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_FOC_SetDirectionForward(Motor_T * p_motor);

#endif
