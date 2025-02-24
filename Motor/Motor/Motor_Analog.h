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
    @file   MotorAnalog.h
    @author FireSourcery
    @brief  Analog functions, require dependencies to map call back
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_ANALOG_CONVERSION_H
#define MOTOR_ANALOG_CONVERSION_H

#include "Motor_FOC.h"
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
#include "Motor_SixStep.h"
#endif
#include "Motor_Debug.h"
#include "Motor.h"

static inline uint16_t Motor_Analog_GetAdcu(const Motor_T * p_motor, MotorAnalog_Channel_T localChannel) { return p_motor->CONST.ANALOG_CONVERSIONS.CONVERSIONS[localChannel].P_STATE->Result; }
static inline uint8_t Motor_Analog_GetAdcu_Msb8(const Motor_T * p_motor, MotorAnalog_Channel_T localChannel) { return Motor_Analog_GetAdcu(p_motor, localChannel) >> (GLOBAL_ANALOG.ADC_BITS - 8U); }

static inline uint16_t Motor_Analog_GetVa(const Motor_T * p_motor) { return p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VA.P_STATE->Result; }
static inline uint16_t Motor_Analog_GetVb(const Motor_T * p_motor) { return p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VB.P_STATE->Result; }
static inline uint16_t Motor_Analog_GetVc(const Motor_T * p_motor) { return p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_VC.P_STATE->Result; }
static inline uint16_t Motor_Analog_GetIa(const Motor_T * p_motor) { return p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IA.P_STATE->Result; }
static inline uint16_t Motor_Analog_GetIb(const Motor_T * p_motor) { return p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IB.P_STATE->Result; }
static inline uint16_t Motor_Analog_GetIc(const Motor_T * p_motor) { return p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_IC.P_STATE->Result; }
static inline uint16_t Motor_Analog_GetHeat(const Motor_T * p_motor) { return p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_HEAT.P_STATE->Result; }

/******************************************************************************/
/*!
    @brief  Adc Capture
*/
/******************************************************************************/
static inline void Motor_Analog_OnCompleteVa(Motor_T * p_motor, uint16_t adcu) { (void)adcu; p_motor->VFlags.A = 1U; }
static inline void Motor_Analog_OnCompleteVb(Motor_T * p_motor, uint16_t adcu) { (void)adcu; p_motor->VFlags.B = 1U; }
static inline void Motor_Analog_OnCompleteVc(Motor_T * p_motor, uint16_t adcu) { (void)adcu; p_motor->VFlags.C = 1U; }
static inline void Motor_Analog_OnCompleteIa(Motor_T * p_motor, uint16_t adcu) { (void)adcu; p_motor->IFlags.A = 1U; }
static inline void Motor_Analog_OnCompleteIb(Motor_T * p_motor, uint16_t adcu) { (void)adcu; p_motor->IFlags.B = 1U; }
static inline void Motor_Analog_OnCompleteIc(Motor_T * p_motor, uint16_t adcu) { (void)adcu; p_motor->IFlags.C = 1U; }

// static inline void Motor_Analog_CaptureVa(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureVa, 0U /* Motor_SixStep_CaptureVBemfA */, adcu); }
// static inline void Motor_Analog_CaptureVb(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureVb, 0U /* Motor_SixStep_CaptureVBemfB */, adcu); }
// static inline void Motor_Analog_CaptureVc(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureVc, 0U /* Motor_SixStep_CaptureVBemfC */, adcu); }
// static inline void Motor_Analog_CaptureIa(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureIa, 0U /* Motor_SixStep_CaptureIa */, adcu); }
// static inline void Motor_Analog_CaptureIb(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureIb, 0U /* Motor_SixStep_CaptureIb */, adcu); }
// static inline void Motor_Analog_CaptureIc(Motor_T * p_motor, uint16_t adcu) { Motor_SetCommutationModeUInt16(p_motor, Motor_FOC_CaptureIc, 0U /* Motor_SixStep_CaptureIc */, adcu); }

#define MOTOR_ANALOG_CONVERSION_VA_INIT(ChannelBase, VaHost, VaPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_VA), (Analog_Setter_T)Motor_Analog_OnCompleteVa, p_Motor, VaHost, VaPin)
#define MOTOR_ANALOG_CONVERSION_VB_INIT(ChannelBase, VbHost, VbPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_VB), (Analog_Setter_T)Motor_Analog_OnCompleteVb, p_Motor, VbHost, VbPin)
#define MOTOR_ANALOG_CONVERSION_VC_INIT(ChannelBase, VcHost, VcPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_VC), (Analog_Setter_T)Motor_Analog_OnCompleteVc, p_Motor, VcHost, VcPin)
#define MOTOR_ANALOG_CONVERSION_IA_INIT(ChannelBase, IaHost, IaPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_IA), (Analog_Setter_T)Motor_Analog_OnCompleteIa, p_Motor, IaHost, IaPin)
#define MOTOR_ANALOG_CONVERSION_IB_INIT(ChannelBase, IbHost, IbPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_IB), (Analog_Setter_T)Motor_Analog_OnCompleteIb, p_Motor, IbHost, IbPin)
#define MOTOR_ANALOG_CONVERSION_IC_INIT(ChannelBase, IcHost, IcPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_IC), (Analog_Setter_T)Motor_Analog_OnCompleteIc, p_Motor, IcHost, IcPin)
#define MOTOR_ANALOG_CONVERSION_HEAT_INIT(ChannelBase, HeatHost, HeatPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_HEAT), NULL, p_Motor, HeatHost, HeatPin)
#define MOTOR_ANALOG_CONVERSION_SIN_INIT(ChannelBase, SinHost, SinPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_SIN), NULL, p_Motor, SinHost, SinPin)
#define MOTOR_ANALOG_CONVERSION_COS_INIT(ChannelBase, CosHost, CosPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_COS), NULL, p_Motor, CosHost, CosPin)

/* For Channel Map */
#define MOTOR_ANALOG_CONVERSION_OF_CHANNEL(p_Motor, LocalChannel) (&((p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[LocalChannel]))
#define MOTOR_ANALOG_CONVERSION_VA(p_Motor) MOTOR_ANALOG_CONVERSION_OF_CHANNEL(p_Motor, MOTOR_ANALOG_CHANNEL_VA)
#define MOTOR_ANALOG_CONVERSION_VB(p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_VB])
#define MOTOR_ANALOG_CONVERSION_VC(p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_VC])
#define MOTOR_ANALOG_CONVERSION_IA(p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_IA])
#define MOTOR_ANALOG_CONVERSION_IB(p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_IB])
#define MOTOR_ANALOG_CONVERSION_IC(p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_IC])
#define MOTOR_ANALOG_CONVERSION_HEAT(p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_HEAT])
#define MOTOR_ANALOG_CONVERSION_SIN(p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_SIN])
#define MOTOR_ANALOG_CONVERSION_COS(p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_COS])


extern void Motor_Analog_MarkVabc(Motor_T * p_motor);
extern void Motor_Analog_MarkIabc(Motor_T * p_motor);
extern void Motor_Analog_StartCalibration(Motor_T * p_motor);
extern bool Motor_Analog_ProcCalibration(Motor_T * p_motor);

extern void Motor_Analog_Calibrate(Motor_T * p_motor);

#endif

// #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
// _MOTOR_ANALOG_CHANNEL_ENTRIES_SIN_COS(ChannelBase, p_Motor) \
// [ChannelBase + MOTOR_ANALOG_CHANNEL_SIN] = MOTOR_ANALOG_CHANNEL_SIN_ENTRY(ChannelBase, p_Motor), \
// [ChannelBase + MOTOR_ANALOG_CHANNEL_COS] = MOTOR_ANALOG_CHANNEL_COS_ENTRY(ChannelBase, p_Motor),
// #else
// #define _MOTOR_ANALOG_CHANNEL_ENTRIES_SIN_COS(ChannelBase, p_Motor)
// #endif

// #define _MOTOR_ANALOG_CHANNEL_ENTRIES(ChannelBase, p_Motor) \
//     [ChannelBase + MOTOR_ANALOG_CHANNEL_VA] = MOTOR_ANALOG_CONVERSION_OF_CHANNEL(p_Motor, MOTOR_ANALOG_CHANNEL_VA), \
//     [ChannelBase + MOTOR_ANALOG_CHANNEL_VB] = MOTOR_ANALOG_CHANNEL_VB_ENTRY(ChannelBase, p_Motor), \
//     [ChannelBase + MOTOR_ANALOG_CHANNEL_VC] = MOTOR_ANALOG_CHANNEL_VC_ENTRY(ChannelBase, p_Motor), \
//     [ChannelBase + MOTOR_ANALOG_CHANNEL_IA] = MOTOR_ANALOG_CHANNEL_IA_ENTRY(ChannelBase, p_Motor), \
//     [ChannelBase + MOTOR_ANALOG_CHANNEL_IB] = MOTOR_ANALOG_CHANNEL_IB_ENTRY(ChannelBase, p_Motor), \
//     [ChannelBase + MOTOR_ANALOG_CHANNEL_IC] = MOTOR_ANALOG_CHANNEL_IC_ENTRY(ChannelBase, p_Motor), \
//     [ChannelBase + MOTOR_ANALOG_CHANNEL_HEAT] = MOTOR_ANALOG_CHANNEL_HEAT_ENTRY(ChannelBase, p_Motor), \
//     _MOTOR_ANALOG_CHANNEL_ENTRIES_SIN_COS(ChannelBase, p_Motor)

