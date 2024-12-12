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

#include "Peripheral/Analog/Analog.h"
#include "Motor.h"

extern void Motor_Analog_CaptureVa(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Analog_CaptureVb(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Analog_CaptureVc(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Analog_CaptureIa(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Analog_CaptureIb(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Analog_CaptureIc(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Analog_CaptureHeat(Motor_T * p_motor, uint16_t adcu);

#define MOTOR_ANALOG_CONVERSION_VA_INIT(ChannelBase, VaHost, VaPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_VA), (Analog_Setter_T)Motor_Analog_CaptureVa, p_Motor, VaHost, VaPin)
#define MOTOR_ANALOG_CONVERSION_VB_INIT(ChannelBase, VbHost, VbPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_VB), (Analog_Setter_T)Motor_Analog_CaptureVb, p_Motor, VbHost, VbPin)
#define MOTOR_ANALOG_CONVERSION_VC_INIT(ChannelBase, VcHost, VcPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_VC), (Analog_Setter_T)Motor_Analog_CaptureVc, p_Motor, VcHost, VcPin)
#define MOTOR_ANALOG_CONVERSION_IA_INIT(ChannelBase, IaHost, IaPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_IA), (Analog_Setter_T)Motor_Analog_CaptureIa, p_Motor, IaHost, IaPin)
#define MOTOR_ANALOG_CONVERSION_IB_INIT(ChannelBase, IbHost, IbPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_IB), (Analog_Setter_T)Motor_Analog_CaptureIb, p_Motor, IbHost, IbPin)
#define MOTOR_ANALOG_CONVERSION_IC_INIT(ChannelBase, IcHost, IcPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_IC), (Analog_Setter_T)Motor_Analog_CaptureIc, p_Motor, IcHost, IcPin)
#define MOTOR_ANALOG_CONVERSION_HEAT_INIT(ChannelBase, HeatHost, HeatPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_HEAT), (Analog_Setter_T)Motor_Analog_CaptureHeat, p_Motor, HeatHost, HeatPin)
#define MOTOR_ANALOG_CONVERSION_SIN_INIT(ChannelBase, SinHost, SinPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_SIN), NULL, p_Motor, SinHost, SinPin)
#define MOTOR_ANALOG_CONVERSION_COS_INIT(ChannelBase, CosHost, CosPin, p_Motor) ANALOG_CONVERSION_INIT((ChannelBase + MOTOR_ANALOG_CHANNEL_COS), NULL, p_Motor, CosHost, CosPin)

/* For Channel Map */
#define MOTOR_ANALOG_CHANNEL_VA_ENTRY(ChannelBase, p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_VA])
#define MOTOR_ANALOG_CHANNEL_VB_ENTRY(ChannelBase, p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_VB])
#define MOTOR_ANALOG_CHANNEL_VC_ENTRY(ChannelBase, p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_VC])
#define MOTOR_ANALOG_CHANNEL_IA_ENTRY(ChannelBase, p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_IA])
#define MOTOR_ANALOG_CHANNEL_IB_ENTRY(ChannelBase, p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_IB])
#define MOTOR_ANALOG_CHANNEL_IC_ENTRY(ChannelBase, p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_IC])
#define MOTOR_ANALOG_CHANNEL_HEAT_ENTRY(ChannelBase, p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_HEAT])
#define MOTOR_ANALOG_CHANNEL_SIN_ENTRY(ChannelBase, p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_SIN])
#define MOTOR_ANALOG_CHANNEL_COS_ENTRY(ChannelBase, p_Motor) (&(p_Motor)->CONST.ANALOG_CONVERSIONS.CONVERSIONS[MOTOR_ANALOG_CHANNEL_COS])


#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
_MOTOR_ANALOG_CHANNEL_ENTRIES_SIN_COS(ChannelBase, p_Motor) \
    [ChannelBase + MOTOR_ANALOG_CHANNEL_SIN] = MOTOR_ANALOG_CHANNEL_SIN_ENTRY(ChannelBase, p_Motor), \
    [ChannelBase + MOTOR_ANALOG_CHANNEL_COS] = MOTOR_ANALOG_CHANNEL_COS_ENTRY(ChannelBase, p_Motor),
#else
#define _MOTOR_ANALOG_CHANNEL_ENTRIES_SIN_COS(ChannelBase, p_Motor)
#endif

#define _MOTOR_ANALOG_CHANNEL_ENTRIES(ChannelBase, p_Motor) \
    [ChannelBase + MOTOR_ANALOG_CHANNEL_VA] = MOTOR_ANALOG_CHANNEL_VA_ENTRY(ChannelBase, p_Motor), \
    [ChannelBase + MOTOR_ANALOG_CHANNEL_VB] = MOTOR_ANALOG_CHANNEL_VB_ENTRY(ChannelBase, p_Motor), \
    [ChannelBase + MOTOR_ANALOG_CHANNEL_VC] = MOTOR_ANALOG_CHANNEL_VC_ENTRY(ChannelBase, p_Motor), \
    [ChannelBase + MOTOR_ANALOG_CHANNEL_IA] = MOTOR_ANALOG_CHANNEL_IA_ENTRY(ChannelBase, p_Motor), \
    [ChannelBase + MOTOR_ANALOG_CHANNEL_IB] = MOTOR_ANALOG_CHANNEL_IB_ENTRY(ChannelBase, p_Motor), \
    [ChannelBase + MOTOR_ANALOG_CHANNEL_IC] = MOTOR_ANALOG_CHANNEL_IC_ENTRY(ChannelBase, p_Motor), \
    [ChannelBase + MOTOR_ANALOG_CHANNEL_HEAT] = MOTOR_ANALOG_CHANNEL_HEAT_ENTRY(ChannelBase, p_Motor), \
    _MOTOR_ANALOG_CHANNEL_ENTRIES_SIN_COS(ChannelBase, p_Motor)



// typedef enum Motor_Analog_Select
// {
//     MOTOR_ANALOG_SELECT_FOC_I_ABC,
//     MOTOR_ANALOG_SELECT_FOC_V_ABC,
// }
// Motor_Analog_Select_T;

// #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
// #define _MOTOR_ANALOG_CONVERSIONS_SIN_COS(SinPin, SinHost, CosPin, CosHost, p_Motor)                      \
//     .CONVERSION_SIN = ANALOG_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_SIN,   0U, p_Motor, SinHost, SinPin), \
//     .CONVERSION_COS = ANALOG_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_COS,   0U, p_Motor, CosHost, CosPin),
// #else
// #define _MOTOR_ANALOG_CONVERSIONS_SIN_COS(SinPin, SinHost, CosPin, CosHost, p_Motor)
// #endif


// #define MOTOR_ANALOG_CONVERSIONS_INIT(VaPin, VaHost, VbPin, VbHost, VcPin, VcHost, IaPin, IaHost, IbPin, IbHost, IcPin, IcHost, HeatPin, HeatHost, SinPin, SinHost, CosPin, CosHost, ChannelBase, p_Motor)       \
// {                                                                                                                                                                                                   \
//     .CONVERSION_VA      = ANALOG_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_VA,   (Analog_Setter_T)Motor_Analog_CaptureVa,   p_Motor,  VaHost,   VaPin),    \
//     .CONVERSION_VB      = ANALOG_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_VB,   (Analog_Setter_T)Motor_Analog_CaptureVb,   p_Motor,  VbHost,   VbPin),    \
//     .CONVERSION_VC      = ANALOG_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_VC,   (Analog_Setter_T)Motor_Analog_CaptureVc,   p_Motor,  VcHost,   VcPin),    \
//     .CONVERSION_IA      = ANALOG_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_IA,   (Analog_Setter_T)Motor_Analog_CaptureIa,   p_Motor,  IaHost,   IaPin),    \
//     .CONVERSION_IB      = ANALOG_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_IB,   (Analog_Setter_T)Motor_Analog_CaptureIb,   p_Motor,  IbHost,   IbPin),    \
//     .CONVERSION_IC      = ANALOG_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_IC,   (Analog_Setter_T)Motor_Analog_CaptureIc,   p_Motor,  IcHost,   IcPin),    \
//     .CONVERSION_HEAT    = ANALOG_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_HEAT, (Analog_Setter_T)Motor_Analog_CaptureHeat, p_Motor,  HeatHost, HeatPin),  \
//     _MOTOR_ANALOG_CONVERSIONS_SIN_COS(SinPin, SinHost, CosPin, CosHost, p_Motor)                                                                       \
// }

extern void Motor_Analog_MarkVabc(Motor_T * p_motor);
extern void Motor_Analog_MarkIabc(Motor_T * p_motor);
extern void Motor_Analog_StartCalibration(Motor_T * p_motor);
extern bool Motor_Analog_ProcCalibration(Motor_T * p_motor);



#endif
