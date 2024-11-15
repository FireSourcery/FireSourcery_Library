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

#include "Peripheral/Analog/AnalogN/AnalogN.h"
#include "Motor.h"

extern void Motor_Analog_CaptureVa(MotorPtr_T p_motor);
extern void Motor_Analog_CaptureVb(MotorPtr_T p_motor);
extern void Motor_Analog_CaptureVc(MotorPtr_T p_motor);
extern void Motor_Analog_CaptureIa(MotorPtr_T p_motor);
extern void Motor_Analog_CaptureIb(MotorPtr_T p_motor);
extern void Motor_Analog_CaptureIc(MotorPtr_T p_motor);

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
    #define _MOTOR_ANALOG_N_CONVERSION_SIN_COS(SinPin, SinHost, CosPin, CosHost, p_Hosts, p_Motor)                                                                  \
        .CONVERSION_SIN = ANALOG_N_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_SIN, 0U, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), SinPin, &(p_Hosts)[SinHost]),   \
        .CONVERSION_COS = ANALOG_N_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_COS, 0U, p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), CosPin, &(p_Hosts)[CosHost]),
#else
    #define _MOTOR_ANALOG_N_CONVERSION_SIN_COS(SinPin, SinHost, CosPin, CosHost, p_Hosts, p_Motor)
#endif

#define MOTOR_ANALOG_CONVERSIONS_INIT(VaPin, VaHost, VbPin, VbHost, VcPin, VcHost, IaPin, IaHost, IbPin, IbHost, IcPin, IcHost, HeatPin, HeatHost, SinPin, SinHost, CosPin, CosHost, p_Hosts, p_Motor)      \
{                                                                                                                                                                                                           \
    .CONVERSION_VA      = ANALOG_N_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_VA, (Analog_Callback_T)Motor_Analog_CaptureVa,  p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), VaPin,   &(p_Hosts)[VaHost]),    \
    .CONVERSION_VB      = ANALOG_N_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_VB, (Analog_Callback_T)Motor_Analog_CaptureVb,  p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), VbPin,   &(p_Hosts)[VbHost]),    \
    .CONVERSION_VC      = ANALOG_N_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_VC, (Analog_Callback_T)Motor_Analog_CaptureVc,  p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), VcPin,   &(p_Hosts)[VcHost]),    \
    .CONVERSION_IA      = ANALOG_N_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_IA, (Analog_Callback_T)Motor_Analog_CaptureIa,  p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), IaPin,   &(p_Hosts)[IaHost]),    \
    .CONVERSION_IB      = ANALOG_N_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_IB, (Analog_Callback_T)Motor_Analog_CaptureIb,  p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), IbPin,   &(p_Hosts)[IbHost]),    \
    .CONVERSION_IC      = ANALOG_N_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_IC, (Analog_Callback_T)Motor_Analog_CaptureIc,  p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), IcPin,   &(p_Hosts)[IcHost]),    \
    .CONVERSION_HEAT    = ANALOG_N_CONVERSION_INIT(MOTOR_ANALOG_CHANNEL_HEAT,   0U,                                     p_Motor, &((p_Motor)->AnalogResults.Channels[0U]), HeatPin, &(p_Hosts)[HeatHost]),  \
    _MOTOR_ANALOG_N_CONVERSION_SIN_COS(SinPin, SinHost, CosPin, CosHost, p_Hosts, p_Motor)                                                                                                                  \
    .ADCS_GROUP_I       = { .Flags = (uint8_t)((1U << IaHost) | (1U << IbHost) | (1U << IcHost) | (1U << SinHost) | (1U << CosHost)), },                                                                    \
    .ADCS_GROUP_V       = { .Flags = (uint8_t)((1U << VaHost) | (1U << VbHost) | (1U << VcHost) | (1U << SinHost) | (1U << CosHost)), },                                                                    \
    .ADCS_GROUP_PWM     = { .Flags = (uint8_t)((1U << VaHost) | (1U << VbHost) | (1U << VcHost) | (1U << IaHost) | (1U << IbHost) | (1U << IcHost)| (1U << SinHost) | (1U << CosHost)), },                  \
}

extern void Motor_Analog_EnqueueVabc(MotorPtr_T p_motor);
extern void Motor_Analog_EnqueueIabc(MotorPtr_T p_motor);
extern void Motor_Analog_StartCalibration(MotorPtr_T p_motor);
extern bool Motor_Analog_ProcCalibration(MotorPtr_T p_motor);

#endif
