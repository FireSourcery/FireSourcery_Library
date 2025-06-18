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
    @file   Motor_Analog.h
    @author FireSourcery
    @brief  ADC Conversions
            - callbacks on Motor_State_T
*/
/******************************************************************************/
#include "Peripheral/Analog/Analog.h"

/*
    [Phase_AnalogConversions]
*/

/* Part of Motor */
typedef const struct Motor Motor_T;
typedef struct Motor_State Motor_State_T;

typedef const struct Motor_Analog
{
    const Analog_Conversion_T CONVERSION_VA;
    const Analog_Conversion_T CONVERSION_VB;
    const Analog_Conversion_T CONVERSION_VC;
    const Analog_Conversion_T CONVERSION_IA;
    const Analog_Conversion_T CONVERSION_IB;
    const Analog_Conversion_T CONVERSION_IC;
    // MotorAnalog_Config_T CONFIG;
}
Motor_Analog_T;

/*
    Pass Struct and Index
*/
#define MOTOR_ANALOG_INIT(AdcVa, IndexVa, AdcVb, IndexVb, AdcVc, IndexVc, AdcIa, IndexIa, AdcIb, IndexIb, AdcIc, IndexIc) \
{ \
    .CONVERSION_VA = ANALOG_CONVERSION_INIT_FROM(AdcVa, IndexVa), \
    .CONVERSION_VB = ANALOG_CONVERSION_INIT_FROM(AdcVb, IndexVb), \
    .CONVERSION_VC = ANALOG_CONVERSION_INIT_FROM(AdcVc, IndexVc), \
    .CONVERSION_IA = ANALOG_CONVERSION_INIT_FROM(AdcIa, IndexIa), \
    .CONVERSION_IB = ANALOG_CONVERSION_INIT_FROM(AdcIb, IndexIb), \
    .CONVERSION_IC = ANALOG_CONVERSION_INIT_FROM(AdcIc, IndexIc), \
}

/*
    Analog_Context_T
*/
#define MOTOR_ANALOG_CONTEXT_VA_INIT(p_MotorState) ANALOG_CONTEXT_INIT(p_MotorState, (Analog_Capture_T)Motor_Analog_CaptureVa)
#define MOTOR_ANALOG_CONTEXT_VB_INIT(p_MotorState) ANALOG_CONTEXT_INIT(p_MotorState, (Analog_Capture_T)Motor_Analog_CaptureVb)
#define MOTOR_ANALOG_CONTEXT_VC_INIT(p_MotorState) ANALOG_CONTEXT_INIT(p_MotorState, (Analog_Capture_T)Motor_Analog_CaptureVc)
#define MOTOR_ANALOG_CONTEXT_IA_INIT(p_MotorState) ANALOG_CONTEXT_INIT(p_MotorState, (Analog_Capture_T)Motor_Analog_CaptureIa)
#define MOTOR_ANALOG_CONTEXT_IB_INIT(p_MotorState) ANALOG_CONTEXT_INIT(p_MotorState, (Analog_Capture_T)Motor_Analog_CaptureIb)
#define MOTOR_ANALOG_CONTEXT_IC_INIT(p_MotorState) ANALOG_CONTEXT_INIT(p_MotorState, (Analog_Capture_T)Motor_Analog_CaptureIc)


/*
    Analog_ConversionChannel_T
*/
#define MOTOR_ANALOG_CONVERSION_VA_INIT(p_Motor, ChannelVa) ANALOG_CONVERSION_CHANNEL_INIT(ChannelVa, MOTOR_ANALOG_CONTEXT_VA_INIT(p_Motor))
#define MOTOR_ANALOG_CONVERSION_VB_INIT(p_Motor, ChannelVb) ANALOG_CONVERSION_CHANNEL_INIT(ChannelVb, MOTOR_ANALOG_CONTEXT_VB_INIT(p_Motor))
#define MOTOR_ANALOG_CONVERSION_VC_INIT(p_Motor, ChannelVc) ANALOG_CONVERSION_CHANNEL_INIT(ChannelVc, MOTOR_ANALOG_CONTEXT_VC_INIT(p_Motor))
#define MOTOR_ANALOG_CONVERSION_IA_INIT(p_Motor, ChannelIa) ANALOG_CONVERSION_CHANNEL_INIT(ChannelIa, MOTOR_ANALOG_CONTEXT_IA_INIT(p_Motor))
#define MOTOR_ANALOG_CONVERSION_IB_INIT(p_Motor, ChannelIb) ANALOG_CONVERSION_CHANNEL_INIT(ChannelIb, MOTOR_ANALOG_CONTEXT_IB_INIT(p_Motor))
#define MOTOR_ANALOG_CONVERSION_IC_INIT(p_Motor, ChannelIc) ANALOG_CONVERSION_CHANNEL_INIT(ChannelIc, MOTOR_ANALOG_CONTEXT_IC_INIT(p_Motor))

// typedef struct Motor_Analog_Config
// {
//     uint16_t IaZeroRef_Adcu;
//     uint16_t IbZeroRef_Adcu;
//     uint16_t IcZeroRef_Adcu;
// }
// Motor_Analog_Config_T;

// /*!
//     @brief Virtual channel identifiers
// */
// typedef enum Motor_Analog_Channel
// {
//     MOTOR_ANALOG_CHANNEL_VA,
//     MOTOR_ANALOG_CHANNEL_VB,
//     MOTOR_ANALOG_CHANNEL_VC,
//     MOTOR_ANALOG_CHANNEL_IA,
//     MOTOR_ANALOG_CHANNEL_IB,
//     MOTOR_ANALOG_CHANNEL_IC,
//     MOTOR_ANALOG_CHANNEL_HEAT,
// }
// Motor_Analog_Channel_T;

/******************************************************************************/
/*!
    @brief Adc Capture
*/
/******************************************************************************/
extern void Motor_Analog_CaptureVa(Motor_State_T * p_motor, adc_result_t adcu);
extern void Motor_Analog_CaptureVb(Motor_State_T * p_motor, adc_result_t adcu);
extern void Motor_Analog_CaptureVc(Motor_State_T * p_motor, adc_result_t adcu);
extern void Motor_Analog_CaptureIa(Motor_State_T * p_motor, adc_result_t adcu);
extern void Motor_Analog_CaptureIb(Motor_State_T * p_motor, adc_result_t adcu);
extern void Motor_Analog_CaptureIc(Motor_State_T * p_motor, adc_result_t adcu);
// extern void Motor_Analog_CaptureHeat(Motor_State_T * p_motor, adc_result_t adcu);

/*
*/
extern void Motor_Analog_MarkVabc(const Motor_T * p_motorConst);
extern void Motor_Analog_MarkIabc(const Motor_T * p_motorConst);
// extern void Motor_Analog_MarkHeat(const Motor_T * p_motorConst);

extern void Motor_Analog_Calibrate(const Motor_T * p_motorConst);


// #define MOTOR_ANALOG_INIT(p_Motor, ChannelVa, ChannelVb, ChannelVc, ChannelIa, ChannelIb, ChannelIc, ChannelHeat) \
// { \
//     .CONVERSION_VA = MOTOR_ANALOG_CONVERSION_VA_INIT(p_Motor, ChannelVa), \
//     .CONVERSION_VB = MOTOR_ANALOG_CONVERSION_VB_INIT(p_Motor, ChannelVb), \
//     .CONVERSION_VC = MOTOR_ANALOG_CONVERSION_VC_INIT(p_Motor, ChannelVc), \
//     .CONVERSION_IA = MOTOR_ANALOG_CONVERSION_IA_INIT(p_Motor, ChannelIa), \
//     .CONVERSION_IB = MOTOR_ANALOG_CONVERSION_IB_INIT(p_Motor, ChannelIb), \
//     .CONVERSION_IC = MOTOR_ANALOG_CONVERSION_IC_INIT(p_Motor, ChannelIc), \
//     .CONVERSION_HEAT = MOTOR_ANALOG_CONVERSION_HEAT_INIT(p_Motor, ChannelHeat), \
// }
