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
    @file     .h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_ANALOG_H
#define MOTOR_ANALOG_H

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
    #define _MOTOR_ANALOG_CHANNEL_COUNT_SIN_COS (2U)
#else
    #define _MOTOR_ANALOG_CHANNEL_COUNT_SIN_COS (0U)
#endif

#define MOTOR_ANALOG_CHANNEL_COUNT     (7U + _MOTOR_ANALOG_CHANNEL_COUNT_SIN_COS)

/*!
    @brief Virtual channel identifiers, index into arrays containing Analog channel
*/
typedef enum MotorAnalog_Channel_Tag
{
    MOTOR_ANALOG_CHANNEL_VA,
    MOTOR_ANALOG_CHANNEL_VB,
    MOTOR_ANALOG_CHANNEL_VC,
    MOTOR_ANALOG_CHANNEL_IA,
    MOTOR_ANALOG_CHANNEL_IB,
    MOTOR_ANALOG_CHANNEL_IC,
    MOTOR_ANALOG_CHANNEL_HEAT,    /* Temperature */
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
    MOTOR_ANALOG_CHANNEL_SIN,
    MOTOR_ANALOG_CHANNEL_COS,
#endif
}
MotorAnalog_Channel_T;

typedef union MotorAnalog_Results_Tag
{
    struct
    {
        analog_result_t Va_Adcu;
        analog_result_t Vb_Adcu;
        analog_result_t Vc_Adcu;
        analog_result_t Ia_Adcu;
        analog_result_t Ib_Adcu;
        analog_result_t Ic_Adcu;
        analog_result_t Heat_Adcu;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        analog_result_t Sin_Adcu;
        analog_result_t Cos_Adcu;
#endif
    };
    analog_result_t Channels[MOTOR_ANALOG_CHANNEL_COUNT];
}
MotorAnalog_Results_T;

typedef struct MotorAnalog_Conversions_Tag
{
    union
    {
        struct
        {
            const AnalogN_Conversion_T CONVERSION_VA;
            const AnalogN_Conversion_T CONVERSION_VB;
            const AnalogN_Conversion_T CONVERSION_VC;
            const AnalogN_Conversion_T CONVERSION_IA;
            const AnalogN_Conversion_T CONVERSION_IB;
            const AnalogN_Conversion_T CONVERSION_IC;
            const AnalogN_Conversion_T CONVERSION_HEAT;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
            const AnalogN_Conversion_T CONVERSION_SIN;
            const AnalogN_Conversion_T CONVERSION_COS;
#endif
        };
        AnalogN_Conversion_T CONVERSIONS[MOTOR_ANALOG_CHANNEL_COUNT];
    };

    const AnalogN_AdcFlags_T ADCS_GROUP_V;
    const AnalogN_AdcFlags_T ADCS_GROUP_I;
    const AnalogN_AdcFlags_T ADCS_GROUP_PWM;
}
MotorAnalog_Conversions_T;

#endif
