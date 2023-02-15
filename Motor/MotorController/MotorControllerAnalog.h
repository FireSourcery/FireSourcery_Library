/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
#ifndef MOT_ANALOG_H
#define MOT_ANALOG_H

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    #define _MOT_ANALOG_CHANNEL_COUNT_HEAT_MOSFETS_TOP_BOT (1U)
#else
    #define _MOT_ANALOG_CHANNEL_COUNT_HEAT_MOSFETS_TOP_BOT (0U)
#endif

#define MOT_ANALOG_CHANNEL_COUNT (8U + _MOT_ANALOG_CHANNEL_COUNT_HEAT_MOSFETS_TOP_BOT)

/*!
    @brief Virtual channel identifiers, index into arrays containing Analog channel
*/
typedef enum MotAnalog_Channel_Tag
{
    MOT_ANALOG_CHANNEL_VSOURCE,             /* V Source, V Battery */
    MOT_ANALOG_CHANNEL_VACC,                /* V Accessories, ~12V */
    MOT_ANALOG_CHANNEL_VSENSE,                /* V Sensors, ~5V */
    MOT_ANALOG_CHANNEL_HEAT_PCB,
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_TOP = MOT_ANALOG_CHANNEL_HEAT_MOSFETS,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_BOT,
#else
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS,
#endif
    MOT_ANALOG_CHANNEL_THROTTLE,
    MOT_ANALOG_CHANNEL_BRAKE,
}
MotAnalog_Channel_T;

typedef union MotAnalog_Results_Tag
{
    struct
    {
        analog_result_t VSource_Adcu;
        analog_result_t VAcc_Adcu;
        analog_result_t VSense_Adcu;
        analog_result_t HeatPcb_Adcu;
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
        analog_result_t HeatMosfetsTop_Adcu;
        analog_result_t HeatMosfetsBot_Adcu;
#else
        analog_result_t HeatMosfets_Adcu;
#endif
        analog_result_t Throttle_Adcu;
        analog_result_t Brake_Adcu;
    };
    analog_result_t Channels[MOT_ANALOG_CHANNEL_COUNT];
}
MotAnalog_Results_T;

typedef struct MotAnalog_Conversions_Tag
{
    union
    {
        struct
        {
            const AnalogN_Conversion_T CONVERSION_VSOURCE;
            const AnalogN_Conversion_T CONVERSION_VACC;
            const AnalogN_Conversion_T CONVERSION_VSENSE;
            const AnalogN_Conversion_T CONVERSION_HEAT_PCB;
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
            const AnalogN_Conversion_T CONVERSION_HEAT_MOSFETS_TOP;
            const AnalogN_Conversion_T CONVERSION_HEAT_MOSFETS_BOT;
#else
            const AnalogN_Conversion_T CONVERSION_HEAT_MOSFETS;
#endif
            const AnalogN_Conversion_T CONVERSION_THROTTLE;
            const AnalogN_Conversion_T CONVERSION_BRAKE;
        };
        AnalogN_Conversion_T CONVERSIONS[MOT_ANALOG_CHANNEL_COUNT];
    };
    const AnalogN_AdcFlags_T ADCS_GROUP_USER;
    const AnalogN_AdcFlags_T ADCS_GROUP_V;
    const AnalogN_AdcFlags_T ADCS_GROUP_HEAT;
}
MotAnalog_Conversions_T;

#endif
