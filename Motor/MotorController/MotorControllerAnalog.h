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
    @file   MotorControllerAnalog.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOT_ANALOG_H
#define MOT_ANALOG_H

#include "Peripheral/Analog/Analog.h"
#include "Motor/Motor/MotorAnalog.h"

#ifndef MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT
    #error "MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT must be defined"
#endif

#define MOT_ANALOG_CHANNEL_COUNT (7U + MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT)

/*!
    @brief Local channel identifiers, index into arrays
*/
typedef enum MotAnalog_Channel
{
    MOT_ANALOG_CHANNEL_VSOURCE,         /* V Source, V Battery */
    MOT_ANALOG_CHANNEL_VSENSE,          /* V Sensors, ~5V */
    MOT_ANALOG_CHANNEL_VACCS,           /* V Accessories, ~12V */

    MOT_ANALOG_CHANNEL_HEAT_PCB,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS,
    _MOT_ANALOG_CHANNEL_HEAT_MOSFETS_END = MOT_ANALOG_CHANNEL_HEAT_MOSFETS + MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT - 1U,

    MOT_ANALOG_CHANNEL_THROTTLE,
    MOT_ANALOG_CHANNEL_BRAKE,
    _MOT_ANALOG_CHANNEL_END = MOT_ANALOG_CHANNEL_COUNT,
}
MotAnalog_Channel_T;

typedef enum MotAnalog_Channel_HeatMosfets
{
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_0 = MOT_ANALOG_CHANNEL_HEAT_MOSFETS,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_1,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_2,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_3,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_TOP = MOT_ANALOG_CHANNEL_HEAT_MOSFETS_0,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_BOT = MOT_ANALOG_CHANNEL_HEAT_MOSFETS_1,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_Q1 = MOT_ANALOG_CHANNEL_HEAT_MOSFETS_0,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_Q2 = MOT_ANALOG_CHANNEL_HEAT_MOSFETS_1,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_Q3 = MOT_ANALOG_CHANNEL_HEAT_MOSFETS_2,
    MOT_ANALOG_CHANNEL_HEAT_MOSFETS_Q4 = MOT_ANALOG_CHANNEL_HEAT_MOSFETS_3,
}
MotAnalog_Channel_HeatMosfets_T;

// typedef union MotAnalog_Results
// {
//     struct
//     {
//         analog_result_t VSource_Adcu;
//         analog_result_t VSense_Adcu;
//         analog_result_t VAccs_Adcu;
//         analog_result_t HeatPcb_Adcu;
//         analog_result_t HeatMosfetsResults_Adcu[MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT];
//         analog_result_t Throttle_Adcu;
//         analog_result_t Brake_Adcu;
//     };
//     analog_result_t Channels[MOT_ANALOG_CHANNEL_COUNT];
// }
// MotAnalog_Results_T;


#endif
