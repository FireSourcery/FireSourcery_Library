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
    @file   .h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOT_ANALOG_CONVERSION_H
#define MOT_ANALOG_CONVERSION_H

#include "Peripheral/Analog/Analog.h"
#include "Motor/Motor/MotorAnalog.h"
#include "MotorControllerAnalog.h"

typedef enum
{
    MOT_ANALOG_CHANNEL_BASE_GENERAL = 0U,
    MOT_ANALOG_CHANNEL_BASE_MOTOR0 = MOT_ANALOG_CHANNEL_BASE_GENERAL + MOT_ANALOG_CHANNEL_COUNT,
    MOT_ANALOG_CHANNEL_BASE_MOTOR1 = MOT_ANALOG_CHANNEL_BASE_MOTOR0 + MOTOR_ANALOG_CHANNEL_COUNT,
    MOT_ANALOG_CHANNEL_BASE_MOTOR2 = MOT_ANALOG_CHANNEL_BASE_MOTOR1 + MOTOR_ANALOG_CHANNEL_COUNT,
    MOT_ANALOG_CHANNEL_BASE_MOTOR3 = MOT_ANALOG_CHANNEL_BASE_MOTOR2 + MOTOR_ANALOG_CHANNEL_COUNT,
}
MotorController_Analog_ChannelGlobal_T;


#define MOT_ANALOG_CONVERSION_INIT(LocalChannel, AdcId, AdcPin, p_Mot) ANALOG_CONVERSION_INIT((MOT_ANALOG_CHANNEL_BASE_GENERAL + LocalChannel), 0U, p_Mot, AdcId, AdcPin, .P_RESULT = &((p_Mot)->AnalogResults.Channels[LocalChannel]))

#define MOT_ANALOG_CONVERSION_VSOURCE_INIT(VSourceHost, VSourcePin, p_Mot)  MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VSOURCE, VSourceHost, VSourcePin, p_Mot)
#define MOT_ANALOG_CONVERSION_VSENSE_INIT(VSenseHost, VSensePin, p_Mot)     MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VSENSE, VSenseHost, VSensePin, p_Mot)
#define MOT_ANALOG_CONVERSION_VACCS_INIT(VAccsHost, VAccsPin, p_Mot)        MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VACCS,  VAccsHost, VAccsPin, p_Mot)

#define MOT_ANALOG_CONVERSION_HEAT_PCB_INIT(HeatPcbHost, HeatPcbPin, p_Mot) MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_PCB, HeatPcbHost, HeatPcbPin, p_Mot)
#define MOT_ANALOG_CONVERSION_HEAT_MOSFETS_INIT_0(HeatMosHost, HeatMosPin, p_Mot) MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_MOSFETS, HeatMosHost, HeatMosPin, p_Mot)
#define MOT_ANALOG_CONVERSION_HEAT_MOSFETS_INIT_N(Id, HeatMosHost, HeatMosPin, p_Mot) MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_MOSFETS + Id, HeatMosHost, HeatMosPin, p_Mot)

#define MOT_ANALOG_CONVERSIONS_INIT_USER_THROTTLE(ThrottleHost, ThrottlePin, p_Mot) MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_THROTTLE, ThrottleHost, ThrottlePin, p_Mot)
#define MOT_ANALOG_CONVERSIONS_INIT_USER_BRAKE(BrakeHost, BrakePin, p_Mot)          MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_BRAKE, BrakeHost, BrakePin, p_Mot)

// #define MOT_ANALOG_CONVERSIONS_INIT(VSourcePin, VSourceHost, VAccPin, VAccHost, VSensePin, VSenseHost, HeatPcbPin, HeatPcbHost, HeatMosTopPin, HeatMosTopHost, HeatMosBotPin, HeatMosBotHost, ThrottlePin, ThrottleHost, BrakePin, BrakeHost, p_Mot) \
// {                                                                                                                                                                                                               \
//     .CONVERSION_VSOURCE     = ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VSOURCE,              0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VSourcePin,      &(p_Hosts)[VSourceHost]),               \
//     .CONVERSION_VSENSE      = ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VSENSE,               0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VSensePin,       &(p_Hosts)[VSenseHost]),                \
//     .CONVERSION_VACCS       = ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VACCS,                0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VAccPin,         &(p_Hosts)[VAccHost]),                  \
//     .CONVERSION_HEAT_PCB    = ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_PCB,             0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), HeatPcbPin,      &(p_Hosts)[HeatPcbHost]),               \
//     _MOTOR_ANALOG_CONVERSION_MOSFETS(HeatMosTopPin, HeatMosTopHost, HeatMosBotPin, HeatMosBotHost, p_Hosts, p_Mot)                                                                                            \
//     .CONVERSION_THROTTLE    = ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_THROTTLE,             0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), ThrottlePin,     &(p_Hosts)[ThrottleHost]),              \
//     .CONVERSION_BRAKE       = ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_BRAKE,                0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), BrakePin,        &(p_Hosts)[BrakeHost]),                 \
//     .ADCS_GROUP_V           = { .Flags = (uint8_t)((1U << VAccHost) | (1U << VSenseHost)),                                  },                                                                                  \
//     .ADCS_GROUP_HEAT        = { .Flags = (uint8_t)((1U << HeatPcbHost) | (1U << HeatMosTopHost) | (1U << HeatMosBotHost)),  },                                                                                  \
//     .ADCS_GROUP_USER        = { .Flags = (uint8_t)((1U << ThrottleHost) | (1U << BrakeHost)),                               },                                                                                  \
// }


#endif
