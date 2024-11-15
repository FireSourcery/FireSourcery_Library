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

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    #define _MOTOR_ANALOG_N_CONVERSION_MOSFETS(TopPin, TopHost, BotPin, BotHost, p_Hosts, p_Mot)     \
        .CONVERSION_HEAT_MOSFETS_TOP     =    ANALOG_N_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_MOSFETS_TOP,     0U, p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), TopPin,     &(p_Hosts)[TopHost]),     \
        .CONVERSION_HEAT_MOSFETS_BOT     =    ANALOG_N_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_MOSFETS_BOT,     0U, p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), BotPin,     &(p_Hosts)[BotHost]),
#else
    #define _MOTOR_ANALOG_N_CONVERSION_MOSFETS(TopPin, TopHost, NullPin, NullHost, p_Hosts, p_Mot)     \
        .CONVERSION_HEAT_MOSFETS         =    ANALOG_N_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_MOSFETS,         0U, p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), TopPin,     &(p_Hosts)[TopHost]),
#endif

#define MOT_ANALOG_CONVERSIONS_INIT(VSourcePin, VSourceHost, VAccPin, VAccHost, VSensePin, VSenseHost, HeatPcbPin, HeatPcbHost, HeatMosTopPin, HeatMosTopHost, HeatMosBotPin, HeatMosBotHost, ThrottlePin, ThrottleHost, BrakePin, BrakeHost, p_Hosts, p_Mot) \
{                                                                                                                                                                                                               \
    .CONVERSION_VSOURCE     = ANALOG_N_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VSOURCE,              0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VSourcePin,      &(p_Hosts)[VSourceHost]),               \
    .CONVERSION_VSENSE      = ANALOG_N_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VSENSE,               0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VSensePin,       &(p_Hosts)[VSenseHost]),                \
    .CONVERSION_VACCS       = ANALOG_N_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VACCS,                0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), VAccPin,         &(p_Hosts)[VAccHost]),                  \
    .CONVERSION_HEAT_PCB    = ANALOG_N_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_PCB,             0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), HeatPcbPin,      &(p_Hosts)[HeatPcbHost]),               \
    _MOTOR_ANALOG_N_CONVERSION_MOSFETS(HeatMosTopPin, HeatMosTopHost, HeatMosBotPin, HeatMosBotHost, p_Hosts, p_Mot)                                                                                            \
    .CONVERSION_THROTTLE    = ANALOG_N_CONVERSION_INIT(MOT_ANALOG_CHANNEL_THROTTLE,             0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), ThrottlePin,     &(p_Hosts)[ThrottleHost]),              \
    .CONVERSION_BRAKE       = ANALOG_N_CONVERSION_INIT(MOT_ANALOG_CHANNEL_BRAKE,                0U,     p_Mot, &((p_Mot)->AnalogResults.Channels[0U]), BrakePin,        &(p_Hosts)[BrakeHost]),                 \
    .ADCS_GROUP_V           = { .Flags = (uint8_t)((1U << VAccHost) | (1U << VSenseHost)),                                  },                                                                                  \
    .ADCS_GROUP_HEAT        = { .Flags = (uint8_t)((1U << HeatPcbHost) | (1U << HeatMosTopHost) | (1U << HeatMosBotHost)),  },                                                                                  \
    .ADCS_GROUP_USER        = { .Flags = (uint8_t)((1U << ThrottleHost) | (1U << BrakeHost)),                               },                                                                                  \
}


// static inline uint16_t MotorController_Analog_GetHeatPcb(const MotorControllerPtr_T p_mc)        { return p_mc->AnalogResults.HeatPcb_Adcu; }
// static inline uint16_t MotorController_Analog_GetHeatMosfets(const MotorControllerPtr_T p_mc)    { return p_mc->AnalogResults.HeatMosfets_Adcu; }
// static inline uint16_t MotorController_User_GetHeat_Adcu(const MotorControllerPtr_T p_mc, uint8_t index)    {  return p_mc->AnalogResultsThermal[index]; }

// typedef enum MotVarId_Instance_ThermistorBoard
// {
//     MOT_VAR_ID_THERMISTOR_PCB,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_0,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_1,
// }
// MotVarId_Instance_ThermistorBoard_T;

// typedef enum MotVarId_Instance_VMonitor
// {
//     MOT_VAR_ID_VMONITOR_SOURCE,
//     MOT_VAR_ID_VMONITOR_SENSOR,
//     MOT_VAR_ID_VMONITOR_ACC,
// }
// MotVarId_Instance_VMonitor_T;

#endif
