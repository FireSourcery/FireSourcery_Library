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

*/
/******************************************************************************/
#ifndef MOT_ANALOG_CONVERSION_H
#define MOT_ANALOG_CONVERSION_H

#include "MotorController.h"
#include "MotorControllerAnalog.h"

#include "Peripheral/Analog/Analog.h"
#include "Motor/Motor/Analog/MotorAnalog.h"



// static inline uint16_t MotorController_Analog_GetVSource(const MotorController_T * p_const)    { return Analog_Channel_GetResult(&p_const->MOT_V_MONITOR_CONVERSIONS.CONVERSION_VSOURCE); }
// static inline uint16_t MotorController_Analog_GetVSense(const MotorController_T * p_const)     { return Analog_Channel_GetResult(&p_const->MOT_V_MONITOR_CONVERSIONS.CONVERSION_VSENSE); }
// static inline uint16_t MotorController_Analog_GetVAccs(const MotorController_T * p_const)      { return Analog_Channel_GetResult(&p_const->MOT_V_MONITOR_CONVERSIONS.CONVERSION_VACCS); }
// static inline uint16_t MotorController_Analog_GetHeatPcb(const MotorController_T * p_const)                     { return Analog_Channel_GetResult(&p_const->MOT_HEAT_MONITOR_CONVERSIONS.CONVERSION_HEAT_PCB); }
// static inline uint16_t MotorController_Analog_GetHeatMosfets(const MotorController_T * p_const, uint8_t index)  { return Analog_Channel_GetResult(&p_const->MOT_HEAT_MONITOR_CONVERSIONS.P_HEAT_MOSFETS_CONVERSIONS[index]); }
// static inline uint16_t MotorController_Analog_GetThrottle(const MotorController_T * p_const)   { return Analog_Channel_GetResult(&p_const->MOT_ANALOG_USER_CONVERSIONS.CONVERSION_THROTTLE); }
// static inline uint16_t MotorController_Analog_GetBrake(const MotorController_T * p_const)      { return Analog_Channel_GetResult(&p_const->MOT_ANALOG_USER_CONVERSIONS.CONVERSION_BRAKE); }

// #define MOT_ANALOG_CONVERSION_INIT(LocalChannel, AdcId, AdcPin, p_Mot) ANALOG_CONVERSION_INIT((MOT_ANALOG_CHANNEL_BASE_GENERAL + LocalChannel), 0U, p_Mot, AdcId, AdcPin)

// #define MOT_ANALOG_CONVERSION_VSOURCE_INIT(VSourceHost, VSourcePin, p_Mot)  MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VSOURCE, VSourceHost, VSourcePin, p_Mot)
// #define MOT_ANALOG_CONVERSION_VSENSE_INIT(VSenseHost, VSensePin, p_Mot)     MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VSENSE, VSenseHost, VSensePin, p_Mot)
// #define MOT_ANALOG_CONVERSION_VACCS_INIT(VAccsHost, VAccsPin, p_Mot)        MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_VACCS, VAccsHost, VAccsPin, p_Mot)

// #define MOT_ANALOG_CONVERSION_HEAT_PCB_INIT(HeatPcbHost, HeatPcbPin, p_Mot)             MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_PCB, HeatPcbHost, HeatPcbPin, p_Mot)
// #define MOT_ANALOG_CONVERSION_HEAT_MOSFETS_INIT_0(HeatMosHost, HeatMosPin, p_Mot)       MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_MOSFETS, HeatMosHost, HeatMosPin, p_Mot)
// #define MOT_ANALOG_CONVERSION_HEAT_MOSFETS_INIT_N(Id, HeatMosHost, HeatMosPin, p_Mot)   MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_HEAT_MOSFETS + Id, HeatMosHost, HeatMosPin, p_Mot)

// #define MOT_ANALOG_CONVERSIONS_INIT_USER_THROTTLE(ThrottleHost, ThrottlePin, p_Mot)     MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_THROTTLE, ThrottleHost, ThrottlePin, p_Mot)
// #define MOT_ANALOG_CONVERSIONS_INIT_USER_BRAKE(BrakeHost, BrakePin, p_Mot)              MOT_ANALOG_CONVERSION_INIT(MOT_ANALOG_CHANNEL_BRAKE, BrakeHost, BrakePin, p_Mot)

bool MotorController_Analog_ProcCalibrate(const MotorController_T * p_const);
void MotorController_Analog_StartCalibrate(const MotorController_T * p_const);

#endif


