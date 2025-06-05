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
    @file   MotVMonitor.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/

#include "Transducer/Voltage/VMonitor.h"
#include "Peripheral/Analog/Analog.h"


/******************************************************************************/
/*!

*/
/******************************************************************************/
typedef struct MotVMonitor
{
    VMonitor_Context_T V_SOURCE;/* Controller Supply */
    VMonitor_Context_T V_ACCESSORIES; /* ~12V */
    VMonitor_Context_T V_ANALOG; /* V Analog Sensors ~5V */
}
MotVMonitor_T;

//init with 1 macrp


/******************************************************************************/
/*!
    Analog Conversions
*/
/******************************************************************************/
// typedef const struct MotVMonitor_Conversions
typedef const struct MotVMonitor_Analog
{
    const Analog_ConversionChannel_T CONVERSION_VSOURCE;
    const Analog_ConversionChannel_T CONVERSION_VSENSE;
    const Analog_ConversionChannel_T CONVERSION_VACCS;
}
MotVMonitor_Analog_T;


static inline void MotVMonitor_Analog_MarkVPower(const MotVMonitor_Analog_T * p_analog)
{
    Analog_Channel_MarkConversion(&p_analog->CONVERSION_VSOURCE);
}

static inline void MotVMonitor_Analog_MarkVBoard(const MotVMonitor_Analog_T * p_analog)
{
    Analog_Channel_MarkConversion(&p_analog->CONVERSION_VSENSE);
    Analog_Channel_MarkConversion(&p_analog->CONVERSION_VACCS);
}

static inline uint16_t MotVMonitor_Analog_GetVPower(const MotVMonitor_Analog_T * p_analog) { return Analog_Channel_GetResult(&p_analog->CONVERSION_VSOURCE); }
static inline uint16_t MotVMonitor_Analog_GetVAnalog(const MotVMonitor_Analog_T * p_analog) { return Analog_Channel_GetResult(&p_analog->CONVERSION_VSENSE); }
static inline uint16_t MotVMonitor_Analog_GetVAccs(const MotVMonitor_Analog_T * p_analog) { return Analog_Channel_GetResult(&p_analog->CONVERSION_VACCS); }



typedef const struct MotVSource_Context
{
    VMonitor_T * P_STATE;
    const Analog_ConversionChannel_T CONVERSION;
}
MotVSource_Context_T;

/******************************************************************************/
/*!
    Thread
*/
/******************************************************************************/
static inline VMonitor_Status_T MotVMonitor_PollVBoard(MotVMonitor_T * p_state, const MotVMonitor_Analog_T * p_analog)
{
    bool isFault = false;

    VMonitor_Poll(&p_state->VAnalog, MotVMonitor_Analog_GetVAnalog(p_analog));
    VMonitor_Poll(&p_state->VAccs, MotVMonitor_Analog_GetVAccs(p_analog));
    // if (VMonitor_IsFault(&p_state->VAnalog) == true) { p_state->FaultFlags.VAnalogLimit = 1U; isFault = true; }
    // if (VMonitor_IsFault(&p_state->VAccs) == true) { p_state->FaultFlags.VAccsLimit = 1U; isFault = true; }

    // if (VMonitor_IsFault(&p_state->VMonitorAccs) == true) { MotorController_StateMachine_SetFault(p_state, ((const MotorController_FaultFlags_T){.VAccsLimit = 1U}).Value); }

    // if (isFault == true) { MotorController_StateMachine_EnterFault(p_const); } /* Sensors checks fault only */

    MotVMonitor_Analog_MarkVBoard(p_analog);
}


