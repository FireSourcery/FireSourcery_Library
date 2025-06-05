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
    @file   MotHeatMonitor.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Transducer/Thermistor/HeatMonitor.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Peripheral/Analog/Analog.h"

typedef HeatMonitor_GroupContext_T MotHeatMonitor_Mosfets_T;

/******************************************************************************/
/*
    Conversions
*/
/******************************************************************************/
typedef const struct MotHeatMonitor_Analog
{
    Analog_ConversionChannel_T PCB_CONVERSION;
    struct { const Analog_ConversionChannel_T * P_ENTRIES; uint8_t COUNT; } MOSFETS_CONVERSIONS;
    // const Analog_ConversionChannel_T * const P_HEAT_MOSFETS_CONVERSIONS;
    // const uint8_t HEAT_MOSFETS_COUNT;
}
MotHeatMonitor_Analog_T;

static inline void MotHeatMonitor_Analog_Mark(const MotHeatMonitor_Analog_T * p_const)
{
    Analog_Channel_MarkConversion(&p_const->PCB_CONVERSION);
    for (uint8_t index = 0U; index < p_const->MOSFETS_CONVERSIONS.COUNT; index++)
        { Analog_Channel_MarkConversion(&p_const->MOSFETS_CONVERSIONS.P_ENTRIES[index]); }
}

static inline uint16_t MotHeatMonitor_Analog_GetHeatPcb(const MotHeatMonitor_Analog_T * p_const) { return Analog_Channel_GetResult(&p_const->PCB_CONVERSION); }
static inline uint16_t MotHeatMonitor_Analog_GetHeatMosfets(const MotHeatMonitor_Analog_T * p_const, uint8_t index) { return Analog_Channel_GetResult(&p_const->MOSFETS_CONVERSIONS.P_ENTRIES[index]); }

static inline uint16_t MotHeatMonitor_Analog_GetHeatMostfetsAll_Ntc(const MotHeatMonitor_Analog_T * p_const)
{
    uint16_t monitorInput = UINT16_MAX; /* Start with max value */
    uint16_t adcu;

    for (uint8_t index = 0U; index < p_const->MOSFETS_CONVERSIONS.COUNT; index++)
    {
        adcu = Analog_Channel_GetResult(&p_const->MOSFETS_CONVERSIONS.P_ENTRIES[index]);
        if (adcu < monitorInput) { monitorInput = adcu; }
    }
}

/******************************************************************************/
/*!
    Thread
*/
/******************************************************************************/
typedef const struct MotHeatMonitor_Context
{
    // HeatMonitor_T * const P_PCB;
    Thermistor_T PCB_THERMISTOR; /* PCB Thermistor */
    struct { const Thermistor_T * P_ENTRIES; uint8_t COUNT; } MOSFETS_THERMISTORS;

    HeatMonitor_T * P_STATE; /* Single monitor on collective Mosfets heats */
    HeatMonitor_Config_T * P_NVM_CONFIG;

    MotHeatMonitor_Analog_T ANALOG; /* Analog Conversions */
}
MotHeatMonitor_Context_T;

static inline void MotHeatMonitor_Init(const MotHeatMonitor_Context_T * p_context)
{
    HeatMonitor_InitFrom(p_context->P_STATE, p_context->P_NVM_CONFIG);
}

/*
    combined status
    return Fault on Fault
*/
static inline HeatMonitor_Status_T MotHeatMonitor_Proc(const MotHeatMonitor_Context_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    bool isFault = false;
    bool isWarning = false;
    bool isEdge = false;

    // MotHeatMonitor_GetOverLimitScalar(&p_context->MOT_HEAT_MONITOR);
    // isEdge |= HeatMonitor_PollMonitorEdge(&p_mc->ThermistorPcb, MotorController_Analog_GetHeatPcb(p_context));
    // if (HeatMonitor_IsFault(&p_mc->ThermistorPcb) == true) { p_mc->FaultFlags.PcbOverheat = 1U; isFault = true; }

    HeatMonitor_Poll(&p_context->P_STATE, MotHeatMonitor_Analog_GetHeatMostfetsAll_Ntc(&p_context->ANALOG));

    for (uint8_t iMosfets = 0U; iMosfets < p_context->MOT_HEAT_MONITOR_CONVERSIONS.HEAT_MOSFETS_COUNT; iMosfets++)
    {
        isEdge |= HeatMonitor_PollMonitor(&p_mc->MosfetsThermistors[iMosfets], MotorController_Analog_GetHeatMosfets(p_context, iMosfets));
        if (HeatMonitor_IsFault(&p_mc->MosfetsThermistors[iMosfets]) == true) { p_mc->FaultFlags.MosfetsOverheat = 1U; isFault = true; }
    }

    if (isFault == true)
    {
        MotorController_StateMachine_EnterFault(p_context); /* Shutdown repeat set ok */
    }
    else
    {
        /* Warning behaviors edge triggered */
        isWarning |= HeatMonitor_IsWarning(&p_mc->ThermistorPcb);
        for (uint8_t iMosfets = 0U; iMosfets < p_context->MOT_HEAT_MONITOR_CONVERSIONS.HEAT_MOSFETS_COUNT; iMosfets++)
        {
            isWarning |= HeatMonitor_IsWarning(&p_mc->MosfetsThermistors[iMosfets]);
        }

        // if ((isWarning == false) && (isEdge == true)) { MotorController_ClearILimitAll(p_mc, MOT_I_LIMIT_HEAT_MC); } /* clear   on edge */

        if (isWarning == true)
        {
            /*
                Thermistor Adcu is roughly linear in Warning region
                Assume Heat Mosfets as highest heat
                constantly compares lowest active limit, alternatively, check and restore prev on clear limit
                Increasing Limit only, reset on warning clear.
            */
            // MotorController_SetILimitAll_Scalar(p_mc, MOT_I_LIMIT_HEAT_MC, HeatMonitor_GetScalarLimit_Percent16(&p_mc->MosfetsThermistors[0U]) / 2); lowest will be set

            // MotorController_SetILimitAll_Scalar(p_mc, MOT_I_LIMIT_HEAT_MC, HeatMonitor_GetScalarLimit_Percent16(&p_mc->MosfetsThermistors[0U]) / 2);

            // if (p_mc->StateFlags.HeatWarning == false)
            // {
            //     // Blinky_BlinkN(&p_mc->Buzzer, 250U, 250U, 1U); //todo disable during init, alternatively disable monitors on init
            //     p_mc->StateFlags.HeatWarning = true;
            // }
        }
        else
        {
            // if (p_mc->StateFlags.HeatWarning == true)
            // {
            //     // MotorController_ClearILimitAll(p_mc, MOT_I_LIMIT_HEAT_MC); //clear mosfet
            //     p_mc->StateFlags.HeatWarning = false;
            // }
        }
    }

    MotHeatMonitor_Analog_Mark(&p_context->MOT_HEAT_MONITOR_CONVERSIONS);

    // for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++) { Motor_Heat_Thread(&p_context->P_MOTOR_CONSTS[iMotor]); }
}


/*

*/
typedef enum MotHeatMonitor_VarInstanceId
{
    MOT_VAR_HEAT_PCB,
    MOT_VAR_HEAT_MOSFETS,
    MOT_VAR_HEAT_MOSFETS_1,
    MOT_VAR_HEAT_MOSFETS_2,
    MOT_VAR_HEAT_MOSFETS_3,
}
MotHeatMonitor_VarInstanceId_T;

static inline HeatMonitor_T * MotorController_User_GetPtrThermistor(const MotHeatMonitor_Context_T * p_context, MotHeatMonitor_VarInstanceId_T index)
{

    const HeatMonitor_T * p_thermistor;
    switch (index)
    {
        case MOT_VAR_HEAT_PCB: p_thermistor = &p_context->PCB_THERMISTOR; break;
        default:
            p_thermistor = ((index - 1U) < p_context->MOSFETS_THERMISTORS.COUNT) ? &p_context->MOSFETS_THERMISTORS.P_ENTRIES[index - 1U] : NULL;
            break;

    }
    return (HeatMonitor_T *)p_thermistor;
}