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
#ifndef ANALOG_N_H
#define ANALOG_N_H

#include "../Analog/Analog.h"
#include "../Analog/Config.h"

#include "Utility/Ring/Ring.h"

#include <stdint.h>
#include <stdbool.h>

typedef union AnalogN_AdcFlags_Tag
{
    struct
    {
        uint32_t Adc0 : 1U;
        uint32_t Adc1 : 1U;
        uint32_t Adc2 : 1U;
        uint32_t Adc3 : 1U;
        uint32_t Adc4 : 1U;
        uint32_t Adc5 : 1U;
        uint32_t Adc6 : 1U;
        uint32_t Adc7 : 1U;
    };
    uint8_t Flags;
}
AnalogN_AdcFlags_T;

typedef const struct AnalogN_Conversion_Tag
{
    const Analog_Conversion_T CONVERSION;
    Analog_T * const P_ANALOG; /* AnalogI in Analog[N] */
}
AnalogN_Conversion_T;

#define ANALOG_N_CONVERSION_HOST_NULL (0x8U)

#define ANALOG_N_CONVERSION_INIT(Channel, OnComplete, p_CallbackContext, p_Results, PinId, p_AnalogHost)    \
{                                                                                                           \
    .CONVERSION     = ANALOG_CONVERSION_INIT(Channel, OnComplete, p_CallbackContext, p_Results, PinId),     \
    .P_ANALOG       = p_AnalogHost,                                                                         \
}

//todo
typedef const struct
{
    analog_channel_t * P_CHANNELS;
    uint8_t CHANNELS_COUNT;
    AnalogN_AdcFlags_T ACTIVE_ADCS;
}
AnalogN_ConversionGroup_T;

typedef const struct AnalogN_Config_Tag
{
    Analog_T * const P_ANALOGS;
    uint8_t ANALOG_COUNT;
}
AnalogN_Config_T;

typedef struct AnalogN_Tag
{
    const AnalogN_Config_T CONFIG;
}
AnalogN_T;

/*
    AnalogN_Conversion_T passed to selected AnalogI
*/
static inline void AnalogN_ActivateConversion(const AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
    (void)p_analogn;
    Analog_ActivateConversion(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
}

static inline bool AnalogN_EnqueueConversion(const AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
    (void)p_analogn;
    return Analog_EnqueueConversion(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
}

/*
    AnalogN Options Enqueue options on all AnalogI
*/
static inline bool AnalogN_EnqueueOptions(const AnalogN_T * p_analogn, const Analog_Options_T * p_options)
{
    bool isSuccess = true;

    for(uint8_t iAnalog = 0U; iAnalog < p_analogn->CONFIG.ANALOG_COUNT; iAnalog++)
    {
        if(Analog_EnqueueOptions(&p_analogn->CONFIG.P_ANALOGS[iAnalog], p_options) == false) { isSuccess = false; }
    }

    return isSuccess;
}

/******************************************************************************/
/*!
    Group
*/
/******************************************************************************/
/*!
    Multithreaded still need to disable all interrupts
    Single Threaded N adc - use activeAdcs
    Multithreaded N adc - not use activeAdcs
    @param input activeAdcs - activeAdcs used by group defined by upper layer
*/
static inline void AnalogN_Group_PauseQueue(const AnalogN_T * p_analogn, AnalogN_AdcFlags_T activeAdcs)
{
#if  defined(CONFIG_ANALOG_MULTITHREADED)
    (void)p_analogn; (void)activeAdcs;
    Critical_Enter();
#elif defined(CONFIG_ANALOG_SINGLE_THREADED)
    for(uint8_t iAnalog = 0U; iAnalog < p_analogn->CONFIG.ANALOG_COUNT; iAnalog++)
    {
        /* Assuming little endian */
        if(((1U << iAnalog) & activeAdcs.Flags) != 0U) { Analog_Group_PauseQueue(&p_analogn->CONFIG.P_ANALOGS[iAnalog]); }
    }
#endif
}

static inline void AnalogN_Group_ResumeQueue(const AnalogN_T * p_analogn, AnalogN_AdcFlags_T activeAdcs)
{
#if  defined(CONFIG_ANALOG_MULTITHREADED) /* Proc all queues, common Critical_Exit */
    (void)p_analogn;
    for(uint8_t iAnalog = 0U; iAnalog < p_analogn->CONFIG.ANALOG_COUNT; iAnalog++)
    {
        if(((1U << iAnalog) & activeAdcs.Flags) != 0U) { _Analog_Group_ResumeQueue(&p_analogn->CONFIG.P_ANALOGS[iAnalog]); }
    }
    Critical_Exit();
#elif defined(CONFIG_ANALOG_SINGLE_THREADED)
    for(uint8_t iAnalog = 0U; iAnalog < p_analogn->CONFIG.ANALOG_COUNT; iAnalog++)
    {
        if(((1U << iAnalog) & activeAdcs.Flags) != 0U) { Analog_Group_ResumeQueue(&p_analogn->CONFIG.P_ANALOGS[iAnalog]); }
    }
#endif
}

static inline bool AnalogN_Group_EnqueueConversion(const AnalogN_T * p_analogn, const AnalogN_Conversion_T * p_conversion)
{
    (void)p_analogn;
    return Analog_Group_EnqueueConversion(p_conversion->P_ANALOG, &p_conversion->CONVERSION);
}

// static inline bool AnalogN_Group_EnqueueOptions(const AnalogN_T * p_analogn, const Analog_Options_T * p_options, AnalogN_AdcFlags_T activeAdcs)
// {
//     for(uint8_t iAnalog = 0U; iAnalog < p_analogn->CONFIG.ANALOG_COUNT; iAnalog++)
//     {
//         if(((1U << iAnalog) & activeAdcs.Flags) != 0U) { Analog_EnqueueOptions(&p_analogn->CONFIG.P_ANALOGS[iAnalog], p_options); }
//     }
// }

extern void AnalogN_Init(const AnalogN_T * p_analogn);

#endif




