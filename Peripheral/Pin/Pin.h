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
    @file     Pin.h
    @author FireSourcery
    @brief    Uniform Wrapper for HAL_Pin
    @version V0
*/
/******************************************************************************/
#ifndef PIN_H
#define PIN_H

#include "Config.h"
#include "HAL_Pin.h"
#include <stdint.h>
#include <stdbool.h>

typedef const struct Pin_Config_Tag
{
    HAL_Pin_T * const P_HAL_PIN;
    const uint32_t ID;
    const uint32_t MASK;
    const bool IS_INVERT; /* Use ground state as on */
}
Pin_Config_T;

typedef struct Pin_Tag
{
    const Pin_Config_T CONFIG;
}
Pin_T;

#define PIN_INIT(p_Hal, Id)        \
{                                \
    .CONFIG =                     \
    {                            \
        .P_HAL_PIN = p_Hal,        \
        .ID = Id,                \
        .MASK = (1UL << Id),    \
        .IS_INVERT = false,        \
    },                            \
}

#define PIN_INIT_INVERT(p_Hal, Id, IsInvert)    \
{                                                \
    .CONFIG =                                     \
    {                                            \
        .P_HAL_PIN = p_Hal,                        \
        .ID = Id,                                \
        .MASK = (1UL << Id),                    \
        .IS_INVERT = IsInvert,                    \
    },                                            \
}

static inline uint32_t _Pin_GetHalArg(const Pin_T * p_pin)
{
#ifdef CONFIG_PIN_HAL_USE_MASK
    return p_pin->CONFIG.MASK;
#elif defined(CONFIG_PIN_HAL_USE_ID)
    return p_pin->CONFIG.ID;
#endif
}

/* Ignore invert check, when handled by upper layer */
static inline void Pin_Output_Low(const Pin_T * p_pin)        { HAL_Pin_WriteOutputOff(p_pin->CONFIG.P_HAL_PIN, _Pin_GetHalArg(p_pin)); }
static inline void Pin_Output_High(const Pin_T * p_pin)     { HAL_Pin_WriteOutputOn(p_pin->CONFIG.P_HAL_PIN, _Pin_GetHalArg(p_pin)); }
static inline void Pin_Output_Toggle(const Pin_T * p_pin)     { HAL_Pin_ToggleOutput(p_pin->CONFIG.P_HAL_PIN, _Pin_GetHalArg(p_pin)); }
static inline void Pin_Output_WritePhysical(const Pin_T * p_pin, bool isOn) { HAL_Pin_WriteOutput(p_pin->CONFIG.P_HAL_PIN, _Pin_GetHalArg(p_pin), isOn); }

/* Include invert check */
static inline void Pin_Output_Off(const Pin_T * p_pin)
{
    if(p_pin->CONFIG.IS_INVERT == true) { Pin_Output_High(p_pin); }
    else                                 { Pin_Output_Low(p_pin); }
}

static inline void Pin_Output_On(const Pin_T * p_pin)
{
    if(p_pin->CONFIG.IS_INVERT == true) { Pin_Output_Low(p_pin); }
    else                                 { Pin_Output_High(p_pin); }
}

static inline void Pin_Output_Write(const Pin_T * p_pin, bool isOn)
{
    bool writeVal = (p_pin->CONFIG.IS_INVERT == true) ? !isOn : isOn;
    Pin_Output_WritePhysical(p_pin, writeVal);
}

/* Ignore invert check, when handled by upper layer */
static inline bool Pin_Input_ReadPhysical(const Pin_T * p_pin) { return HAL_Pin_ReadInput(p_pin->CONFIG.P_HAL_PIN, _Pin_GetHalArg(p_pin)); }

/* Include invert check */
static inline bool Pin_Input_Read(const Pin_T * p_pin)
{
    bool readVal = Pin_Input_ReadPhysical(p_pin);
    return (p_pin->CONFIG.IS_INVERT == true) ? !readVal : readVal;
}

/******************************************************************************/
/*!
*/
/******************************************************************************/
extern void Pin_Output_Init(Pin_T * p_pin);
extern void Pin_Input_Init(Pin_T * p_pin);
extern void Pin_Deinit(const Pin_T * p_pin);
extern void Pin_EnableInvert(Pin_T * p_pin);
extern void Pin_DisableInvert(Pin_T * p_pin);

#endif
