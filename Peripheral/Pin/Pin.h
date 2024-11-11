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
    @file   Pin.h
    @author FireSourcery
    @brief  Uniform Wrapper for HAL_Pin
    @version V0
*/
/******************************************************************************/
#ifndef PIN_H
#define PIN_H

#include "Config.h"
#include "HAL_Pin.h"
#include <stdint.h>
#include <stdbool.h>

typedef const struct Pin_Const
{
    HAL_Pin_T * const P_HAL_PIN;
    const uint32_t ID;
    // const uint32_t MASK;
    const bool IS_INVERT; /* Use ground state as on */
}
Pin_Const_T;

typedef struct Pin
{
    const Pin_Const_T CONST;
}
Pin_T;

#ifdef CONFIG_PIN_HAL_USE_MASK
#define _PIN_INIT_ID(Id)  (1UL << Id)
#elif defined(CONFIG_PIN_HAL_USE_ID)
#define _PIN_INIT_ID(Id)  (Id)
#endif

#define PIN_INIT(p_Hal, Id)         \
{                                   \
    .CONST =                       \
    {                               \
        .P_HAL_PIN = p_Hal,         \
        .ID = _PIN_INIT_ID(Id),     \
        .IS_INVERT = false,         \
    },                              \
}

#define PIN_INIT_INVERT(p_Hal, Id, IsInvert)    \
{                                               \
    .CONST =                                   \
    {                                           \
        .P_HAL_PIN = p_Hal,                     \
        .ID = _PIN_INIT_ID(Id),                 \
        .IS_INVERT = IsInvert,                  \
    },                                          \
}

#define PIN_INIT_VA(p_Hal, Id, ...) PIN_INIT_INVERT(p_Hal, Id, ...)

// static inline uint32_t _Pin_GetHalId(const Pin_T * p_pin)
// {
// #ifdef CONFIG_PIN_HAL_USE_MASK
//     return p_pin->CONST.MASK;
// #elif defined(CONFIG_PIN_HAL_USE_ID)
//     return p_pin->CONST.ID;
// #endif
// }

/* Ignore invert check, when handled by upper layer */
static inline void Pin_Output_Low(const Pin_T * p_pin)      { HAL_Pin_WriteOutputOff(p_pin->CONST.P_HAL_PIN, p_pin->CONST.ID); }
static inline void Pin_Output_High(const Pin_T * p_pin)     { HAL_Pin_WriteOutputOn(p_pin->CONST.P_HAL_PIN, p_pin->CONST.ID); }
static inline void Pin_Output_Toggle(const Pin_T * p_pin)   { HAL_Pin_ToggleOutput(p_pin->CONST.P_HAL_PIN, p_pin->CONST.ID); }
static inline void Pin_Output_WritePhysical(const Pin_T * p_pin, bool isOn) { HAL_Pin_WriteOutput(p_pin->CONST.P_HAL_PIN, p_pin->CONST.ID, isOn); }

/* Include invert check */
static inline void Pin_Output_Off(const Pin_T * p_pin)  { if(p_pin->CONST.IS_INVERT == true) { Pin_Output_High(p_pin); } else { Pin_Output_Low(p_pin); } }
static inline void Pin_Output_On(const Pin_T * p_pin)   { if(p_pin->CONST.IS_INVERT == true) { Pin_Output_Low(p_pin); } else { Pin_Output_High(p_pin); } }
static inline void Pin_Output_Write(const Pin_T * p_pin, bool isOn) { Pin_Output_WritePhysical(p_pin, (isOn ^ p_pin->CONST.IS_INVERT)); }

/* Ignore invert check, when handled by upper layer */
static inline bool Pin_Input_ReadPhysical(const Pin_T * p_pin) { return HAL_Pin_ReadInput(p_pin->CONST.P_HAL_PIN, p_pin->CONST.ID); }

/* Include invert check */
static inline bool Pin_Input_Read(const Pin_T * p_pin) { return (Pin_Input_ReadPhysical(p_pin) ^ p_pin->CONST.IS_INVERT); }

/******************************************************************************/
/*!
*/
/******************************************************************************/
extern void Pin_Output_Init(Pin_T * p_pin);
extern void Pin_Input_Init(Pin_T * p_pin);
extern void Pin_Deinit(const Pin_T * p_pin);

#endif
