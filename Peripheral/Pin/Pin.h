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

*/
/******************************************************************************/
#ifndef PIN_H
#define PIN_H

#include "Config.h"
#include "HAL_Pin.h"
#include <stdint.h>
#include <stdbool.h>

typedef const struct Pin
{
    HAL_Pin_T * const P_HAL_PIN;
    const uint32_t ID; // const uint32_t MASK;
    const bool IS_INVERT; /* Use ground state as on */
}
Pin_T;

#ifdef CONFIG_PIN_HAL_USE_MASK
#define _PIN_INIT_ID(Id)  (1UL << Id)
#elif defined(CONFIG_PIN_HAL_USE_ID)
#define _PIN_INIT_ID(Id)  (Id)
#endif

#define PIN_INIT(p_Hal, Id) { .P_HAL_PIN = p_Hal, .ID = _PIN_INIT_ID(Id), .IS_INVERT = false, } /* Default to not inverted */

#define PIN_INIT_INVERT(p_Hal, Id, IsInvert) { .P_HAL_PIN = p_Hal, .ID = _PIN_INIT_ID(Id), .IS_INVERT = IsInvert, }


/* As voltage level. Ignore invert check, when handled by upper layer */
static inline void Pin_Output_Low(const Pin_T * p_pin)      { HAL_Pin_WriteOutputOff(p_pin->P_HAL_PIN, p_pin->ID); }
static inline void Pin_Output_High(const Pin_T * p_pin)     { HAL_Pin_WriteOutputOn(p_pin->P_HAL_PIN, p_pin->ID); }
static inline void Pin_Output_Toggle(const Pin_T * p_pin)   { HAL_Pin_ToggleOutput(p_pin->P_HAL_PIN, p_pin->ID); }
static inline void Pin_Output_WritePhysical(const Pin_T * p_pin, bool isOn) { HAL_Pin_WriteOutput(p_pin->P_HAL_PIN, p_pin->ID, isOn); }
static inline bool Pin_Output_ReadPhysical(const Pin_T * p_pin) { return HAL_Pin_ReadOutput(p_pin->P_HAL_PIN, p_pin->ID); }

/* As On/Off. Include invert check */
static inline void Pin_Output_Off(const Pin_T * p_pin) { if (p_pin->IS_INVERT == true) { Pin_Output_High(p_pin); } else { Pin_Output_Low(p_pin); } }
static inline void Pin_Output_On(const Pin_T * p_pin) { if (p_pin->IS_INVERT == true) { Pin_Output_Low(p_pin); } else { Pin_Output_High(p_pin); } }
static inline void Pin_Output_Write(const Pin_T * p_pin, bool isOn) { Pin_Output_WritePhysical(p_pin, (isOn ^ p_pin->IS_INVERT)); }
static inline bool Pin_Output_Read(const Pin_T * p_pin) { return Pin_Output_ReadPhysical(p_pin) ^ p_pin->IS_INVERT; }

/* As voltage level. Ignore invert check, when handled by upper layer */
static inline bool Pin_Input_ReadPhysical(const Pin_T * p_pin) { return HAL_Pin_ReadInput(p_pin->P_HAL_PIN, p_pin->ID); }

/* As On/Off. Include invert check */
static inline bool Pin_Input_Read(const Pin_T * p_pin) { return (Pin_Input_ReadPhysical(p_pin) ^ p_pin->IS_INVERT); }

// static inline uint32_t Pin_Module_MaskOf(const Pin_T * p_pin, bool isOn) { return (isOn ^ p_pin->IS_INVERT) ? p_pin->ID : 0; }
static inline uint32_t Pin_Module_MaskOf(const Pin_T * p_pin, bool isOn) { return (isOn ? p_pin->ID : 0UL); }


// static inline bool FastPin_Input_ReadPhysical(const FastPin_T * p_pin) { return HAL_FastPin_ReadInput(p_pin->P_HAL_PIN, p_pin->ID); }


/******************************************************************************/
/*!
*/
/******************************************************************************/
extern void Pin_Output_Init(const Pin_T * p_pin);
extern void Pin_Input_Init(const Pin_T * p_pin);
extern void Pin_Deinit(const Pin_T * p_pin);

#endif
