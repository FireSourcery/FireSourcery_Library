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
    @file   Phase_Analog.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../Phase/Phase_Types.h"

/* Interface for Capture Results */
typedef struct Phase_Input
{
    Phase_Triplet_T Vabc;
    Phase_Bitmask_T VFlags;
    Phase_Triplet_T Iabc;
    Phase_Bitmask_T IFlags;
}
Phase_Input_T;

static inline int16_t Phase_Input_GetVa_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->Vabc.A; }
static inline int16_t Phase_Input_GetVb_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->Vabc.B; }
static inline int16_t Phase_Input_GetVc_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->Vabc.C; }
static inline int16_t Phase_Input_GetIa_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->Iabc.A; }
static inline int16_t Phase_Input_GetIb_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->Iabc.B; }
static inline int16_t Phase_Input_GetIc_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->Iabc.C; }