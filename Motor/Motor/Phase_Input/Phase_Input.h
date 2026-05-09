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
    @file   Phase_Input.h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "../Phase/Phase_Types.h"


// #if     defined(PHASE_V_SENSORS_ISOLATED)
// #elif   defined(PHASE_V_SENSORS_ANALOG)
// #else
// #define PHASE_V_SENSORS_ANALOG
// #endif

// #if     defined(MOTOR_I_SENSORS_AB)
// #elif   defined(MOTOR_I_SENSORS_ABC)
// #else
//     #define MOTOR_I_SENSORS_ABC
// #endif

// #if     defined(MOTOR_V_SENSORS_ISOLATED)
// #elif   defined(MOTOR_V_SENSORS_ANALOG)
// #else
//     #define MOTOR_V_SENSORS_ANALOG
// #endif


/*
    Transport Data Interface
    Interface for Capture Results, compile time selection between capture
*/
/* Phase_Feedback/Capture/Measure */
typedef struct Phase_Input
{
    Phase_Data_T V;
    Phase_Data_T I;
}
Phase_Input_T;


/* light units wrapper */
static inline int16_t Phase_Input_GetVa_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->V.Values.A; }
static inline int16_t Phase_Input_GetVb_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->V.Values.B; }
static inline int16_t Phase_Input_GetVc_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->V.Values.C; }
static inline int16_t Phase_Input_GetIa_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->I.Values.A; }
static inline int16_t Phase_Input_GetIb_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->I.Values.B; }
static inline int16_t Phase_Input_GetIc_Fract16(volatile const Phase_Input_T * p_phase) { return p_phase->I.Values.C; }


static void Phase_Input_ClearI(volatile Phase_Input_T * p_phase) { p_phase->I = (Phase_Data_T){ 0 }; }
static void Phase_Input_ClearV(volatile Phase_Input_T * p_phase) { p_phase->V = (Phase_Data_T){ 0 }; }

/******************************************************************************/
/*!
    Var Interface
*/
/******************************************************************************/
typedef enum Phase_InputVar
{
    PHASE_INPUT_VAR_VA,
    PHASE_INPUT_VAR_VB,
    PHASE_INPUT_VAR_VC,
    PHASE_INPUT_VAR_IA,
    PHASE_INPUT_VAR_IB,
    PHASE_INPUT_VAR_IC,
}
Phase_InputVar_T;


static inline int Phase_InputVar_Get(const Phase_Input_T * p_phase, Phase_InputVar_T var)
{
    switch (var)
    {
        case PHASE_INPUT_VAR_VA: return p_phase->V.Values.A;
        case PHASE_INPUT_VAR_VB: return p_phase->V.Values.B;
        case PHASE_INPUT_VAR_VC: return p_phase->V.Values.C;
        case PHASE_INPUT_VAR_IA: return p_phase->I.Values.A;
        case PHASE_INPUT_VAR_IB: return p_phase->I.Values.B;
        case PHASE_INPUT_VAR_IC: return p_phase->I.Values.C;
        default: return 0;
    }
}

