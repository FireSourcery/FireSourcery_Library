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
    @file   MuxInput.h
    @author FireSourcery
    @brief  Generic Interface Mux
*/
/******************************************************************************/
#include "Type/Accessor/Accessor.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef MOT_USER_INPUT_MUX_SOURCES_MAX
#define MOT_USER_INPUT_MUX_SOURCES_MAX (8U)
#endif


// typedef void (*MuxInput_SetDestField_T)(void * p_inputDest, int fieldId, uintptr_t value);
// typedef uintptr_t(*MuxInput_GetDestField_T)(void * p_inputDest, int fieldId);

// typedef uintptr_t(*MuxInput_Capture_T)(void * p_inputDest, int fieldId);
// typedef void (*MuxInput_SetInput_T)(void * p_context, uintptr_t value);
// typedef void (*MuxInput_SetDestField_T)(void * p_sourceDest, void * p_inputDest,  uintptr_t value);

/* [InputSource] */
/* _VTable with fixed context, "StateMachine State" */
typedef const struct VTable
{
    Accessor_T ACCESSOR;
    void (*CAPTURE)(const void * p_adapter, void * p_input);
    void (*PROCESS)(const void * p_adapter);

    // MuxInput_SetDestField_T SET_INPUT_FIELD;
    // MuxInput_Proc_T INIT; /* Re init peripheral registers */
    // void (*CaptureInputs)(const void * p_adapter, void * p_input);
    // bool (*PollEdge)(const void * p_adapter);
    // Motor_UserDirection_T(*GetDirection)(const void * p_adapter);
    // MuxInput_T * P_VTABLE;

    // void * P_INPUT_STATE;
    // const void * P_SOURCE;  // Points to MotAnalogUser_T, Socket Cmd, etc.
    // const void * P_DEST; //optionally directly copy
}
VTable_T;

static inline void VTable_SetField(const VTable_T * p_vtable, void * p_context, int fieldId, int value)
{
    Accessor_Set(&p_vtable->ACCESSOR, p_context, fieldId, value);
}

/*

*/
// typedef struct Mux_State
// {
//     VTable_T * p_Selected;
//     // User_Input_T InputBuffer; // result of capture
// //     InputMux_SourceId_T ActiveSourceId;
// //     bool AutoSelectEnabled;
// }
// Mux_State_T;

/*
    Interface Mux
    Mux with multiple input sources
    "StateMachine"
*/
typedef const struct Mux
{
    // Mux_State_T * P_STATE;
    VTable_T * const * PP_SOURCES;     /* Array of input sources */
    uint8_t SOURCE_COUNT;
    void * P_CONTEXT;
    // const VTable_T ** PP_SOURCES;     /* Array of input sources */
    // uint8_t SOURCE_COUNT;
}
Mux_T;

static inline VTable_T * Mux_Select(const Mux_T * p_mux, int select) { return (select < p_mux->SOURCE_COUNT) ? p_mux->PP_SOURCES[select] : NULL; }
// directly set, can be templated for permanent selection
static inline void Mux_SetField(const Mux_T * p_mux, int select, int fieldId, uintptr_t value) { VTable_SetField(p_mux->PP_SOURCES[select], p_mux->P_CONTEXT, fieldId, value); }

// static inline void Mux_SetField(const Mux_T * p_mux, int select, int fieldId, uintptr_t value) { VTable_SetField(Mux_Select(p_mux, select), p_mux->P_CONTEXT, fieldId, value); }

/******************************************************************************/
/* Public API */
/******************************************************************************/
