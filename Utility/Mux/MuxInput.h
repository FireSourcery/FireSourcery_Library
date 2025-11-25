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
#include <stdint.h>
#include <stdbool.h>

#ifndef MOT_USER_INPUT_MUX_SOURCES_MAX
#define MOT_USER_INPUT_MUX_SOURCES_MAX (8U)
#endif


typedef void (*MuxInput_SetDestField_T)(void * p_inputDest, int fieldId, uintptr_t value);
typedef uintptr_t(*MuxInput_GetDestField_T)(void * p_inputDest, int fieldId);

typedef uintptr_t(*MuxInput_Capture_T)(void * p_inputDest, int fieldId);
// typedef void (*MuxInput_SetInput_T)(void * p_context, uintptr_t value);

/* [InputSource] */
/* _VTable with fixed context, State */
typedef const struct MuxInput
{
    MuxInput_SetDestField_T SET_INPUT_FIELD;

    // MuxInput_Proc_T INIT; /* Re init peripheral registers */
    // void (*CaptureInputs)(const void * p_adapter, void * p_input);
    // bool (*PollEdge)(const void * p_adapter);
    // Motor_User_Direction_T(*GetDirection)(const void * p_adapter);

    // MuxInput_T * P_VTABLE;
    void * P_INPUT_STATE;
    const void * P_SOURCE;  // Points to MotAnalogUser_T, Socket Cmd, etc.
    const void * P_DEST; //optionally directly copy
}
MuxInput_T;

static inline void MuxInput_SetField(const MuxInput_T * p_input, int fieldId, uintptr_t value)
{
    p_input->SET_INPUT_FIELD(p_input->P_DEST, fieldId, value);
}

/*

*/
typedef struct Mux_State
{
    MuxInput_T * p_selected;
    // Motor_User_Input_T InputBuffer; // result of capture
//     InputMux_SourceId_T ActiveSourceId;
//     InputMux_SourceId_T PreviousSourceId;
//     uint8_t SourceCount;
//     bool AutoSelectEnabled;
}
Mux_State_T;

/*
    Mux with multiple input sources
*/
typedef const struct Mux
{
    Mux_State_T * P_STATE;
    const MuxInput_T ** PP_SOURCES;     /* Array of input sources */
    uint8_t SOURCE_COUNT;
}
Mux_T;

static inline void Mux_SetField(const Mux_T * p_mux, int fieldId, uintptr_t value) { MuxInput_SetField(p_mux->P_STATE->p_selected, fieldId, value); }



/******************************************************************************/
/* Public API */
/******************************************************************************/
// static inline void InputMux_Init(const InputMux_T * p_mux, InputMux_SourceId_T defaultSourceId)
// {
//     p_mux->P_STATE->ActiveSourceId = defaultSourceId;
//     p_mux->P_STATE->PreviousSourceId = defaultSourceId;
//     p_mux->P_STATE->SourceCount = p_mux->SOURCE_COUNT;
//     p_mux->P_STATE->AutoSelectEnabled = false;
// }

// static inline void InputMux_SelectSource(const InputMux_T * p_mux, InputMux_SourceId_T sourceId)
// {
//     if (p_mux->P_VTABLE->SelectSource != NULL)
//     {
//         p_mux->P_VTABLE->SelectSource(p_mux->P_STATE, sourceId);
//     }
// }

// static inline InputMux_SourceId_T InputMux_GetActiveSource(const InputMux_T * p_mux)
// {
//     return (p_mux->P_VTABLE->GetActiveSource != NULL) ?
//            p_mux->P_VTABLE->GetActiveSource(p_mux->P_STATE) :
//            p_mux->P_STATE->ActiveSourceId;
// }

// static inline const Input_T * InputMux_GetActiveInput(const InputMux_T * p_mux)
// {
//     InputMux_SourceId_T activeId = p_mux->P_STATE->ActiveSourceId;
//     return (activeId < p_mux->SOURCE_COUNT) ? p_mux->PP_SOURCES[activeId] : NULL;
// }

// static inline void InputMux_CaptureInput(const InputMux_T * p_mux)
// {
//     if (p_mux->P_VTABLE->CaptureInput != NULL)
//     {
//         p_mux->P_VTABLE->CaptureInput(p_mux->P_STATE);
//     }
// }

// static inline void InputMux_EnableAutoSelect(const InputMux_T * p_mux, bool enable)
// {
//     p_mux->P_STATE->AutoSelectEnabled = enable;
// }

// static inline void InputMux_AutoSelect(const InputMux_T * p_mux)
// {
//     if (p_mux->P_STATE->AutoSelectEnabled && p_mux->P_VTABLE->AutoSelect != NULL)
//     {
//         p_mux->P_VTABLE->AutoSelect(p_mux->P_STATE);
//     }
// }