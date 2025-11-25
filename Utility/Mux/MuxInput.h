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
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#ifndef MOT_USER_INPUT_MUX_SOURCES_MAX
#define MOT_USER_INPUT_MUX_SOURCES_MAX (8U)
#endif

// static inline void MotorController_User_SetCmdValue(MuxInput_State_T * p_context, int16_t userCmd)

typedef void (*MuxInput_SetDestField_T)(void * p_inputDest, int indexId, uintptr_t value);
typedef uintptr_t(*MuxInput_GetDestField_T)(void * p_inputDest, int indexId);

typedef uintptr_t(*MuxInput_Capture_T)(void * p_inputDest, int indexId);


// typedef void (*MuxInput_SetInput_T)(void * p_context, uintptr_t value);

/* [InputSource] */
/* _VTable with fixed context, State*/
typedef const struct MuxInput
{
    // MuxInput_Proc_T INIT; /* Re init peripheral registers */
    MuxInput_SetDestField_T SET_INPUT_FIELD;

    // void (*CaptureInputs)(const void * p_adapter, void * p_input);
    // bool (*PollEdge)(const void * p_adapter);
    // Motor_User_Direction_T(*GetDirection)(const void * p_adapter);

    /* Select active input source */
    // void (*SelectSource)(void * p_context, InputMux_SourceId_T sourceId);

    /* Get currently active source */
    // InputMux_SourceId_T(*GetActiveSource)(const void * p_context);

    /* Check if source is valid/available */
    // bool (*IsSourceValid)(const void * p_context, InputMux_SourceId_T sourceId);

    void * P_INPUT_STATE;
    const void * P_SOURCE;
}
MuxInput_T;

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
    // MuxInput_T * P_VTABLE;
    void * P_STATE;
    // const void * P_SOURCE;  // Points to MotAnalogUser_T, Socket Cmd, etc.
    /* Array of input sources */
    const MuxInput_T ** PP_SOURCES;
    uint8_t SOURCE_COUNT;

    /* Optional: Priority array for auto-selection */
    // const uint8_t * P_PRIORITY_MAP;
}
Mux_T;


// void MuxInput_Init(const MuxInput_T * p_adapter)
// {

// }


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