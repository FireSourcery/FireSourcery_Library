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
    @file   MotorAnalog.h
    @author FireSourcery
    @brief  Motor Analog Interface. independent of ADC

*/
/******************************************************************************/
#include "MotorAnalogRef.h"
#include "Math/Fixed/fract16.h"

/*
    [Phase_Analog]
*/
/* reserved for abstract interface in case of off-chip capture */

typedef volatile struct MotorAnalog_State
{
    int16_t Va_Fract16;
    int16_t Vb_Fract16;
    int16_t Vc_Fract16;
    int16_t Ia_Fract16;
    int16_t Ib_Fract16;
    int16_t Ic_Fract16;
    // Bits VBatchSync;
    // Bits IBatchSync;
}
MotorAnalog_State_T;

static inline int16_t MotorAnalog_GetVa_Fract16(const MotorAnalog_State_T * p_analogState) { return p_analogState->Va_Fract16; }
static inline int16_t MotorAnalog_GetVb_Fract16(const MotorAnalog_State_T * p_analogState) { return p_analogState->Vb_Fract16; }
static inline int16_t MotorAnalog_GetVc_Fract16(const MotorAnalog_State_T * p_analogState) { return p_analogState->Vc_Fract16; }
static inline int16_t MotorAnalog_GetIa_Fract16(const MotorAnalog_State_T * p_analogState) { return p_analogState->Ia_Fract16; }
static inline int16_t MotorAnalog_GetIb_Fract16(const MotorAnalog_State_T * p_analogState) { return p_analogState->Ib_Fract16; }
static inline int16_t MotorAnalog_GetIc_Fract16(const MotorAnalog_State_T * p_analogState) { return p_analogState->Ic_Fract16; }

// typedef volatile struct MotorAnalog_VSource
// {
//     uint16_t Value_Fract16;
//     uint32_t InvScalar;
// }
// MotorAnalog_VSource_T;

/*
    static Global State for VSource/VPhaseBus
*/
/* static in .c */
extern void MotorAnalog_InitVSource_V(uint16_t vSource_V);
extern void MotorAnalog_CaptureVSource_Adcu(uint16_t vSource_Adcu);
extern uint16_t MotorAnalog_GetVSource_Fract16(void);
extern uint16_t MotorAnalog_GetVSource_Adcu(void);
extern uint16_t MotorAnalog_GetVSource_V(void);
extern uint32_t MotorAnalog_GetVSourceInvScalar(void);
// extern uint16_t MotorAnalog_GetVNominal_V(void);