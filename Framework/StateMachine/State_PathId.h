#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   State_PathId.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>

/******************************************************************************/
/*
    PathId solves layer namespace
    scoped/typed getter also allows file / dependency namespace
*/
/*!
    @brief  Encodes the entire active substate path into a 32-bit ID.

    Encoding formats (selectable):
      - 4-bit fields × 8 levels: supports 15 substates per level, 8 depth max   <- current
      - 8-bit fields × 4 levels: supports 255 substates per level, 4 depth max

    Root state -> leaf state maps to LSB -> MSB.
    Unused (deeper) fields are 0.

    e.g. for path Root(2) -> Sub(3) -> SubSub(1):
      4-bit: 0x00000132   (Depth0 = 2, Depth1 = 3, Depth2 = 1)
      8-bit: 0x00010302

    [Id 0 at depth >= 1 means "no state at this level"].
    This is what lets the depth be recovered from the value alone, with no separate depth field:
    the highest set nibble is the leaf. A parent and its first child are otherwise indistinguishable,
    so a sub-state id space must begin at 1 and reserve 0 for the bare parent. Root ids (depth 0)
    are unconstrained — 0 is a valid root.

    A consequence: a state that forgets to define [PATH_ID] reads as root id 0 rather than as
    "unset". [State_IsPathIdValid] exists to catch that; see [StateMachine_GetPathId].
*/
/******************************************************************************/
typedef const union State_PathId
{
    uint32_t Id;
    struct
    {
        uint32_t Depth0 : 4;
        uint32_t Depth1 : 4;
        uint32_t Depth2 : 4;
        uint32_t Depth3 : 4;
        uint32_t Depth4 : 4;
        uint32_t Depth5 : 4;
        uint32_t Depth6 : 4;
        uint32_t Depth7 : 4;
    };
}
State_PathId_T;

/*
    .ID = (State_PathId_T){ ROOT_ID, SUB_ID, ... }.Id
*/
// #define STATE_PATH_ID(ROOT_ID, ...) (State_PathId_T){ ROOT_ID, __VA_ARGS__ }.Id


#define STATE_ID_BITS     (4U)
#define STATE_ID_MASK     (0xFU)
#define STATE_ID_MAX      (15U)  /* per level. 0 reserved as "no state" at depth >= 1 */
#define STATE_PATH_DEPTH_MAX (32U / STATE_ID_BITS)

/*
*/
/* Base building block: place a value at a given depth */
#define _STATE_ID(Value, Depth)    ((state_t)(Value) << ((Depth) * STATE_ID_BITS))

// /* Construct IDs at each nesting depth */
// #define STATE_ID_0(Root)                (_STATE_ID(Root, 0))
// #define STATE_ID_1(Root, D1)            (_STATE_ID(Root, 0) | _STATE_ID(D1, 1))
// #define STATE_ID_2(Root, D1, D2)        (_STATE_ID(Root, 0) | _STATE_ID(D1, 1) | _STATE_ID(D2, 2))
// #define STATE_ID_3(Root, D1, D2, D3)    (_STATE_ID(Root, 0) | _STATE_ID(D1, 1) | _STATE_ID(D2, 2) | _STATE_ID(D3, 3))

// #define _STATE_ID_FOLD(Head, ...) (_STATE_ID(Head, Depth) __VA_OPT__(| _STATE_ID_FOLD_1((Depth) + 1, __VA_ARGS__)))
#define _STATE_ID_FOLD_1(D1, ...) (_STATE_ID(D1, 1) __VA_OPT__(| _STATE_ID_FOLD_2(__VA_ARGS__)))
#define _STATE_ID_FOLD_2(D2, ...) (_STATE_ID(D2, 2) __VA_OPT__(| _STATE_ID_FOLD_3(__VA_ARGS__)))
#define _STATE_ID_FOLD_3(D3, ...) (_STATE_ID(D3, 3) __VA_OPT__(| _STATE_ID_FOLD_4(__VA_ARGS__)))
#define _STATE_ID_FOLD_4(D4, ...) (_STATE_ID(D4, 4) __VA_OPT__(| _STATE_ID_FOLD_5(__VA_ARGS__)))
#define _STATE_ID_FOLD_5(D5, ...) (_STATE_ID(D5, 5) __VA_OPT__(| _STATE_ID_FOLD_6(__VA_ARGS__)))
#define _STATE_ID_FOLD_6(D6, ...) (_STATE_ID(D6, 6) __VA_OPT__(| _STATE_ID_FOLD_7(__VA_ARGS__)))
#define _STATE_ID_FOLD_7(D7, ...) (_STATE_ID(D7, 7))  /* Max depth 7 (8th level) */

#define STATE_PATH_ID(D0, ...) (_STATE_ID(D0, 0) __VA_OPT__(| _STATE_ID_FOLD_1(__VA_ARGS__)))


