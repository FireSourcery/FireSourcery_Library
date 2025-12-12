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
    @file   Phase.h
    @author FireSourcery
    @brief  Phase Bits Common
*/
/******************************************************************************/
#include <stdint.h>
#include <assert.h>
#include <sys/types.h>

/******************************************************************************/
/*!
    as interface header
*/
/******************************************************************************/
/*
    3-Phase Active/Align
*/
typedef enum Phase_Id
{
    PHASE_ID_0      = (0b000U),
    PHASE_ID_A      = (0b001U),
    PHASE_ID_INV_C  = (0b011U),
    PHASE_ID_B      = (0b010U),
    PHASE_ID_INV_A  = (0b110U),
    PHASE_ID_C      = (0b100U),
    PHASE_ID_INV_B  = (0b101U),
    PHASE_ID_ABC    = (0b111U),
}
Phase_Id_T;

typedef union Phase_Bitmask
{
    struct
    {
        uint8_t A : 1U;
        uint8_t B : 1U;
        uint8_t C : 1U;
        uint8_t Resv : 5U;
    };
    uint8_t Bits;
}
Phase_Bitmask_T;

static inline Phase_Bitmask_T Phase_Bitmask(Phase_Id_T id) { return (Phase_Bitmask_T) { .Bits = id }; }

/* Virtual CCW */
static inline Phase_Id_T Phase_NextOf(Phase_Id_T id)
{
    static const Phase_Id_T TABLE[] =
    {
        [PHASE_ID_A]        = PHASE_ID_INV_C,      /* 0 -> 60 */
        [PHASE_ID_INV_C]    = PHASE_ID_B,          /* 60 -> 120 */
        [PHASE_ID_B]        = PHASE_ID_INV_A,      /* 120 -> 180 */
        [PHASE_ID_INV_A]    = PHASE_ID_C,          /* 180 -> 240 */
        [PHASE_ID_C]        = PHASE_ID_INV_B,      /* 240 -> 300 */
        [PHASE_ID_INV_B]    = PHASE_ID_A,          /* 300 -> 360 */
        [PHASE_ID_0]        = PHASE_ID_0,          /* 0 -> 0 */
        [PHASE_ID_ABC]      = PHASE_ID_ABC,        /* 0 -> 0 */
        // [PHASE_ID_0]        = PHASE_ID_A,          /* Init as 0 */
    };

    return TABLE[id];
}

static inline Phase_Id_T Phase_PrevOf(Phase_Id_T id)
{
    static const Phase_Id_T TABLE[] =
    {
        [PHASE_ID_A]        = PHASE_ID_INV_B,      /* 0 -> 300 */
        [PHASE_ID_INV_B]    = PHASE_ID_C,          /* 300 -> 240 */
        [PHASE_ID_C]        = PHASE_ID_INV_A,      /* 240 -> 180 */
        [PHASE_ID_INV_A]    = PHASE_ID_B,          /* 180 -> 120 */
        [PHASE_ID_B]        = PHASE_ID_INV_C,      /* 120 -> 60 */
        [PHASE_ID_INV_C]    = PHASE_ID_A,          /* 60 -> 0 */
        [PHASE_ID_0]        = PHASE_ID_0,          /* 0 -> 0 */
        [PHASE_ID_ABC]      = PHASE_ID_ABC,        /* 0 -> 0 */
    };

    return TABLE[id];
}

// static inline Phase_Id_T Phase_DirectOf(Phase_Id_T id, int sign)
// {
//     if (sign > 0) { return Phase_NextOf(id); }
//     else if (sign < 0) { return Phase_PrevOf(id); }
//     else { return id; /* no change */ }
// }

static inline uint16_t Phase_AngleOf(Phase_Id_T id)
{
    static const uint16_t ANGLE_TABLE[] =
    {
        [PHASE_ID_A]        = 0U,         /* 0 */
        [PHASE_ID_INV_C]    = 10922U,     /* 60 */
        [PHASE_ID_B]        = 21845U,     /* 120 */
        [PHASE_ID_INV_A]    = 32768U,     /* 180 */
        [PHASE_ID_C]        = 43690U,     /* 240 */
        [PHASE_ID_INV_B]    = 54613U,     /* 300 */
        [PHASE_ID_0]        = 0U,
        [PHASE_ID_ABC]      = 0U,
    };

    return ANGLE_TABLE[id];
}

// Phase_Id_T Phase_IdOfAngle(angle16_t angle)
// {
//     Phase_Id_T sectorId;
//     if (angle < ANGLE16_180)
//     {
//         if      (angle < ANGLE16_60)    { sectorId = PHASE_ID_A; }
//         else if (angle < ANGLE16_120)   { sectorId = PHASE_ID_INV_C; }
//         else                            { sectorId = PHASE_ID_B; }
//     }
//     else
//     {
//         if      (angle < ANGLE16_240)   { sectorId = PHASE_ID_INV_A; }
//         else if (angle < ANGLE16_300)   { sectorId = PHASE_ID_C; }
//         else                            { sectorId = PHASE_ID_INV_B; }
//     }
//     return sectorId;
// }

typedef enum Phase_Index
{
    PHASE_INDEX_A = 0U,
    PHASE_INDEX_B = 1U,
    PHASE_INDEX_C = 2U,
}
Phase_Index_T;

typedef union Phase_Triplet
{
    struct
    {
        int16_t A;
        int16_t B;
        int16_t C;
    };
    int16_t Values[3U];
}
Phase_Triplet_T;

static inline int16_t Phase_ValueAlignedOf(Phase_Triplet_T * p_input, Phase_Id_T id)
{
    switch (id)
    {
        case PHASE_ID_A:        return p_input->A;
        case PHASE_ID_B:        return p_input->B;
        case PHASE_ID_C:        return p_input->C;
        case PHASE_ID_INV_A:    return (0 - p_input->A);
        case PHASE_ID_INV_B:    return (0 - p_input->B);
        case PHASE_ID_INV_C:    return (0 - p_input->C);
        default:                return 0;
    }
}

// typedef struct Phase_Data_T
// {
//     Phase_Triplet_T Values;
//     Phase_Bitmask_T Flags;
// }
// Phase_Data_T;