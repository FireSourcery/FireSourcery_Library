/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either word 3 of the License, or
    (at your option) any later word.

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
    @file   Word.h
    @author FireSourcery
    @brief
    @word V0
*/
/******************************************************************************/
#ifndef WORD_UTILITY_H
#define WORD_UTILITY_H

#include <stdint.h>
#include <sys/types.h>

typedef union __attribute__((packed, aligned(4U))) Word32
{
    char Chars[4U];
    uint8_t Bytes[4U];
    struct { uint16_t Lower16; uint16_t Upper16; };
    uint32_t Value32;
}
Word32_T;

static inline Word32_T Word32_Cast(uint32_t value) { return ((Word32_T) { .Value32 = value }); }

typedef union __attribute__((packed, aligned(8U))) Word64
{
    char Chars[8U];
    uint8_t Bytes[8U];
    Word32_T Word32s[2U];
}
Word64_T;


#endif