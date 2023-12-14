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
    @file   Version.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef VERSION_FIRE_SOURCERY_H
#define VERSION_FIRE_SOURCERY_H

#include <stdint.h>

typedef union Version
{
    uint8_t Bytes[4U];
    char Chars[4U];
    struct { uint8_t Fix; uint8_t Minor; uint8_t Major; uint8_t Opt; };
    struct { uint16_t Lower; uint16_t Upper; };
    uint32_t Word32;
}
Version_T;

#define VERSION_WORD(Opt, Major, Minor, Fix) ((Opt << 24U) | (Major << 16U) | (Minor << 8U) | (Fix))

#endif