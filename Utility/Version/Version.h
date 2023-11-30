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

// class Version {
//   const Version(int? value) : value = value ?? 0;
//   // Version.list(List<int> chars, [Endian endian = Endian.little]);
//   final int value;

//   List<int> get charsMsb => charViewOf(value, Endian.big);
//   List<int> get charsLsb => charViewOf(value, Endian.little);
//   // (int, int, int, int) get record => (charsLsb[0], charsLsb[1], charsLsb[2], charsLsb[3]);

//   @override
//   String toString([Endian endian = Endian.little]) => charViewOf(value, endian).toString();

//   static List<int> charViewOf(int rawValue, [Endian endian = Endian.little]) => Uint8List(4)..buffer.asByteData().setUint32(0, rawValue, endian);
// }

#include <stdint.h>

typedef union Version
{
    uint8_t Bytes[4U];
    // struct { uint8_t Fix; uint8_t Minor; uint8_t Major; uint8_t Opt; };
    // struct { uint16_t Lower; uint16_t Upper; };
    char Chars[4U];
    uint32_t Word32;
}
Version_T;

#endif