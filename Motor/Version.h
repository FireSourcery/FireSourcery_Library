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
    @brief  For all Motors
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_VERSION_H
#define MOTOR_VERSION_H

/* Library Software Version */
#define MOTOR_LIBRARY_VERSION_OPT       0U
#define MOTOR_LIBRARY_VERSION_MAJOR     0U
#define MOTOR_LIBRARY_VERSION_MINOR     0U
#define MOTOR_LIBRARY_VERSION_FIX       1U
#define MOTOR_LIBRARY_VERSION           ((MOTOR_LIBRARY_VERSION_OPT << 24U) | (MOTOR_LIBRARY_VERSION_MAJOR << 16U) | (MOTOR_LIBRARY_VERSION_MINOR << 8U) | (MOTOR_LIBRARY_VERSION_FIX))


static inline uint8_t Motor_LibraryVersionIndex(uint8_t charIndex)
{
    uint8_t versionChar;
    switch (charIndex)
    {
        case 0U: versionChar = MOTOR_LIBRARY_VERSION_FIX;    break;
        case 1U: versionChar = MOTOR_LIBRARY_VERSION_MINOR;  break;
        case 2U: versionChar = MOTOR_LIBRARY_VERSION_MAJOR;  break;
        case 3U: versionChar = MOTOR_LIBRARY_VERSION_OPT;    break;
        default: versionChar = 0U; break;
    }
    return versionChar;
}

#endif
