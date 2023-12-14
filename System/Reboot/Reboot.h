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
    @file   Reboot.h
    @author FireSourcery
    @brief  Implements Critical Section
    @version V0
*/
/******************************************************************************/
#ifndef REBOOT_H
#define REBOOT_H

#ifdef CONFIG_SYSTEM_MCU_ARM

#include "External/CMSIS/Core/Include/cmsis_compiler.h"
// caller includes header

static inline void Reboot(void)
{
    NVIC_SystemReset();
}

#endif



#endif