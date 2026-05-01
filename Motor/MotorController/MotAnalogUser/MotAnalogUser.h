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
    @file   MotAnalogUser.h
    @author FireSourcery
    @brief  Analog input role assignments. Pedals (throttle, brake) live as
            slots in MotorController_T.AINS[]; this header names the slots.
*/
/******************************************************************************/
#include "Transducer/UserIn/UserAIn.h"


#ifndef MOT_USER_DIN_COUNT
#define MOT_USER_DIN_COUNT 2U
#endif

#ifndef MOT_USER_AIN_COUNT
#define MOT_USER_AIN_COUNT 2U
#endif

/*
    Fixed-slot role binding for AINS[]
*/
typedef enum MotAnalogUser_AinId
{
    MOT_AIN_THROTTLE = 0U,
    MOT_AIN_BRAKE    = 1U,
    MOT_AIN_COUNT
}
MotAnalogUser_AinId_T;
