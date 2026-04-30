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
    @file   MotorController_MotPark.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../../MotorController_StateMachine.h"


static inline void MotorController_CallParkPin(const MotorController_T * p_dev, UserDIn_Edge_T edge)
{
    switch (edge)
    {
        case USER_DIN_EDGE_RISING:  MotorController_EnterPark(p_dev); break;
        case USER_DIN_EDGE_FALLING: MotorController_EnterMain(p_dev); break;
        default: break;
    }
}

#if defined(MOTOR_CONTROLLER_OPT_DIN_PARK_ID)
// static inline UserDIn_T * MotorController_ParkPin(const MotorController_T * p_dev) { return (UserDIn_T *)&p_dev->OPT_DINS[MOTOR_CONTROLLER_OPT_DIN_PARK_ID]; }
// static inline UserDIn_T * MotorController_ParkPin(const MotorController_T * p_dev) { return (UserDIn_T *)&p_dev->PARK_PIN; }

#include "ParkPin/MotorController_ParkPin.h"
static inline void MotorController_ProcOptDin(const MotorController_T * p_dev)
{
    MotorController_CallParkPin(p_dev, UserDIn_PollEdge(&p_dev->OPT_DINS[MOTOR_CONTROLLER_OPT_DIN_PARK_ID]));
}
#endif