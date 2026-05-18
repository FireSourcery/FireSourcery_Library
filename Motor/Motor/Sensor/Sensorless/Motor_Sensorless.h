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
    @file   Motor_Sensorless.h
    @author FireSourcery
*/
/******************************************************************************/

#include "../../Motor.h"

/* Start the sensorless align → ramp → closed-loop handoff chain.
   Enters MOTOR_STATE_OPEN_LOOP at the SENSORLESS_ALIGN substate; the substate
   chain transitions to MOTOR_STATE_RUN once the observer reports lock. */
extern void Motor_Sensorless_StartRunChain(Motor_T * p_motor);
