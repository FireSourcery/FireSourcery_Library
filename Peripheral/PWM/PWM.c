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
    @file   PWM.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "PWM.h"

#include "HAL_PWM.h"

#include <stdint.h>
#include <stdbool.h>

void PWM_Init(PWM_T * p_pwm)
{
    HAL_PWM_InitModule(p_pwm->CONFIG.P_HAL_PWM);
    HAL_PWM_InitChannel(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID);
}

void PWM_InitChannel(PWM_T * p_pwm) { HAL_PWM_InitChannel(p_pwm->CONFIG.P_HAL_PWM, p_pwm->CONFIG.CHANNEL_ID); }
void PWM_InitModule(PWM_T * p_pwm) { HAL_PWM_InitModule(p_pwm->CONFIG.P_HAL_PWM); }

