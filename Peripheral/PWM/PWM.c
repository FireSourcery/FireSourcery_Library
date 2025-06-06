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

*/
/******************************************************************************/
#include "PWM.h"

#include "HAL_PWM.h"

#include <stdint.h>
#include <stdbool.h>

void PWM_Init(const PWM_T * p_pwm)
{
    HAL_PWM_InitModule(p_pwm->P_HAL_PWM);
    HAL_PWM_InitChannel(p_pwm->P_HAL_PWM, p_pwm->CHANNEL_ID);
}

/* Init As Channel */
void PWM_Channel_Init(const PWM_T * p_pwm)
{
    HAL_PWM_InitChannel(p_pwm->P_HAL_PWM, p_pwm->CHANNEL_ID);
}

/* Init As Module */
void PWM_Module_Init(const PWM_Module_T * p_pwm)
{
    HAL_PWM_InitModule(p_pwm->P_HAL_PWM_MODULE);
    HAL_PWM_InitModulePeriod(p_pwm->P_HAL_PWM_MODULE, p_pwm->PERIOD_TICKS);
}

