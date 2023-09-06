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
    @file   Global_Motor.h
    @author FireSourcery
    @brief  For all Motors
    @version V0
*/
/******************************************************************************/
#ifndef GLOBAL_MOTOR_H
#define GLOBAL_MOTOR_H

#include "Config.h"
#include <stdint.h>
#include <stdbool.h>

/* Library Software Version */
#define MOTOR_LIBRARY_VERSION_OPT        0U
#define MOTOR_LIBRARY_VERSION_MAJOR      0U
#define MOTOR_LIBRARY_VERSION_MINOR      0U
#define MOTOR_LIBRARY_VERSION_BUGFIX     1U
#define MOTOR_LIBRARY_VERSION_ID         ((MOTOR_LIBRARY_VERSION_OPT << 24U) | (MOTOR_LIBRARY_VERSION_MAJOR << 16U) | (MOTOR_LIBRARY_VERSION_MINOR << 8U) | (MOTOR_LIBRARY_VERSION_BUGFIX))

/* Global Static Const  */
typedef const struct Global_Motor_Tag
{
    const uint16_t CONTROL_FREQ;
    const uint16_t V_MAX_VOLTS;         /* VSource Limit */
    const uint16_t V_ABC_R1;
    const uint16_t V_ABC_R2;
    const uint16_t I_MAX_ADCU;          /* Sensor calibration. Zero-To-Peak, derived from sensor hardware */
    const uint16_t I_MAX_AMPS;          /* Motor I controller rating. pass to Linear_ADC. Unit conversion, UI input/output, param set. */
    const uint16_t ALIGN_VPWM_MAX;
    const uint32_t CONTROL_ANALOG_DIVIDER;  /* In Pow2 - 1 */
    const uint8_t INIT_WAIT;
    // OpenLoopZcdTransition
}
Global_Motor_T;

/* MISRA violation */
/* Define in Main App */
extern const Global_Motor_T GLOBAL_MOTOR;

extern void Global_Motor_InitVSourceRef_V(uint16_t vSource);
extern uint16_t Global_Motor_GetVSource_V(void);

#endif
