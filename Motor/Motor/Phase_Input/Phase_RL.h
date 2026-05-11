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
    @file   Phase_RL.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../Math/motor_electrical_math.h"
#include "../Phase_Input/Phase_Calibration.h"

#include "../Motor_ControlFreq.h"


typedef struct
{
    uint16_t Rs;
    uint16_t Ls;
}
Phase_RL_T;

/* L requires base time config. alternatively move Motor_ControlFreq */
static inline Phase_RL_T Phase_RL_Fract16OfSi(uint16_t rs_mOhms, uint16_t ls_uHenries)
{
    return (Phase_RL_T)
    {
        .Rs = rs_pu_of_mohm(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), rs_mOhms),
        .Ls = l_pu_of_uh(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), ls_uHenries)
    };
}

static inline Phase_RL_T Phase_RL_SiOfFract16(uint16_t rs, uint16_t ls)
{
    return (Phase_RL_T)
    {
        .Rs = rs_mohm_of_pu(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), rs),
        .Ls = l_uh_of_pu(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), ls)
    };
}