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
    @file   Motor_Encoder.h
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include "../../Motor_StateMachine.h"
#include "../../Motor.h"
// #include "../../Motor_Var.h"
// #include "../../Motor_FOC.h"
// #include "../MotorSensor.h"
#include "Transducer/Encoder/Encoder_ModeDT.h"


void Motor_Encoder_StartHoming(Motor_State_T * p_motor);
void Motor_Encoder_CalibrateHomeOffset(Motor_State_T * p_motor);

void Motor_Encoder_StartUpChain(Motor_State_T * p_motor);

/*  */
extern int32_t _Motor_VarConfig_Encoder_Get(const Motor_State_T * p_motor, Encoder_ConfigId_T varId);
extern void _Motor_VarConfig_Encoder_Set(Motor_State_T * p_motor, Encoder_ConfigId_T varId, int32_t varValue);

extern int Motor_VarConfig_Encoder_Get(const Motor_State_T * p_motor, int varId);
extern void Motor_VarConfig_Encoder_Set(Motor_State_T * p_motor, int varId, int varValue);

#endif

