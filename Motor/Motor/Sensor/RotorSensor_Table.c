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
    @file   RotorSensor_Table.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "MotorSensor_Table.h"


/******************************************************************************/
/*!
    Var Id
*/
/******************************************************************************/
int MotorSensor_VarType_Get(const MotorSensor_Table_T * p_table, MotorSensor_Id_T typeId, int varId)
{
    switch (typeId)
    {
        // case ROTOR_SENSOR_ID_HALL:    return Hall_VarId_Get(&p_table->HALL.HALL, varId); break;
        case ROTOR_SENSOR_ID_ENCODER: return Enocder_ModeDT_VarId_Get(p_table->ENCODER.ENCODER.P_STATE, varId); break;
            // case ROTOR_SENSOR_ID_SIN_COS: return SinCos_VarId_Get(&p_table->SIN_COS.SIN_COS, varId); break;
            // case ROTOR_SENSOR_ID_SENSORLESS: return Sensorless_VarId_Get(&p_table->SENSORLESS.SENSORLESS, varId); break;
        default: return 0; // or some error value
    }
}

int MotorSensor_VarTypeConfig_Get(const MotorSensor_Table_T * p_table, MotorSensor_Id_T typeId, int varId)
{
    switch (typeId)
    {
        case ROTOR_SENSOR_ID_HALL:    return _Hall_ConfigId_Get(p_table->HALL.HALL.P_STATE, varId); break;
        case ROTOR_SENSOR_ID_ENCODER: return _Encoder_ConfigId_Get(p_table->ENCODER.ENCODER.P_STATE, varId); break;
    }
}

void MotorSensor_VarTypeConfig_Set(const MotorSensor_Table_T * p_table, MotorSensor_Id_T typeId, int varId, int varValue)
{
    switch (typeId)
    {
        case ROTOR_SENSOR_ID_HALL:
            // if (varId == HALL_CONFIG_RUN_CALIBRATION) { Motor_Hall_Calibrate(&p_table->HALL.HALL); return; }
            Hall_ConfigId_Set(&p_table->HALL.HALL, varId, varValue);
            break;

        case ROTOR_SENSOR_ID_ENCODER:
            Encoder_ConfigId_Set(&p_table->ENCODER.ENCODER, varId, varValue);
            break;
        default: break;
    }
}