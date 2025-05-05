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
    @file   Hall.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Hall.h"

#include <string.h>


/*
    PinA, B, C HAL initialized in main app
*/
void Hall_Init(Hall_T * p_hall)
{
    Pin_Input_Init(&p_hall->PinA);
    Pin_Input_Init(&p_hall->PinB);
    Pin_Input_Init(&p_hall->PinC);

    if(p_hall->CONST.P_NVM_CONFIG != 0U) { memcpy(&p_hall->Config, p_hall->CONST.P_NVM_CONFIG, sizeof(Hall_Config_T)); }
    // p_hall->Direction = HALL_DIRECTION_CCW;
}

// void Hall_Init(Hall_T * p_hall, Hall_Config_T * p_config)
// {
//     Pin_Input_Init(&p_hall->PinA);
//     Pin_Input_Init(&p_hall->PinB);
//     Pin_Input_Init(&p_hall->PinC);

//     if(p_config != NULL) { memcpy(&p_hall->Config, p_config, sizeof(Hall_Config_T)); }
//     // p_hall->Direction = HALL_DIRECTION_CCW;
// }

/*
    180 Degree Active
    Sensors are aligned to motor phase, 0 degree offset.
*/
void Hall_StartCalibrate(Hall_T * p_hall) { p_hall->Sensors.Value = 0U; } /* Next poll is edge */

void Hall_CalibrateState(Hall_T * p_hall, Hall_Id_T calibratedId)
{
    p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = calibratedId;
    /* disable isr */
    // if (Hall_PollCaptureSensors(p_hall) == true) { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = calibratedId; }
    // else { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = HALL_ANGLE_ERROR_0; }
}

void Hall_SetConfigId(Hall_T * p_hall, Hall_ConfigId_T varId, int32_t varValue)
{
    switch (varId)
    {
        case HALL_CONFIG_SENSOR_TABLE_1: p_hall->Config.SensorsTable[1U] = varValue; break;
        case HALL_CONFIG_SENSOR_TABLE_2: p_hall->Config.SensorsTable[2U] = varValue; break;
        case HALL_CONFIG_SENSOR_TABLE_3: p_hall->Config.SensorsTable[3U] = varValue; break;
        case HALL_CONFIG_SENSOR_TABLE_4: p_hall->Config.SensorsTable[4U] = varValue; break;
        case HALL_CONFIG_SENSOR_TABLE_5: p_hall->Config.SensorsTable[5U] = varValue; break;
        case HALL_CONFIG_SENSOR_TABLE_6: p_hall->Config.SensorsTable[6U] = varValue; break;
    }
}



