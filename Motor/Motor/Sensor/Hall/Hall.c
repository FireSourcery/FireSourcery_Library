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

*/
/******************************************************************************/
#include "Hall.h"

#include <string.h>


void Hall_InitFrom(const Hall_T * p_hall, const Hall_Config_T * p_config)
{
    Pin_Input_Init(&p_hall->PIN_A);
    Pin_Input_Init(&p_hall->PIN_B);
    Pin_Input_Init(&p_hall->PIN_C);

    if (p_config != NULL) { memcpy(&p_hall->P_STATE->Config, p_config, sizeof(Hall_Config_T)); }
}

/*
    PinA, B, C HAL initialized in main app
*/
void Hall_Init(const Hall_T * p_hall)
{
    Hall_InitFrom(p_hall, p_hall->P_NVM_CONFIG);
}


/*
    180 Degree Active
    Sensors are aligned to motor phase, 0 degree offset.
*/
void Hall_StartCalibrate(const Hall_T * p_hall) { p_hall->P_STATE->Sensors.Value = 0U; } /* Next poll is edge */

void Hall_CalibrateState(const Hall_T * p_hall, Hall_Id_T calibratedId)
{
    p_hall->P_STATE->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = calibratedId;
}

/* disable isr */
// void Hall_CalibrateState(Hall_T * p_hall, Hall_Id_T calibratedId)
// {
//     if (Hall_PollCaptureSensors(p_hall) == true) { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = calibratedId; }
//     else { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = HALL_ANGLE_ERROR_0; }
// }

/*
    Verify
*/
bool Hall_Verify(uint8_t sensorsValue)
{
    return ((sensorsValue != HALL_ANGLE_ERROR_0) && (sensorsValue != HALL_ANGLE_ERROR_7));
}

/* Check wiring */
bool Hall_IsStateValid(const Hall_T * p_hall)
{
    return Hall_Verify(Hall_ReadSensors(p_hall).Value);
}

bool Hall_IsTableValid(const Hall_State_T * p_hall)
{
    bool valid = true;
    bool once[HALL_SENSORS_TABLE_LENGTH] = { false };

    for (uint8_t index = 1U; index < HALL_SENSORS_TABLE_LENGTH - 1U; index++) /* 1-6 */
    {
        if (Hall_Verify(p_hall->Config.SensorsTable[index]) == false) { valid = false; break; }
        if (once[p_hall->Config.SensorsTable[index]] == true) { valid = false; break; }
        once[p_hall->Config.SensorsTable[index]] = true;
    }

    return valid;
}

/*
    Id Interface
*/
// void _Hall_ConfigId_Set(Hall_Config_T * p_hall, Hall_ConfigId_T varId, int varValue)
void _Hall_ConfigId_Set(Hall_State_T * p_hall, Hall_ConfigId_T varId, int varValue)
{
    switch (varId)
    {
        case HALL_CONFIG_SENSOR_TABLE_1: p_hall->Config.SensorsTable[1U] = varValue; break;
        case HALL_CONFIG_SENSOR_TABLE_2: p_hall->Config.SensorsTable[2U] = varValue; break;
        case HALL_CONFIG_SENSOR_TABLE_3: p_hall->Config.SensorsTable[3U] = varValue; break;
        case HALL_CONFIG_SENSOR_TABLE_4: p_hall->Config.SensorsTable[4U] = varValue; break;
        case HALL_CONFIG_SENSOR_TABLE_5: p_hall->Config.SensorsTable[5U] = varValue; break;
        case HALL_CONFIG_SENSOR_TABLE_6: p_hall->Config.SensorsTable[6U] = varValue; break;
        // case HALL_CONFIG_RUN_CALIBRATION: break;
        default: break;
    }
}

int _Hall_ConfigId_Get(const Hall_State_T * p_hall, Hall_ConfigId_T varId)
{
    int value = 0;
    switch (varId)
    {
        case HALL_CONFIG_SENSOR_TABLE_1: value = p_hall->Config.SensorsTable[1U]; break;
        case HALL_CONFIG_SENSOR_TABLE_2: value = p_hall->Config.SensorsTable[2U]; break;
        case HALL_CONFIG_SENSOR_TABLE_3: value = p_hall->Config.SensorsTable[3U]; break;
        case HALL_CONFIG_SENSOR_TABLE_4: value = p_hall->Config.SensorsTable[4U]; break;
        case HALL_CONFIG_SENSOR_TABLE_5: value = p_hall->Config.SensorsTable[5U]; break;
        case HALL_CONFIG_SENSOR_TABLE_6: value = p_hall->Config.SensorsTable[6U]; break;
        // case HALL_CONFIG_RUN_CALIBRATION: break;
        default: break;
    }
    return value;
}

void Hall_ConfigId_Set(const Hall_T * p_hall, Hall_ConfigId_T varId, int varValue)
{
    if (p_hall == NULL) return;
    _Hall_ConfigId_Set(p_hall->P_STATE, varId, varValue);
}

int Hall_ConfigId_Get(const Hall_T * p_hall, Hall_ConfigId_T varId)
{
    if (p_hall == NULL) return 0;
    return _Hall_ConfigId_Get(p_hall->P_STATE, varId);
}

