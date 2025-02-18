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
    p_hall->Direction = HALL_DIRECTION_CCW;
}

void Hall_SetSensorsTable(Hall_T * p_hall, uint8_t sensorsA, uint8_t sensorsInvC, uint8_t sensorsB, uint8_t sensorsInvA, uint8_t sensorsC, uint8_t sensorsInvB)
{
    p_hall->Config.SensorsTable[sensorsA]       = HALL_SENSORS_VIRTUAL_A;
    p_hall->Config.SensorsTable[sensorsInvC]    = HALL_SENSORS_VIRTUAL_INV_C;
    p_hall->Config.SensorsTable[sensorsB]       = HALL_SENSORS_VIRTUAL_B;
    p_hall->Config.SensorsTable[sensorsInvA]    = HALL_SENSORS_VIRTUAL_INV_A;
    p_hall->Config.SensorsTable[sensorsC]       = HALL_SENSORS_VIRTUAL_C;
    p_hall->Config.SensorsTable[sensorsInvB]    = HALL_SENSORS_VIRTUAL_INV_B;
    p_hall->Config.SensorsTable[0U] = 0U;
    p_hall->Config.SensorsTable[7U] = 7U;
}

/*
    180 Degree Active
    Sensors are aligned to motor phase, 0 degree offset.
*/
static void CalibratePhaseA(Hall_T * p_hall)    { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = HALL_SENSORS_VIRTUAL_A; }
static void CalibratePhaseInvC(Hall_T * p_hall) { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = HALL_SENSORS_VIRTUAL_INV_C; }
static void CalibratePhaseB(Hall_T * p_hall)    { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = HALL_SENSORS_VIRTUAL_B; }
static void CalibratePhaseInvA(Hall_T * p_hall) { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = HALL_SENSORS_VIRTUAL_INV_A; }
static void CalibratePhaseC(Hall_T * p_hall)    { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = HALL_SENSORS_VIRTUAL_C; }
static void CalibratePhaseInvB(Hall_T * p_hall) { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = HALL_SENSORS_VIRTUAL_INV_B; }


/* For 180 degree active. 120 degree active todo */
void Hall_StartCalibrate(Hall_T * p_hall)       { Hall_ResetCapture(p_hall); /* p_hall->Config.BoundaryType = 0U; */ }
void Hall_CalibratePhaseA(Hall_T * p_hall)      { if (Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseA(p_hall); /* p_hall->Config.BoundaryType++; */ } }
void Hall_CalibratePhaseInvC(Hall_T * p_hall)   { if (Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseInvC(p_hall); /* p_hall->Config.BoundaryType++; */ } }
void Hall_CalibratePhaseB(Hall_T * p_hall)      { if (Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseB(p_hall); /* p_hall->Config.BoundaryType++; */ } }
void Hall_CalibratePhaseInvA(Hall_T * p_hall)   { if (Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseInvA(p_hall); /* p_hall->Config.BoundaryType++; */ } }
void Hall_CalibratePhaseC(Hall_T * p_hall)      { if (Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseC(p_hall); /* p_hall->Config.BoundaryType++; */ } }
void Hall_CalibratePhaseInvB(Hall_T * p_hall)   { if (Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseInvB(p_hall); /* p_hall->Config.BoundaryType++; */ } }


void Hall_CalibrateStateAs(Hall_T * p_hall, Hall_Id_T calibratedId) { if (Hall_PollCaptureSensors(p_hall) == true) { p_hall->Config.SensorsTable[Hall_ReadSensors(p_hall).Value] = calibratedId; } }
