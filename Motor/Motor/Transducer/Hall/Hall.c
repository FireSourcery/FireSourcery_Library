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

const uint16_t _HALL_ANGLE_TABLE[HALL_SENSORS_TABLE_LENGTH] =
{
    [HALL_ANGLE_330_30]     = 0U,         /* 0 */
    [HALL_ANGLE_30_90]      = 10922U,     /* 60 */
    [HALL_ANGLE_90_150]     = 21845U,     /* 120 */
    [HALL_ANGLE_150_210]    = 32768U,     /* 180 */
    [HALL_ANGLE_210_270]    = 43690U,     /* 240 */
    [HALL_ANGLE_270_330]    = 54613U,     /* 300 */
};

/*
    PinA,B,C HAL initialized in main app
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
void Hall_CalibratePhaseA(Hall_T * p_hall)      { if(Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseA(p_hall); /* p_hall->Config.BoundaryType++; */ } }
void Hall_CalibratePhaseInvC(Hall_T * p_hall)   { if(Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseInvC(p_hall); /* p_hall->Config.BoundaryType++; */ } }
void Hall_CalibratePhaseB(Hall_T * p_hall)      { if(Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseB(p_hall); /* p_hall->Config.BoundaryType++; */ } }
void Hall_CalibratePhaseInvA(Hall_T * p_hall)   { if(Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseInvA(p_hall); /* p_hall->Config.BoundaryType++; */ } }
void Hall_CalibratePhaseC(Hall_T * p_hall)      { if(Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseC(p_hall); /* p_hall->Config.BoundaryType++; */ } }
void Hall_CalibratePhaseInvB(Hall_T * p_hall)   { if(Hall_PollCaptureSensors(p_hall) == true) { CalibratePhaseInvB(p_hall); /* p_hall->Config.BoundaryType++; */ } }

// void Hall_Calibrate
// (
//     Hall_T * p_hall,
//     (void)(*activePhase)(void * p_phaseContext, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC),
//     void * p_phaseContext,
//     (void)(*wait)(void * p_waitContext),
//     void * p_waitContext
// )
// {
//     const uint16_t duty = p_motor->Config.AlignPower_ScalarU16;
//     bool isComplete = false;

//     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     {
//         switch (p_motor->CalibrationStateIndex)
//         {
//         case 0U:
//             Phase_ActivateDuty(&p_motor->Phase, duty, 0U, 0U);
//             p_motor->CalibrationStateIndex = 1U;
//             break;

//         case 1U:
//             Hall_CalibratePhaseA(&p_motor->Hall);
//             Phase_ActivateDuty(&p_motor->Phase, duty, duty, 0U);
//             p_motor->CalibrationStateIndex = 2U;
//             break;

//         case 2U:
//             Hall_CalibratePhaseInvC(&p_motor->Hall);
//             Phase_ActivateDuty(&p_motor->Phase, 0U, duty, 0U);
//             p_motor->CalibrationStateIndex = 3U;
//             break;

//         case 3U:
//             Hall_CalibratePhaseB(&p_motor->Hall);
//             Phase_ActivateDuty(&p_motor->Phase, 0U, duty, duty);
//             p_motor->CalibrationStateIndex = 4U;
//             break;

//         case 4U:
//             Hall_CalibratePhaseInvA(&p_motor->Hall);
//             Phase_ActivateDuty(&p_motor->Phase, 0U, 0U, duty);
//             p_motor->CalibrationStateIndex = 5U;
//             break;

//         case 5U:
//             Hall_CalibratePhaseC(&p_motor->Hall);
//             Phase_ActivateDuty(&p_motor->Phase, duty, 0U, duty);
//             p_motor->CalibrationStateIndex = 6U;
//             break;

//         case 6U:
//             Hall_CalibratePhaseInvB(&p_motor->Hall);
//             Phase_Float(&p_motor->Phase);
//             isComplete = true;
//             break;

//         default:
//             break;
//         }
//     }

//     return isComplete;
// }