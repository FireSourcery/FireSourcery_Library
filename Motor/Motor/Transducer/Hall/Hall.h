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
    @file   Hall.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef HALL_H
#define HALL_H

#include "Config.h"
#include "Peripheral/Pin/Pin.h"

#include <stdbool.h>
#include <stdint.h>

/* Virtual State Where ID => 0bCBA */
#define HALL_SENSORS_VIRTUAL_A      (0b001U)
#define HALL_SENSORS_VIRTUAL_B      (0b010U)
#define HALL_SENSORS_VIRTUAL_C      (0b100U)
/* 180 Degree active sensors */
#define HALL_SENSORS_VIRTUAL_INV_A  (0b110U)
#define HALL_SENSORS_VIRTUAL_INV_B  (0b101U)
#define HALL_SENSORS_VIRTUAL_INV_C  (0b011U)

#define HALL_SENSORS_TABLE_LENGTH (8U)

#define HALL_DIRECTION_STATE(idPrev, idNew) ((idPrev << 3U) | idNew)

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
typedef void (*Hall_CommutationPhase_T)(void * p_context);
#endif

typedef union Hall_Sensors
{
    struct
    {
        uint8_t A : 1U;
        uint8_t B : 1U;
        uint8_t C : 1U;
        uint8_t Resv : 5U;
    };
    uint8_t Value;
}
Hall_Sensors_T;

/*
    Hall sensor ID with aliases. ID base value reflects 3 bit sensor state, or sequential ID
    if both sequential ID, and sensor state is needed, another table must be used
*/
typedef enum Hall_Id
{
    /*
        Rotor Angle, CCW is increasing Angle
    */
    HALL_ANGLE_ERROR_0 = 0U,
    HALL_ANGLE_330_30 = HALL_SENSORS_VIRTUAL_A,
    HALL_ANGLE_30_90 = HALL_SENSORS_VIRTUAL_INV_C,
    HALL_ANGLE_90_150 = HALL_SENSORS_VIRTUAL_B,
    HALL_ANGLE_150_210 = HALL_SENSORS_VIRTUAL_INV_A,
    HALL_ANGLE_210_270 = HALL_SENSORS_VIRTUAL_C,
    HALL_ANGLE_270_330 = HALL_SENSORS_VIRTUAL_INV_B,
    HALL_ANGLE_ERROR_7 = 7U,

    /* Rotor Angle Id via boundary from CCW and CW */
    HALL_ANGLE_CCW_30 = HALL_ANGLE_30_90,
    HALL_ANGLE_CCW_90 = HALL_ANGLE_90_150,
    HALL_ANGLE_CCW_150 = HALL_ANGLE_150_210,
    HALL_ANGLE_CCW_210 = HALL_ANGLE_210_270,
    HALL_ANGLE_CCW_270 = HALL_ANGLE_270_330,
    HALL_ANGLE_CCW_330 = HALL_ANGLE_330_30,

    HALL_ANGLE_CW_30 = HALL_ANGLE_330_30,
    HALL_ANGLE_CW_90 = HALL_ANGLE_30_90,
    HALL_ANGLE_CW_150 = HALL_ANGLE_90_150,
    HALL_ANGLE_CW_210 = HALL_ANGLE_150_210,
    HALL_ANGLE_CW_270 = HALL_ANGLE_210_270,
    HALL_ANGLE_CW_330 = HALL_ANGLE_270_330,

    /*
        Commutation angle - CCW direction, 90 degree
    */
    HALL_COMMUTATION_PHASE_ERROR_0 = 0U,
    HALL_COMMUTATION_PHASE_AC = HALL_SENSORS_VIRTUAL_INV_B,
    HALL_COMMUTATION_PHASE_BC = HALL_SENSORS_VIRTUAL_A,
    HALL_COMMUTATION_PHASE_BA = HALL_SENSORS_VIRTUAL_INV_C,
    HALL_COMMUTATION_PHASE_CA = HALL_SENSORS_VIRTUAL_B,
    HALL_COMMUTATION_PHASE_CB = HALL_SENSORS_VIRTUAL_INV_A,
    HALL_COMMUTATION_PHASE_AB = HALL_SENSORS_VIRTUAL_C,
    HALL_COMMUTATION_PHASE_ERROR_7 = 7U,

    HALL_COMMUTATION_ANGLE_30 = HALL_COMMUTATION_PHASE_AC,
    HALL_COMMUTATION_ANGLE_90 = HALL_COMMUTATION_PHASE_BC,
    HALL_COMMUTATION_ANGLE_150 = HALL_COMMUTATION_PHASE_BA,
    HALL_COMMUTATION_ANGLE_210 = HALL_COMMUTATION_PHASE_CA,
    HALL_COMMUTATION_ANGLE_270 = HALL_COMMUTATION_PHASE_CB,
    HALL_COMMUTATION_ANGLE_330 = HALL_COMMUTATION_PHASE_AB,

    /*
        Sector number
    */
    // HALL_SECTOR_0 = 0U,
    // HALL_SECTOR_1 = HALL_SENSORS_VIRTUAL_A,
    // HALL_SECTOR_2 = HALL_SENSORS_VIRTUAL_INV_C,
    // HALL_SECTOR_3 = HALL_SENSORS_VIRTUAL_B,
    // HALL_SECTOR_4 = HALL_SENSORS_VIRTUAL_INV_A,
    // HALL_SECTOR_5 = HALL_SENSORS_VIRTUAL_C,
    // HALL_SECTOR_6 = HALL_SENSORS_VIRTUAL_INV_B,
    // HALL_SECTOR_7 = 7U,
    // HALL_SECTOR_8 = 0U,
}
Hall_Id_T;

typedef enum Hall_Direction
{
    HALL_DIRECTION_CW = -1,
    HALL_DIRECTION_UNKNOWN = 0,
    HALL_DIRECTION_CCW = 1,
}
Hall_Direction_T;

typedef struct Hall_Config
{
    Hall_Id_T SensorsTable[HALL_SENSORS_TABLE_LENGTH];
    // uint8_t BoundaryType; /* 3 states 120 degree active, or 6 step, 180 degree active */
}
Hall_Config_T;

typedef const struct Hall_Const
{
    const Hall_Config_T * const P_NVM_CONFIG;
}
Hall_Const_T;

typedef struct Hall
{
    const Hall_Const_T CONST;
    Pin_T PinA;
    Pin_T PinB;
    Pin_T PinC;
    Hall_Config_T Config;
    Hall_Sensors_T Sensors;     /* Save last physical read */
    Hall_Sensors_T SensorsPrev; /* Separate States for table */
    Hall_Direction_T Direction;
    uint16_t Angle;
#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
    Hall_CommutationPhase_T CommuntationTable[HALL_SENSORS_TABLE_LENGTH];
    void * p_CommutationContext;
#endif
}
Hall_T;

#define HALL_INIT(p_PinAHal, PinAId, p_PinBHal, PinBId, p_PinCHal, PinCId, p_Config)    \
{                                                               \
    .CONST =                                                    \
    {                                                           \
        .P_NVM_CONFIG = p_Config,                               \
    },                                                          \
    .PinA = PIN_INIT(p_PinAHal, PinAId),                        \
    .PinB = PIN_INIT(p_PinBHal, PinBId),                        \
    .PinC = PIN_INIT(p_PinCHal, PinCId),                        \
}

/******************************************************************************/
/* Stateless Conversions */
/******************************************************************************/

/* Six 60 degree boundaries, +/- 30 degrees */
/* Three 120 degree boundaries, +/- 60 degrees */
static const uint16_t _HALL_ANGLE_BOUNDARY = 5461U; /* 30 Degrees */

/* +180 degrees */
static inline uint8_t _Hall_Inverse(uint8_t sensors) { return (~sensors & 0x07U); }

/* Center of Hall sensor angle */
static inline uint16_t _Hall_Angle16Of(Hall_Id_T virtualId)
{
    static const uint16_t _HALL_ANGLE_TABLE[HALL_SENSORS_TABLE_LENGTH] =
    {
        [HALL_ANGLE_330_30]     = 0U,         /* 0 */
        [HALL_ANGLE_30_90]      = 10922U,     /* 60 */
        [HALL_ANGLE_90_150]     = 21845U,     /* 120 */
        [HALL_ANGLE_150_210]    = 32768U,     /* 180 */
        [HALL_ANGLE_210_270]    = 43690U,     /* 240 */
        [HALL_ANGLE_270_330]    = 54613U,     /* 300 */
        [HALL_ANGLE_ERROR_0]    = 0U,
        [HALL_ANGLE_ERROR_7]    = 0U,
    };

    return _HALL_ANGLE_TABLE[virtualId];
}

static inline uint16_t _Hall_Angle16BoundaryOf(Hall_Id_T virtualId, int16_t direction)
{
    return (_Hall_Angle16Of(virtualId) - ((int16_t)direction * _HALL_ANGLE_BOUNDARY));
}

static inline Hall_Direction_T _Hall_DirectionOf(Hall_Id_T idPrev, Hall_Id_T idNew)
{
    Hall_Direction_T direction;

    switch ((idPrev << 3U) | idNew)
    {
        case ((HALL_SENSORS_VIRTUAL_A << 3U)        | HALL_SENSORS_VIRTUAL_INV_C):  direction = HALL_DIRECTION_CCW; break;
        case ((HALL_SENSORS_VIRTUAL_INV_C << 3U)    | HALL_SENSORS_VIRTUAL_B):      direction = HALL_DIRECTION_CCW; break;
        case ((HALL_SENSORS_VIRTUAL_B << 3U)        | HALL_SENSORS_VIRTUAL_INV_A):  direction = HALL_DIRECTION_CCW; break;
        case ((HALL_SENSORS_VIRTUAL_INV_A << 3U)    | HALL_SENSORS_VIRTUAL_C):      direction = HALL_DIRECTION_CCW; break;
        case ((HALL_SENSORS_VIRTUAL_C << 3U)        | HALL_SENSORS_VIRTUAL_INV_B):  direction = HALL_DIRECTION_CCW; break;
        case ((HALL_SENSORS_VIRTUAL_INV_B << 3U)    | HALL_SENSORS_VIRTUAL_A):      direction = HALL_DIRECTION_CCW; break;

        case ((HALL_SENSORS_VIRTUAL_A << 3U)        | HALL_SENSORS_VIRTUAL_INV_B):  direction = HALL_DIRECTION_CW;  break;
        case ((HALL_SENSORS_VIRTUAL_INV_B << 3U)    | HALL_SENSORS_VIRTUAL_C):      direction = HALL_DIRECTION_CW;  break;
        case ((HALL_SENSORS_VIRTUAL_C << 3U)        | HALL_SENSORS_VIRTUAL_INV_A):  direction = HALL_DIRECTION_CW;  break;
        case ((HALL_SENSORS_VIRTUAL_INV_A << 3U)    | HALL_SENSORS_VIRTUAL_B):      direction = HALL_DIRECTION_CW;  break;
        case ((HALL_SENSORS_VIRTUAL_B << 3U)        | HALL_SENSORS_VIRTUAL_INV_C):  direction = HALL_DIRECTION_CW;  break;
        case ((HALL_SENSORS_VIRTUAL_INV_C << 3U)    | HALL_SENSORS_VIRTUAL_A):      direction = HALL_DIRECTION_CW;  break;

        default: direction = HALL_DIRECTION_UNKNOWN; break;
    }

    return direction;
}

/******************************************************************************/
/* Values with only Calibration State */
/******************************************************************************/
/* Virtual Id */
static inline Hall_Id_T Hall_IdOf(const Hall_T * p_hall, uint8_t physicalSensors) { return p_hall->Config.SensorsTable[physicalSensors]; }

/* Angle Approximation */
static inline uint16_t Hall_Angle16Of(const Hall_T * p_hall, uint8_t physicalSensors, int16_t direction)
{
    return _Hall_Angle16BoundaryOf(p_hall->Config.SensorsTable[physicalSensors], direction);
}

static inline uint16_t Hall_DirectionOf(const Hall_T * p_hall, uint8_t sensorsPrev, uint8_t sensorsNew)
{
    return  _Hall_DirectionOf(p_hall->Config.SensorsTable[sensorsPrev], p_hall->Config.SensorsTable[sensorsNew]);
}

// static inline uint16_t _Hall_GetAngle16(const Hall_T * p_hall) { return Hall_Angle16Of(p_hall, p_hall->Sensors.Value, p_hall->Direction); }

/******************************************************************************/
/* Capture State */
/******************************************************************************/
/*
    Physical Sensors
*/
static inline Hall_Sensors_T Hall_ReadSensors(const Hall_T * p_hall)
{
    Hall_Sensors_T sensors;
    sensors.A = Pin_Input_ReadPhysical(&p_hall->PinA);
    sensors.B = Pin_Input_ReadPhysical(&p_hall->PinB);
    sensors.C = Pin_Input_ReadPhysical(&p_hall->PinC);
    return sensors;
}

static inline void Hall_CaptureSensors_ISR(Hall_T * p_hall)
{
    p_hall->SensorsPrev.Value = p_hall->Sensors.Value;
    p_hall->Sensors.Value = Hall_ReadSensors(p_hall).Value;
}

/*
    Capture sensor on Hall edge, angle boundary
    return true on every phase edge. i.e 6x per Hall cycle
*/
static inline bool Hall_PollCaptureSensors(Hall_T * p_hall)
{
    uint8_t sensorsValue = Hall_ReadSensors(p_hall).Value;
    bool isEdge = (sensorsValue != p_hall->Sensors.Value);
    if (isEdge == true)
    {
        p_hall->SensorsPrev.Value = p_hall->Sensors.Value;
        p_hall->Sensors.Value = sensorsValue;
    }
    return (isEdge);
}


static inline Hall_Direction_T Hall_CaptureDirection(Hall_T * p_hall)
{
    // p_hall->DirectionPrev = p_hall->Direction;
    p_hall->Direction = Hall_DirectionOf(p_hall, p_hall->SensorsPrev.Value, p_hall->Sensors.Value);
    return p_hall->Direction;
}

/* Optionally store the angle */
static inline uint16_t Hall_CaptureAngle(Hall_T * p_hall)
{
    p_hall->Angle = Hall_Angle16Of(p_hall, p_hall->Sensors.Value, p_hall->Direction);
    return p_hall->Angle;
}

static inline void Hall_CaptureState_ISR(Hall_T * p_hall)
{
    Hall_CaptureSensors_ISR(p_hall);
    Hall_CaptureAngle(p_hall);
    Hall_CaptureDirection(p_hall);
}

static inline bool Hall_PollCaptureState(Hall_T * p_hall)
{
    bool isEdge = Hall_PollCaptureSensors(p_hall);
    if (isEdge == true)
    {
        Hall_CaptureAngle(p_hall);
        Hall_CaptureDirection(p_hall);
    }
    return (isEdge);
}

/*
    return true once per hall cycle
*/
static inline bool Hall_PollEdgeA(const Hall_T * p_hall) { return ((Hall_ReadSensors(p_hall).A == 1U) && ((p_hall->Sensors.A) == 0U)); }

/******************************************************************************/
/* Results from Capture */
/******************************************************************************/

static inline Hall_Sensors_T Hall_GetSensors(const Hall_T * p_hall) { return p_hall->Sensors; }
static inline Hall_Id_T Hall_GetId(const Hall_T * p_hall) { return Hall_IdOf(p_hall, p_hall->Sensors.Value); }
static inline uint16_t Hall_GetAngle16(const Hall_T * p_hall) { return p_hall->Angle; }

/* */
static inline Hall_Direction_T Hall_GetDirection(const Hall_T * p_hall) { return (p_hall->Direction); }
/* Sets direction => commutation, angle degrees16 conversion */
static inline void Hall_SetDirection(Hall_T * p_hall, Hall_Direction_T direction) { p_hall->Direction = direction; }

/* Next poll is edge */
static inline void Hall_ResetCapture(Hall_T * p_hall) { p_hall->Sensors.Value = 0U; p_hall->Angle = 0U; }

static inline void Hall_SetInitial(Hall_T * p_hall)
{
    Hall_CaptureSensors_ISR(p_hall);
    p_hall->Angle = Hall_Angle16Of(p_hall, p_hall->Sensors.Value, HALL_DIRECTION_UNKNOWN); /* assume middle */
}

/*
    Six-step commutation Id
    returns based on Direction
*/
static inline Hall_Id_T Hall_CommutationIdOf(const Hall_T * p_hall, uint8_t physicalSensors)
{
    return (p_hall->Direction == HALL_DIRECTION_CW) ? p_hall->Config.SensorsTable[_Hall_Inverse(physicalSensors)] : p_hall->Config.SensorsTable[physicalSensors]; /* Offset 180 */
}

static inline Hall_Id_T Hall_GetCommutationId(const Hall_T * p_hall) { return Hall_CommutationIdOf(p_hall, p_hall->Sensors.Value); }

/*
    Verify
*/
static inline bool Hall_Verify(const Hall_T * p_hall, uint8_t sensorsValue)
{
    return ((sensorsValue != HALL_ANGLE_ERROR_0) && (sensorsValue != HALL_ANGLE_ERROR_7));
}

static inline bool Hall_IsSensorsStateValid(const Hall_T * p_hall)
{
    return Hall_Verify(p_hall, Hall_ReadSensors(p_hall).Value);
}

static inline bool Hall_IsSensorsTableValid(const Hall_T * p_hall)
{
    bool isSuccess = true;
    for (uint8_t index = 1U; index < HALL_SENSORS_TABLE_LENGTH - 1U; index++) /* 1-6 */
    {
        if (Hall_Verify(p_hall, p_hall->Config.SensorsTable[index]) == false) { isSuccess = false; break; }
    }
    return isSuccess;
}

/******************************************************************************/
/*

*/
/******************************************************************************/
extern void Hall_Init(Hall_T * p_hall);
// extern void Hall_SetSensorsTable(Hall_T * p_hall, uint8_t sensorsA, uint8_t sensorsInvC, uint8_t sensorsB, uint8_t sensorsInvA, uint8_t sensorsC, uint8_t sensorsInvB);
extern void Hall_StartCalibrate(Hall_T * p_hall);
extern void Hall_CalibrateState(Hall_T * p_hall, Hall_Id_T calibratedId);

// extern void Hall_CalibratePhaseA(Hall_T * p_hall);
// extern void Hall_CalibratePhaseInvC(Hall_T * p_hall);
// extern void Hall_CalibratePhaseB(Hall_T * p_hall);
// extern void Hall_CalibratePhaseInvA(Hall_T * p_hall);
// extern void Hall_CalibratePhaseC(Hall_T * p_hall);
// extern void Hall_CalibratePhaseInvB(Hall_T * p_hall);


#endif

