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
#define    HALL_SENSORS_VIRTUAL_A         (0b001U)
#define    HALL_SENSORS_VIRTUAL_B         (0b010U)
#define    HALL_SENSORS_VIRTUAL_C         (0b100U)
/* 180 Degree active sensors */
#define    HALL_SENSORS_VIRTUAL_INV_A     (0b110U)
#define    HALL_SENSORS_VIRTUAL_INV_B     (0b101U)
#define    HALL_SENSORS_VIRTUAL_INV_C     (0b011U)

#define    HALL_SENSORS_TABLE_LENGTH     (8U)

#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
typedef void (*Hall_CommutationPhase_T)(void * p_userData);
#endif

typedef union Hall_Sensors
{
    struct
    {
        uint8_t A : 1U;
        uint8_t B : 1U;
        uint8_t C : 1U;
        uint8_t Resv3 : 1U;
        uint8_t Resv4 : 1U;
        uint8_t Resv5 : 1U;
        uint8_t Resv6 : 1U;
        uint8_t Resv7 : 1U;
    };
    uint8_t Value;
}
Hall_Sensors_T;

/*
    Hall sensor ID. ID base value reflects 3 bit sensor state, or sequential ID
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
    HALL_COMMUTATION_PHASE_AC = HALL_SENSORS_VIRTUAL_INV_B,
    HALL_COMMUTATION_PHASE_BC = HALL_SENSORS_VIRTUAL_A,
    HALL_COMMUTATION_PHASE_BA = HALL_SENSORS_VIRTUAL_INV_C,
    HALL_COMMUTATION_PHASE_CA = HALL_SENSORS_VIRTUAL_B,
    HALL_COMMUTATION_PHASE_CB = HALL_SENSORS_VIRTUAL_INV_A,
    HALL_COMMUTATION_PHASE_AB = HALL_SENSORS_VIRTUAL_C,
    HALL_COMMUTATION_PHASE_ERROR_0 = 0U,
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
    HALL_SECTOR_0 = 0U,
    HALL_SECTOR_1 = HALL_SENSORS_VIRTUAL_A,
    HALL_SECTOR_2 = HALL_SENSORS_VIRTUAL_INV_C,
    HALL_SECTOR_3 = HALL_SENSORS_VIRTUAL_B,
    HALL_SECTOR_4 = HALL_SENSORS_VIRTUAL_INV_A,
    HALL_SECTOR_5 = HALL_SENSORS_VIRTUAL_C,
    HALL_SECTOR_6 = HALL_SENSORS_VIRTUAL_INV_B,
    HALL_SECTOR_7 = 7U,
}
Hall_Id_T;

typedef enum Hall_Direction
{
    HALL_DIRECTION_CCW = 0U,
    HALL_DIRECTION_CW = 1U,
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
    // void (* const HAL_INIT)(void);
}
Hall_Const_T;

typedef struct Hall
{
    const Hall_Const_T CONST;
    Pin_T PinA;
    Pin_T PinB;
    Pin_T PinC;
    Hall_Config_T Config;
    Hall_Direction_T Direction;
    Hall_Sensors_T Sensors;        /* Save last physical read */
    volatile uint16_t Angle;
#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
    Hall_CommutationPhase_T CommuntationTable[HALL_SENSORS_TABLE_LENGTH];
    void * p_CommutationContext;
#endif
}
Hall_T;

#define HALL_INIT(p_PinAHal, PinAId, p_PinBHal, PinBId, p_PinCHal, PinCId, p_Config)    \
{                                                               \
    .CONST =                                                   \
    {                                                           \
        .P_NVM_CONFIG = p_Config,                               \
    },                                                          \
    .PinA = PIN_INIT(p_PinAHal, PinAId),                        \
    .PinB = PIN_INIT(p_PinBHal, PinBId),                        \
    .PinC = PIN_INIT(p_PinCHal, PinCId),                        \
}

extern const uint16_t _HALL_ANGLE_TABLE[HALL_SENSORS_TABLE_LENGTH];

/* +180 degrees */
static inline uint8_t InverseHall(uint8_t sensors) { return (~sensors & 0x07U); }

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
    if(isEdge == true) { p_hall->Sensors.Value = sensorsValue; }
    return (isEdge);
}


/*
    Virtual Id
*/
static inline Hall_Id_T Hall_IdOf(Hall_T * p_hall, uint8_t physicalSensors) { return p_hall->Config.SensorsTable[physicalSensors]; }


/*
    Angle Approximation
*/
static inline uint16_t _Hall_Angle16Of(Hall_T * p_hall, uint8_t physicalSensors)
{
    return _HALL_ANGLE_TABLE[p_hall->Config.SensorsTable[physicalSensors]];
}

/* returns based on Direction */
/* 60 degree boundaries, +/- 30 degrees, activate 120:90:60 degrees ahead, 90 degrees min to reach next boundary. handle outside */
/* 120 degree boundaries, +/- 60 degrees, activate 150:30 degrees ahead, 150 degrees to reach next boundary. handle outside */
static inline uint16_t Hall_Angle16Of(Hall_T * p_hall, uint8_t physicalSensors)
{
    uint16_t angle16 = _Hall_Angle16Of(p_hall, physicalSensors);
    // uint16_t boundary = (p_hall->Config.BoundaryType == 3U) ? 10922U : 5461U;
    uint16_t boundary = 5461U;
    return ((p_hall->Direction == HALL_DIRECTION_CW) ? angle16 + boundary : angle16 - boundary); /* unsigned angle wraps */
}

static inline void Hall_CaptureAngle_ISR(Hall_T * p_hall)
{
    Hall_CaptureSensors_ISR(p_hall);
    p_hall->Angle = Hall_Angle16Of(p_hall, p_hall->Sensors.Value);
}

static inline bool Hall_PollCaptureAngle(Hall_T * p_hall)
{
    bool isEdge = Hall_PollCaptureSensors(p_hall);
    if(isEdge == true) { p_hall->Angle = Hall_Angle16Of(p_hall, p_hall->Sensors.Value); }
    return (isEdge);
}


/*
    return true once per hall cycle
*/
static inline bool Hall_PollEdgeA(Hall_T * p_hall) { return ((Hall_ReadSensors(p_hall).A == true) && ((p_hall->Sensors.A) == false)); }

/* Next poll is edge */
static inline void Hall_ResetCapture(Hall_T * p_hall) { p_hall->Sensors.Value = 0U; p_hall->Angle = 0U; }

static inline void Hall_SetInitial(Hall_T * p_hall) { Hall_CaptureAngle_ISR(p_hall); }

/*
    Six-step commutation Id
    returns based on Direction
*/
static inline Hall_Id_T Hall_CommutationIdOf(Hall_T * p_hall, uint8_t physicalSensors)
{
    return (p_hall->Direction == HALL_DIRECTION_CW) ? p_hall->Config.SensorsTable[InverseHall(physicalSensors)] : p_hall->Config.SensorsTable[physicalSensors]; /* Offset 180 */
}
static inline Hall_Id_T Hall_GetCommutationId(Hall_T * p_hall) { return Hall_CommutationIdOf(p_hall, p_hall->Sensors.Value); }

static inline Hall_Direction_T Hall_GetDirection(Hall_T * p_hall) { return (p_hall->Direction); }
/* Sets direction => commutation, angle degrees16 conversion */
static inline void Hall_SetDirection(Hall_T * p_hall, Hall_Direction_T dir) { p_hall->Direction = dir; }
static inline void Hall_ToggleDirection(Hall_T * p_hall) { p_hall->Direction = ~p_hall->Direction; }

static inline Hall_Sensors_T Hall_GetSensors(Hall_T * p_hall) { return p_hall->Sensors; }
static inline uint8_t Hall_GetSensorsValue(Hall_T * p_hall) { return p_hall->Sensors.Value; }
static inline Hall_Id_T Hall_GetId(Hall_T * p_hall) { return Hall_IdOf(p_hall, p_hall->Sensors.Value); }
static inline uint16_t Hall_GetAngle16(Hall_T * p_hall) { return p_hall->Angle; }
static inline bool Hall_GetSensorA(Hall_T * p_hall) { return p_hall->Sensors.A; }
static inline bool Hall_GetSensorB(Hall_T * p_hall) { return p_hall->Sensors.B; }
static inline bool Hall_GetSensorC(Hall_T * p_hall) { return p_hall->Sensors.C; }

/******************************************************************************/
/*

*/
/******************************************************************************/
extern void Hall_Init(Hall_T * p_hall);
// extern void Hall_SetSensorsTable(Hall_T * p_hall, uint8_t sensorsA, uint8_t sensorsInvC, uint8_t sensorsB, uint8_t sensorsInvA, uint8_t sensorsC, uint8_t sensorsInvB);
extern void Hall_StartCalibrate(Hall_T * p_hall);
extern void Hall_CalibratePhaseA(Hall_T * p_hall);
extern void Hall_CalibratePhaseInvC(Hall_T * p_hall);
extern void Hall_CalibratePhaseB(Hall_T * p_hall);
extern void Hall_CalibratePhaseInvA(Hall_T * p_hall);
extern void Hall_CalibratePhaseC(Hall_T * p_hall);
extern void Hall_CalibratePhaseInvB(Hall_T * p_hall);

#endif
