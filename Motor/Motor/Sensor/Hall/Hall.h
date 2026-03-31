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

*/
/******************************************************************************/
#ifndef HALL_H
#define HALL_H

#include "Math/math_general.h"
#include "Peripheral/Pin/Pin.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!

*/
/******************************************************************************/
/* Virtual State Where ID => 0bCBA */
#define HALL_SENSORS_VIRTUAL_A      (0b001U)
#define HALL_SENSORS_VIRTUAL_B      (0b010U)
#define HALL_SENSORS_VIRTUAL_C      (0b100U)
/* 180 Degree active sensors */
#define HALL_SENSORS_VIRTUAL_INV_A  (0b110U)
#define HALL_SENSORS_VIRTUAL_INV_B  (0b101U)
#define HALL_SENSORS_VIRTUAL_INV_C  (0b011U)

#define HALL_SENSORS_TABLE_LENGTH (8U)

#define HALL_ID_BITS (3U)

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
    A->B->C as positive/CCW by convention. This is determined by the order of the virtual sensor state defined above.
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
    // uint8_t BoundaryType;
}
Hall_Config_T;

typedef struct Hall_State
{
    Hall_Config_T Config;
    Hall_Sensors_T Sensors;     /* Save last physical read */
    Hall_Id_T Id;               /* resolved */
    Hall_Id_T IdPrev;           /* resolved */
    Hall_Direction_T Direction;
    uint16_t Angle;
}
Hall_State_T;

typedef const struct Hall
{
    Pin_T PIN_A;
    Pin_T PIN_B;
    Pin_T PIN_C;
    // HAL_Timer_T * P_HAL_TIMER;    /*!< DeltaT Timer. */
    // uint32_t TIMER_FREQ;                          /*!< DeltaT Timer Freq. Divisible by SAMPLE_FREQ */
    Hall_State_T * P_STATE;
    const Hall_Config_T * P_NVM_CONFIG;
}
Hall_T;

#define HALL_STATE_ALLOC() (&(Hall_State_T){0})

/*  */
#define HALL_INIT(PinA, PinB, PinC, p_State, p_Config) \
    { .PIN_A = PinA, .PIN_B = PinB, .PIN_C = PinC, .P_NVM_CONFIG = (p_Config), .P_STATE = (p_State), }

/* Init from primitive */
#define HALL_INIT_FROM(p_PinAHal, PinAId, p_PinBHal, PinBId, p_PinCHal, PinCId, p_State, p_Config) \
{                                                                \
    .PIN_A = PIN_INIT(p_PinAHal, PinAId),                        \
    .PIN_B = PIN_INIT(p_PinBHal, PinBId),                        \
    .PIN_C = PIN_INIT(p_PinCHal, PinCId),                        \
    .P_STATE = (p_State),                                        \
    .P_NVM_CONFIG = (p_Config),                                  \
}

// #define HALL_ALLOC(p_PinAHal, PinAId, p_PinBHal, PinBId, p_PinCHal, PinCId, p_Config) \
//     HALL_INIT_FROM(p_PinAHal, PinAId, p_PinBHal, PinBId, p_PinCHal, PinCId, HALL_STATE_ALLOC(), p_Config)


/******************************************************************************/
/*
    Stateless Conversions
*/
/******************************************************************************/
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

static inline uint16_t _Hall_Angle16BoundaryOf(Hall_Id_T virtualId, Hall_Direction_T direction)
{
    static const uint16_t _HALL_ANGLE_BOUNDARY = 5461U; /* +/- 30 degrees, from six 60 degree boundaries */

    return (_Hall_Angle16Of(virtualId) - ((int16_t)direction * _HALL_ANGLE_BOUNDARY)); /* minus on CCW. Boundary occurs prior to midpoint */
}

#define HALL_DIRECTION_STATE(idPrev, idNew) ((idPrev << HALL_ID_BITS) | idNew)

static inline Hall_Direction_T _Hall_DirectionOf_Lut(Hall_Id_T idPrev, Hall_Id_T idNew)
{
    static const int8_t TABLE[64] = {
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_A,     HALL_SENSORS_VIRTUAL_INV_C)] =  1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_INV_C, HALL_SENSORS_VIRTUAL_B)]     =  1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_B,     HALL_SENSORS_VIRTUAL_INV_A)] =  1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_INV_A, HALL_SENSORS_VIRTUAL_C)]     =  1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_C,     HALL_SENSORS_VIRTUAL_INV_B)] =  1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_INV_B, HALL_SENSORS_VIRTUAL_A)]     =  1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_A,     HALL_SENSORS_VIRTUAL_INV_B)] = -1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_INV_B, HALL_SENSORS_VIRTUAL_C)]     = -1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_C,     HALL_SENSORS_VIRTUAL_INV_A)] = -1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_INV_A, HALL_SENSORS_VIRTUAL_B)]     = -1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_B,     HALL_SENSORS_VIRTUAL_INV_C)] = -1,
        [HALL_DIRECTION_STATE(HALL_SENSORS_VIRTUAL_INV_C, HALL_SENSORS_VIRTUAL_A)]     = -1,
    };
    return (Hall_Direction_T)TABLE[HALL_DIRECTION_STATE(idPrev, idNew)];
}

/* Rated Electrical Angle Speed < ANGLE60/2 */
static inline Hall_Direction_T _Hall_DirectionOf_Sign(Hall_Id_T idPrev, Hall_Id_T idNew)
{
    return (Hall_Direction_T)math_sign((int16_t)(_Hall_Angle16Of(idNew) - _Hall_Angle16Of(idPrev))); /* subtraction wraps.  0 - 54613 wraps to 10923 */
}

static inline Hall_Direction_T _Hall_DirectionOf(Hall_Id_T idPrev, Hall_Id_T idNew)
{
    return _Hall_DirectionOf_Lut(idPrev, idNew);
}


/******************************************************************************/
/*
    Capture State
*/
/******************************************************************************/
/* Virtual Id - resolve physical sensors via calibration table */
static inline Hall_Id_T _Hall_IdOf(const Hall_State_T * p_hall, uint8_t physicalSensors) { return p_hall->Config.SensorsTable[physicalSensors]; }

/*
    Physical Sensors
*/
static inline Hall_Sensors_T Hall_ReadSensors(const Hall_T * p_hall)
{
    return (Hall_Sensors_T)
    {
        .A = Pin_Input_ReadPhysical(&p_hall->PIN_A),
        .B = Pin_Input_ReadPhysical(&p_hall->PIN_B),
        .C = Pin_Input_ReadPhysical(&p_hall->PIN_C),
    };
}

static inline void _Hall_CaptureSensors(Hall_State_T * p_state, Hall_Sensors_T sensors)
{
    // p_state->SensorsPrev.Value = p_state->Sensors.Value;
    p_state->Sensors.Value = sensors.Value;
    p_state->IdPrev = p_state->Id;
    p_state->Id = _Hall_IdOf(p_state, sensors.Value);
}

static inline void Hall_CaptureSensors_ISR(const Hall_T * p_hall)
{
    _Hall_CaptureSensors(p_hall->P_STATE, Hall_ReadSensors(p_hall));
}

/*!
    Capture sensor on Hall edge, angle boundary
    @return true on every phase edge. i.e 6x per Hall cycle
*/
static inline bool Hall_PollCaptureSensors(const Hall_T * p_hall)
{
    Hall_Sensors_T sensors = Hall_ReadSensors(p_hall);
    if (p_hall->P_STATE->Sensors.Value != sensors.Value)
    {
        _Hall_CaptureSensors(p_hall->P_STATE, sensors);
        return true;
    }
    else { return false; }
}

static inline Hall_Direction_T _Hall_CaptureEdge(Hall_State_T * p_state, Hall_Sensors_T sensors)
{
    if (p_state->Sensors.Value != sensors.Value) { _Hall_CaptureSensors(p_state, sensors); }
    return _Hall_DirectionOf(p_state->IdPrev, p_state->Id);
}

static inline Hall_Direction_T Hall_PollEdge(const Hall_T * p_hall)
{
    return _Hall_CaptureEdge(p_hall->P_STATE, Hall_ReadSensors(p_hall));
}

/*
    return true once per electrical cycle
*/
// static inline bool Hall_PollEdgeA(const Hall_T * p_hall) { return ((Hall_ReadSensors(p_hall).A == 1U) && (p_hall->P_STATE->Sensors.A == 0U)); }

/******************************************************************************/
/*
    Query from resolved IDs (stored at capture time)
*/
/******************************************************************************/
static inline Hall_Id_T Hall_GetId(const Hall_State_T * p_hall) { return p_hall->Id; }
static inline Hall_Direction_T Hall_GetSensorDirection(const Hall_State_T * p_hall) { return _Hall_DirectionOf(p_hall->IdPrev, p_hall->Id); }
static inline uint16_t Hall_GetAngleAs(const Hall_State_T * p_hall, Hall_Direction_T direction) { return _Hall_Angle16BoundaryOf(p_hall->Id, direction); }
static inline uint16_t Hall_GetSensorAngle(const Hall_State_T * p_hall) { return _Hall_Angle16BoundaryOf(p_hall->Id, Hall_GetSensorDirection(p_hall)); }

/*
   Module stores additional state
*/
/*
    Supplementary Capture with derived values
    alternatively caller store
*/
static inline Hall_Direction_T Hall_ResolveDirection(Hall_State_T * p_hall)
{
    p_hall->Direction = Hall_GetSensorDirection(p_hall);
    return p_hall->Direction;
}

/* always capture with the most recent direction. jitter is acounted for */
/* Store the angle for repeat read on interpolation */
static inline uint16_t Hall_ResolveAngle(Hall_State_T * p_hall)
{
    p_hall->Angle = _Hall_Angle16BoundaryOf(p_hall->Id, p_hall->Direction);
    return p_hall->Angle;
}

/* Next poll is edge */
// static inline void Hall_Reset(Hall_T * p_hall) { p_hall->Sensors.Value = 0U; p_hall->Angle = 0U; }
static inline void Hall_ZeroInitial(const Hall_T * p_hall)
{
    Hall_CaptureSensors_ISR(p_hall);
    p_hall->P_STATE->Angle = _Hall_Angle16BoundaryOf(p_hall->P_STATE->Id, HALL_DIRECTION_UNKNOWN); /* assume middle */
}


/*
    set external or capture
    Sets direction =>  angle degrees16 conversion
*/
// static inline void Hall_CaptureDirectionComp(Hall_State_T * p_hall, Hall_Direction_T direction) { p_hall->Direction = direction; }

/*

*/
static inline Hall_Sensors_T Hall_GetSensors(const Hall_State_T * p_hall) { return p_hall->Sensors; }
static inline uint16_t Hall_GetAngle16(const Hall_State_T * p_hall) { return p_hall->Angle; }
static inline Hall_Direction_T Hall_GetDirection(const Hall_State_T * p_hall) { return (p_hall->Direction); }
// static inline Hall_Id_T Hall_GetCommutationId(const Hall_State_T * p_hall) { return Hall_CommutationIdOf(p_hall, p_hall->Sensors.Value); }

/******************************************************************************/
/*

*/
/******************************************************************************/
extern void Hall_InitFrom(const Hall_T * p_hall, const Hall_Config_T * p_config);
extern void Hall_Init(const Hall_T * p_hall);

extern void Hall_StartCalibrate(const Hall_T * p_hall);
extern void Hall_CalibrateState(const Hall_T * p_hall, Hall_Id_T calibratedId);

extern bool Hall_Verify(uint8_t sensorsValue);
extern bool Hall_IsTableValid(const Hall_State_T * p_hall);
extern bool Hall_IsStateValid(const Hall_T * p_hall);


/*
    Id Interface
*/
typedef enum Hall_VarId
{
    HALL_VAR_SENSOR_STATE,
    HALL_VAR_SENSOR_ID,
}
Hall_VarId_T;

void Hall_VarId_Set(const Hall_T * p_hall, Hall_VarId_T varId, int varValue);
int Hall_VarId_Get(const Hall_T * p_hall, Hall_VarId_T varId);

typedef enum Hall_ConfigId
{
    HALL_CONFIG_SENSOR_TABLE_1,
    HALL_CONFIG_SENSOR_TABLE_2,
    HALL_CONFIG_SENSOR_TABLE_3,
    HALL_CONFIG_SENSOR_TABLE_4,
    HALL_CONFIG_SENSOR_TABLE_5,
    HALL_CONFIG_SENSOR_TABLE_6,
}
Hall_ConfigId_T;

extern int _Hall_ConfigId_Get(const Hall_State_T * p_hall, Hall_ConfigId_T varId);
extern void _Hall_ConfigId_Set(Hall_State_T * p_hall, Hall_ConfigId_T varId, int varValue);

void Hall_ConfigId_Set(const Hall_T * p_hall, Hall_ConfigId_T varId, int varValue);
int Hall_ConfigId_Get(const Hall_T * p_hall, Hall_ConfigId_T varId);

#endif

