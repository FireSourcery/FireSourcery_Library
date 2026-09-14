#pragma once

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
    @file   UserAIn.h
    @author FireSourcery
    @brief  Analog input with optional digital enable pin and edge detection
            Potentiometer polling and edge detection
*/
/******************************************************************************/
#include "UserDIn.h"
#include "Math/Linear/Linear_Q16.h"

#include <stdint.h>
#include <stdbool.h>

/*!
    @brief Configuration for UserAIn module
*/
typedef struct UserAIn_Config
{
    /* Determine Linear_T Units at runtime */
    uint16_t AdcZero;                   /* Minimum ADC value for 0% */
    uint16_t AdcMax;                    /* Maximum ADC value for 100% */
    bool UseEdgePin;            /* User option. */
    // bool IsEnabled;                  /* Software enable/disable */
    // uint16_t Threshold;
    // uint16_t FilterShift;
    // uint16_t Hysteresis;
}
UserAIn_Config_T;

/******************************************************************************/
/*
    Runtime State
*/
/******************************************************************************/
typedef struct UserAIn_State
{
    uint16_t Value;                     /* Current filtered value. Percent16 by default */
    uint16_t ValuePrev;                 /* Previous value for edge detection */
    uint16_t RawValue_Adcu;             /* Raw ADC reading */
    Linear_T Units;                     /* ADC to percentage conversion */
    UserAIn_Config_T Config;            /* Hold for runtime updates */
}
UserAIn_State_T;

/******************************************************************************/
/*
    Context as compile time constant
*/
/******************************************************************************/
typedef const struct UserAIn
{
    UserDIn_T * P_EDGE_PIN;                 /* Optional digital pin acts as enable gate for analog capture, and edge source for edge detection. */
    UserAIn_State_T * P_STATE;
    const UserAIn_Config_T * P_NVM_CONFIG;  /* Configuration for ADC to percentage conversion */
    uint8_t FILTER_SHIFT;                   /* Filtering Ratio */
    // const volatile uint16_t * P_ADC_VALUE; /* optionally Pointer to ADC register/value */
}
UserAIn_T;

#define USER_AIN_STATE_ALLOC() (&(UserAIn_State_T){0})

/* Handle P_EDGE_PIN->P_HAL_PIN == NULL as empty opject */

/******************************************************************************/
/*
    Private Helper Functions
*/
/******************************************************************************/
static inline bool _UserAIn_IsEdgePinPassthrough(const UserDIn_T * p_pin) { return (p_pin == NULL) || UserDIn_Modal_IsDisable(p_pin) || UserDIn_GetState(p_pin); }
static inline bool _UserAIn_IsEdgePinEnabled(const UserDIn_T * p_pin) { return (p_pin != NULL) && !UserDIn_Modal_IsDisable(p_pin); }
static inline bool _UserAIn_IsEdgePinOn(const UserDIn_T * p_pin) { return (p_pin != NULL) && UserDIn_GetState(p_pin); } /* for diagnostics */

// skip null check with modal mode UserDIn_T EDGE_PIN config as empty object
// static inline bool _UserAIn_IsEdgePinPassthrough(const UserDIn_T * p_pin) { return UserDIn_Modal_IsDisable(p_pin) || UserDIn_GetState(p_pin); }
// static inline bool _UserAIn_IsEdgePinEnabled(const UserDIn_T * p_pin) { return !UserDIn_Modal_IsDisable(p_pin); }


/******************************************************************************/
/*
    State Query Functions
*/
/******************************************************************************/
/*
*/
static inline bool _UserAIn_IsOn(const UserAIn_State_T * p_state) { return (p_state->Value > 0U); }
static inline uint16_t _UserAIn_GetValue(const UserAIn_State_T * p_state) { return p_state->Value; }

/* Edge as threshold */
static inline bool UserAIn_IsOn(const UserAIn_T * p_dev) { return _UserAIn_IsEdgePinPassthrough(p_dev->P_EDGE_PIN) ? _UserAIn_IsOn(p_dev->P_STATE) : false; }

/*! @return Percent16 by default */
/* Check IsOn on get, rather than overwrite 0 when off, Value remains prev captured value */
static inline uint16_t UserAIn_GetValue(const UserAIn_T * p_dev) { return _UserAIn_IsEdgePinPassthrough(p_dev->P_EDGE_PIN) ? _UserAIn_GetValue(p_dev->P_STATE) : 0; }

/*
    Analog value only substate without EdgePin
    Valid for full state capture, or handle EdgePin in getter functions
*/
static inline bool _UserAIn_IsRisingEdge(const UserAIn_State_T * p_state) { return (p_state->ValuePrev <= 0U) && (p_state->Value > 0U); }
static inline bool _UserAIn_IsFallingEdge(const UserAIn_State_T * p_state) { return (p_state->ValuePrev > 0U) && (p_state->Value <= 0U); }
static inline bool _UserAIn_IsEdge(const UserAIn_State_T * p_state) { return (_UserAIn_IsRisingEdge(p_state) || _UserAIn_IsFallingEdge(p_state)); }

/*!
    Edge detection - considers EdgePin status
    @brief Check for edge without polling (query current state only)
    @note Uses digital pin edge if present, otherwise analog threshold edge
*/
static inline bool UserAIn_IsRisingEdge(const UserAIn_T * p_dev) { return _UserAIn_IsEdgePinEnabled(p_dev->P_EDGE_PIN) ? UserDIn_IsRisingEdge(p_dev->P_EDGE_PIN) : _UserAIn_IsRisingEdge(p_dev->P_STATE); }
static inline bool UserAIn_IsFallingEdge(const UserAIn_T * p_dev) { return _UserAIn_IsEdgePinEnabled(p_dev->P_EDGE_PIN) ? UserDIn_IsFallingEdge(p_dev->P_EDGE_PIN) : _UserAIn_IsFallingEdge(p_dev->P_STATE); }
static inline bool UserAIn_IsEdge(const UserAIn_T * p_dev) { return _UserAIn_IsEdgePinEnabled(p_dev->P_EDGE_PIN) ? UserDIn_IsEdge(p_dev->P_EDGE_PIN) : _UserAIn_IsEdge(p_dev->P_STATE); }


/******************************************************************************/
/*
    Public Functions
*/
/******************************************************************************/
extern void UserAIn_InitFrom(const UserAIn_T * p_dev, const UserAIn_Config_T * p_config);
extern void UserAIn_Init(const UserAIn_T * p_dev);

extern void UserAIn_ReinitScale(const UserAIn_T * p_dev);
extern void UserAIn_ApplyConfig(const UserAIn_T * p_dev);

/* Polling functions */
extern void UserAIn_CaptureValue(const UserAIn_T * p_dev, uint16_t value_adcu);
extern bool UserAIn_PollEdge(const UserAIn_T * p_dev, uint16_t value_adcu);
extern bool UserAIn_PollRisingEdge(const UserAIn_T * p_dev, uint16_t value_adcu);
extern bool UserAIn_PollFallingEdge(const UserAIn_T * p_dev, uint16_t value_adcu);



// static inline int _UserAIn_Var_Get(const UserAIn_T * p_dev, int id)
// {
//     int32_t value = 0;
//     switch (id)
//     {
//         case USER_AIN_VALUE:           value = UserAIn_GetValue(p_dev);                            break;
//         case USER_AIN_IS_ON:          value = UserAIn_IsOn(p_dev);                                   break;
//         case USER_AIN_GATE:        value = _UserAIn_IsEdgePinOn(p_dev->P_EDGE_PIN);                                   break;
//         default: break;
//     }
//     return value;
// }


typedef enum UserAIn_ConfigId
{
    USER_AIN_ZERO_ADCU,
    USER_AIN_MAX_ADCU,
    USER_AIN_EDGE_PIN_IS_ENABLE,
}
UserAIn_ConfigId_T;

static inline int _UserAIn_Config_Get(const UserAIn_Config_T * p_config, UserAIn_ConfigId_T configId)
{
    switch (configId)
    {
        case USER_AIN_ZERO_ADCU:            return p_config->AdcZero;
        case USER_AIN_MAX_ADCU:             return p_config->AdcMax;
        case USER_AIN_EDGE_PIN_IS_ENABLE:   return p_config->UseEdgePin;
        default: return 0;
    }
}

static inline void _UserAIn_Config_Set(UserAIn_Config_T * p_config, UserAIn_ConfigId_T configId, int value)
{
    switch (configId)
    {
        case USER_AIN_ZERO_ADCU:            p_config->AdcZero = (uint16_t)value;    break;
        case USER_AIN_MAX_ADCU:             p_config->AdcMax = (uint16_t)value;     break;
        case USER_AIN_EDGE_PIN_IS_ENABLE:   p_config->UseEdgePin = (bool)value;     break;
        default: break;
    }
}

/*
    Config is owned by the device, loaded from P_NVM_CONFIG on init.
    Set propagates: ADC bounds rescale the linear units, UseEdgePin re-gates the edge pin.
*/
static inline int UserAIn_Config_Get(const UserAIn_T * p_dev, UserAIn_ConfigId_T configId) { return _UserAIn_Config_Get(&p_dev->P_STATE->Config, configId); }

static inline void UserAIn_Config_Set(const UserAIn_T * p_dev, UserAIn_ConfigId_T configId, int value)
{
    _UserAIn_Config_Set(&p_dev->P_STATE->Config, configId, value);
    UserAIn_ApplyConfig(p_dev);
}

static inline int UserAIn_Config_GetInstance(const UserAIn_T * p_array, uint8_t length, uint8_t instance, UserAIn_ConfigId_T configId)
{
    if (instance >= length) { return 0; }
    return UserAIn_Config_Get(&p_array[instance], configId);
}

static inline void UserAIn_Config_SetInstance(const UserAIn_T * p_array, uint8_t length, uint8_t instance, UserAIn_ConfigId_T configId, int value)
{
    if (instance >= length) { return; }
    UserAIn_Config_Set(&p_array[instance], configId, value);
}
