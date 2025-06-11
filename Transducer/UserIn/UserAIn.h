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
#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Q16.h"

#include <stdint.h>
#include <stdbool.h>

/*!
    @brief Configuration for UserAIn module
*/
typedef struct UserAIn_Config
{
    /* determine Linear_T Units at runtime */
    uint16_t AdcZero;                   /* Minimum ADC value for 0% */
    uint16_t AdcMax;                    /* Maximum ADC value for 100% */
    // bool UseEdgePin;
    // uint16_t Threshold;
    // uint16_t FilterShift;
    // uint16_t Histeresis;
}
UserAIn_Config_T;

/******************************************************************************/
/*
    Runtime State
*/
/******************************************************************************/
typedef struct UserAIn_State
{
    Linear_T Units;                     /* ADC to percentage conversion */
    uint16_t Value;                     /* Current filtered value. Percent16 by default */
    uint16_t ValuePrev;                 /* Previous value for edge detection */
    uint16_t RawValue_Adcu;             /* Raw ADC reading */
    // bool IsEnabled;                  /* Software enable/disable */
    // uint16_t Threshold;
    UserAIn_Config_T Config;            /* Hold for runtime updates */
    // uint16_t FilterShift;
}
UserAIn_State_T;

/******************************************************************************/
/*
    Context as compile time constant
*/
/******************************************************************************/
typedef const struct UserAIn
{
    // optionally direct pointer
    // const volatile uint16_t * P_ADC_VALUE; /* Pointer to ADC register/value */
    /* Digital pin acts as enable gate for analog capture, and edge source for edge detection. */
    const UserDIn_T * P_EDGE_PIN;       /* Optional digital pin for threshold/enable */
    // const UserDIn_T EDGE_PIN;
    UserAIn_State_T * P_STATE;
    uint8_t FILTER_SHIFT;                   /* Filtering Ratio */
    const UserAIn_Config_T * P_NVM_CONFIG;  /* Configuration for ADC to percentage conversion */
}
UserAIn_T;

#define USER_AIN_STATE_ALLOC() (&(UserAIn_State_T){0})

#define USER_AIN_INIT(p_EdgePin, p_State, Filter, p_Config) \
    { .P_EDGE_PIN = p_EdgePin, .P_STATE = p_State, .FILTER_SHIFT = Filter, .P_NVM_CONFIG = p_Config, }

#define USER_AIN_ALLOC(p_EdgePin, Filter, p_Config) USER_AIN_INIT(p_EdgePin, &(UserAIn_State_T){0}, Filter, p_Config)

/******************************************************************************/
/*
    Private Helper Functions
*/
/******************************************************************************/
static inline bool _UserAIn_IsEdgePinPassthrough(const UserDIn_T * p_pin) { return (p_pin == NULL) || UserDIn_GetState(p_pin); }

/*
    Analog value only substate without EdgePin
    Valid for full state capture, or handle EdgePin in getter functions
*/
static inline bool _UserAIn_IsRisingEdge(const UserAIn_State_T * p_state) { return (p_state->ValuePrev <= 0U) && (p_state->Value > 0U); }
static inline bool _UserAIn_IsFallingEdge(const UserAIn_State_T * p_state) { return (p_state->ValuePrev > 0U) && (p_state->Value <= 0U); }
static inline bool _UserAIn_IsEdge(const UserAIn_State_T * p_state) { return (_UserAIn_IsRisingEdge(p_state) || _UserAIn_IsFallingEdge(p_state)); }
// static inline bool _UserAIn_IsEdge(const UserAIn_State_T * p_state) { return is_value_edge(p_state->ValuePrev, p_state->Value); }
static inline bool _UserAIn_IsOn(const UserAIn_State_T * p_state) { return (p_state->Value > 0U); }
static inline uint16_t _UserAIn_GetValue(const UserAIn_State_T * p_state) { return p_state->Value; }

/******************************************************************************/
/*
    State Query Functions
*/
/******************************************************************************/
/* Edge as threshold */
// static inline bool UserAIn_IsOn(const UserAIn_T * p_context) { return (_UserAIn_IsEdgePinPassthrough(p_context->P_EDGE_PIN) && _UserAIn_IsOn(p_context->P_STATE)); }
static inline bool UserAIn_IsOn(const UserAIn_T * p_context) { return _UserAIn_IsEdgePinPassthrough(p_context->P_EDGE_PIN) ? _UserAIn_IsOn(p_context->P_STATE) : false; }

/*! @return Percent16 by default */
/* Check IsOn on get, rather than overwrite 0 when off, Value remains prev captured value */
static inline uint16_t UserAIn_GetValue(const UserAIn_T * p_context) { return _UserAIn_IsEdgePinPassthrough(p_context->P_EDGE_PIN) ? _UserAIn_GetValue(p_context->P_STATE) : 0U; }

/*!
    Edge detection - considers EdgePin status
    @brief Check for edge without polling (query current state only)
    @note Uses digital pin edge if present, otherwise analog threshold edge
*/
static inline bool UserAIn_IsRisingEdge(const UserAIn_T * p_context) { return (p_context->P_EDGE_PIN != NULL) ? UserDIn_IsRisingEdge(p_context->P_EDGE_PIN) : _UserAIn_IsRisingEdge(p_context->P_STATE); }
static inline bool UserAIn_IsFallingEdge(const UserAIn_T * p_context) { return (p_context->P_EDGE_PIN != NULL) ? UserDIn_IsFallingEdge(p_context->P_EDGE_PIN) : _UserAIn_IsFallingEdge(p_context->P_STATE); }
static inline bool UserAIn_IsEdge(const UserAIn_T * p_context) { return (p_context->P_EDGE_PIN != NULL) ? UserDIn_IsEdge(p_context->P_EDGE_PIN) : _UserAIn_IsEdge(p_context->P_STATE); }


/******************************************************************************/
/*
    Public Functions
*/
/******************************************************************************/
extern void UserAIn_InitFrom(const UserAIn_T * p_context, const UserAIn_Config_T * p_config);
extern void UserAIn_Init(const UserAIn_T * p_context);

/* Polling functions */
extern void UserAIn_CaptureValue(const UserAIn_T * p_context, uint16_t value_adcu);
extern bool UserAIn_Poll(const UserAIn_T * p_context, uint16_t value_adcu);
extern bool UserAIn_PollRisingEdge(const UserAIn_T * p_context, uint16_t value_adcu);
extern bool UserAIn_PollFallingEdge(const UserAIn_T * p_context, uint16_t value_adcu);
extern bool UserAIn_PollEdge(const UserAIn_T * p_context, uint16_t value_adcu);

