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
    @file   Hysteresis.h
    @author FireSourcery
    @brief  Reusable hysteresis behavior for threshold monitoring
*/
/******************************************************************************/
#include "math_hysteresis.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Basic Hysteresis
*/
/******************************************************************************/
/* Region without previous on/off state */
typedef enum Hysteresis_Region
{
    HYSTERESIS_REGION_OFF = 0,          /* Value in inactive region */
    HYSTERESIS_REGION_ON = 1,           /* Value in active region */
    HYSTERESIS_REGION_DEADBAND = -1,    /* Value in deadband - maintains previous state */
                                        /* consistent with boolean state this way */
}
Hysteresis_Region_T;

/*
    Complete hysteresis instance
    Caller handle direction
*/
typedef struct Hysteresis
{
    int32_t Setpoint;       /* Activation threshold (trip point) */
    int32_t Resetpoint;     /* Deactivation threshold (reset point) */

    int32_t Output;     /* Conditioned value as state */
    bool OutputState;   /* Current logical state. true as past the setpoint, low or high */    // alternatively derive with getter

    // alternatively caller handle edge detection
    // int32_t OutputPrev;
    // bool OutputStatePrev;
}
Hysteresis_T;


/******************************************************************************/
/*
    Core Functions - Simplified with Unified Region
*/
/******************************************************************************/
/******************************************************************************/
/*
    High-acting: setpoint > resetpoint
    activate when value >= setpoint, deactivate when value <= resetpoint
*/
/******************************************************************************/
/* Apply hysteresis conditioning to input value */
/* hysteresis effective in reset direction only */
static inline int32_t Hysteresis_OutputOf(const Hysteresis_T * p_hyst, int32_t input)
{
    return hysteresis_on_falling(p_hyst->Setpoint, p_hyst->Resetpoint, p_hyst->Output, input);
}

/* Alternative implementation */
static inline bool Hysteresis_OutputStateOf(const Hysteresis_T * p_hyst, int32_t input)
{
    return hysteresis_output_state(p_hyst->Setpoint, p_hyst->Resetpoint, p_hyst->OutputState, input);
}

/* Determine region of any value - works for both input analysis and output state */
/* output returns [HYSTERESIS_REGION_DEADBAND] when inactive only */
static inline Hysteresis_Region_T Hysteresis_RegionOf(const Hysteresis_T * p_hyst, int32_t value)
{
    if (value <= p_hyst->Resetpoint)    { return HYSTERESIS_REGION_OFF; }
    else if (value >= p_hyst->Setpoint) { return HYSTERESIS_REGION_ON; }
    else                                { return HYSTERESIS_REGION_DEADBAND; }
}

/*

*/
static inline bool Hysteresis_PollOutput(Hysteresis_T * p_hyst, int32_t input)
{
    p_hyst->Output = Hysteresis_OutputOf(p_hyst, input);
    return (Hysteresis_RegionOf(p_hyst, p_hyst->Output) == HYSTERESIS_REGION_ON);
}

/* Storing boolean state */
static inline bool Hysteresis_Poll(Hysteresis_T * p_hyst, int32_t input)
{
    p_hyst->Output = Hysteresis_OutputOf(p_hyst, input);
    // p_hyst->OutputStatePrev = p_hyst->OutputState; /* Save previous state for edge detection */
    p_hyst->OutputState = (Hysteresis_RegionOf(p_hyst, p_hyst->Output) == HYSTERESIS_REGION_ON);
    return p_hyst->OutputState;
}

/* Getters for output state */
/* output in deadband on rising only, effectively off */
static inline Hysteresis_Region_T Hysteresis_GetOutputRegion(const Hysteresis_T * p_hyst) { return Hysteresis_RegionOf(p_hyst, p_hyst->Output); }
static inline bool Hysteresis_GetOutputState(const Hysteresis_T * p_hyst) { return (p_hyst->Output >= p_hyst->Setpoint); }


/******************************************************************************/
/*
    Caller select implementation
    alternatively map virtual *-1 comparison
*/
/******************************************************************************/
/******************************************************************************/
/*
    Low-acting: setpoint < resetpoint
    activate when value <= setpoint, deactivate when value >= resetpoint
*/
/******************************************************************************/
static inline bool Hysteresis_Inverted_OutputStateOf(const Hysteresis_T * p_hyst, int32_t input)
{
    return hysteresis_output_state_inverted(p_hyst->Setpoint, p_hyst->Resetpoint, p_hyst->OutputState, input);
}

static inline int32_t Hysteresis_Inverted_OutputOf(const Hysteresis_T * p_hyst, int32_t input)
{
    return hysteresis_on_rising(p_hyst->Setpoint, p_hyst->Resetpoint, p_hyst->Output, input);
}

static inline Hysteresis_Region_T Hysteresis_Inverted_RegionOf(const Hysteresis_T * p_hyst, int32_t value)
{
    if (value >= p_hyst->Resetpoint)    { return HYSTERESIS_REGION_OFF; }
    else if (value <= p_hyst->Setpoint) { return HYSTERESIS_REGION_ON; }
    else                                { return HYSTERESIS_REGION_DEADBAND; }
}

static inline bool Hysteresis_Inverted_PollOutput(Hysteresis_T * p_hyst, int32_t input)
{
    p_hyst->Output = Hysteresis_Inverted_OutputOf(p_hyst, input);
    return (Hysteresis_Inverted_RegionOf(p_hyst, p_hyst->Output) == HYSTERESIS_REGION_ON);
}

static inline bool Hysteresis_Inverted_Poll(Hysteresis_T * p_hyst, int32_t input)
{
    p_hyst->Output = Hysteresis_Inverted_OutputOf(p_hyst, input);
    // p_hyst->OutputStatePrev = p_hyst->OutputState; /* Save previous state for edge detection */
    p_hyst->OutputState = (Hysteresis_Inverted_RegionOf(p_hyst, p_hyst->Output) == HYSTERESIS_REGION_ON);
    return p_hyst->OutputState;
}


/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    Rising/Falling symetric
    High-acting: Setpoint > Resetpoint
*/
static inline int32_t Hysteresis_OutputFilter(const Hysteresis_T * p_hyst, int32_t input)
{
    return hysteresis_deadband_filter(p_hyst->Setpoint, p_hyst->Resetpoint, p_hyst->Output, input);
}

static inline int32_t Hysteresis_PollOutputFilter(Hysteresis_T * p_hyst, int32_t input)
{
    // p_hyst->OutputPrev = p_hyst->Output; /* Save previous state for edge detection */
    p_hyst->Output = Hysteresis_OutputFilter(p_hyst, input);
    return p_hyst->Output;
}

/* Bidirectional delta filter */
// static inline int32_t Hysteresis_OutputFilter_Delta(const Hysteresis_T * p_hyst, int32_t input)
// {
//     return hysteresis_delta_filter(abs(p_hyst->Setpoint - p_hyst->Resetpoint), p_hyst->Output, input);
// }

// static inline int32_t Hysteresis_PollOutputFilter_Delta(Hysteresis_T * p_hyst, int32_t input)
// {
//     // p_hyst->OutputPrev = p_hyst->Output; /* Save previous state for edge detection */
//     p_hyst->Output = Hysteresis_OutputFilter_Delta(p_hyst, input);
//     return p_hyst->Output;
// }

/* Boolean interpretations */
/* via OutputValue without OutputState */
// static inline bool Hysteresis_OutputState(const Hysteresis_T * p_hyst) { return (Hysteresis_RegionOf(p_hyst, p_hyst->Output) == HYSTERESIS_REGION_ON); }
// static inline bool Hysteresis_IsOutputOn(const Hysteresis_T * p_hyst) { return (Hysteresis_RegionOf(p_hyst, p_hyst->Output) == HYSTERESIS_REGION_ON); }
// static inline bool Hysteresis_IsOutputOff(const Hysteresis_T * p_hyst) { return (Hysteresis_RegionOf(p_hyst, p_hyst->Output) != HYSTERESIS_REGION_DEADBAND); }
// static inline bool Hysteresis_IsOutputInDeadband(const Hysteresis_T * p_hyst) { return Hysteresis_RegionOf(p_hyst, p_hyst->Output) == HYSTERESIS_REGION_DEADBAND; }
/* passthrough direction to user, correct on query */
// static inline sign_t Hysteresis_OutputState(const Hysteresis_T * p_hyst) { return math_sign(p_hyst->Output); }
// static inline bool Hysteresis_OutputStateAs(const Hysteresis_T * p_hyst, int directionOn) { return directionOn * p_hyst->Output > 0; }
// static inline bool Hysteresis_IsOutputSetpointRegion(const Hysteresis_T * p_hyst )
// static inline bool Hysteresis_IsOutputResetRegion(const Hysteresis_T * p_hyst)


/******************************************************************************/
/*
    Query Functions
*/
/******************************************************************************/
static inline bool Hysteresis_OutputState(const Hysteresis_T * p_hyst) { return p_hyst->OutputState; }
static inline bool Hysteresis_IsOutputOn(const Hysteresis_T * p_hyst) { return p_hyst->OutputState; }
static inline bool Hysteresis_IsOutputOff(const Hysteresis_T * p_hyst) { return !p_hyst->OutputState; }

static inline bool Hysteresis_IsActiveHigh(const Hysteresis_T * p_hyst) { return (p_hyst->Setpoint > p_hyst->Resetpoint); }
static inline bool Hysteresis_IsActiveLow(const Hysteresis_T * p_hyst) { return (p_hyst->Setpoint < p_hyst->Resetpoint); }

static inline int32_t Hysteresis_GetOutputValue(const Hysteresis_T * p_hyst) { return p_hyst->Output; }
// static inline Hysteresis_Region_T Hysteresis_GetOutputRegion(const Hysteresis_T * p_hyst) { return Hysteresis_RegionOf(p_hyst, p_hyst->Output); }

/* On captured On/Off State */
// static inline bool Hysteresis_IsEdge(const Hysteresis_T * p_hyst) { return is_edge(p_hyst->OutputStatePrev, p_hyst->OutputState); }
// static inline bool Hysteresis_IsSetpointEdge(const Hysteresis_T * p_hyst) { return is_rising_edge(p_hyst->OutputStatePrev, p_hyst->OutputState); }
// static inline bool Hysteresis_IsResetpointEdge(const Hysteresis_T * p_hyst) { return is_falling_edge(p_hyst->OutputStatePrev, p_hyst->OutputState); }

/*
    Config
*/
static inline int32_t Hysteresis_GetDeadbandWidth(const Hysteresis_T * p_hyst) { return abs(p_hyst->Setpoint - p_hyst->Resetpoint); }

/******************************************************************************/
/*
*/
/******************************************************************************/
static inline void Hysteresis_Reset(Hysteresis_T * p_hyst)
{
    p_hyst->Output = p_hyst->Resetpoint;  /* Reset to inactive state */
    // p_hyst->OutputStatePrev = false;
    p_hyst->OutputState = false; /* Reset state */
}


/******************************************************************************/
/*
    Init
*/
/******************************************************************************/
/* Init form config types. Setpoint/Resetpoint as default. */
// void Hysteresis_InitFromThresholdLevel(Hysteresis_T * p_hyst, Threshold_Level_T * p_level)
// {
// }

// void Hysteresis_InitFromRangeAsHigh(Hysteresis_T * p_hyst, Threshold_Range_T * p_range)
// {
// }

/*!
    @brief  Initialize hysteresis with explicit setpoint and resetpoint
            (setpoint == resetpoint) => Hysteresis region is skipped.
*/
static void Hysteresis_InitThresholds(Hysteresis_T * p_hyst, int32_t setpoint, int32_t resetpoint)
{
    p_hyst->Setpoint = setpoint;
    p_hyst->Resetpoint = resetpoint;

    // p_hyst->IsActiveLow = (setpoint < resetpoint);
    // p_hyst->Direction = (setpoint - resetpoint); /* 0 for disable */

    p_hyst->Output = resetpoint;  /* Start at inactive state (resetpoint is always inactive) */
    p_hyst->OutputState = false;
    // p_hyst->OutputStatePrev = false;`
}


/*

*/
// static void Hysteresis_InitBand(Hysteresis_T * p_hyst, int32_t setpoint, int32_t deadband_width)
static void Hysteresis_InitAsActiveHigh(Hysteresis_T * p_hyst, int32_t setpoint, uint32_t deadband_width)
{
    Hysteresis_InitThresholds(p_hyst, setpoint, setpoint - deadband_width);
}


/*!
    @brief  Initialize symmetric hysteresis around a center point
            Active high
*/
// void Hysteresis_InitSymmetric(Hysteresis_T * p_hyst, int32_t center_point, uint32_t deadband_width)
// {
//     int32_t half_band = deadband_width / 2;
//     Hysteresis_InitThresholds(p_hyst, center_point + half_band, center_point - half_band);
// }

/* Low alarm: setpoint is below reset */
// static void Hysteresis_Inverted_InitBand(Hysteresis_T * p_hyst, int32_t setpoint, int32_t deadband_width)
static void Hysteresis_InitAsActiveLow(Hysteresis_T * p_hyst, int32_t setpoint, uint32_t deadband_width)
{
    Hysteresis_InitThresholds(p_hyst, setpoint, setpoint + deadband_width);
}

// static void Hysteresis_Inverted_InitSymmetric(Hysteresis_T * p_hyst, int32_t center_point, uint32_t deadband_width)
// {
//     int32_t half_band = deadband_width / 2;
//     Hysteresis_InitThresholds(p_hyst, center_point - half_band, center_point + half_band);
// }

// /******************************************************************************/
// /*
//     extern
// */
// /******************************************************************************/
// extern void Hysteresis_InitThresholds(Hysteresis_T * p_hyst, int32_t setpoint, int32_t resetpoint);
// extern void Hysteresis_InitAsActiveHigh(Hysteresis_T * p_hyst, int32_t setpoint, int32_t deadband_width);
// extern void Hysteresis_InitAsActiveLow(Hysteresis_T * p_hyst, int32_t setpoint, int32_t deadband_width);
// // extern void Hysteresis_Init_BandActiveLow(Hysteresis_T * p_hyst, int32_t setpoint, int32_t deadband_width);
// // extern void Hysteresis_InitSymmetric(Hysteresis_T * p_hyst, int32_t center_point, int32_t deadband_width);

/******************************************************************************/
/*
    Internal Direction
*/
/******************************************************************************/
// static inline Hysteresis_Region_T Hysteresis_RegionOf(const Hysteresis_T * p_hyst, int32_t value)
// {
//     return p_hyst->IsActiveLow ? Hysteresis_Inverted_RegionOf(p_hyst, value) : Hysteresis_RegionOf(p_hyst, value);
// }

// static inline int32_t Hysteresis_OutputOf(const Hysteresis_T * p_hyst, int32_t input)
// {
//     return p_hyst->IsActiveLow ? Hysteresis_Inverted_OutputOf(p_hyst, input) : Hysteresis_OutputOf(p_hyst, input);
// }

// /*
//     altneratively derive from boolean state
// */
// static inline bool Hysteresis_OutputStateOf(const Hysteresis_T * p_hyst, int32_t input)
// {
//     return p_hyst->IsActiveLow ? Hysteresis_Inverted_OutputStateOf(p_hyst, input) : Hysteresis_OutputStateOf(p_hyst, input);
// }

/* Thread */
/* Update and return current region */
// static inline bool Hysteresis_Poll(Hysteresis_T * p_hyst, int32_t value)
// {
//     p_hyst->Output = Hysteresis_OutputOf(p_hyst, value);
//     p_hyst->OutputStatePrev = p_hyst->OutputState; /* Save previous state for edge detection */
//     p_hyst->OutputState = (Hysteresis_RegionOf(p_hyst, p_hyst->Output) == HYSTERESIS_REGION_ON);
//     return p_hyst->OutputState;
// }


/******************************************************************************/
/*
    Update Functions
*/
/******************************************************************************/
// static inline int32_t Hysteresis_UpdateValue(Hysteresis_T * p_hyst, int32_t input)
// {
//     const Hysteresis_Config_T * cfg = &p_hyst->config;
//     int32_t new_output;

//     /* Apply appropriate filtering based on mode */
//     switch (cfg->filter_mode)
//     {
//         case HYSTERESIS_FILTER_PASSTHROUGH:
//             if (cfg->is_inverted)
//             {
//                 new_output = hysteresis_deadband_filter(cfg->threshold_clear, cfg->threshold_trigger, p_hyst->state.output_value, input);
//             }
//             else
//             {
//                 new_output = hysteresis_deadband_filter(cfg->threshold_trigger, cfg->threshold_clear, p_hyst->state.output_value, input);
//             }
//             break;

//         case HYSTERESIS_FILTER_STICKY_TRIGGER:
//             if (cfg->is_inverted)
//             {
//                 new_output = hysteresis_on_falling(cfg->threshold_clear, cfg->threshold_trigger, p_hyst->state.output_value, input);
//             }
//             else
//             {
//                 new_output = hysteresis_on_falling(cfg->threshold_trigger, cfg->threshold_clear, p_hyst->state.output_value, input);
//             }
//             break;

//         case HYSTERESIS_FILTER_CLAMP:
//             if (cfg->is_inverted)
//             {
//                 new_output = hysteresis_clamp_filter(cfg->threshold_clear, cfg->threshold_trigger, p_hyst->state.output_value, input);
//             }
//             else
//             {
//                 new_output = hysteresis_clamp_filter(cfg->threshold_trigger, cfg->threshold_clear, p_hyst->state.output_value, input);
//             }
//             break;

//         case HYSTERESIS_FILTER_DEADBAND:
//         default:
//             /* Pure deadband - only output changes when outside band */
//             if (Hysteresis_IsInputInDeadband(p_hyst, input))
//             {
//                 new_output = p_hyst->state.output_value;  /* Maintain */
//             }
//             else
//             {
//                 new_output = input;  /* Pass through */
//             }
//             break;
//     }

//     p_hyst->state.output_value = new_output;
//     return new_output;
// }


// typedef enum Hysteresis_Mode
// {
//     HYSTERESIS_MODE_LOW_ACTIVE = -1,    /* Low-acting hysteresis: activate on low, deactivate on high */
//     HYSTERESIS_MODE_DISABLED = 0,       /* Disabled hysteresis: no activation/deactivation */
//     HYSTERESIS_MODE_HIGH_ACTIVE = 1,    /* High-acting hysteresis: activate on high, deactivate on low */
// }
// Hysteresis_Mode_T;