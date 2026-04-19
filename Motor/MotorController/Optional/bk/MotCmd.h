#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   MotCmd.h
    @author FireSourcery
    @brief  Drive command source multiplexer. Buffers the active Motor_Input_T
            along with which source produced it (Analog / Serial / CAN) and a
            per-source stale-watchdog. The top-level state machine reads the
            unified command; individual sources push via MotCmd_Set*_FromX().
*/
/******************************************************************************/
#include "Motor/Motor/Motor_User.h" /* Motor_Input_T, Motor_Direction_T, Motor_FeedbackMode_T, Phase_Output_T */
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Source identity
*/
/******************************************************************************/
typedef enum MotCmd_Source
{
    MOT_CMD_SOURCE_NONE,
    MOT_CMD_SOURCE_ANALOG,
    MOT_CMD_SOURCE_SERIAL,
    MOT_CMD_SOURCE_CAN,
    MOT_CMD_SOURCE_COUNT,
}
MotCmd_Source_T;

/******************************************************************************/
/*
    Config — NVM
*/
/******************************************************************************/
typedef struct MotCmd_Config
{
    MotCmd_Source_T PreferredSource;    /* Source that owns CmdValue when present */
    uint16_t StaleTimeoutMs;            /* 0 = never stale */
    uint8_t AllowFallbackToAnalog : 1U; /* drop to analog if preferred goes stale */
    uint8_t RequireExplicitStart  : 1U; /* reject non-zero cmd until a zero is seen first */
}
MotCmd_Config_T;

/******************************************************************************/
/*
    Runtime state
*/
/******************************************************************************/
typedef struct MotCmd_State
{
    Motor_Input_T Cmd;                              /* Latest unified command */
    MotCmd_Source_T ActiveSource;                   /* Which source last wrote Cmd */
    uint32_t LastUpdateMs[MOT_CMD_SOURCE_COUNT];    /* Per-source last-update tick */
    bool HasZeroed[MOT_CMD_SOURCE_COUNT];           /* For RequireExplicitStart */
    MotCmd_Config_T Config;
}
MotCmd_State_T;

/******************************************************************************/
/*
    Compile-time context
*/
/******************************************************************************/
typedef const struct MotCmd
{
    MotCmd_State_T * P_STATE;
    const volatile uint32_t * P_TIMER;      /* Free-running ms timer */
    const MotCmd_Config_T * P_NVM_CONFIG;
}
MotCmd_T;

#define MOT_CMD_STATE_ALLOC() (&(MotCmd_State_T){0})

#define MOT_CMD_INIT(p_State, p_TimerMs, p_Config) (MotCmd_T) \
{                                                             \
    .P_STATE = (p_State),                                     \
    .P_TIMER = (p_TimerMs),                                   \
    .P_NVM_CONFIG = (p_Config),                               \
}

/******************************************************************************/
/*
    Internal helpers
*/
/******************************************************************************/
static inline uint32_t _MotCmd_Now(const MotCmd_T * p_cmd) { return *p_cmd->P_TIMER; }

static inline bool _MotCmd_IsStale(const MotCmd_T * p_cmd, MotCmd_Source_T src)
{
    uint16_t timeout = p_cmd->P_STATE->Config.StaleTimeoutMs;
    if (timeout == 0U) { return false; }
    return ((_MotCmd_Now(p_cmd) - p_cmd->P_STATE->LastUpdateMs[src]) > timeout);
}

static inline bool _MotCmd_SourceAccepts(const MotCmd_T * p_cmd, MotCmd_Source_T src)
{
    MotCmd_Source_T preferred = p_cmd->P_STATE->Config.PreferredSource;
    if (src == preferred) { return true; }
    /* Lower-priority sources only drive when preferred is absent/stale */
    if ((src == MOT_CMD_SOURCE_ANALOG) && p_cmd->P_STATE->Config.AllowFallbackToAnalog)
    {
        return _MotCmd_IsStale(p_cmd, preferred);
    }
    return false;
}

/******************************************************************************/
/*
    Push API — one per source
*/
/******************************************************************************/
static inline void _MotCmd_TouchSource(const MotCmd_T * p_cmd, MotCmd_Source_T src)
{
    p_cmd->P_STATE->LastUpdateMs[src] = _MotCmd_Now(p_cmd);
}

static inline bool MotCmd_SetFrom(const MotCmd_T * p_cmd, MotCmd_Source_T src, const Motor_Input_T * p_input)
{
    _MotCmd_TouchSource(p_cmd, src);
    if (_MotCmd_SourceAccepts(p_cmd, src) == false) { return false; }

    if (p_cmd->P_STATE->Config.RequireExplicitStart && (p_cmd->P_STATE->HasZeroed[src] == false))
    {
        if (p_input->CmdValue != 0) { return false; }
        p_cmd->P_STATE->HasZeroed[src] = true;
    }

    p_cmd->P_STATE->Cmd = *p_input;
    p_cmd->P_STATE->ActiveSource = src;
    return true;
}

static inline bool MotCmd_SetCmdValueFrom(const MotCmd_T * p_cmd, MotCmd_Source_T src, int16_t value)
{
    _MotCmd_TouchSource(p_cmd, src);
    if (_MotCmd_SourceAccepts(p_cmd, src) == false) { return false; }

    if (p_cmd->P_STATE->Config.RequireExplicitStart && (p_cmd->P_STATE->HasZeroed[src] == false))
    {
        if (value != 0) { return false; }
        p_cmd->P_STATE->HasZeroed[src] = true;
    }

    p_cmd->P_STATE->Cmd.CmdValue = value;
    p_cmd->P_STATE->ActiveSource = src;
    return true;
}

static inline bool MotCmd_SetDirectionFrom(const MotCmd_T * p_cmd, MotCmd_Source_T src, Motor_Direction_T dir)
{
    _MotCmd_TouchSource(p_cmd, src);
    if (_MotCmd_SourceAccepts(p_cmd, src) == false) { return false; }
    p_cmd->P_STATE->Cmd.Direction = dir;
    return true;
}

static inline bool MotCmd_SetPhaseOutputFrom(const MotCmd_T * p_cmd, MotCmd_Source_T src, Phase_Output_T phase)
{
    _MotCmd_TouchSource(p_cmd, src);
    if (_MotCmd_SourceAccepts(p_cmd, src) == false) { return false; }
    p_cmd->P_STATE->Cmd.PhaseOutput = phase;
    return true;
}

static inline bool MotCmd_SetFeedbackModeFrom(const MotCmd_T * p_cmd, MotCmd_Source_T src, Motor_FeedbackMode_T fb)
{
    _MotCmd_TouchSource(p_cmd, src);
    if (_MotCmd_SourceAccepts(p_cmd, src) == false) { return false; }
    p_cmd->P_STATE->Cmd.FeedbackMode = fb;
    return true;
}

/******************************************************************************/
/*
    Read API — consumed by state machine
*/
/******************************************************************************/
static inline const Motor_Input_T * MotCmd_GetInput(const MotCmd_T * p_cmd) { return &p_cmd->P_STATE->Cmd; }
static inline int16_t MotCmd_GetCmdValue(const MotCmd_T * p_cmd)            { return p_cmd->P_STATE->Cmd.CmdValue; }
static inline Motor_Direction_T MotCmd_GetDirection(const MotCmd_T * p_cmd) { return p_cmd->P_STATE->Cmd.Direction; }
static inline MotCmd_Source_T MotCmd_GetActiveSource(const MotCmd_T * p_cmd){ return p_cmd->P_STATE->ActiveSource; }

static inline bool MotCmd_IsSourceStale(const MotCmd_T * p_cmd, MotCmd_Source_T src) { return _MotCmd_IsStale(p_cmd, src); }

static inline bool MotCmd_IsActiveStale(const MotCmd_T * p_cmd)
{
    return _MotCmd_IsStale(p_cmd, p_cmd->P_STATE->Config.PreferredSource);
}

/******************************************************************************/
/*
    Thread tick — run on the 1ms thread. Forces safe cmd when active source
    is stale and fallback is disabled.
*/
/******************************************************************************/
static inline void MotCmd_Proc(const MotCmd_T * p_cmd)
{
    if (MotCmd_IsActiveStale(p_cmd) == true)
    {
        if (p_cmd->P_STATE->Config.AllowFallbackToAnalog == false)
        {
            p_cmd->P_STATE->Cmd.CmdValue = 0;
            p_cmd->P_STATE->Cmd.PhaseOutput = PHASE_OUTPUT_FLOAT;
            p_cmd->P_STATE->ActiveSource = MOT_CMD_SOURCE_NONE;
        }
    }
}

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void MotCmd_Init(const MotCmd_T * p_cmd);
extern void MotCmd_ResetStartGate(const MotCmd_T * p_cmd, MotCmd_Source_T src);
extern void MotCmd_SetPreferredSource(const MotCmd_T * p_cmd, MotCmd_Source_T src);
extern void MotCmd_SetStaleTimeoutMs(const MotCmd_T * p_cmd, uint16_t timeoutMs);

/******************************************************************************/
/*
    VarId / ConfigId
*/
/******************************************************************************/
typedef enum MotCmd_VarId
{
    MOT_CMD_VAR_CMD_VALUE,
    MOT_CMD_VAR_DIRECTION,
    MOT_CMD_VAR_ACTIVE_SOURCE,
    MOT_CMD_VAR_IS_STALE,
}
MotCmd_VarId_T;

typedef enum MotCmd_ConfigId
{
    MOT_CMD_CONFIG_PREFERRED_SOURCE,
    MOT_CMD_CONFIG_STALE_TIMEOUT_MS,
    MOT_CMD_CONFIG_ALLOW_FALLBACK_ANALOG,
    MOT_CMD_CONFIG_REQUIRE_EXPLICIT_START,
}
MotCmd_ConfigId_T;

extern int32_t MotCmd_VarId_Get(const MotCmd_T * p_cmd, MotCmd_VarId_T id);
extern int32_t MotCmd_ConfigId_Get(const MotCmd_T * p_cmd, MotCmd_ConfigId_T id);
extern void MotCmd_ConfigId_Set(const MotCmd_T * p_cmd, MotCmd_ConfigId_T id, int32_t value);
