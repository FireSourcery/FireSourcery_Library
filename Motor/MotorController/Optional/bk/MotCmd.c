/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   MotCmd.c
    @author FireSourcery
*/
/******************************************************************************/
#include "MotCmd.h"
#include <string.h>

void MotCmd_Init(const MotCmd_T * p_cmd)
{
    memset(p_cmd->P_STATE, 0, sizeof(MotCmd_State_T));
    if (p_cmd->P_NVM_CONFIG != NULL) { p_cmd->P_STATE->Config = *p_cmd->P_NVM_CONFIG; }
    p_cmd->P_STATE->ActiveSource = MOT_CMD_SOURCE_NONE;
    p_cmd->P_STATE->Cmd.Direction = MOTOR_DIRECTION_NULL;
    p_cmd->P_STATE->Cmd.PhaseOutput = PHASE_OUTPUT_FLOAT;
}

void MotCmd_ResetStartGate(const MotCmd_T * p_cmd, MotCmd_Source_T src)
{
    if ((unsigned)src < MOT_CMD_SOURCE_COUNT) { p_cmd->P_STATE->HasZeroed[src] = false; }
}

void MotCmd_SetPreferredSource(const MotCmd_T * p_cmd, MotCmd_Source_T src)
{
    p_cmd->P_STATE->Config.PreferredSource = src;
    for (unsigned i = 0; i < MOT_CMD_SOURCE_COUNT; i++) { p_cmd->P_STATE->HasZeroed[i] = false; }
}

void MotCmd_SetStaleTimeoutMs(const MotCmd_T * p_cmd, uint16_t timeoutMs)
{
    p_cmd->P_STATE->Config.StaleTimeoutMs = timeoutMs;
}

int32_t MotCmd_VarId_Get(const MotCmd_T * p_cmd, MotCmd_VarId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_CMD_VAR_CMD_VALUE:     value = p_cmd->P_STATE->Cmd.CmdValue;       break;
        case MOT_CMD_VAR_DIRECTION:     value = p_cmd->P_STATE->Cmd.Direction;      break;
        case MOT_CMD_VAR_ACTIVE_SOURCE: value = p_cmd->P_STATE->ActiveSource;       break;
        case MOT_CMD_VAR_IS_STALE:      value = MotCmd_IsActiveStale(p_cmd);        break;
        default: break;
    }
    return value;
}

int32_t MotCmd_ConfigId_Get(const MotCmd_T * p_cmd, MotCmd_ConfigId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_CMD_CONFIG_PREFERRED_SOURCE:       value = p_cmd->P_STATE->Config.PreferredSource;         break;
        case MOT_CMD_CONFIG_STALE_TIMEOUT_MS:       value = p_cmd->P_STATE->Config.StaleTimeoutMs;          break;
        case MOT_CMD_CONFIG_ALLOW_FALLBACK_ANALOG:  value = p_cmd->P_STATE->Config.AllowFallbackToAnalog;   break;
        case MOT_CMD_CONFIG_REQUIRE_EXPLICIT_START: value = p_cmd->P_STATE->Config.RequireExplicitStart;    break;
        default: break;
    }
    return value;
}

void MotCmd_ConfigId_Set(const MotCmd_T * p_cmd, MotCmd_ConfigId_T id, int32_t value)
{
    switch (id)
    {
        case MOT_CMD_CONFIG_PREFERRED_SOURCE:       MotCmd_SetPreferredSource(p_cmd, (MotCmd_Source_T)value); break;
        case MOT_CMD_CONFIG_STALE_TIMEOUT_MS:       p_cmd->P_STATE->Config.StaleTimeoutMs = (uint16_t)value;           break;
        case MOT_CMD_CONFIG_ALLOW_FALLBACK_ANALOG:  p_cmd->P_STATE->Config.AllowFallbackToAnalog = (value != 0);       break;
        case MOT_CMD_CONFIG_REQUIRE_EXPLICIT_START: p_cmd->P_STATE->Config.RequireExplicitStart = (value != 0);        break;
        default: break;
    }
}
