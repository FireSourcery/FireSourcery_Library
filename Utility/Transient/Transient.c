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
    @file   Transient.c
    @author FireSourcery
    @brief  One-shot transient data recorder.
*/
/******************************************************************************/
#include "Transient.h"

#include <string.h>

static int32_t ReadSized(const volatile void * p_var, uint8_t size)
{
    switch (size)
    {
        case 1U: return (int32_t)(*(const volatile int8_t *)p_var);
        case 2U: return (int32_t)(*(const volatile int16_t *)p_var);
        case 4U: return (int32_t)(*(const volatile int32_t *)p_var);
        default: return 0;
    }
}

static void WriteSized(void * p_dest, uint8_t size, int32_t value)
{
    switch (size)
    {
        case 1U: *(int8_t *)p_dest  = (int8_t)value;  break;
        case 2U: *(int16_t *)p_dest = (int16_t)value; break;
        case 4U: *(int32_t *)p_dest = value;          break;
        default: break;
    }
}

static bool TriggerFires(Transient_Trigger_T type, int32_t prev, int32_t current, int32_t threshold, bool hasPrev)
{
    switch (type)
    {
        case TRANSIENT_TRIGGER_LEVEL_HIGH:   return (current >= threshold);
        case TRANSIENT_TRIGGER_LEVEL_LOW:    return (current <= threshold);
        case TRANSIENT_TRIGGER_RISING:       return (hasPrev && prev < threshold && current >= threshold);
        case TRANSIENT_TRIGGER_FALLING:      return (hasPrev && prev > threshold && current <= threshold);
        case TRANSIENT_TRIGGER_MANUAL:       return false;
        default:                             return false;
    }
}

void Transient_Init(const Transient_T * p_trans)
{
    Transient_State_T * p_state = p_trans->P_STATE;
    p_state->Status = TRANSIENT_STATUS_IDLE;
    p_state->SampleIndex = 0U;
    p_state->MissedCount = 0U;
    p_state->TriggerTick = 0U;
    p_state->TriggerType = TRANSIENT_TRIGGER_MANUAL;
    p_state->TriggerThreshold = 0;
    p_state->PrevTriggerValue = 0;
    p_state->HasPrevTrigger = false;
    if (p_trans->P_BUFFER != NULL) { memset(p_trans->P_BUFFER, 0, (size_t)p_trans->SAMPLE_SIZE * p_trans->BUFFER_LENGTH); }
}

void Transient_Configure(const Transient_T * p_trans, Transient_Trigger_T type, int32_t threshold)
{
    Transient_State_T * p_state = p_trans->P_STATE;
    p_state->TriggerType = type;
    p_state->TriggerThreshold = threshold;
    p_state->HasPrevTrigger = false;
}

void Transient_Reset(const Transient_T * p_trans)
{
    Transient_State_T * p_state = p_trans->P_STATE;
    p_state->Status = TRANSIENT_STATUS_IDLE;
    p_state->SampleIndex = 0U;
    p_state->MissedCount = 0U;
    p_state->TriggerTick = 0U;
    p_state->HasPrevTrigger = false;
}

void Transient_Arm(const Transient_T * p_trans)
{
    Transient_Reset(p_trans);
    p_trans->P_STATE->Status = TRANSIENT_STATUS_ARMED;
}

void Transient_Disarm(const Transient_T * p_trans)
{
    if (p_trans->P_STATE->Status == TRANSIENT_STATUS_ARMED) { p_trans->P_STATE->Status = TRANSIENT_STATUS_IDLE; }
}

void Transient_Start(const Transient_T * p_trans)
{
    Transient_State_T * p_state = p_trans->P_STATE;
    p_state->SampleIndex = 0U;
    p_state->MissedCount = 0U;
    p_state->Status = TRANSIENT_STATUS_RECORDING;
}

void Transient_Stop(const Transient_T * p_trans)
{
    Transient_State_T * p_state = p_trans->P_STATE;
    if (p_state->Status == TRANSIENT_STATUS_RECORDING) { p_state->Status = TRANSIENT_STATUS_COMPLETE; }
}

/*
    Evaluate trigger condition. Call on a control thread (1ms or faster).
    On fire: transitions ARMED -> RECORDING; the next Transient_Sample() is sample 0.
*/
void Transient_PollTrigger(const Transient_T * p_trans)
{
    Transient_State_T * p_state = p_trans->P_STATE;
    if (p_state->Status != TRANSIENT_STATUS_ARMED) { return; }

    int32_t current = ReadSized(p_trans->P_TRIGGER_VAR, p_trans->TRIGGER_SIZE);
    bool fires = TriggerFires(p_state->TriggerType, p_state->PrevTriggerValue, current, p_state->TriggerThreshold, p_state->HasPrevTrigger);
    p_state->PrevTriggerValue = current;
    p_state->HasPrevTrigger = true;

    if (fires)
    {
        p_state->SampleIndex = 0U;
        p_state->MissedCount = 0U;
        p_state->TriggerTick = 0U;
        p_state->Status = TRANSIENT_STATUS_RECORDING;
    }
}

/*
    Capture one sample. Call at the desired recording rate.
    No-op unless RECORDING. Transitions to COMPLETE when buffer fills.
*/
void Transient_Sample(const Transient_T * p_trans)
{
    Transient_State_T * p_state = p_trans->P_STATE;
    if (p_state->Status != TRANSIENT_STATUS_RECORDING) { return; }

    uint32_t index = p_state->SampleIndex;
    if (index >= p_trans->BUFFER_LENGTH)
    {
        p_state->Status = TRANSIENT_STATUS_COMPLETE;
        return;
    }

    int32_t value = ReadSized(p_trans->P_SAMPLE_VAR, p_trans->SAMPLE_SIZE);
    uint8_t * p_slot = (uint8_t *)p_trans->P_BUFFER + (index * p_trans->SAMPLE_SIZE);
    WriteSized(p_slot, p_trans->SAMPLE_SIZE, value);

    p_state->SampleIndex = index + 1U;
    p_state->TriggerTick = index + 1U;

    if (p_state->SampleIndex >= p_trans->BUFFER_LENGTH) { p_state->Status = TRANSIENT_STATUS_COMPLETE; }
}

/*
    Combined polling entry point when trigger and sampling share a thread.
*/
void Transient_Proc(const Transient_T * p_trans)
{
    switch (p_trans->P_STATE->Status)
    {
        case TRANSIENT_STATUS_ARMED:     Transient_PollTrigger(p_trans); break;
        case TRANSIENT_STATUS_RECORDING: Transient_Sample(p_trans);      break;
        default: break;
    }
}

int32_t Transient_ReadAt(const Transient_T * p_trans, uint32_t index)
{
    if (index >= p_trans->P_STATE->SampleIndex) { return 0; }
    const uint8_t * p_slot = (const uint8_t *)p_trans->P_BUFFER + (index * p_trans->SAMPLE_SIZE);
    return ReadSized(p_slot, p_trans->SAMPLE_SIZE);
}
