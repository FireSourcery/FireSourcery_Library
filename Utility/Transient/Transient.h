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
    @file   Transient.h
    @author FireSourcery
    @brief  One-shot transient data recorder. Captures N samples of a variable
            into a user-sized buffer starting on a trigger condition.
            Caller drives sampling cadence via Transient_Sample(); trigger is
            evaluated by Transient_PollTrigger() on the same or a slower thread.
*/
/******************************************************************************/
#ifndef TRANSIENT_UTILITY_H
#define TRANSIENT_UTILITY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
    Trigger comparison against a separate variable + threshold.
    Edge modes require two PollTrigger() calls straddling the crossing.
*/
typedef enum Transient_Trigger
{
    TRANSIENT_TRIGGER_MANUAL,       /* No auto-trigger; caller invokes Transient_Start() */
    TRANSIENT_TRIGGER_RISING,       /* prev < threshold && current >= threshold */
    TRANSIENT_TRIGGER_FALLING,      /* prev > threshold && current <= threshold */
    TRANSIENT_TRIGGER_LEVEL_HIGH,   /* current >= threshold */
    TRANSIENT_TRIGGER_LEVEL_LOW,    /* current <= threshold */
}
Transient_Trigger_T;

typedef enum Transient_Status
{
    TRANSIENT_STATUS_IDLE,          /* Not armed */
    TRANSIENT_STATUS_ARMED,         /* PollTrigger active */
    TRANSIENT_STATUS_RECORDING,     /* Sampling into buffer */
    TRANSIENT_STATUS_COMPLETE,      /* Buffer full; ready for readback */
}
Transient_Status_T;

typedef struct Transient_State
{
    volatile Transient_Status_T Status;
    volatile uint32_t SampleIndex;      /* Next write index; also final count when COMPLETE */
    volatile uint32_t MissedCount;      /* Samples past buffer end */
    volatile uint32_t TriggerTick;      /* Sample count since trigger fired */

    /* Runtime-tunable for end-user experimentation */
    Transient_Trigger_T TriggerType;
    int32_t TriggerThreshold;

    int32_t PrevTriggerValue;           /* For edge detection */
    bool HasPrevTrigger;
}
Transient_State_T;

typedef const struct Transient
{
    const volatile void * P_SAMPLE_VAR;     /* Variable being recorded */
    uint8_t SAMPLE_SIZE;                    /* 1, 2, or 4 bytes */

    const volatile void * P_TRIGGER_VAR;    /* Variable compared against threshold (may equal P_SAMPLE_VAR) */
    uint8_t TRIGGER_SIZE;                   /* 1, 2, or 4 bytes */

    void * P_BUFFER;                        /* SAMPLE_SIZE * BUFFER_LENGTH bytes */
    uint32_t BUFFER_LENGTH;                 /* Max samples */

    Transient_State_T * P_STATE;
}
Transient_T;

#define TRANSIENT_STATE_ALLOC() (&(Transient_State_T){0})
#define TRANSIENT_BUFFER_ALLOC(SampleSize, Length) ((uint8_t[(SampleSize) * (Length)]){0})

#define TRANSIENT_INIT(p_SampleVar, SampleSize, p_TriggerVar, TriggerSize, p_Buffer, Length, p_State) \
    (Transient_T)                               \
    {                                           \
        .P_SAMPLE_VAR       = (p_SampleVar),    \
        .SAMPLE_SIZE        = (SampleSize),     \
        .P_TRIGGER_VAR      = (p_TriggerVar),   \
        .TRIGGER_SIZE       = (TriggerSize),    \
        .P_BUFFER           = (p_Buffer),       \
        .BUFFER_LENGTH      = (Length),         \
        .P_STATE            = (p_State),        \
    }

#define TRANSIENT_ALLOC(p_SampleVar, SampleSize, p_TriggerVar, TriggerSize, Length) \
    TRANSIENT_INIT(p_SampleVar, SampleSize, p_TriggerVar, TriggerSize, \
        TRANSIENT_BUFFER_ALLOC(SampleSize, Length), Length, TRANSIENT_STATE_ALLOC())

/******************************************************************************/
/*
    Inline status
*/
/******************************************************************************/
static inline Transient_Status_T Transient_GetStatus(const Transient_T * p_trans)   { return p_trans->P_STATE->Status; }
static inline bool Transient_IsArmed(const Transient_T * p_trans)                   { return p_trans->P_STATE->Status == TRANSIENT_STATUS_ARMED; }
static inline bool Transient_IsRecording(const Transient_T * p_trans)               { return p_trans->P_STATE->Status == TRANSIENT_STATUS_RECORDING; }
static inline bool Transient_IsComplete(const Transient_T * p_trans)                { return p_trans->P_STATE->Status == TRANSIENT_STATUS_COMPLETE; }
static inline uint32_t Transient_GetCount(const Transient_T * p_trans)              { return p_trans->P_STATE->SampleIndex; }
static inline uint32_t Transient_GetMissedCount(const Transient_T * p_trans)        { return p_trans->P_STATE->MissedCount; }
static inline uint32_t Transient_GetTriggerTick(const Transient_T * p_trans)        { return p_trans->P_STATE->TriggerTick; }
static inline uint32_t Transient_GetCapacity(const Transient_T * p_trans)           { return p_trans->BUFFER_LENGTH; }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void Transient_Init(const Transient_T * p_trans);
extern void Transient_Configure(const Transient_T * p_trans, Transient_Trigger_T type, int32_t threshold);
extern void Transient_Arm(const Transient_T * p_trans);
extern void Transient_Disarm(const Transient_T * p_trans);
extern void Transient_Start(const Transient_T * p_trans);
extern void Transient_Stop(const Transient_T * p_trans);
extern void Transient_Reset(const Transient_T * p_trans);

extern void Transient_PollTrigger(const Transient_T * p_trans);
extern void Transient_Sample(const Transient_T * p_trans);
extern void Transient_Proc(const Transient_T * p_trans);

extern int32_t Transient_ReadAt(const Transient_T * p_trans, uint32_t index);

#endif
