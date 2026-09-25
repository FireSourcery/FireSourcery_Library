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
    @file   VBemf.h
    @author FireSourcery
    @brief  Back-EMF zero-crossing detection for six-step sensorless commutation.
*/
/******************************************************************************/
#ifndef VBEMF_H
#define VBEMF_H

#include "Transducer/Pulse/PulseTimer.h"
#include "Math/Fixed/fract16.h"
#include "Math/math_general.h"

#include <stdint.h>
#include <stdbool.h>

/*
    Demagnetization window after commutation, as a fraction of the ZC period.
    Freewheeling current through the body diode holds the floating phase at a
    rail and would cross the threshold immediately.
*/
#define VBEMF_BLANK_SHIFT (2U) /* DeltaT / 4 */

/*
    Consecutive crossings before the observer is trusted to drive commutation.
*/
#define VBEMF_RELIABLE_COUNT (10U)

typedef enum VBemf_ZeroCross
{
    VBEMF_ZERO_CROSS_NONE,      /* Still approaching the threshold */
    VBEMF_ZERO_CROSS_FOUND,     /* Crossing bracketed by the last 2 samples */
    VBEMF_ZERO_CROSS_MISSED,    /* Both samples past the threshold */
}
VBemf_ZeroCross_T;

typedef struct VBemf_State
{
    /* Floating phase samples. Held as signed distance to threshold, rising-normalized. */
    int32_t Emf;
    int32_t EmfPrev;
    uint32_t TimeSample;        /* Timer ticks at Emf, from commutation */
    uint32_t TimeSamplePrev;

    uint32_t TimeCommutation;   /* Timer value at last commutation */
    uint32_t TimeBlank;

    bool IsRising;              /* Expected slope, alternates per commutation */
    bool IsFound;               /* Crossing resolved this commutation */
    uint32_t TimeZeroCross;     /* Commutation to crossing. 30 degrees */
    uint16_t Count;             /* Consecutive crossings */
}
VBemf_State_T;

typedef const struct VBemf
{
    PulseTimer_T TIMER;         /* DeltaT between crossings. 60 degrees */
    VBemf_State_T * P_STATE;
}
VBemf_T;

#define VBEMF_STATE_ALLOC() (&(VBemf_State_T){0})

#define VBEMF_INIT(TimerStruct, p_State) (VBemf_T) { .TIMER = TimerStruct, .P_STATE = (p_State), }


/******************************************************************************/
/*!
    Capture
*/
/******************************************************************************/
/*
    Rising and falling crossings differ only in sign. Normalizing the distance
    rather than the sample keeps the threshold offset out of the negation --
    negating VPhase alone would compare against -vZero.
*/
static inline int32_t _VBemf_EmfOf(bool isRising, fract16_t vPhase, fract16_t vZero)
{
    return (isRising == true) ? ((int32_t)vPhase - vZero) : ((int32_t)vZero - vPhase);
}

static inline bool VBemf_IsBlanked(const VBemf_T * p_bemf)
{
    return ((*p_bemf->TIMER.P_EXTENDED_TIMER - p_bemf->P_STATE->TimeCommutation) < p_bemf->P_STATE->TimeBlank);
}

static inline void VBemf_CaptureVPhase(const VBemf_T * p_bemf, fract16_t vPhase, fract16_t vZero)
{
    VBemf_State_T * p_state = p_bemf->P_STATE;

    p_state->EmfPrev = p_state->Emf;
    p_state->Emf = _VBemf_EmfOf(p_state->IsRising, vPhase, vZero);
    p_state->TimeSamplePrev = p_state->TimeSample;
    p_state->TimeSample = *p_bemf->TIMER.P_EXTENDED_TIMER - p_state->TimeCommutation;
}


/******************************************************************************/
/*!
    Zero Crossing

    The crossing falls between two samples. Sample spacing is the control period,
    so without interpolation the commutation instant quantizes to a full tick --
    at 20kHz that is a significant fraction of the 60 degree window at speed.
*/
/******************************************************************************/
/*
    Position of the crossing within [t0, t1], by similar triangles on the segment.
        f = -e0 / (e1 - e0),  e0 < 0 <= e1
*/
static inline ufract16_t _VBemf_CrossFractionOf(int32_t emfPrev, int32_t emf)
{
    return (ufract16_t)fract16_div(-emfPrev, emf - emfPrev);
}

static inline uint32_t _VBemf_CrossTimeOf(const VBemf_State_T * p_state)
{
    return p_state->TimeSamplePrev + fract16_mul(p_state->TimeSample - p_state->TimeSamplePrev, _VBemf_CrossFractionOf(p_state->EmfPrev, p_state->Emf));
}

/*
    A miss means the crossing landed inside the blank window or between two
    skipped samples. Fabricating a timestamp from it poisons the DeltaT average,
    so it resets the reliability count instead.
*/
static inline VBemf_ZeroCross_T VBemf_PollZeroCross(const VBemf_T * p_bemf)
{
    VBemf_State_T * p_state = p_bemf->P_STATE;

    if ((p_state->IsFound == true) || (p_state->Emf < 0)) { return VBEMF_ZERO_CROSS_NONE; }

    p_state->IsFound = true;

    if (p_state->EmfPrev >= 0) { p_state->Count = 0U; return VBEMF_ZERO_CROSS_MISSED; }

    p_state->TimeZeroCross = _VBemf_CrossTimeOf(p_state);
    p_state->Count++;
    PulseTimer_CaptureEdge(&p_bemf->TIMER);
    return VBEMF_ZERO_CROSS_FOUND;
}


/******************************************************************************/
/*!
    Commutation

    Crossing leads commutation by 30 degrees; DeltaT between crossings spans 60.
*/
/******************************************************************************/
static inline uint32_t VBemf_CommutationDelay(const VBemf_T * p_bemf)
{
    return p_bemf->TIMER.P_STATE->DeltaT / 2U;
}

static inline bool VBemf_PollCommutation(const VBemf_T * p_bemf)
{
    return ((*p_bemf->TIMER.P_EXTENDED_TIMER - p_bemf->P_STATE->TimeCommutation) > (p_bemf->P_STATE->TimeZeroCross + VBemf_CommutationDelay(p_bemf)));
}

/*
    Called on commutation. Alternates the expected slope and re-arms the window.
*/
static inline void VBemf_OnCommutation(const VBemf_T * p_bemf)
{
    VBemf_State_T * p_state = p_bemf->P_STATE;

    p_state->TimeCommutation = *p_bemf->TIMER.P_EXTENDED_TIMER;
    p_state->TimeBlank = p_bemf->TIMER.P_STATE->DeltaT >> VBEMF_BLANK_SHIFT;
    p_state->IsRising = !p_state->IsRising;
    p_state->IsFound = false;
    p_state->TimeSample = 0U;
    p_state->TimeSamplePrev = 0U;
}

static inline bool VBemf_IsReliable(const VBemf_T * p_bemf) { return (p_bemf->P_STATE->Count > VBEMF_RELIABLE_COUNT); }

static inline uint32_t VBemf_GetElecFreq(const VBemf_T * p_bemf) { return PulseTimer_DeltaT_Freq(&p_bemf->TIMER) / 6U; }

static inline bool VBemf_IsStop(const VBemf_T * p_bemf) { return PulseTimer_IsExtendedStop(&p_bemf->TIMER); }

#endif
