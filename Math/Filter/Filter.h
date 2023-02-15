/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

typedef struct Filter_Tag
{
    // uint32_t * p_Buffer;
    // uint8_t BufferSize;
    // bool IsBufferFull;
    uint16_t Index;
    int32_t Coefficient;
    int32_t Accumulator;
}
Filter_T;

static inline void Filter_InitAvg(Filter_T * p_filter)
{
    p_filter->Accumulator = 0;
    p_filter->Index = 0U;
}

static inline int32_t Filter_Avg(Filter_T * p_filter, int32_t in)
{
    p_filter->Accumulator += in;
    p_filter->Index++;
    return p_filter->Accumulator / p_filter->Index;
}

#endif
