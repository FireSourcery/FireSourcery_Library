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
    @file   Analog_ADC_Batch.h
    @author FireSourcery
    @brief  Hardware sequenced conversion batches.

    A batch is a fixed channel set on one ADC, converted and transferred as a unit by hardware,
        e.g. Trigger -> PDB back-to-back pre-triggers -> SC1[slots] -> COCO -> DMA R[slots] -> P_CHANNEL_RESULTS[slots]
    with 1 callback on complete.

    Channel ID == SC1 slot == R[] index == P_CHANNEL_RESULTS index.
    Unselected batch results remain valid, as the last converted values.

    Board integration:
        - Configure trigger and transfer resources. Transfer destination P_CHANNEL_RESULTS.
        - Provide Analog_ADC_T.SELECT_BATCH, routing triggers to a channel mask.
        - Transfer complete ISR: clear the Hw flag, then Analog_ADC_OnBatchComplete_ISR().
*/
/******************************************************************************/
#include "Analog_ADC.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>




