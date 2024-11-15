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
    @file   Linear_ADC.h
    @author FireSourcery
    @brief    Scale ADCU to provided reference value
            Fast Frac16 without division. No physical units scalar for precision.
    @version V0
*/
/******************************************************************************/
#ifndef LINEAR_ADC_H
#define LINEAR_ADC_H

#include "Linear_Frac16.h"
#include <stdint.h>

/******************************************************************************/
/*!
    From ADCU
*/
/******************************************************************************/
static inline int32_t Linear_ADC_CalcFrac16(const Linear_T * p_linear, uint16_t adcu) { return Linear_Frac16(p_linear, adcu); }
static inline int16_t Linear_ADC_CalcFracS16(const Linear_T * p_linear, uint16_t adcu) { return Linear_Frac16_Signed(p_linear, adcu); }
static inline uint16_t Linear_ADC_CalcFracU16(const Linear_T * p_linear, uint16_t adcu) { return Linear_Frac16_Unsigned(p_linear, adcu); }
static inline uint16_t Linear_ADC_CalcFracU16_Abs(const Linear_T * p_linear, uint16_t adcu) { return Linear_Frac16_Unsigned_Abs(p_linear, adcu); }
static inline int32_t Linear_ADC_CalcPhysical(const Linear_T * p_linear, uint16_t adcu) { return Linear_Frac16_Units(p_linear, adcu); }

/******************************************************************************/
/*!
    Intermediary
*/
/******************************************************************************/
static inline int32_t Linear_ADC_CalcPhysical_Frac16(const Linear_T * p_linear, uint16_t frac16) { return Linear_Frac16_Units16(p_linear, frac16); }

/* Division in this function */
static inline int32_t Linear_ADC_CalcFrac16_Physical(const Linear_T * p_linear, int32_t units) { return Linear_Frac16_InvUnits16(p_linear, units); }

/******************************************************************************/
/*!
    To ADCU
*/
/******************************************************************************/
/* Division in this function */
static inline uint16_t Linear_ADC_CalcAdcu_Physical(const Linear_T * p_linear, int16_t units) { return Linear_Frac16_InvUnits(p_linear, units); }
static inline uint16_t Linear_ADC_CalcAdcu_FracS16(const Linear_T * p_linear, int32_t signedFrac16) { return Linear_Frac16_InvSigned(p_linear, signedFrac16); }
static inline uint16_t Linear_ADC_CalcAdcu_FracU16(const Linear_T * p_linear, uint32_t unsignedFrac16) { return Linear_Frac16_InvUnsigned(p_linear, unsignedFrac16); }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Linear_ADC_Init(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuRef, int16_t physicalZero, int16_t physicalRef);
extern void Linear_ADC_Init_ZeroToPeak(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuZtPRef, int16_t physicalZero, int16_t physicalRef);
extern void Linear_ADC_Init_MinMax(Linear_T * p_linear, uint16_t adcuMin, uint16_t adcuMax, int16_t physicalMin, int16_t physicalMax);
extern void Linear_ADC_Init_Inverted(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuRef, int16_t physicalZero, int16_t physicalRef);
extern void Linear_ADC_SetInverted(Linear_T * p_linear);
extern void Linear_ADC_Init_PeakToPeakMilliV(Linear_T * p_linear, uint16_t adcVRef_MilliV, uint16_t adcMax, uint16_t min_MilliV, uint16_t max_MilliV, int16_t physicalZero, int16_t physicalRef);
extern void Linear_ADC_Init_ZeroToPeakMilliV(Linear_T * p_linear, uint16_t adcVRef_MilliV, uint16_t adcMax, uint16_t zero_MilliV, uint16_t max_MilliV, int16_t physicalZero, int16_t physicalRef);

#endif
