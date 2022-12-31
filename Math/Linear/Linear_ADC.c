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
	@file 	Linear_ADC.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Linear_ADC.h"

/******************************************************************************/
/*!
	f16(adcu) = adc_frac16
	f16(adcuZero) = 0
	f16(adcuRef) = 65535
	f(adcu) = physical
	f(adcuZero) = 0
	f(adcuRef) = physicalRef

	adcu to frac16 conversion returns without division
	adcu to physical returns without division
	division in physical to adcu, frac16 to physical units
*/
/******************************************************************************/
void Linear_ADC_Init(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuRef, int16_t physicalZero, int16_t physicalRef)
{
	Linear_Frac16_Init(p_linear, adcuZero, adcuRef, physicalZero, physicalRef);
}

void Linear_ADC_Init_ZeroToPeak(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuZtPRef, int16_t physicalZero, int16_t physicalRef)
{
	Linear_Frac16_Init(p_linear, adcuZero, adcuZero + adcuZtPRef, physicalZero, physicalRef);
}

void Linear_ADC_Init_MinMax(Linear_T * p_linear, uint16_t adcuMin, uint16_t adcuMax, int16_t physicalMin, int16_t physicalMax)
{
	Linear_Frac16_Init(p_linear, (adcuMin + adcuMax) / 2, adcuMax, (physicalMin + physicalMax) / 2, physicalMax);
}


/******************************************************************************/
/*!
	Inverted pivot on 0
	f(adcuZero) = 0
	f(adcuRef) = -physicalRef
	f(-adcuRef) = physicalRef
	f16(adcuRef) = -65536
	f16(-adcuRef) = 65536
*/
/******************************************************************************/
void Linear_ADC_InitInverted(Linear_T * p_linear)
{
	p_linear->Slope = 0 - p_linear->Slope;
	p_linear->InvSlope = 0 - p_linear->InvSlope;
}

void Linear_ADC_Init_Inverted(Linear_T * p_linear, uint16_t adcuZero, uint16_t adcuRef, int16_t physicalZero, int16_t physicalRef)
{
	Linear_ADC_Init(p_linear, adcuZero, adcuRef, physicalZero, physicalRef);
	Linear_ADC_InitInverted(p_linear);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
void Linear_ADC_Init_PeakToPeakMilliV(Linear_T * p_linear, uint16_t adcVRef_MilliV, uint16_t adcMax, uint16_t min_MilliV, uint16_t max_MilliV, int16_t physicalZero, int16_t physicalRef)
{
	uint16_t adcuZero = ((uint32_t)max_MilliV + min_MilliV) * adcMax / 2U / adcVRef_MilliV;
	uint16_t adcuRef = (uint32_t)max_MilliV * adcMax / adcVRef_MilliV;
	Linear_ADC_Init(p_linear, adcuZero, adcuRef, physicalZero, physicalRef);
}

void Linear_ADC_Init_ZeroToPeakMilliV(Linear_T * p_linear, uint16_t adcVRef_MilliV, uint16_t adcMax, uint16_t zero_MilliV, uint16_t max_MilliV, int16_t physicalZero, int16_t physicalRef)
{
	uint16_t adcuZero = (uint32_t)zero_MilliV * adcMax / adcVRef_MilliV;
	uint16_t adcuRef = (uint32_t)max_MilliV * adcMax / adcVRef_MilliV;
	Linear_ADC_Init(p_linear, adcuZero, adcuRef, physicalZero, physicalRef);
}
